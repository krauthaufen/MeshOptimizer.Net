// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
// This work is based on:
// Pedro Sander, Diego Nehab and Joshua Barczak. Fast Triangle Reordering for Vertex Locality and Reduced Overdraw. 2007
module MeshOptPort.OverdrawOptimizer

open System
open MeshOptPort.Allocator
open MeshOptPort.Quantization

let private calculateSortData (sort_data: nativeptr<float32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (clusters: nativeptr<uint32>) (cluster_count: int) =
    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    let mutable mesh_centroid_0 = 0.0f
    let mutable mesh_centroid_1 = 0.0f
    let mutable mesh_centroid_2 = 0.0f

    for i = 0 to vertex_count - 1 do
        let p = NPtr.add vertex_positions (vertex_stride_float * i)

        mesh_centroid_0 <- mesh_centroid_0 + NPtr.get p 0
        mesh_centroid_1 <- mesh_centroid_1 + NPtr.get p 1
        mesh_centroid_2 <- mesh_centroid_2 + NPtr.get p 2

    let vc = float32 vertex_count
    mesh_centroid_0 <- mesh_centroid_0 / vc
    mesh_centroid_1 <- mesh_centroid_1 / vc
    mesh_centroid_2 <- mesh_centroid_2 / vc

    for cluster = 0 to cluster_count - 1 do
        let cluster_begin = int (NPtr.get clusters cluster) * 3
        let cluster_end =
            if cluster + 1 < cluster_count then int (NPtr.get clusters (cluster + 1)) * 3
            else index_count
        assert (cluster_begin < cluster_end)

        let mutable cluster_area = 0.0f
        let mutable cluster_centroid_0 = 0.0f
        let mutable cluster_centroid_1 = 0.0f
        let mutable cluster_centroid_2 = 0.0f
        let mutable cluster_normal_0 = 0.0f
        let mutable cluster_normal_1 = 0.0f
        let mutable cluster_normal_2 = 0.0f

        let mutable i = cluster_begin
        while i < cluster_end do
            let p0 = NPtr.add vertex_positions (vertex_stride_float * int (NPtr.get indices (i + 0)))
            let p1 = NPtr.add vertex_positions (vertex_stride_float * int (NPtr.get indices (i + 1)))
            let p2 = NPtr.add vertex_positions (vertex_stride_float * int (NPtr.get indices (i + 2)))

            let p10_0 = NPtr.get p1 0 - NPtr.get p0 0
            let p10_1 = NPtr.get p1 1 - NPtr.get p0 1
            let p10_2 = NPtr.get p1 2 - NPtr.get p0 2

            let p20_0 = NPtr.get p2 0 - NPtr.get p0 0
            let p20_1 = NPtr.get p2 1 - NPtr.get p0 1
            let p20_2 = NPtr.get p2 2 - NPtr.get p0 2

            let normalx = p10_1 * p20_2 - p10_2 * p20_1
            let normaly = p10_2 * p20_0 - p10_0 * p20_2
            let normalz = p10_0 * p20_1 - p10_1 * p20_0

            let area = MathF.Sqrt(normalx * normalx + normaly * normaly + normalz * normalz)

            cluster_centroid_0 <- cluster_centroid_0 + (NPtr.get p0 0 + NPtr.get p1 0 + NPtr.get p2 0) * (area / 3.0f)
            cluster_centroid_1 <- cluster_centroid_1 + (NPtr.get p0 1 + NPtr.get p1 1 + NPtr.get p2 1) * (area / 3.0f)
            cluster_centroid_2 <- cluster_centroid_2 + (NPtr.get p0 2 + NPtr.get p1 2 + NPtr.get p2 2) * (area / 3.0f)
            cluster_normal_0 <- cluster_normal_0 + normalx
            cluster_normal_1 <- cluster_normal_1 + normaly
            cluster_normal_2 <- cluster_normal_2 + normalz
            cluster_area <- cluster_area + area

            i <- i + 3

        let inv_cluster_area = if cluster_area = 0.0f then 0.0f else 1.0f / cluster_area

        cluster_centroid_0 <- cluster_centroid_0 * inv_cluster_area
        cluster_centroid_1 <- cluster_centroid_1 * inv_cluster_area
        cluster_centroid_2 <- cluster_centroid_2 * inv_cluster_area

        let cluster_normal_length = MathF.Sqrt(cluster_normal_0 * cluster_normal_0 + cluster_normal_1 * cluster_normal_1 + cluster_normal_2 * cluster_normal_2)
        let inv_cluster_normal_length = if cluster_normal_length = 0.0f then 0.0f else 1.0f / cluster_normal_length

        cluster_normal_0 <- cluster_normal_0 * inv_cluster_normal_length
        cluster_normal_1 <- cluster_normal_1 * inv_cluster_normal_length
        cluster_normal_2 <- cluster_normal_2 * inv_cluster_normal_length

        let centroid_vector_0 = cluster_centroid_0 - mesh_centroid_0
        let centroid_vector_1 = cluster_centroid_1 - mesh_centroid_1
        let centroid_vector_2 = cluster_centroid_2 - mesh_centroid_2

        NPtr.set sort_data cluster (centroid_vector_0 * cluster_normal_0 + centroid_vector_1 * cluster_normal_1 + centroid_vector_2 * cluster_normal_2)

let private calculateSortOrderRadix (sort_order: nativeptr<uint32>) (sort_data: nativeptr<float32>) (sort_keys: nativeptr<uint16>) (cluster_count: int) =
    // compute sort data bounds and renormalize, using fixed point snorm
    let mutable sort_data_max = 1e-3f

    for i = 0 to cluster_count - 1 do
        let dpa = MathF.Abs(NPtr.get sort_data i)
        sort_data_max <- if sort_data_max < dpa then dpa else sort_data_max

    let sort_bits = 11

    for i = 0 to cluster_count - 1 do
        // note that we flip distribution since high dot product should come first
        let sort_key = 0.5f - 0.5f * (NPtr.get sort_data i / sort_data_max)
        NPtr.set sort_keys i (uint16 (meshopt_quantizeUnorm sort_key sort_bits &&& ((1 <<< sort_bits) - 1)))

    // fill histogram for counting sort
    let histogram = Array.zeroCreate<uint32> (1 <<< sort_bits)

    for i = 0 to cluster_count - 1 do
        let key = int (NPtr.get sort_keys i)
        histogram.[key] <- histogram.[key] + 1u

    // compute offsets based on histogram data
    let mutable histogram_sum = 0u

    for i = 0 to (1 <<< sort_bits) - 1 do
        let count = histogram.[i]
        histogram.[i] <- histogram_sum
        histogram_sum <- histogram_sum + count

    assert (int histogram_sum = cluster_count)

    // compute sort order based on offsets
    for i = 0 to cluster_count - 1 do
        let key = int (NPtr.get sort_keys i)
        NPtr.set sort_order (int histogram.[key]) (uint32 i)
        histogram.[key] <- histogram.[key] + 1u

let private updateCache (a: uint32) (b: uint32) (c: uint32) (cache_size: uint32) (cache_timestamps: nativeptr<uint32>) (timestamp: byref<uint32>) : uint32 =
    let mutable cache_misses = 0u

    // if vertex is not in cache, put it in cache
    if timestamp - NPtr.get cache_timestamps (int a) > cache_size then
        NPtr.set cache_timestamps (int a) timestamp
        timestamp <- timestamp + 1u
        cache_misses <- cache_misses + 1u

    if timestamp - NPtr.get cache_timestamps (int b) > cache_size then
        NPtr.set cache_timestamps (int b) timestamp
        timestamp <- timestamp + 1u
        cache_misses <- cache_misses + 1u

    if timestamp - NPtr.get cache_timestamps (int c) > cache_size then
        NPtr.set cache_timestamps (int c) timestamp
        timestamp <- timestamp + 1u
        cache_misses <- cache_misses + 1u

    cache_misses

let private generateHardBoundaries (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (cache_size: uint32) (cache_timestamps: nativeptr<uint32>) : int =
    NPtr.memset cache_timestamps 0uy (vertex_count * sizeof<uint32>)

    let mutable timestamp = cache_size + 1u

    let face_count = index_count / 3

    let mutable result = 0

    for i = 0 to face_count - 1 do
        let m = updateCache (NPtr.get indices (i * 3 + 0)) (NPtr.get indices (i * 3 + 1)) (NPtr.get indices (i * 3 + 2)) cache_size cache_timestamps &timestamp

        // when all three vertices are not in the cache it's usually relatively safe to assume that this is a new patch in the mesh
        // that is disjoint from previous vertices; sometimes it might come back to reference existing vertices but that frequently
        // suggests an inefficiency in the vertex cache optimization algorithm
        // usually the first triangle has 3 misses unless it's degenerate - thus we make sure the first cluster always starts with 0
        if i = 0 || m = 3u then
            NPtr.set destination result (uint32 i)
            result <- result + 1

    assert (result <= index_count / 3)

    result

let private generateSoftBoundaries (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (clusters: nativeptr<uint32>) (cluster_count: int) (cache_size: uint32) (threshold: float32) (cache_timestamps: nativeptr<uint32>) : int =
    NPtr.memset cache_timestamps 0uy (vertex_count * sizeof<uint32>)

    let mutable timestamp = 0u

    let mutable result = 0

    for it = 0 to cluster_count - 1 do
        let start = int (NPtr.get clusters it)
        let end_ = if it + 1 < cluster_count then int (NPtr.get clusters (it + 1)) else index_count / 3
        assert (start < end_)

        // reset cache
        timestamp <- timestamp + cache_size + 1u

        // measure cluster ACMR
        let mutable cluster_misses = 0u

        for i = start to end_ - 1 do
            let m = updateCache (NPtr.get indices (i * 3 + 0)) (NPtr.get indices (i * 3 + 1)) (NPtr.get indices (i * 3 + 2)) cache_size cache_timestamps &timestamp
            cluster_misses <- cluster_misses + m

        let cluster_threshold = threshold * (float32 cluster_misses / float32 (end_ - start))

        // first cluster always starts from the hard cluster boundary
        NPtr.set destination result (uint32 start)
        result <- result + 1

        // reset cache
        timestamp <- timestamp + cache_size + 1u

        let mutable running_misses = 0u
        let mutable running_faces = 0u

        for i = start to end_ - 1 do
            let m = updateCache (NPtr.get indices (i * 3 + 0)) (NPtr.get indices (i * 3 + 1)) (NPtr.get indices (i * 3 + 2)) cache_size cache_timestamps &timestamp
            running_misses <- running_misses + m
            running_faces <- running_faces + 1u

            if float32 running_misses / float32 running_faces <= cluster_threshold then
                // we have reached the target ACMR with the current triangle so we need to start a new cluster on the next one
                // note that this may mean that we add 'end_' to destination for the last triangle, which will imply that the last
                // cluster is empty; however, the 'pop_back' after the loop will clean it up
                NPtr.set destination result (uint32 (i + 1))
                result <- result + 1

                // reset cache
                timestamp <- timestamp + cache_size + 1u

                running_misses <- 0u
                running_faces <- 0u

        // each time we reach the target ACMR we flush the cluster
        // this means that the last cluster is by definition not very good - there are frequent cases where we are left with a few triangles
        // in the last cluster, producing a very bad ACMR and significantly penalizing the overall results
        // thus we remove the last cluster boundary, merging the last complete cluster with the last incomplete one
        // there are sometimes cases when the last cluster is actually good enough - in which case the code above would have added 'end_'
        // to the cluster boundary array which we need to remove anyway - this code will do that automatically
        if NPtr.get destination (result - 1) <> uint32 start then
            result <- result - 1

    assert (result >= cluster_count)
    assert (result <= index_count / 3)

    result

let meshopt_optimizeOverdraw (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (threshold: float32) =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    use allocator = new meshopt_Allocator()

    // guard for empty meshes
    if index_count = 0 || vertex_count = 0 then
        ()
    else

    // support in-place optimization
    let mutable indices = indices
    if NPtr.toNI destination = NPtr.toNI indices then
        let indices_copy = allocator.allocate<uint32>(index_count)
        NPtr.memcpy indices_copy indices (index_count * sizeof<uint32>)
        indices <- indices_copy

    let cache_size = 16u

    let cache_timestamps = allocator.allocate<uint32>(vertex_count)

    // generate hard boundaries from full-triangle cache misses
    let hard_clusters = allocator.allocate<uint32>(index_count / 3)
    let hard_cluster_count = generateHardBoundaries hard_clusters indices index_count vertex_count cache_size cache_timestamps

    // generate soft boundaries
    let soft_clusters = allocator.allocate<uint32>(index_count / 3 + 1)
    let soft_cluster_count = generateSoftBoundaries soft_clusters indices index_count vertex_count hard_clusters hard_cluster_count cache_size threshold cache_timestamps

    let clusters = soft_clusters
    let cluster_count = soft_cluster_count

    // fill sort data
    let sort_data = allocator.allocate<float32>(cluster_count)
    calculateSortData sort_data indices index_count vertex_positions vertex_count vertex_positions_stride clusters cluster_count

    // sort clusters using sort data
    let sort_keys = allocator.allocate<uint16>(cluster_count)
    let sort_order = allocator.allocate<uint32>(cluster_count)
    calculateSortOrderRadix sort_order sort_data sort_keys cluster_count

    // fill output buffer
    let mutable offset = 0

    for it = 0 to cluster_count - 1 do
        let cluster = int (NPtr.get sort_order it)
        assert (cluster < cluster_count)

        let cluster_begin = int (NPtr.get clusters cluster) * 3
        let cluster_end =
            if cluster + 1 < cluster_count then int (NPtr.get clusters (cluster + 1)) * 3
            else index_count
        assert (cluster_begin < cluster_end)

        NPtr.memcpy (NPtr.add destination offset) (NPtr.add indices cluster_begin) ((cluster_end - cluster_begin) * sizeof<uint32>)
        offset <- offset + (cluster_end - cluster_begin)

    assert (offset = index_count)
