// This file is part of MeshOptimizerDotNet; see meshoptimizer.h for version/license details
// This work is based on:
// Tom Forsyth. Linear-Speed Vertex Cache Optimisation. 2006
// Pedro Sander, Diego Nehab and Joshua Barczak. Fast Triangle Reordering for Vertex Locality and Reduced Overdraw. 2007
module MeshOptimizerDotNet.VCacheOptimizer

open MeshOptimizerDotNet.Allocator

[<Literal>]
let private kCacheSizeMax = 16

[<Literal>]
let private kValenceMax = 8

// Vertex score table: cache[1 + kCacheSizeMax] and live[1 + kValenceMax]
type private VertexScoreTable = {
    cache: float32[]
    live: float32[]
}

// Tuned to minimize the ACMR of a GPU that has a cache profile similar to NVidia and AMD
let private kVertexScoreTable = {
    cache = [| 0.0f; 0.779f; 0.791f; 0.789f; 0.981f; 0.843f; 0.726f; 0.847f; 0.882f; 0.867f; 0.799f; 0.642f; 0.613f; 0.600f; 0.568f; 0.372f; 0.234f |]
    live = [| 0.0f; 0.995f; 0.713f; 0.450f; 0.404f; 0.059f; 0.005f; 0.147f; 0.006f |]
}

// Tuned to minimize the encoded index buffer size
let private kVertexScoreTableStrip = {
    cache = [| 0.0f; 1.000f; 1.000f; 1.000f; 0.453f; 0.561f; 0.490f; 0.459f; 0.179f; 0.526f; 0.000f; 0.227f; 0.184f; 0.490f; 0.112f; 0.050f; 0.131f |]
    live = [| 0.0f; 0.956f; 0.786f; 0.577f; 0.558f; 0.618f; 0.549f; 0.499f; 0.489f |]
}

type private TriangleAdjacency = {
    mutable counts: nativeptr<uint32>
    mutable offsets: nativeptr<uint32>
    mutable data: nativeptr<uint32>
}

let private buildTriangleAdjacency (adjacency: TriangleAdjacency) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (allocator: meshopt_Allocator) =
    let face_count = index_count / 3

    // allocate arrays
    adjacency.counts <- allocator.allocate<uint32>(vertex_count)
    adjacency.offsets <- allocator.allocate<uint32>(vertex_count)
    adjacency.data <- allocator.allocate<uint32>(index_count)

    // fill triangle counts
    NPtr.memset adjacency.counts 0uy (vertex_count * sizeof<uint32>)

    for i = 0 to index_count - 1 do
        let idx = int (NPtr.get indices i)
        assert (idx < vertex_count)
        NPtr.set adjacency.counts idx (NPtr.get adjacency.counts idx + 1u)

    // fill offset table
    let mutable offset = 0u

    for i = 0 to vertex_count - 1 do
        NPtr.set adjacency.offsets i offset
        offset <- offset + NPtr.get adjacency.counts i

    assert (offset = uint32 index_count)

    // fill triangle data
    for i = 0 to face_count - 1 do
        let a = int (NPtr.get indices (i * 3 + 0))
        let b = int (NPtr.get indices (i * 3 + 1))
        let c = int (NPtr.get indices (i * 3 + 2))

        let oa = int (NPtr.get adjacency.offsets a)
        NPtr.set adjacency.data oa (uint32 i)
        NPtr.set adjacency.offsets a (uint32 oa + 1u)

        let ob = int (NPtr.get adjacency.offsets b)
        NPtr.set adjacency.data ob (uint32 i)
        NPtr.set adjacency.offsets b (uint32 ob + 1u)

        let oc = int (NPtr.get adjacency.offsets c)
        NPtr.set adjacency.data oc (uint32 i)
        NPtr.set adjacency.offsets c (uint32 oc + 1u)

    // fix offsets that have been disturbed by the previous pass
    for i = 0 to vertex_count - 1 do
        assert (NPtr.get adjacency.offsets i >= NPtr.get adjacency.counts i)
        NPtr.set adjacency.offsets i (NPtr.get adjacency.offsets i - NPtr.get adjacency.counts i)

let private getNextVertexDeadEnd (dead_end: nativeptr<uint32>) (dead_end_top: byref<uint32>) (input_cursor: byref<uint32>) (live_triangles: nativeptr<uint32>) (vertex_count: int) : uint32 =
    // check dead-end stack
    let mutable found = false
    let mutable result = ~~~0u

    while dead_end_top > 0u && not found do
        dead_end_top <- dead_end_top - 1u
        let vertex = NPtr.get dead_end (int dead_end_top)

        if NPtr.get live_triangles (int vertex) > 0u then
            result <- vertex
            found <- true

    if not found then
        // input order
        while input_cursor < uint32 vertex_count && not found do
            if NPtr.get live_triangles (int input_cursor) > 0u then
                result <- input_cursor
                found <- true
            else
                input_cursor <- input_cursor + 1u

    result

let private getNextVertexNeighbor (next_candidates: nativeptr<uint32>) (candidates_begin: int) (candidates_end: int) (live_triangles: nativeptr<uint32>) (cache_timestamps: nativeptr<uint32>) (timestamp: uint32) (cache_size: uint32) : uint32 =
    let mutable best_candidate = ~~~0u
    let mutable best_priority = -1

    for idx = candidates_begin to candidates_end - 1 do
        let vertex = NPtr.get next_candidates idx

        // otherwise we don't need to process it
        if NPtr.get live_triangles (int vertex) > 0u then
            let mutable priority = 0

            // will it be in cache after fanning?
            if 2u * NPtr.get live_triangles (int vertex) + timestamp - NPtr.get cache_timestamps (int vertex) <= cache_size then
                priority <- int (timestamp - NPtr.get cache_timestamps (int vertex)) // position in cache

            if priority > best_priority then
                best_candidate <- vertex
                best_priority <- priority

    best_candidate

let private vertexScore (table: VertexScoreTable) (cache_position: int) (live_triangles: uint32) : float32 =
    assert (cache_position >= -1 && cache_position < kCacheSizeMax)

    let live_triangles_clamped = if live_triangles < uint32 kValenceMax then int live_triangles else kValenceMax

    table.cache.[1 + cache_position] + table.live.[live_triangles_clamped]

let private getNextTriangleDeadEnd (input_cursor: byref<uint32>) (emitted_flags: nativeptr<byte>) (face_count: int) : uint32 =
    let mutable found = false
    let mutable result = ~~~0u

    // input order
    while input_cursor < uint32 face_count && not found do
        if NPtr.get emitted_flags (int input_cursor) = 0uy then
            result <- input_cursor
            found <- true
        else
            input_cursor <- input_cursor + 1u

    result

let private meshopt_optimizeVertexCacheTable (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (table: VertexScoreTable) =
    assert (index_count % 3 = 0)

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

    let cache_size = 16
    assert (cache_size <= kCacheSizeMax)

    let face_count = index_count / 3

    // build adjacency information
    let adjacency = { counts = NPtr.ofNI 0n; offsets = NPtr.ofNI 0n; data = NPtr.ofNI 0n }
    buildTriangleAdjacency adjacency indices index_count vertex_count allocator

    // live triangle counts; note, we alias adjacency.counts as we remove triangles after emitting them so the counts always match
    let live_triangles = adjacency.counts

    // emitted flags
    let emitted_flags = allocator.allocate<byte>(face_count)
    NPtr.memset emitted_flags 0uy face_count

    // compute initial vertex scores
    let vertex_scores = allocator.allocate<float32>(vertex_count)

    for i = 0 to vertex_count - 1 do
        NPtr.set vertex_scores i (vertexScore table -1 (NPtr.get live_triangles i))

    // compute triangle scores
    let triangle_scores = allocator.allocate<float32>(face_count)

    for i = 0 to face_count - 1 do
        let a = int (NPtr.get indices (i * 3 + 0))
        let b = int (NPtr.get indices (i * 3 + 1))
        let c = int (NPtr.get indices (i * 3 + 2))

        NPtr.set triangle_scores i (NPtr.get vertex_scores a + NPtr.get vertex_scores b + NPtr.get vertex_scores c)

    let cache_holder = Array.zeroCreate<uint32>(2 * (kCacheSizeMax + 4))
    let mutable cache_offset = 0
    let mutable cache_new_offset = kCacheSizeMax + 4
    let mutable cache_count = 0

    let mutable current_triangle = 0u
    let mutable input_cursor = 1u

    let mutable output_triangle = 0

    while current_triangle <> ~~~0u do
        assert (output_triangle < face_count)

        let a = NPtr.get indices (int current_triangle * 3 + 0)
        let b = NPtr.get indices (int current_triangle * 3 + 1)
        let c = NPtr.get indices (int current_triangle * 3 + 2)

        // output indices
        NPtr.set destination (output_triangle * 3 + 0) a
        NPtr.set destination (output_triangle * 3 + 1) b
        NPtr.set destination (output_triangle * 3 + 2) c
        output_triangle <- output_triangle + 1

        // update emitted flags
        NPtr.set emitted_flags (int current_triangle) 1uy
        NPtr.set triangle_scores (int current_triangle) 0.0f

        // new triangle
        let mutable cache_write = 0
        cache_holder.[cache_new_offset + cache_write] <- a
        cache_write <- cache_write + 1
        cache_holder.[cache_new_offset + cache_write] <- b
        cache_write <- cache_write + 1
        cache_holder.[cache_new_offset + cache_write] <- c
        cache_write <- cache_write + 1

        // old triangles
        for i = 0 to cache_count - 1 do
            let index = cache_holder.[cache_offset + i]

            cache_holder.[cache_new_offset + cache_write] <- index
            let keep = (if index <> a then 1 else 0) &&& (if index <> b then 1 else 0) &&& (if index <> c then 1 else 0)
            cache_write <- cache_write + keep

        // swap cache and cache_new
        let temp = cache_offset
        cache_offset <- cache_new_offset
        cache_new_offset <- temp
        cache_count <- if cache_write > cache_size then cache_size else cache_write

        // remove emitted triangle from adjacency data
        // this makes sure that we spend less time traversing these lists on subsequent iterations
        // live triangle counts are updated as a byproduct of these adjustments
        for k = 0 to 2 do
            let index = int (NPtr.get indices (int current_triangle * 3 + k))

            let neighbors_offset = int (NPtr.get adjacency.offsets index)
            let mutable neighbors_size = int (NPtr.get adjacency.counts index)

            let mutable j = 0
            let mutable found = false
            while j < neighbors_size && not found do
                let tri = NPtr.get adjacency.data (neighbors_offset + j)

                if tri = current_triangle then
                    NPtr.set adjacency.data (neighbors_offset + j) (NPtr.get adjacency.data (neighbors_offset + neighbors_size - 1))
                    NPtr.set adjacency.counts index (uint32 (neighbors_size - 1))
                    found <- true
                else
                    j <- j + 1

        let mutable best_triangle = ~~~0u
        let mutable best_score = 0.0f

        // update cache positions, vertex scores and triangle scores, and find next best triangle
        for i = 0 to cache_write - 1 do
            let index = int cache_holder.[cache_offset + i]

            // no need to update scores if we are never going to use this vertex
            if NPtr.get adjacency.counts index > 0u then
                let cache_position = if i >= cache_size then -1 else i

                // update vertex score
                let score = vertexScore table cache_position (NPtr.get live_triangles index)
                let score_diff = score - NPtr.get vertex_scores index

                NPtr.set vertex_scores index score

                // update scores of vertex triangles
                let nb_offset = int (NPtr.get adjacency.offsets index)
                let nb_count = int (NPtr.get adjacency.counts index)

                for j = 0 to nb_count - 1 do
                    let tri = int (NPtr.get adjacency.data (nb_offset + j))
                    assert (NPtr.get emitted_flags tri = 0uy)

                    let tri_score = NPtr.get triangle_scores tri + score_diff
                    assert (tri_score > 0.0f)

                    if best_score < tri_score then
                        best_triangle <- uint32 tri
                        best_score <- tri_score

                    NPtr.set triangle_scores tri tri_score

        // step through input triangles in order if we hit a dead-end
        current_triangle <- best_triangle

        if current_triangle = ~~~0u then
            current_triangle <- getNextTriangleDeadEnd &input_cursor emitted_flags face_count

    assert (input_cursor = uint32 face_count)
    assert (output_triangle = face_count)

let meshopt_optimizeVertexCache (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) =
    meshopt_optimizeVertexCacheTable destination indices index_count vertex_count kVertexScoreTable

let meshopt_optimizeVertexCacheStrip (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) =
    meshopt_optimizeVertexCacheTable destination indices index_count vertex_count kVertexScoreTableStrip

let meshopt_optimizeVertexCacheFifo (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (cache_size: uint32) =
    assert (index_count % 3 = 0)
    assert (cache_size >= 3u)

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

    let face_count = index_count / 3

    // build adjacency information
    let adjacency = { counts = NPtr.ofNI 0n; offsets = NPtr.ofNI 0n; data = NPtr.ofNI 0n }
    buildTriangleAdjacency adjacency indices index_count vertex_count allocator

    // live triangle counts
    let live_triangles = allocator.allocate<uint32>(vertex_count)
    NPtr.memcpy live_triangles adjacency.counts (vertex_count * sizeof<uint32>)

    // cache time stamps
    let cache_timestamps = allocator.allocate<uint32>(vertex_count)
    NPtr.memset cache_timestamps 0uy (vertex_count * sizeof<uint32>)

    // dead-end stack
    let dead_end = allocator.allocate<uint32>(index_count)
    let mutable dead_end_top = 0u

    // emitted flags
    let emitted_flags = allocator.allocate<byte>(face_count)
    NPtr.memset emitted_flags 0uy face_count

    let mutable current_vertex = 0u

    let mutable timestamp = cache_size + 1u
    let mutable input_cursor = 1u // vertex to restart from in case of dead-end

    let mutable output_triangle = 0

    while current_vertex <> ~~~0u do
        let next_candidates_begin = int dead_end_top

        // emit all vertex neighbors
        let neighbors_offset = int (NPtr.get adjacency.offsets (int current_vertex))
        let neighbors_count = int (NPtr.get adjacency.counts (int current_vertex))

        for idx = 0 to neighbors_count - 1 do
            let triangle = NPtr.get adjacency.data (neighbors_offset + idx)

            if NPtr.get emitted_flags (int triangle) = 0uy then
                let a = NPtr.get indices (int triangle * 3 + 0)
                let b = NPtr.get indices (int triangle * 3 + 1)
                let c = NPtr.get indices (int triangle * 3 + 2)

                // output indices
                NPtr.set destination (output_triangle * 3 + 0) a
                NPtr.set destination (output_triangle * 3 + 1) b
                NPtr.set destination (output_triangle * 3 + 2) c
                output_triangle <- output_triangle + 1

                // update dead-end stack
                NPtr.set dead_end (int dead_end_top + 0) a
                NPtr.set dead_end (int dead_end_top + 1) b
                NPtr.set dead_end (int dead_end_top + 2) c
                dead_end_top <- dead_end_top + 3u

                // update live triangle counts
                NPtr.set live_triangles (int a) (NPtr.get live_triangles (int a) - 1u)
                NPtr.set live_triangles (int b) (NPtr.get live_triangles (int b) - 1u)
                NPtr.set live_triangles (int c) (NPtr.get live_triangles (int c) - 1u)

                // update cache info
                // if vertex is not in cache, put it in cache
                if timestamp - NPtr.get cache_timestamps (int a) > cache_size then
                    NPtr.set cache_timestamps (int a) timestamp
                    timestamp <- timestamp + 1u

                if timestamp - NPtr.get cache_timestamps (int b) > cache_size then
                    NPtr.set cache_timestamps (int b) timestamp
                    timestamp <- timestamp + 1u

                if timestamp - NPtr.get cache_timestamps (int c) > cache_size then
                    NPtr.set cache_timestamps (int c) timestamp
                    timestamp <- timestamp + 1u

                // update emitted flags
                NPtr.set emitted_flags (int triangle) 1uy

        // next candidates are the ones we pushed to dead-end stack just now
        let next_candidates_end = int dead_end_top

        // get next vertex
        current_vertex <- getNextVertexNeighbor dead_end next_candidates_begin next_candidates_end live_triangles cache_timestamps timestamp cache_size

        if current_vertex = ~~~0u then
            current_vertex <- getNextVertexDeadEnd dead_end &dead_end_top &input_cursor live_triangles vertex_count

    assert (output_triangle = face_count)
