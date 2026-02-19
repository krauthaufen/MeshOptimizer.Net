// This file is part of MeshOptimizer.Net; see meshoptimizer.h for version/license details
module MeshOptimizer.Net.IndexAnalyzer

open MeshOptimizer.Net
open MeshOptimizer.Net.Allocator

let meshopt_analyzeVertexCache (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (cache_size: uint32) (warp_size: uint32) (primgroup_size: uint32) : meshopt_VertexCacheStatistics =
    assert (index_count % 3 = 0)
    assert (cache_size >= 3u)
    assert (warp_size = 0u || warp_size >= 3u)

    use allocator = new meshopt_Allocator()

    let mutable result = meshopt_VertexCacheStatistics()

    let mutable warp_offset = 0u
    let mutable primgroup_offset = 0u

    let cache_timestamps = allocator.allocate<uint32>(vertex_count)
    NPtr.memset cache_timestamps 0uy (vertex_count * sizeof<uint32>)

    let mutable timestamp = cache_size + 1u

    let mutable i = 0
    while i < index_count do
        let a = NPtr.get indices (i + 0)
        let b = NPtr.get indices (i + 1)
        let c = NPtr.get indices (i + 2)
        assert (a < uint32 vertex_count && b < uint32 vertex_count && c < uint32 vertex_count)

        let ac = (timestamp - NPtr.get cache_timestamps (int a)) > cache_size
        let bc = (timestamp - NPtr.get cache_timestamps (int b)) > cache_size
        let cc = (timestamp - NPtr.get cache_timestamps (int c)) > cache_size

        // flush cache if triangle doesn't fit into warp or into the primitive buffer
        if (primgroup_size <> 0u && primgroup_offset = primgroup_size) ||
           (warp_size <> 0u && warp_offset + (if ac then 1u else 0u) + (if bc then 1u else 0u) + (if cc then 1u else 0u) > warp_size) then
            result.warps_executed <- result.warps_executed + (if warp_offset > 0u then 1u else 0u)
            warp_offset <- 0u
            primgroup_offset <- 0u
            // reset cache
            timestamp <- timestamp + cache_size + 1u

        // update cache and add vertices to warp
        for j = 0 to 2 do
            let index = NPtr.get indices (i + j)
            if (timestamp - NPtr.get cache_timestamps (int index)) > cache_size then
                NPtr.set cache_timestamps (int index) timestamp
                timestamp <- timestamp + 1u
                result.vertices_transformed <- result.vertices_transformed + 1u
                warp_offset <- warp_offset + 1u

        primgroup_offset <- primgroup_offset + 1u
        i <- i + 3

    let mutable unique_vertex_count = 0

    for i = 0 to vertex_count - 1 do
        if NPtr.get cache_timestamps i > 0u then
            unique_vertex_count <- unique_vertex_count + 1

    result.warps_executed <- result.warps_executed + (if warp_offset > 0u then 1u else 0u)

    result.acmr <- if index_count = 0 then 0.0f else float32 result.vertices_transformed / float32 (index_count / 3)
    result.atvr <- if unique_vertex_count = 0 then 0.0f else float32 result.vertices_transformed / float32 unique_vertex_count

    result

let meshopt_analyzeVertexFetch (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (vertex_size: int) : meshopt_VertexFetchStatistics =
    assert (index_count % 3 = 0)
    assert (vertex_size > 0 && vertex_size <= 256)

    use allocator = new meshopt_Allocator()

    let mutable result = meshopt_VertexFetchStatistics()

    let vertex_visited = allocator.allocate<byte>(vertex_count)
    NPtr.memset vertex_visited 0uy vertex_count

    let kCacheLine = 64
    let kCacheSize = 128 * 1024

    // simple direct mapped cache
    let cache = Array.zeroCreate<int>(kCacheSize / kCacheLine)

    for i = 0 to index_count - 1 do
        let index = int (NPtr.get indices i)
        assert (index < vertex_count)

        NPtr.set vertex_visited index 1uy

        let start_address = index * vertex_size
        let end_address = start_address + vertex_size

        let start_tag = start_address / kCacheLine
        let end_tag = (end_address + kCacheLine - 1) / kCacheLine

        assert (start_tag < end_tag)

        for tag = start_tag to end_tag - 1 do
            let line = tag % cache.Length

            // we store +1 since cache is filled with 0 by default
            if cache.[line] <> tag + 1 then
                result.bytes_fetched <- result.bytes_fetched + uint32 kCacheLine
            cache.[line] <- tag + 1

    let mutable unique_vertex_count = 0

    for i = 0 to vertex_count - 1 do
        if NPtr.get vertex_visited i > 0uy then
            unique_vertex_count <- unique_vertex_count + 1

    result.overfetch <- if unique_vertex_count = 0 then 0.0f else float32 result.bytes_fetched / float32 (unique_vertex_count * vertex_size)

    result
