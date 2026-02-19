// This file is part of MeshOptimizer.Net; see meshoptimizer.h for version/license details
module MeshOptimizer.Net.VFetchOptimizer

open System
open MeshOptimizer.Net.Allocator

let meshopt_optimizeVertexFetchRemap (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) : int =
    assert (index_count % 3 = 0)

    NPtr.memset destination 0xFFuy (vertex_count * sizeof<uint32>)

    let mutable next_vertex = 0u

    for i = 0 to index_count - 1 do
        let index = int (NPtr.get indices i)
        assert (index < vertex_count)

        if NPtr.get destination index = ~~~0u then
            NPtr.set destination index next_vertex
            next_vertex <- next_vertex + 1u

    assert (int next_vertex <= vertex_count)

    int next_vertex

let meshopt_optimizeVertexFetch (destination: nativeint) (indices: nativeptr<uint32>) (index_count: int) (vertices: nativeint) (vertex_count: int) (vertex_size: int) : int =
    assert (index_count % 3 = 0)
    assert (vertex_size > 0 && vertex_size <= 256)

    use allocator = new meshopt_Allocator()

    // support in-place optimization
    let mutable vertices = vertices
    if destination = vertices then
        let vertices_copy = allocator.allocate<byte>(vertex_count * vertex_size)
        NPtr.memcpy vertices_copy (NPtr.ofNI<byte> vertices) (vertex_count * vertex_size)
        vertices <- NPtr.toNI vertices_copy

    // build vertex remap table
    let vertex_remap = allocator.allocate<uint32>(vertex_count)
    NPtr.memset vertex_remap 0xFFuy (vertex_count * sizeof<uint32>)

    let mutable next_vertex = 0u

    for i = 0 to index_count - 1 do
        let index = int (NPtr.get indices i)
        assert (index < vertex_count)

        let remap = NPtr.get vertex_remap index

        if remap = ~~~0u then
            // add vertex
            let dst = NPtr.add (NPtr.ofNI<byte> destination) (int next_vertex * vertex_size)
            let src = NPtr.add (NPtr.ofNI<byte> vertices) (index * vertex_size)
            NPtr.memcpy dst src vertex_size

            NPtr.set vertex_remap index next_vertex
            next_vertex <- next_vertex + 1u

            // modify indices in place
            NPtr.set indices i (next_vertex - 1u)
        else
            // modify indices in place
            NPtr.set indices i remap

    assert (int next_vertex <= vertex_count)

    int next_vertex
