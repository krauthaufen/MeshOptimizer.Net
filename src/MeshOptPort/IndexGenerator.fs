// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
module MeshOptPort.IndexGenerator

// This work is based on:
// Matthias Teschner, Bruno Heidelberger, Matthias Mueller, Danat Pomeranets, Markus Gross. Optimized Spatial Hashing for Collision Detection of Deformable Objects. 2003
// John McDonald, Mark Kilgard. Crack-Free Point-Normal Triangles using Adjacent Edge Normals. 2010
// John Hable. Variable Rate Shading with Visibility Buffer Rendering. 2024

open System
open MeshOptPort.Allocator

/// Delegate for custom vertex equality callback used by meshopt_generateVertexRemapCustom.
/// Parameters: context (nativeint), lhs index (uint32), rhs index (uint32). Returns non-zero if equal.
type VertexEqualCallback = delegate of nativeint * uint32 * uint32 -> int

// ---------------------------------------------------------------------------
// Internal hash helpers
// ---------------------------------------------------------------------------

/// MurmurHash2 operating on 4-byte chunks
let private hashUpdate4 (h: uint32) (key: nativeptr<byte>) (len: int) : uint32 =
    let m = 0x5bd1e995u
    let r = 24
    let mutable h = h
    let mutable offset = 0
    let mutable remaining = len
    while remaining >= 4 do
        let mutable k = NPtr.get (NPtr.cast<byte, uint32> (NPtr.add key offset)) 0
        k <- k * m
        k <- k ^^^ (k >>> r)
        k <- k * m
        h <- h * m
        h <- h ^^^ k
        offset <- offset + 4
        remaining <- remaining - 4
    h

/// Compare byte ranges for equality (memcmp == 0 equivalent)
let private memcmp (a: nativeptr<byte>) (b: nativeptr<byte>) (len: int) : bool =
    let mutable equal = true
    let mutable i = 0
    while i < len && equal do
        if NPtr.get a i <> NPtr.get b i then equal <- false
        i <- i + 1
    equal

// ---------------------------------------------------------------------------
// Hash bucket sizing
// ---------------------------------------------------------------------------

let private hashBuckets (count: int) : int =
    let mutable buckets = 1
    while buckets < count + count / 4 do
        buckets <- buckets * 2
    buckets

// ---------------------------------------------------------------------------
// Generic hash-table lookup (returns index into table)
// ---------------------------------------------------------------------------

let private hashLookupIndex (table: nativeptr<uint32>) (buckets: int) (hashFn: uint32 -> int) (equalFn: uint32 -> uint32 -> bool) (key: uint32) (empty: uint32) : int =
    assert (buckets > 0)
    let hashmod = buckets - 1
    let mutable bucket = (hashFn key) &&& hashmod
    let mutable result = -1
    let mutable probe = 0
    while probe <= hashmod && result = -1 do
        let item = NPtr.get table bucket
        if item = empty then
            result <- bucket
        elif equalFn item key then
            result <- bucket
        else
            bucket <- (bucket + probe + 1) &&& hashmod
            probe <- probe + 1
    assert (result >= 0)
    result

/// Hash-table lookup for uint64 keys (used by EdgeHasher)
let private hashLookupIndex64 (table: nativeptr<uint64>) (buckets: int) (hashFn: uint64 -> int) (equalFn: uint64 -> uint64 -> bool) (key: uint64) (empty: uint64) : int =
    assert (buckets > 0)
    let hashmod = buckets - 1
    let mutable bucket = (hashFn key) &&& hashmod
    let mutable result = -1
    let mutable probe = 0
    while probe <= hashmod && result = -1 do
        let item = NPtr.get table bucket
        if item = empty then
            result <- bucket
        elif equalFn item key then
            result <- bucket
        else
            bucket <- (bucket + probe + 1) &&& hashmod
            probe <- probe + 1
    assert (result >= 0)
    result

// ---------------------------------------------------------------------------
// Vertex hashing: single buffer
// ---------------------------------------------------------------------------

let private vertexHash (vertices: nativeptr<byte>) (vertex_size: int) (vertex_stride: int) (index: uint32) : int =
    int (hashUpdate4 0u (NPtr.add vertices (int index * vertex_stride)) vertex_size)

let private vertexEqual (vertices: nativeptr<byte>) (vertex_size: int) (vertex_stride: int) (lhs: uint32) (rhs: uint32) : bool =
    memcmp (NPtr.add vertices (int lhs * vertex_stride)) (NPtr.add vertices (int rhs * vertex_stride)) vertex_size

// ---------------------------------------------------------------------------
// Vertex hashing: multiple streams
// ---------------------------------------------------------------------------

let private vertexStreamHash (streams: nativeptr<meshopt_Stream>) (stream_count: int) (index: uint32) : int =
    let mutable h = 0u
    for i = 0 to stream_count - 1 do
        let s = NPtr.get streams i
        let data : nativeptr<byte> = NPtr.ofNI s.data
        h <- hashUpdate4 h (NPtr.add data (int index * int s.stride)) (int s.size)
    int h

let private vertexStreamEqual (streams: nativeptr<meshopt_Stream>) (stream_count: int) (lhs: uint32) (rhs: uint32) : bool =
    let mutable equal = true
    let mutable i = 0
    while i < stream_count && equal do
        let s = NPtr.get streams i
        let data : nativeptr<byte> = NPtr.ofNI s.data
        if not (memcmp (NPtr.add data (int lhs * int s.stride)) (NPtr.add data (int rhs * int s.stride)) (int s.size)) then
            equal <- false
        i <- i + 1
    equal

// ---------------------------------------------------------------------------
// Custom vertex hashing (position-based with optional callback)
// ---------------------------------------------------------------------------

let private vertexCustomHash (vertex_positions: nativeptr<float32>) (vertex_stride_float: int) (index: uint32) : int =
    let key : nativeptr<uint32> = NPtr.cast (NPtr.add vertex_positions (int index * vertex_stride_float))
    let mutable x = NPtr.get key 0
    let mutable y = NPtr.get key 1
    let mutable z = NPtr.get key 2

    // replace negative zero with zero
    x <- if x = 0x80000000u then 0u else x
    y <- if y = 0x80000000u then 0u else y
    z <- if z = 0x80000000u then 0u else z

    // scramble bits to make sure that integer coordinates have entropy in lower bits
    x <- x ^^^ (x >>> 17)
    y <- y ^^^ (y >>> 17)
    z <- z ^^^ (z >>> 17)

    // Optimized Spatial Hashing for Collision Detection of Deformable Objects
    int ((x * 73856093u) ^^^ (y * 19349663u) ^^^ (z * 83492791u))

let private vertexCustomEqual (vertex_positions: nativeptr<float32>) (vertex_stride_float: int) (callback: VertexEqualCallback) (context: nativeint) (lhs: uint32) (rhs: uint32) : bool =
    let lp = NPtr.add vertex_positions (int lhs * vertex_stride_float)
    let rp = NPtr.add vertex_positions (int rhs * vertex_stride_float)

    if NPtr.get lp 0 <> NPtr.get rp 0 || NPtr.get lp 1 <> NPtr.get rp 1 || NPtr.get lp 2 <> NPtr.get rp 2 then
        false
    elif Object.ReferenceEquals(callback, null) then
        true
    else
        callback.Invoke(context, lhs, rhs) <> 0

// ---------------------------------------------------------------------------
// Edge hashing
// ---------------------------------------------------------------------------

let private edgeHash (remap: nativeptr<uint32>) (edge: uint64) : int =
    let e0 = uint32 (edge >>> 32)
    let e1 = uint32 edge
    let mutable h1 = NPtr.get remap (int e0)
    let mutable h2 = NPtr.get remap (int e1)
    let m = 0x5bd1e995u
    // MurmurHash64B finalizer
    h1 <- h1 ^^^ (h2 >>> 18)
    h1 <- h1 * m
    h2 <- h2 ^^^ (h1 >>> 22)
    h2 <- h2 * m
    h1 <- h1 ^^^ (h2 >>> 17)
    h1 <- h1 * m
    h2 <- h2 ^^^ (h1 >>> 19)
    h2 <- h2 * m
    int h2

let private edgeEqual (remap: nativeptr<uint32>) (lhs: uint64) (rhs: uint64) : bool =
    let l0 = uint32 (lhs >>> 32)
    let l1 = uint32 lhs
    let r0 = uint32 (rhs >>> 32)
    let r1 = uint32 rhs
    NPtr.get remap (int l0) = NPtr.get remap (int r0) && NPtr.get remap (int l1) = NPtr.get remap (int r1)

// ---------------------------------------------------------------------------
// buildPositionRemap (internal helper)
// ---------------------------------------------------------------------------

let private buildPositionRemap (remap: nativeptr<uint32>) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (allocator: meshopt_Allocator) =
    let vertices : nativeptr<byte> = NPtr.cast vertex_positions
    let hashFn = vertexHash vertices (3 * sizeof<float32>) vertex_positions_stride
    let equalFn = vertexEqual vertices (3 * sizeof<float32>) vertex_positions_stride

    let vertex_table_size = hashBuckets vertex_count
    let vertex_table = allocator.allocate<uint32>(vertex_table_size)
    NPtr.memset vertex_table 0xFFuy (vertex_table_size * sizeof<uint32>)

    for i = 0 to vertex_count - 1 do
        let index = uint32 i
        let entry = hashLookupIndex vertex_table vertex_table_size hashFn equalFn index ~~~0u
        if NPtr.get vertex_table entry = ~~~0u then
            NPtr.set vertex_table entry index
        NPtr.set remap (int index) (NPtr.get vertex_table entry)

    allocator.deallocate(NPtr.toNI vertex_table)

// ---------------------------------------------------------------------------
// generateVertexRemap (generic internal helper)
// ---------------------------------------------------------------------------

let private generateVertexRemapInternal (remap: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (hashFn: uint32 -> int) (equalFn: uint32 -> uint32 -> bool) (allocator: meshopt_Allocator) : int =
    NPtr.memset remap 0xFFuy (vertex_count * sizeof<uint32>)

    let table_size = hashBuckets vertex_count
    let table = allocator.allocate<uint32>(table_size)
    NPtr.memset table 0xFFuy (table_size * sizeof<uint32>)

    let mutable next_vertex = 0u

    for i = 0 to index_count - 1 do
        let index =
            if NPtr.toNI indices = 0n then uint32 i
            else NPtr.get indices i
        assert (int index < vertex_count)

        if NPtr.get remap (int index) <> ~~~0u then
            () // already remapped, skip
        else
            let entry = hashLookupIndex table table_size hashFn equalFn index ~~~0u

            if NPtr.get table entry = ~~~0u then
                NPtr.set table entry index
                NPtr.set remap (int index) next_vertex
                next_vertex <- next_vertex + 1u
            else
                assert (NPtr.get remap (int (NPtr.get table entry)) <> ~~~0u)
                NPtr.set remap (int index) (NPtr.get remap (int (NPtr.get table entry)))

    assert (int next_vertex <= vertex_count)
    int next_vertex

// ---------------------------------------------------------------------------
// generateShadowBuffer (generic internal helper)
// ---------------------------------------------------------------------------

let private generateShadowBufferInternal (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (hashFn: uint32 -> int) (equalFn: uint32 -> uint32 -> bool) (allocator: meshopt_Allocator) =
    let remap = allocator.allocate<uint32>(vertex_count)
    NPtr.memset remap 0xFFuy (vertex_count * sizeof<uint32>)

    let table_size = hashBuckets vertex_count
    let table = allocator.allocate<uint32>(table_size)
    NPtr.memset table 0xFFuy (table_size * sizeof<uint32>)

    for i = 0 to index_count - 1 do
        let index = NPtr.get indices i
        assert (int index < vertex_count)

        if NPtr.get remap (int index) = ~~~0u then
            let entry = hashLookupIndex table table_size hashFn equalFn index ~~~0u

            if NPtr.get table entry = ~~~0u then
                NPtr.set table entry index

            NPtr.set remap (int index) (NPtr.get table entry)

        NPtr.set destination i (NPtr.get remap (int index))

// ---------------------------------------------------------------------------
// remapVertices (internal helper)
// ---------------------------------------------------------------------------

let private remapVertices (destination: nativeptr<byte>) (vertices: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (remap: nativeptr<uint32>) =
    for i = 0 to vertex_count - 1 do
        if NPtr.get remap i <> ~~~0u then
            assert (int (NPtr.get remap i) < vertex_count)
            NPtr.memcpy
                (NPtr.add destination (int (NPtr.get remap i) * vertex_size))
                (NPtr.add vertices (i * vertex_size))
                vertex_size

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

let meshopt_generateVertexRemap (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertices: nativeint) (vertex_count: int) (vertex_size: int) : int =
    assert (NPtr.toNI indices = 0n || index_count % 3 = 0)
    assert (NPtr.toNI indices <> 0n || index_count = vertex_count)
    assert (vertex_size > 0 && vertex_size <= 256)

    use allocator = new meshopt_Allocator()

    let vb : nativeptr<byte> = NPtr.ofNI vertices
    let hashFn = vertexHash vb vertex_size vertex_size
    let equalFn = vertexEqual vb vertex_size vertex_size

    generateVertexRemapInternal destination indices index_count vertex_count hashFn equalFn allocator

let meshopt_generateVertexRemapMulti (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (streams: nativeptr<meshopt_Stream>) (stream_count: int) : int =
    assert (NPtr.toNI indices = 0n || index_count % 3 = 0)
    assert (NPtr.toNI indices <> 0n || index_count = vertex_count)
    assert (stream_count > 0 && stream_count <= 16)

    for i = 0 to stream_count - 1 do
        let s = NPtr.get streams i
        assert (int s.size > 0 && int s.size <= 256)
        assert (s.size <= s.stride)

    use allocator = new meshopt_Allocator()

    let hashFn = vertexStreamHash streams stream_count
    let equalFn = vertexStreamEqual streams stream_count

    generateVertexRemapInternal destination indices index_count vertex_count hashFn equalFn allocator

let meshopt_generateVertexRemapCustom (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (callback: VertexEqualCallback) (context: nativeint) : int =
    assert (NPtr.toNI indices = 0n || index_count % 3 = 0)
    assert (NPtr.toNI indices <> 0n || index_count = vertex_count)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    use allocator = new meshopt_Allocator()

    let vertex_stride_float = vertex_positions_stride / sizeof<float32>
    let hashFn = vertexCustomHash vertex_positions vertex_stride_float
    let equalFn = vertexCustomEqual vertex_positions vertex_stride_float callback context

    generateVertexRemapInternal destination indices index_count vertex_count hashFn equalFn allocator

let meshopt_remapVertexBuffer (destination: nativeint) (vertices: nativeint) (vertex_count: int) (vertex_size: int) (remap: nativeptr<uint32>) =
    assert (vertex_size > 0 && vertex_size <= 256)

    use allocator = new meshopt_Allocator()

    // support in-place remap
    let mutable vertices = vertices
    if destination = vertices then
        let vertices_copy = allocator.allocate<byte>(vertex_count * vertex_size)
        NPtr.memcpy vertices_copy (NPtr.ofNI<byte> vertices) (vertex_count * vertex_size)
        vertices <- NPtr.toNI vertices_copy

    remapVertices (NPtr.ofNI destination) (NPtr.ofNI vertices) vertex_count vertex_size remap

let meshopt_remapIndexBuffer (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (remap: nativeptr<uint32>) =
    assert (index_count % 3 = 0)

    for i = 0 to index_count - 1 do
        let index =
            if NPtr.toNI indices = 0n then uint32 i
            else NPtr.get indices i
        assert (NPtr.get remap (int index) <> ~~~0u)
        NPtr.set destination i (NPtr.get remap (int index))

let meshopt_generateShadowIndexBuffer (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertices: nativeint) (vertex_count: int) (vertex_size: int) (vertex_stride: int) =
    assert (NPtr.toNI indices <> 0n)
    assert (index_count % 3 = 0)
    assert (vertex_size > 0 && vertex_size <= 256)
    assert (vertex_size <= vertex_stride)

    use allocator = new meshopt_Allocator()

    let vb : nativeptr<byte> = NPtr.ofNI vertices
    let hashFn = vertexHash vb vertex_size vertex_stride
    let equalFn = vertexEqual vb vertex_size vertex_stride

    generateShadowBufferInternal destination indices index_count vertex_count hashFn equalFn allocator

let meshopt_generateShadowIndexBufferMulti (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (streams: nativeptr<meshopt_Stream>) (stream_count: int) =
    assert (NPtr.toNI indices <> 0n)
    assert (index_count % 3 = 0)
    assert (stream_count > 0 && stream_count <= 16)

    for i = 0 to stream_count - 1 do
        let s = NPtr.get streams i
        assert (int s.size > 0 && int s.size <= 256)
        assert (s.size <= s.stride)

    use allocator = new meshopt_Allocator()

    let hashFn = vertexStreamHash streams stream_count
    let equalFn = vertexStreamEqual streams stream_count

    generateShadowBufferInternal destination indices index_count vertex_count hashFn equalFn allocator

let meshopt_generatePositionRemap (destination: nativeptr<uint32>) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) =
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    use allocator = new meshopt_Allocator()

    let vertex_stride_float = vertex_positions_stride / sizeof<float32>
    let hashFn = vertexCustomHash vertex_positions vertex_stride_float
    let equalFn = vertexCustomEqual vertex_positions vertex_stride_float (Unchecked.defaultof<VertexEqualCallback>) 0n

    let table_size = hashBuckets vertex_count
    let table = allocator.allocate<uint32>(table_size)
    NPtr.memset table 0xFFuy (table_size * sizeof<uint32>)

    for i = 0 to vertex_count - 1 do
        let entry = hashLookupIndex table table_size hashFn equalFn (uint32 i) ~~~0u

        if NPtr.get table entry = ~~~0u then
            NPtr.set table entry (uint32 i)

        NPtr.set destination i (NPtr.get table entry)

let meshopt_generateAdjacencyIndexBuffer (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    use allocator = new meshopt_Allocator()

    let next = [| 1; 2; 0; 1 |]

    // build position remap: for each vertex, which other (canonical) vertex does it map to?
    let remap = allocator.allocate<uint32>(vertex_count)
    buildPositionRemap remap vertex_positions vertex_count vertex_positions_stride allocator

    // build edge set; this stores all triangle edges but we can look these up by any other wedge
    let edgeHashFn = edgeHash remap
    let edgeEqualFn = edgeEqual remap

    let edge_table_size = hashBuckets index_count
    let edge_table = allocator.allocate<uint64>(edge_table_size)
    let edge_vertex_table = allocator.allocate<uint32>(edge_table_size)

    NPtr.memset edge_table 0xFFuy (edge_table_size * sizeof<uint64>)
    NPtr.memset edge_vertex_table 0xFFuy (edge_table_size * sizeof<uint32>)

    let mutable i = 0
    while i < index_count do
        for e = 0 to 2 do
            let i0 = NPtr.get indices (i + e)
            let i1 = NPtr.get indices (i + next.[e])
            let i2 = NPtr.get indices (i + next.[e + 1])
            assert (int i0 < vertex_count && int i1 < vertex_count && int i2 < vertex_count)

            let edge = (uint64 i0 <<< 32) ||| uint64 i1
            let entry = hashLookupIndex64 edge_table edge_table_size edgeHashFn edgeEqualFn edge ~~~0UL

            if NPtr.get edge_table entry = ~~~0UL then
                NPtr.set edge_table entry edge
                // store vertex opposite to the edge
                NPtr.set edge_vertex_table entry i2
        i <- i + 3

    // build resulting index buffer: 6 indices for each input triangle
    let mutable i = 0
    while i < index_count do
        let patch = Array.zeroCreate<uint32> 6

        for e = 0 to 2 do
            let i0 = NPtr.get indices (i + e)
            let i1 = NPtr.get indices (i + next.[e])
            assert (int i0 < vertex_count && int i1 < vertex_count)

            // note: this refers to the opposite edge!
            let edge = (uint64 i1 <<< 32) ||| uint64 i0
            let oppe = hashLookupIndex64 edge_table edge_table_size edgeHashFn edgeEqualFn edge ~~~0UL

            patch.[e * 2 + 0] <- i0
            patch.[e * 2 + 1] <- if NPtr.get edge_table oppe = ~~~0UL then i0 else NPtr.get edge_vertex_table oppe

        for j = 0 to 5 do
            NPtr.set destination (i * 2 + j) patch.[j]
        i <- i + 3

let meshopt_generateTessellationIndexBuffer (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    use allocator = new meshopt_Allocator()

    let next = [| 1; 2; 0 |]

    // build position remap: for each vertex, which other (canonical) vertex does it map to?
    let remap = allocator.allocate<uint32>(vertex_count)
    buildPositionRemap remap vertex_positions vertex_count vertex_positions_stride allocator

    // build edge set; this stores all triangle edges but we can look these up by any other wedge
    let edgeHashFn = edgeHash remap
    let edgeEqualFn = edgeEqual remap

    let edge_table_size = hashBuckets index_count
    let edge_table = allocator.allocate<uint64>(edge_table_size)
    NPtr.memset edge_table 0xFFuy (edge_table_size * sizeof<uint64>)

    let mutable i = 0
    while i < index_count do
        for e = 0 to 2 do
            let i0 = NPtr.get indices (i + e)
            let i1 = NPtr.get indices (i + next.[e])
            assert (int i0 < vertex_count && int i1 < vertex_count)

            let edge = (uint64 i0 <<< 32) ||| uint64 i1
            let entry = hashLookupIndex64 edge_table edge_table_size edgeHashFn edgeEqualFn edge ~~~0UL

            if NPtr.get edge_table entry = ~~~0UL then
                NPtr.set edge_table entry edge
        i <- i + 3

    // build resulting index buffer: 12 indices for each input triangle
    let mutable i = 0
    while i < index_count do
        let patch = Array.zeroCreate<uint32> 12

        for e = 0 to 2 do
            let i0 = NPtr.get indices (i + e)
            let i1 = NPtr.get indices (i + next.[e])
            assert (int i0 < vertex_count && int i1 < vertex_count)

            // note: this refers to the opposite edge!
            let edge = (uint64 i1 <<< 32) ||| uint64 i0
            let oppeIdx = hashLookupIndex64 edge_table edge_table_size edgeHashFn edgeEqualFn edge ~~~0UL
            let mutable oppe = NPtr.get edge_table oppeIdx

            // use the same edge if opposite edge doesn't exist (border)
            oppe <- if oppe = ~~~0UL then ((uint64 i0 <<< 32) ||| uint64 i1) else oppe

            // triangle index (0, 1, 2)
            patch.[e] <- i0

            // opposite edge (3, 4; 5, 6; 7, 8)
            patch.[3 + e * 2 + 0] <- uint32 oppe
            patch.[3 + e * 2 + 1] <- uint32 (oppe >>> 32)

            // dominant vertex (9, 10, 11)
            patch.[9 + e] <- NPtr.get remap (int i0)

        for j = 0 to 11 do
            NPtr.set destination (i * 4 + j) patch.[j]
        i <- i + 3

let meshopt_generateProvokingIndexBuffer (destination: nativeptr<uint32>) (reorder: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) : int =
    assert (index_count % 3 = 0)

    use allocator = new meshopt_Allocator()

    let remap = allocator.allocate<uint32>(vertex_count)
    NPtr.memset remap 0xFFuy (vertex_count * sizeof<uint32>)

    // compute vertex valence; this is used to prioritize least used corner
    // note: we use 8-bit counters for performance; for outlier vertices the valence is incorrect but that just affects the heuristic
    let valence = allocator.allocate<byte>(vertex_count)
    NPtr.memset valence 0uy vertex_count

    for i = 0 to index_count - 1 do
        let index = NPtr.get indices i
        assert (int index < vertex_count)
        NPtr.set valence (int index) (NPtr.get valence (int index) + 1uy)

    let mutable reorder_offset = 0u

    // assign provoking vertices; leave the rest for the next pass
    let mutable i = 0
    while i < index_count do
        let mutable a = NPtr.get indices (i + 0)
        let mutable b = NPtr.get indices (i + 1)
        let mutable c = NPtr.get indices (i + 2)
        assert (int a < vertex_count && int b < vertex_count && int c < vertex_count)

        // try to rotate triangle such that provoking vertex hasn't been seen before
        // if multiple vertices are new, prioritize the one with least valence
        // this reduces the risk that a future triangle will have all three vertices seen
        let va = if NPtr.get remap (int a) = ~~~0u then uint32 (NPtr.get valence (int a)) else ~~~0u
        let vb = if NPtr.get remap (int b) = ~~~0u then uint32 (NPtr.get valence (int b)) else ~~~0u
        let vc = if NPtr.get remap (int c) = ~~~0u then uint32 (NPtr.get valence (int c)) else ~~~0u

        if vb <> ~~~0u && vb <= va && vb <= vc then
            // abc -> bca
            let t = a
            a <- b; b <- c; c <- t
        elif vc <> ~~~0u && vc <= va && vc <= vb then
            // abc -> cab
            let t = c
            c <- b; b <- a; a <- t

        let newidx = reorder_offset

        // now remap[a] = ~0u or all three vertices are old
        // recording remap[a] makes it possible to remap future references to the same index, conserving space
        if NPtr.get remap (int a) = ~~~0u then
            NPtr.set remap (int a) newidx

        // we need to clone the provoking vertex to get a unique index
        // if all three are used the choice is arbitrary since no future triangle will be able to reuse any of these
        NPtr.set reorder (int reorder_offset) a
        reorder_offset <- reorder_offset + 1u

        // note: first vertex is final, the other two will be fixed up in next pass
        NPtr.set destination (i + 0) newidx
        NPtr.set destination (i + 1) b
        NPtr.set destination (i + 2) c

        // update vertex valences for corner heuristic
        NPtr.set valence (int a) (NPtr.get valence (int a) - 1uy)
        NPtr.set valence (int b) (NPtr.get valence (int b) - 1uy)
        NPtr.set valence (int c) (NPtr.get valence (int c) - 1uy)

        i <- i + 3

    // remap or clone non-provoking vertices (iterating to skip provoking vertices)
    let mutable step = 1
    let mutable i = 1
    while i < index_count do
        let index = NPtr.get destination i

        if NPtr.get remap (int index) = ~~~0u then
            // we haven't seen the vertex before as a provoking vertex
            // to maintain the reference to the original vertex we need to clone it
            let newidx = reorder_offset

            NPtr.set remap (int index) newidx
            NPtr.set reorder (int reorder_offset) index
            reorder_offset <- reorder_offset + 1u

        NPtr.set destination i (NPtr.get remap (int index))

        i <- i + step
        step <- step ^^^ 3  // alternates between 1 and 2

    assert (int reorder_offset <= vertex_count + index_count / 3)
    int reorder_offset
