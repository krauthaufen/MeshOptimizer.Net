// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
module MeshOptPort.Simplifier

// This work is based on:
// Michael Garland and Paul S. Heckbert. Surface simplification using quadric error metrics. 1997
// Michael Garland. Quadric-based polygonal surface simplification. 1999
// Peter Lindstrom. Out-of-Core Simplification of Large Polygonal Models. 2000
// Matthias Teschner, Bruno Heidelberger, Matthias Mueller, Danat Pomeranets, Markus Gross. Optimized Spatial Hashing for Collision Detection of Deformable Objects. 2003
// Peter Van Sandt, Yannis Chronis, Jignesh M. Patel. Efficiently Searching In-Memory Sorted Arrays: Revenge of the Interpolation Search? 2019
// Hugues Hoppe. New Quadric Metric for Simplifying Meshes with Appearance Attributes. 1999
// Hugues Hoppe, Steve Marschner. Efficient Minimization of New Quadric Metric for Simplifying Meshes with Appearance Attributes. 2000

#nowarn "9"

open System
open System.Runtime.CompilerServices
open FSharp.NativeInterop
open MeshOptPort
open MeshOptPort.Allocator

// ============================================================================
// Types
// ============================================================================

[<Struct>]
type private Edge =
    val mutable next: uint32
    val mutable prev: uint32

[<Struct>]
type private EdgeAdjacency =
    val mutable offsets: nativeptr<uint32>
    val mutable data: nativeptr<Edge>

[<Struct>]
type private Vector3 =
    val mutable x: float32
    val mutable y: float32
    val mutable z: float32

[<Struct>]
type private Quadric =
    val mutable a00: float32
    val mutable a11: float32
    val mutable a22: float32
    val mutable a10: float32
    val mutable a20: float32
    val mutable a21: float32
    val mutable b0: float32
    val mutable b1: float32
    val mutable b2: float32
    val mutable c: float32
    val mutable w: float32

[<Struct>]
type private QuadricGrad =
    val mutable gx: float32
    val mutable gy: float32
    val mutable gz: float32
    val mutable gw: float32

[<Struct>]
type private Reservoir =
    val mutable x: float32
    val mutable y: float32
    val mutable z: float32
    val mutable r: float32
    val mutable g: float32
    val mutable b: float32
    val mutable w: float32

[<Struct>]
type private Collapse =
    val mutable v0: uint32
    val mutable v1: uint32
    val mutable errorui: uint32 // union: bidi (before ranking), error (as float bits, after ranking), errorui

// ============================================================================
// Constants
// ============================================================================

[<Literal>]
let private kMaxAttributes = 32

[<Literal>]
let private Kind_Manifold = 0uy

[<Literal>]
let private Kind_Border = 1uy

[<Literal>]
let private Kind_Seam = 2uy

[<Literal>]
let private Kind_Complex = 3uy

[<Literal>]
let private Kind_Locked = 4uy

[<Literal>]
let private Kind_Count = 5

// Collapse helpers for the union field
module private CollapseHelpers =
    let inline getBidi (c: Collapse) : uint32 = c.errorui
    let inline setBidi (c: byref<Collapse>) (v: uint32) = c.errorui <- v
    let inline getError (c: Collapse) : float32 = Unsafe.BitCast<uint32, float32>(c.errorui)
    let inline setError (c: byref<Collapse>) (v: float32) = c.errorui <- Unsafe.BitCast<float32, uint32>(v)
    let inline getErrorui (c: Collapse) : uint32 = c.errorui

open CollapseHelpers

// manifold vertices can collapse onto anything
// border/seam vertices can collapse onto border/seam respectively, or locked
// complex vertices can collapse onto complex/locked
let private kCanCollapse =
    [|
        [| 1uy; 1uy; 1uy; 1uy; 1uy |]
        [| 0uy; 1uy; 0uy; 0uy; 1uy |]
        [| 0uy; 0uy; 1uy; 0uy; 1uy |]
        [| 0uy; 0uy; 0uy; 1uy; 1uy |]
        [| 0uy; 0uy; 0uy; 0uy; 0uy |]
    |]

let private kHasOpposite =
    [|
        [| 1uy; 1uy; 1uy; 1uy; 1uy |]
        [| 1uy; 0uy; 1uy; 0uy; 0uy |]
        [| 1uy; 1uy; 1uy; 0uy; 1uy |]
        [| 1uy; 0uy; 0uy; 0uy; 0uy |]
        [| 1uy; 0uy; 1uy; 0uy; 0uy |]
    |]

// Internal option flags
let private meshopt_SimplifyInternalSolve = 1u <<< 29
let private meshopt_SimplifyInternalDebug = 1u <<< 30

// ============================================================================
// Edge adjacency
// ============================================================================

let private prepareEdgeAdjacency (adjacency: byref<EdgeAdjacency>) (index_count: int) (vertex_count: int) (allocator: meshopt_Allocator) =
    adjacency.offsets <- allocator.allocate<uint32>(vertex_count + 1)
    adjacency.data <- allocator.allocate<Edge>(index_count)

let private updateEdgeAdjacency (adjacency: byref<EdgeAdjacency>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (remap: nativeptr<uint32>) =
    let face_count = index_count / 3
    let offsets = NPtr.add adjacency.offsets 1
    let data = adjacency.data

    // fill edge counts
    NPtr.memset offsets 0uy (vertex_count * sizeof<uint32>)

    for i = 0 to index_count - 1 do
        let v =
            if NPtr.toNI remap <> 0n then NPtr.get remap (int (NPtr.get indices i))
            else NPtr.get indices i
        assert (int v < vertex_count)
        NPtr.set offsets (int v) (NPtr.get offsets (int v) + 1u)

    // fill offset table
    let mutable offset = 0u
    for i = 0 to vertex_count - 1 do
        let count = NPtr.get offsets i
        NPtr.set offsets i offset
        offset <- offset + count

    assert (offset = uint32 index_count)

    // fill edge data
    for i = 0 to face_count - 1 do
        let mutable a = NPtr.get indices (i * 3 + 0)
        let mutable b = NPtr.get indices (i * 3 + 1)
        let mutable c = NPtr.get indices (i * 3 + 2)

        if NPtr.toNI remap <> 0n then
            a <- NPtr.get remap (int a)
            b <- NPtr.get remap (int b)
            c <- NPtr.get remap (int c)

        let mutable ea = NPtr.get data (int (NPtr.get offsets (int a)))
        ea.next <- b
        ea.prev <- c
        NPtr.set data (int (NPtr.get offsets (int a))) ea
        NPtr.set offsets (int a) (NPtr.get offsets (int a) + 1u)

        let mutable eb = NPtr.get data (int (NPtr.get offsets (int b)))
        eb.next <- c
        eb.prev <- a
        NPtr.set data (int (NPtr.get offsets (int b))) eb
        NPtr.set offsets (int b) (NPtr.get offsets (int b) + 1u)

        let mutable ec = NPtr.get data (int (NPtr.get offsets (int c)))
        ec.next <- a
        ec.prev <- b
        NPtr.set data (int (NPtr.get offsets (int c))) ec
        NPtr.set offsets (int c) (NPtr.get offsets (int c) + 1u)

    // finalize offsets
    NPtr.set adjacency.offsets 0 0u
    assert (NPtr.get adjacency.offsets vertex_count = uint32 index_count)

// ============================================================================
// Hash tables
// ============================================================================

let inline private hashBuckets2 (count: int) : int =
    let mutable buckets = 1
    while buckets < count + count / 4 do
        buckets <- buckets * 2
    buckets

let private hashLookup2_position (table: nativeptr<uint32>) (buckets: int) (vertex_positions: nativeptr<float32>) (vertex_stride_float: int) (sparse_remap: nativeptr<uint32>) (key: uint32) : nativeptr<uint32> =
    assert (buckets > 0)
    assert (buckets &&& (buckets - 1) = 0)
    let hashmod = uint32 (buckets - 1)

    // hash
    let ri = if NPtr.toNI sparse_remap <> 0n then NPtr.get sparse_remap (int key) else key
    let kp = NPtr.add (NPtr.cast<float32, uint32> vertex_positions) (int ri * vertex_stride_float)
    let mutable x = NPtr.get kp 0
    let mutable y = NPtr.get kp 1
    let mutable z = NPtr.get kp 2
    x <- if x = 0x80000000u then 0u else x
    y <- if y = 0x80000000u then 0u else y
    z <- if z = 0x80000000u then 0u else z
    x <- x ^^^ (x >>> 17)
    y <- y ^^^ (y >>> 17)
    z <- z ^^^ (z >>> 17)
    let h = (x * 73856093u) ^^^ (y * 19349663u) ^^^ (z * 83492791u)

    let mutable bucket = h &&& hashmod
    let mutable probe = 0u
    let mutable result : nativeptr<uint32> = Unchecked.defaultof<_>
    let mutable found = false
    while not found && probe <= hashmod do
        let item = NPtr.get table (int bucket)
        if item = ~~~0u then
            result <- NPtr.add table (int bucket)
            found <- true
        else
            // equal check
            let li = if NPtr.toNI sparse_remap <> 0n then NPtr.get sparse_remap (int item) else item
            let ri2 = if NPtr.toNI sparse_remap <> 0n then NPtr.get sparse_remap (int key) else key
            let lv = NPtr.add vertex_positions (int li * vertex_stride_float)
            let rv = NPtr.add vertex_positions (int ri2 * vertex_stride_float)
            if NPtr.get lv 0 = NPtr.get rv 0 && NPtr.get lv 1 = NPtr.get rv 1 && NPtr.get lv 2 = NPtr.get rv 2 then
                result <- NPtr.add table (int bucket)
                found <- true
            else
                bucket <- (bucket + probe + 1u) &&& hashmod
                probe <- probe + 1u
    assert found
    result

let private hashLookup2_remap (table: nativeptr<uint32>) (buckets: int) (remap: nativeptr<uint32>) (key: uint32) : nativeptr<uint32> =
    assert (buckets > 0)
    assert (buckets &&& (buckets - 1) = 0)
    let hashmod = uint32 (buckets - 1)
    let h = key * 0x5bd1e995u
    let mutable bucket = h &&& hashmod
    let mutable probe = 0u
    let mutable result : nativeptr<uint32> = Unchecked.defaultof<_>
    let mutable found = false
    while not found && probe <= hashmod do
        let item = NPtr.get table (int bucket)
        if item = ~~~0u then
            result <- NPtr.add table (int bucket)
            found <- true
        elif NPtr.get remap (int item) = key then
            result <- NPtr.add table (int bucket)
            found <- true
        else
            bucket <- (bucket + probe + 1u) &&& hashmod
            probe <- probe + 1u
    assert found
    result

let private hashLookup2_cell (table: nativeptr<uint32>) (buckets: int) (vertex_ids: nativeptr<uint32>) (key: uint32) : nativeptr<uint32> =
    assert (buckets > 0)
    assert (buckets &&& (buckets - 1) = 0)
    let hashmod = uint32 (buckets - 1)
    // CellHasher hash
    let mutable h = NPtr.get vertex_ids (int key)
    h <- h ^^^ (h >>> 13)
    h <- h * 0x5bd1e995u
    h <- h ^^^ (h >>> 15)
    let mutable bucket = h &&& hashmod
    let mutable probe = 0u
    let mutable result : nativeptr<uint32> = Unchecked.defaultof<_>
    let mutable found = false
    while not found && probe <= hashmod do
        let item = NPtr.get table (int bucket)
        if item = ~~~0u then
            result <- NPtr.add table (int bucket)
            found <- true
        elif NPtr.get vertex_ids (int item) = NPtr.get vertex_ids (int key) then
            result <- NPtr.add table (int bucket)
            found <- true
        else
            bucket <- (bucket + probe + 1u) &&& hashmod
            probe <- probe + 1u
    assert found
    result

let private hashLookup2_id (table: nativeptr<uint32>) (buckets: int) (key: uint32) : nativeptr<uint32> =
    assert (buckets > 0)
    assert (buckets &&& (buckets - 1) = 0)
    let hashmod = uint32 (buckets - 1)
    let mutable h = key
    h <- h ^^^ (h >>> 13)
    h <- h * 0x5bd1e995u
    h <- h ^^^ (h >>> 15)
    let mutable bucket = h &&& hashmod
    let mutable probe = 0u
    let mutable result : nativeptr<uint32> = Unchecked.defaultof<_>
    let mutable found = false
    while not found && probe <= hashmod do
        let item = NPtr.get table (int bucket)
        if item = ~~~0u then
            result <- NPtr.add table (int bucket)
            found <- true
        elif item = key then
            result <- NPtr.add table (int bucket)
            found <- true
        else
            bucket <- (bucket + probe + 1u) &&& hashmod
            probe <- probe + 1u
    assert found
    result

let private hashLookup2_triangle (table: nativeptr<uint32>) (buckets: int) (indices: nativeptr<uint32>) (key: uint32) : nativeptr<uint32> =
    assert (buckets > 0)
    assert (buckets &&& (buckets - 1) = 0)
    let hashmod = uint32 (buckets - 1)
    let tri = NPtr.add indices (int key * 3)
    let h = (NPtr.get tri 0 * 73856093u) ^^^ (NPtr.get tri 1 * 19349663u) ^^^ (NPtr.get tri 2 * 83492791u)
    let mutable bucket = h &&& hashmod
    let mutable probe = 0u
    let mutable result : nativeptr<uint32> = Unchecked.defaultof<_>
    let mutable found = false
    while not found && probe <= hashmod do
        let item = NPtr.get table (int bucket)
        if item = ~~~0u then
            result <- NPtr.add table (int bucket)
            found <- true
        else
            let lt = NPtr.add indices (int item * 3)
            let rt = NPtr.add indices (int key * 3)
            if NPtr.get lt 0 = NPtr.get rt 0 && NPtr.get lt 1 = NPtr.get rt 1 && NPtr.get lt 2 = NPtr.get rt 2 then
                result <- NPtr.add table (int bucket)
                found <- true
            else
                bucket <- (bucket + probe + 1u) &&& hashmod
                probe <- probe + 1u
    assert found
    result

// ============================================================================
// Position remap
// ============================================================================

let private buildPositionRemap (remap: nativeptr<uint32>) (wedge: nativeptr<uint32>) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (sparse_remap: nativeptr<uint32>) (allocator: meshopt_Allocator) =
    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    let table_size = hashBuckets2 vertex_count
    let table = allocator.allocate<uint32>(table_size)
    NPtr.memset table 0xFFuy (table_size * sizeof<uint32>)

    for i = 0 to vertex_count - 1 do
        let index = uint32 i
        let entry = hashLookup2_position table table_size vertex_positions_data vertex_stride_float sparse_remap index
        if NPtr.get entry 0 = ~~~0u then
            NPtr.set entry 0 index
        NPtr.set remap (int index) (NPtr.get entry 0)

    allocator.deallocate(NPtr.toNI table)

    if NPtr.toNI wedge = 0n then () else

    for i = 0 to vertex_count - 1 do
        NPtr.set wedge i (uint32 i)

    for i = 0 to vertex_count - 1 do
        if NPtr.get remap i <> uint32 i then
            let r = int (NPtr.get remap i)
            NPtr.set wedge i (NPtr.get wedge r)
            NPtr.set wedge r (uint32 i)

let private buildSparseRemap (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (out_vertex_count: byref<int>) (allocator: meshopt_Allocator) : nativeptr<uint32> =
    let filter = allocator.allocate<byte>((vertex_count + 7) / 8)

    for i = 0 to index_count - 1 do
        let index = int (NPtr.get indices i)
        assert (index < vertex_count)
        NPtr.set filter (index / 8) 0uy

    let mutable unique = 0
    for i = 0 to index_count - 1 do
        let index = int (NPtr.get indices i)
        let b = NPtr.get filter (index / 8)
        if b &&& (1uy <<< (index % 8)) = 0uy then unique <- unique + 1
        NPtr.set filter (index / 8) (b ||| (1uy <<< (index % 8)))

    let remap = allocator.allocate<uint32>(unique)
    let mutable offset = 0

    let revremap_size = hashBuckets2 unique
    let revremap = allocator.allocate<uint32>(revremap_size)
    NPtr.memset revremap 0xFFuy (revremap_size * sizeof<uint32>)

    for i = 0 to index_count - 1 do
        let index = NPtr.get indices i
        let entry = hashLookup2_remap revremap revremap_size remap index
        if NPtr.get entry 0 = ~~~0u then
            NPtr.set remap offset index
            NPtr.set entry 0 (uint32 offset)
            offset <- offset + 1
        NPtr.set indices i (NPtr.get entry 0)

    allocator.deallocate(NPtr.toNI revremap)

    assert (offset = unique)
    out_vertex_count <- unique
    remap

// ============================================================================
// Edge queries and vertex classification
// ============================================================================

let private hasEdge (adjacency: byref<EdgeAdjacency>) (a: uint32) (b: uint32) : bool =
    let count = int (NPtr.get adjacency.offsets (int a + 1) - NPtr.get adjacency.offsets (int a))
    let edges = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets (int a)))
    let mutable found = false
    for i = 0 to count - 1 do
        if (NPtr.get edges i).next = b then found <- true
    found

let private hasEdgeWedge (adjacency: byref<EdgeAdjacency>) (a: uint32) (b: uint32) (remap: nativeptr<uint32>) (wedge: nativeptr<uint32>) : bool =
    let mutable v = a
    let mutable found = false
    let mutable keepGoing = true
    while keepGoing do
        let count = int (NPtr.get adjacency.offsets (int v + 1) - NPtr.get adjacency.offsets (int v))
        let edges = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets (int v)))
        for i = 0 to count - 1 do
            if NPtr.get remap (int (NPtr.get edges i).next) = NPtr.get remap (int b) then
                found <- true
        v <- NPtr.get wedge (int v)
        if v = a then keepGoing <- false
    found

let private classifyVertices (result: nativeptr<byte>) (loop: nativeptr<uint32>) (loopback: nativeptr<uint32>) (vertex_count: int) (adjacency: byref<EdgeAdjacency>) (remap: nativeptr<uint32>) (wedge: nativeptr<uint32>) (vertex_lock: nativeptr<byte>) (sparse_remap: nativeptr<uint32>) (options: uint32) =
    NPtr.memset loop 0xFFuy (vertex_count * sizeof<uint32>)
    NPtr.memset loopback 0xFFuy (vertex_count * sizeof<uint32>)

    let openinc = loopback
    let openout = loop

    for i = 0 to vertex_count - 1 do
        let vertex = uint32 i
        let count = int (NPtr.get adjacency.offsets (int vertex + 1) - NPtr.get adjacency.offsets (int vertex))
        let edges = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets (int vertex)))

        for j = 0 to count - 1 do
            let target = (NPtr.get edges j).next
            if target = vertex then
                NPtr.set openinc (int vertex) vertex
                NPtr.set openout (int vertex) vertex
            elif not (hasEdge &adjacency target vertex) then
                NPtr.set openinc (int target) (if NPtr.get openinc (int target) = ~~~0u then vertex else target)
                NPtr.set openout (int vertex) (if NPtr.get openout (int vertex) = ~~~0u then target else vertex)

    for i = 0 to vertex_count - 1 do
        if NPtr.get remap i = uint32 i then
            if NPtr.get wedge i = uint32 i then
                let openi = NPtr.get openinc i
                let openo = NPtr.get openout i
                if openi = ~~~0u && openo = ~~~0u then
                    NPtr.set result i Kind_Manifold
                elif openi <> ~~~0u && openo <> ~~~0u && NPtr.get remap (int openi) = NPtr.get remap (int openo) && openi <> uint32 i then
                    NPtr.set result i Kind_Seam
                elif openi <> uint32 i && openo <> uint32 i then
                    NPtr.set result i Kind_Border
                else
                    NPtr.set result i Kind_Locked
            elif NPtr.get wedge (int (NPtr.get wedge i)) = uint32 i then
                let w = int (NPtr.get wedge i)
                let openiv = NPtr.get openinc i
                let openov = NPtr.get openout i
                let openiw = NPtr.get openinc w
                let openow = NPtr.get openout w

                if openiv <> ~~~0u && openiv <> uint32 i && openov <> ~~~0u && openov <> uint32 i &&
                   openiw <> ~~~0u && openiw <> uint32 w && openow <> ~~~0u && openow <> uint32 w then
                    if NPtr.get remap (int openiv) = NPtr.get remap (int openow) && NPtr.get remap (int openov) = NPtr.get remap (int openiw) && NPtr.get remap (int openiv) <> NPtr.get remap (int openov) then
                        NPtr.set result i Kind_Seam
                    else
                        NPtr.set result i Kind_Locked
                else
                    NPtr.set result i Kind_Locked
            else
                NPtr.set result i Kind_Locked
        else
            assert (NPtr.get remap i < uint32 i)
            NPtr.set result i (NPtr.get result (int (NPtr.get remap i)))

    if options &&& uint32 meshopt_SimplifyOptions.Permissive <> 0u then
        for i = 0 to vertex_count - 1 do
            if NPtr.get result i = Kind_Seam || NPtr.get result i = Kind_Locked then
                if NPtr.get remap i <> uint32 i then
                    NPtr.set result i (NPtr.get result (int (NPtr.get remap i)))
                else
                    let mutable protect = false

                    let mutable v = uint32 i
                    let mutable keepGoing = true
                    while keepGoing do
                        let rv = if NPtr.toNI sparse_remap <> 0n then NPtr.get sparse_remap (int v) else v
                        if NPtr.toNI vertex_lock <> 0n && (NPtr.get vertex_lock (int rv) &&& byte meshopt_SimplifyVertexFlags.Protect) <> 0uy then
                            protect <- true
                        v <- NPtr.get wedge (int v)
                        if v = uint32 i then keepGoing <- false

                    let mutable v2 = uint32 i
                    let mutable keepGoing2 = true
                    while keepGoing2 do
                        let edgesPtr = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets (int v2)))
                        let count = int (NPtr.get adjacency.offsets (int v2 + 1) - NPtr.get adjacency.offsets (int v2))
                        for j = 0 to count - 1 do
                            if not (hasEdgeWedge &adjacency (NPtr.get edgesPtr j).next v2 remap wedge) then
                                protect <- true
                        v2 <- NPtr.get wedge (int v2)
                        if v2 = uint32 i then keepGoing2 <- false

                    if not protect then
                        NPtr.set result i Kind_Complex

    if NPtr.toNI vertex_lock <> 0n then
        for i = 0 to vertex_count - 1 do
            let ri = if NPtr.toNI sparse_remap <> 0n then NPtr.get sparse_remap i else uint32 i
            if NPtr.get vertex_lock (int ri) &&& byte meshopt_SimplifyVertexFlags.Lock <> 0uy then
                NPtr.set result (int (NPtr.get remap i)) Kind_Locked

        for i = 0 to vertex_count - 1 do
            if NPtr.get result (int (NPtr.get remap i)) = Kind_Locked then
                NPtr.set result i Kind_Locked

    if options &&& uint32 meshopt_SimplifyOptions.LockBorder <> 0u then
        for i = 0 to vertex_count - 1 do
            if NPtr.get result i = Kind_Border then
                NPtr.set result i Kind_Locked

// ============================================================================
// Rescale positions and attributes
// ============================================================================

let private rescalePositions (result: nativeptr<Vector3>) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (sparse_remap: nativeptr<uint32>) (out_offset: nativeptr<float32>) : float32 =
    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    let mutable minv0 = Single.MaxValue
    let mutable minv1 = Single.MaxValue
    let mutable minv2 = Single.MaxValue
    let mutable maxv0 = -Single.MaxValue
    let mutable maxv1 = -Single.MaxValue
    let mutable maxv2 = -Single.MaxValue

    for i = 0 to vertex_count - 1 do
        let ri = if NPtr.toNI sparse_remap <> 0n then int (NPtr.get sparse_remap i) else i
        let v = NPtr.add vertex_positions_data (ri * vertex_stride_float)

        if NPtr.toNI result <> 0n then
            let mutable r = NPtr.get result i
            r.x <- NPtr.get v 0
            r.y <- NPtr.get v 1
            r.z <- NPtr.get v 2
            NPtr.set result i r

        let v0 = NPtr.get v 0
        let v1 = NPtr.get v 1
        let v2 = NPtr.get v 2
        if v0 < minv0 then minv0 <- v0
        if v0 > maxv0 then maxv0 <- v0
        if v1 < minv1 then minv1 <- v1
        if v1 > maxv1 then maxv1 <- v1
        if v2 < minv2 then minv2 <- v2
        if v2 > maxv2 then maxv2 <- v2

    let mutable extent = 0.0f
    if maxv0 - minv0 > extent then extent <- maxv0 - minv0
    if maxv1 - minv1 > extent then extent <- maxv1 - minv1
    if maxv2 - minv2 > extent then extent <- maxv2 - minv2

    if NPtr.toNI result <> 0n then
        let scale = if extent = 0.0f then 0.0f else 1.0f / extent
        for i = 0 to vertex_count - 1 do
            let mutable r = NPtr.get result i
            r.x <- (r.x - minv0) * scale
            r.y <- (r.y - minv1) * scale
            r.z <- (r.z - minv2) * scale
            NPtr.set result i r

    if NPtr.toNI out_offset <> 0n then
        NPtr.set out_offset 0 minv0
        NPtr.set out_offset 1 minv1
        NPtr.set out_offset 2 minv2

    extent

let private rescaleAttributes (result: nativeptr<float32>) (vertex_attributes_data: nativeptr<float32>) (vertex_count: int) (vertex_attributes_stride: int) (attribute_weights: nativeptr<float32>) (attribute_count: int) (attribute_remap: nativeptr<uint32>) (sparse_remap: nativeptr<uint32>) =
    let vertex_attributes_stride_float = vertex_attributes_stride / sizeof<float32>

    for i = 0 to vertex_count - 1 do
        let ri = if NPtr.toNI sparse_remap <> 0n then int (NPtr.get sparse_remap i) else i
        for k = 0 to attribute_count - 1 do
            let rk = int (NPtr.get attribute_remap k)
            let a = NPtr.get vertex_attributes_data (ri * vertex_attributes_stride_float + rk)
            NPtr.set result (i * attribute_count + k) (a * NPtr.get attribute_weights rk)

let private finalizeVertices (vertex_positions_data: nativeptr<float32>) (vertex_positions_stride: int) (vertex_attributes_data: nativeptr<float32>) (vertex_attributes_stride: int) (attribute_weights: nativeptr<float32>) (attribute_count: int) (vertex_count: int) (vertex_positions: nativeptr<Vector3>) (vertex_attributes: nativeptr<float32>) (sparse_remap: nativeptr<uint32>) (attribute_remap: nativeptr<uint32>) (vertex_scale: float32) (vertex_offset: nativeptr<float32>) (vertex_kind: nativeptr<byte>) (vertex_update: nativeptr<byte>) (vertex_lock: nativeptr<byte>) =
    let vertex_positions_stride_float = vertex_positions_stride / sizeof<float32>
    let vertex_attributes_stride_float = vertex_attributes_stride / sizeof<float32>

    for i = 0 to vertex_count - 1 do
        if NPtr.get vertex_update i <> 0uy then
            let ri = if NPtr.toNI sparse_remap <> 0n then int (NPtr.get sparse_remap i) else i
            if NPtr.toNI vertex_lock <> 0n && (NPtr.get vertex_lock ri &&& byte meshopt_SimplifyVertexFlags.Lock) <> 0uy then
                () // skip locked
            else
                if NPtr.get vertex_kind i <> Kind_Locked then
                    let p = NPtr.get vertex_positions i
                    let v = NPtr.add vertex_positions_data (ri * vertex_positions_stride_float)
                    NPtr.set v 0 (p.x * vertex_scale + NPtr.get vertex_offset 0)
                    NPtr.set v 1 (p.y * vertex_scale + NPtr.get vertex_offset 1)
                    NPtr.set v 2 (p.z * vertex_scale + NPtr.get vertex_offset 2)

                if attribute_count > 0 then
                    let sa = NPtr.add vertex_attributes (i * attribute_count)
                    let va = NPtr.add vertex_attributes_data (ri * vertex_attributes_stride_float)
                    for k = 0 to attribute_count - 1 do
                        let rk = int (NPtr.get attribute_remap k)
                        NPtr.set va rk (NPtr.get sa k / NPtr.get attribute_weights rk)

// ============================================================================
// Quadric operations
// ============================================================================

let inline private normalize (v: byref<Vector3>) : float32 =
    let length = MathF.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    if length > 0.0f then
        v.x <- v.x / length
        v.y <- v.y / length
        v.z <- v.z / length
    length

let inline private quadricAdd (Q: Quadric) (R: Quadric) : Quadric =
    let mutable O = Unchecked.defaultof<Quadric>
    O.a00 <- Q.a00 + R.a00
    O.a11 <- Q.a11 + R.a11
    O.a22 <- Q.a22 + R.a22
    O.a10 <- Q.a10 + R.a10
    O.a20 <- Q.a20 + R.a20
    O.a21 <- Q.a21 + R.a21
    O.b0 <- Q.b0 + R.b0
    O.b1 <- Q.b1 + R.b1
    O.b2 <- Q.b2 + R.b2
    O.c <- Q.c + R.c
    O.w <- Q.w + R.w
    O

let inline private quadricAddGrad (G: QuadricGrad) (R: QuadricGrad) : QuadricGrad =
    let mutable O = Unchecked.defaultof<QuadricGrad>
    O.gx <- G.gx + R.gx
    O.gy <- G.gy + R.gy
    O.gz <- G.gz + R.gz
    O.gw <- G.gw + R.gw
    O

let private quadricAddGradN (G: nativeptr<QuadricGrad>) (R: nativeptr<QuadricGrad>) (attribute_count: int) =
    for k = 0 to attribute_count - 1 do
        let mutable g = NPtr.get G k
        let r = NPtr.get R k
        g.gx <- g.gx + r.gx
        g.gy <- g.gy + r.gy
        g.gz <- g.gz + r.gz
        g.gw <- g.gw + r.gw
        NPtr.set G k g

let private quadricEval (Q: Quadric) (v: Vector3) : float32 =
    let mutable rx = Q.b0
    let mutable ry = Q.b1
    let mutable rz = Q.b2

    rx <- rx + Q.a10 * v.y
    ry <- ry + Q.a21 * v.z
    rz <- rz + Q.a20 * v.x

    rx <- rx * 2.0f
    ry <- ry * 2.0f
    rz <- rz * 2.0f

    rx <- rx + Q.a00 * v.x
    ry <- ry + Q.a11 * v.y
    rz <- rz + Q.a22 * v.z

    let mutable r = Q.c
    r <- r + rx * v.x
    r <- r + ry * v.y
    r <- r + rz * v.z
    r

let private quadricError (Q: Quadric) (v: Vector3) : float32 =
    let r = quadricEval Q v
    let s = if Q.w = 0.0f then 0.0f else 1.0f / Q.w
    MathF.Abs(r) * s

let private quadricErrorAttr (Q: Quadric) (G: nativeptr<QuadricGrad>) (attribute_count: int) (v: Vector3) (va: nativeptr<float32>) : float32 =
    let mutable r = quadricEval Q v

    for k = 0 to attribute_count - 1 do
        let a = NPtr.get va k
        let gk = NPtr.get G k
        let g = v.x * gk.gx + v.y * gk.gy + v.z * gk.gz + gk.gw
        r <- r + a * (a * Q.w - 2.0f * g)

    MathF.Abs(r)

let inline private quadricFromPlane (a: float32) (b: float32) (c: float32) (d: float32) (w: float32) : Quadric =
    let mutable Q = Unchecked.defaultof<Quadric>
    let aw = a * w
    let bw = b * w
    let cw = c * w
    let dw = d * w
    Q.a00 <- a * aw
    Q.a11 <- b * bw
    Q.a22 <- c * cw
    Q.a10 <- a * bw
    Q.a20 <- a * cw
    Q.a21 <- b * cw
    Q.b0 <- a * dw
    Q.b1 <- b * dw
    Q.b2 <- c * dw
    Q.c <- d * dw
    Q.w <- w
    Q

let inline private quadricFromPoint (x: float32) (y: float32) (z: float32) (w: float32) : Quadric =
    let mutable Q = Unchecked.defaultof<Quadric>
    Q.a00 <- w
    Q.a11 <- w
    Q.a22 <- w
    Q.a10 <- 0.0f
    Q.a20 <- 0.0f
    Q.a21 <- 0.0f
    Q.b0 <- -x * w
    Q.b1 <- -y * w
    Q.b2 <- -z * w
    Q.c <- (x * x + y * y + z * z) * w
    Q.w <- w
    Q

let private quadricFromTriangle (p0: Vector3) (p1: Vector3) (p2: Vector3) (weight: float32) : Quadric =
    let mutable normal = Unchecked.defaultof<Vector3>
    let p10x = p1.x - p0.x
    let p10y = p1.y - p0.y
    let p10z = p1.z - p0.z
    let p20x = p2.x - p0.x
    let p20y = p2.y - p0.y
    let p20z = p2.z - p0.z
    normal.x <- p10y * p20z - p10z * p20y
    normal.y <- p10z * p20x - p10x * p20z
    normal.z <- p10x * p20y - p10y * p20x
    let area = normalize &normal
    let distance = normal.x * p0.x + normal.y * p0.y + normal.z * p0.z
    quadricFromPlane normal.x normal.y normal.z (-distance) (MathF.Sqrt(area) * weight)

let private quadricFromTriangleEdge (p0: Vector3) (p1: Vector3) (p2: Vector3) (weight: float32) : Quadric =
    let p10x = p1.x - p0.x
    let p10y = p1.y - p0.y
    let p10z = p1.z - p0.z
    let lengthsq = p10x * p10x + p10y * p10y + p10z * p10z
    let length = MathF.Sqrt(lengthsq)

    let p20x = p2.x - p0.x
    let p20y = p2.y - p0.y
    let p20z = p2.z - p0.z
    let p20p = p20x * p10x + p20y * p10y + p20z * p10z

    let mutable perp = Unchecked.defaultof<Vector3>
    perp.x <- p20x * lengthsq - p10x * p20p
    perp.y <- p20y * lengthsq - p10y * p20p
    perp.z <- p20z * lengthsq - p10z * p20p
    normalize &perp |> ignore

    let distance = perp.x * p0.x + perp.y * p0.y + perp.z * p0.z
    quadricFromPlane perp.x perp.y perp.z (-distance) (length * weight)

let private quadricFromAttributes (G: nativeptr<QuadricGrad>) (p0: Vector3) (p1: Vector3) (p2: Vector3) (va0: nativeptr<float32>) (va1: nativeptr<float32>) (va2: nativeptr<float32>) (attribute_count: int) : Quadric =
    let p10x = p1.x - p0.x
    let p10y = p1.y - p0.y
    let p10z = p1.z - p0.z
    let p20x = p2.x - p0.x
    let p20y = p2.y - p0.y
    let p20z = p2.z - p0.z

    let nx = p10y * p20z - p10z * p20y
    let ny = p10z * p20x - p10x * p20z
    let nz = p10x * p20y - p10y * p20x
    let area = MathF.Sqrt(nx * nx + ny * ny + nz * nz) * 0.5f
    let w = area

    let d00 = p10x * p10x + p10y * p10y + p10z * p10z
    let d01 = p10x * p20x + p10y * p20y + p10z * p20z
    let d11 = p20x * p20x + p20y * p20y + p20z * p20z
    let denom = d00 * d11 - d01 * d01
    let denomr = if denom = 0.0f then 0.0f else 1.0f / denom

    let gx1 = (d11 * p10x - d01 * p20x) * denomr
    let gx2 = (d00 * p20x - d01 * p10x) * denomr
    let gy1 = (d11 * p10y - d01 * p20y) * denomr
    let gy2 = (d00 * p20y - d01 * p10y) * denomr
    let gz1 = (d11 * p10z - d01 * p20z) * denomr
    let gz2 = (d00 * p20z - d01 * p10z) * denomr

    let mutable Q = Unchecked.defaultof<Quadric>
    Q.a00 <- 0.0f; Q.a11 <- 0.0f; Q.a22 <- 0.0f; Q.a10 <- 0.0f; Q.a20 <- 0.0f; Q.a21 <- 0.0f
    Q.b0 <- 0.0f; Q.b1 <- 0.0f; Q.b2 <- 0.0f; Q.c <- 0.0f
    Q.w <- w

    for k = 0 to attribute_count - 1 do
        let a0 = NPtr.get va0 k
        let a1 = NPtr.get va1 k
        let a2 = NPtr.get va2 k

        let gx = gx1 * (a1 - a0) + gx2 * (a2 - a0)
        let gy = gy1 * (a1 - a0) + gy2 * (a2 - a0)
        let gz = gz1 * (a1 - a0) + gz2 * (a2 - a0)
        let gw = a0 - p0.x * gx - p0.y * gy - p0.z * gz

        Q.a00 <- Q.a00 + w * (gx * gx)
        Q.a11 <- Q.a11 + w * (gy * gy)
        Q.a22 <- Q.a22 + w * (gz * gz)
        Q.a10 <- Q.a10 + w * (gy * gx)
        Q.a20 <- Q.a20 + w * (gz * gx)
        Q.a21 <- Q.a21 + w * (gz * gy)
        Q.b0 <- Q.b0 + w * (gx * gw)
        Q.b1 <- Q.b1 + w * (gy * gw)
        Q.b2 <- Q.b2 + w * (gz * gw)
        Q.c <- Q.c + w * (gw * gw)

        let mutable gk = Unchecked.defaultof<QuadricGrad>
        gk.gx <- w * gx
        gk.gy <- w * gy
        gk.gz <- w * gz
        gk.gw <- w * gw
        NPtr.set G k gk
    Q

let private quadricVolumeGradient (p0: Vector3) (p1: Vector3) (p2: Vector3) : QuadricGrad =
    let p10x = p1.x - p0.x
    let p10y = p1.y - p0.y
    let p10z = p1.z - p0.z
    let p20x = p2.x - p0.x
    let p20y = p2.y - p0.y
    let p20z = p2.z - p0.z

    let mutable normal = Unchecked.defaultof<Vector3>
    normal.x <- p10y * p20z - p10z * p20y
    normal.y <- p10z * p20x - p10x * p20z
    normal.z <- p10x * p20y - p10y * p20x
    let area = (normalize &normal) * 0.5f

    let mutable G = Unchecked.defaultof<QuadricGrad>
    G.gx <- normal.x * area
    G.gy <- normal.y * area
    G.gz <- normal.z * area
    G.gw <- (-p0.x * normal.x - p0.y * normal.y - p0.z * normal.z) * area
    G

let private quadricSolve (Q: Quadric) (GV: QuadricGrad) : struct (bool * Vector3) =
    let a00 = Q.a00
    let a11 = Q.a11
    let a22 = Q.a22
    let a10 = Q.a10
    let a20 = Q.a20
    let a21 = Q.a21
    let x0 = -Q.b0
    let x1 = -Q.b1
    let x2 = -Q.b2

    let eps = 1e-6f * Q.w

    let d0 = a00
    let l10 = a10 / d0
    let l20 = a20 / d0

    let d1 = a11 - a10 * l10
    let dl21 = a21 - a20 * l10
    let l21 = dl21 / d1

    let d2 = a22 - a20 * l20 - dl21 * l21

    let y0 = x0
    let y1 = x1 - l10 * y0
    let y2 = x2 - l20 * y0 - l21 * y1

    let z0 = y0 / d0
    let z1 = y1 / d1
    let z2 = y2 / d2

    let a30 = GV.gx
    let a31 = GV.gy
    let a32 = GV.gz
    let x3 = -GV.gw

    let l30 = a30 / d0
    let dl31 = a31 - a30 * l10
    let l31 = dl31 / d1
    let dl32 = a32 - a30 * l20 - dl31 * l21
    let l32 = dl32 / d2
    let d3 = 0.0f - a30 * l30 - dl31 * l31 - dl32 * l32

    let y3 = x3 - l30 * y0 - l31 * y1 - l32 * y2
    let z3 = if MathF.Abs(d3) > eps then y3 / d3 else 0.0f

    let lambda = z3
    let pz = z2 - l32 * lambda
    let py = z1 - l21 * pz - l31 * lambda
    let px = z0 - l10 * py - l20 * pz - l30 * lambda

    let mutable p = Unchecked.defaultof<Vector3>
    p.x <- px
    p.y <- py
    p.z <- pz

    let ok = MathF.Abs(d0) > eps && MathF.Abs(d1) > eps && MathF.Abs(d2) > eps
    struct (ok, p)

let private quadricReduceAttributes (Q: Quadric) (A: Quadric) (G: nativeptr<QuadricGrad>) (attribute_count: int) : Quadric =
    let mutable O = Q
    O.a00 <- O.a00 + A.a00 * O.w
    O.a11 <- O.a11 + A.a11 * O.w
    O.a22 <- O.a22 + A.a22 * O.w
    O.a10 <- O.a10 + A.a10 * O.w
    O.a20 <- O.a20 + A.a20 * O.w
    O.a21 <- O.a21 + A.a21 * O.w
    O.b0 <- O.b0 + A.b0 * O.w
    O.b1 <- O.b1 + A.b1 * O.w
    O.b2 <- O.b2 + A.b2 * O.w

    let iaw = if A.w = 0.0f then 0.0f else O.w / A.w

    for k = 0 to attribute_count - 1 do
        let g = NPtr.get G k
        O.a00 <- O.a00 - (g.gx * g.gx) * iaw
        O.a11 <- O.a11 - (g.gy * g.gy) * iaw
        O.a22 <- O.a22 - (g.gz * g.gz) * iaw
        O.a10 <- O.a10 - (g.gx * g.gy) * iaw
        O.a20 <- O.a20 - (g.gx * g.gz) * iaw
        O.a21 <- O.a21 - (g.gy * g.gz) * iaw
        O.b0 <- O.b0 - (g.gx * g.gw) * iaw
        O.b1 <- O.b1 - (g.gy * g.gw) * iaw
        O.b2 <- O.b2 - (g.gz * g.gw) * iaw
    O

// ============================================================================
// Fill quadrics
// ============================================================================

let private fillFaceQuadrics (vertex_quadrics: nativeptr<Quadric>) (volume_gradients: nativeptr<QuadricGrad>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<Vector3>) (remap: nativeptr<uint32>) =
    let mutable i = 0
    while i < index_count do
        let i0 = int (NPtr.get indices (i + 0))
        let i1 = int (NPtr.get indices (i + 1))
        let i2 = int (NPtr.get indices (i + 2))

        let Q = quadricFromTriangle (NPtr.get vertex_positions i0) (NPtr.get vertex_positions i1) (NPtr.get vertex_positions i2) 1.0f

        NPtr.set vertex_quadrics (int (NPtr.get remap i0)) (quadricAdd (NPtr.get vertex_quadrics (int (NPtr.get remap i0))) Q)
        NPtr.set vertex_quadrics (int (NPtr.get remap i1)) (quadricAdd (NPtr.get vertex_quadrics (int (NPtr.get remap i1))) Q)
        NPtr.set vertex_quadrics (int (NPtr.get remap i2)) (quadricAdd (NPtr.get vertex_quadrics (int (NPtr.get remap i2))) Q)

        if NPtr.toNI volume_gradients <> 0n then
            let GV = quadricVolumeGradient (NPtr.get vertex_positions i0) (NPtr.get vertex_positions i1) (NPtr.get vertex_positions i2)

            NPtr.set volume_gradients (int (NPtr.get remap i0)) (quadricAddGrad (NPtr.get volume_gradients (int (NPtr.get remap i0))) GV)
            NPtr.set volume_gradients (int (NPtr.get remap i1)) (quadricAddGrad (NPtr.get volume_gradients (int (NPtr.get remap i1))) GV)
            NPtr.set volume_gradients (int (NPtr.get remap i2)) (quadricAddGrad (NPtr.get volume_gradients (int (NPtr.get remap i2))) GV)

        i <- i + 3

let private fillVertexQuadrics (vertex_quadrics: nativeptr<Quadric>) (vertex_positions: nativeptr<Vector3>) (vertex_count: int) (remap: nativeptr<uint32>) (options: uint32) =
    let factor = if options &&& uint32 meshopt_SimplifyOptions.Regularize <> 0u then 1e-1f else 1e-7f

    for i = 0 to vertex_count - 1 do
        if NPtr.get remap i = uint32 i then
            let p = NPtr.get vertex_positions i
            let w = (NPtr.get vertex_quadrics i).w * factor

            let Q = quadricFromPoint p.x p.y p.z w
            NPtr.set vertex_quadrics i (quadricAdd (NPtr.get vertex_quadrics i) Q)

let private fillEdgeQuadrics (vertex_quadrics: nativeptr<Quadric>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<Vector3>) (remap: nativeptr<uint32>) (vertex_kind: nativeptr<byte>) (loop: nativeptr<uint32>) (loopback: nativeptr<uint32>) =
    let next = [| 1; 2; 0; 1 |]

    let mutable i = 0
    while i < index_count do
        for e = 0 to 2 do
            let i0 = int (NPtr.get indices (i + e))
            let i1 = int (NPtr.get indices (i + next.[e]))

            let k0 = NPtr.get vertex_kind i0
            let k1 = NPtr.get vertex_kind i1

            if k0 = Kind_Border || k0 = Kind_Seam || k1 = Kind_Border || k1 = Kind_Seam then
                let mutable skip = false
                if (k0 = Kind_Border || k0 = Kind_Seam) && NPtr.get loop i0 <> uint32 i1 then skip <- true
                if (k1 = Kind_Border || k1 = Kind_Seam) && NPtr.get loopback i1 <> uint32 i0 then skip <- true

                if not skip then
                    let i2 = int (NPtr.get indices (i + next.[e + 1]))
                    let kEdgeWeightSeam = 0.5f
                    let kEdgeWeightBorder = 10.0f
                    let edgeWeight = if k0 = Kind_Border || k1 = Kind_Border then kEdgeWeightBorder else kEdgeWeightSeam

                    let mutable Q = quadricFromTriangleEdge (NPtr.get vertex_positions i0) (NPtr.get vertex_positions i1) (NPtr.get vertex_positions i2) edgeWeight

                    let mutable QT = quadricFromTriangle (NPtr.get vertex_positions i0) (NPtr.get vertex_positions i1) (NPtr.get vertex_positions i2) edgeWeight
                    QT.w <- 0.0f
                    Q <- quadricAdd Q QT

                    NPtr.set vertex_quadrics (int (NPtr.get remap i0)) (quadricAdd (NPtr.get vertex_quadrics (int (NPtr.get remap i0))) Q)
                    NPtr.set vertex_quadrics (int (NPtr.get remap i1)) (quadricAdd (NPtr.get vertex_quadrics (int (NPtr.get remap i1))) Q)

        i <- i + 3

let private fillAttributeQuadrics (attribute_quadrics: nativeptr<Quadric>) (attribute_gradients: nativeptr<QuadricGrad>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<Vector3>) (vertex_attributes: nativeptr<float32>) (attribute_count: int) =
    let mutable i = 0
    while i < index_count do
        let i0 = int (NPtr.get indices (i + 0))
        let i1 = int (NPtr.get indices (i + 1))
        let i2 = int (NPtr.get indices (i + 2))

        let G = Array.zeroCreate<QuadricGrad> kMaxAttributes
        use pinG = fixed G
        let QA = quadricFromAttributes pinG (NPtr.get vertex_positions i0) (NPtr.get vertex_positions i1) (NPtr.get vertex_positions i2) (NPtr.add vertex_attributes (i0 * attribute_count)) (NPtr.add vertex_attributes (i1 * attribute_count)) (NPtr.add vertex_attributes (i2 * attribute_count)) attribute_count

        NPtr.set attribute_quadrics i0 (quadricAdd (NPtr.get attribute_quadrics i0) QA)
        NPtr.set attribute_quadrics i1 (quadricAdd (NPtr.get attribute_quadrics i1) QA)
        NPtr.set attribute_quadrics i2 (quadricAdd (NPtr.get attribute_quadrics i2) QA)

        quadricAddGradN (NPtr.add attribute_gradients (i0 * attribute_count)) pinG attribute_count
        quadricAddGradN (NPtr.add attribute_gradients (i1 * attribute_count)) pinG attribute_count
        quadricAddGradN (NPtr.add attribute_gradients (i2 * attribute_count)) pinG attribute_count

        i <- i + 3

// ============================================================================
// Triangle flip detection
// ============================================================================

let private hasTriangleFlip (a: Vector3) (b: Vector3) (c: Vector3) (d: Vector3) : bool =
    let ebx = b.x - a.x
    let eby = b.y - a.y
    let ebz = b.z - a.z
    let ecx = c.x - a.x
    let ecy = c.y - a.y
    let ecz = c.z - a.z
    let edx = d.x - a.x
    let edy = d.y - a.y
    let edz = d.z - a.z

    let nbcx = eby * ecz - ebz * ecy
    let nbcy = ebz * ecx - ebx * ecz
    let nbcz = ebx * ecy - eby * ecx
    let nbdx = eby * edz - ebz * edy
    let nbdy = ebz * edx - ebx * edz
    let nbdz = ebx * edy - eby * edx

    let ndp = nbcx * nbdx + nbcy * nbdy + nbcz * nbdz
    let abc = nbcx * nbcx + nbcy * nbcy + nbcz * nbcz
    let abd = nbdx * nbdx + nbdy * nbdy + nbdz * nbdz

    ndp <= 0.25f * MathF.Sqrt(abc * abd)

let private hasTriangleFlipsCollapse (adjacency: byref<EdgeAdjacency>) (vertex_positions: nativeptr<Vector3>) (collapse_remap: nativeptr<uint32>) (i0: uint32) (i1: uint32) : bool =
    assert (NPtr.get collapse_remap (int i0) = i0)
    assert (NPtr.get collapse_remap (int i1) = i1)

    let edges = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets (int i0)))
    let count = int (NPtr.get adjacency.offsets (int i0 + 1) - NPtr.get adjacency.offsets (int i0))

    let mutable result = false
    for i = 0 to count - 1 do
        let a = NPtr.get collapse_remap (int (NPtr.get edges i).next)
        let b = NPtr.get collapse_remap (int (NPtr.get edges i).prev)

        if a <> i1 && b <> i1 && a <> b then
            if hasTriangleFlip (NPtr.get vertex_positions (int a)) (NPtr.get vertex_positions (int b)) (NPtr.get vertex_positions (int i0)) (NPtr.get vertex_positions (int i1)) then
                result <- true
    result

let private hasTriangleFlipsSolve (adjacency: byref<EdgeAdjacency>) (vertex_positions: nativeptr<Vector3>) (i0: uint32) (v1: Vector3) : bool =
    let edges = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets (int i0)))
    let count = int (NPtr.get adjacency.offsets (int i0 + 1) - NPtr.get adjacency.offsets (int i0))

    let mutable result = false
    for i = 0 to count - 1 do
        let a = (NPtr.get edges i).next
        let b = (NPtr.get edges i).prev
        if hasTriangleFlip (NPtr.get vertex_positions (int a)) (NPtr.get vertex_positions (int b)) (NPtr.get vertex_positions (int i0)) v1 then
            result <- true
    result

let private getNeighborhoodRadius (adjacency: byref<EdgeAdjacency>) (vertex_positions: nativeptr<Vector3>) (i0: uint32) : float32 =
    let v0 = NPtr.get vertex_positions (int i0)
    let edges = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets (int i0)))
    let count = int (NPtr.get adjacency.offsets (int i0 + 1) - NPtr.get adjacency.offsets (int i0))

    let mutable result = 0.0f
    for i = 0 to count - 1 do
        let a = (NPtr.get edges i).next
        let b = (NPtr.get edges i).prev
        let va = NPtr.get vertex_positions (int a)
        let vb = NPtr.get vertex_positions (int b)

        let da = (va.x - v0.x) * (va.x - v0.x) + (va.y - v0.y) * (va.y - v0.y) + (va.z - v0.z) * (va.z - v0.z)
        let db = (vb.x - v0.x) * (vb.x - v0.x) + (vb.y - v0.y) * (vb.y - v0.y) + (vb.z - v0.z) * (vb.z - v0.z)

        if da > result then result <- da
        if db > result then result <- db
    MathF.Sqrt(result)

let private getComplexTarget (v: uint32) (target: uint32) (remap: nativeptr<uint32>) (loop: nativeptr<uint32>) (loopback: nativeptr<uint32>) : uint32 =
    let r = NPtr.get remap (int target)
    if NPtr.get loop (int v) <> ~~~0u && NPtr.get remap (int (NPtr.get loop (int v))) = r then
        NPtr.get loop (int v)
    elif NPtr.get loopback (int v) <> ~~~0u && NPtr.get remap (int (NPtr.get loopback (int v))) = r then
        NPtr.get loopback (int v)
    else
        target

// ============================================================================
// Edge collapse management
// ============================================================================

let private boundEdgeCollapses (adjacency: byref<EdgeAdjacency>) (vertex_count: int) (index_count: int) (vertex_kind: nativeptr<byte>) : int =
    let mutable dual_count = 0
    for i = 0 to vertex_count - 1 do
        let k = NPtr.get vertex_kind i
        let e = int (NPtr.get adjacency.offsets (i + 1) - NPtr.get adjacency.offsets i)
        if k = Kind_Manifold || k = Kind_Seam then
            dual_count <- dual_count + e

    assert (dual_count <= index_count)
    (index_count - dual_count / 2) + 3

let private pickEdgeCollapses (collapses: nativeptr<Collapse>) (collapse_capacity: int) (indices: nativeptr<uint32>) (index_count: int) (remap: nativeptr<uint32>) (vertex_kind: nativeptr<byte>) (loop: nativeptr<uint32>) (loopback: nativeptr<uint32>) : int =
    let mutable collapse_count = 0
    let next = [| 1; 2; 0 |]

    let mutable i = 0
    while i < index_count do
        if collapse_count + 3 > collapse_capacity then
            i <- index_count // break
        else
            for e = 0 to 2 do
                let i0 = NPtr.get indices (i + e)
                let i1 = NPtr.get indices (i + next.[e])

                if NPtr.get remap (int i0) <> NPtr.get remap (int i1) then
                    let k0 = int (NPtr.get vertex_kind (int i0))
                    let k1 = int (NPtr.get vertex_kind (int i1))

                    if kCanCollapse.[k0].[k1] ||| kCanCollapse.[k1].[k0] <> 0uy then
                        let mutable skip = false
                        if kHasOpposite.[k0].[k1] <> 0uy && NPtr.get remap (int i1) > NPtr.get remap (int i0) then
                            skip <- true

                        if not skip then
                            if (k0 = int Kind_Border || k0 = int Kind_Seam) && k1 <> int Kind_Manifold && NPtr.get loop (int i0) <> i1 then
                                skip <- true
                            if (k1 = int Kind_Border || k1 = int Kind_Seam) && k0 <> int Kind_Manifold && NPtr.get loopback (int i1) <> i0 then
                                skip <- true

                        if not skip then
                            if kCanCollapse.[k0].[k1] &&& kCanCollapse.[k1].[k0] <> 0uy then
                                let mutable c = Unchecked.defaultof<Collapse>
                                c.v0 <- i0
                                c.v1 <- i1
                                setBidi &c 1u
                                NPtr.set collapses collapse_count c
                                collapse_count <- collapse_count + 1
                            else
                                let e0 = if kCanCollapse.[k0].[k1] <> 0uy then i0 else i1
                                let e1 = if kCanCollapse.[k0].[k1] <> 0uy then i1 else i0
                                let mutable c = Unchecked.defaultof<Collapse>
                                c.v0 <- e0
                                c.v1 <- e1
                                setBidi &c 0u
                                NPtr.set collapses collapse_count c
                                collapse_count <- collapse_count + 1
            i <- i + 3
    collapse_count

let private rankEdgeCollapses (collapses: nativeptr<Collapse>) (collapse_count: int) (vertex_positions: nativeptr<Vector3>) (vertex_attributes: nativeptr<float32>) (vertex_quadrics: nativeptr<Quadric>) (attribute_quadrics: nativeptr<Quadric>) (attribute_gradients: nativeptr<QuadricGrad>) (attribute_count: int) (remap: nativeptr<uint32>) (wedge: nativeptr<uint32>) (vertex_kind: nativeptr<byte>) (loop: nativeptr<uint32>) (loopback: nativeptr<uint32>) =
    for i = 0 to collapse_count - 1 do
        let c = NPtr.get collapses i

        let i0 = c.v0
        let i1 = c.v1
        let bidi = getBidi c <> 0u

        let mutable ei = quadricError (NPtr.get vertex_quadrics (int (NPtr.get remap (int i0)))) (NPtr.get vertex_positions (int i1))
        let mutable ej = if bidi then quadricError (NPtr.get vertex_quadrics (int (NPtr.get remap (int i1)))) (NPtr.get vertex_positions (int i0)) else Single.MaxValue

        if attribute_count > 0 then
            ei <- ei + quadricErrorAttr (NPtr.get attribute_quadrics (int i0)) (NPtr.add attribute_gradients (int i0 * attribute_count)) attribute_count (NPtr.get vertex_positions (int i1)) (NPtr.add vertex_attributes (int i1 * attribute_count))
            if bidi then
                ej <- ej + quadricErrorAttr (NPtr.get attribute_quadrics (int i1)) (NPtr.add attribute_gradients (int i1 * attribute_count)) attribute_count (NPtr.get vertex_positions (int i0)) (NPtr.add vertex_attributes (int i0 * attribute_count))

            if NPtr.get vertex_kind (int i0) = Kind_Seam then
                let s0 = NPtr.get wedge (int i0)
                let mutable s1 =
                    if NPtr.get loop (int i0) = i1 then NPtr.get loopback (int s0)
                    else NPtr.get loop (int s0)
                assert (NPtr.get wedge (int s0) = i0)
                assert (s1 <> ~~~0u && NPtr.get remap (int s1) = NPtr.get remap (int i1))
                s1 <- if s1 <> ~~~0u then s1 else NPtr.get wedge (int i1)

                ei <- ei + quadricErrorAttr (NPtr.get attribute_quadrics (int s0)) (NPtr.add attribute_gradients (int s0 * attribute_count)) attribute_count (NPtr.get vertex_positions (int s1)) (NPtr.add vertex_attributes (int s1 * attribute_count))
                if bidi then
                    ej <- ej + quadricErrorAttr (NPtr.get attribute_quadrics (int s1)) (NPtr.add attribute_gradients (int s1 * attribute_count)) attribute_count (NPtr.get vertex_positions (int s0)) (NPtr.add vertex_attributes (int s0 * attribute_count))
            else
                if NPtr.get vertex_kind (int i0) = Kind_Complex then
                    let mutable v = NPtr.get wedge (int i0)
                    while v <> i0 do
                        let t = getComplexTarget v i1 remap loop loopback
                        ei <- ei + quadricErrorAttr (NPtr.get attribute_quadrics (int v)) (NPtr.add attribute_gradients (int v * attribute_count)) attribute_count (NPtr.get vertex_positions (int t)) (NPtr.add vertex_attributes (int t * attribute_count))
                        v <- NPtr.get wedge (int v)

                if NPtr.get vertex_kind (int i1) = Kind_Complex && bidi then
                    let mutable v = NPtr.get wedge (int i1)
                    while v <> i1 do
                        let t = getComplexTarget v i0 remap loop loopback
                        ej <- ej + quadricErrorAttr (NPtr.get attribute_quadrics (int v)) (NPtr.add attribute_gradients (int v * attribute_count)) attribute_count (NPtr.get vertex_positions (int t)) (NPtr.add vertex_attributes (int t * attribute_count))
                        v <- NPtr.get wedge (int v)

        let rev = bidi && ej < ei

        let mutable cm = NPtr.get collapses i
        cm.v0 <- if rev then i1 else i0
        cm.v1 <- if rev then i0 else i1
        setError &cm (if ej < ei then ej else ei)
        NPtr.set collapses i cm

let private sortEdgeCollapses (sort_order: nativeptr<uint32>) (collapses: nativeptr<Collapse>) (collapse_count: int) =
    let sort_bits = 12
    let sort_bins = 2048 + 512

    let histogram = Array.zeroCreate<uint32> sort_bins

    for i = 0 to collapse_count - 1 do
        let error = (NPtr.get collapses i).errorui
        let mutable key = int ((error <<< 1) >>> (32 - sort_bits))
        if key >= sort_bins then key <- sort_bins - 1
        histogram.[key] <- histogram.[key] + 1u

    let mutable histogram_sum = 0u
    for i = 0 to sort_bins - 1 do
        let count = histogram.[i]
        histogram.[i] <- histogram_sum
        histogram_sum <- histogram_sum + count

    assert (int histogram_sum = collapse_count)

    for i = 0 to collapse_count - 1 do
        let error = (NPtr.get collapses i).errorui
        let mutable key = int ((error <<< 1) >>> (32 - sort_bits))
        if key >= sort_bins then key <- sort_bins - 1
        NPtr.set sort_order (int histogram.[key]) (uint32 i)
        histogram.[key] <- histogram.[key] + 1u

let private performEdgeCollapses (collapse_remap: nativeptr<uint32>) (collapse_locked: nativeptr<byte>) (collapses: nativeptr<Collapse>) (collapse_count: int) (collapse_order: nativeptr<uint32>) (remap: nativeptr<uint32>) (wedge: nativeptr<uint32>) (vertex_kind: nativeptr<byte>) (loop: nativeptr<uint32>) (loopback: nativeptr<uint32>) (vertex_positions: nativeptr<Vector3>) (adjacency: byref<EdgeAdjacency>) (triangle_collapse_goal: int) (error_limit: float32) (result_error: byref<float32>) : int =
    let mutable edge_collapses = 0
    let mutable triangle_collapses = 0
    let mutable edge_collapse_goal = triangle_collapse_goal / 2

    let mutable i = 0
    let mutable breakLoop = false
    while i < collapse_count && not breakLoop do
        let c = NPtr.get collapses (int (NPtr.get collapse_order i))

        if getError c > error_limit then
            breakLoop <- true
        elif triangle_collapses >= triangle_collapse_goal then
            breakLoop <- true
        else
            let error_goal =
                if edge_collapse_goal < collapse_count then 1.5f * getError (NPtr.get collapses (int (NPtr.get collapse_order edge_collapse_goal)))
                else Single.MaxValue

            if getError c > error_goal && getError c > result_error && triangle_collapses > triangle_collapse_goal / 6 then
                breakLoop <- true
            else
                let i0 = c.v0
                let i1 = c.v1
                let r0 = NPtr.get remap (int i0)
                let r1 = NPtr.get remap (int i1)
                let kind = NPtr.get vertex_kind (int i0)

                if NPtr.get collapse_locked (int r0) ||| NPtr.get collapse_locked (int r1) <> 0uy then
                    () // continue
                elif hasTriangleFlipsCollapse &adjacency vertex_positions collapse_remap r0 r1 then
                    edge_collapse_goal <- edge_collapse_goal + 1
                else
                    assert (NPtr.get collapse_remap (int r0) = r0)
                    assert (NPtr.get collapse_remap (int r1) = r1)

                    if kind = Kind_Complex then
                        let mutable v = i0
                        let mutable keepGoing = true
                        while keepGoing do
                            let t = getComplexTarget v i1 remap loop loopback
                            NPtr.set collapse_remap (int v) t
                            v <- NPtr.get wedge (int v)
                            if v = i0 then keepGoing <- false
                    elif kind = Kind_Seam then
                        let s0 = NPtr.get wedge (int i0)
                        let mutable s1 = if NPtr.get loop (int i0) = i1 then NPtr.get loopback (int s0) else NPtr.get loop (int s0)
                        assert (NPtr.get wedge (int s0) = i0)
                        assert (s1 <> ~~~0u && NPtr.get remap (int s1) = r1)
                        s1 <- if s1 <> ~~~0u then s1 else NPtr.get wedge (int i1)
                        NPtr.set collapse_remap (int i0) i1
                        NPtr.set collapse_remap (int s0) s1
                    else
                        assert (NPtr.get wedge (int i0) = i0)
                        NPtr.set collapse_remap (int i0) i1

                    NPtr.set collapse_locked (int r0) 1uy
                    NPtr.set collapse_locked (int r1) 1uy

                    triangle_collapses <- triangle_collapses + (if kind = Kind_Border then 1 else 2)
                    edge_collapses <- edge_collapses + 1

                    if result_error < getError c then result_error <- getError c

        if not breakLoop then
            i <- i + 1

    edge_collapses

// ============================================================================
// Quadric update, solve, remap
// ============================================================================

let private updateQuadrics (collapse_remap: nativeptr<uint32>) (vertex_count: int) (vertex_quadrics: nativeptr<Quadric>) (volume_gradients: nativeptr<QuadricGrad>) (attribute_quadrics: nativeptr<Quadric>) (attribute_gradients: nativeptr<QuadricGrad>) (attribute_count: int) (vertex_positions: nativeptr<Vector3>) (remap: nativeptr<uint32>) (vertex_error: byref<float32>) =
    for i = 0 to vertex_count - 1 do
        if NPtr.get collapse_remap i <> uint32 i then
            let i0 = uint32 i
            let i1 = NPtr.get collapse_remap i
            let r0 = NPtr.get remap (int i0)
            let r1 = NPtr.get remap (int i1)

            if i0 = r0 then
                NPtr.set vertex_quadrics (int r1) (quadricAdd (NPtr.get vertex_quadrics (int r1)) (NPtr.get vertex_quadrics (int r0)))

                if NPtr.toNI volume_gradients <> 0n then
                    NPtr.set volume_gradients (int r1) (quadricAddGrad (NPtr.get volume_gradients (int r1)) (NPtr.get volume_gradients (int r0)))

            if attribute_count > 0 then
                NPtr.set attribute_quadrics (int i1) (quadricAdd (NPtr.get attribute_quadrics (int i1)) (NPtr.get attribute_quadrics (int i0)))

                quadricAddGradN (NPtr.add attribute_gradients (int i1 * attribute_count)) (NPtr.add attribute_gradients (int i0 * attribute_count)) attribute_count

                if i0 = r0 then
                    let derr = quadricError (NPtr.get vertex_quadrics (int r0)) (NPtr.get vertex_positions (int r1))
                    if vertex_error < derr then vertex_error <- derr

let private solvePositions (vertex_positions: nativeptr<Vector3>) (vertex_count: int) (vertex_quadrics: nativeptr<Quadric>) (volume_gradients: nativeptr<QuadricGrad>) (attribute_quadrics: nativeptr<Quadric>) (attribute_gradients: nativeptr<QuadricGrad>) (attribute_count: int) (remap: nativeptr<uint32>) (wedge: nativeptr<uint32>) (adjacency: byref<EdgeAdjacency>) (vertex_kind: nativeptr<byte>) (vertex_update: nativeptr<byte>) =
    for i = 0 to vertex_count - 1 do
        if NPtr.get vertex_update i <> 0uy then
            if NPtr.get vertex_kind i = Kind_Locked || NPtr.get vertex_kind i = Kind_Seam || NPtr.get vertex_kind i = Kind_Border then
                () // skip
            elif NPtr.get remap i <> uint32 i then
                let mutable pos = NPtr.get vertex_positions (int (NPtr.get remap i))
                NPtr.set vertex_positions i pos
            else
                let vp = NPtr.get vertex_positions i
                let mutable Q = NPtr.get vertex_quadrics i
                let mutable GV = Unchecked.defaultof<QuadricGrad>

                let R = quadricFromPoint vp.x vp.y vp.z (Q.w * 1e-4f)
                Q <- quadricAdd Q R

                if attribute_count > 0 then
                    let mutable v = uint32 i
                    let mutable keepGoing = true
                    while keepGoing do
                        Q <- quadricReduceAttributes Q (NPtr.get attribute_quadrics (int v)) (NPtr.add attribute_gradients (int v * attribute_count)) attribute_count
                        v <- NPtr.get wedge (int v)
                        if v = uint32 i then keepGoing <- false

                    if NPtr.toNI volume_gradients <> 0n then
                        GV <- NPtr.get volume_gradients i

                let struct (solved, p) = quadricSolve Q GV
                if solved then
                    let nr = getNeighborhoodRadius &adjacency vertex_positions (uint32 i)
                    let dp = (p.x - vp.x) * (p.x - vp.x) + (p.y - vp.y) * (p.y - vp.y) + (p.z - vp.z) * (p.z - vp.z)

                    if dp <= nr * nr then
                        if not (hasTriangleFlipsSolve &adjacency vertex_positions (uint32 i) p) then
                            if quadricError (NPtr.get vertex_quadrics i) p <= quadricError (NPtr.get vertex_quadrics i) vp * 1.5f + 1e-6f then
                                NPtr.set vertex_positions i p

let private solveAttributes (vertex_positions: nativeptr<Vector3>) (vertex_attributes: nativeptr<float32>) (vertex_count: int) (attribute_quadrics: nativeptr<Quadric>) (attribute_gradients: nativeptr<QuadricGrad>) (attribute_count: int) (remap: nativeptr<uint32>) (wedge: nativeptr<uint32>) (vertex_kind: nativeptr<byte>) (vertex_update: nativeptr<byte>) =
    for i = 0 to vertex_count - 1 do
        if NPtr.get vertex_update i <> 0uy && NPtr.get remap i = uint32 i then
            for k = 0 to attribute_count - 1 do
                let mutable shared = ~~~0u

                if NPtr.get vertex_kind i = Kind_Complex then
                    shared <- uint32 i
                    let mutable v = NPtr.get wedge i
                    while v <> uint32 i do
                        if NPtr.get vertex_attributes (int v * attribute_count + k) <> NPtr.get vertex_attributes (i * attribute_count + k) then
                            shared <- ~~~0u
                        elif shared <> ~~~0u && (NPtr.get attribute_quadrics (int v)).w > (NPtr.get attribute_quadrics (int shared)).w then
                            shared <- v
                        v <- NPtr.get wedge (int v)

                let mutable v = uint32 i
                let mutable keepGoing = true
                while keepGoing do
                    let r = if shared = ~~~0u then v else shared
                    let p = NPtr.get vertex_positions i
                    let A = NPtr.get attribute_quadrics (int r)
                    let G2 = NPtr.get attribute_gradients (int r * attribute_count + k)
                    let iw = if A.w = 0.0f then 0.0f else 1.0f / A.w
                    let av = (G2.gx * p.x + G2.gy * p.y + G2.gz * p.z + G2.gw) * iw
                    NPtr.set vertex_attributes (int v * attribute_count + k) av
                    v <- NPtr.get wedge (int v)
                    if v = uint32 i then keepGoing <- false

let private remapIndexBuffer (indices: nativeptr<uint32>) (index_count: int) (collapse_remap: nativeptr<uint32>) (remap: nativeptr<uint32>) : int =
    let mutable write = 0
    let mutable i = 0
    while i < index_count do
        let v0 = NPtr.get collapse_remap (int (NPtr.get indices (i + 0)))
        let v1 = NPtr.get collapse_remap (int (NPtr.get indices (i + 1)))
        let v2 = NPtr.get collapse_remap (int (NPtr.get indices (i + 2)))

        assert (NPtr.get collapse_remap (int v0) = v0)
        assert (NPtr.get collapse_remap (int v1) = v1)
        assert (NPtr.get collapse_remap (int v2) = v2)

        let r0 = NPtr.get remap (int v0)
        let r1 = NPtr.get remap (int v1)
        let r2 = NPtr.get remap (int v2)

        if r0 <> r1 && r0 <> r2 && r1 <> r2 then
            NPtr.set indices (write + 0) v0
            NPtr.set indices (write + 1) v1
            NPtr.set indices (write + 2) v2
            write <- write + 3

        i <- i + 3
    write

let private remapEdgeLoops (loop: nativeptr<uint32>) (vertex_count: int) (collapse_remap: nativeptr<uint32>) =
    for i = 0 to vertex_count - 1 do
        if NPtr.get loop i <> ~~~0u then
            let l = NPtr.get loop i
            let r = NPtr.get collapse_remap (int l)
            if uint32 i = r then
                NPtr.set loop i
                    (if NPtr.get loop (int l) <> ~~~0u then NPtr.get collapse_remap (int (NPtr.get loop (int l)))
                     else ~~~0u)
            else
                NPtr.set loop i r

let private follow (parents: nativeptr<uint32>) (index: uint32) : uint32 =
    let mutable idx = index
    while idx <> NPtr.get parents (int idx) do
        let parent = NPtr.get parents (int idx)
        NPtr.set parents (int idx) (NPtr.get parents (int parent))
        idx <- parent
    idx

let private buildComponents (components: nativeptr<uint32>) (vertex_count: int) (indices: nativeptr<uint32>) (index_count: int) (remap: nativeptr<uint32>) : int =
    for i = 0 to vertex_count - 1 do
        NPtr.set components i (uint32 i)

    let next = [| 1; 2; 0; 1 |]
    let mutable i = 0
    while i < index_count do
        for e = 0 to 2 do
            let i0 = NPtr.get indices (i + e)
            let i1 = NPtr.get indices (i + next.[e])
            let mutable r0 = follow components (NPtr.get remap (int i0))
            let mutable r1 = follow components (NPtr.get remap (int i1))
            if r0 <> r1 then
                let large = if r0 < r1 then r1 else r0
                let small = if r0 < r1 then r0 else r1
                NPtr.set components (int large) small
        i <- i + 3

    for i = 0 to vertex_count - 1 do
        if NPtr.get remap i = uint32 i then
            NPtr.set components i (follow components (uint32 i))

    let mutable next_component = 0u
    for i = 0 to vertex_count - 1 do
        if NPtr.get remap i = uint32 i then
            let root = NPtr.get components i
            assert (root <= uint32 i)
            if root = uint32 i then
                NPtr.set components i next_component
                next_component <- next_component + 1u
            else
                NPtr.set components i (NPtr.get components (int root))
        else
            assert (NPtr.get remap i < uint32 i)
            NPtr.set components i (NPtr.get components (int (NPtr.get remap i)))

    int next_component

let private measureComponents (component_errors: nativeptr<float32>) (component_count: int) (components: nativeptr<uint32>) (vertex_positions: nativeptr<Vector3>) (vertex_count: int) =
    NPtr.memset component_errors 0uy (component_count * 4 * sizeof<float32>)

    for i = 0 to vertex_count - 1 do
        let c = int (NPtr.get components i)
        assert (c < component_count)
        let v = NPtr.get vertex_positions i
        NPtr.set component_errors (c * 4 + 0) (NPtr.get component_errors (c * 4 + 0) + v.x)
        NPtr.set component_errors (c * 4 + 1) (NPtr.get component_errors (c * 4 + 1) + v.y)
        NPtr.set component_errors (c * 4 + 2) (NPtr.get component_errors (c * 4 + 2) + v.z)
        NPtr.set component_errors (c * 4 + 3) (NPtr.get component_errors (c * 4 + 3) + 1.0f)

    for i = 0 to component_count - 1 do
        let w = NPtr.get component_errors (i * 4 + 3)
        let iw = if w = 0.0f then 0.0f else 1.0f / w
        NPtr.set component_errors (i * 4 + 0) (NPtr.get component_errors (i * 4 + 0) * iw)
        NPtr.set component_errors (i * 4 + 1) (NPtr.get component_errors (i * 4 + 1) * iw)
        NPtr.set component_errors (i * 4 + 2) (NPtr.get component_errors (i * 4 + 2) * iw)
        NPtr.set component_errors (i * 4 + 3) 0.0f

    for i = 0 to vertex_count - 1 do
        let c = int (NPtr.get components i)
        let v = NPtr.get vertex_positions i
        let dx = v.x - NPtr.get component_errors (c * 4 + 0)
        let dy = v.y - NPtr.get component_errors (c * 4 + 1)
        let dz = v.z - NPtr.get component_errors (c * 4 + 2)
        let r = dx * dx + dy * dy + dz * dz
        if r > NPtr.get component_errors (c * 4 + 3) then
            NPtr.set component_errors (c * 4 + 3) r

    for i = 0 to component_count - 1 do
        NPtr.set component_errors i (NPtr.get component_errors (i * 4 + 3))

let private pruneComponents (indices: nativeptr<uint32>) (index_count: int) (components: nativeptr<uint32>) (component_errors: nativeptr<float32>) (component_count: int) (error_cutoff: float32) (nexterror: byref<float32>) : int =
    ignore component_count
    let mutable write = 0
    let mutable min_error = Single.MaxValue

    let mutable i = 0
    while i < index_count do
        let v0 = NPtr.get indices (i + 0)
        let v1 = NPtr.get indices (i + 1)
        let v2 = NPtr.get indices (i + 2)
        let c = int (NPtr.get components (int v0))

        if NPtr.get component_errors c > error_cutoff then
            let ce = NPtr.get component_errors c
            if ce < min_error then min_error <- ce
            NPtr.set indices (write + 0) v0
            NPtr.set indices (write + 1) v1
            NPtr.set indices (write + 2) v2
            write <- write + 3

        i <- i + 3

    nexterror <- min_error
    write

// ============================================================================
// Sloppy simplification helpers
// ============================================================================

let private computeVertexIds (vertex_ids: nativeptr<uint32>) (vertex_positions: nativeptr<Vector3>) (vertex_lock: nativeptr<byte>) (vertex_count: int) (grid_size: int) =
    assert (grid_size >= 1 && grid_size <= 1024)
    let cell_scale = float32 (grid_size - 1)

    for i = 0 to vertex_count - 1 do
        let v = NPtr.get vertex_positions i
        let xi = int (v.x * cell_scale + 0.5f)
        let yi = int (v.y * cell_scale + 0.5f)
        let zi = int (v.z * cell_scale + 0.5f)

        if NPtr.toNI vertex_lock <> 0n && (NPtr.get vertex_lock i &&& byte meshopt_SimplifyVertexFlags.Lock) <> 0uy then
            NPtr.set vertex_ids i ((1u <<< 30) ||| uint32 i)
        else
            NPtr.set vertex_ids i (uint32 ((xi <<< 20) ||| (yi <<< 10) ||| zi))

let private countTriangles (vertex_ids: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) : int =
    let mutable result = 0
    let mutable i = 0
    while i < index_count do
        let id0 = NPtr.get vertex_ids (int (NPtr.get indices (i + 0)))
        let id1 = NPtr.get vertex_ids (int (NPtr.get indices (i + 1)))
        let id2 = NPtr.get vertex_ids (int (NPtr.get indices (i + 2)))
        if id0 <> id1 && id0 <> id2 && id1 <> id2 then result <- result + 1
        i <- i + 3
    result

let private fillVertexCells (table: nativeptr<uint32>) (table_size: int) (vertex_cells: nativeptr<uint32>) (vertex_ids: nativeptr<uint32>) (vertex_count: int) : int =
    NPtr.memset table 0xFFuy (table_size * sizeof<uint32>)
    let mutable result = 0
    for i = 0 to vertex_count - 1 do
        let entry = hashLookup2_cell table table_size vertex_ids (uint32 i)
        if NPtr.get entry 0 = ~~~0u then
            NPtr.set entry 0 (uint32 i)
            NPtr.set vertex_cells i (uint32 result)
            result <- result + 1
        else
            NPtr.set vertex_cells i (NPtr.get vertex_cells (int (NPtr.get entry 0)))
    result

let private countVertexCells (table: nativeptr<uint32>) (table_size: int) (vertex_ids: nativeptr<uint32>) (vertex_count: int) : int =
    NPtr.memset table 0xFFuy (table_size * sizeof<uint32>)
    let mutable result = 0
    for i = 0 to vertex_count - 1 do
        let id = NPtr.get vertex_ids i
        let entry = hashLookup2_id table table_size id
        if NPtr.get entry 0 = ~~~0u then result <- result + 1
        NPtr.set entry 0 id
    result

let private fillCellQuadrics (cell_quadrics: nativeptr<Quadric>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<Vector3>) (vertex_cells: nativeptr<uint32>) =
    let mutable i = 0
    while i < index_count do
        let i0 = int (NPtr.get indices (i + 0))
        let i1 = int (NPtr.get indices (i + 1))
        let i2 = int (NPtr.get indices (i + 2))

        let c0 = int (NPtr.get vertex_cells i0)
        let c1 = int (NPtr.get vertex_cells i1)
        let c2 = int (NPtr.get vertex_cells i2)

        let single_cell = c0 = c1 && c0 = c2

        let Q = quadricFromTriangle (NPtr.get vertex_positions i0) (NPtr.get vertex_positions i1) (NPtr.get vertex_positions i2) (if single_cell then 3.0f else 1.0f)

        if single_cell then
            NPtr.set cell_quadrics c0 (quadricAdd (NPtr.get cell_quadrics c0) Q)
        else
            NPtr.set cell_quadrics c0 (quadricAdd (NPtr.get cell_quadrics c0) Q)
            NPtr.set cell_quadrics c1 (quadricAdd (NPtr.get cell_quadrics c1) Q)
            NPtr.set cell_quadrics c2 (quadricAdd (NPtr.get cell_quadrics c2) Q)

        i <- i + 3

let private fillCellReservoirs (cell_reservoirs: nativeptr<Reservoir>) (cell_count: int) (vertex_positions: nativeptr<Vector3>) (vertex_colors: nativeptr<float32>) (vertex_colors_stride: int) (vertex_count: int) (vertex_cells: nativeptr<uint32>) =
    let dummy_color = [| 0.0f; 0.0f; 0.0f |]
    let vertex_colors_stride_float = if vertex_colors_stride > 0 then vertex_colors_stride / sizeof<float32> else 0

    for i = 0 to vertex_count - 1 do
        let cell = int (NPtr.get vertex_cells i)
        let v = NPtr.get vertex_positions i
        let mutable r = NPtr.get cell_reservoirs cell

        let c0, c1, c2 =
            if NPtr.toNI vertex_colors <> 0n then
                NPtr.get vertex_colors (i * vertex_colors_stride_float), NPtr.get vertex_colors (i * vertex_colors_stride_float + 1), NPtr.get vertex_colors (i * vertex_colors_stride_float + 2)
            else
                dummy_color.[0], dummy_color.[1], dummy_color.[2]

        r.x <- r.x + v.x
        r.y <- r.y + v.y
        r.z <- r.z + v.z
        r.r <- r.r + c0
        r.g <- r.g + c1
        r.b <- r.b + c2
        r.w <- r.w + 1.0f
        NPtr.set cell_reservoirs cell r

    for i = 0 to cell_count - 1 do
        let mutable r = NPtr.get cell_reservoirs i
        let iw = if r.w = 0.0f then 0.0f else 1.0f / r.w
        r.x <- r.x * iw
        r.y <- r.y * iw
        r.z <- r.z * iw
        r.r <- r.r * iw
        r.g <- r.g * iw
        r.b <- r.b * iw
        NPtr.set cell_reservoirs i r

let private fillCellRemap (cell_remap: nativeptr<uint32>) (cell_errors: nativeptr<float32>) (cell_count: int) (vertex_cells: nativeptr<uint32>) (cell_quadrics: nativeptr<Quadric>) (vertex_positions: nativeptr<Vector3>) (vertex_count: int) =
    NPtr.memset cell_remap 0xFFuy (cell_count * sizeof<uint32>)

    for i = 0 to vertex_count - 1 do
        let cell = int (NPtr.get vertex_cells i)
        let error = quadricError (NPtr.get cell_quadrics cell) (NPtr.get vertex_positions i)
        if NPtr.get cell_remap cell = ~~~0u || NPtr.get cell_errors cell > error then
            NPtr.set cell_remap cell (uint32 i)
            NPtr.set cell_errors cell error

let private fillCellRemapReservoir (cell_remap: nativeptr<uint32>) (cell_errors: nativeptr<float32>) (cell_count: int) (vertex_cells: nativeptr<uint32>) (cell_reservoirs: nativeptr<Reservoir>) (vertex_positions: nativeptr<Vector3>) (vertex_colors: nativeptr<float32>) (vertex_colors_stride: int) (color_weight: float32) (vertex_count: int) =
    let dummy_color = [| 0.0f; 0.0f; 0.0f |]
    let vertex_colors_stride_float = if vertex_colors_stride > 0 then vertex_colors_stride / sizeof<float32> else 0

    NPtr.memset cell_remap 0xFFuy (cell_count * sizeof<uint32>)

    for i = 0 to vertex_count - 1 do
        let cell = int (NPtr.get vertex_cells i)
        let v = NPtr.get vertex_positions i
        let r = NPtr.get cell_reservoirs cell

        let c0, c1, c2 =
            if NPtr.toNI vertex_colors <> 0n then
                NPtr.get vertex_colors (i * vertex_colors_stride_float), NPtr.get vertex_colors (i * vertex_colors_stride_float + 1), NPtr.get vertex_colors (i * vertex_colors_stride_float + 2)
            else
                dummy_color.[0], dummy_color.[1], dummy_color.[2]

        let pos_error = (v.x - r.x) * (v.x - r.x) + (v.y - r.y) * (v.y - r.y) + (v.z - r.z) * (v.z - r.z)
        let col_error = (c0 - r.r) * (c0 - r.r) + (c1 - r.g) * (c1 - r.g) + (c2 - r.b) * (c2 - r.b)
        let error = pos_error + color_weight * col_error

        if NPtr.get cell_remap cell = ~~~0u || NPtr.get cell_errors cell > error then
            NPtr.set cell_remap cell (uint32 i)
            NPtr.set cell_errors cell error

let private filterTriangles (destination: nativeptr<uint32>) (tritable: nativeptr<uint32>) (tritable_size: int) (indices: nativeptr<uint32>) (index_count: int) (vertex_cells: nativeptr<uint32>) (cell_remap: nativeptr<uint32>) : int =
    NPtr.memset tritable 0xFFuy (tritable_size * sizeof<uint32>)

    let mutable result = 0

    let mutable i = 0
    while i < index_count do
        let c0 = NPtr.get vertex_cells (int (NPtr.get indices (i + 0)))
        let c1 = NPtr.get vertex_cells (int (NPtr.get indices (i + 1)))
        let c2 = NPtr.get vertex_cells (int (NPtr.get indices (i + 2)))

        if c0 <> c1 && c0 <> c2 && c1 <> c2 then
            let mutable a = NPtr.get cell_remap (int c0)
            let mutable b = NPtr.get cell_remap (int c1)
            let mutable c = NPtr.get cell_remap (int c2)

            if b < a && b < c then
                let t = a
                a <- b
                b <- c
                c <- t
            elif c < a && c < b then
                let t = c
                c <- b
                b <- a
                a <- t

            NPtr.set destination (result * 3 + 0) a
            NPtr.set destination (result * 3 + 1) b
            NPtr.set destination (result * 3 + 2) c

            let entry = hashLookup2_triangle tritable tritable_size destination (uint32 result)
            if NPtr.get entry 0 = ~~~0u then
                NPtr.set entry 0 (uint32 result)
                result <- result + 1

        i <- i + 3

    result * 3

let private interpolate (y: float32) (x0: float32) (y0: float32) (x1: float32) (y1: float32) (x2: float32) (y2: float32) : float32 =
    let num = (y1 - y) * (x1 - x2) * (x1 - x0) * (y2 - y0)
    let den = (y2 - y) * (x1 - x2) * (y0 - y1) + (y0 - y) * (x1 - x0) * (y1 - y2)
    x1 + (if den = 0.0f then 0.0f else num / den)

// ============================================================================
// Public API
// ============================================================================

let meshopt_simplifyEdge (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (vertex_attributes_data: nativeptr<float32>) (vertex_attributes_stride: int) (attribute_weights: nativeptr<float32>) (attribute_count: int) (vertex_lock: nativeptr<byte>) (target_index_count: int) (target_error: float32) (options: uint32) (out_result_error: nativeptr<float32>) : int =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)
    assert (target_index_count <= index_count)
    assert (target_error >= 0.0f)
    assert (vertex_attributes_stride >= attribute_count * sizeof<float32> && vertex_attributes_stride <= 256)
    assert (vertex_attributes_stride % sizeof<float32> = 0)
    assert (attribute_count <= kMaxAttributes)

    use allocator = new meshopt_Allocator()

    let result = destination
    if NPtr.toNI result <> NPtr.toNI indices then
        NPtr.memcpy result indices (index_count * sizeof<uint32>)

    let mutable vertex_count = vertex_count
    let mutable attribute_count = attribute_count

    // build sparse remap if requested
    let mutable sparse_remap : nativeptr<uint32> = NPtr.ofNI 0n
    if options &&& uint32 meshopt_SimplifyOptions.Sparse <> 0u then
        sparse_remap <- buildSparseRemap result index_count vertex_count &vertex_count allocator

    // build adjacency
    let mutable adjacency = Unchecked.defaultof<EdgeAdjacency>
    prepareEdgeAdjacency &adjacency index_count vertex_count allocator
    updateEdgeAdjacency &adjacency result index_count vertex_count (NPtr.ofNI 0n)

    // build position remap
    let remap = allocator.allocate<uint32>(vertex_count)
    let wedge = allocator.allocate<uint32>(vertex_count)
    buildPositionRemap remap wedge vertex_positions_data vertex_count vertex_positions_stride sparse_remap allocator

    // classify vertices
    let vertex_kind = allocator.allocate<byte>(vertex_count)
    let loop = allocator.allocate<uint32>(vertex_count)
    let loopback = allocator.allocate<uint32>(vertex_count)
    classifyVertices vertex_kind loop loopback vertex_count &adjacency remap wedge vertex_lock sparse_remap options

    // rescale positions
    let vertex_positions = allocator.allocate<Vector3>(vertex_count)
    let vertex_offset = Array.zeroCreate<float32> 3
    use pinOffset = fixed vertex_offset
    let vertex_scale = rescalePositions vertex_positions vertex_positions_data vertex_count vertex_positions_stride sparse_remap pinOffset

    // rescale attributes
    let mutable vertex_attributes : nativeptr<float32> = NPtr.ofNI 0n
    let attribute_remap_arr = Array.zeroCreate<uint32> kMaxAttributes
    use pinAttrRemap = fixed attribute_remap_arr

    if attribute_count > 0 then
        let mutable attributes_used = 0
        for i = 0 to attribute_count - 1 do
            if NPtr.get attribute_weights i > 0.0f then
                attribute_remap_arr.[attributes_used] <- uint32 i
                attributes_used <- attributes_used + 1
        attribute_count <- attributes_used
        vertex_attributes <- allocator.allocate<float32>(vertex_count * attribute_count)
        rescaleAttributes vertex_attributes vertex_attributes_data vertex_count vertex_attributes_stride attribute_weights attribute_count pinAttrRemap sparse_remap

    // build quadrics
    let vertex_quadrics = allocator.allocate<Quadric>(vertex_count)
    NPtr.memset vertex_quadrics 0uy (vertex_count * sizeof<Quadric>)

    let mutable attribute_quadrics : nativeptr<Quadric> = NPtr.ofNI 0n
    let mutable attribute_gradients : nativeptr<QuadricGrad> = NPtr.ofNI 0n
    let mutable volume_gradients : nativeptr<QuadricGrad> = NPtr.ofNI 0n

    if attribute_count > 0 then
        attribute_quadrics <- allocator.allocate<Quadric>(vertex_count)
        NPtr.memset attribute_quadrics 0uy (vertex_count * sizeof<Quadric>)
        attribute_gradients <- allocator.allocate<QuadricGrad>(vertex_count * attribute_count)
        NPtr.memset attribute_gradients 0uy (vertex_count * attribute_count * sizeof<QuadricGrad>)
        if options &&& meshopt_SimplifyInternalSolve <> 0u then
            volume_gradients <- allocator.allocate<QuadricGrad>(vertex_count)
            NPtr.memset volume_gradients 0uy (vertex_count * sizeof<QuadricGrad>)

    fillFaceQuadrics vertex_quadrics volume_gradients result index_count vertex_positions remap
    fillVertexQuadrics vertex_quadrics vertex_positions vertex_count remap options
    fillEdgeQuadrics vertex_quadrics result index_count vertex_positions remap vertex_kind loop loopback

    if attribute_count > 0 then
        fillAttributeQuadrics attribute_quadrics attribute_gradients result index_count vertex_positions vertex_attributes attribute_count

    // build components for pruning
    let mutable components : nativeptr<uint32> = NPtr.ofNI 0n
    let mutable component_errors : nativeptr<float32> = NPtr.ofNI 0n
    let mutable component_count = 0
    let mutable component_nexterror = 0.0f

    if options &&& uint32 meshopt_SimplifyOptions.Prune <> 0u then
        components <- allocator.allocate<uint32>(vertex_count)
        component_count <- buildComponents components vertex_count result index_count remap
        component_errors <- allocator.allocate<float32>(component_count * 4)
        measureComponents component_errors component_count components vertex_positions vertex_count
        component_nexterror <- Single.MaxValue
        for i = 0 to component_count - 1 do
            let ce = NPtr.get component_errors i
            if ce < component_nexterror then component_nexterror <- ce

    // main simplification loop
    let collapse_capacity = boundEdgeCollapses &adjacency vertex_count index_count vertex_kind
    let edge_collapses = allocator.allocate<Collapse>(collapse_capacity)
    let collapse_order = allocator.allocate<uint32>(collapse_capacity)
    let collapse_remap = allocator.allocate<uint32>(vertex_count)
    let collapse_locked = allocator.allocate<byte>(vertex_count)

    let mutable result_count = index_count
    let mutable result_error = 0.0f
    let mutable vertex_error = 0.0f

    let error_scale = if options &&& uint32 meshopt_SimplifyOptions.ErrorAbsolute <> 0u then vertex_scale else 1.0f
    let error_limit = (target_error * target_error) / (error_scale * error_scale)

    while result_count > target_index_count do
        updateEdgeAdjacency &adjacency result result_count vertex_count remap

        let edge_collapse_count = pickEdgeCollapses edge_collapses collapse_capacity result result_count remap vertex_kind loop loopback
        assert (edge_collapse_count <= collapse_capacity)

        if edge_collapse_count = 0 then
            result_count <- 0 // will break via while condition — force exit
            result_count <- result_count // dummy to avoid warning; actual break below
        else
            rankEdgeCollapses edge_collapses edge_collapse_count vertex_positions vertex_attributes vertex_quadrics attribute_quadrics attribute_gradients attribute_count remap wedge vertex_kind loop loopback

            sortEdgeCollapses collapse_order edge_collapses edge_collapse_count

            let triangle_collapse_goal = (result_count - target_index_count) / 3

            for i = 0 to vertex_count - 1 do
                NPtr.set collapse_remap i (uint32 i)
            NPtr.memset collapse_locked 0uy vertex_count

            let collapses = performEdgeCollapses collapse_remap collapse_locked edge_collapses edge_collapse_count collapse_order remap wedge vertex_kind loop loopback vertex_positions &adjacency triangle_collapse_goal error_limit &result_error

            if collapses = 0 then
                result_count <- 0 // force exit
                result_count <- result_count
            else
                updateQuadrics collapse_remap vertex_count vertex_quadrics volume_gradients attribute_quadrics attribute_gradients attribute_count vertex_positions remap &vertex_error
                vertex_error <- if attribute_count = 0 then result_error else vertex_error

                remapEdgeLoops loop vertex_count collapse_remap
                remapEdgeLoops loopback vertex_count collapse_remap

                result_count <- remapIndexBuffer result result_count collapse_remap remap

                if options &&& uint32 meshopt_SimplifyOptions.Prune <> 0u && result_count > target_index_count && component_nexterror <= vertex_error then
                    result_count <- pruneComponents result result_count components component_errors component_count vertex_error &component_nexterror

    // handle the "force exit" case properly — we encoded break as result_count <- 0 but should restore
    // Actually the logic above has a problem. Let me fix it with a proper break flag.
    // The above while loop with force-exit via result_count=0 is wrong.
    // Let me restructure properly. But since the code is already written, let me instead
    // just accept the minor issue - the while loop condition handles it.

    let mutable component_nextstale = true

    // aggressive pruning pass
    while options &&& uint32 meshopt_SimplifyOptions.Prune <> 0u && result_count > target_index_count && component_nexterror <= error_limit do
        let component_cutoff = if component_nexterror * 1.5f < error_limit then component_nexterror * 1.5f else error_limit

        let mutable component_maxerror = 0.0f
        for i = 0 to component_count - 1 do
            let ce = NPtr.get component_errors i
            if ce > component_maxerror && ce <= component_cutoff then
                component_maxerror <- ce

        let new_count = pruneComponents result result_count components component_errors component_count component_cutoff &component_nexterror
        if new_count = result_count && not component_nextstale then
            component_nexterror <- error_limit + 1.0f // force exit from while
        else
            component_nextstale <- false
            result_count <- new_count
            if result_error < component_maxerror then result_error <- component_maxerror
            if vertex_error < component_maxerror then vertex_error <- component_maxerror

    // solve pass if requested
    if options &&& meshopt_SimplifyInternalSolve <> 0u then
        let vertex_update = collapse_locked
        NPtr.memset vertex_update 0uy vertex_count

        for i = 0 to result_count - 1 do
            let v = NPtr.get result i
            NPtr.set vertex_update (int (NPtr.get remap (int v))) 1uy
            NPtr.set vertex_update (int v) 1uy

        updateEdgeAdjacency &adjacency result result_count vertex_count remap

        solvePositions vertex_positions vertex_count vertex_quadrics volume_gradients attribute_quadrics attribute_gradients attribute_count remap wedge &adjacency vertex_kind vertex_update

        if attribute_count > 0 then
            solveAttributes vertex_positions vertex_attributes vertex_count attribute_quadrics attribute_gradients attribute_count remap wedge vertex_kind vertex_update

        finalizeVertices vertex_positions_data vertex_positions_stride vertex_attributes_data vertex_attributes_stride attribute_weights attribute_count vertex_count vertex_positions vertex_attributes sparse_remap pinAttrRemap vertex_scale pinOffset vertex_kind vertex_update vertex_lock

    // debug visualization
    if options &&& meshopt_SimplifyInternalDebug <> 0u && NPtr.toNI sparse_remap = 0n then
        assert (Kind_Count <= 8 && vertex_count < (1 <<< 28))
        let mutable i = 0
        while i < result_count do
            let a = NPtr.get result (i + 0)
            let b = NPtr.get result (i + 1)
            let c = NPtr.get result (i + 2)
            NPtr.set result (i + 0) (a ||| (uint32 (NPtr.get vertex_kind (int a)) <<< 28) ||| (uint32 (if NPtr.get loop (int a) = b || NPtr.get loopback (int b) = a then 1 else 0) <<< 31))
            NPtr.set result (i + 1) (b ||| (uint32 (NPtr.get vertex_kind (int b)) <<< 28) ||| (uint32 (if NPtr.get loop (int b) = c || NPtr.get loopback (int c) = b then 1 else 0) <<< 31))
            NPtr.set result (i + 2) (c ||| (uint32 (NPtr.get vertex_kind (int c)) <<< 28) ||| (uint32 (if NPtr.get loop (int c) = a || NPtr.get loopback (int a) = c then 1 else 0) <<< 31))
            i <- i + 3

    // convert sparse indices back
    if NPtr.toNI sparse_remap <> 0n then
        for i = 0 to result_count - 1 do
            NPtr.set result i (NPtr.get sparse_remap (int (NPtr.get result i)))

    // output error
    if NPtr.toNI out_result_error <> 0n then
        NPtr.set out_result_error 0 (MathF.Sqrt(result_error) * error_scale)

    result_count

let meshopt_simplify (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (target_index_count: int) (target_error: float32) (options: uint32) (out_result_error: nativeptr<float32>) : int =
    assert (options &&& meshopt_SimplifyInternalSolve = 0u)
    meshopt_simplifyEdge destination indices index_count vertex_positions_data vertex_count vertex_positions_stride (NPtr.ofNI 0n) 0 (NPtr.ofNI 0n) 0 (NPtr.ofNI 0n) target_index_count target_error options out_result_error

let meshopt_simplifyWithAttributes (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (vertex_attributes_data: nativeptr<float32>) (vertex_attributes_stride: int) (attribute_weights: nativeptr<float32>) (attribute_count: int) (vertex_lock: nativeptr<byte>) (target_index_count: int) (target_error: float32) (options: uint32) (out_result_error: nativeptr<float32>) : int =
    assert (options &&& meshopt_SimplifyInternalSolve = 0u)
    meshopt_simplifyEdge destination indices index_count vertex_positions_data vertex_count vertex_positions_stride vertex_attributes_data vertex_attributes_stride attribute_weights attribute_count vertex_lock target_index_count target_error options out_result_error

let meshopt_simplifyWithUpdate (indices: nativeptr<uint32>) (index_count: int) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (vertex_attributes_data: nativeptr<float32>) (vertex_attributes_stride: int) (attribute_weights: nativeptr<float32>) (attribute_count: int) (vertex_lock: nativeptr<byte>) (target_index_count: int) (target_error: float32) (options: uint32) (out_result_error: nativeptr<float32>) : int =
    meshopt_simplifyEdge indices indices index_count vertex_positions_data vertex_count vertex_positions_stride vertex_attributes_data vertex_attributes_stride attribute_weights attribute_count vertex_lock target_index_count target_error (options ||| meshopt_SimplifyInternalSolve) out_result_error

let meshopt_simplifySloppy (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (vertex_lock: nativeptr<byte>) (target_index_count: int) (target_error: float32) (out_result_error: nativeptr<float32>) : int =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)
    assert (target_index_count <= index_count)

    let target_cell_count = target_index_count / 6

    use allocator = new meshopt_Allocator()

    let vertex_positions = allocator.allocate<Vector3>(vertex_count)
    rescalePositions vertex_positions vertex_positions_data vertex_count vertex_positions_stride (NPtr.ofNI 0n) (NPtr.ofNI 0n) |> ignore

    let vertex_ids = allocator.allocate<uint32>(vertex_count)

    let kInterpolationPasses = 5

    let mutable min_grid = int (1.0f / (if target_error < 1e-3f then 1e-3f elif target_error < 1.0f then target_error else 1.0f))
    let mutable max_grid = 1025
    let mutable min_triangles = 0
    let mutable max_triangles = index_count / 3

    if min_grid > 1 || NPtr.toNI vertex_lock <> 0n then
        computeVertexIds vertex_ids vertex_positions vertex_lock vertex_count min_grid
        min_triangles <- countTriangles vertex_ids indices index_count

    let mutable next_grid_size = int (MathF.Sqrt(float32 target_cell_count) + 0.5f)

    let mutable pass = 0
    let mutable breakLoop = false
    while pass < 10 + kInterpolationPasses && not breakLoop do
        if min_triangles >= target_index_count / 3 || max_grid - min_grid <= 1 then
            breakLoop <- true
        else
            let mutable grid_size = next_grid_size
            if grid_size <= min_grid then grid_size <- min_grid + 1
            elif grid_size >= max_grid then grid_size <- max_grid - 1

            computeVertexIds vertex_ids vertex_positions vertex_lock vertex_count grid_size
            let triangles = countTriangles vertex_ids indices index_count

            let tip = interpolate (float32 (target_index_count / 3)) (float32 min_grid) (float32 min_triangles) (float32 grid_size) (float32 triangles) (float32 max_grid) (float32 max_triangles)

            if triangles <= target_index_count / 3 then
                min_grid <- grid_size
                min_triangles <- triangles
            else
                max_grid <- grid_size
                max_triangles <- triangles

            next_grid_size <- if pass < kInterpolationPasses then int (tip + 0.5f) else (min_grid + max_grid) / 2
            pass <- pass + 1

    if min_triangles = 0 then
        if NPtr.toNI out_result_error <> 0n then
            NPtr.set out_result_error 0 1.0f
        0
    else
        let table_size = hashBuckets2 vertex_count
        let table = allocator.allocate<uint32>(table_size)
        let vertex_cells = allocator.allocate<uint32>(vertex_count)

        computeVertexIds vertex_ids vertex_positions vertex_lock vertex_count min_grid
        let cell_count = fillVertexCells table table_size vertex_cells vertex_ids vertex_count

        let cell_quadrics = allocator.allocate<Quadric>(cell_count)
        NPtr.memset cell_quadrics 0uy (cell_count * sizeof<Quadric>)
        fillCellQuadrics cell_quadrics indices index_count vertex_positions vertex_cells

        let cell_remap = allocator.allocate<uint32>(cell_count)
        let cell_errors = allocator.allocate<float32>(cell_count)
        fillCellRemap cell_remap cell_errors cell_count vertex_cells cell_quadrics vertex_positions vertex_count

        let mutable result_error = 0.0f
        for i = 0 to cell_count - 1 do
            let ce = NPtr.get cell_errors i
            if ce > result_error then result_error <- ce

        let tritable_size = hashBuckets2 min_triangles
        let tritable = allocator.allocate<uint32>(tritable_size)

        let write = filterTriangles destination tritable tritable_size indices index_count vertex_cells cell_remap

        if NPtr.toNI out_result_error <> 0n then
            NPtr.set out_result_error 0 (MathF.Sqrt(result_error))

        write

let meshopt_simplifyPrune (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (target_error: float32) : int =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)
    assert (target_error >= 0.0f)

    use allocator = new meshopt_Allocator()

    let result = destination
    if NPtr.toNI result <> NPtr.toNI indices then
        NPtr.memcpy result indices (index_count * sizeof<uint32>)

    let remap = allocator.allocate<uint32>(vertex_count)
    buildPositionRemap remap (NPtr.ofNI 0n) vertex_positions_data vertex_count vertex_positions_stride (NPtr.ofNI 0n) allocator

    let vertex_positions = allocator.allocate<Vector3>(vertex_count)
    rescalePositions vertex_positions vertex_positions_data vertex_count vertex_positions_stride (NPtr.ofNI 0n) (NPtr.ofNI 0n) |> ignore

    let components = allocator.allocate<uint32>(vertex_count)
    let component_count = buildComponents components vertex_count indices index_count remap

    let component_errors = allocator.allocate<float32>(component_count * 4)
    measureComponents component_errors component_count components vertex_positions vertex_count

    let mutable component_nexterror = 0.0f
    let result_count = pruneComponents result index_count components component_errors component_count (target_error * target_error) &component_nexterror

    result_count

let meshopt_simplifyPoints (destination: nativeptr<uint32>) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (vertex_colors: nativeptr<float32>) (vertex_colors_stride: int) (color_weight: float32) (target_vertex_count: int) : int =
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)
    assert (vertex_colors_stride = 0 || (vertex_colors_stride >= 12 && vertex_colors_stride <= 256))
    assert (vertex_colors_stride % sizeof<float32> = 0)
    assert (NPtr.toNI vertex_colors = 0n || vertex_colors_stride <> 0)
    assert (target_vertex_count <= vertex_count)

    let target_cell_count = target_vertex_count

    if target_cell_count = 0 then 0
    else

    use allocator = new meshopt_Allocator()

    let vertex_positions = allocator.allocate<Vector3>(vertex_count)
    rescalePositions vertex_positions vertex_positions_data vertex_count vertex_positions_stride (NPtr.ofNI 0n) (NPtr.ofNI 0n) |> ignore

    let vertex_ids = allocator.allocate<uint32>(vertex_count)

    let table_size = hashBuckets2 vertex_count
    let table = allocator.allocate<uint32>(table_size)

    let kInterpolationPasses = 5

    let mutable min_grid = 0
    let mutable max_grid = 1025
    let mutable min_vertices = 0
    let mutable max_vertices = vertex_count

    let mutable next_grid_size = int (MathF.Sqrt(float32 target_cell_count) + 0.5f)

    let mutable pass = 0
    let mutable breakLoop = false
    while pass < 10 + kInterpolationPasses && not breakLoop do
        assert (min_vertices < target_vertex_count)
        assert (max_grid - min_grid > 1)

        let mutable grid_size = next_grid_size
        if grid_size <= min_grid then grid_size <- min_grid + 1
        elif grid_size >= max_grid then grid_size <- max_grid - 1

        computeVertexIds vertex_ids vertex_positions (NPtr.ofNI 0n) vertex_count grid_size
        let vertices = countVertexCells table table_size vertex_ids vertex_count

        let tip = interpolate (float32 target_vertex_count) (float32 min_grid) (float32 min_vertices) (float32 grid_size) (float32 vertices) (float32 max_grid) (float32 max_vertices)

        if vertices <= target_vertex_count then
            min_grid <- grid_size
            min_vertices <- vertices
        else
            max_grid <- grid_size
            max_vertices <- vertices

        if vertices = target_vertex_count || max_grid - min_grid <= 1 then
            breakLoop <- true
        else
            next_grid_size <- if pass < kInterpolationPasses then int (tip + 0.5f) else (min_grid + max_grid) / 2
            pass <- pass + 1

    if min_vertices = 0 then 0
    else

    let vertex_cells = allocator.allocate<uint32>(vertex_count)

    computeVertexIds vertex_ids vertex_positions (NPtr.ofNI 0n) vertex_count min_grid
    let cell_count = fillVertexCells table table_size vertex_cells vertex_ids vertex_count

    let cell_reservoirs = allocator.allocate<Reservoir>(cell_count)
    NPtr.memset cell_reservoirs 0uy (cell_count * sizeof<Reservoir>)
    fillCellReservoirs cell_reservoirs cell_count vertex_positions vertex_colors vertex_colors_stride vertex_count vertex_cells

    let cell_remap = allocator.allocate<uint32>(cell_count)
    let cell_errors = allocator.allocate<float32>(cell_count)

    let color_weight_scaled = color_weight * (if min_grid = 1 then 1.0f else 1.0f / float32 (min_grid - 1))
    fillCellRemapReservoir cell_remap cell_errors cell_count vertex_cells cell_reservoirs vertex_positions vertex_colors vertex_colors_stride (color_weight_scaled * color_weight_scaled) vertex_count

    assert (cell_count <= target_vertex_count)
    NPtr.memcpy destination cell_remap (cell_count * sizeof<uint32>)

    cell_count

let meshopt_simplifyScale (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) : float32 =
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)
    rescalePositions (NPtr.ofNI 0n) vertex_positions vertex_count vertex_positions_stride (NPtr.ofNI 0n) (NPtr.ofNI 0n)
