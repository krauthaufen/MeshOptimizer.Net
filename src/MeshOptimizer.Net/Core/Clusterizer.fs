// This file is part of MeshOptimizer.Net; see meshoptimizer.h for version/license details
module MeshOptimizer.Net.Clusterizer

// This work is based on:
// Graham Wihlidal. Optimizing the Graphics Pipeline with Compute. 2016
// Matthaeus Chajdas. GeometryFX 1.2 - Cluster Culling. 2016
// Jack Ritter. An Efficient Bounding Sphere. 1990
// Thomas Larsson. Fast and Tight Fitting Bounding Spheres. 2008
// Ingo Wald, Vlastimil Havran. On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N). 2006

#nowarn "9"

open System
open System.Runtime.CompilerServices
open FSharp.NativeInterop
open MeshOptimizer.Net
open MeshOptimizer.Net.Allocator

// This must be <= 256 since meshlet indices are stored as bytes
[<Literal>]
let private kMeshletMaxVertices = 256

// A reasonable limit is around 2*max_vertices or less
[<Literal>]
let private kMeshletMaxTriangles = 512

// We keep a limited number of seed triangles and add a few triangles per finished meshlet
[<Literal>]
let private kMeshletMaxSeeds = 256

[<Literal>]
let private kMeshletAddSeeds = 4

// To avoid excessive recursion for malformed inputs, we limit the maximum depth of the tree
[<Literal>]
let private kMeshletMaxTreeDepth = 50

[<Struct>]
type private TriangleAdjacency2 =
    val mutable counts: nativeptr<uint32>
    val mutable offsets: nativeptr<uint32>
    val mutable data: nativeptr<uint32>

[<Struct>]
type private Cone =
    val mutable px: float32
    val mutable py: float32
    val mutable pz: float32
    val mutable nx: float32
    val mutable ny: float32
    val mutable nz: float32

[<Struct>]
type private KDNode =
    val mutable splitOrIndex: uint32  // union: split (as float bits) or index
    val mutable axisAndChildren: uint32  // axis: 2 bits, children: 30 bits

[<Struct>]
type private BVHBoxT =
    val mutable min_0: float32
    val mutable min_1: float32
    val mutable min_2: float32
    val mutable min_3: float32
    val mutable max_0: float32
    val mutable max_1: float32
    val mutable max_2: float32
    val mutable max_3: float32

[<Struct>]
type private BVHBox =
    val mutable min_0: float32
    val mutable min_1: float32
    val mutable min_2: float32
    val mutable max_0: float32
    val mutable max_1: float32
    val mutable max_2: float32

module private KDNodeHelpers =
    let inline getSplit (n: KDNode) : float32 = Unsafe.BitCast<uint32, float32>(n.splitOrIndex)
    let inline setSplit (n: byref<KDNode>) (v: float32) = n.splitOrIndex <- Unsafe.BitCast<float32, uint32>(v)
    let inline getIndex (n: KDNode) : uint32 = n.splitOrIndex
    let inline setIndex (n: byref<KDNode>) (v: uint32) = n.splitOrIndex <- v
    let inline getAxis (n: KDNode) : uint32 = n.axisAndChildren &&& 3u
    let inline getChildren (n: KDNode) : uint32 = n.axisAndChildren >>> 2
    let inline setAxisAndChildren (n: byref<KDNode>) (axis: uint32) (children: uint32) =
        n.axisAndChildren <- (axis &&& 3u) ||| (children <<< 2)

open KDNodeHelpers

let private buildTriangleAdjacency (adjacency: byref<TriangleAdjacency2>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (allocator: meshopt_Allocator) =
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
        NPtr.set adjacency.offsets a (uint32 (oa + 1))

        let ob = int (NPtr.get adjacency.offsets b)
        NPtr.set adjacency.data ob (uint32 i)
        NPtr.set adjacency.offsets b (uint32 (ob + 1))

        let oc = int (NPtr.get adjacency.offsets c)
        NPtr.set adjacency.data oc (uint32 i)
        NPtr.set adjacency.offsets c (uint32 (oc + 1))

    // fix offsets that have been disturbed by the previous pass
    for i = 0 to vertex_count - 1 do
        assert (NPtr.get adjacency.offsets i >= NPtr.get adjacency.counts i)
        NPtr.set adjacency.offsets i (NPtr.get adjacency.offsets i - NPtr.get adjacency.counts i)

let private buildTriangleAdjacencySparse (adjacency: byref<TriangleAdjacency2>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (allocator: meshopt_Allocator) =
    let face_count = index_count / 3

    // sparse mode can build adjacency more quickly by ignoring unused vertices, using a bit to mark visited vertices
    let sparse_seen = 1u <<< 31
    assert (uint32 index_count < sparse_seen)

    // allocate arrays
    adjacency.counts <- allocator.allocate<uint32>(vertex_count)
    adjacency.offsets <- allocator.allocate<uint32>(vertex_count)
    adjacency.data <- allocator.allocate<uint32>(index_count)

    // fill triangle counts
    for i = 0 to index_count - 1 do
        assert (int (NPtr.get indices i) < vertex_count)

    for i = 0 to index_count - 1 do
        NPtr.set adjacency.counts (int (NPtr.get indices i)) 0u

    for i = 0 to index_count - 1 do
        let idx = int (NPtr.get indices i)
        NPtr.set adjacency.counts idx (NPtr.get adjacency.counts idx + 1u)

    // fill offset table; uses sparse_seen bit to tag visited vertices
    let mutable offset = 0u

    for i = 0 to index_count - 1 do
        let v = int (NPtr.get indices i)

        if (NPtr.get adjacency.counts v &&& sparse_seen) = 0u then
            NPtr.set adjacency.offsets v offset
            offset <- offset + NPtr.get adjacency.counts v
            NPtr.set adjacency.counts v (NPtr.get adjacency.counts v ||| sparse_seen)

    assert (offset = uint32 index_count)

    // fill triangle data
    for i = 0 to face_count - 1 do
        let a = int (NPtr.get indices (i * 3 + 0))
        let b = int (NPtr.get indices (i * 3 + 1))
        let c = int (NPtr.get indices (i * 3 + 2))

        let oa = int (NPtr.get adjacency.offsets a)
        NPtr.set adjacency.data oa (uint32 i)
        NPtr.set adjacency.offsets a (uint32 (oa + 1))

        let ob = int (NPtr.get adjacency.offsets b)
        NPtr.set adjacency.data ob (uint32 i)
        NPtr.set adjacency.offsets b (uint32 (ob + 1))

        let oc = int (NPtr.get adjacency.offsets c)
        NPtr.set adjacency.data oc (uint32 i)
        NPtr.set adjacency.offsets c (uint32 (oc + 1))

    // fix offsets that have been disturbed by the previous pass
    // also fix counts (that were marked with sparse_seen by the first pass)
    for i = 0 to index_count - 1 do
        let v = int (NPtr.get indices i)

        if NPtr.get adjacency.counts v &&& sparse_seen <> 0u then
            NPtr.set adjacency.counts v (NPtr.get adjacency.counts v &&& (~~~sparse_seen))

            assert (NPtr.get adjacency.offsets v >= NPtr.get adjacency.counts v)
            NPtr.set adjacency.offsets v (NPtr.get adjacency.offsets v - NPtr.get adjacency.counts v)

let private clearUsed (used: nativeptr<int16>) (vertex_count: int) (indices: nativeptr<uint32>) (index_count: int) =
    // for sparse inputs, it's faster to only clear vertices referenced by the index buffer
    if vertex_count <= index_count then
        NPtr.memset used 0xFFuy (vertex_count * sizeof<int16>)
    else
        for i = 0 to index_count - 1 do
            assert (int (NPtr.get indices i) < vertex_count)
            NPtr.set used (int (NPtr.get indices i)) -1s

let private kAxes =
    [|
        [| 1.0f; 0.0f; 0.0f |]
        [| 0.0f; 1.0f; 0.0f |]
        [| 0.0f; 0.0f; 1.0f |]
        [| 0.57735026f; 0.57735026f; 0.57735026f |]
        [| -0.57735026f; 0.57735026f; 0.57735026f |]
        [| 0.57735026f; -0.57735026f; 0.57735026f |]
        [| 0.57735026f; 0.57735026f; -0.57735026f |]
    |]

let private computeBoundingSphere (result: nativeptr<float32>) (points: nativeptr<float32>) (count: int) (points_stride: int) (radii: nativeptr<float32>) (radii_stride: int) (axis_count: int) =
    assert (count > 0)
    assert (axis_count <= 7)

    let points_stride_float = points_stride / sizeof<float32>
    let radii_stride_float = radii_stride / sizeof<float32>

    // find extremum points along all axes; for each axis we get a pair of points with min/max coordinates
    let pmin = Array.zeroCreate<int> 7
    let pmax = Array.zeroCreate<int> 7
    let tmin = Array.create 7 Single.MaxValue
    let tmax = Array.create 7 (-Single.MaxValue)

    for i = 0 to count - 1 do
        let p = NPtr.add points (i * points_stride_float)
        let r = NPtr.get radii (i * radii_stride_float)

        for axis = 0 to axis_count - 1 do
            let ax = kAxes.[axis]

            let tp = ax.[0] * NPtr.get p 0 + ax.[1] * NPtr.get p 1 + ax.[2] * NPtr.get p 2
            let tpmin = tp - r
            let tpmax = tp + r

            pmin.[axis] <- if tpmin < tmin.[axis] then i else pmin.[axis]
            pmax.[axis] <- if tpmax > tmax.[axis] then i else pmax.[axis]
            tmin.[axis] <- if tpmin < tmin.[axis] then tpmin else tmin.[axis]
            tmax.[axis] <- if tpmax > tmax.[axis] then tpmax else tmax.[axis]

    // find the pair of points with largest distance
    let mutable paxis = 0
    let mutable paxisdr = 0.0f

    for axis = 0 to axis_count - 1 do
        let p1 = NPtr.add points (pmin.[axis] * points_stride_float)
        let p2 = NPtr.add points (pmax.[axis] * points_stride_float)
        let r1 = NPtr.get radii (pmin.[axis] * radii_stride_float)
        let r2 = NPtr.get radii (pmax.[axis] * radii_stride_float)

        let d2 =
            (NPtr.get p2 0 - NPtr.get p1 0) * (NPtr.get p2 0 - NPtr.get p1 0) +
            (NPtr.get p2 1 - NPtr.get p1 1) * (NPtr.get p2 1 - NPtr.get p1 1) +
            (NPtr.get p2 2 - NPtr.get p1 2) * (NPtr.get p2 2 - NPtr.get p1 2)
        let dr = MathF.Sqrt(d2) + r1 + r2

        if dr > paxisdr then
            paxisdr <- dr
            paxis <- axis

    // use the longest segment as the initial sphere diameter
    let p1 = NPtr.add points (pmin.[paxis] * points_stride_float)
    let p2 = NPtr.add points (pmax.[paxis] * points_stride_float)
    let r1 = NPtr.get radii (pmin.[paxis] * radii_stride_float)
    let r2 = NPtr.get radii (pmax.[paxis] * radii_stride_float)

    let paxisd =
        MathF.Sqrt(
            (NPtr.get p2 0 - NPtr.get p1 0) * (NPtr.get p2 0 - NPtr.get p1 0) +
            (NPtr.get p2 1 - NPtr.get p1 1) * (NPtr.get p2 1 - NPtr.get p1 1) +
            (NPtr.get p2 2 - NPtr.get p1 2) * (NPtr.get p2 2 - NPtr.get p1 2))
    let paxisk = if paxisd > 0.0f then (paxisd + r2 - r1) / (2.0f * paxisd) else 0.0f

    let mutable center_0 = NPtr.get p1 0 + (NPtr.get p2 0 - NPtr.get p1 0) * paxisk
    let mutable center_1 = NPtr.get p1 1 + (NPtr.get p2 1 - NPtr.get p1 1) * paxisk
    let mutable center_2 = NPtr.get p1 2 + (NPtr.get p2 2 - NPtr.get p1 2) * paxisk
    let mutable radius = paxisdr / 2.0f

    // iteratively adjust the sphere up until all points fit
    for i = 0 to count - 1 do
        let p = NPtr.add points (i * points_stride_float)
        let r = NPtr.get radii (i * radii_stride_float)

        let d2 =
            (NPtr.get p 0 - center_0) * (NPtr.get p 0 - center_0) +
            (NPtr.get p 1 - center_1) * (NPtr.get p 1 - center_1) +
            (NPtr.get p 2 - center_2) * (NPtr.get p 2 - center_2)
        let d = MathF.Sqrt(d2)

        if d + r > radius then
            let k = if d > 0.0f then (d + r - radius) / (2.0f * d) else 0.0f

            center_0 <- center_0 + k * (NPtr.get p 0 - center_0)
            center_1 <- center_1 + k * (NPtr.get p 1 - center_1)
            center_2 <- center_2 + k * (NPtr.get p 2 - center_2)
            radius <- (radius + d + r) / 2.0f

    NPtr.set result 0 center_0
    NPtr.set result 1 center_1
    NPtr.set result 2 center_2
    NPtr.set result 3 radius

let private getMeshletScore (distance: float32) (spread: float32) (cone_weight: float32) (expected_radius: float32) : float32 =
    let cone = 1.0f - spread * cone_weight
    let cone_clamped = if cone < 1e-3f then 1e-3f else cone

    (1.0f + distance / expected_radius * (1.0f - cone_weight)) * cone_clamped

let private getMeshletCone (acc: Cone) (triangle_count: uint32) : Cone =
    let mutable result = acc

    let center_scale = if triangle_count = 0u then 0.0f else 1.0f / float32 triangle_count

    result.px <- result.px * center_scale
    result.py <- result.py * center_scale
    result.pz <- result.pz * center_scale

    let axis_length = result.nx * result.nx + result.ny * result.ny + result.nz * result.nz
    let axis_scale = if axis_length = 0.0f then 0.0f else 1.0f / MathF.Sqrt(axis_length)

    result.nx <- result.nx * axis_scale
    result.ny <- result.ny * axis_scale
    result.nz <- result.nz * axis_scale

    result

let private computeTriangleCones (triangles: nativeptr<Cone>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) : float32 =
    ignore vertex_count

    let vertex_stride_float = vertex_positions_stride / sizeof<float32>
    let face_count = index_count / 3

    let mutable mesh_area = 0.0f

    for i = 0 to face_count - 1 do
        let a = int (NPtr.get indices (i * 3 + 0))
        let b = int (NPtr.get indices (i * 3 + 1))
        let c = int (NPtr.get indices (i * 3 + 2))
        assert (a < vertex_count && b < vertex_count && c < vertex_count)

        let p0 = NPtr.add vertex_positions (vertex_stride_float * a)
        let p1 = NPtr.add vertex_positions (vertex_stride_float * b)
        let p2 = NPtr.add vertex_positions (vertex_stride_float * c)

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
        let invarea = if area = 0.0f then 0.0f else 1.0f / area

        let mutable tri = NPtr.get triangles i
        tri.px <- (NPtr.get p0 0 + NPtr.get p1 0 + NPtr.get p2 0) / 3.0f
        tri.py <- (NPtr.get p0 1 + NPtr.get p1 1 + NPtr.get p2 1) / 3.0f
        tri.pz <- (NPtr.get p0 2 + NPtr.get p1 2 + NPtr.get p2 2) / 3.0f
        tri.nx <- normalx * invarea
        tri.ny <- normaly * invarea
        tri.nz <- normalz * invarea
        NPtr.set triangles i tri

        mesh_area <- mesh_area + area

    mesh_area

let private appendMeshlet (meshlet: byref<meshopt_Meshlet>) (a: uint32) (b: uint32) (c: uint32) (used: nativeptr<int16>) (meshlets: nativeptr<meshopt_Meshlet>) (meshlet_vertices: nativeptr<uint32>) (meshlet_triangles: nativeptr<byte>) (meshlet_offset: int) (max_vertices: int) (max_triangles: int) (split: bool) : bool =
    let av = NPtr.get used (int a)
    let bv = NPtr.get used (int b)
    let cv = NPtr.get used (int c)

    let mutable result = false

    let used_extra = (if av < 0s then 1 else 0) + (if bv < 0s then 1 else 0) + (if cv < 0s then 1 else 0)

    if int meshlet.vertex_count + used_extra > max_vertices || int meshlet.triangle_count >= max_triangles || split then
        NPtr.set meshlets meshlet_offset meshlet

        for j = 0 to int meshlet.vertex_count - 1 do
            NPtr.set used (int (NPtr.get meshlet_vertices (int meshlet.vertex_offset + j))) -1s

        meshlet.vertex_offset <- meshlet.vertex_offset + meshlet.vertex_count
        meshlet.triangle_offset <- meshlet.triangle_offset + meshlet.triangle_count * 3u
        meshlet.vertex_count <- 0u
        meshlet.triangle_count <- 0u

        result <- true

    if NPtr.get used (int a) < 0s then
        NPtr.set used (int a) (int16 meshlet.vertex_count)
        NPtr.set meshlet_vertices (int meshlet.vertex_offset + int meshlet.vertex_count) a
        meshlet.vertex_count <- meshlet.vertex_count + 1u

    if NPtr.get used (int b) < 0s then
        NPtr.set used (int b) (int16 meshlet.vertex_count)
        NPtr.set meshlet_vertices (int meshlet.vertex_offset + int meshlet.vertex_count) b
        meshlet.vertex_count <- meshlet.vertex_count + 1u

    if NPtr.get used (int c) < 0s then
        NPtr.set used (int c) (int16 meshlet.vertex_count)
        NPtr.set meshlet_vertices (int meshlet.vertex_offset + int meshlet.vertex_count) c
        meshlet.vertex_count <- meshlet.vertex_count + 1u

    NPtr.set meshlet_triangles (int meshlet.triangle_offset + int meshlet.triangle_count * 3 + 0) (byte (NPtr.get used (int a)))
    NPtr.set meshlet_triangles (int meshlet.triangle_offset + int meshlet.triangle_count * 3 + 1) (byte (NPtr.get used (int b)))
    NPtr.set meshlet_triangles (int meshlet.triangle_offset + int meshlet.triangle_count * 3 + 2) (byte (NPtr.get used (int c)))
    meshlet.triangle_count <- meshlet.triangle_count + 1u

    result

let private getNeighborTriangle (meshlet: meshopt_Meshlet) (meshlet_cone: Cone) (meshlet_vertices: nativeptr<uint32>) (indices: nativeptr<uint32>) (adjacency: TriangleAdjacency2) (triangles: nativeptr<Cone>) (live_triangles: nativeptr<uint32>) (used: nativeptr<int16>) (meshlet_expected_radius: float32) (cone_weight: float32) : uint32 =
    let mutable best_triangle = ~~~0u
    let mutable best_priority = 5
    let mutable best_score = Single.MaxValue

    for i = 0 to int meshlet.vertex_count - 1 do
        let index = int (NPtr.get meshlet_vertices (int meshlet.vertex_offset + i))

        let neighbors = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets index))
        let neighbors_size = int (NPtr.get adjacency.counts index)

        for j = 0 to neighbors_size - 1 do
            let triangle = NPtr.get neighbors j
            let a = int (NPtr.get indices (int triangle * 3 + 0))
            let b = int (NPtr.get indices (int triangle * 3 + 1))
            let c = int (NPtr.get indices (int triangle * 3 + 2))

            let extra = (if NPtr.get used a < 0s then 1 else 0) + (if NPtr.get used b < 0s then 1 else 0) + (if NPtr.get used c < 0s then 1 else 0)
            assert (extra <= 2)

            let mutable priority = -1

            // triangles that don't add new vertices to meshlets are max. priority
            if extra = 0 then
                priority <- 0
            // artificially increase the priority of dangling triangles as they're expensive to add to new meshlets
            elif NPtr.get live_triangles a = 1u || NPtr.get live_triangles b = 1u || NPtr.get live_triangles c = 1u then
                priority <- 1
            // if two vertices have live count of 2, removing this triangle will make another triangle dangling which is good for overall flow
            elif (if NPtr.get live_triangles a = 2u then 1 else 0) + (if NPtr.get live_triangles b = 2u then 1 else 0) + (if NPtr.get live_triangles c = 2u then 1 else 0) >= 2 then
                priority <- 1 + extra
            // otherwise adjust priority to be after the above cases, 3 or 4 based on used[] count
            else
                priority <- 2 + extra

            // since topology-based priority is always more important than the score, we can skip scoring in some cases
            if priority <= best_priority then
                let tri_cone = NPtr.get triangles (int triangle)

                let dx = tri_cone.px - meshlet_cone.px
                let dy = tri_cone.py - meshlet_cone.py
                let dz = tri_cone.pz - meshlet_cone.pz
                let distance = MathF.Sqrt(dx * dx + dy * dy + dz * dz)
                let spread = tri_cone.nx * meshlet_cone.nx + tri_cone.ny * meshlet_cone.ny + tri_cone.nz * meshlet_cone.nz

                let score = getMeshletScore distance spread cone_weight meshlet_expected_radius

                // note that topology-based priority is always more important than the score
                // this helps maintain reasonable effectiveness of meshlet data and reduces scoring cost
                if priority < best_priority || score < best_score then
                    best_triangle <- triangle
                    best_priority <- priority
                    best_score <- score

    best_triangle

let private appendSeedTriangles (seeds: nativeptr<uint32>) (meshlet: meshopt_Meshlet) (meshlet_vertices: nativeptr<uint32>) (indices: nativeptr<uint32>) (adjacency: TriangleAdjacency2) (triangles: nativeptr<Cone>) (live_triangles: nativeptr<uint32>) (cornerx: float32) (cornery: float32) (cornerz: float32) : int =
    let best_seeds = Array.create kMeshletAddSeeds (~~~0u)
    let best_live = Array.create kMeshletAddSeeds (~~~0u)
    let best_score = Array.create kMeshletAddSeeds Single.MaxValue

    for i = 0 to int meshlet.vertex_count - 1 do
        let index = int (NPtr.get meshlet_vertices (int meshlet.vertex_offset + i))

        let mutable best_neighbor = ~~~0u
        let mutable best_neighbor_live = ~~~0u

        // find the neighbor with the smallest live metric
        let neighbors = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets index))
        let neighbors_size = int (NPtr.get adjacency.counts index)

        for j = 0 to neighbors_size - 1 do
            let triangle = NPtr.get neighbors j
            let a = int (NPtr.get indices (int triangle * 3 + 0))
            let b = int (NPtr.get indices (int triangle * 3 + 1))
            let c = int (NPtr.get indices (int triangle * 3 + 2))

            let live = NPtr.get live_triangles a + NPtr.get live_triangles b + NPtr.get live_triangles c

            if live < best_neighbor_live then
                best_neighbor <- triangle
                best_neighbor_live <- live

        // add the neighbor to the list of seeds; the list is unsorted and the replacement criteria is approximate
        if best_neighbor <> ~~~0u then
            let tri = NPtr.get triangles (int best_neighbor)
            let dx = tri.px - cornerx
            let dy = tri.py - cornery
            let dz = tri.pz - cornerz
            let best_neighbor_score = MathF.Sqrt(dx * dx + dy * dy + dz * dz)

            let mutable j = 0
            let mutable found = false
            while j < kMeshletAddSeeds && not found do
                // non-strict comparison reduces the number of duplicate seeds (triangles adjacent to multiple vertices)
                if best_neighbor_live < best_live.[j] || (best_neighbor_live = best_live.[j] && best_neighbor_score <= best_score.[j]) then
                    best_seeds.[j] <- best_neighbor
                    best_live.[j] <- best_neighbor_live
                    best_score.[j] <- best_neighbor_score
                    found <- true
                j <- j + 1

    // add surviving seeds to the meshlet
    let mutable seed_count = 0

    for i = 0 to kMeshletAddSeeds - 1 do
        if best_seeds.[i] <> ~~~0u then
            NPtr.set seeds seed_count best_seeds.[i]
            seed_count <- seed_count + 1

    seed_count

let private pruneSeedTriangles (seeds: nativeptr<uint32>) (seed_count: int) (emitted_flags: nativeptr<byte>) : int =
    let mutable result = 0

    for i = 0 to seed_count - 1 do
        let index = NPtr.get seeds i

        NPtr.set seeds result index
        result <- result + (if NPtr.get emitted_flags (int index) = 0uy then 1 else 0)

    result

let private selectSeedTriangle (seeds: nativeptr<uint32>) (seed_count: int) (indices: nativeptr<uint32>) (triangles: nativeptr<Cone>) (live_triangles: nativeptr<uint32>) (cornerx: float32) (cornery: float32) (cornerz: float32) : uint32 =
    let mutable best_seed = ~~~0u
    let mutable best_live = ~~~0u
    let mutable best_score = Single.MaxValue

    for i = 0 to seed_count - 1 do
        let index = NPtr.get seeds i
        let a = int (NPtr.get indices (int index * 3 + 0))
        let b = int (NPtr.get indices (int index * 3 + 1))
        let c = int (NPtr.get indices (int index * 3 + 2))

        let live = NPtr.get live_triangles a + NPtr.get live_triangles b + NPtr.get live_triangles c
        let tri = NPtr.get triangles (int index)
        let dx = tri.px - cornerx
        let dy = tri.py - cornery
        let dz = tri.pz - cornerz
        let score = MathF.Sqrt(dx * dx + dy * dy + dz * dz)

        if live < best_live || (live = best_live && score < best_score) then
            best_seed <- index
            best_live <- live
            best_score <- score

    best_seed

let private kdtreePartition (indices: nativeptr<uint32>) (count: int) (points: nativeptr<float32>) (stride: int) (axis: int) (pivot: float32) : int =
    let mutable m = 0

    // invariant: elements in range [0, m) are < pivot, elements in range [m, i) are >= pivot
    for i = 0 to count - 1 do
        let v = NPtr.get points (int (NPtr.get indices i) * stride + axis)

        // swap(m, i) unconditionally
        let t = NPtr.get indices m
        NPtr.set indices m (NPtr.get indices i)
        NPtr.set indices i t

        // when v >= pivot, we swap i with m without advancing it, preserving invariants
        if v < pivot then m <- m + 1

    m

let private kdtreeBuildLeaf (offset: int) (nodes: nativeptr<KDNode>) (node_count: int) (indices: nativeptr<uint32>) (count: int) : int =
    assert (offset + count <= node_count)
    ignore node_count

    let mutable result = NPtr.get nodes offset
    setIndex &result (NPtr.get indices 0)
    setAxisAndChildren &result 3u (uint32 count)
    NPtr.set nodes offset result

    // all remaining points are stored in nodes immediately following the leaf
    for i = 1 to count - 1 do
        let mutable tail = NPtr.get nodes (offset + i)
        setIndex &tail (NPtr.get indices i)
        setAxisAndChildren &tail 3u (~~~0u >>> 2) // bogus value to prevent misuse
        NPtr.set nodes (offset + i) tail

    offset + count

let rec private kdtreeBuild (offset: int) (nodes: nativeptr<KDNode>) (node_count: int) (points: nativeptr<float32>) (stride: int) (indices: nativeptr<uint32>) (count: int) (leaf_size: int) (depth: int) : int =
    assert (count > 0)
    assert (offset < node_count)

    if count <= leaf_size then
        kdtreeBuildLeaf offset nodes node_count indices count
    else
        let mean = [| 0.0f; 0.0f; 0.0f |]
        let vars = [| 0.0f; 0.0f; 0.0f |]
        let mutable runc = 1.0f

        // gather statistics on the points in the subtree using Welford's algorithm
        for i = 0 to count - 1 do
            let runs = 1.0f / runc
            let point = NPtr.add points (int (NPtr.get indices i) * stride)

            for k = 0 to 2 do
                let delta = NPtr.get point k - mean.[k]
                mean.[k] <- mean.[k] + delta * runs
                vars.[k] <- vars.[k] + delta * (NPtr.get point k - mean.[k])

            runc <- runc + 1.0f

        // split axis is one where the variance is largest
        let axis =
            if vars.[0] >= vars.[1] && vars.[0] >= vars.[2] then 0
            elif vars.[1] >= vars.[2] then 1
            else 2

        let split = mean.[axis]
        let middle = kdtreePartition indices count points stride axis split

        // when the partition is degenerate simply consolidate the points into a single node
        // this also ensures recursion depth is bounded on pathological inputs
        if middle <= leaf_size / 2 || middle >= count - leaf_size / 2 || depth >= kMeshletMaxTreeDepth then
            kdtreeBuildLeaf offset nodes node_count indices count
        else
            let mutable result = NPtr.get nodes offset
            setSplit &result split
            setAxisAndChildren &result (uint32 axis) 0u // children set later
            NPtr.set nodes offset result

            // left subtree is right after our node
            let next_offset = kdtreeBuild (offset + 1) nodes node_count points stride indices middle leaf_size (depth + 1)

            // distance to the right subtree is represented explicitly
            assert (next_offset - offset > 1)
            let mutable result2 = NPtr.get nodes offset
            setAxisAndChildren &result2 (uint32 axis) (uint32 (next_offset - offset - 1))
            NPtr.set nodes offset result2

            kdtreeBuild next_offset nodes node_count points stride (NPtr.add indices middle) (count - middle) leaf_size (depth + 1)

let rec private kdtreeNearest (nodes: nativeptr<KDNode>) (root: int) (points: nativeptr<float32>) (stride: int) (emitted_flags: nativeptr<byte>) (position: nativeptr<float32>) (result: byref<uint32>) (limit: byref<float32>) =
    let node = NPtr.get nodes root

    if getChildren node = 0u then
        ()
    elif getAxis node = 3u then
        // leaf
        let mutable inactive = true

        for i = 0 to int (getChildren node) - 1 do
            let index = getIndex (NPtr.get nodes (root + i))

            if NPtr.get emitted_flags (int index) = 0uy then
                inactive <- false

                let point = NPtr.add points (int index * stride)

                let dx = NPtr.get point 0 - NPtr.get position 0
                let dy = NPtr.get point 1 - NPtr.get position 1
                let dz = NPtr.get point 2 - NPtr.get position 2
                let distance = MathF.Sqrt(dx * dx + dy * dy + dz * dz)

                if distance < limit then
                    result <- index
                    limit <- distance

        // deactivate leaves that no longer have items to emit
        if inactive then
            let mutable n = NPtr.get nodes root
            setAxisAndChildren &n 3u 0u
            NPtr.set nodes root n
    else
        // branch; we order recursion to process the node that search position is in first
        let delta = NPtr.get position (int (getAxis node)) - getSplit node
        let first = if delta <= 0.0f then 0u else getChildren node
        let second = first ^^^ getChildren node

        // deactivate branches that no longer have items to emit to accelerate traversal
        // note that we do this *before* recursing which delays deactivation but keeps tail calls
        if (getChildren (NPtr.get nodes (root + 1 + int first)) ||| getChildren (NPtr.get nodes (root + 1 + int second))) = 0u then
            let mutable n = NPtr.get nodes root
            setAxisAndChildren &n (getAxis node) 0u
            NPtr.set nodes root n

        // recursion depth is bounded by tree depth (which is limited by construction)
        kdtreeNearest nodes (root + 1 + int first) points stride emitted_flags position &result &limit

        // only process the other node if it can have a match based on closest distance so far
        if MathF.Abs(delta) <= limit then
            kdtreeNearest nodes (root + 1 + int second) points stride emitted_flags position &result &limit

let private boxMerge (box: byref<BVHBoxT>) (other: BVHBox) : float32 =
    box.min_0 <- if other.min_0 < box.min_0 then other.min_0 else box.min_0
    box.min_1 <- if other.min_1 < box.min_1 then other.min_1 else box.min_1
    box.min_2 <- if other.min_2 < box.min_2 then other.min_2 else box.min_2

    box.max_0 <- if other.max_0 > box.max_0 then other.max_0 else box.max_0
    box.max_1 <- if other.max_1 > box.max_1 then other.max_1 else box.max_1
    box.max_2 <- if other.max_2 > box.max_2 then other.max_2 else box.max_2

    let sx = box.max_0 - box.min_0
    let sy = box.max_1 - box.min_1
    let sz = box.max_2 - box.min_2
    sx * sy + sx * sz + sy * sz

let inline private radixFloat (v: uint32) : uint32 =
    // if sign bit is 0, flip sign bit
    // if sign bit is 1, flip everything
    let mask = (int v >>> 31 |> uint32) ||| 0x80000000u
    v ^^^ mask

let private computeHistogram (hist: nativeptr<uint32>) (data: nativeptr<float32>) (count: int) =
    // hist is [1024][3] = 3072 elements
    NPtr.memset hist 0uy (1024 * 3 * sizeof<uint32>)

    let bits : nativeptr<uint32> = NPtr.cast data

    // compute 3 10-bit histograms in parallel (dropping 2 LSB)
    for i = 0 to count - 1 do
        let id = radixFloat (NPtr.get bits i)

        let idx0 = int ((id >>> 2) &&& 1023u)
        let idx1 = int ((id >>> 12) &&& 1023u)
        let idx2 = int ((id >>> 22) &&& 1023u)

        NPtr.set hist (idx0 * 3 + 0) (NPtr.get hist (idx0 * 3 + 0) + 1u)
        NPtr.set hist (idx1 * 3 + 1) (NPtr.get hist (idx1 * 3 + 1) + 1u)
        NPtr.set hist (idx2 * 3 + 2) (NPtr.get hist (idx2 * 3 + 2) + 1u)

    let mutable sum0 = 0u
    let mutable sum1 = 0u
    let mutable sum2 = 0u

    // replace histogram data with prefix histogram sums in-place
    for i = 0 to 1023 do
        let hx = NPtr.get hist (i * 3 + 0)
        let hy = NPtr.get hist (i * 3 + 1)
        let hz = NPtr.get hist (i * 3 + 2)

        NPtr.set hist (i * 3 + 0) sum0
        NPtr.set hist (i * 3 + 1) sum1
        NPtr.set hist (i * 3 + 2) sum2

        sum0 <- sum0 + hx
        sum1 <- sum1 + hy
        sum2 <- sum2 + hz

    assert (int sum0 = count && int sum1 = count && int sum2 = count)

let private radixPass (destination: nativeptr<uint32>) (source: nativeptr<uint32>) (keys: nativeptr<float32>) (count: int) (hist: nativeptr<uint32>) (pass: int) =
    let bits : nativeptr<uint32> = NPtr.cast keys
    let bitoff = pass * 10 + 2 // drop 2 LSB to be able to use 3 10-bit passes

    for i = 0 to count - 1 do
        let id = int ((radixFloat (NPtr.get bits (int (NPtr.get source i))) >>> bitoff) &&& 1023u)

        NPtr.set destination (int (NPtr.get hist (id * 3 + pass))) (NPtr.get source i)
        NPtr.set hist (id * 3 + pass) (NPtr.get hist (id * 3 + pass) + 1u)

let private bvhPrepare (boxes: nativeptr<BVHBox>) (centroids: nativeptr<float32>) (indices: nativeptr<uint32>) (face_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_stride_float: int) =
    ignore vertex_count

    for i = 0 to face_count - 1 do
        let a = int (NPtr.get indices (i * 3 + 0))
        let b = int (NPtr.get indices (i * 3 + 1))
        let c = int (NPtr.get indices (i * 3 + 2))
        assert (a < vertex_count && b < vertex_count && c < vertex_count)

        let va = NPtr.add vertex_positions (vertex_stride_float * a)
        let vb = NPtr.add vertex_positions (vertex_stride_float * b)
        let vc = NPtr.add vertex_positions (vertex_stride_float * c)

        let mutable box = NPtr.get boxes i

        // min
        let va0 = NPtr.get va 0
        let va1 = NPtr.get va 1
        let va2 = NPtr.get va 2
        let vb0 = NPtr.get vb 0
        let vb1 = NPtr.get vb 1
        let vb2 = NPtr.get vb 2
        let vc0 = NPtr.get vc 0
        let vc1 = NPtr.get vc 1
        let vc2 = NPtr.get vc 2

        box.min_0 <- if va0 < vb0 then va0 else vb0
        box.min_0 <- if vc0 < box.min_0 then vc0 else box.min_0
        box.min_1 <- if va1 < vb1 then va1 else vb1
        box.min_1 <- if vc1 < box.min_1 then vc1 else box.min_1
        box.min_2 <- if va2 < vb2 then va2 else vb2
        box.min_2 <- if vc2 < box.min_2 then vc2 else box.min_2

        box.max_0 <- if va0 > vb0 then va0 else vb0
        box.max_0 <- if vc0 > box.max_0 then vc0 else box.max_0
        box.max_1 <- if va1 > vb1 then va1 else vb1
        box.max_1 <- if vc1 > box.max_1 then vc1 else box.max_1
        box.max_2 <- if va2 > vb2 then va2 else vb2
        box.max_2 <- if vc2 > box.max_2 then vc2 else box.max_2

        NPtr.set boxes i box

        NPtr.set centroids (i + face_count * 0) ((box.min_0 + box.max_0) / 2.0f)
        NPtr.set centroids (i + face_count * 1) ((box.min_1 + box.max_1) / 2.0f)
        NPtr.set centroids (i + face_count * 2) ((box.min_2 + box.max_2) / 2.0f)

let private bvhCountVertices (order: nativeptr<uint32>) (count: int) (used: nativeptr<int16>) (indices: nativeptr<uint32>) (out_ptr: nativeptr<uint32>) : int =
    // count number of unique vertices
    let mutable used_vertices = 0
    for i = 0 to count - 1 do
        let index = int (NPtr.get order i)
        let a = int (NPtr.get indices (index * 3 + 0))
        let b = int (NPtr.get indices (index * 3 + 1))
        let c = int (NPtr.get indices (index * 3 + 2))

        used_vertices <- used_vertices + (if NPtr.get used a < 0s then 1 else 0) + (if NPtr.get used b < 0s then 1 else 0) + (if NPtr.get used c < 0s then 1 else 0)
        NPtr.set used a 1s
        NPtr.set used b 1s
        NPtr.set used c 1s

        if NPtr.toNI out_ptr <> 0n then
            NPtr.set out_ptr i (uint32 used_vertices)

    // reset used[] for future invocations
    for i = 0 to count - 1 do
        let index = int (NPtr.get order i)
        let a = int (NPtr.get indices (index * 3 + 0))
        let b = int (NPtr.get indices (index * 3 + 1))
        let c = int (NPtr.get indices (index * 3 + 2))

        NPtr.set used a -1s
        NPtr.set used b -1s
        NPtr.set used c -1s

    used_vertices

let private bvhCountVerticesNoOut (order: nativeptr<uint32>) (count: int) (used: nativeptr<int16>) (indices: nativeptr<uint32>) : int =
    bvhCountVertices order count used indices (NPtr.ofNI 0n)

let private bvhPackLeaf (boundary: nativeptr<byte>) (count: int) =
    // mark meshlet boundary for future reassembly
    assert (count > 0)

    NPtr.set boundary 0 1uy
    NPtr.memset (NPtr.add boundary 1) 0uy (count - 1)

let private bvhPackTail (boundary: nativeptr<byte>) (order: nativeptr<uint32>) (count: int) (used: nativeptr<int16>) (indices: nativeptr<uint32>) (max_vertices: int) (max_triangles: int) =
    let mutable i = 0
    while i < count do
        let chunk = if i + max_triangles <= count then max_triangles else count - i

        if bvhCountVerticesNoOut (NPtr.add order i) chunk used indices <= max_vertices then
            bvhPackLeaf (NPtr.add boundary i) chunk
            i <- i + chunk
        else
            // chunk is vertex bound, split it into smaller meshlets
            assert (chunk > max_vertices / 3)

            bvhPackLeaf (NPtr.add boundary i) (max_vertices / 3)
            i <- i + max_vertices / 3

let private bvhDivisible (count: int) (min: int) (max: int) : bool =
    // count is representable as a sum of values in [min..max] if it in range of [k*min..k*min+k*(max-min)]
    // equivalent to ceil(count / max) <= floor(count / min), but the form below allows using idiv
    // we avoid expensive integer divisions in the common case where min is <= max/2
    if min * 2 <= max then count >= min
    else count % min <= (count / min) * (max - min)

let private bvhComputeArea (areas: nativeptr<float32>) (boxes: nativeptr<BVHBox>) (order: nativeptr<uint32>) (count: int) =
    let mutable accuml = BVHBoxT()
    accuml.min_0 <- Single.MaxValue
    accuml.min_1 <- Single.MaxValue
    accuml.min_2 <- Single.MaxValue
    accuml.min_3 <- 0.0f
    accuml.max_0 <- -Single.MaxValue
    accuml.max_1 <- -Single.MaxValue
    accuml.max_2 <- -Single.MaxValue
    accuml.max_3 <- 0.0f

    let mutable accumr = accuml

    for i = 0 to count - 1 do
        let larea = boxMerge &accuml (NPtr.get boxes (int (NPtr.get order i)))
        let rarea = boxMerge &accumr (NPtr.get boxes (int (NPtr.get order (count - 1 - i))))

        NPtr.set areas i larea
        NPtr.set areas (i + count) rarea

let private bvhPivot (areas: nativeptr<float32>) (vertices: nativeptr<uint32>) (count: int) (step: int) (min: int) (max: int) (fill: float32) (maxfill: int) (out_cost: nativeptr<float32>) : int =
    let aligned = count >= min * 2 && bvhDivisible count min max
    let endd = if aligned then count - min else count - 1

    let rmaxfill = 1.0f / float32 maxfill

    // find best split that minimizes SAH
    let mutable bestsplit = 0
    let mutable bestcost = Single.MaxValue

    let mutable i = min - 1
    while i < endd do
        let lsplit = i + 1
        let rsplit = count - (i + 1)

        if bvhDivisible lsplit min max then
            if not aligned || bvhDivisible rsplit min max then
                // areas[x] = inclusive surface area of boxes[0..x]
                // areas[count-1-x] = inclusive surface area of boxes[x..count-1]
                let larea = NPtr.get areas i
                let rarea = NPtr.get areas ((count - 1 - (i + 1)) + count)
                let mutable cost = larea * float32 lsplit + rarea * float32 rsplit

                if cost <= bestcost then
                    // use vertex fill when splitting vertex limited clusters
                    let lfill = if NPtr.toNI vertices <> 0n then int (NPtr.get vertices i) else lsplit
                    let rfill = if NPtr.toNI vertices <> 0n then int (NPtr.get vertices i) else rsplit

                    // fill cost; use floating point math to round up to maxfill to avoid expensive integer modulo
                    let lrest = int (float32 (lfill + maxfill - 1) * rmaxfill) * maxfill - lfill
                    let rrest = int (float32 (rfill + maxfill - 1) * rmaxfill) * maxfill - rfill

                    cost <- cost + fill * (float32 lrest * larea + float32 rrest * rarea)

                    if cost < bestcost then
                        bestcost <- cost
                        bestsplit <- i + 1

        i <- i + step

    NPtr.set out_cost 0 bestcost
    bestsplit

let private bvhPartition (target: nativeptr<uint32>) (order: nativeptr<uint32>) (sides: nativeptr<byte>) (split: int) (count: int) =
    let mutable l = 0
    let mutable r = split

    for i = 0 to count - 1 do
        let side = int (NPtr.get sides (int (NPtr.get order i)))
        NPtr.set target (if side <> 0 then r else l) (NPtr.get order i)
        l <- l + 1
        l <- l - side
        r <- r + side

    assert (l = split && r = count)

let rec private bvhSplit (boxes: nativeptr<BVHBox>) (orderx: nativeptr<uint32>) (ordery: nativeptr<uint32>) (orderz: nativeptr<uint32>) (boundary: nativeptr<byte>) (count: int) (depth: int) (scratch: nativeptr<float32>) (used: nativeptr<int16>) (indices: nativeptr<uint32>) (max_vertices: int) (min_triangles: int) (max_triangles: int) (fill_weight: float32) =
    if count <= max_triangles && bvhCountVerticesNoOut orderx count used indices <= max_vertices then
        bvhPackLeaf boundary count
    else
        let axes = [| orderx; ordery; orderz |]

        // we can use step=1 unconditionally but to reduce the cost for min=max case we use step=max
        let step = if min_triangles = max_triangles && count > max_triangles then max_triangles else 1

        // if we could not pack the meshlet, we must be vertex bound
        let mint = if count <= max_triangles && max_vertices / 3 < min_triangles then max_vertices / 3 else min_triangles
        let maxfill = if count <= max_triangles then max_vertices else max_triangles

        // find best split that minimizes SAH
        let mutable bestk = -1
        let mutable bestsplit = 0
        let mutable bestcost = Single.MaxValue

        for k = 0 to 2 do
            let areas = scratch
            let mutable vertices : nativeptr<uint32> = NPtr.ofNI 0n

            bvhComputeArea areas boxes axes.[k] count

            if count <= max_triangles then
                // for vertex bound clusters, count number of unique vertices for each split
                vertices <- NPtr.cast<float32, uint32> (NPtr.add areas (2 * count))
                bvhCountVertices axes.[k] count used indices vertices |> ignore

            let mutable axiscost = Single.MaxValue
            let axiscost_ptr = NativePtr.stackalloc<float32> 1
            NPtr.set axiscost_ptr 0 Single.MaxValue
            let axissplit = bvhPivot areas vertices count step mint max_triangles fill_weight maxfill axiscost_ptr
            axiscost <- NPtr.get axiscost_ptr 0

            if axissplit <> 0 && axiscost < bestcost then
                bestk <- k
                bestcost <- axiscost
                bestsplit <- axissplit

        // this may happen if SAH costs along the admissible splits are NaN, or due to imbalanced splits on pathological inputs
        if bestk < 0 || depth >= kMeshletMaxTreeDepth then
            bvhPackTail boundary orderx count used indices max_vertices max_triangles
        else
            // mark sides of split for partitioning
            let sides : nativeptr<byte> = NPtr.cast<uint32, byte> (NPtr.add (NPtr.cast<float32, uint32> scratch) count)

            for i = 0 to bestsplit - 1 do
                NPtr.set sides (int (NPtr.get axes.[bestk] i)) 0uy

            for i = bestsplit to count - 1 do
                NPtr.set sides (int (NPtr.get axes.[bestk] i)) 1uy

            // partition all axes into two sides, maintaining order
            let temp : nativeptr<uint32> = NPtr.cast scratch

            for k = 0 to 2 do
                if k <> bestk then
                    let axis = axes.[k]
                    NPtr.memcpy temp axis (count * sizeof<uint32>)
                    bvhPartition axis temp sides bestsplit count

            // recursion depth is bounded due to max depth check above
            bvhSplit boxes orderx ordery orderz boundary bestsplit (depth + 1) scratch used indices max_vertices min_triangles max_triangles fill_weight
            bvhSplit boxes (NPtr.add orderx bestsplit) (NPtr.add ordery bestsplit) (NPtr.add orderz bestsplit) (NPtr.add boundary bestsplit) (count - bestsplit) (depth + 1) scratch used indices max_vertices min_triangles max_triangles fill_weight

// ---- Public API ----

let meshopt_buildMeshletsBound (index_count: int) (max_vertices: int) (max_triangles: int) : int =
    assert (index_count % 3 = 0)
    assert (max_vertices >= 3 && max_vertices <= kMeshletMaxVertices)
    assert (max_triangles >= 1 && max_triangles <= kMeshletMaxTriangles)

    // meshlet construction is limited by max vertices and max triangles per meshlet
    // the worst case is that the input is an unindexed stream since this equally stresses both limits
    // note that we assume that in the worst case, we leave 2 vertices unpacked in each meshlet - if we have space for 3 we can pack any triangle
    let max_vertices_conservative = max_vertices - 2
    let meshlet_limit_vertices = (index_count + max_vertices_conservative - 1) / max_vertices_conservative
    let meshlet_limit_triangles = (index_count / 3 + max_triangles - 1) / max_triangles

    if meshlet_limit_vertices > meshlet_limit_triangles then meshlet_limit_vertices else meshlet_limit_triangles

let meshopt_buildMeshletsFlex (meshlets: nativeptr<meshopt_Meshlet>) (meshlet_vertices: nativeptr<uint32>) (meshlet_triangles: nativeptr<byte>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (max_vertices: int) (min_triangles: int) (max_triangles: int) (cone_weight: float32) (split_factor: float32) : int =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    assert (max_vertices >= 3 && max_vertices <= kMeshletMaxVertices)
    assert (min_triangles >= 1 && min_triangles <= max_triangles && max_triangles <= kMeshletMaxTriangles)

    assert (cone_weight >= 0.0f && cone_weight <= 1.0f)
    assert (split_factor >= 0.0f)

    if index_count = 0 then
        0
    else

    use allocator = new meshopt_Allocator()

    let mutable adjacency = Unchecked.defaultof<TriangleAdjacency2>
    if vertex_count > index_count && index_count < int (1u <<< 31) then
        buildTriangleAdjacencySparse &adjacency indices index_count vertex_count allocator
    else
        buildTriangleAdjacency &adjacency indices index_count vertex_count allocator

    // live triangle counts; note, we alias adjacency.counts as we remove triangles after emitting them so the counts always match
    let live_triangles = adjacency.counts

    let face_count = index_count / 3

    let emitted_flags = allocator.allocate<byte>(face_count)
    NPtr.memset emitted_flags 0uy face_count

    // for each triangle, precompute centroid & normal to use for scoring
    let triangles = allocator.allocate<Cone>(face_count)
    let mesh_area = computeTriangleCones triangles indices index_count vertex_positions vertex_count vertex_positions_stride

    // assuming each meshlet is a square patch, expected radius is sqrt(expected area)
    let triangle_area_avg = if face_count = 0 then 0.0f else mesh_area / float32 face_count * 0.5f
    let meshlet_expected_radius = MathF.Sqrt(triangle_area_avg * float32 max_triangles) * 0.5f

    // build a kd-tree for nearest neighbor lookup
    let kdindices = allocator.allocate<uint32>(face_count)
    for i = 0 to face_count - 1 do
        NPtr.set kdindices i (uint32 i)

    let nodes = allocator.allocate<KDNode>(face_count * 2)
    let cone0ptr = NPtr.cast<Cone, float32> triangles  // &triangles[0].px
    kdtreeBuild 0 nodes (face_count * 2) cone0ptr (sizeof<Cone> / sizeof<float32>) kdindices face_count 8 0 |> ignore

    // find a specific corner of the mesh to use as a starting point for meshlet flow
    let mutable cornerx = Single.MaxValue
    let mutable cornery = Single.MaxValue
    let mutable cornerz = Single.MaxValue

    for i = 0 to face_count - 1 do
        let tri = NPtr.get triangles i

        cornerx <- if cornerx > tri.px then tri.px else cornerx
        cornery <- if cornery > tri.py then tri.py else cornery
        cornerz <- if cornerz > tri.pz then tri.pz else cornerz

    // index of the vertex in the meshlet, -1 if the vertex isn't used
    let used = allocator.allocate<int16>(vertex_count)
    clearUsed used vertex_count indices index_count

    // initial seed triangle is the one closest to the corner
    let mutable initial_seed = ~~~0u
    let mutable initial_score = Single.MaxValue

    for i = 0 to face_count - 1 do
        let tri = NPtr.get triangles i

        let dx = tri.px - cornerx
        let dy = tri.py - cornery
        let dz = tri.pz - cornerz
        let score = MathF.Sqrt(dx * dx + dy * dy + dz * dz)

        if initial_seed = ~~~0u || score < initial_score then
            initial_seed <- uint32 i
            initial_score <- score

    // seed triangles to continue meshlet flow
    let seeds = Array.zeroCreate<uint32> kMeshletMaxSeeds
    let mutable seed_count = 0

    let mutable meshlet = meshopt_Meshlet()
    let mutable meshlet_offset = 0

    let mutable meshlet_cone_acc = Cone()

    let mutable cont = true
    while cont do
        let meshlet_cone = getMeshletCone meshlet_cone_acc meshlet.triangle_count

        let mutable best_triangle = ~~~0u

        // for the first triangle, we don't have a meshlet cone yet, so we use the initial seed
        // to continue the meshlet, we select an adjacent triangle based on connectivity and spatial scoring
        if meshlet_offset = 0 && meshlet.triangle_count = 0u then
            best_triangle <- initial_seed
        else
            best_triangle <- getNeighborTriangle meshlet meshlet_cone meshlet_vertices indices adjacency triangles live_triangles used meshlet_expected_radius cone_weight

        let mutable split = false

        // when we run out of adjacent triangles we need to switch to spatial search; we currently just pick the closest triangle irrespective of connectivity
        if best_triangle = ~~~0u then
            let position = NativePtr.stackalloc<float32> 3
            NPtr.set position 0 meshlet_cone.px
            NPtr.set position 1 meshlet_cone.py
            NPtr.set position 2 meshlet_cone.pz
            let mutable index = ~~~0u
            let mutable distance = Single.MaxValue

            kdtreeNearest nodes 0 cone0ptr (sizeof<Cone> / sizeof<float32>) emitted_flags position &index &distance

            best_triangle <- index
            split <- int meshlet.triangle_count >= min_triangles && split_factor > 0.0f && distance > meshlet_expected_radius * split_factor

        if best_triangle = ~~~0u then
            cont <- false
        else

        let best_extra =
            (if NPtr.get used (int (NPtr.get indices (int best_triangle * 3 + 0))) < 0s then 1 else 0) +
            (if NPtr.get used (int (NPtr.get indices (int best_triangle * 3 + 1))) < 0s then 1 else 0) +
            (if NPtr.get used (int (NPtr.get indices (int best_triangle * 3 + 2))) < 0s then 1 else 0)

        // if the best triangle doesn't fit into current meshlet, we re-select using seeds to maintain global flow
        if split || (int meshlet.vertex_count + best_extra > max_vertices || int meshlet.triangle_count >= max_triangles) then
            // pin the seeds array
            use pinned = fixed seeds
            let seedsPtr = pinned

            seed_count <- pruneSeedTriangles seedsPtr seed_count emitted_flags
            seed_count <- if seed_count + kMeshletAddSeeds <= kMeshletMaxSeeds then seed_count else kMeshletMaxSeeds - kMeshletAddSeeds
            seed_count <- seed_count + appendSeedTriangles (NPtr.add seedsPtr seed_count) meshlet meshlet_vertices indices adjacency triangles live_triangles cornerx cornery cornerz

            let best_seed = selectSeedTriangle seedsPtr seed_count indices triangles live_triangles cornerx cornery cornerz

            // we may not find a valid seed triangle if the mesh is disconnected as seeds are based on adjacency
            best_triangle <- if best_seed <> ~~~0u then best_seed else best_triangle

        let a = NPtr.get indices (int best_triangle * 3 + 0)
        let b = NPtr.get indices (int best_triangle * 3 + 1)
        let c = NPtr.get indices (int best_triangle * 3 + 2)
        assert (int a < vertex_count && int b < vertex_count && int c < vertex_count)

        // add meshlet to the output; when the current meshlet is full we reset the accumulated bounds
        if appendMeshlet &meshlet a b c used meshlets meshlet_vertices meshlet_triangles meshlet_offset max_vertices max_triangles split then
            meshlet_offset <- meshlet_offset + 1
            meshlet_cone_acc <- Cone()

        // remove emitted triangle from adjacency data
        // this makes sure that we spend less time traversing these lists on subsequent iterations
        // live triangle counts are updated as a byproduct of these adjustments
        for k = 0 to 2 do
            let index = int (NPtr.get indices (int best_triangle * 3 + k))

            let neighbors = NPtr.add adjacency.data (int (NPtr.get adjacency.offsets index))
            let neighbors_size = int (NPtr.get adjacency.counts index)

            let mutable found = false
            let mutable ii = 0
            while ii < neighbors_size && not found do
                let tri = NPtr.get neighbors ii

                if tri = best_triangle then
                    NPtr.set neighbors ii (NPtr.get neighbors (neighbors_size - 1))
                    NPtr.set adjacency.counts index (NPtr.get adjacency.counts index - 1u)
                    found <- true

                ii <- ii + 1

        // update aggregated meshlet cone data for scoring subsequent triangles
        let bt = NPtr.get triangles (int best_triangle)
        meshlet_cone_acc.px <- meshlet_cone_acc.px + bt.px
        meshlet_cone_acc.py <- meshlet_cone_acc.py + bt.py
        meshlet_cone_acc.pz <- meshlet_cone_acc.pz + bt.pz
        meshlet_cone_acc.nx <- meshlet_cone_acc.nx + bt.nx
        meshlet_cone_acc.ny <- meshlet_cone_acc.ny + bt.ny
        meshlet_cone_acc.nz <- meshlet_cone_acc.nz + bt.nz

        assert (NPtr.get emitted_flags (int best_triangle) = 0uy)
        NPtr.set emitted_flags (int best_triangle) 1uy

    if meshlet.triangle_count <> 0u then
        NPtr.set meshlets meshlet_offset meshlet
        meshlet_offset <- meshlet_offset + 1

    assert (meshlet_offset <= meshopt_buildMeshletsBound index_count max_vertices min_triangles)
    assert (int meshlet.triangle_offset + int meshlet.triangle_count * 3 <= index_count && int meshlet.vertex_offset + int meshlet.vertex_count <= index_count)
    meshlet_offset

let meshopt_buildMeshlets (meshlets: nativeptr<meshopt_Meshlet>) (meshlet_vertices: nativeptr<uint32>) (meshlet_triangles: nativeptr<byte>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (max_vertices: int) (max_triangles: int) (cone_weight: float32) : int =
    meshopt_buildMeshletsFlex meshlets meshlet_vertices meshlet_triangles indices index_count vertex_positions vertex_count vertex_positions_stride max_vertices max_triangles max_triangles cone_weight 0.0f

let meshopt_buildMeshletsScan (meshlets: nativeptr<meshopt_Meshlet>) (meshlet_vertices: nativeptr<uint32>) (meshlet_triangles: nativeptr<byte>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (max_vertices: int) (max_triangles: int) : int =
    assert (index_count % 3 = 0)

    assert (max_vertices >= 3 && max_vertices <= kMeshletMaxVertices)
    assert (max_triangles >= 1 && max_triangles <= kMeshletMaxTriangles)

    use allocator = new meshopt_Allocator()

    // index of the vertex in the meshlet, -1 if the vertex isn't used
    let used = allocator.allocate<int16>(vertex_count)
    clearUsed used vertex_count indices index_count

    let mutable meshlet = meshopt_Meshlet()
    let mutable meshlet_offset = 0

    let mutable i = 0
    while i < index_count do
        let a = NPtr.get indices (i + 0)
        let b = NPtr.get indices (i + 1)
        let c = NPtr.get indices (i + 2)
        assert (int a < vertex_count && int b < vertex_count && int c < vertex_count)

        // appends triangle to the meshlet and writes previous meshlet to the output if full
        if appendMeshlet &meshlet a b c used meshlets meshlet_vertices meshlet_triangles meshlet_offset max_vertices max_triangles false then
            meshlet_offset <- meshlet_offset + 1

        i <- i + 3

    if meshlet.triangle_count <> 0u then
        NPtr.set meshlets meshlet_offset meshlet
        meshlet_offset <- meshlet_offset + 1

    assert (meshlet_offset <= meshopt_buildMeshletsBound index_count max_vertices max_triangles)
    assert (int meshlet.triangle_offset + int meshlet.triangle_count * 3 <= index_count && int meshlet.vertex_offset + int meshlet.vertex_count <= index_count)
    meshlet_offset

let meshopt_buildMeshletsSpatial (meshlets: nativeptr<meshopt_Meshlet>) (meshlet_vertices: nativeptr<uint32>) (meshlet_triangles: nativeptr<byte>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (max_vertices: int) (min_triangles: int) (max_triangles: int) (fill_weight: float32) : int =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    assert (max_vertices >= 3 && max_vertices <= kMeshletMaxVertices)
    assert (min_triangles >= 1 && min_triangles <= max_triangles && max_triangles <= kMeshletMaxTriangles)

    if index_count = 0 then
        0
    else

    let face_count = index_count / 3
    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    use allocator = new meshopt_Allocator()

    // 3 floats plus 1 uint for sorting, or
    // 2 floats plus 1 uint for pivoting, or
    // 1 uint plus 1 byte for partitioning
    let scratch = allocator.allocate<float32>(face_count * 4)

    // compute bounding boxes and centroids for sorting
    let boxes = allocator.allocate<BVHBox>(face_count + 1) // padding for SIMD
    bvhPrepare boxes scratch indices face_count vertex_positions vertex_count vertex_stride_float
    NPtr.memset (NPtr.add boxes face_count) 0uy sizeof<BVHBox>

    let axes = allocator.allocate<uint32>(face_count * 3)
    let temp : nativeptr<uint32> = NPtr.add (NPtr.cast<float32, uint32> scratch) (face_count * 3)

    for k = 0 to 2 do
        let order = NPtr.add axes (k * face_count)
        let keys = NPtr.add scratch (k * face_count)

        let hist = allocator.allocate<uint32>(1024 * 3)
        computeHistogram hist keys face_count

        // 3-pass radix sort computes the resulting order into axes
        for i = 0 to face_count - 1 do
            NPtr.set temp i (uint32 i)

        radixPass order temp keys face_count hist 0
        radixPass temp order keys face_count hist 1
        radixPass order temp keys face_count hist 2

        allocator.deallocate(NPtr.toNI hist)

    // index of the vertex in the meshlet, -1 if the vertex isn't used
    let used = allocator.allocate<int16>(vertex_count)
    clearUsed used vertex_count indices index_count

    let boundary = allocator.allocate<byte>(face_count)

    bvhSplit boxes axes (NPtr.add axes face_count) (NPtr.add axes (face_count * 2)) boundary face_count 0 scratch used indices max_vertices min_triangles max_triangles fill_weight

    // compute the desired number of meshlets; note that on some meshes with a lot of vertex bound clusters this might go over the bound
    let mutable meshlet_count = 0
    for i = 0 to face_count - 1 do
        assert (NPtr.get boundary i <= 1uy)
        meshlet_count <- meshlet_count + int (NPtr.get boundary i)

    let meshlet_bound = meshopt_buildMeshletsBound index_count max_vertices min_triangles

    // pack triangles into meshlets according to the order and boundaries marked by bvhSplit
    let mutable meshlet = meshopt_Meshlet()
    let mutable meshlet_offset = 0
    let mutable meshlet_pending = meshlet_count

    for i = 0 to face_count - 1 do
        assert (NPtr.get boundary i <= 1uy)
        let mutable split = i > 0 && NPtr.get boundary i = 1uy

        // while we are over the limit, we ignore boundary[] data and disable splits until we free up enough space
        if split && meshlet_count > meshlet_bound && meshlet_offset + meshlet_pending >= meshlet_bound then
            split <- false

        let index = int (NPtr.get axes i)
        assert (index < face_count)

        let a = NPtr.get indices (index * 3 + 0)
        let b = NPtr.get indices (index * 3 + 1)
        let c = NPtr.get indices (index * 3 + 2)

        // appends triangle to the meshlet and writes previous meshlet to the output if full
        if appendMeshlet &meshlet a b c used meshlets meshlet_vertices meshlet_triangles meshlet_offset max_vertices max_triangles split then
            meshlet_offset <- meshlet_offset + 1
        meshlet_pending <- meshlet_pending - int (NPtr.get boundary i)

    if meshlet.triangle_count <> 0u then
        NPtr.set meshlets meshlet_offset meshlet
        meshlet_offset <- meshlet_offset + 1

    assert (meshlet_offset <= meshlet_bound)
    assert (int meshlet.triangle_offset + int meshlet.triangle_count * 3 <= index_count && int meshlet.vertex_offset + int meshlet.vertex_count <= index_count)
    meshlet_offset

let meshopt_computeClusterBounds (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) : meshopt_Bounds =
    assert (index_count % 3 = 0)
    assert (index_count / 3 <= kMeshletMaxTriangles)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    ignore vertex_count

    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    // compute triangle normals and gather triangle corners
    let normals = Array.zeroCreate<float32> (kMeshletMaxTriangles * 3)
    let corners = Array.zeroCreate<float32> (kMeshletMaxTriangles * 3 * 3)
    let mutable triangleCount = 0

    let mutable i = 0
    while i < index_count do
        let a = int (NPtr.get indices (i + 0))
        let b = int (NPtr.get indices (i + 1))
        let c = int (NPtr.get indices (i + 2))
        assert (a < vertex_count && b < vertex_count && c < vertex_count)

        let p0 = NPtr.add vertex_positions (vertex_stride_float * a)
        let p1 = NPtr.add vertex_positions (vertex_stride_float * b)
        let p2 = NPtr.add vertex_positions (vertex_stride_float * c)

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

        // no need to include degenerate triangles - they will be invisible anyway
        if area <> 0.0f then
            // record triangle normals & corners for future use
            normals.[triangleCount * 3 + 0] <- normalx / area
            normals.[triangleCount * 3 + 1] <- normaly / area
            normals.[triangleCount * 3 + 2] <- normalz / area

            corners.[triangleCount * 9 + 0] <- NPtr.get p0 0
            corners.[triangleCount * 9 + 1] <- NPtr.get p0 1
            corners.[triangleCount * 9 + 2] <- NPtr.get p0 2
            corners.[triangleCount * 9 + 3] <- NPtr.get p1 0
            corners.[triangleCount * 9 + 4] <- NPtr.get p1 1
            corners.[triangleCount * 9 + 5] <- NPtr.get p1 2
            corners.[triangleCount * 9 + 6] <- NPtr.get p2 0
            corners.[triangleCount * 9 + 7] <- NPtr.get p2 1
            corners.[triangleCount * 9 + 8] <- NPtr.get p2 2
            triangleCount <- triangleCount + 1

        i <- i + 3

    let mutable bounds = meshopt_Bounds()

    // degenerate cluster, no valid triangles => trivial reject (cone data is 0)
    if triangleCount = 0 then
        bounds
    else

    let rzero = 0.0f

    // compute cluster bounding sphere; we'll use the center to determine normal cone apex as well
    let psphere = NativePtr.stackalloc<float32> 4
    NPtr.memset psphere 0uy (4 * sizeof<float32>)

    use pinnedCorners = fixed corners
    let cornersPtr = pinnedCorners
    use pinnedRzero = fixed &rzero
    let rzeroPtr : nativeptr<float32> = pinnedRzero

    computeBoundingSphere psphere cornersPtr (triangleCount * 3) (sizeof<float32> * 3) rzeroPtr 0 7

    let center_0 = NPtr.get psphere 0
    let center_1 = NPtr.get psphere 1
    let center_2 = NPtr.get psphere 2

    // treating triangle normals as points, find the bounding sphere - the sphere center determines the optimal cone axis
    let nsphere = NativePtr.stackalloc<float32> 4
    NPtr.memset nsphere 0uy (4 * sizeof<float32>)

    use pinnedNormals = fixed normals
    let normalsPtr = pinnedNormals

    computeBoundingSphere nsphere normalsPtr triangleCount (sizeof<float32> * 3) rzeroPtr 0 3

    let mutable axis_0 = NPtr.get nsphere 0
    let mutable axis_1 = NPtr.get nsphere 1
    let mutable axis_2 = NPtr.get nsphere 2
    let axislength = MathF.Sqrt(axis_0 * axis_0 + axis_1 * axis_1 + axis_2 * axis_2)
    let invaxislength = if axislength = 0.0f then 0.0f else 1.0f / axislength

    axis_0 <- axis_0 * invaxislength
    axis_1 <- axis_1 * invaxislength
    axis_2 <- axis_2 * invaxislength

    // compute a tight cone around all normals, mindp = cos(angle/2)
    let mutable mindp = 1.0f

    for i = 0 to triangleCount - 1 do
        let dp = normals.[i * 3 + 0] * axis_0 + normals.[i * 3 + 1] * axis_1 + normals.[i * 3 + 2] * axis_2
        mindp <- if dp < mindp then dp else mindp

    // fill bounding sphere info; note that below we can return bounds without cone information for degenerate cones
    bounds.center_0 <- center_0
    bounds.center_1 <- center_1
    bounds.center_2 <- center_2
    bounds.radius <- NPtr.get psphere 3

    // degenerate cluster, normal cone is larger than a hemisphere => trivial accept
    // note that if mindp is positive but close to 0, the triangle intersection code below gets less stable
    // we arbitrarily decide that if a normal cone is ~168 degrees wide or more, the cone isn't useful
    if mindp <= 0.1f then
        bounds.cone_cutoff <- 1.0f
        bounds.cone_cutoff_s8 <- 127y
        bounds
    else

    let mutable maxt = 0.0f

    // we need to find the point on center-t*axis ray that lies in negative half-space of all triangles
    for i = 0 to triangleCount - 1 do
        // dot(center-t*axis-corner, trinormal) = 0
        // dot(center-corner, trinormal) - t * dot(axis, trinormal) = 0
        let cx = center_0 - corners.[i * 9 + 0]
        let cy = center_1 - corners.[i * 9 + 1]
        let cz = center_2 - corners.[i * 9 + 2]

        let dc = cx * normals.[i * 3 + 0] + cy * normals.[i * 3 + 1] + cz * normals.[i * 3 + 2]
        let dn = axis_0 * normals.[i * 3 + 0] + axis_1 * normals.[i * 3 + 1] + axis_2 * normals.[i * 3 + 2]

        // dn should be larger than mindp cutoff above
        assert (dn > 0.0f)
        let t = dc / dn

        maxt <- if t > maxt then t else maxt

    // cone apex should be in the negative half-space of all cluster triangles by construction
    bounds.cone_apex_0 <- center_0 - axis_0 * maxt
    bounds.cone_apex_1 <- center_1 - axis_1 * maxt
    bounds.cone_apex_2 <- center_2 - axis_2 * maxt

    // note: this axis is the axis of the normal cone, but our test for perspective camera effectively negates the axis
    bounds.cone_axis_0 <- axis_0
    bounds.cone_axis_1 <- axis_1
    bounds.cone_axis_2 <- axis_2

    // cos(a) for normal cone is mindp; we need to add 90 degrees on both sides and invert the cone
    // which gives us -cos(a+90) = -(-sin(a)) = sin(a) = sqrt(1 - cos^2(a))
    bounds.cone_cutoff <- MathF.Sqrt(1.0f - mindp * mindp)

    // quantize axis & cutoff to 8-bit SNORM format
    bounds.cone_axis_s8_0 <- sbyte (Quantization.meshopt_quantizeSnorm bounds.cone_axis_0 8)
    bounds.cone_axis_s8_1 <- sbyte (Quantization.meshopt_quantizeSnorm bounds.cone_axis_1 8)
    bounds.cone_axis_s8_2 <- sbyte (Quantization.meshopt_quantizeSnorm bounds.cone_axis_2 8)

    // for the 8-bit test to be conservative, we need to adjust the cutoff by measuring the max. error
    let cone_axis_s8_e0 = MathF.Abs(float32 bounds.cone_axis_s8_0 / 127.0f - bounds.cone_axis_0)
    let cone_axis_s8_e1 = MathF.Abs(float32 bounds.cone_axis_s8_1 / 127.0f - bounds.cone_axis_1)
    let cone_axis_s8_e2 = MathF.Abs(float32 bounds.cone_axis_s8_2 / 127.0f - bounds.cone_axis_2)

    // note that we need to round this up instead of rounding to nearest, hence +1
    let cone_cutoff_s8 = int (127.0f * (bounds.cone_cutoff + cone_axis_s8_e0 + cone_axis_s8_e1 + cone_axis_s8_e2) + 1.0f)

    bounds.cone_cutoff_s8 <- if cone_cutoff_s8 > 127 then 127y else sbyte cone_cutoff_s8

    bounds

let meshopt_computeMeshletBounds (meshlet_vertices: nativeptr<uint32>) (meshlet_triangles: nativeptr<byte>) (triangle_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) : meshopt_Bounds =
    assert (triangle_count <= kMeshletMaxTriangles)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    let indices = Array.zeroCreate<uint32> (kMeshletMaxTriangles * 3)

    for i = 0 to triangle_count * 3 - 1 do
        let index = NPtr.get meshlet_vertices (int (NPtr.get meshlet_triangles i))
        assert (int index < vertex_count)
        indices.[i] <- index

    use pinnedIndices = fixed indices
    meshopt_computeClusterBounds pinnedIndices (triangle_count * 3) vertex_positions vertex_count vertex_positions_stride

let meshopt_computeSphereBounds (positions: nativeptr<float32>) (count: int) (positions_stride: int) (radii: nativeptr<float32>) (radii_stride: int) : meshopt_Bounds =
    assert (positions_stride >= 12 && positions_stride <= 256)
    assert (positions_stride % sizeof<float32> = 0)
    assert ((radii_stride >= 4 && radii_stride <= 256) || NPtr.toNI radii = 0n)
    assert (radii_stride % sizeof<float32> = 0)

    let mutable bounds = meshopt_Bounds()

    if count = 0 then
        bounds
    else

    let rzero = 0.0f
    let psphere = NativePtr.stackalloc<float32> 4
    NPtr.memset psphere 0uy (4 * sizeof<float32>)

    use pinnedRzero = fixed &rzero
    let rzeroPtr : nativeptr<float32> = pinnedRzero

    let actualRadii = if NPtr.toNI radii <> 0n then radii else rzeroPtr
    let actualRadiiStride = if NPtr.toNI radii <> 0n then radii_stride else 0

    computeBoundingSphere psphere positions count positions_stride actualRadii actualRadiiStride 7

    bounds.center_0 <- NPtr.get psphere 0
    bounds.center_1 <- NPtr.get psphere 1
    bounds.center_2 <- NPtr.get psphere 2
    bounds.radius <- NPtr.get psphere 3

    bounds

let meshopt_optimizeMeshlet (meshlet_vertices: nativeptr<uint32>) (meshlet_triangles: nativeptr<byte>) (triangle_count: int) (vertex_count: int) =
    assert (triangle_count <= kMeshletMaxTriangles)
    assert (vertex_count <= kMeshletMaxVertices)

    let indices = meshlet_triangles
    let vertices = meshlet_vertices

    // cache tracks vertex timestamps (corresponding to triangle index! all 3 vertices are added at the same time and never removed)
    let cache = Array.zeroCreate<byte> kMeshletMaxVertices

    // note that we start from a value that means all vertices aren't in cache
    let mutable cache_last = 128uy
    let cache_cutoff = 3uy // 3 triangles = ~5..9 vertices depending on reuse

    for i = 0 to triangle_count - 1 do
        let mutable next = -1
        let mutable next_match = -1

        let mutable j = i
        while j < triangle_count do
            let a = int (NPtr.get indices (j * 3 + 0))
            let b = int (NPtr.get indices (j * 3 + 1))
            let c = int (NPtr.get indices (j * 3 + 2))
            assert (a < vertex_count && b < vertex_count && c < vertex_count)

            // score each triangle by how many vertices are in cache
            // note: the distance is computed using unsigned 8-bit values, so cache timestamp overflow is handled gracefully
            let aok = if (cache_last - cache.[a]) < cache_cutoff then 1 else 0
            let bok = if (cache_last - cache.[b]) < cache_cutoff then 1 else 0
            let cok = if (cache_last - cache.[c]) < cache_cutoff then 1 else 0

            if aok + bok + cok > next_match then
                next <- j
                next_match <- aok + bok + cok

                // note that we could end up with all 3 vertices in the cache, but 2 is enough for ~strip traversal
                if next_match >= 2 then
                    j <- triangle_count // break

            j <- j + 1

        assert (next >= 0)

        let a = NPtr.get indices (next * 3 + 0)
        let b = NPtr.get indices (next * 3 + 1)
        let c = NPtr.get indices (next * 3 + 2)

        // shift triangles before the next one forward so that we always keep an ordered partition
        // note: this could have swapped triangles [i] and [next] but that distorts the order and may skew the output sequence
        // memmove(indices + (i + 1) * 3, indices + i * 3, (next - i) * 3 * sizeof(unsigned char))
        let src = NPtr.add indices (i * 3)
        let dst = NPtr.add indices ((i + 1) * 3)
        let moveBytes = (next - i) * 3
        // use Buffer.MemoryCopy for overlapping memmove
        System.Buffer.MemoryCopy(NPtr.toVoid src, NPtr.toVoid dst, int64 moveBytes, int64 moveBytes)

        NPtr.set indices (i * 3 + 0) a
        NPtr.set indices (i * 3 + 1) b
        NPtr.set indices (i * 3 + 2) c

        // cache timestamp is the same between all vertices of each triangle to reduce overflow
        cache_last <- cache_last + 1uy
        cache.[int a] <- cache_last
        cache.[int b] <- cache_last
        cache.[int c] <- cache_last

    // rotate triangles to maximize compressibility
    Array.Clear(cache, 0, vertex_count)

    for i = 0 to triangle_count - 1 do
        let mutable a = NPtr.get indices (i * 3 + 0)
        let mutable b = NPtr.get indices (i * 3 + 1)
        let mutable c = NPtr.get indices (i * 3 + 2)

        // if only the middle vertex has been used, rotate triangle to ensure new vertices are always sequential
        if cache.[int a] = 0uy && cache.[int b] <> 0uy && cache.[int c] = 0uy then
            // abc -> bca
            let t = a
            a <- b
            b <- c
            c <- t
        elif cache.[int a] = 0uy && cache.[int b] = 0uy && cache.[int c] = 0uy then
            // out of three edges, the edge ab can not be reused by subsequent triangles in some encodings
            // if subsequent triangles don't share edges ca or bc, we can rotate the triangle to fix this
            let mutable needab = false
            let mutable needbc = false
            let mutable needca = false

            let mutable j = i + 1
            while j < triangle_count && j <= i + int cache_cutoff do
                let oa = NPtr.get indices (j * 3 + 0)
                let ob = NPtr.get indices (j * 3 + 1)
                let oc = NPtr.get indices (j * 3 + 2)

                // note: edge comparisons are reversed as reused edges are flipped
                needab <- needab || (oa = b && ob = a) || (ob = b && oc = a) || (oc = b && oa = a)
                needbc <- needbc || (oa = c && ob = b) || (ob = c && oc = b) || (oc = c && oa = b)
                needca <- needca || (oa = a && ob = c) || (ob = a && oc = c) || (oc = a && oa = c)
                j <- j + 1

            if needab && not needbc then
                // abc -> bca
                let t = a
                a <- b
                b <- c
                c <- t
            elif needab && not needca then
                // abc -> cab
                let t = c
                c <- b
                b <- a
                a <- t

        NPtr.set indices (i * 3 + 0) a
        NPtr.set indices (i * 3 + 1) b
        NPtr.set indices (i * 3 + 2) c

        cache.[int a] <- 1uy
        cache.[int b] <- 1uy
        cache.[int c] <- 1uy

    // reorder meshlet vertices for access locality assuming index buffer is scanned sequentially
    let order = Array.zeroCreate<uint32> kMeshletMaxVertices

    let remap = Array.create kMeshletMaxVertices -1s

    let mutable vertex_offset = 0

    for i = 0 to triangle_count * 3 - 1 do
        let idx = int (NPtr.get indices i)

        if remap.[idx] < 0s then
            remap.[idx] <- int16 vertex_offset
            order.[vertex_offset] <- NPtr.get vertices idx
            vertex_offset <- vertex_offset + 1

        NPtr.set indices i (byte remap.[idx])

    assert (vertex_offset <= vertex_count)

    use pinnedOrder = fixed order
    NPtr.memcpy vertices pinnedOrder (vertex_offset * sizeof<uint32>)
