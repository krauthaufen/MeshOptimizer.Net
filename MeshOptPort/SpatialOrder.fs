// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
module MeshOptPort.SpatialOrder

// This work is based on:
// Fabian Giesen. Decoding Morton codes. 2009

open System
open MeshOptPort.Allocator

// "Insert" two 0 bits after each of the 20 low bits of x
let private part1By2 (x: uint64) : uint64 =
    let mutable x = x &&& 0x000fffffUL
    x <- (x ^^^ (x <<< 32)) &&& 0x000f00000000ffffUL
    x <- (x ^^^ (x <<< 16)) &&& 0x000f0000ff0000ffUL
    x <- (x ^^^ (x <<< 8))  &&& 0x000f00f00f00f00fUL
    x <- (x ^^^ (x <<< 4))  &&& 0x00c30c30c30c30c3UL
    x <- (x ^^^ (x <<< 2))  &&& 0x0249249249249249UL
    x

let private computeOrder (result: nativeptr<uint64>) (vertex_positions_data: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (morton: bool) =
    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    let mutable minv0 = Single.MaxValue
    let mutable minv1 = Single.MaxValue
    let mutable minv2 = Single.MaxValue
    let mutable maxv0 = -Single.MaxValue
    let mutable maxv1 = -Single.MaxValue
    let mutable maxv2 = -Single.MaxValue

    for i = 0 to vertex_count - 1 do
        let vbase = i * vertex_stride_float
        let vj0 = NPtr.get vertex_positions_data (vbase + 0)
        let vj1 = NPtr.get vertex_positions_data (vbase + 1)
        let vj2 = NPtr.get vertex_positions_data (vbase + 2)

        minv0 <- if minv0 > vj0 then vj0 else minv0
        maxv0 <- if maxv0 < vj0 then vj0 else maxv0
        minv1 <- if minv1 > vj1 then vj1 else minv1
        maxv1 <- if maxv1 < vj1 then vj1 else maxv1
        minv2 <- if minv2 > vj2 then vj2 else minv2
        maxv2 <- if maxv2 < vj2 then vj2 else maxv2

    let mutable extent = 0.0f
    extent <- if (maxv0 - minv0) > extent then (maxv0 - minv0) else extent
    extent <- if (maxv1 - minv1) > extent then (maxv1 - minv1) else extent
    extent <- if (maxv2 - minv2) > extent then (maxv2 - minv2) else extent

    // rescale each axis to 16 bits to get 48-bit Morton codes
    let scale = if extent = 0.0f then 0.0f else 65535.0f / extent

    // generate Morton order based on the position inside a unit cube
    for i = 0 to vertex_count - 1 do
        let vbase = i * vertex_stride_float
        let v0 = NPtr.get vertex_positions_data (vbase + 0)
        let v1 = NPtr.get vertex_positions_data (vbase + 1)
        let v2 = NPtr.get vertex_positions_data (vbase + 2)

        let x = int ((v0 - minv0) * scale + 0.5f)
        let y = int ((v1 - minv1) * scale + 0.5f)
        let z = int ((v2 - minv2) * scale + 0.5f)

        if morton then
            NPtr.set result i (part1By2 (uint64 x) ||| (part1By2 (uint64 y) <<< 1) ||| (part1By2 (uint64 z) <<< 2))
        else
            NPtr.set result i ((uint64 x <<< 0) ||| (uint64 y <<< 20) ||| (uint64 z <<< 40))

let private radixSort10 (destination: nativeptr<uint32>) (source: nativeptr<uint32>) (keys: nativeptr<uint16>) (count: int) =
    let hist = Array.zeroCreate<uint32> 1024

    // compute histogram (assume keys are 10-bit)
    for i = 0 to count - 1 do
        let k = int (NPtr.get keys i)
        hist.[k] <- hist.[k] + 1u

    let mutable sum = 0u

    // replace histogram data with prefix histogram sums in-place
    for i = 0 to 1023 do
        let h = hist.[i]
        hist.[i] <- sum
        sum <- sum + h

    assert (int sum = count)

    // reorder values
    for i = 0 to count - 1 do
        let id = int (NPtr.get keys (int (NPtr.get source i)))
        let pos = int hist.[id]
        NPtr.set destination pos (NPtr.get source i)
        hist.[id] <- hist.[id] + 1u

// Flattened hist[256][2] -> hist[512] with indexing [i*2+pass]
let private computeHistogram (hist: uint32[]) (data: nativeptr<uint16>) (count: int) =
    Array.Clear(hist, 0, 512)

    // compute 2 8-bit histograms in parallel
    for i = 0 to count - 1 do
        let id = uint64 (NPtr.get data i)
        let idx0 = int ((id >>> 0) &&& 255UL)
        let idx1 = int ((id >>> 8) &&& 255UL)
        hist.[idx0 * 2 + 0] <- hist.[idx0 * 2 + 0] + 1u
        hist.[idx1 * 2 + 1] <- hist.[idx1 * 2 + 1] + 1u

    let mutable sum0 = 0u
    let mutable sum1 = 0u

    // replace histogram data with prefix histogram sums in-place
    for i = 0 to 255 do
        let h0 = hist.[i * 2 + 0]
        let h1 = hist.[i * 2 + 1]

        hist.[i * 2 + 0] <- sum0
        hist.[i * 2 + 1] <- sum1

        sum0 <- sum0 + h0
        sum1 <- sum1 + h1

    assert (int sum0 = count && int sum1 = count)

let private radixPass (destination: nativeptr<uint32>) (source: nativeptr<uint32>) (keys: nativeptr<uint16>) (count: int) (hist: uint32[]) (pass: int) =
    let bitoff = pass * 8

    for i = 0 to count - 1 do
        let id = int ((uint32 (NPtr.get keys (int (NPtr.get source i))) >>> bitoff) &&& 255u)
        let pos = int hist.[id * 2 + pass]
        NPtr.set destination pos (NPtr.get source i)
        hist.[id * 2 + pass] <- hist.[id * 2 + pass] + 1u

let private partitionPoints (target: nativeptr<uint32>) (order: nativeptr<uint32>) (sides: nativeptr<byte>) (split: int) (count: int) =
    let mutable l = 0
    let mutable r = split

    for i = 0 to count - 1 do
        let side = int (NPtr.get sides (int (NPtr.get order i)))
        NPtr.set target (if side <> 0 then r else l) (NPtr.get order i)
        l <- l + 1
        l <- l - side
        r <- r + side

    assert (l = split && r = count)

let rec private splitPoints (destination: nativeptr<uint32>) (orderx: nativeptr<uint32>) (ordery: nativeptr<uint32>) (orderz: nativeptr<uint32>) (keys: nativeptr<uint64>) (count: int) (scratch: nativeptr<uint32>) (cluster_size: int) =
    if count <= cluster_size then
        NPtr.memcpy destination orderx (count * sizeof<uint32>)
    else
        let axes = [| orderx; ordery; orderz |]

        let mutable bestk = -1
        let mutable bestdim = 0u

        for k = 0 to 2 do
            let mask = (1u <<< 20) - 1u
            let lastVal = uint32 (NPtr.get keys (int (NPtr.get axes.[k] (count - 1))) >>> (k * 20)) &&& mask
            let firstVal = uint32 (NPtr.get keys (int (NPtr.get axes.[k] 0)) >>> (k * 20)) &&& mask
            let dim = lastVal - firstVal

            if dim >= bestdim then
                bestk <- k
                bestdim <- dim

        assert (bestk >= 0)

        // split roughly in half, with the left split always being aligned to cluster size
        let split = ((count / 2) + cluster_size - 1) / cluster_size * cluster_size
        assert (split > 0 && split < count)

        // mark sides of split for partitioning
        let sides : nativeptr<byte> = NPtr.cast (NPtr.add scratch count)

        for i = 0 to split - 1 do
            NPtr.set sides (int (NPtr.get axes.[bestk] i)) 0uy

        for i = split to count - 1 do
            NPtr.set sides (int (NPtr.get axes.[bestk] i)) 1uy

        // partition all axes into two sides, maintaining order
        let temp = scratch

        for k = 0 to 2 do
            if k <> bestk then
                let axis = axes.[k]
                NPtr.memcpy temp axis (count * sizeof<uint32>)
                partitionPoints axis temp sides split count

        // recursion depth is logarithmic and bounded as we always split in approximately half
        splitPoints destination orderx ordery orderz keys split scratch cluster_size
        splitPoints (NPtr.add destination split) (NPtr.add orderx split) (NPtr.add ordery split) (NPtr.add orderz split) keys (count - split) scratch cluster_size

let meshopt_spatialSortRemap (destination: nativeptr<uint32>) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) =
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    use allocator = new meshopt_Allocator()

    let keys = allocator.allocate<uint64>(vertex_count)
    computeOrder keys vertex_positions vertex_count vertex_positions_stride true

    let scratch = allocator.allocate<uint32>(vertex_count * 2) // 4b for order + 2b for keys
    let keyk : nativeptr<uint16> = NPtr.cast (NPtr.add scratch vertex_count)

    for i = 0 to vertex_count - 1 do
        NPtr.set destination i (uint32 i)

    let order = [| scratch; destination |]

    // 5-pass radix sort computes the resulting order into scratch
    for k = 0 to 4 do
        // copy 10-bit key segments into keyk to reduce cache pressure during radix pass
        for i = 0 to vertex_count - 1 do
            NPtr.set keyk i (uint16 ((NPtr.get keys i >>> (k * 10)) &&& 1023UL))

        radixSort10 order.[k % 2] order.[(k + 1) % 2] keyk vertex_count

    // since our remap table is mapping old=>new, we need to reverse it
    for i = 0 to vertex_count - 1 do
        NPtr.set destination (int (NPtr.get scratch i)) (uint32 i)

let meshopt_spatialSortTriangles (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    let face_count = index_count / 3
    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    use allocator = new meshopt_Allocator()

    let centroids = allocator.allocate<float32>(face_count * 3)

    for i = 0 to face_count - 1 do
        let a = int (NPtr.get indices (i * 3 + 0))
        let b = int (NPtr.get indices (i * 3 + 1))
        let c = int (NPtr.get indices (i * 3 + 2))
        assert (a < vertex_count && b < vertex_count && c < vertex_count)

        let va = NPtr.add vertex_positions (a * vertex_stride_float)
        let vb = NPtr.add vertex_positions (b * vertex_stride_float)
        let vc = NPtr.add vertex_positions (c * vertex_stride_float)

        NPtr.set centroids (i * 3 + 0) ((NPtr.get va 0 + NPtr.get vb 0 + NPtr.get vc 0) / 3.0f)
        NPtr.set centroids (i * 3 + 1) ((NPtr.get va 1 + NPtr.get vb 1 + NPtr.get vc 1) / 3.0f)
        NPtr.set centroids (i * 3 + 2) ((NPtr.get va 2 + NPtr.get vb 2 + NPtr.get vc 2) / 3.0f)

    let remap = allocator.allocate<uint32>(face_count)

    meshopt_spatialSortRemap remap centroids face_count (sizeof<float32> * 3)

    // support in-order remap
    let mutable indices = indices
    if NPtr.toNI destination = NPtr.toNI indices then
        let indices_copy = allocator.allocate<uint32>(index_count)
        NPtr.memcpy indices_copy indices (index_count * sizeof<uint32>)
        indices <- indices_copy

    for i = 0 to face_count - 1 do
        let a = NPtr.get indices (i * 3 + 0)
        let b = NPtr.get indices (i * 3 + 1)
        let c = NPtr.get indices (i * 3 + 2)
        let r = int (NPtr.get remap i)

        NPtr.set destination (r * 3 + 0) a
        NPtr.set destination (r * 3 + 1) b
        NPtr.set destination (r * 3 + 2) c

let meshopt_spatialClusterPoints (destination: nativeptr<uint32>) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (cluster_size: int) =
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)
    assert (cluster_size > 0)

    use allocator = new meshopt_Allocator()

    let keys = allocator.allocate<uint64>(vertex_count)
    computeOrder keys vertex_positions vertex_count vertex_positions_stride false

    let order = allocator.allocate<uint32>(vertex_count * 3)
    let scratch = allocator.allocate<uint32>(vertex_count * 2) // 4b for order + 1b for side or 2b for keys
    let keyk : nativeptr<uint16> = NPtr.cast (NPtr.add scratch vertex_count)

    for k = 0 to 2 do
        // copy 16-bit key segments into keyk to reduce cache pressure during radix pass
        for i = 0 to vertex_count - 1 do
            NPtr.set keyk i (uint16 (NPtr.get keys i >>> (k * 20)))

        let hist = Array.zeroCreate<uint32> 512
        computeHistogram hist keyk vertex_count

        for i = 0 to vertex_count - 1 do
            NPtr.set (NPtr.add order (k * vertex_count)) i (uint32 i)

        radixPass scratch (NPtr.add order (k * vertex_count)) keyk vertex_count hist 0
        radixPass (NPtr.add order (k * vertex_count)) scratch keyk vertex_count hist 1

    splitPoints destination order (NPtr.add order vertex_count) (NPtr.add order (2 * vertex_count)) keys vertex_count scratch cluster_size
