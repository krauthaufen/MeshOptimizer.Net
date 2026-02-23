// This file is part of MeshOptimizerDotNet; see meshoptimizer.h for version/license details
module MeshOptimizerDotNet.MeshletCodec

#nowarn "9"

open System
open System.Runtime.InteropServices
open System.Runtime.Intrinsics
open System.Runtime.Intrinsics.X86
open System.Runtime.Intrinsics.Arm
open MeshOptimizerDotNet
open MeshOptimizerDotNet.Allocator

// EdgeFifo8 is uint32[8][2] stored as flat uint32[16] with indexing [i*2+0] and [i*2+1]
let private rotateTriangle (a: uint32) (b: uint32) (c: uint32) : int =
    if a > b && a > c then 1
    elif b > c then 2
    else 0

let private getEdgeFifo8 (fifo: uint32[]) (a: uint32) (b: uint32) (c: uint32) (offset: int) : int =
    let mutable result = -1
    let mutable i = 0
    while i < 8 && result = -1 do
        let index = (offset - 1 - i) &&& 7

        let e0 = fifo.[index * 2 + 0]
        let e1 = fifo.[index * 2 + 1]

        if e0 = a && e1 = b then
            result <- (i <<< 2) ||| 0
        elif e0 = b && e1 = c then
            result <- (i <<< 2) ||| 1
        elif e0 = c && e1 = a then
            result <- (i <<< 2) ||| 2

        i <- i + 1

    result

let private pushEdgeFifo8 (fifo: uint32[]) (a: uint32) (b: uint32) (offset: byref<int>) =
    fifo.[offset * 2 + 0] <- a
    fifo.[offset * 2 + 1] <- b
    offset <- (offset + 1) &&& 7

let private encodeTriangles (codes: nativeptr<byte>) (extra: nativeptr<byte>) (triangles: nativeptr<byte>) (triangle_count: int) : int =
    let edgefifo = Array.create 16 0xFFFFFFFFu  // 8 entries * 2
    let mutable edgefifooffset = 0
    let mutable next = 0u

    // 4-bit triangle codes give us 16 options
    NPtr.memset codes 0uy ((triangle_count + 1) / 2)

    let rotations = [| 0; 1; 2; 0; 1 |]

    let mutable extra_offset = 0

    for i = 0 to triangle_count - 1 do
        let fer = getEdgeFifo8 edgefifo (uint32 (NPtr.get triangles (i * 3 + 0))) (uint32 (NPtr.get triangles (i * 3 + 1))) (uint32 (NPtr.get triangles (i * 3 + 2))) edgefifooffset

        if fer >= 0 && (fer >>> 2) < 6 then
            // note: getEdgeFifo8 implicitly rotates triangles by matching a/b to existing edge
            let orderBase = fer &&& 3

            let a = uint32 (NPtr.get triangles (i * 3 + rotations.[orderBase + 0]))
            let b = uint32 (NPtr.get triangles (i * 3 + rotations.[orderBase + 1]))
            let c = uint32 (NPtr.get triangles (i * 3 + rotations.[orderBase + 2]))

            let fec =
                if c = next then
                    next <- next + 1u
                    0
                else 1

            let code = uint32 ((fer >>> 2) * 2 + fec)

            NPtr.set codes (i / 2) (NPtr.get codes (i / 2) ||| byte (code <<< ((i &&& 1) * 4)))

            if fec <> 0 then
                NPtr.set extra extra_offset (byte c)
                extra_offset <- extra_offset + 1

            pushEdgeFifo8 edgefifo c b &edgefifooffset
            pushEdgeFifo8 edgefifo a c &edgefifooffset
        else
            // rotate triangles to minimize the need for extra vertices
            let rotation = rotateTriangle (uint32 (NPtr.get triangles (i * 3 + 0))) (uint32 (NPtr.get triangles (i * 3 + 1))) (uint32 (NPtr.get triangles (i * 3 + 2)))
            let orderBase = rotation

            let a = uint32 (NPtr.get triangles (i * 3 + rotations.[orderBase + 0]))
            let b = uint32 (NPtr.get triangles (i * 3 + rotations.[orderBase + 1]))
            let c = uint32 (NPtr.get triangles (i * 3 + rotations.[orderBase + 2]))

            // fe must be continuous
            let fea =
                if a = next && b = next + 1u && c = next + 2u then
                    next <- next + 1u
                    0
                else 1
            let feb =
                if b = next && c = next + 1u then
                    next <- next + 1u
                    0
                else 1
            let fec =
                if c = next then
                    next <- next + 1u
                    0
                else 1

            assert (fea = 1 || feb = 0)
            assert (feb = 1 || fec = 0)

            let code = uint32 (12 + (fea + feb + fec))

            NPtr.set codes (i / 2) (NPtr.get codes (i / 2) ||| byte (code <<< ((i &&& 1) * 4)))

            if fea <> 0 then
                NPtr.set extra extra_offset (byte a)
                extra_offset <- extra_offset + 1
            if feb <> 0 then
                NPtr.set extra extra_offset (byte b)
                extra_offset <- extra_offset + 1
            if fec <> 0 then
                NPtr.set extra extra_offset (byte c)
                extra_offset <- extra_offset + 1

            pushEdgeFifo8 edgefifo c b &edgefifooffset
            pushEdgeFifo8 edgefifo a c &edgefifooffset

    extra_offset

let private encodeVertices (ctrl: nativeptr<byte>) (data: nativeptr<byte>) (vertices: nativeptr<uint32>) (vertex_count: int) : int =
    // grouped varint, 2 bit per value to indicate 0/1/2/3 byte deltas, with per-group 4-byte fallback
    NPtr.memset ctrl 0uy ((vertex_count + 3) / 4)

    let mutable data_offset = 0
    let mutable last = 0xFFFFFFFFu

    let mutable i = 0
    while i < vertex_count do
        let gv = Array.zeroCreate<uint32> 4

        let mutable k = 0
        while k < 4 && i + k < vertex_count do
            let d = NPtr.get vertices (i + k) - last - 1u
            let v = (d <<< 1) ^^^ (uint32 (int d >>> 31))

            gv.[k] <- v
            last <- NPtr.get vertices (i + k)
            k <- k + 1

        // if any value needs 4 bytes, or if *all* values need 3 bytes, we use 4 bytes for all values
        let use4 =
            (gv.[0] ||| gv.[1] ||| gv.[2] ||| gv.[3]) > 0xffffffu ||
            (gv.[0] > 0xffffu && gv.[1] > 0xffffu && gv.[2] > 0xffffu && gv.[3] > 0xffffu)

        for k = 0 to 3 do
            let v = gv.[k]

            let code =
                if use4 then 3
                elif v = 0u then 0
                elif v < 256u then 1
                elif v < 65536u then 2
                else 3

            if code > 0 then
                NPtr.set data data_offset (byte (v &&& 0xffu))
                data_offset <- data_offset + 1
            if code > 1 then
                NPtr.set data data_offset (byte ((v >>> 8) &&& 0xffu))
                data_offset <- data_offset + 1
            if code > 2 then
                NPtr.set data data_offset (byte ((v >>> 16) &&& 0xffu))
                data_offset <- data_offset + 1
            if use4 then
                NPtr.set data data_offset (byte ((v >>> 24) &&& 0xffu))
                data_offset <- data_offset + 1

            // split low and high bits into two nibbles for better packing
            NPtr.set ctrl (i / 4) (NPtr.get ctrl (i / 4) ||| byte (((code &&& 1) <<< k) ||| ((code >>> 1) <<< (k + 4))))

        i <- i + 4

    data_offset

// Scalar writeTriangle overloads
let inline private writeTriangleUInt (triangles: nativeptr<uint32>) (i: int) (fifo: uint32) =
    // output triangle is stored without extra edge vertex (0xcbac => 0xcba)
    NPtr.set triangles i (fifo >>> 8)

let inline private writeTriangleByte (triangles: nativeptr<byte>) (i: int) (fifo: uint32) =
    NPtr.set triangles (i * 3 + 0) (byte (fifo >>> 8))
    NPtr.set triangles (i * 3 + 1) (byte (fifo >>> 16))
    NPtr.set triangles (i * 3 + 2) (byte (fifo >>> 24))

// Scalar decodeTriangles for uint32 output
let private decodeTrianglesUInt (triangles: nativeptr<uint32>) (codes: nativeptr<byte>) (extra: nativeptr<byte>) (bound: nativeptr<byte>) (triangle_count: int) : nativeptr<byte> =
    let mutable next = 0u
    let fifo = Array.zeroCreate<uint32> 3  // two edge fifo entries in one uint: 0xcbac
    let mutable extra_offset = 0
    let bound_offset = int (NPtr.toNI bound - NPtr.toNI extra)
    let mutable failed = false

    let mutable i = 0
    while i < triangle_count && not failed do
        if extra_offset > bound_offset then
            failed <- true
        else
            let code = (uint32 (NPtr.get codes (i / 2)) >>> ((i &&& 1) * 4)) &&& 0xFu
            let mutable tri = 0u

            if code < 12u then
                // reuse
                let edge = fifo.[int code / 4]
                let edge = edge >>> (int ((code <<< 3) &&& 16u))  // shift by 16 if bit 1 is set

                // 0-1 extra vertices
                let e = NPtr.get extra extra_offset
                let c =
                    if code &&& 1u <> 0u then
                        extra_offset <- extra_offset + 1
                        uint32 e
                    else
                        let v = next
                        next <- next + 1u
                        v

                // repack triangle into edge format (0xcbac)
                tri <- ((edge &&& 0xffu) <<< 16) ||| (edge &&& 0xff00u) ||| c ||| (c <<< 24)
            else
                // restart
                let fea = if code > 12u then 1 else 0
                let feb = if code > 13u then 1 else 0
                let fec = if code > 14u then 1 else 0

                // 0-3 extra vertices
                let e_a = NPtr.get extra extra_offset
                let a =
                    if fea <> 0 then
                        extra_offset <- extra_offset + 1
                        uint32 e_a
                    else
                        let v = next
                        next <- next + 1u
                        v

                let e_b = NPtr.get extra extra_offset
                let b =
                    if feb <> 0 then
                        extra_offset <- extra_offset + 1
                        uint32 e_b
                    else
                        let v = next
                        next <- next + 1u
                        v

                let e_c = NPtr.get extra extra_offset
                let c =
                    if fec <> 0 then
                        extra_offset <- extra_offset + 1
                        uint32 e_c
                    else
                        let v = next
                        next <- next + 1u
                        v

                // repack triangle into edge format (0xcbac)
                tri <- c ||| (a <<< 8) ||| (b <<< 16) ||| (c <<< 24)

            writeTriangleUInt triangles i tri

            fifo.[2] <- fifo.[1]
            fifo.[1] <- fifo.[0]
            fifo.[0] <- tri

            i <- i + 1

    if failed then NPtr.ofNI 0n
    else NPtr.add extra extra_offset

// Scalar decodeTriangles for byte output
let private decodeTrianglesByte (triangles: nativeptr<byte>) (codes: nativeptr<byte>) (extra: nativeptr<byte>) (bound: nativeptr<byte>) (triangle_count: int) : nativeptr<byte> =
    let mutable next = 0u
    let fifo = Array.zeroCreate<uint32> 3
    let mutable extra_offset = 0
    let bound_offset = int (NPtr.toNI bound - NPtr.toNI extra)
    let mutable failed = false

    let mutable i = 0
    while i < triangle_count && not failed do
        if extra_offset > bound_offset then
            failed <- true
        else
            let code = (uint32 (NPtr.get codes (i / 2)) >>> ((i &&& 1) * 4)) &&& 0xFu
            let mutable tri = 0u

            if code < 12u then
                let edge = fifo.[int code / 4]
                let edge = edge >>> (int ((code <<< 3) &&& 16u))

                let e = NPtr.get extra extra_offset
                let c =
                    if code &&& 1u <> 0u then
                        extra_offset <- extra_offset + 1
                        uint32 e
                    else
                        let v = next
                        next <- next + 1u
                        v

                tri <- ((edge &&& 0xffu) <<< 16) ||| (edge &&& 0xff00u) ||| c ||| (c <<< 24)
            else
                let fea = if code > 12u then 1 else 0
                let feb = if code > 13u then 1 else 0
                let fec = if code > 14u then 1 else 0

                let e_a = NPtr.get extra extra_offset
                let a =
                    if fea <> 0 then
                        extra_offset <- extra_offset + 1
                        uint32 e_a
                    else
                        let v = next
                        next <- next + 1u
                        v

                let e_b = NPtr.get extra extra_offset
                let b =
                    if feb <> 0 then
                        extra_offset <- extra_offset + 1
                        uint32 e_b
                    else
                        let v = next
                        next <- next + 1u
                        v

                let e_c = NPtr.get extra extra_offset
                let c =
                    if fec <> 0 then
                        extra_offset <- extra_offset + 1
                        uint32 e_c
                    else
                        let v = next
                        next <- next + 1u
                        v

                tri <- c ||| (a <<< 8) ||| (b <<< 16) ||| (c <<< 24)

            writeTriangleByte triangles i tri

            fifo.[2] <- fifo.[1]
            fifo.[1] <- fifo.[0]
            fifo.[0] <- tri

            i <- i + 1

    if failed then NPtr.ofNI 0n
    else NPtr.add extra extra_offset

// Scalar decodeVertices for uint32 output
let private decodeVerticesUInt (vertices: nativeptr<uint32>) (ctrl: nativeptr<byte>) (data: nativeptr<byte>) (bound: nativeptr<byte>) (vertex_count: int) : nativeptr<byte> =
    let mutable last = 0xFFFFFFFFu
    let mutable data_offset = 0
    let bound_offset = int (NPtr.toNI bound - NPtr.toNI data)
    let mutable failed = false

    let mutable i = 0
    while i < vertex_count && not failed do
        if data_offset > bound_offset then
            failed <- true
        else
            let code4 = NPtr.get ctrl (i / 4)

            for k = 0 to 3 do
                let code = ((int code4 >>> k) &&& 1) ||| ((int code4 >>> (k + 3)) &&& 2)
                let length = if code4 = 0xffuy then 4 else code

                // branchlessly read up to 4 bytes
                let mask = if length = 4 then 0xFFFFFFFFu else (1u <<< (8 * length)) - 1u
                let v =
                    (uint32 (NPtr.get data (data_offset + 0)) |||
                     (uint32 (NPtr.get data (data_offset + 1)) <<< 8) |||
                     (uint32 (NPtr.get data (data_offset + 2)) <<< 16) |||
                     (uint32 (NPtr.get data (data_offset + 3)) <<< 24)) &&& mask

                // unzigzag + 1
                let d = (v >>> 1) ^^^ (uint32 (-(int (v &&& 1u))))
                let r = last + d + 1u

                if i + k < vertex_count then
                    NPtr.set vertices (i + k) r

                data_offset <- data_offset + length
                last <- r

            i <- i + 4

    if failed then NPtr.ofNI 0n
    else NPtr.add data data_offset

// Scalar decodeVertices for uint16 output
let private decodeVerticesUShort (vertices: nativeptr<uint16>) (ctrl: nativeptr<byte>) (data: nativeptr<byte>) (bound: nativeptr<byte>) (vertex_count: int) : nativeptr<byte> =
    let mutable last = 0xFFFFFFFFu
    let mutable data_offset = 0
    let bound_offset = int (NPtr.toNI bound - NPtr.toNI data)
    let mutable failed = false

    let mutable i = 0
    while i < vertex_count && not failed do
        if data_offset > bound_offset then
            failed <- true
        else
            let code4 = NPtr.get ctrl (i / 4)

            for k = 0 to 3 do
                let code = ((int code4 >>> k) &&& 1) ||| ((int code4 >>> (k + 3)) &&& 2)
                let length = if code4 = 0xffuy then 4 else code

                let mask = if length = 4 then 0xFFFFFFFFu else (1u <<< (8 * length)) - 1u
                let v =
                    (uint32 (NPtr.get data (data_offset + 0)) |||
                     (uint32 (NPtr.get data (data_offset + 1)) <<< 8) |||
                     (uint32 (NPtr.get data (data_offset + 2)) <<< 16) |||
                     (uint32 (NPtr.get data (data_offset + 3)) <<< 24)) &&& mask

                let d = (v >>> 1) ^^^ (uint32 (-(int (v &&& 1u))))
                let r = last + d + 1u

                if i + k < vertex_count then
                    NPtr.set vertices (i + k) (uint16 r)

                data_offset <- data_offset + length
                last <- r

            i <- i + 4

    if failed then NPtr.ofNI 0n
    else NPtr.add data data_offset

// Scalar decodeMeshlet
let private decodeMeshletScalar (vertices: voidptr) (triangles: voidptr) (codes: nativeptr<byte>) (ctrl: nativeptr<byte>) (data: nativeptr<byte>) (bound: nativeptr<byte>) (vertex_count: int) (triangle_count: int) (vertex_size: int) (triangle_size: int) : int =
    let mutable data = data

    if vertex_size = 4 then
        data <- decodeVerticesUInt (NPtr.ofVoid vertices) ctrl data bound vertex_count
    else
        data <- decodeVerticesUShort (NPtr.ofVoid vertices) ctrl data bound vertex_count

    if NPtr.toNI data = 0n then
        -2
    else

    if triangle_size = 4 then
        data <- decodeTrianglesUInt (NPtr.ofVoid triangles) codes data bound triangle_count
    else
        data <- decodeTrianglesByte (NPtr.ofVoid triangles) codes data bound triangle_count

    if NPtr.toNI data = 0n then
        -2
    elif NPtr.toNI data = NPtr.toNI bound then
        0
    else
        -3

// ============================================================================
// SIMD decode tables
// ============================================================================

// SIMD state is stored in a single 16b register as follows:
// 0..5: 6 next extra bytes
// 6..14: 9 bytes = 3 triangles worth of index data
// 15: 'next' byte

let private kDecodeTableMasks : byte[][] = Array.init 256 (fun _ -> Array.zeroCreate 16)
let private kDecodeTableExtra : byte[] = Array.zeroCreate 256

// for SIMD vertex decoding we need to unpack 4 values with 0-4 bytes in each
let private kDecodeTableVerts : byte[][] = Array.init 256 (fun _ -> Array.zeroCreate 16)
let private kDecodeTableLength : byte[] = Array.zeroCreate 256

let private decodeBuildTables () =
    // fill triangle decoding tables for each combination of two triangle codes
    for code = 0 to 255 do
        let shuf = Array.zeroCreate<byte> 16
        let next = Array.zeroCreate<byte> 16
        let mutable extra = 0
        let mutable nextoff = 0

        // state 6..8 will always contain the last decoded triangle
        shuf.[6] <- 12uy
        shuf.[7] <- 13uy
        shuf.[8] <- 14uy

        // state 15 will contain next (potentially incremented a few times)
        shuf.[15] <- 15uy

        for k = 0 to 1 do
            let tri = (code >>> (k * 4)) &&& 0xf

            if tri < 12 then
                if k = 1 && tri / 4 = 0 then
                    // decode one of two edges from the triangle we just decoded earlier
                    shuf.[9 + k * 3] <- shuf.[9 + (if tri &&& 2 <> 0 then 2 else 0)]
                    next.[9 + k * 3] <- next.[9 + (if tri &&& 2 <> 0 then 2 else 0)]

                    shuf.[10 + k * 3] <- shuf.[9 + (if tri &&& 2 <> 0 then 1 else 2)]
                    next.[10 + k * 3] <- next.[9 + (if tri &&& 2 <> 0 then 1 else 2)]
                else
                    // reuse: edge comes from the history based on edge index
                    let trioff = 6 + k * 3 + (2 - tri / 4) * 3

                    // edge cb or ac
                    shuf.[9 + k * 3] <- byte (trioff + (if tri &&& 2 <> 0 then 2 else 0))
                    shuf.[10 + k * 3] <- byte (trioff + (if tri &&& 2 <> 0 then 1 else 2))

                // third vertex is either next or comes from extra
                let ec = tri &&& 1
                shuf.[11 + k * 3] <- if ec <> 0 then byte extra else 15uy
                next.[11 + k * 3] <- if ec <> 0 then 0uy else byte nextoff
                extra <- extra + ec
                nextoff <- nextoff + 1 - ec
            else
                // restart: three vertices, each comes from next or extra
                let fea = if tri > 12 then 1 else 0
                let feb = if tri > 13 then 1 else 0
                let fec = if tri > 14 then 1 else 0

                shuf.[9 + k * 3] <- if fea <> 0 then byte extra else 15uy
                next.[9 + k * 3] <- if fea <> 0 then 0uy else byte nextoff
                extra <- extra + fea
                nextoff <- nextoff + 1 - fea

                shuf.[10 + k * 3] <- if feb <> 0 then byte extra else 15uy
                next.[10 + k * 3] <- if feb <> 0 then 0uy else byte nextoff
                extra <- extra + feb
                nextoff <- nextoff + 1 - feb

                shuf.[11 + k * 3] <- if fec <> 0 then byte extra else 15uy
                next.[11 + k * 3] <- if fec <> 0 then 0uy else byte nextoff
                extra <- extra + fec
                nextoff <- nextoff + 1 - fec

        // next needs to advance
        next.[15] <- byte nextoff

        // next[0..8] = 0 trivially; next[9] must also be 0
        assert (next.[9] = 0uy)
        Array.Copy(next, 10, kDecodeTableMasks.[code], 0, 6)
        Array.Copy(shuf, 6, kDecodeTableMasks.[code], 6, 10)
        kDecodeTableExtra.[code] <- byte extra

    // fill vertex decoding tables for each combination of four vertex references
    for i = 0 to 255 do
        let shuf = Array.zeroCreate<byte> 16
        let mutable offset = 0

        for k = 0 to 3 do
            let code = ((i >>> k) &&& 1) ||| ((i >>> (k + 3)) &&& 2)
            let length = if i = 0xff then 4 else code  // 0/1/2/3 bytes, or all 4 bytes if i==0xff

            shuf.[k * 4 + 0] <- if length > 0 then byte (offset + 0) else 0x80uy
            shuf.[k * 4 + 1] <- if length > 1 then byte (offset + 1) else 0x80uy
            shuf.[k * 4 + 2] <- if length > 2 then byte (offset + 2) else 0x80uy
            shuf.[k * 4 + 3] <- if length > 3 then byte (offset + 3) else 0x80uy

            offset <- offset + length

        Array.Copy(shuf, kDecodeTableVerts.[i], 16)
        kDecodeTableLength.[i] <- byte offset

// Initialize decode tables
let mutable private gDecodeTablesInitialized = false

let private ensureTablesInitialized () =
    if not gDecodeTablesInitialized then
        decodeBuildTables ()
        gDecodeTablesInitialized <- true

// ============================================================================
// SSE4.1 SIMD helpers
// ============================================================================

let inline private decodeTriangleGroupSse (state: Vector128<byte>) (code: byte) (extra: byref<nativeptr<byte>>) : Vector128<byte> =
    let shuf = Sse2.LoadVector128(&&kDecodeTableMasks.[int code].[0] |> NPtr.cast<byte, byte>)
    let next = Sse2.ShiftLeftLogical128BitLane(shuf, 10uy)

    // patch first 6 bytes with current extra and roll state forward
    let ext = Sse2.LoadScalarVector128(extra |> NPtr.cast<byte, int64> |> NPtr.toVoid |> NPtr.ofVoid<int64>)
    let ext = ext.AsByte()
    let state = Sse41.Blend(state.AsInt16(), ext.AsInt16(), 7uy).AsByte()
    let state = Sse2.Add(Ssse3.Shuffle(state, shuf), next)

    extra <- NPtr.add extra (int kDecodeTableExtra.[int code])

    state

let inline private decodeVertexGroupSse (last: Vector128<int>) (code: byte) (data: byref<nativeptr<byte>>) : Vector128<int> =
    let word = Sse2.LoadVector128(data |> NPtr.cast<byte, byte>)
    let shuf = Sse2.LoadVector128(&&kDecodeTableVerts.[int code].[0] |> NPtr.cast<byte, byte>)

    let v = Ssse3.Shuffle(word, shuf).AsInt32()

    // unzigzag+1
    let xl = Sse2.Subtract(Vector128<int>.Zero, Sse2.And(v, Vector128.Create(1)))
    let xr = Sse2.ShiftRightLogical(v, 1uy)
    let mutable x = Sse2.Add(Sse2.Xor(xl, xr), Vector128.Create(1))

    // prefix sum
    x <- Sse2.Add(x, Sse2.ShiftLeftLogical128BitLane(x, 8uy).AsInt32())
    x <- Sse2.Add(x, Sse2.ShiftLeftLogical128BitLane(x, 4uy).AsInt32())
    x <- Sse2.Add(x, Sse2.Shuffle(last, 0xFFuy))

    data <- NPtr.add data (int kDecodeTableLength.[int code])

    x

// ============================================================================
// NEON SIMD helpers
// ============================================================================

let inline private decodeTriangleGroupNeon (state: Vector128<byte>) (code: byte) (extra: byref<nativeptr<byte>>) : Vector128<byte> =
    let shuf = AdvSimd.LoadVector128(&&kDecodeTableMasks.[int code].[0] |> NPtr.cast<byte, byte>)
    let next = AdvSimd.ExtractVector128(Vector128.Create(0uy), shuf, 6uy)

    // patch first 6 bytes with current extra and roll state forward
    let ext = AdvSimd.LoadVector64(extra)
    let ext = Vector128.Create(ext, Vector64.Create(0uy))
    let mask6 = Vector128.Create(0xFFuy, 0xFFuy, 0xFFuy, 0xFFuy, 0xFFuy, 0xFFuy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy)
    let state = AdvSimd.BitwiseSelect(mask6, ext, state)
    let state = AdvSimd.Add(AdvSimd.Arm64.VectorTableLookup(state, shuf), next)

    extra <- NPtr.add extra (int kDecodeTableExtra.[int code])

    state

let inline private decodeVertexGroupNeon (last: Vector128<uint32>) (code: byte) (data: byref<nativeptr<byte>>) : Vector128<uint32> =
    let word = AdvSimd.LoadVector128(data |> NPtr.cast<byte, byte>)
    let shuf = AdvSimd.LoadVector128(&&kDecodeTableVerts.[int code].[0] |> NPtr.cast<byte, byte>)

    let v = AdvSimd.Arm64.VectorTableLookup(word, shuf).AsUInt32()

    // unzigzag+1
    let xl = AdvSimd.Subtract(Vector128.Create(0u), AdvSimd.And(v, Vector128.Create(1u)))
    let xr = AdvSimd.ShiftRightLogical(v, 1uy)
    let mutable x = AdvSimd.Add(AdvSimd.Xor(xl, xr), Vector128.Create(1u))

    // prefix sum
    x <- AdvSimd.Add(x, AdvSimd.ExtractVector128(Vector128.Create(0u), x, 2uy))
    x <- AdvSimd.Add(x, AdvSimd.ExtractVector128(Vector128.Create(0u), x, 3uy))
    x <- AdvSimd.Add(x, Vector128.Create(last.GetElement(3)))

    data <- NPtr.add data (int kDecodeTableLength.[int code])

    x

// ============================================================================
// SIMD decodeTriangles for uint32 output
// ============================================================================

let private decodeTrianglesSimdUInt (triangles: nativeptr<uint32>) (codes: nativeptr<byte>) (extra: nativeptr<byte>) (bound: nativeptr<byte>) (triangle_count: int) : nativeptr<byte> =
    let mutable extra = extra
    let mutable codes_offset = 0

    let groups = triangle_count / 2

    if Sse41.IsSupported then
        let repack = Vector128.Create(9uy, 10uy, 11uy, 0xFFuy, 12uy, 13uy, 14uy, 0xFFuy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy)
        let mutable state = Vector128<byte>.Zero
        let mutable failed = false

        // process all complete groups
        let mutable i = 0
        while i < groups && not failed do
            let code = NPtr.get codes codes_offset
            codes_offset <- codes_offset + 1

            if NPtr.toNI extra > NPtr.toNI bound then
                failed <- true
            else
                state <- decodeTriangleGroupSse state code &extra

                // write 6 bytes of new triangle data into output, formatted as 8 bytes with 0 padding
                let r = Ssse3.Shuffle(state, repack)
                let storeAddr : nativeptr<int64> = NPtr.cast<uint32, int64> (NPtr.add triangles (i * 2))
                Sse2.StoreScalar(storeAddr, r.AsInt64())

                i <- i + 1

        if failed then NPtr.ofNI 0n
        else
            // process a 1 triangle tail
            if triangle_count &&& 1 <> 0 then
                let code = NPtr.get codes codes_offset
                codes_offset <- codes_offset + 1

                if NPtr.toNI extra > NPtr.toNI bound then NPtr.ofNI 0n
                else
                    state <- decodeTriangleGroupSse state code &extra

                    let tail = NPtr.add triangles (triangle_count &&& ~~~1)

                    let r = Ssse3.Shuffle(state, repack)
                    NPtr.set tail 0 (uint32 (Sse2.ConvertToInt32(r.AsInt32())))

                    extra
            else
                extra

    elif AdvSimd.Arm64.IsSupported then
        let repack = Vector64.Create(9uy, 10uy, 11uy, 0xFFuy, 12uy, 13uy, 14uy, 0xFFuy)
        let mutable state = Vector128<byte>.Zero
        let mutable failed = false

        let mutable i = 0
        while i < groups && not failed do
            let code = NPtr.get codes codes_offset
            codes_offset <- codes_offset + 1

            if NPtr.toNI extra > NPtr.toNI bound then
                failed <- true
            else
                state <- decodeTriangleGroupNeon state code &extra

                let r = AdvSimd.Arm64.VectorTableLookup(state, Vector128.Create(repack, Vector64.Create(0uy))).AsUInt32()
                // Store lower 64-bits (2 uint32s)
                AdvSimd.Store(NPtr.add triangles (i * 2) |> NPtr.cast<uint32, uint32>, r.GetLower())

                i <- i + 1

        if failed then NPtr.ofNI 0n
        else
            if triangle_count &&& 1 <> 0 then
                let code = NPtr.get codes codes_offset
                codes_offset <- codes_offset + 1

                if NPtr.toNI extra > NPtr.toNI bound then NPtr.ofNI 0n
                else
                    state <- decodeTriangleGroupNeon state code &extra

                    let tail = NPtr.add triangles (triangle_count &&& ~~~1)
                    let r = AdvSimd.Arm64.VectorTableLookup(state, Vector128.Create(repack, Vector64.Create(0uy))).AsUInt32()
                    NPtr.set tail 0 (r.GetElement(0))

                    extra
            else
                extra
    else
        // scalar fallback
        decodeTrianglesUInt triangles codes extra bound triangle_count

// ============================================================================
// SIMD decodeTriangles for byte output
// ============================================================================

let private decodeTrianglesSimdByte (triangles: nativeptr<byte>) (codes: nativeptr<byte>) (extra: nativeptr<byte>) (bound: nativeptr<byte>) (triangle_count: int) : nativeptr<byte> =
    let mutable extra = extra
    let mutable codes_offset = 0

    // process 2 *pairs* at a time (12-byte write) followed by a tail pair
    let groups = (triangle_count + 1) / 4

    if Sse41.IsSupported then
        let mutable state = Vector128<byte>.Zero
        let mutable failed = false

        let mutable i = 0
        while i < groups && not failed do
            let code0 = NPtr.get codes codes_offset
            codes_offset <- codes_offset + 1
            let code1 = NPtr.get codes codes_offset
            codes_offset <- codes_offset + 1

            if NPtr.toNI extra > NPtr.toNI bound then
                failed <- true
            else
                state <- decodeTriangleGroupSse state code0 &extra

                // write first decoded triangle and first index of second decoded triangle
                let r0 = Sse2.ShiftRightLogical128BitLane(state, 9uy)
                // Store 4 bytes (unaligned int write)
                let r0val = Sse2.ConvertToInt32(r0.AsInt32())
                let dst0 = NPtr.add triangles (i * 12) |> NPtr.cast<byte, int>
                NPtr.set dst0 0 r0val

                state <- decodeTriangleGroupSse state code1 &extra

                // write last two indices plus two new ones
                let r1 = Sse2.ShiftRightLogical128BitLane(state, 7uy)
                Sse2.StoreScalar(NPtr.add triangles (i * 12 + 4) |> NPtr.cast<byte, int64> |> NPtr.toVoid |> NPtr.ofVoid<int64>, r1.AsInt64())

                i <- i + 1

        if failed then NPtr.ofNI 0n
        else
            // process a 1-2 triangle tail
            if groups * 4 < triangle_count then
                let code = NPtr.get codes codes_offset
                codes_offset <- codes_offset + 1

                if NPtr.toNI extra > NPtr.toNI bound then NPtr.ofNI 0n
                else
                    state <- decodeTriangleGroupSse state code &extra

                    let tail = NPtr.add triangles ((triangle_count &&& ~~~3) * 3)

                    let r = Sse2.ShiftRightLogical128BitLane(state, 9uy)

                    let dst = tail |> NPtr.cast<byte, int>
                    NPtr.set dst 0 (Sse2.ConvertToInt32(r.AsInt32()))
                    if (triangle_count &&& 3) > 1 then
                        let dst4 = NPtr.add tail 4 |> NPtr.cast<byte, int>
                        NPtr.set dst4 0 (Sse41.Extract(r.AsInt32(), 1uy))

                    extra
            else
                extra

    elif AdvSimd.Arm64.IsSupported then
        let mutable state = Vector128<byte>.Zero
        let mutable failed = false

        let mutable i = 0
        while i < groups && not failed do
            let code0 = NPtr.get codes codes_offset
            codes_offset <- codes_offset + 1
            let code1 = NPtr.get codes codes_offset
            codes_offset <- codes_offset + 1

            if NPtr.toNI extra > NPtr.toNI bound then
                failed <- true
            else
                state <- decodeTriangleGroupNeon state code0 &extra

                let r0 = AdvSimd.ExtractVector128(state, Vector128<byte>.Zero, 9uy)
                let dst0 = NPtr.add triangles (i * 12) |> NPtr.cast<byte, uint32>
                NPtr.set dst0 0 (r0.AsUInt32().GetElement(0))

                state <- decodeTriangleGroupNeon state code1 &extra

                let r1 = AdvSimd.ExtractVector128(state, Vector128<byte>.Zero, 7uy)
                // Store 8 bytes
                AdvSimd.Store(NPtr.add triangles (i * 12 + 4), r1.GetLower())

                i <- i + 1

        if failed then NPtr.ofNI 0n
        else
            if groups * 4 < triangle_count then
                let code = NPtr.get codes codes_offset
                codes_offset <- codes_offset + 1

                if NPtr.toNI extra > NPtr.toNI bound then NPtr.ofNI 0n
                else
                    state <- decodeTriangleGroupNeon state code &extra

                    let tail = NPtr.add triangles ((triangle_count &&& ~~~3) * 3)

                    let r = AdvSimd.ExtractVector128(state, Vector128<byte>.Zero, 9uy)

                    let dst = tail |> NPtr.cast<byte, uint32>
                    NPtr.set dst 0 (r.AsUInt32().GetElement(0))
                    if (triangle_count &&& 3) > 1 then
                        let dst4 = NPtr.add tail 4 |> NPtr.cast<byte, uint32>
                        NPtr.set dst4 0 (r.AsUInt32().GetElement(1))

                    extra
            else
                extra
    else
        // scalar fallback
        decodeTrianglesByte triangles codes extra bound triangle_count

// ============================================================================
// SIMD decodeVertices for uint32 output
// ============================================================================

let private decodeVerticesSimdUInt (vertices: nativeptr<uint32>) (ctrl: nativeptr<byte>) (data: nativeptr<byte>) (bound: nativeptr<byte>) (vertex_count: int) : nativeptr<byte> =
    let mutable data = data
    let mutable ctrl_offset = 0

    if Sse41.IsSupported then
        let mutable last = Vector128.Create(-1)
        let groups = vertex_count / 4
        let mutable failed = false

        let mutable i = 0
        while i < groups && not failed do
            let code = NPtr.get ctrl ctrl_offset
            ctrl_offset <- ctrl_offset + 1
            if NPtr.toNI data > NPtr.toNI bound then
                failed <- true
            else
                last <- decodeVertexGroupSse last code &data

                Sse2.Store(NPtr.add vertices (i * 4) |> NPtr.cast<uint32, int>, last)

                i <- i + 1

        if failed then NPtr.ofNI 0n
        else
            // process a 1-3 vertex tail
            if vertex_count &&& 3 <> 0 then
                let code = NPtr.get ctrl ctrl_offset
                ctrl_offset <- ctrl_offset + 1

                if NPtr.toNI data > NPtr.toNI bound then NPtr.ofNI 0n
                else
                    last <- decodeVertexGroupSse last code &data

                    let tail = NPtr.add vertices (vertex_count &&& ~~~3)

                    NPtr.set tail 0 (uint32 (Sse2.ConvertToInt32(last)))
                    if (vertex_count &&& 3) > 1 then
                        NPtr.set tail 1 (uint32 (Sse41.Extract(last, 1uy)))
                    if (vertex_count &&& 3) > 2 then
                        NPtr.set tail 2 (uint32 (Sse41.Extract(last, 2uy)))

                    data
            else
                data

    elif AdvSimd.Arm64.IsSupported then
        let mutable last = Vector128.Create(0xFFFFFFFFu)
        let groups = vertex_count / 4
        let mutable failed = false

        let mutable i = 0
        while i < groups && not failed do
            let code = NPtr.get ctrl ctrl_offset
            ctrl_offset <- ctrl_offset + 1
            if NPtr.toNI data > NPtr.toNI bound then
                failed <- true
            else
                last <- decodeVertexGroupNeon last code &data

                AdvSimd.Store(NPtr.add vertices (i * 4), last)

                i <- i + 1

        if failed then NPtr.ofNI 0n
        else
            if vertex_count &&& 3 <> 0 then
                let code = NPtr.get ctrl ctrl_offset
                ctrl_offset <- ctrl_offset + 1

                if NPtr.toNI data > NPtr.toNI bound then NPtr.ofNI 0n
                else
                    last <- decodeVertexGroupNeon last code &data

                    let tail = NPtr.add vertices (vertex_count &&& ~~~3)

                    NPtr.set tail 0 (last.GetElement(0))
                    if (vertex_count &&& 3) > 1 then
                        NPtr.set tail 1 (last.GetElement(1))
                    if (vertex_count &&& 3) > 2 then
                        NPtr.set tail 2 (last.GetElement(2))

                    data
            else
                data
    else
        // scalar fallback
        decodeVerticesUInt vertices ctrl data bound vertex_count

// ============================================================================
// SIMD decodeVertices for uint16 output
// ============================================================================

let private decodeVerticesSimdUShort (vertices: nativeptr<uint16>) (ctrl: nativeptr<byte>) (data: nativeptr<byte>) (bound: nativeptr<byte>) (vertex_count: int) : nativeptr<byte> =
    let mutable data = data
    let mutable ctrl_offset = 0

    if Sse41.IsSupported then
        let repack = Vector128.Create(0uy, 1uy, 4uy, 5uy, 8uy, 9uy, 12uy, 13uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy, 0uy)
        let mutable last = Vector128.Create(-1)

        // if the number of vertices mod 4 is 3, we can overwrite up to 2 bytes in the main loop
        let groups = (vertex_count + 1) / 4
        let mutable failed = false

        let mutable i = 0
        while i < groups && not failed do
            let code = NPtr.get ctrl ctrl_offset
            ctrl_offset <- ctrl_offset + 1

            if NPtr.toNI data > NPtr.toNI bound then
                failed <- true
            else
                last <- decodeVertexGroupSse last code &data

                let r = Ssse3.Shuffle(last.AsByte(), repack)
                Sse2.StoreScalar(NPtr.add vertices (i * 4) |> NPtr.cast<uint16, int64> |> NPtr.toVoid |> NPtr.ofVoid<int64>, r.AsInt64())

                i <- i + 1

        if failed then NPtr.ofNI 0n
        else
            // process a 1-2 vertex tail
            if groups * 4 < vertex_count then
                let code = NPtr.get ctrl ctrl_offset
                ctrl_offset <- ctrl_offset + 1

                if NPtr.toNI data > NPtr.toNI bound then NPtr.ofNI 0n
                else
                    last <- decodeVertexGroupSse last code &data

                    let tail = NPtr.add vertices (vertex_count &&& ~~~3)

                    // _mm_shufflelo_epi16(last, 8) = shuffle with control 0b00_00_10_00 = 8
                    let r = Sse2.ShuffleLow(last.AsInt16(), 8uy)
                    let dst = tail |> NPtr.cast<uint16, int>
                    NPtr.set dst 0 (Sse2.ConvertToInt32(r.AsInt32()))

                    data
            else
                data

    elif AdvSimd.Arm64.IsSupported then
        let mutable last = Vector128.Create(0xFFFFFFFFu)

        let groups = (vertex_count + 1) / 4
        let mutable failed = false

        let mutable i = 0
        while i < groups && not failed do
            let code = NPtr.get ctrl ctrl_offset
            ctrl_offset <- ctrl_offset + 1

            if NPtr.toNI data > NPtr.toNI bound then
                failed <- true
            else
                last <- decodeVertexGroupNeon last code &data

                let r = AdvSimd.ExtractNarrowingLower(last)
                AdvSimd.Store(NPtr.add vertices (i * 4), r)

                i <- i + 1

        if failed then NPtr.ofNI 0n
        else
            if groups * 4 < vertex_count then
                let code = NPtr.get ctrl ctrl_offset
                ctrl_offset <- ctrl_offset + 1

                if NPtr.toNI data > NPtr.toNI bound then NPtr.ofNI 0n
                else
                    last <- decodeVertexGroupNeon last code &data

                    let tail = NPtr.add vertices (vertex_count &&& ~~~3)
                    let r = AdvSimd.ExtractNarrowingLower(last)
                    // Store 4 bytes (2 uint16)
                    let dst = tail |> NPtr.cast<uint16, uint32>
                    NPtr.set dst 0 (r.AsUInt32().GetElement(0))

                    data
            else
                data
    else
        // scalar fallback
        decodeVerticesUShort vertices ctrl data bound vertex_count

// ============================================================================
// SIMD decodeMeshlet
// ============================================================================

let private decodeMeshletSimd (raw: bool) (vertices: voidptr) (triangles: voidptr) (codes: nativeptr<byte>) (ctrl: nativeptr<byte>) (data: nativeptr<byte>) (bound: nativeptr<byte>) (vertex_count: int) (triangle_count: int) (vertex_size: int) (triangle_size: int) : int =
    ensureTablesInitialized ()

    let mutable data = data

    // decodes 4 vertices at a time with tail processing
    // raw decoding skips tail processing by rounding up vertex count
    if vertex_size = 4 || raw then
        let vc = if raw then (vertex_count + 3) &&& ~~~3 else vertex_count
        data <- decodeVerticesSimdUInt (NPtr.ofVoid vertices) ctrl data bound vc
    else
        data <- decodeVerticesSimdUShort (NPtr.ofVoid vertices) ctrl data bound vertex_count

    if NPtr.toNI data = 0n then
        -2
    else

    // decodes 2/4 triangles at a time with tail processing
    // raw decoding skips tail processing by rounding up triangle count
    if triangle_size = 4 || raw then
        let tc = if raw then (triangle_count + 1) &&& ~~~1 else triangle_count
        data <- decodeTrianglesSimdUInt (NPtr.ofVoid triangles) codes data bound tc
    else
        data <- decodeTrianglesSimdByte (NPtr.ofVoid triangles) codes data bound triangle_count

    if NPtr.toNI data = 0n then
        -2
    elif NPtr.toNI data = NPtr.toNI bound then
        0
    else
        -3

// ============================================================================
// Public API
// ============================================================================

let meshopt_encodeMeshletBound (max_vertices: int) (max_triangles: int) : int =
    let codes_size = (max_triangles + 1) / 2
    let extra_size = max_triangles * 3

    let ctrl_size = (max_vertices + 3) / 4
    let data_size = (max_vertices + 3) / 4 * 16  // worst case: 16 bytes per vertex group

    let gap_size = if codes_size + ctrl_size < 16 then 16 - (codes_size + ctrl_size) else 0

    codes_size + extra_size + ctrl_size + data_size + gap_size

let meshopt_encodeMeshlet (buffer: nativeptr<byte>) (buffer_size: int) (vertices: nativeptr<uint32>) (vertex_count: int) (triangles: nativeptr<byte>) (triangle_count: int) : int =
    assert (triangle_count <= 256 && vertex_count <= 256)

    // 4 bits per triangle + up to three bytes of extra data
    let codes = Array.zeroCreate<byte> 128  // 256/2
    let extra = Array.zeroCreate<byte> 768  // 256*3
    let codes_size = (triangle_count + 1) / 2

    // Pin arrays for native pointer access
    use codesPin = fixed &codes.[0]
    use extraPin = fixed &extra.[0]

    let extra_size = encodeTriangles codesPin extraPin triangles triangle_count
    assert (extra_size <= extra.Length)

    // 2 bits per vertex + up to 4 bytes of actual data
    let ctrl = Array.zeroCreate<byte> 64   // 256/4
    let data = Array.zeroCreate<byte> 1024 // 256*4
    let ctrl_size = (vertex_count + 3) / 4

    use ctrlPin = fixed &ctrl.[0]
    use dataPin = fixed &data.[0]

    let data_size = encodeVertices ctrlPin dataPin vertices vertex_count
    assert (data_size <= data.Length)

    // we need to ensure that up to 16 bytes after extra+data are available for SIMD decoding
    let gap_size = if codes_size + ctrl_size < 16 then 16 - (codes_size + ctrl_size) else 0

    let result = codes_size + extra_size + ctrl_size + data_size + gap_size

    if result > buffer_size then
        0
    else

    // variable-size data first
    NPtr.memcpy buffer dataPin data_size
    let mutable offset = data_size

    NPtr.memcpy (NPtr.add buffer offset) extraPin extra_size
    offset <- offset + extra_size

    // gap (for accelerated decoding) separates variable-size and fixed-size data
    NPtr.memset (NPtr.add buffer offset) 0uy gap_size
    offset <- offset + gap_size

    // fixed-size data last; it can be located from buffer end during decoding
    NPtr.memcpy (NPtr.add buffer offset) ctrlPin ctrl_size
    offset <- offset + ctrl_size

    NPtr.memcpy (NPtr.add buffer offset) codesPin codes_size
    offset <- offset + codes_size

    result

let meshopt_decodeMeshlet (vertices: voidptr) (vertex_count: int) (vertex_size: int) (triangles: voidptr) (triangle_count: int) (triangle_size: int) (buffer: nativeptr<byte>) (buffer_size: int) : int =
    assert (triangle_count <= 256 && vertex_count <= 256)
    assert (vertex_size = 4 || vertex_size = 2)
    assert (triangle_size = 4 || triangle_size = 3)

    // layout must match encoding
    let codes_size = (triangle_count + 1) / 2
    let ctrl_size = (vertex_count + 3) / 4
    let gap_size = if codes_size + ctrl_size < 16 then 16 - (codes_size + ctrl_size) else 0

    if buffer_size < codes_size + ctrl_size + gap_size then
        -2
    else

    let endPtr = NPtr.add buffer buffer_size
    let codes = NPtr.add endPtr (-codes_size)
    let ctrl = NPtr.add codes (-ctrl_size)
    let data = buffer

    // gap ensures we have at least 16 bytes available after bound
    let bound = NPtr.add ctrl (-gap_size)
    assert (NPtr.toNI bound >= NPtr.toNI buffer && NPtr.toNI (NPtr.add bound 16) <= NPtr.toNI (NPtr.add buffer buffer_size))

    // Use SIMD path if available, otherwise scalar fallback
    if Sse41.IsSupported || AdvSimd.Arm64.IsSupported then
        ensureTablesInitialized ()
        decodeMeshletSimd false vertices triangles codes ctrl data bound vertex_count triangle_count vertex_size triangle_size
    else
        decodeMeshletScalar vertices triangles codes ctrl data bound vertex_count triangle_count vertex_size triangle_size

let meshopt_decodeMeshletRaw (vertices: nativeptr<uint32>) (vertex_count: int) (triangles: nativeptr<uint32>) (triangle_count: int) (buffer: nativeptr<byte>) (buffer_size: int) : int =
    assert (triangle_count <= 256 && vertex_count <= 256)

    // layout must match encoding
    let codes_size = (triangle_count + 1) / 2
    let ctrl_size = (vertex_count + 3) / 4
    let gap_size = if codes_size + ctrl_size < 16 then 16 - (codes_size + ctrl_size) else 0

    if buffer_size < codes_size + ctrl_size + gap_size then
        -2
    else

    let endPtr = NPtr.add buffer buffer_size
    let codes = NPtr.add endPtr (-codes_size)
    let ctrl = NPtr.add codes (-ctrl_size)
    let data = buffer

    let bound = NPtr.add ctrl (-gap_size)
    assert (NPtr.toNI bound >= NPtr.toNI buffer && NPtr.toNI (NPtr.add bound 16) <= NPtr.toNI (NPtr.add buffer buffer_size))

    if Sse41.IsSupported || AdvSimd.Arm64.IsSupported then
        ensureTablesInitialized ()
        decodeMeshletSimd true (NPtr.toVoid vertices) (NPtr.toVoid triangles) codes ctrl data bound vertex_count triangle_count 4 4
    else
        decodeMeshletScalar (NPtr.toVoid vertices) (NPtr.toVoid triangles) codes ctrl data bound vertex_count triangle_count 4 4
