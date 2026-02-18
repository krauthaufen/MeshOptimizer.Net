// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
module MeshOptPort.VertexCodec

open System
open System.Runtime.CompilerServices
open System.Runtime.InteropServices
open System.Runtime.Intrinsics
open System.Runtime.Intrinsics.X86
open System.Runtime.Intrinsics.Arm
open MeshOptPort
open MeshOptPort.Allocator

let private kVertexHeader = 0xa0uy

let mutable private gEncodeVertexVersion = 1
let private kDecodeVertexVersion = 1

let private kVertexBlockSizeBytes = 8192
let private kVertexBlockMaxSize = 256
let private kByteGroupSize = 16
let private kByteGroupDecodeLimit = 24
let private kTailMinSizeV0 = 32
let private kTailMinSizeV1 = 24

let private kBitsV0 = [| 0; 2; 4; 8 |]
let private kBitsV1 = [| 0; 1; 2; 4; 8 |]

let private kEncodeDefaultLevel = 2

let private getVertexBlockSize (vertex_size: int) : int =
    // make sure the entire block fits into the scratch buffer and is aligned to byte group size
    // note: the block size is implicitly part of the format, so we can't change it without breaking compatibility
    let result = (kVertexBlockSizeBytes / vertex_size) &&& ~~~(kByteGroupSize - 1)
    if result < kVertexBlockMaxSize then result else kVertexBlockMaxSize

let inline private rotate (v: uint32) (r: int) : uint32 =
    (v <<< r) ||| (v >>> ((32 - r) &&& 31))

let inline private zigzagByte (v: byte) : byte =
    (0uy - (v >>> 7)) ^^^ (v <<< 1)

let inline private zigzagUInt16 (v: uint16) : uint16 =
    (0us - (v >>> 15)) ^^^ (v <<< 1)

let inline private unzigzagByte (v: byte) : byte =
    (0uy - (v &&& 1uy)) ^^^ (v >>> 1)

let inline private unzigzagUInt16 (v: uint16) : uint16 =
    (0us - (v &&& 1us)) ^^^ (v >>> 1)

let inline private unzigzagUInt32 (v: uint32) : uint32 =
    (0u - (v &&& 1u)) ^^^ (v >>> 1)

// ------- Encode helpers -------

let private encodeBytesGroupZero (buffer: nativeptr<byte>) : bool =
    assert (kByteGroupSize = sizeof<uint64> * 2)
    let vp = NPtr.cast<byte, uint64> buffer
    let v0 = NPtr.get vp 0
    let v1 = NPtr.get vp 1
    (v0 ||| v1) = 0UL

let private encodeBytesGroupMeasure (buffer: nativeptr<byte>) (bits: int) : int =
    assert (bits >= 0 && bits <= 8)

    if bits = 0 then
        if encodeBytesGroupZero buffer then 0 else System.Int32.MaxValue // size_t(-1) in C++
    elif bits = 8 then
        kByteGroupSize
    else

    let mutable result = kByteGroupSize * bits / 8

    let sentinel = byte ((1 <<< bits) - 1)

    for i = 0 to kByteGroupSize - 1 do
        if NPtr.get buffer i >= sentinel then
            result <- result + 1

    result

let private encodeBytesGroup (data: nativeptr<byte>) (dataOff: byref<int>) (buffer: nativeptr<byte>) (bits: int) =
    assert (bits >= 0 && bits <= 8)
    assert (kByteGroupSize % 8 = 0)

    if bits = 0 then
        ()
    elif bits = 8 then
        NPtr.memcpy (NPtr.add data dataOff) buffer (kByteGroupSize)
        dataOff <- dataOff + kByteGroupSize
    else

    let byte_size = 8 / bits
    assert (kByteGroupSize % byte_size = 0)

    // fixed portion: bits bits for each value
    // variable portion: full byte for each out-of-range value (using 1...1 as sentinel)
    let sentinel = byte ((1 <<< bits) - 1)

    let mutable i = 0
    while i < kByteGroupSize do
        let mutable b = 0uy

        for k = 0 to byte_size - 1 do
            let v = NPtr.get buffer (i + k)
            let enc = if v >= sentinel then sentinel else v
            b <- (b <<< bits) ||| enc

        // encode 1-bit groups in reverse bit order
        // this makes them faster to decode alongside other groups
        let b =
            if bits = 1 then
                byte (((uint64 b * 0x80200802UL) &&& 0x0884422110UL) * 0x0101010101UL >>> 32)
            else b

        NPtr.set data dataOff b
        dataOff <- dataOff + 1

        i <- i + byte_size

    for i = 0 to kByteGroupSize - 1 do
        let v = NPtr.get buffer i

        // branchless append of out-of-range values
        NPtr.set data dataOff v
        if v >= sentinel then
            dataOff <- dataOff + 1

let private encodeBytes (data: nativeptr<byte>) (dataOff: byref<int>) (data_end: int) (buffer: nativeptr<byte>) (buffer_size: int) (bits: int[]) : bool =
    assert (buffer_size % kByteGroupSize = 0)

    let headerStart = dataOff

    // round number of groups to 4 to get number of header bytes
    let header_size = (buffer_size / kByteGroupSize + 3) / 4

    if data_end - dataOff < header_size then
        false
    else

    dataOff <- dataOff + header_size

    NPtr.memset (NPtr.add data headerStart) 0uy header_size

    let mutable last_bits = -1
    let mutable i = 0
    let mutable ok = true

    while i < buffer_size && ok do
        if data_end - dataOff < kByteGroupDecodeLimit then
            ok <- false
        else

        let mutable best_bitk = 3
        let mutable best_size = encodeBytesGroupMeasure (NPtr.add buffer i) bits.[best_bitk]

        for bitk = 0 to 2 do
            let size = encodeBytesGroupMeasure (NPtr.add buffer i) bits.[bitk]

            // favor consistent bit selection across groups, but never replace literals
            if size < best_size || (size = best_size && bits.[bitk] = last_bits && bits.[best_bitk] <> 8) then
                best_bitk <- bitk
                best_size <- size

        let header_offset = i / kByteGroupSize
        let hdr = NPtr.get data (headerStart + header_offset / 4)
        NPtr.set data (headerStart + header_offset / 4) (hdr ||| byte (best_bitk <<< ((header_offset % 4) * 2)))

        let best_bits = bits.[best_bitk]
        let savedOff = dataOff
        encodeBytesGroup data &dataOff (NPtr.add buffer i) best_bits

        assert (dataOff - savedOff = best_size)
        last_bits <- best_bits

        i <- i + kByteGroupSize

    ok

let private encodeDeltas1_byte (buffer: nativeptr<byte>) (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (k: int) =
    let p = ref (NPtr.get last_vertex k)
    let mutable vertex = NPtr.add vertex_data k

    for i = 0 to vertex_count - 1 do
        let v = NPtr.get vertex 0
        let d = zigzagByte (v - p.Value)
        NPtr.set buffer i d
        p.Value <- v
        vertex <- NPtr.add vertex vertex_size

let private encodeDeltas1_ushort (buffer: nativeptr<byte>) (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (k: int) =
    let k0 = k &&& ~~~1
    let ks = (k &&& 1) * 8

    let mutable p = uint16 (NPtr.get last_vertex k0) ||| (uint16 (NPtr.get last_vertex (k0 + 1)) <<< 8)
    let mutable vertex = NPtr.add vertex_data k0

    for i = 0 to vertex_count - 1 do
        let v = uint16 (NPtr.get vertex 0) ||| (uint16 (NPtr.get vertex 1) <<< 8)
        let d = zigzagUInt16 (v - p)
        NPtr.set buffer i (byte (d >>> ks))
        p <- v
        vertex <- NPtr.add vertex vertex_size

let private encodeDeltas1_uint_xor (buffer: nativeptr<byte>) (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (k: int) (rot: int) =
    let k0 = k &&& ~~~3
    let ks = (k &&& 3) * 8

    let mutable p =
        uint32 (NPtr.get last_vertex k0)
        ||| (uint32 (NPtr.get last_vertex (k0 + 1)) <<< 8)
        ||| (uint32 (NPtr.get last_vertex (k0 + 2)) <<< 16)
        ||| (uint32 (NPtr.get last_vertex (k0 + 3)) <<< 24)

    let mutable vertex = NPtr.add vertex_data k0

    for i = 0 to vertex_count - 1 do
        let v =
            uint32 (NPtr.get vertex 0)
            ||| (uint32 (NPtr.get vertex 1) <<< 8)
            ||| (uint32 (NPtr.get vertex 2) <<< 16)
            ||| (uint32 (NPtr.get vertex 3) <<< 24)
        let d = rotate (v ^^^ p) rot
        NPtr.set buffer i (byte (d >>> ks))
        p <- v
        vertex <- NPtr.add vertex vertex_size

let private encodeDeltas (buffer: nativeptr<byte>) (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (k: int) (channel: int) =
    match channel &&& 3 with
    | 0 -> encodeDeltas1_byte buffer vertex_data vertex_count vertex_size last_vertex k
    | 1 -> encodeDeltas1_ushort buffer vertex_data vertex_count vertex_size last_vertex k
    | 2 -> encodeDeltas1_uint_xor buffer vertex_data vertex_count vertex_size last_vertex k (channel >>> 4)
    | _ -> assert false

let private estimateBits (v: byte) : int =
    if v <= 15uy then (if v <= 3uy then (if v = 0uy then 0 else 2) else 4) else 8

let private estimateRotate (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (k: int) (group_size: int) : int =
    let sizes = Array.zeroCreate<int> 8

    let mutable vertex = NPtr.add vertex_data k
    let mutable last =
        uint32 (NPtr.get vertex 0)
        ||| (uint32 (NPtr.get vertex 1) <<< 8)
        ||| (uint32 (NPtr.get vertex 2) <<< 16)
        ||| (uint32 (NPtr.get vertex 3) <<< 24)

    let mutable i = 0
    while i < vertex_count do
        let mutable bitg = 0u

        // calculate bit consistency mask for the group
        let mutable j = 0
        while j < group_size && i + j < vertex_count do
            let v =
                uint32 (NPtr.get vertex 0)
                ||| (uint32 (NPtr.get vertex 1) <<< 8)
                ||| (uint32 (NPtr.get vertex 2) <<< 16)
                ||| (uint32 (NPtr.get vertex 3) <<< 24)
            let d = v ^^^ last

            bitg <- bitg ||| d
            last <- v
            vertex <- NPtr.add vertex vertex_size
            j <- j + 1

        for j = 0 to 7 do
            let bitr = rotate bitg j

            sizes.[j] <- sizes.[j] + estimateBits (byte (bitr >>> 0)) + estimateBits (byte (bitr >>> 8))
            sizes.[j] <- sizes.[j] + estimateBits (byte (bitr >>> 16)) + estimateBits (byte (bitr >>> 24))

        i <- i + group_size

    let mutable best_rot = 0
    for rot = 1 to 7 do
        best_rot <- if sizes.[rot] < sizes.[best_rot] then rot else best_rot

    best_rot

let private estimateChannel (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (k: int) (vertex_block_size: int) (block_skip: int) (max_channel: int) (xor_rot: int) : int =
    let block = Array.zeroCreate<byte> kVertexBlockMaxSize
    assert (vertex_block_size <= kVertexBlockMaxSize)

    let last_vertex = Array.zeroCreate<byte> 256

    let sizes = Array.zeroCreate<int> 3
    assert (max_channel <= 3)

    let mutable i = 0
    while i < vertex_count do
        let block_size = if i + vertex_block_size < vertex_count then vertex_block_size else vertex_count - i
        let block_size_aligned = (block_size + kByteGroupSize - 1) &&& ~~~(kByteGroupSize - 1)

        let srcOff = if i = 0 then 0 else i - 1
        use lastPin = fixed last_vertex
        NPtr.memcpy lastPin (NPtr.add vertex_data (srcOff * vertex_size)) vertex_size

        // we sometimes encode elements we didn't fill when rounding to kByteGroupSize
        if block_size < block_size_aligned then
            for idx = block_size to block_size_aligned - 1 do
                block.[idx] <- 0uy

        for channel = 0 to max_channel - 1 do
            for j = 0 to 3 do
                use blockPin = fixed block
                use lastPin = fixed last_vertex
                encodeDeltas blockPin (NPtr.add vertex_data (i * vertex_size)) block_size vertex_size lastPin (k + j) (channel ||| (xor_rot <<< 4))

                let mutable ig = 0
                while ig < block_size do
                    // to maximize encoding performance we only evaluate 1/2/4/8 bit groups
                    let size1 = encodeBytesGroupMeasure (NPtr.add blockPin ig) 1
                    let size2 = encodeBytesGroupMeasure (NPtr.add blockPin ig) 2
                    let size4 = encodeBytesGroupMeasure (NPtr.add blockPin ig) 4
                    let size8 = encodeBytesGroupMeasure (NPtr.add blockPin ig) 8

                    let mutable best_size = if size1 < size2 then size1 else size2
                    best_size <- if best_size < size4 then best_size else size4
                    best_size <- if best_size < size8 then best_size else size8

                    sizes.[channel] <- sizes.[channel] + best_size
                    ig <- ig + kByteGroupSize

        i <- i + vertex_block_size * block_skip

    let mutable best_channel = 0
    for channel = 1 to max_channel - 1 do
        best_channel <- if sizes.[channel] < sizes.[best_channel] then channel else best_channel

    if best_channel = 2 then best_channel ||| (xor_rot <<< 4) else best_channel

let private estimateControlZero (buffer: nativeptr<byte>) (vertex_count_aligned: int) : bool =
    let mutable i = 0
    let mutable allZero = true
    while i < vertex_count_aligned && allZero do
        if not (encodeBytesGroupZero (NPtr.add buffer i)) then
            allZero <- false
        i <- i + kByteGroupSize
    allZero

let private estimateControl (buffer: nativeptr<byte>) (vertex_count: int) (vertex_count_aligned: int) (level: int) : int =
    if estimateControlZero buffer vertex_count_aligned then
        2 // zero encoding
    elif level = 0 then
        1 // 1248 encoding in level 0 for encoding speed
    else

    // round number of groups to 4 to get number of header bytes
    let header_size = (vertex_count_aligned / kByteGroupSize + 3) / 4

    let mutable est_bytes0 = header_size
    let mutable est_bytes1 = header_size

    let mutable i = 0
    while i < vertex_count_aligned do
        // assumes kBitsV1[] = {0, 1, 2, 4, 8} for performance
        let size0 = encodeBytesGroupMeasure (NPtr.add buffer i) 0
        let size1 = encodeBytesGroupMeasure (NPtr.add buffer i) 1
        let size2 = encodeBytesGroupMeasure (NPtr.add buffer i) 2
        let size4 = encodeBytesGroupMeasure (NPtr.add buffer i) 4
        let size8 = encodeBytesGroupMeasure (NPtr.add buffer i) 8

        // both control modes have access to 1/2/4 bit encoding
        let size12 = if size1 < size2 then size1 else size2
        let size124 = if size12 < size4 then size12 else size4

        // each control mode has access to 0/8 bit encoding respectively
        est_bytes0 <- est_bytes0 + (if size124 < size0 then size124 else size0)
        est_bytes1 <- est_bytes1 + (if size124 < size8 then size124 else size8)

        i <- i + kByteGroupSize

    // pick shortest control entry but prefer literal encoding
    if est_bytes0 < vertex_count || est_bytes1 < vertex_count then
        if est_bytes0 < est_bytes1 then 0 else 1
    else
        3 // literal encoding

let private encodeVertexBlock (data: nativeptr<byte>) (dataOff: byref<int>) (data_end: int) (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (channels: byte[]) (version: int) (level: int) : bool =
    assert (vertex_count > 0 && vertex_count <= kVertexBlockMaxSize)
    assert (vertex_size % 4 = 0)

    let buffer = Array.zeroCreate<byte> kVertexBlockMaxSize
    assert (buffer.Length % kByteGroupSize = 0)

    let vertex_count_aligned = (vertex_count + kByteGroupSize - 1) &&& ~~~(kByteGroupSize - 1)

    let control_size = if version = 0 then 0 else vertex_size / 4
    if data_end - dataOff < control_size then
        false
    else

    let controlStart = dataOff
    dataOff <- dataOff + control_size

    NPtr.memset (NPtr.add data controlStart) 0uy control_size

    let mutable k = 0
    let mutable ok = true
    while k < vertex_size && ok do
        use bufPin = fixed buffer
        let ch = if version = 0 then 0 else int channels.[k / 4]
        encodeDeltas bufPin vertex_data vertex_count vertex_size last_vertex k ch

        let mutable ctrl = 0

        if version <> 0 then
            ctrl <- estimateControl bufPin vertex_count vertex_count_aligned level

            assert (uint32 ctrl < 4u)
            let hdr = NPtr.get data (controlStart + k / 4)
            NPtr.set data (controlStart + k / 4) (hdr ||| byte (ctrl <<< ((k % 4) * 2)))

        if ctrl = 3 then
            // literal encoding
            if data_end - dataOff < vertex_count then
                ok <- false
            else
                NPtr.memcpy (NPtr.add data dataOff) bufPin vertex_count
                dataOff <- dataOff + vertex_count
        elif ctrl <> 2 then // non-zero encoding
            let bitsArr = if version = 0 then kBitsV0 else kBitsV1.[ctrl..]
            if not (encodeBytes data &dataOff data_end bufPin vertex_count_aligned bitsArr) then
                ok <- false

        k <- k + 1

    if ok then
        NPtr.memcpy last_vertex (NPtr.add vertex_data (vertex_size * (vertex_count - 1))) vertex_size

    ok

// ------- Scalar decode helpers -------

let inline private decodeBytesGroupNext (data_var: byref<nativeptr<byte>>) (buffer: byref<nativeptr<byte>>) (byte_: byref<byte>) (bits: int) =
    let enc = byte_ >>> (8 - bits)
    byte_ <- byte_ <<< bits
    let encv = NPtr.get data_var 0
    let sentinel = byte ((1 <<< bits) - 1)
    NPtr.set buffer 0 (if enc = sentinel then encv else enc)
    buffer <- NPtr.add buffer 1
    if enc = sentinel then
        data_var <- NPtr.add data_var 1

let private decodeBytesGroup (data: nativeptr<byte>) (dataOff: byref<int>) (buffer: nativeptr<byte>) (bits: int) =
    match bits with
    | 0 ->
        NPtr.memset buffer 0uy kByteGroupSize
    | 1 ->
        let mutable data_var = NPtr.add data (dataOff + 2)
        let mutable buf = buffer

        // 2 groups with 8 1-bit values in each byte (reversed from the order in other groups)
        let mutable byte_ = NPtr.get data dataOff
        byte_ <- byte (((uint64 byte_ * 0x80200802UL) &&& 0x0884422110UL) * 0x0101010101UL >>> 32)
        for _ = 0 to 7 do decodeBytesGroupNext &data_var &buf &byte_ 1

        byte_ <- NPtr.get data (dataOff + 1)
        byte_ <- byte (((uint64 byte_ * 0x80200802UL) &&& 0x0884422110UL) * 0x0101010101UL >>> 32)
        for _ = 0 to 7 do decodeBytesGroupNext &data_var &buf &byte_ 1

        dataOff <- int (NPtr.toNI data_var - NPtr.toNI data)
    | 2 ->
        let mutable data_var = NPtr.add data (dataOff + 4)
        let mutable buf = buffer

        // 4 groups with 4 2-bit values in each byte
        for g = 0 to 3 do
            let mutable byte_ = NPtr.get data (dataOff + g)
            for _ = 0 to 3 do decodeBytesGroupNext &data_var &buf &byte_ 2

        dataOff <- int (NPtr.toNI data_var - NPtr.toNI data)
    | 4 ->
        let mutable data_var = NPtr.add data (dataOff + 8)
        let mutable buf = buffer

        // 8 groups with 2 4-bit values in each byte
        for g = 0 to 7 do
            let mutable byte_ = NPtr.get data (dataOff + g)
            for _ = 0 to 1 do decodeBytesGroupNext &data_var &buf &byte_ 4

        dataOff <- int (NPtr.toNI data_var - NPtr.toNI data)
    | 8 ->
        NPtr.memcpy buffer (NPtr.add data dataOff) kByteGroupSize
        dataOff <- dataOff + kByteGroupSize
    | _ ->
        assert false // unreachable

let private decodeBytes (data: nativeptr<byte>) (dataOff: byref<int>) (data_end: int) (buffer: nativeptr<byte>) (buffer_size: int) (bits: int[]) : bool =
    assert (buffer_size % kByteGroupSize = 0)

    // round number of groups to 4 to get number of header bytes
    let header_size = (buffer_size / kByteGroupSize + 3) / 4
    if data_end - dataOff < header_size then
        false
    else

    let headerStart = dataOff
    dataOff <- dataOff + header_size

    let mutable i = 0
    let mutable ok = true
    while i < buffer_size && ok do
        if data_end - dataOff < kByteGroupDecodeLimit then
            ok <- false
        else

        let header_offset = i / kByteGroupSize
        let bitsk = (int (NPtr.get data (headerStart + header_offset / 4)) >>> ((header_offset % 4) * 2)) &&& 3

        decodeBytesGroup data &dataOff (NPtr.add buffer i) bits.[bitsk]
        i <- i + kByteGroupSize

    ok

let private decodeDeltas1_byte (buffer: nativeptr<byte>) (transposed: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) =
    for k = 0 to 3 do
        let mutable vertex_offset = k
        let mutable p = NPtr.get last_vertex k

        for i = 0 to vertex_count - 1 do
            let v = NPtr.get buffer (i + vertex_count * k)
            let v = unzigzagByte v + p
            NPtr.set transposed (vertex_offset) v
            p <- v
            vertex_offset <- vertex_offset + vertex_size

let private decodeDeltas1_ushort (buffer: nativeptr<byte>) (transposed: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) =
    let mutable kk = 0
    while kk < 4 do
        let mutable vertex_offset = kk
        let mutable p = uint16 (NPtr.get last_vertex 0) ||| (uint16 (NPtr.get last_vertex 1) <<< 8)
        let last_vert = NPtr.add last_vertex 2

        for i = 0 to vertex_count - 1 do
            let mutable v = uint16 (NPtr.get buffer i)
            v <- v ||| (uint16 (NPtr.get buffer (i + vertex_count)) <<< 8)
            v <- unzigzagUInt16 v + p
            NPtr.set transposed (vertex_offset) (byte v)
            NPtr.set transposed (vertex_offset + 1) (byte (v >>> 8))
            p <- v
            vertex_offset <- vertex_offset + vertex_size

        // This was a template over T=unsigned short, sizeof(T)=2, loop k=0..3 step 2
        // We need to advance buffer by vertex_count * 2 and last_vertex by 2
        // But we only loop once (k=0, stepping by 2, then k=2 next)
        kk <- kk + 2 // sizeof(unsigned short) = 2

let private decodeDeltas1_uint_xor (buffer: nativeptr<byte>) (transposed: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (rot: int) =
    // sizeof(unsigned int) = 4, so loop: k=0, step 4, runs once
    let mutable vertex_offset = 0
    let mutable p =
        uint32 (NPtr.get last_vertex 0)
        ||| (uint32 (NPtr.get last_vertex 1) <<< 8)
        ||| (uint32 (NPtr.get last_vertex 2) <<< 16)
        ||| (uint32 (NPtr.get last_vertex 3) <<< 24)

    for i = 0 to vertex_count - 1 do
        let mutable v = uint32 (NPtr.get buffer i)
        v <- v ||| (uint32 (NPtr.get buffer (i + vertex_count)) <<< 8)
        v <- v ||| (uint32 (NPtr.get buffer (i + vertex_count * 2)) <<< 16)
        v <- v ||| (uint32 (NPtr.get buffer (i + vertex_count * 3)) <<< 24)
        v <- rotate v rot ^^^ p
        NPtr.set transposed (vertex_offset) (byte v)
        NPtr.set transposed (vertex_offset + 1) (byte (v >>> 8))
        NPtr.set transposed (vertex_offset + 2) (byte (v >>> 16))
        NPtr.set transposed (vertex_offset + 3) (byte (v >>> 24))
        p <- v
        vertex_offset <- vertex_offset + vertex_size

let private decodeVertexBlockScalar (data: nativeptr<byte>) (dataOff: byref<int>) (data_end: int) (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (channels: nativeptr<byte>) (version: int) : bool =
    assert (vertex_count > 0 && vertex_count <= kVertexBlockMaxSize)

    let buffer = Array.zeroCreate<byte> (kVertexBlockMaxSize * 4)
    let transposed = Array.zeroCreate<byte> kVertexBlockSizeBytes

    let vertex_count_aligned = (vertex_count + kByteGroupSize - 1) &&& ~~~(kByteGroupSize - 1)
    assert (vertex_count <= vertex_count_aligned)

    let control_size = if version = 0 then 0 else vertex_size / 4
    if data_end - dataOff < control_size then
        false
    else

    let controlStart = dataOff
    dataOff <- dataOff + control_size

    let mutable k = 0
    let mutable ok = true
    while k < vertex_size && ok do
        let ctrl_byte = if version = 0 then 0uy else NPtr.get data (controlStart + k / 4)

        for j = 0 to 3 do
            if ok then
                let ctrl = (int ctrl_byte >>> (j * 2)) &&& 3

                use bufPin = fixed buffer

                if ctrl = 3 then
                    // literal encoding
                    if data_end - dataOff < vertex_count then
                        ok <- false
                    else
                        NPtr.memcpy (NPtr.add bufPin (j * vertex_count)) (NPtr.add data dataOff) vertex_count
                        dataOff <- dataOff + vertex_count
                elif ctrl = 2 then
                    // zero encoding
                    NPtr.memset (NPtr.add bufPin (j * vertex_count)) 0uy vertex_count
                else
                    let bitsArr = if version = 0 then kBitsV0 else kBitsV1.[ctrl..]
                    if not (decodeBytes data &dataOff data_end (NPtr.add bufPin (j * vertex_count)) vertex_count_aligned bitsArr) then
                        ok <- false

        if ok then
            let channel = if version = 0 then 0 else int (NPtr.get channels (k / 4))

            use bufPin = fixed buffer
            use trPin = fixed transposed

            match channel &&& 3 with
            | 0 ->
                decodeDeltas1_byte bufPin (NPtr.add trPin k) vertex_count vertex_size (NPtr.add last_vertex k)
            | 1 ->
                decodeDeltas1_ushort bufPin (NPtr.add trPin k) vertex_count vertex_size (NPtr.add last_vertex k)
            | 2 ->
                decodeDeltas1_uint_xor bufPin (NPtr.add trPin k) vertex_count vertex_size (NPtr.add last_vertex k) ((32 - (channel >>> 4)) &&& 31)
            | _ ->
                ok <- false // invalid channel type

        k <- k + 4

    if ok then
        use trPin = fixed transposed
        NPtr.memcpy vertex_data trPin (vertex_count * vertex_size)
        NPtr.memcpy last_vertex (NPtr.add trPin (vertex_size * (vertex_count - 1))) vertex_size

    ok

// ------- SIMD decode tables (used by SSE/SSSE3 and NEON paths) -------

let private kDecodeBytesGroupShuffle : byte[][] =
    Array.init 256 (fun mask ->
        let shuffle = Array.zeroCreate<byte> 8
        let mutable count = 0uy
        for i = 0 to 7 do
            let maski = (mask >>> i) &&& 1
            shuffle.[i] <- if maski <> 0 then count else 0x80uy
            count <- count + byte maski
        shuffle
    )

let private kDecodeBytesGroupCount : byte[] =
    Array.init 256 (fun mask ->
        let mutable count = 0uy
        for i = 0 to 7 do
            count <- count + byte ((mask >>> i) &&& 1)
        count
    )

// ------- SSE/SSSE3 SIMD decode path -------

let private decodeShuffleMaskSse (mask0: byte) (mask1: byte) : Vector128<byte> =
    // Load 8 bytes from shuffle tables into low 64 bits of vector
    let sm0arr = kDecodeBytesGroupShuffle.[int mask0]
    let sm1arr = kDecodeBytesGroupShuffle.[int mask1]
    let sm1off = kDecodeBytesGroupCount.[int mask0]

    // Build 16-byte array for the combined shuffle mask
    let combined = Array.zeroCreate<byte> 16
    Array.Copy(sm0arr, 0, combined, 0, 8)
    for i = 0 to 7 do
        combined.[8 + i] <- sm1arr.[i] + sm1off
    use pin = fixed combined
    Sse2.LoadVector128(pin)

let private decodeBytesGroupSimdSse (data: nativeptr<byte>) (dataOff: byref<int>) (buffer: nativeptr<byte>) (hbits: int) =
    match hbits with
    | 0 | 4 ->
        let result = Vector128<byte>.Zero
        Sse2.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)

    | 1 | 6 ->
        // latency opt: compute count from data
        let mutable data32 = uint32 (NPtr.get data dataOff) ||| (uint32 (NPtr.get data (dataOff+1)) <<< 8) ||| (uint32 (NPtr.get data (dataOff+2)) <<< 16) ||| (uint32 (NPtr.get data (dataOff+3)) <<< 24)
        data32 <- data32 &&& (data32 >>> 1)
        let data64 = (uint64 data32 <<< 30) ||| (uint64 data32 &&& 0x3fffffffUL)
        let datacnt = int ((data64 &&& 0x1111111111111111UL) * 0x1111111111111111UL >>> 60)

        let sel2 = Sse2.ConvertScalarToVector128Int32(NPtr.get (NPtr.cast<byte, int> (NPtr.add data dataOff)) 0).AsByte()
        let rest = Sse2.LoadVector128(NPtr.ofNI<byte> (NPtr.toNI (NPtr.add data (dataOff + 4))))

        let sel22 = Sse2.UnpackLow(Sse2.ShiftRightLogical(sel2.AsInt16(), 4uy).AsByte(), sel2)
        let sel2222 = Sse2.UnpackLow(Sse2.ShiftRightLogical(sel22.AsInt16(), 2uy).AsByte(), sel22)
        let sel = Sse2.And(sel2222, Vector128.Create(3uy))

        let mask = Sse2.CompareEqual(sel, Vector128.Create(3uy))
        let mask16 = Sse2.MoveMask(mask)
        let mask0 = byte (mask16 &&& 255)
        let mask1 = byte (mask16 >>> 8)

        let shuf = decodeShuffleMaskSse mask0 mask1
        let result = Sse2.Or(Ssse3.Shuffle(rest, shuf), Sse2.AndNot(mask, sel))

        Sse2.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)

        dataOff <- dataOff + 4 + datacnt

    | 2 | 7 ->
        // latency opt: compute count from data
        let mutable data64 =
            uint64 (NPtr.get data dataOff)
            ||| (uint64 (NPtr.get data (dataOff+1)) <<< 8)
            ||| (uint64 (NPtr.get data (dataOff+2)) <<< 16)
            ||| (uint64 (NPtr.get data (dataOff+3)) <<< 24)
            ||| (uint64 (NPtr.get data (dataOff+4)) <<< 32)
            ||| (uint64 (NPtr.get data (dataOff+5)) <<< 40)
            ||| (uint64 (NPtr.get data (dataOff+6)) <<< 48)
            ||| (uint64 (NPtr.get data (dataOff+7)) <<< 56)
        data64 <- data64 &&& (data64 >>> 1)
        data64 <- data64 &&& (data64 >>> 2)
        let datacnt = int ((data64 &&& 0x1111111111111111UL) * 0x1111111111111111UL >>> 60)

        // _mm_loadl_epi64 loads 8 bytes into lower 64 bits
        let sel4 = Sse2.LoadScalarVector128(NPtr.cast<byte, uint64> (NPtr.add data dataOff)).AsByte()
        let rest = Sse2.LoadVector128(NPtr.ofNI<byte> (NPtr.toNI (NPtr.add data (dataOff + 8))))

        let sel44 : Vector128<byte> = Sse2.UnpackLow(Sse2.ShiftRightLogical(sel4.AsInt16(), 4uy).AsByte(), sel4)
        let sel = Sse2.And(sel44, Vector128.Create(15uy))

        let mask = Sse2.CompareEqual(sel, Vector128.Create(15uy))
        let mask16 = Sse2.MoveMask(mask)
        let mask0 = byte (mask16 &&& 255)
        let mask1 = byte (mask16 >>> 8)

        let shuf = decodeShuffleMaskSse mask0 mask1
        let result = Sse2.Or(Ssse3.Shuffle(rest, shuf), Sse2.AndNot(mask, sel))

        Sse2.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)

        dataOff <- dataOff + 8 + datacnt

    | 3 | 8 ->
        let result = Sse2.LoadVector128(NPtr.ofNI<byte> (NPtr.toNI (NPtr.add data dataOff)))
        Sse2.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)
        dataOff <- dataOff + 16

    | 5 ->
        let rest = Sse2.LoadVector128(NPtr.ofNI<byte> (NPtr.toNI (NPtr.add data (dataOff + 2))))
        let mask0 = NPtr.get data dataOff
        let mask1 = NPtr.get data (dataOff + 1)

        let shuf = decodeShuffleMaskSse mask0 mask1
        let result = Ssse3.Shuffle(rest, shuf)

        Sse2.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)
        dataOff <- dataOff + 2 + int kDecodeBytesGroupCount.[int mask0] + int kDecodeBytesGroupCount.[int mask1]

    | _ -> assert false // unreachable

// ------- NEON SIMD decode path -------

let private neonMoveMask (mask: Vector128<byte>) : struct(byte * byte) =
    // magic constant found using z3 SMT assuming mask has 8 groups of 0xff or 0x00
    let magic = 0x000103070f1f3f80UL
    let mask2 = mask.AsUInt64()
    let m0 = byte ((mask2.GetElement(0) * magic) >>> 56)
    let m1 = byte ((mask2.GetElement(1) * magic) >>> 56)
    struct(m0, m1)

let private shuffleBytesNeon (mask0: byte) (mask1: byte) (rest0: Vector64<byte>) (rest1: Vector64<byte>) : Vector128<byte> =
    let sm0arr = kDecodeBytesGroupShuffle.[int mask0]
    let sm1arr = kDecodeBytesGroupShuffle.[int mask1]
    use sm0Pin = fixed sm0arr
    use sm1Pin = fixed sm1arr
    let sm0 = AdvSimd.LoadVector64(sm0Pin)
    let sm1 = AdvSimd.LoadVector64(sm1Pin)

    // vtbl1_u8 equivalent: expand 64-bit table to 128, lookup, take lower half
    let rest0_128 = Vector128.Create(rest0, Vector64<byte>.Zero)
    let rest1_128 = Vector128.Create(rest1, Vector64<byte>.Zero)
    let sm0_128 = Vector128.Create(sm0, Vector64.Create(0x80uy))
    let sm1_128 = Vector128.Create(sm1, Vector64.Create(0x80uy))
    let r0 = AdvSimd.Arm64.VectorTableLookup(rest0_128, sm0_128).GetLower()
    let r1 = AdvSimd.Arm64.VectorTableLookup(rest1_128, sm1_128).GetLower()

    Vector128.Create(r0, r1)

let private decodeBytesGroupSimdNeon (data: nativeptr<byte>) (dataOff: byref<int>) (buffer: nativeptr<byte>) (hbits: int) =
    match hbits with
    | 0 | 4 ->
        let result = Vector128<byte>.Zero
        AdvSimd.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)

    | 1 | 6 ->
        // latency opt
        let mutable data32 = uint32 (NPtr.get data dataOff) ||| (uint32 (NPtr.get data (dataOff+1)) <<< 8) ||| (uint32 (NPtr.get data (dataOff+2)) <<< 16) ||| (uint32 (NPtr.get data (dataOff+3)) <<< 24)
        data32 <- data32 &&& (data32 >>> 1)
        let data64 = (uint64 data32 <<< 30) ||| (uint64 data32 &&& 0x3fffffffUL)
        let datacnt = int ((data64 &&& 0x1111111111111111UL) * 0x1111111111111111UL >>> 60)

        let sel2 = AdvSimd.LoadVector64(NPtr.add data dataOff)
        let sel22pair = AdvSimd.Arm64.ZipLow(AdvSimd.ShiftRightLogical(sel2, 4uy), sel2)
        let sel2222lo = AdvSimd.Arm64.ZipLow(AdvSimd.ShiftRightLogical(sel22pair, 2uy), sel22pair)
        let sel2222hi = AdvSimd.Arm64.ZipHigh(AdvSimd.ShiftRightLogical(sel22pair, 2uy), sel22pair)
        let sel = AdvSimd.And(Vector128.Create(sel2222lo, sel2222hi), Vector128.Create(3uy))

        let mask = AdvSimd.CompareEqual(sel, Vector128.Create(3uy))
        let struct(mask0, mask1) = neonMoveMask mask

        let rest0 = AdvSimd.LoadVector64(NPtr.add data (dataOff + 4))
        let rest1 = AdvSimd.LoadVector64(NPtr.add data (dataOff + 4 + int kDecodeBytesGroupCount.[int mask0]))

        let shuffled = shuffleBytesNeon mask0 mask1 rest0 rest1
        let result = AdvSimd.BitwiseSelect(mask, shuffled, sel)

        AdvSimd.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)
        dataOff <- dataOff + 4 + datacnt

    | 2 | 7 ->
        // latency opt
        let mutable data64 =
            uint64 (NPtr.get data dataOff)
            ||| (uint64 (NPtr.get data (dataOff+1)) <<< 8)
            ||| (uint64 (NPtr.get data (dataOff+2)) <<< 16)
            ||| (uint64 (NPtr.get data (dataOff+3)) <<< 24)
            ||| (uint64 (NPtr.get data (dataOff+4)) <<< 32)
            ||| (uint64 (NPtr.get data (dataOff+5)) <<< 40)
            ||| (uint64 (NPtr.get data (dataOff+6)) <<< 48)
            ||| (uint64 (NPtr.get data (dataOff+7)) <<< 56)
        data64 <- data64 &&& (data64 >>> 1)
        data64 <- data64 &&& (data64 >>> 2)
        let datacnt = int ((data64 &&& 0x1111111111111111UL) * 0x1111111111111111UL >>> 60)

        let sel4 = AdvSimd.LoadVector64(NPtr.add data dataOff)
        let sel4masked = AdvSimd.And(sel4, Vector64.Create(15uy))
        let sel44lo = AdvSimd.Arm64.ZipLow(AdvSimd.ShiftRightLogical(sel4, 4uy), sel4masked)
        let sel44hi = AdvSimd.Arm64.ZipHigh(AdvSimd.ShiftRightLogical(sel4, 4uy), sel4masked)
        let sel = Vector128.Create(sel44lo, sel44hi)

        let mask = AdvSimd.CompareEqual(sel, Vector128.Create(15uy))
        let struct(mask0, mask1) = neonMoveMask mask

        let rest0 = AdvSimd.LoadVector64(NPtr.add data (dataOff + 8))
        let rest1 = AdvSimd.LoadVector64(NPtr.add data (dataOff + 8 + int kDecodeBytesGroupCount.[int mask0]))

        let shuffled = shuffleBytesNeon mask0 mask1 rest0 rest1
        let result = AdvSimd.BitwiseSelect(mask, shuffled, sel)

        AdvSimd.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)
        dataOff <- dataOff + 8 + datacnt

    | 3 | 8 ->
        let result = AdvSimd.LoadVector128(NPtr.add data dataOff)
        AdvSimd.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)
        dataOff <- dataOff + 16

    | 5 ->
        let mask0 = NPtr.get data dataOff
        let mask1 = NPtr.get data (dataOff + 1)

        let rest0 = AdvSimd.LoadVector64(NPtr.add data (dataOff + 2))
        let rest1 = AdvSimd.LoadVector64(NPtr.add data (dataOff + 2 + int kDecodeBytesGroupCount.[int mask0]))

        let result = shuffleBytesNeon mask0 mask1 rest0 rest1

        AdvSimd.Store(NPtr.ofNI<byte> (NPtr.toNI buffer), result)
        dataOff <- dataOff + 2 + int kDecodeBytesGroupCount.[int mask0] + int kDecodeBytesGroupCount.[int mask1]

    | _ -> assert false // unreachable

// ------- Generic SIMD decode bytes (parameterized by group decoder) -------

let private decodeBytesSimdGeneric (useSse: bool) (data: nativeptr<byte>) (dataOff: byref<int>) (data_end: int) (buffer: nativeptr<byte>) (buffer_size: int) (hshift: int) : bool =
    assert (buffer_size % kByteGroupSize = 0)
    assert (kByteGroupSize = 16)

    // round number of groups to 4 to get number of header bytes
    let header_size = (buffer_size / kByteGroupSize + 3) / 4
    if data_end - dataOff < header_size then
        false
    else

    let headerStart = dataOff
    dataOff <- dataOff + header_size

    let mutable i = 0
    let mutable ok = true

    // fast-path: process 4 groups at a time, do a shared bounds check
    while i + kByteGroupSize * 4 <= buffer_size && data_end - dataOff >= kByteGroupDecodeLimit * 4 && ok do
        let header_offset = i / kByteGroupSize
        let header_byte = NPtr.get data (headerStart + header_offset / 4)

        if useSse then decodeBytesGroupSimdSse data &dataOff (NPtr.add buffer (i + kByteGroupSize * 0)) (hshift + (int header_byte &&& 3))
        else decodeBytesGroupSimdNeon data &dataOff (NPtr.add buffer (i + kByteGroupSize * 0)) (hshift + (int header_byte &&& 3))
        if useSse then decodeBytesGroupSimdSse data &dataOff (NPtr.add buffer (i + kByteGroupSize * 1)) (hshift + ((int header_byte >>> 2) &&& 3))
        else decodeBytesGroupSimdNeon data &dataOff (NPtr.add buffer (i + kByteGroupSize * 1)) (hshift + ((int header_byte >>> 2) &&& 3))
        if useSse then decodeBytesGroupSimdSse data &dataOff (NPtr.add buffer (i + kByteGroupSize * 2)) (hshift + ((int header_byte >>> 4) &&& 3))
        else decodeBytesGroupSimdNeon data &dataOff (NPtr.add buffer (i + kByteGroupSize * 2)) (hshift + ((int header_byte >>> 4) &&& 3))
        if useSse then decodeBytesGroupSimdSse data &dataOff (NPtr.add buffer (i + kByteGroupSize * 3)) (hshift + ((int header_byte >>> 6) &&& 3))
        else decodeBytesGroupSimdNeon data &dataOff (NPtr.add buffer (i + kByteGroupSize * 3)) (hshift + ((int header_byte >>> 6) &&& 3))

        i <- i + kByteGroupSize * 4

    // slow-path: process remaining groups
    while i < buffer_size && ok do
        if data_end - dataOff < kByteGroupDecodeLimit then
            ok <- false
        else

        let header_offset = i / kByteGroupSize
        let header_byte = NPtr.get data (headerStart + header_offset / 4)

        if useSse then decodeBytesGroupSimdSse data &dataOff (NPtr.add buffer i) (hshift + ((int header_byte >>> ((header_offset % 4) * 2)) &&& 3))
        else decodeBytesGroupSimdNeon data &dataOff (NPtr.add buffer i) (hshift + ((int header_byte >>> ((header_offset % 4) * 2)) &&& 3))
        i <- i + kByteGroupSize

    ok

// ------- SSE/SSSE3 transpose & delta helpers -------

let private transpose8Sse (x0: byref<Vector128<byte>>) (x1: byref<Vector128<byte>>) (x2: byref<Vector128<byte>>) (x3: byref<Vector128<byte>>) =
    let t0 = Sse2.UnpackLow(x0, x1)
    let t1 = Sse2.UnpackHigh(x0, x1)
    let t2 = Sse2.UnpackLow(x2, x3)
    let t3 = Sse2.UnpackHigh(x2, x3)

    x0 <- Sse2.UnpackLow(t0.AsInt16(), t2.AsInt16()).AsByte()
    x1 <- Sse2.UnpackHigh(t0.AsInt16(), t2.AsInt16()).AsByte()
    x2 <- Sse2.UnpackLow(t1.AsInt16(), t3.AsInt16()).AsByte()
    x3 <- Sse2.UnpackHigh(t1.AsInt16(), t3.AsInt16()).AsByte()

let private unzigzag8Sse (v: Vector128<byte>) : Vector128<byte> =
    let xl = Sse2.Subtract(Vector128<byte>.Zero.AsSByte(), Sse2.And(v, Vector128.Create(1uy)).AsSByte()).AsByte()
    let xr = Sse2.And(Sse2.ShiftRightLogical(v.AsInt16(), 1uy).AsByte(), Vector128.Create(127uy))
    Sse2.Xor(xl, xr)

let private unzigzag16Sse (v: Vector128<byte>) : Vector128<byte> =
    let xl = Sse2.Subtract(Vector128<int16>.Zero, Sse2.And(v.AsInt16(), Vector128.Create(1s))).AsByte()
    let xr = Sse2.ShiftRightLogical(v.AsUInt16(), 1uy).AsByte()
    Sse2.Xor(xl, xr)

let private rotate32Sse (v: Vector128<byte>) (r: int) : Vector128<byte> =
    Sse2.Or(Sse2.ShiftLeftLogical(v.AsInt32(), byte r), Sse2.ShiftRightLogical(v.AsInt32(), byte (32 - r))).AsByte()

let private decodeDeltas4SimdSse (channel: int) (buffer: nativeptr<byte>) (transposed: nativeptr<byte>) (vertex_count_aligned: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (rot: int) =
    let mutable pi = Sse2.ConvertScalarToVector128Int32(NPtr.get (NPtr.cast<byte, int> last_vertex) 0).AsByte()

    let mutable savep = transposed
    let mutable j = 0

    while j < vertex_count_aligned do
        let mutable r0 = Sse2.LoadVector128(NPtr.ofNI<byte> (NPtr.toNI (NPtr.add buffer (j + 0 * vertex_count_aligned))))
        let mutable r1 = Sse2.LoadVector128(NPtr.ofNI<byte> (NPtr.toNI (NPtr.add buffer (j + 1 * vertex_count_aligned))))
        let mutable r2 = Sse2.LoadVector128(NPtr.ofNI<byte> (NPtr.toNI (NPtr.add buffer (j + 2 * vertex_count_aligned))))
        let mutable r3 = Sse2.LoadVector128(NPtr.ofNI<byte> (NPtr.toNI (NPtr.add buffer (j + 3 * vertex_count_aligned))))

        transpose8Sse &r0 &r1 &r2 &r3

        // Process each of r0..r3
        let inline processReg (r: Vector128<byte>) =
            let r =
                if channel = 0 then unzigzag8Sse r
                elif channel = 1 then unzigzag16Sse r
                else rotate32Sse r rot

            // GRP4: extract 4 dwords
            let t0 = r
            let t1 = Sse2.Shuffle(r.AsInt32(), 1uy).AsByte()
            let t2 = Sse2.Shuffle(r.AsInt32(), 2uy).AsByte()
            let t3 = Sse2.Shuffle(r.AsInt32(), 3uy).AsByte()

            // FIXD: accumulate prefix sum
            let inline fix (t: Vector128<byte>) =
                let result =
                    if channel = 0 then Sse2.Add(pi.AsSByte(), t.AsSByte()).AsByte()
                    elif channel = 1 then Sse2.Add(pi.AsInt16(), t.AsInt16()).AsByte()
                    else Sse2.Xor(pi, t)
                pi <- result
                result

            let ft0 = fix t0
            let ft1 = fix t1
            let ft2 = fix t2
            let ft3 = fix t3

            // SAVE: store low 32 bits
            NPtr.set (NPtr.cast<byte, int> savep) 0 (Sse2.ConvertToInt32(ft0.AsInt32()))
            savep <- NPtr.add savep vertex_size
            NPtr.set (NPtr.cast<byte, int> savep) 0 (Sse2.ConvertToInt32(ft1.AsInt32()))
            savep <- NPtr.add savep vertex_size
            NPtr.set (NPtr.cast<byte, int> savep) 0 (Sse2.ConvertToInt32(ft2.AsInt32()))
            savep <- NPtr.add savep vertex_size
            NPtr.set (NPtr.cast<byte, int> savep) 0 (Sse2.ConvertToInt32(ft3.AsInt32()))
            savep <- NPtr.add savep vertex_size

        processReg r0
        processReg r1
        processReg r2
        processReg r3

        j <- j + 16

// ------- NEON transpose & delta helpers -------

let private transpose8Neon (x0: byref<Vector128<byte>>) (x1: byref<Vector128<byte>>) (x2: byref<Vector128<byte>>) (x3: byref<Vector128<byte>>) =
    let t01lo = AdvSimd.Arm64.ZipLow(x0, x1)
    let t01hi = AdvSimd.Arm64.ZipHigh(x0, x1)
    let t23lo = AdvSimd.Arm64.ZipLow(x2, x3)
    let t23hi = AdvSimd.Arm64.ZipHigh(x2, x3)

    x0 <- AdvSimd.Arm64.ZipLow(t01lo.AsUInt16(), t23lo.AsUInt16()).AsByte()
    x1 <- AdvSimd.Arm64.ZipHigh(t01lo.AsUInt16(), t23lo.AsUInt16()).AsByte()
    x2 <- AdvSimd.Arm64.ZipLow(t01hi.AsUInt16(), t23hi.AsUInt16()).AsByte()
    x3 <- AdvSimd.Arm64.ZipHigh(t01hi.AsUInt16(), t23hi.AsUInt16()).AsByte()

let private unzigzag8Neon (v: Vector128<byte>) : Vector128<byte> =
    let xl = AdvSimd.Negate(AdvSimd.And(v, Vector128.Create(1uy)).AsSByte()).AsByte()
    let xr = AdvSimd.ShiftRightLogical(v, 1uy)
    AdvSimd.Xor(xl, xr)

let private unzigzag16Neon (v: Vector128<byte>) : Vector128<byte> =
    let vv = v.AsUInt16()
    let xl = AdvSimd.Negate(AdvSimd.And(vv, Vector128.Create(1us)).AsInt16()).AsByte()
    let xr = AdvSimd.ShiftRightLogical(vv, 1uy).AsByte()
    AdvSimd.Xor(xl, xr)

let private rotate32Neon (v: Vector128<byte>) (r: int) : Vector128<byte> =
    let v32 = v.AsUInt32()
    let left = AdvSimd.ShiftLogical(v32, Vector128.Create(r))
    let right = AdvSimd.ShiftLogical(v32, Vector128.Create(r - 32))
    AdvSimd.Or(left, right).AsByte()

let private decodeDeltas4SimdNeon (channel: int) (buffer: nativeptr<byte>) (transposed: nativeptr<byte>) (vertex_count_aligned: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (rot: int) =
    // Load last_vertex as 4-byte value into low 32 bits of a 64-bit vector
    let mutable pi = AdvSimd.LoadVector64(last_vertex)

    let mutable savep = transposed
    let mutable j = 0

    while j < vertex_count_aligned do
        let mutable r0 = AdvSimd.LoadVector128(NPtr.add buffer (j + 0 * vertex_count_aligned))
        let mutable r1 = AdvSimd.LoadVector128(NPtr.add buffer (j + 1 * vertex_count_aligned))
        let mutable r2 = AdvSimd.LoadVector128(NPtr.add buffer (j + 2 * vertex_count_aligned))
        let mutable r3 = AdvSimd.LoadVector128(NPtr.add buffer (j + 3 * vertex_count_aligned))

        transpose8Neon &r0 &r1 &r2 &r3

        let inline processReg (r: Vector128<byte>) =
            let r =
                if channel = 0 then unzigzag8Neon r
                elif channel = 1 then unzigzag16Neon r
                else rotate32Neon r rot

            // GRP4: extract 4 dwords as Vector64<byte>
            let t0 = r.GetLower()
            let t1 = Vector64.Create(r.GetLower().AsUInt32().GetElement(1)).AsByte()
            let t2 = r.GetUpper()
            let t3 = Vector64.Create(r.GetUpper().AsUInt32().GetElement(1)).AsByte()

            // FIXD: accumulate prefix sum with 64-bit vector
            let inline fix (t: Vector64<byte>) =
                let result =
                    if channel = 0 then AdvSimd.Add(pi, t)
                    elif channel = 1 then AdvSimd.Add(pi.AsUInt16(), t.AsUInt16()).AsByte()
                    else AdvSimd.Xor(pi, t)
                pi <- result
                result

            let ft0 = fix t0
            let ft1 = fix t1
            let ft2 = fix t2
            let ft3 = fix t3

            // SAVE: store low 32 bits
            NPtr.set (NPtr.cast<byte, uint32> savep) 0 (ft0.AsUInt32().GetElement(0))
            savep <- NPtr.add savep vertex_size
            NPtr.set (NPtr.cast<byte, uint32> savep) 0 (ft1.AsUInt32().GetElement(0))
            savep <- NPtr.add savep vertex_size
            NPtr.set (NPtr.cast<byte, uint32> savep) 0 (ft2.AsUInt32().GetElement(0))
            savep <- NPtr.add savep vertex_size
            NPtr.set (NPtr.cast<byte, uint32> savep) 0 (ft3.AsUInt32().GetElement(0))
            savep <- NPtr.add savep vertex_size

        processReg r0
        processReg r1
        processReg r2
        processReg r3

        j <- j + 16

// ------- SIMD decode vertex block (SSE path) -------

let private decodeVertexBlockSimdSse (data: nativeptr<byte>) (dataOff: byref<int>) (data_end: int) (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (channels: nativeptr<byte>) (version: int) : bool =
    assert (vertex_count > 0 && vertex_count <= kVertexBlockMaxSize)

    let buffer = Array.zeroCreate<byte> (kVertexBlockMaxSize * 4)
    let transposed = Array.zeroCreate<byte> kVertexBlockSizeBytes

    let vertex_count_aligned = (vertex_count + kByteGroupSize - 1) &&& ~~~(kByteGroupSize - 1)

    let control_size = if version = 0 then 0 else vertex_size / 4
    if data_end - dataOff < control_size then
        false
    else

    let controlStart = dataOff
    dataOff <- dataOff + control_size

    let mutable k = 0
    let mutable ok = true
    while k < vertex_size && ok do
        let ctrl_byte = if version = 0 then 0uy else NPtr.get data (controlStart + k / 4)

        for j = 0 to 3 do
            if ok then
                let ctrl = (int ctrl_byte >>> (j * 2)) &&& 3

                use bufPin = fixed buffer

                if ctrl = 3 then
                    // literal encoding; safe to over-copy due to tail
                    if data_end - dataOff < vertex_count_aligned then
                        ok <- false
                    else
                        NPtr.memcpy (NPtr.add bufPin (j * vertex_count_aligned)) (NPtr.add data dataOff) vertex_count_aligned
                        dataOff <- dataOff + vertex_count
                elif ctrl = 2 then
                    // zero encoding
                    NPtr.memset (NPtr.add bufPin (j * vertex_count_aligned)) 0uy vertex_count_aligned
                else
                    // for v0, headers are mapped to 0..3; for v1, headers are mapped to 4..8
                    let hshift = if version = 0 then 0 else 4 + ctrl

                    if not (decodeBytesSimdGeneric true data &dataOff data_end (NPtr.add bufPin (j * vertex_count_aligned)) vertex_count_aligned hshift) then
                        ok <- false

        if ok then
            let channel = if version = 0 then 0 else int (NPtr.get channels (k / 4))

            use bufPin = fixed buffer
            use trPin = fixed transposed

            match channel &&& 3 with
            | 0 ->
                decodeDeltas4SimdSse 0 bufPin (NPtr.add trPin k) vertex_count_aligned vertex_size (NPtr.add last_vertex k) 0
            | 1 ->
                decodeDeltas4SimdSse 1 bufPin (NPtr.add trPin k) vertex_count_aligned vertex_size (NPtr.add last_vertex k) 0
            | 2 ->
                decodeDeltas4SimdSse 2 bufPin (NPtr.add trPin k) vertex_count_aligned vertex_size (NPtr.add last_vertex k) ((32 - (channel >>> 4)) &&& 31)
            | _ ->
                ok <- false // invalid channel type

        k <- k + 4

    if ok then
        use trPin = fixed transposed
        NPtr.memcpy vertex_data trPin (vertex_count * vertex_size)
        NPtr.memcpy last_vertex (NPtr.add trPin (vertex_size * (vertex_count - 1))) vertex_size

    ok

// ------- SIMD decode vertex block (NEON path) -------

let private decodeVertexBlockSimdNeon (data: nativeptr<byte>) (dataOff: byref<int>) (data_end: int) (vertex_data: nativeptr<byte>) (vertex_count: int) (vertex_size: int) (last_vertex: nativeptr<byte>) (channels: nativeptr<byte>) (version: int) : bool =
    assert (vertex_count > 0 && vertex_count <= kVertexBlockMaxSize)

    let buffer = Array.zeroCreate<byte> (kVertexBlockMaxSize * 4)
    let transposed = Array.zeroCreate<byte> kVertexBlockSizeBytes

    let vertex_count_aligned = (vertex_count + kByteGroupSize - 1) &&& ~~~(kByteGroupSize - 1)

    let control_size = if version = 0 then 0 else vertex_size / 4
    if data_end - dataOff < control_size then
        false
    else

    let controlStart = dataOff
    dataOff <- dataOff + control_size

    let mutable k = 0
    let mutable ok = true
    while k < vertex_size && ok do
        let ctrl_byte = if version = 0 then 0uy else NPtr.get data (controlStart + k / 4)

        for j = 0 to 3 do
            if ok then
                let ctrl = (int ctrl_byte >>> (j * 2)) &&& 3

                use bufPin = fixed buffer

                if ctrl = 3 then
                    if data_end - dataOff < vertex_count_aligned then
                        ok <- false
                    else
                        NPtr.memcpy (NPtr.add bufPin (j * vertex_count_aligned)) (NPtr.add data dataOff) vertex_count_aligned
                        dataOff <- dataOff + vertex_count
                elif ctrl = 2 then
                    NPtr.memset (NPtr.add bufPin (j * vertex_count_aligned)) 0uy vertex_count_aligned
                else
                    let hshift = if version = 0 then 0 else 4 + ctrl

                    if not (decodeBytesSimdGeneric false data &dataOff data_end (NPtr.add bufPin (j * vertex_count_aligned)) vertex_count_aligned hshift) then
                        ok <- false

        if ok then
            let channel = if version = 0 then 0 else int (NPtr.get channels (k / 4))

            use bufPin = fixed buffer
            use trPin = fixed transposed

            match channel &&& 3 with
            | 0 ->
                decodeDeltas4SimdNeon 0 bufPin (NPtr.add trPin k) vertex_count_aligned vertex_size (NPtr.add last_vertex k) 0
            | 1 ->
                decodeDeltas4SimdNeon 1 bufPin (NPtr.add trPin k) vertex_count_aligned vertex_size (NPtr.add last_vertex k) 0
            | 2 ->
                decodeDeltas4SimdNeon 2 bufPin (NPtr.add trPin k) vertex_count_aligned vertex_size (NPtr.add last_vertex k) ((32 - (channel >>> 4)) &&& 31)
            | _ ->
                ok <- false

        k <- k + 4

    if ok then
        use trPin = fixed transposed
        NPtr.memcpy vertex_data trPin (vertex_count * vertex_size)
        NPtr.memcpy last_vertex (NPtr.add trPin (vertex_size * (vertex_count - 1))) vertex_size

    ok

// ------- Public API -------

let meshopt_encodeVertexBufferLevel (buffer: nativeptr<byte>) (buffer_size: int) (vertices: nativeint) (vertex_count: int) (vertex_size: int) (level: int) (version: int) : int =
    assert (vertex_size > 0 && vertex_size <= 256)
    assert (vertex_size % 4 = 0)
    assert (level >= 0 && level <= 9)
    assert (version < 0 || uint32 version <= uint32 kDecodeVertexVersion)

    let version = if version < 0 then gEncodeVertexVersion else version

    let vertex_data : nativeptr<byte> = NPtr.ofNI vertices

    let mutable dataOff = 0
    let data_end = buffer_size

    if data_end - dataOff < 1 then
        0
    else

    NPtr.set buffer dataOff (kVertexHeader ||| byte version)
    dataOff <- dataOff + 1

    let first_vertex = Array.zeroCreate<byte> 256
    if vertex_count > 0 then
        use fvPin = fixed first_vertex
        NPtr.memcpy fvPin vertex_data vertex_size

    let last_vertex = Array.zeroCreate<byte> 256
    Array.Copy(first_vertex, last_vertex, vertex_size)

    let vertex_block_size = getVertexBlockSize vertex_size

    let channels = Array.zeroCreate<byte> 64
    if version <> 0 && level > 1 && vertex_count > 1 then
        let mutable k = 0
        while k < vertex_size do
            let rot = if level >= 3 then estimateRotate vertex_data vertex_count vertex_size k 16 else 0
            let channel = estimateChannel vertex_data vertex_count vertex_size k vertex_block_size 3 (if level >= 3 then 3 else 2) rot

            assert (uint32 channel < 2u || ((channel &&& 3) = 2 && uint32 (channel >>> 4) < 8u))
            channels.[k / 4] <- byte channel
            k <- k + 4

    let mutable vertex_offset = 0
    let mutable ok = true

    while vertex_offset < vertex_count && ok do
        let block_size = if vertex_offset + vertex_block_size < vertex_count then vertex_block_size else vertex_count - vertex_offset

        use lvPin = fixed last_vertex
        if not (encodeVertexBlock buffer &dataOff data_end (NPtr.add vertex_data (vertex_offset * vertex_size)) block_size vertex_size lvPin channels version level) then
            ok <- false
        else
            vertex_offset <- vertex_offset + block_size

    if not ok then
        0
    else

    let tail_size = vertex_size + (if version = 0 then 0 else vertex_size / 4)
    let tail_size_min = if version = 0 then kTailMinSizeV0 else kTailMinSizeV1
    let tail_size_pad = if tail_size < tail_size_min then tail_size_min else tail_size

    if data_end - dataOff < tail_size_pad then
        0
    else

    if tail_size < tail_size_pad then
        NPtr.memset (NPtr.add buffer dataOff) 0uy (tail_size_pad - tail_size)
        dataOff <- dataOff + (tail_size_pad - tail_size)

    use fvPin = fixed first_vertex
    NPtr.memcpy (NPtr.add buffer dataOff) fvPin vertex_size
    dataOff <- dataOff + vertex_size

    if version <> 0 then
        use chPin = fixed channels
        NPtr.memcpy (NPtr.add buffer dataOff) chPin (vertex_size / 4)
        dataOff <- dataOff + vertex_size / 4

    assert (dataOff >= tail_size)
    assert (dataOff <= buffer_size)

    dataOff

let meshopt_encodeVertexBuffer (buffer: nativeptr<byte>) (buffer_size: int) (vertices: nativeint) (vertex_count: int) (vertex_size: int) : int =
    meshopt_encodeVertexBufferLevel buffer buffer_size vertices vertex_count vertex_size kEncodeDefaultLevel gEncodeVertexVersion

let meshopt_encodeVertexBufferBound (vertex_count: int) (vertex_size: int) : int =
    assert (vertex_size > 0 && vertex_size <= 256)
    assert (vertex_size % 4 = 0)

    let vertex_block_size = getVertexBlockSize vertex_size
    let vertex_block_count = (vertex_count + vertex_block_size - 1) / vertex_block_size

    let vertex_block_control_size = vertex_size / 4
    let vertex_block_header_size = (vertex_block_size / kByteGroupSize + 3) / 4
    let vertex_block_data_size = vertex_block_size

    let tail_size = vertex_size + (vertex_size / 4)
    let tail_size_min = if kTailMinSizeV0 > kTailMinSizeV1 then kTailMinSizeV0 else kTailMinSizeV1
    let tail_size_pad = if tail_size < tail_size_min then tail_size_min else tail_size
    assert (tail_size_pad >= kByteGroupDecodeLimit)

    1 + vertex_block_count * vertex_size * (vertex_block_control_size + vertex_block_header_size + vertex_block_data_size) + tail_size_pad

let meshopt_encodeVertexVersion (version: int) =
    assert (uint32 version <= uint32 kDecodeVertexVersion)
    gEncodeVertexVersion <- version

let meshopt_decodeVertexVersion (buffer: nativeptr<byte>) (buffer_size: int) : int =
    if buffer_size < 1 then
        -1
    else

    let header = NPtr.get buffer 0

    if (header &&& 0xf0uy) <> kVertexHeader then
        -1
    else

    let version = int (header &&& 0x0fuy)
    if version > kDecodeVertexVersion then
        -1
    else

    version

let meshopt_decodeVertexBuffer (destination: nativeint) (vertex_count: int) (vertex_size: int) (buffer: nativeptr<byte>) (buffer_size: int) : int =
    assert (vertex_size > 0 && vertex_size <= 256)
    assert (vertex_size % 4 = 0)

    let vertex_data : nativeptr<byte> = NPtr.ofNI destination

    let mutable dataOff = 0
    let data_end = buffer_size

    if data_end - dataOff < 1 then
        -2
    else

    let data_header = NPtr.get buffer dataOff
    dataOff <- dataOff + 1

    if (data_header &&& 0xf0uy) <> kVertexHeader then
        -1
    else

    let version = int (data_header &&& 0x0fuy)
    if version > kDecodeVertexVersion then
        -1
    else

    let tail_size = vertex_size + (if version = 0 then 0 else vertex_size / 4)
    let tail_size_min = if version = 0 then kTailMinSizeV0 else kTailMinSizeV1
    let tail_size_pad = if tail_size < tail_size_min then tail_size_min else tail_size

    if data_end - dataOff < tail_size_pad then
        -2
    else

    let tail = data_end - tail_size

    let last_vertex = Array.zeroCreate<byte> 256
    use lvPin = fixed last_vertex
    NPtr.memcpy lvPin (NPtr.add buffer tail) vertex_size

    let channels = NPtr.add buffer (tail + vertex_size) // may be null-like if version=0 but won't be read

    let vertex_block_size = getVertexBlockSize vertex_size

    let mutable vertex_offset = 0
    let mutable error = 0

    while vertex_offset < vertex_count && error = 0 do
        let block_size = if vertex_offset + vertex_block_size < vertex_count then vertex_block_size else vertex_count - vertex_offset

        let blockOk =
            if Ssse3.IsSupported then
                decodeVertexBlockSimdSse buffer &dataOff data_end (NPtr.add vertex_data (vertex_offset * vertex_size)) block_size vertex_size lvPin channels version
            elif AdvSimd.Arm64.IsSupported then
                decodeVertexBlockSimdNeon buffer &dataOff data_end (NPtr.add vertex_data (vertex_offset * vertex_size)) block_size vertex_size lvPin channels version
            else
                decodeVertexBlockScalar buffer &dataOff data_end (NPtr.add vertex_data (vertex_offset * vertex_size)) block_size vertex_size lvPin channels version
        if not blockOk then
            error <- -2
        else
            vertex_offset <- vertex_offset + block_size

    if error <> 0 then
        error
    elif data_end - dataOff <> tail_size_pad then
        -3
    else
        0
