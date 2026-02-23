// This file is part of MeshOptimizerDotNet; see meshoptimizer.h for version/license details
// This work is based on:
// Fabian Giesen. Simple lossless index buffer compression & follow-up. 2013
// Conor Stokes. Vertex Cache Optimised Index Buffer Compression. 2014
module MeshOptimizerDotNet.IndexCodec

let private kIndexHeader = 0xe0uy
let private kSequenceHeader = 0xd0uy

let mutable private gEncodeIndexVersion = 1
let private kDecodeIndexVersion = 1

let private kCodeAuxEncodingTable =
    [| 0x00uy; 0x76uy; 0x87uy; 0x56uy; 0x67uy; 0x78uy; 0xa9uy; 0x86uy
       0x65uy; 0x89uy; 0x68uy; 0x98uy; 0x01uy; 0x69uy; 0x00uy; 0x00uy |]

let private rotateTriangle (_a: uint32) (b: uint32) (c: uint32) (next: uint32) : int =
    if b = next then 1
    elif c = next then 2
    else 0

// EdgeFifo is uint32[16][2] stored as flat uint32[32] with indexing [i*2+0] and [i*2+1]
let private getEdgeFifo (fifo: uint32[]) (a: uint32) (b: uint32) (c: uint32) (offset: int) : int =
    let mutable result = -1
    let mutable i = 0
    while i < 16 && result = -1 do
        let index = (offset - 1 - i) &&& 15
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

let private pushEdgeFifo (fifo: uint32[]) (a: uint32) (b: uint32) (offset: byref<int>) =
    fifo.[offset * 2 + 0] <- a
    fifo.[offset * 2 + 1] <- b
    offset <- (offset + 1) &&& 15

// VertexFifo is uint32[16]
let private getVertexFifo (fifo: uint32[]) (v: uint32) (offset: int) : int =
    let mutable result = -1
    let mutable i = 0
    while i < 16 && result = -1 do
        let index = (offset - 1 - i) &&& 15
        if fifo.[index] = v then
            result <- i
        i <- i + 1

    result

let private pushVertexFifo (fifo: uint32[]) (v: uint32) (offset: byref<int>) (cond: int) =
    fifo.[offset] <- v
    offset <- (offset + cond) &&& 15

let private encodeVByte (buffer: nativeptr<byte>) (offset: byref<int>) (v: uint32) =
    let mutable v = v
    let mutable cont = true
    while cont do
        NPtr.set buffer offset (byte ((v &&& 127u) ||| (if v > 127u then 128u else 0u)))
        offset <- offset + 1
        v <- v >>> 7
        cont <- v <> 0u

let private decodeVByte (buffer: nativeptr<byte>) (offset: byref<int>) : uint32 =
    let lead = NPtr.get buffer offset
    offset <- offset + 1

    if lead < 128uy then
        uint32 lead
    else
        let mutable result = uint32 (lead &&& 127uy)
        let mutable shift = 7
        let mutable i = 0
        let mutable cont = true
        while i < 4 && cont do
            let group = NPtr.get buffer offset
            offset <- offset + 1
            result <- result ||| (uint32 (group &&& 127uy) <<< shift)
            shift <- shift + 7
            if group < 128uy then cont <- false
            i <- i + 1
        result

let private encodeIndex (buffer: nativeptr<byte>) (offset: byref<int>) (index: uint32) (last: uint32) =
    let d = index - last
    // int(d) >> 31 is arithmetic right shift (sign-extending); F# >> on int is arithmetic
    let v = (d <<< 1) ^^^ (uint32 (int d >>> 31))
    encodeVByte buffer &offset v

let private decodeIndex (buffer: nativeptr<byte>) (offset: byref<int>) (last: uint32) : uint32 =
    let v = decodeVByte buffer &offset
    // -int(v & 1) produces 0 or 0xFFFFFFFF
    let d = (v >>> 1) ^^^ (uint32 (-(int (v &&& 1u))))
    last + d

let private getCodeAuxIndex (v: byte) (table: byte[]) : int =
    let mutable result = -1
    let mutable i = 0
    while i < 16 && result = -1 do
        if table.[i] = v then
            result <- i
        i <- i + 1
    result

let private writeTriangle (destination: nativeint) (offset: int) (index_size: int) (a: uint32) (b: uint32) (c: uint32) =
    if index_size = 2 then
        let dst = NPtr.ofNI<uint16> destination
        NPtr.set dst (offset + 0) (uint16 a)
        NPtr.set dst (offset + 1) (uint16 b)
        NPtr.set dst (offset + 2) (uint16 c)
    else
        let dst = NPtr.ofNI<uint32> destination
        NPtr.set dst (offset + 0) a
        NPtr.set dst (offset + 1) b
        NPtr.set dst (offset + 2) c

let meshopt_encodeIndexBuffer (buffer: nativeptr<byte>) (buffer_size: int) (indices: nativeptr<uint32>) (index_count: int) : int =
    assert (index_count % 3 = 0)

    // the minimum valid encoding is header, 1 byte per triangle and a 16-byte codeaux table
    if buffer_size < 1 + index_count / 3 + 16 then
        0
    else

    let version = gEncodeIndexVersion

    NPtr.set buffer 0 (kIndexHeader ||| byte version)

    let edgefifo = Array.create 32 0xFFFFFFFFu
    let vertexfifo = Array.create 16 0xFFFFFFFFu

    let mutable edgefifooffset = 0
    let mutable vertexfifooffset = 0

    let mutable next = 0u
    let mutable last = 0u

    let mutable code_offset = 1
    let data_start = 1 + index_count / 3
    let mutable data_offset = data_start
    let data_safe_end = buffer_size - 16

    let fecmax = if version >= 1 then 13 else 15

    let rotations = [| 0; 1; 2; 0; 1 |]

    let codeaux_table = kCodeAuxEncodingTable

    let mutable failed = false
    let mutable i = 0
    while i < index_count && not failed do
        // make sure we have enough space to write a triangle
        // each triangle writes at most 16 bytes: 1b for codeaux and 5b for each free index
        if data_offset > data_safe_end then
            failed <- true
        else

        let fer = getEdgeFifo edgefifo (NPtr.get indices (i + 0)) (NPtr.get indices (i + 1)) (NPtr.get indices (i + 2)) edgefifooffset

        if fer >= 0 && (fer >>> 2) < 15 then
            // note: getEdgeFifo implicitly rotates triangles by matching a/b to existing edge
            let orderBase = fer &&& 3

            let a = NPtr.get indices (i + rotations.[orderBase + 0])
            let b = NPtr.get indices (i + rotations.[orderBase + 1])
            let c = NPtr.get indices (i + rotations.[orderBase + 2])

            // encode edge index and vertex fifo index, next or free index
            let fe = fer >>> 2
            let fc = getVertexFifo vertexfifo c vertexfifooffset

            let mutable fec =
                if fc >= 1 && fc < fecmax then fc
                elif c = next then
                    next <- next + 1u
                    0
                else 15

            if fec = 15 && version >= 1 then
                // encode last-1 and last+1 to optimize strip-like sequences
                if c + 1u = last then
                    fec <- 13
                    last <- c
                if c = last + 1u then
                    fec <- 14
                    last <- c

            NPtr.set buffer code_offset (byte ((fe <<< 4) ||| fec))
            code_offset <- code_offset + 1

            // note that we need to update the last index since free indices are delta-encoded
            if fec = 15 then
                encodeIndex buffer &data_offset c last
                last <- c

            // we only need to push third vertex since first two are likely already in the vertex fifo
            if fec = 0 || fec >= fecmax then
                pushVertexFifo vertexfifo c &vertexfifooffset 1

            // we only need to push two new edges to edge fifo since the third one is already there
            pushEdgeFifo edgefifo c b &edgefifooffset
            pushEdgeFifo edgefifo a c &edgefifooffset
        else
            let rotation = rotateTriangle (NPtr.get indices (i + 0)) (NPtr.get indices (i + 1)) (NPtr.get indices (i + 2)) next
            let orderBase = rotation

            let a = NPtr.get indices (i + rotations.[orderBase + 0])
            let b = NPtr.get indices (i + rotations.[orderBase + 1])
            let c = NPtr.get indices (i + rotations.[orderBase + 2])

            // if a/b/c are 0/1/2, we emit a reset code
            let mutable reset = false

            if a = 0u && b = 1u && c = 2u && next > 0u && version >= 1 then
                reset <- true
                next <- 0u
                // reset vertex fifo to make sure we don't accidentally reference vertices from that in the future
                for vi = 0 to 15 do
                    vertexfifo.[vi] <- 0xFFFFFFFFu

            let fb = getVertexFifo vertexfifo b vertexfifooffset
            let fc = getVertexFifo vertexfifo c vertexfifooffset

            // after rotation, a is almost always equal to next, so we don't waste bits on FIFO encoding for a
            // note: decoder implicitly assumes that if feb=fec=0, then fea=0 (reset code); this is enforced by rotation
            let fea =
                if a = next then
                    next <- next + 1u
                    0
                else 15

            let feb =
                if fb >= 0 && fb < 14 then fb + 1
                elif b = next then
                    next <- next + 1u
                    0
                else 15

            let fec =
                if fc >= 0 && fc < 14 then fc + 1
                elif c = next then
                    next <- next + 1u
                    0
                else 15

            // we encode feb & fec in 4 bits using a table if possible, and as a full byte otherwise
            let codeaux = byte ((feb <<< 4) ||| fec)
            let codeauxindex = getCodeAuxIndex codeaux codeaux_table

            // <14 encodes an index into codeaux table, 14 encodes fea=0, 15 encodes fea=15
            if fea = 0 && codeauxindex >= 0 && codeauxindex < 14 && not reset then
                NPtr.set buffer code_offset (byte ((15 <<< 4) ||| codeauxindex))
                code_offset <- code_offset + 1
            else
                NPtr.set buffer code_offset (byte ((15 <<< 4) ||| 14 ||| fea))
                code_offset <- code_offset + 1
                NPtr.set buffer data_offset codeaux
                data_offset <- data_offset + 1

            // note that we need to update the last index since free indices are delta-encoded
            if fea = 15 then
                encodeIndex buffer &data_offset a last
                last <- a

            if feb = 15 then
                encodeIndex buffer &data_offset b last
                last <- b

            if fec = 15 then
                encodeIndex buffer &data_offset c last
                last <- c

            // only push vertices that weren't already in fifo
            if fea = 0 || fea = 15 then
                pushVertexFifo vertexfifo a &vertexfifooffset 1

            if feb = 0 || feb = 15 then
                pushVertexFifo vertexfifo b &vertexfifooffset 1

            if fec = 0 || fec = 15 then
                pushVertexFifo vertexfifo c &vertexfifooffset 1

            // all three edges aren't in the fifo; pushing all of them is important so that we can match them for later triangles
            pushEdgeFifo edgefifo b a &edgefifooffset
            pushEdgeFifo edgefifo c b &edgefifooffset
            pushEdgeFifo edgefifo a c &edgefifooffset

        i <- i + 3

    if failed then
        0
    else

    // make sure we have enough space to write codeaux table
    if data_offset > data_safe_end then
        0
    else

    // add codeaux encoding table to the end of the stream; this is used for decoding codeaux *and* as padding
    for j = 0 to 15 do
        assert ((codeaux_table.[j] &&& 0xfuy) <> 0xfuy && (codeaux_table.[j] >>> 4) <> 0xfuy)
        NPtr.set buffer (data_offset + j) codeaux_table.[j]
    data_offset <- data_offset + 16

    // since we encode restarts as codeaux without a table reference, we need to make sure 00 is encoded as a table reference
    assert (codeaux_table.[0] = 0uy)

    assert (data_offset >= index_count / 3 + 16)
    assert (data_offset <= buffer_size)

    data_offset

let meshopt_encodeIndexBufferBound (index_count: int) (vertex_count: int) : int =
    assert (index_count % 3 = 0)

    // compute number of bits required for each index
    let mutable vertex_bits = 1u

    while vertex_bits < 32u && uint32 vertex_count > (1u <<< int vertex_bits) do
        vertex_bits <- vertex_bits + 1u

    // worst-case encoding is 2 header bytes + 3 varint-7 encoded index deltas
    let vertex_groups = (vertex_bits + 1u + 6u) / 7u

    1 + (index_count / 3) * (2 + 3 * int vertex_groups) + 16

let meshopt_encodeIndexVersion (version: int) =
    assert (uint32 version <= uint32 kDecodeIndexVersion)
    gEncodeIndexVersion <- version

let meshopt_decodeIndexVersion (buffer: nativeptr<byte>) (buffer_size: int) : int =
    if buffer_size < 1 then
        -1
    else

    let header = NPtr.get buffer 0

    if (header &&& 0xf0uy) <> kIndexHeader && (header &&& 0xf0uy) <> kSequenceHeader then
        -1
    else

    let version = int (header &&& 0x0fuy)
    if version > kDecodeIndexVersion then
        -1
    else

    version

let meshopt_decodeIndexBuffer (destination: nativeint) (index_count: int) (index_size: int) (buffer: nativeptr<byte>) (buffer_size: int) : int =
    assert (index_count % 3 = 0)
    assert (index_size = 2 || index_size = 4)

    // the minimum valid encoding is header, 1 byte per triangle and a 16-byte codeaux table
    if buffer_size < 1 + index_count / 3 + 16 then
        -2
    else

    if (NPtr.get buffer 0 &&& 0xf0uy) <> kIndexHeader then
        -1
    else

    let version = int (NPtr.get buffer 0 &&& 0x0fuy)
    if version > kDecodeIndexVersion then
        -1
    else

    let edgefifo = Array.create 32 0xFFFFFFFFu
    let vertexfifo = Array.create 16 0xFFFFFFFFu

    let mutable edgefifooffset = 0
    let mutable vertexfifooffset = 0

    let mutable next = 0u
    let mutable last = 0u

    let fecmax = if version >= 1 then 13 else 15

    // since we store 16-byte codeaux table at the end, triangle data has to begin before data_safe_end
    let mutable code_offset = 1
    let mutable data_offset = 1 + index_count / 3
    let data_safe_end = buffer_size - 16

    let codeaux_table_offset = data_safe_end

    let mutable error = 0
    let mutable i = 0
    while i < index_count && error = 0 do
        // make sure we have enough data to read for a triangle
        if data_offset > data_safe_end then
            error <- -2
        else

        let codetri = NPtr.get buffer code_offset
        code_offset <- code_offset + 1

        if codetri < 0xf0uy then
            let fe = int codetri >>> 4

            // fifo reads are wrapped around 16 entry buffer
            let fifoIdx = (edgefifooffset - 1 - fe) &&& 15
            let a = edgefifo.[fifoIdx * 2 + 0]
            let b = edgefifo.[fifoIdx * 2 + 1]
            let mutable c = 0u

            let fec = int codetri &&& 15

            // note: this is the most common path in the entire decoder
            if fec < fecmax then
                // fifo reads are wrapped around 16 entry buffer
                let cf = vertexfifo.[(vertexfifooffset - 1 - fec) &&& 15]
                c <- if fec = 0 then next else cf

                let fec0 = if fec = 0 then 1 else 0
                next <- next + uint32 fec0

                // push vertex fifo must match the encoding step *exactly* otherwise the data will not be decoded correctly
                pushVertexFifo vertexfifo c &vertexfifooffset fec0
            else
                // fec - (fec ^ 3) decodes 13, 14 into -1, 1
                // note that we need to update the last index since free indices are delta-encoded
                if fec <> 15 then
                    c <- last + uint32 (fec - (fec ^^^ 3))
                    last <- c
                else
                    c <- decodeIndex buffer &data_offset last
                    last <- c

                // push vertex/edge fifo must match the encoding step *exactly* otherwise the data will not be decoded correctly
                pushVertexFifo vertexfifo c &vertexfifooffset 1

            // push edge fifo must match the encoding step *exactly* otherwise the data will not be decoded correctly
            pushEdgeFifo edgefifo c b &edgefifooffset
            pushEdgeFifo edgefifo a c &edgefifooffset

            // output triangle
            writeTriangle destination i index_size a b c
        else
            // fast path: read codeaux from the table
            if codetri < 0xfeuy then
                let codeaux = NPtr.get buffer (codeaux_table_offset + (int codetri &&& 15))

                // note: table can't contain feb/fec=15
                let feb = int codeaux >>> 4
                let fec = int codeaux &&& 15

                // fifo reads are wrapped around 16 entry buffer
                // also note that we increment next for all three vertices before decoding indices - this matches encoder behavior
                let a = next
                next <- next + 1u

                let bf = vertexfifo.[(vertexfifooffset - feb) &&& 15]
                let b = if feb = 0 then next else bf

                let feb0 = if feb = 0 then 1 else 0
                next <- next + uint32 feb0

                let cf = vertexfifo.[(vertexfifooffset - fec) &&& 15]
                let c = if fec = 0 then next else cf

                let fec0 = if fec = 0 then 1 else 0
                next <- next + uint32 fec0

                // output triangle
                writeTriangle destination i index_size a b c

                // push vertex/edge fifo must match the encoding step *exactly* otherwise the data will not be decoded correctly
                pushVertexFifo vertexfifo a &vertexfifooffset 1
                pushVertexFifo vertexfifo b &vertexfifooffset feb0
                pushVertexFifo vertexfifo c &vertexfifooffset fec0

                pushEdgeFifo edgefifo b a &edgefifooffset
                pushEdgeFifo edgefifo c b &edgefifooffset
                pushEdgeFifo edgefifo a c &edgefifooffset
            else
                // slow path: read a full byte for codeaux instead of using a table lookup
                let codeaux = NPtr.get buffer data_offset
                data_offset <- data_offset + 1

                let fea = if codetri = 0xfeuy then 0 else 15
                let feb = int codeaux >>> 4
                let fec = int codeaux &&& 15

                // reset: codeaux is 0 but encoded as not-a-table
                if codeaux = 0uy then
                    next <- 0u

                // fifo reads are wrapped around 16 entry buffer
                // also note that we increment next for all three vertices before decoding indices - this matches encoder behavior
                let mutable a =
                    if fea = 0 then
                        let v = next
                        next <- next + 1u
                        v
                    else 0u

                let mutable b =
                    if feb = 0 then
                        let v = next
                        next <- next + 1u
                        v
                    else vertexfifo.[(vertexfifooffset - feb) &&& 15]

                let mutable c =
                    if fec = 0 then
                        let v = next
                        next <- next + 1u
                        v
                    else vertexfifo.[(vertexfifooffset - fec) &&& 15]

                // note that we need to update the last index since free indices are delta-encoded
                if fea = 15 then
                    a <- decodeIndex buffer &data_offset last
                    last <- a

                if feb = 15 then
                    b <- decodeIndex buffer &data_offset last
                    last <- b

                if fec = 15 then
                    c <- decodeIndex buffer &data_offset last
                    last <- c

                // output triangle
                writeTriangle destination i index_size a b c

                // push vertex/edge fifo must match the encoding step *exactly* otherwise the data will not be decoded correctly
                pushVertexFifo vertexfifo a &vertexfifooffset 1
                pushVertexFifo vertexfifo b &vertexfifooffset (if feb = 0 || feb = 15 then 1 else 0)
                pushVertexFifo vertexfifo c &vertexfifooffset (if fec = 0 || fec = 15 then 1 else 0)

                pushEdgeFifo edgefifo b a &edgefifooffset
                pushEdgeFifo edgefifo c b &edgefifooffset
                pushEdgeFifo edgefifo a c &edgefifooffset

        i <- i + 3

    if error <> 0 then
        error
    else

    // we should've read all data bytes and stopped at the boundary between data and codeaux table
    if data_offset <> data_safe_end then
        -3
    else

    0

let meshopt_encodeIndexSequence (buffer: nativeptr<byte>) (buffer_size: int) (indices: nativeptr<uint32>) (index_count: int) : int =
    // the minimum valid encoding is header, 1 byte per index and a 4-byte tail
    if buffer_size < 1 + index_count + 4 then
        0
    else

    let version = gEncodeIndexVersion

    NPtr.set buffer 0 (kSequenceHeader ||| byte version)

    let mutable last = [| 0u; 0u |]
    let mutable current = 0u

    let mutable data_offset = 1
    let data_safe_end = buffer_size - 4

    let mutable failed = false
    let mutable i = 0
    while i < index_count && not failed do
        // make sure we have enough data to write
        // each index writes at most 5 bytes of data; there's a 4 byte tail after data_safe_end
        if data_offset >= data_safe_end then
            failed <- true
        else

        let index = NPtr.get indices i

        // this is a heuristic that switches between baselines when the delta grows too large
        let cd = int (index - last.[int current])
        let absCd = if cd < 0 then -cd else cd
        current <- current ^^^ (if absCd >= 30 then 1u else 0u)

        // encode delta from the last index
        let d = index - last.[int current]
        // int(d) >> 31 is arithmetic right shift
        let v = (d <<< 1) ^^^ (uint32 (int d >>> 31))

        // note: low bit encodes the index of the last baseline which will be used for reconstruction
        encodeVByte buffer &data_offset ((v <<< 1) ||| current)

        // update last for the next iteration that uses it
        last.[int current] <- index

        i <- i + 1

    if failed then
        0
    else

    // make sure we have enough space to write tail
    if data_offset > data_safe_end then
        0
    else

    for k = 0 to 3 do
        NPtr.set buffer (data_offset + k) 0uy
    data_offset <- data_offset + 4

    data_offset

let meshopt_encodeIndexSequenceBound (index_count: int) (vertex_count: int) : int =
    // compute number of bits required for each index
    let mutable vertex_bits = 1u

    while vertex_bits < 32u && uint32 vertex_count > (1u <<< int vertex_bits) do
        vertex_bits <- vertex_bits + 1u

    // worst-case encoding is 1 varint-7 encoded index delta for a K bit value and an extra bit
    let vertex_groups = (vertex_bits + 1u + 1u + 6u) / 7u

    1 + index_count * int vertex_groups + 4

let meshopt_decodeIndexSequence (destination: nativeint) (index_count: int) (index_size: int) (buffer: nativeptr<byte>) (buffer_size: int) : int =
    // the minimum valid encoding is header, 1 byte per index and a 4-byte tail
    if buffer_size < 1 + index_count + 4 then
        -2
    else

    if (NPtr.get buffer 0 &&& 0xf0uy) <> kSequenceHeader then
        -1
    else

    let version = int (NPtr.get buffer 0 &&& 0x0fuy)
    if version > kDecodeIndexVersion then
        -1
    else

    let mutable data_offset = 1
    let data_safe_end = buffer_size - 4

    let mutable last = [| 0u; 0u |]

    let mutable error = 0
    let mutable i = 0
    while i < index_count && error = 0 do
        // make sure we have enough data to read
        if data_offset >= data_safe_end then
            error <- -2
        else

        let v = decodeVByte buffer &data_offset

        // decode the index of the last baseline
        let current = v &&& 1u
        let v = v >>> 1

        // reconstruct index as a delta
        // -int(v & 1) produces 0 or 0xFFFFFFFF
        let d = (v >>> 1) ^^^ (uint32 (-(int (v &&& 1u))))
        let index = last.[int current] + d

        // update last for the next iteration that uses it
        last.[int current] <- index

        if index_size = 2 then
            let dst = NPtr.ofNI<uint16> destination
            NPtr.set dst i (uint16 index)
        else
            let dst = NPtr.ofNI<uint32> destination
            NPtr.set dst i index

        i <- i + 1

    if error <> 0 then
        error
    else

    // we should've read all data bytes and stopped at the boundary between data and tail
    if data_offset <> data_safe_end then
        -3
    else

    0
