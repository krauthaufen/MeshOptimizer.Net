// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
module MeshOptPort.Stripifier

open System
open MeshOptPort.Allocator

// buffer is an array of triangles stored flat: buffer[i*3+j]
let private findStripFirst (buffer: uint32[]) (buffer_size: int) (valence: nativeptr<byte>) : int =
    let mutable index = 0
    let mutable iv = ~~~0u

    for i = 0 to buffer_size - 1 do
        let va = uint32 (NPtr.get valence (int buffer.[i * 3 + 0]))
        let vb = uint32 (NPtr.get valence (int buffer.[i * 3 + 1]))
        let vc = uint32 (NPtr.get valence (int buffer.[i * 3 + 2]))
        let v = if va < vb && va < vc then va elif vb < vc then vb else vc

        if v < iv then
            index <- i
            iv <- v

    index

let private findStripNext (buffer: uint32[]) (buffer_size: int) (e0: uint32) (e1: uint32) : int =
    let mutable result = -1
    let mutable i = 0
    while i < buffer_size && result = -1 do
        let a = buffer.[i * 3 + 0]
        let b = buffer.[i * 3 + 1]
        let c = buffer.[i * 3 + 2]

        if e0 = a && e1 = b then
            result <- (i <<< 2) ||| 2
        elif e0 = b && e1 = c then
            result <- (i <<< 2) ||| 0
        elif e0 = c && e1 = a then
            result <- (i <<< 2) ||| 1

        i <- i + 1

    result

let meshopt_stripify (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (vertex_count: int) (restart_index: uint32) : int =
    assert (NPtr.toNI destination <> NPtr.toNI indices)
    assert (index_count % 3 = 0)

    use allocator = new meshopt_Allocator()

    let buffer_capacity = 8
    let buffer = Array.zeroCreate<uint32>(buffer_capacity * 3)
    let mutable buffer_size = 0

    let mutable index_offset = 0

    let strip = Array.zeroCreate<uint32> 2
    let mutable parity = 0u

    let mutable strip_size = 0

    // compute vertex valence
    let valence = allocator.allocate<byte>(vertex_count)
    NPtr.memset valence 0uy vertex_count

    for i = 0 to index_count - 1 do
        let index = int (NPtr.get indices i)
        assert (index < vertex_count)
        let v = NPtr.get valence index
        NPtr.set valence index (v + 1uy)

    let mutable next = -1

    while buffer_size > 0 || index_offset < index_count do
        assert (next < 0 || (next >>> 2 < buffer_size && (next &&& 3) < 3))

        // fill triangle buffer
        while buffer_size < buffer_capacity && index_offset < index_count do
            buffer.[buffer_size * 3 + 0] <- NPtr.get indices (index_offset + 0)
            buffer.[buffer_size * 3 + 1] <- NPtr.get indices (index_offset + 1)
            buffer.[buffer_size * 3 + 2] <- NPtr.get indices (index_offset + 2)
            buffer_size <- buffer_size + 1
            index_offset <- index_offset + 3

        assert (buffer_size > 0)

        if next >= 0 then
            let i = next >>> 2
            let a = buffer.[i * 3 + 0]
            let b = buffer.[i * 3 + 1]
            let c = buffer.[i * 3 + 2]
            let v = buffer.[i * 3 + (next &&& 3)]

            // ordered removal from the buffer
            for k = i * 3 to (buffer_size - 2) * 3 + 2 do
                buffer.[k] <- buffer.[k + 3]
            buffer_size <- buffer_size - 1

            // update vertex valences
            NPtr.set valence (int a) (NPtr.get valence (int a) - 1uy)
            NPtr.set valence (int b) (NPtr.get valence (int b) - 1uy)
            NPtr.set valence (int c) (NPtr.get valence (int c) - 1uy)

            // find next triangle
            let cont = findStripNext buffer buffer_size (if parity <> 0u then strip.[1] else v) (if parity <> 0u then v else strip.[1])
            let swap = if cont < 0 then findStripNext buffer buffer_size (if parity <> 0u then v else strip.[0]) (if parity <> 0u then strip.[0] else v) else -1

            if cont < 0 && swap >= 0 then
                NPtr.set destination strip_size strip.[0]
                strip_size <- strip_size + 1
                NPtr.set destination strip_size v
                strip_size <- strip_size + 1

                strip.[1] <- v
                next <- swap
            else
                NPtr.set destination strip_size v
                strip_size <- strip_size + 1

                strip.[0] <- strip.[1]
                strip.[1] <- v
                parity <- parity ^^^ 1u

                next <- cont
        else
            let i = findStripFirst buffer buffer_size valence
            let mutable a = buffer.[i * 3 + 0]
            let mutable b = buffer.[i * 3 + 1]
            let mutable c = buffer.[i * 3 + 2]

            // ordered removal from the buffer
            for k = i * 3 to (buffer_size - 2) * 3 + 2 do
                buffer.[k] <- buffer.[k + 3]
            buffer_size <- buffer_size - 1

            // update vertex valences
            NPtr.set valence (int a) (NPtr.get valence (int a) - 1uy)
            NPtr.set valence (int b) (NPtr.get valence (int b) - 1uy)
            NPtr.set valence (int c) (NPtr.get valence (int c) - 1uy)

            // pre-rotate the triangle
            let ea = findStripNext buffer buffer_size c b
            let eb = findStripNext buffer buffer_size a c
            let ec = findStripNext buffer buffer_size b a

            let mutable mine = System.Int32.MaxValue
            mine <- if ea >= 0 && mine > ea then ea else mine
            mine <- if eb >= 0 && mine > eb then eb else mine
            mine <- if ec >= 0 && mine > ec then ec else mine

            if ea = mine then
                next <- ea
            elif eb = mine then
                let t = a in a <- b; b <- c; c <- t
                next <- eb
            elif ec = mine then
                let t = c in c <- b; b <- a; a <- t
                next <- ec

            if restart_index <> 0u then
                if strip_size > 0 then
                    NPtr.set destination strip_size restart_index
                    strip_size <- strip_size + 1

                NPtr.set destination strip_size a
                strip_size <- strip_size + 1
                NPtr.set destination strip_size b
                strip_size <- strip_size + 1
                NPtr.set destination strip_size c
                strip_size <- strip_size + 1

                strip.[0] <- b
                strip.[1] <- c
                parity <- 1u
            else
                if strip_size > 0 then
                    NPtr.set destination strip_size strip.[1]
                    strip_size <- strip_size + 1
                    NPtr.set destination strip_size a
                    strip_size <- strip_size + 1

                let e0 = if parity <> 0u then c else b
                let e1 = if parity <> 0u then b else c

                NPtr.set destination strip_size a
                strip_size <- strip_size + 1
                NPtr.set destination strip_size e0
                strip_size <- strip_size + 1
                NPtr.set destination strip_size e1
                strip_size <- strip_size + 1

                strip.[0] <- e0
                strip.[1] <- e1
                parity <- parity ^^^ 1u

    strip_size

let meshopt_stripifyBound (index_count: int) : int =
    assert (index_count % 3 = 0)
    (index_count / 3) * 5

let meshopt_unstripify (destination: nativeptr<uint32>) (indices: nativeptr<uint32>) (index_count: int) (restart_index: uint32) : int =
    assert (NPtr.toNI destination <> NPtr.toNI indices)

    let mutable offset = 0
    let mutable start = 0

    for i = 0 to index_count - 1 do
        if restart_index <> 0u && NPtr.get indices i = restart_index then
            start <- i + 1
        elif i - start >= 2 then
            let mutable a = NPtr.get indices (i - 2)
            let mutable b = NPtr.get indices (i - 1)
            let c = NPtr.get indices i

            if (i - start) &&& 1 <> 0 then
                let t = a in a <- b; b <- t

            if a <> b && a <> c && b <> c then
                NPtr.set destination (offset + 0) a
                NPtr.set destination (offset + 1) b
                NPtr.set destination (offset + 2) c
                offset <- offset + 3

    offset

let meshopt_unstripifyBound (index_count: int) : int =
    assert (index_count = 0 || index_count >= 3)
    if index_count = 0 then 0 else (index_count - 2) * 3
