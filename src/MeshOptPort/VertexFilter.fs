// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
module MeshOptPort.VertexFilter

#nowarn "9"

open System
open System.Runtime.CompilerServices
open System.Runtime.Intrinsics
open System.Runtime.Intrinsics.X86
open System.Runtime.Intrinsics.Arm
open MeshOptPort
open MeshOptPort.Quantization

// ---- scalar fallbacks ----

let private decodeFilterOctScalar8 (data: nativeptr<sbyte>) (count: int) =
    let max = float32 ((1 <<< (1 * 8 - 1)) - 1) // 127.f

    for i = 0 to count - 1 do
        // convert x and y to floats and reconstruct z; this assumes zf encodes 1.f at the same bit count
        let x0 = float32 (NPtr.get data (i * 4 + 0))
        let y0 = float32 (NPtr.get data (i * 4 + 1))
        let z0 = float32 (NPtr.get data (i * 4 + 2)) - abs x0 - abs y0

        // fixup octahedral coordinates for z<0
        let t = if z0 >= 0.0f then 0.0f else z0

        let x1 = x0 + (if x0 >= 0.0f then t else -t)
        let y1 = y0 + (if y0 >= 0.0f then t else -t)

        // compute normal length & scale
        let l = sqrt (x1 * x1 + y1 * y1 + z0 * z0)
        let s = max / l

        // rounded signed float->int
        let xf = int (x1 * s + (if x1 >= 0.0f then 0.5f else -0.5f))
        let yf = int (y1 * s + (if y1 >= 0.0f then 0.5f else -0.5f))
        let zf = int (z0 * s + (if z0 >= 0.0f then 0.5f else -0.5f))

        NPtr.set data (i * 4 + 0) (sbyte xf)
        NPtr.set data (i * 4 + 1) (sbyte yf)
        NPtr.set data (i * 4 + 2) (sbyte zf)

let private decodeFilterOctScalar16 (data: nativeptr<int16>) (count: int) =
    let max = float32 ((1 <<< (2 * 8 - 1)) - 1) // 32767.f

    for i = 0 to count - 1 do
        let x0 = float32 (NPtr.get data (i * 4 + 0))
        let y0 = float32 (NPtr.get data (i * 4 + 1))
        let z0 = float32 (NPtr.get data (i * 4 + 2)) - abs x0 - abs y0

        let t = if z0 >= 0.0f then 0.0f else z0

        let x1 = x0 + (if x0 >= 0.0f then t else -t)
        let y1 = y0 + (if y0 >= 0.0f then t else -t)

        let l = sqrt (x1 * x1 + y1 * y1 + z0 * z0)
        let s = max / l

        let xf = int (x1 * s + (if x1 >= 0.0f then 0.5f else -0.5f))
        let yf = int (y1 * s + (if y1 >= 0.0f then 0.5f else -0.5f))
        let zf = int (z0 * s + (if z0 >= 0.0f then 0.5f else -0.5f))

        NPtr.set data (i * 4 + 0) (int16 xf)
        NPtr.set data (i * 4 + 1) (int16 yf)
        NPtr.set data (i * 4 + 2) (int16 zf)

let private decodeFilterQuatScalar (data: nativeptr<int16>) (count: int) =
    let scale = 32767.0f / sqrt 2.0f

    for i = 0 to count - 1 do
        // recover scale from the high byte of the component
        let sf = int (NPtr.get data (i * 4 + 3)) ||| 3
        let s = float32 sf

        // convert x/y/z to floating point
        let x = float32 (NPtr.get data (i * 4 + 0))
        let y = float32 (NPtr.get data (i * 4 + 1))
        let z = float32 (NPtr.get data (i * 4 + 2))

        // reconstruct w as a square root
        let ws = s * s
        let ww = ws * 2.0f - x * x - y * y - z * z
        let w = sqrt (if ww >= 0.0f then ww else 0.0f)

        // compute final scale
        let ss = scale / s

        // rounded signed float->int
        let xf = int (x * ss + (if x >= 0.0f then 0.5f else -0.5f))
        let yf = int (y * ss + (if y >= 0.0f then 0.5f else -0.5f))
        let zf = int (z * ss + (if z >= 0.0f then 0.5f else -0.5f))
        let wf = int (w * ss + 0.5f)

        let qc = int (NPtr.get data (i * 4 + 3)) &&& 3

        // output order is dictated by input index
        NPtr.set data (i * 4 + ((qc + 1) &&& 3)) (int16 xf)
        NPtr.set data (i * 4 + ((qc + 2) &&& 3)) (int16 yf)
        NPtr.set data (i * 4 + ((qc + 3) &&& 3)) (int16 zf)
        NPtr.set data (i * 4 + ((qc + 0) &&& 3)) (int16 wf)

let private decodeFilterExpScalar (data: nativeptr<uint32>) (count: int) =
    for i = 0 to count - 1 do
        let v = NPtr.get data i

        // decode mantissa and exponent
        let m = int (v <<< 8) >>> 8
        let e = int v >>> 24

        // optimized version of ldexp(float(m), e)
        let ui = uint32 (e + 127) <<< 23
        let f = Unsafe.BitCast<uint32, float32>(ui)
        let result = f * float32 m

        NPtr.set data i (Unsafe.BitCast<float32, uint32>(result))

let private decodeFilterColorScalar8 (data: nativeptr<byte>) (count: int) =
    let max = float32 ((1 <<< (1 * 8)) - 1) // 255.f

    for i = 0 to count - 1 do
        // recover scale from alpha high bit
        let mutable as_ = int (NPtr.get data (i * 4 + 3))
        as_ <- as_ ||| (as_ >>> 1)
        as_ <- as_ ||| (as_ >>> 2)
        as_ <- as_ ||| (as_ >>> 4)
        as_ <- as_ ||| (as_ >>> 8) // noop for 8-bit

        // convert to RGB in fixed point (co/cg are sign extended)
        let y = int (NPtr.get data (i * 4 + 0))
        let co = int (sbyte (NPtr.get data (i * 4 + 1)))
        let cg = int (sbyte (NPtr.get data (i * 4 + 2)))

        let r = y + co - cg
        let g = y + cg
        let b = y - co - cg

        // expand alpha by one bit to match other components
        let a0 = int (NPtr.get data (i * 4 + 3))
        let a = ((a0 <<< 1) &&& as_) ||| (a0 &&& 1)

        // compute scaling factor
        let ss = max / float32 as_

        // rounded float->int
        let rf = int (float32 r * ss + 0.5f)
        let gf = int (float32 g * ss + 0.5f)
        let bf = int (float32 b * ss + 0.5f)
        let af = int (float32 a * ss + 0.5f)

        NPtr.set data (i * 4 + 0) (byte rf)
        NPtr.set data (i * 4 + 1) (byte gf)
        NPtr.set data (i * 4 + 2) (byte bf)
        NPtr.set data (i * 4 + 3) (byte af)

let private decodeFilterColorScalar16 (data: nativeptr<uint16>) (count: int) =
    let max = float32 ((1 <<< (2 * 8)) - 1) // 65535.f

    for i = 0 to count - 1 do
        let mutable as_ = int (NPtr.get data (i * 4 + 3))
        as_ <- as_ ||| (as_ >>> 1)
        as_ <- as_ ||| (as_ >>> 2)
        as_ <- as_ ||| (as_ >>> 4)
        as_ <- as_ ||| (as_ >>> 8)

        let y = int (NPtr.get data (i * 4 + 0))
        let co = int (int16 (NPtr.get data (i * 4 + 1)))
        let cg = int (int16 (NPtr.get data (i * 4 + 2)))

        let r = y + co - cg
        let g = y + cg
        let b = y - co - cg

        let a0 = int (NPtr.get data (i * 4 + 3))
        let a = ((a0 <<< 1) &&& as_) ||| (a0 &&& 1)

        let ss = max / float32 as_

        let rf = int (float32 r * ss + 0.5f)
        let gf = int (float32 g * ss + 0.5f)
        let bf = int (float32 b * ss + 0.5f)
        let af = int (float32 a * ss + 0.5f)

        NPtr.set data (i * 4 + 0) (uint16 rf)
        NPtr.set data (i * 4 + 1) (uint16 gf)
        NPtr.set data (i * 4 + 2) (uint16 bf)
        NPtr.set data (i * 4 + 3) (uint16 af)

// ---- SIMD dispatch helper ----

let inline private dispatchSimd (processSimd: nativeptr<'T> -> int -> unit) (processScalar: nativeptr<'T> -> int -> unit) (data: nativeptr<'T>) (count: int) (stride: int) =
    assert (stride <= 4)

    let count4 = count &&& ~~~3
    processSimd data count4

    if count4 < count then
        // tail: max stride 4, max count 4 => 16 elements
        let tail: 'T[] = Array.zeroCreate 16
        let tail_size = (count - count4) * stride * sizeof<'T>
        assert (tail_size <= 16 * sizeof<'T>)

        use pinned = fixed tail
        NPtr.memcpy pinned (NPtr.add data (count4 * stride)) tail_size
        processSimd pinned (count - count4)
        NPtr.memcpy (NPtr.add data (count4 * stride)) pinned tail_size

let inline private rotateleft64 (v: uint64) (x: int) : uint64 =
    (v <<< (x &&& 63)) ||| (v >>> ((64 - x) &&& 63))

// ---- SSE SIMD implementations ----

let private decodeFilterOctSimd8_SSE (data: nativeptr<sbyte>) (count: int) =
    let sign = Vector128.Create(-0.0f) // sign bit mask

    let mutable i = 0
    while i < count do
        let n4 = Sse2.LoadVector128(NPtr.cast<sbyte, int> (NPtr.add data (i * 4)))

        // sign-extends each of x,y in [x y ? ?] with arithmetic shifts
        let xf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(n4, 24uy), 24uy)
        let yf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(n4, 16uy), 24uy)

        // unpack z; note that z is unsigned so we technically don't need to sign extend it
        let zf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(n4, 8uy), 24uy)

        // convert x and y to floats and reconstruct z; this assumes zf encodes 1.f at the same bit count
        let x = Sse2.ConvertToVector128Single(xf)
        let y = Sse2.ConvertToVector128Single(yf)
        let z = Sse.Subtract(Sse2.ConvertToVector128Single(zf), Sse.Add(Sse.AndNot(sign, x), Sse.AndNot(sign, y)))

        // fixup octahedral coordinates for z<0
        let t = Sse.Min(z, Vector128.Create(0.0f))

        let x = Sse.Add(x, Sse.Xor(t, Sse.And(x, sign)))
        let y = Sse.Add(y, Sse.Xor(t, Sse.And(y, sign)))

        // compute normal length & scale
        let ll = Sse.Add(Sse.Multiply(x, x), Sse.Add(Sse.Multiply(y, y), Sse.Multiply(z, z)))
        let s = Sse.Multiply(Vector128.Create(127.0f), Sse.Reciprocal(Sse.Sqrt(ll)))

        // rounded signed float->int
        let xr = Sse2.ConvertToVector128Int32(Sse.Multiply(x, s))
        let yr = Sse2.ConvertToVector128Int32(Sse.Multiply(y, s))
        let zr = Sse2.ConvertToVector128Int32(Sse.Multiply(z, s))

        // combine xr/yr/zr into final value
        let mutable res = Sse2.And(n4, Vector128.Create(int 0xff000000))
        res <- Sse2.Or(res, Sse2.And(xr, Vector128.Create(0xff)))
        res <- Sse2.Or(res, Sse2.ShiftLeftLogical(Sse2.And(yr, Vector128.Create(0xff)), 8uy))
        res <- Sse2.Or(res, Sse2.ShiftLeftLogical(Sse2.And(zr, Vector128.Create(0xff)), 16uy))

        Sse2.Store(NPtr.cast<sbyte, int> (NPtr.add data (i * 4)), res)
        i <- i + 4

let private decodeFilterOctSimd16_SSE (data: nativeptr<int16>) (count: int) =
    let sign = Vector128.Create(-0.0f)

    let mutable i = 0
    while i < count do
        let n4_0 = Sse.LoadVector128(NPtr.cast<int16, float32> (NPtr.add data ((i + 0) * 4)))
        let n4_1 = Sse.LoadVector128(NPtr.cast<int16, float32> (NPtr.add data ((i + 2) * 4)))

        // gather both x/y 16-bit pairs in each 32-bit lane
        // _MM_SHUFFLE(2,0,2,0) = (2<<<6) ||| (0<<<4) ||| (2<<<2) ||| 0 = 136uy = 0x88
        let n4 = Sse.Shuffle(n4_0, n4_1, 0x88uy).AsInt32()

        // sign-extends each of x,y in [x y] with arithmetic shifts
        let xf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(n4, 16uy), 16uy)
        let yf = Sse2.ShiftRightArithmetic(n4, 16uy)

        // unpack z; note that z is unsigned so we don't need to sign extend it
        // _MM_SHUFFLE(3,1,3,1) = (3<<<6) ||| (1<<<4) ||| (3<<<2) ||| 1 = 221uy = 0xDD
        let z4 = Sse.Shuffle(n4_0, n4_1, 0xDDuy).AsInt32()
        let zf = Sse2.And(z4, Vector128.Create(0x7fff))

        // convert x and y to floats and reconstruct z
        let x = Sse2.ConvertToVector128Single(xf)
        let y = Sse2.ConvertToVector128Single(yf)
        let z = Sse.Subtract(Sse2.ConvertToVector128Single(zf), Sse.Add(Sse.AndNot(sign, x), Sse.AndNot(sign, y)))

        // fixup octahedral coordinates for z<0
        let t = Sse.Min(z, Vector128.Create(0.0f))

        let x = Sse.Add(x, Sse.Xor(t, Sse.And(x, sign)))
        let y = Sse.Add(y, Sse.Xor(t, Sse.And(y, sign)))

        // compute normal length & scale
        let ll = Sse.Add(Sse.Multiply(x, x), Sse.Add(Sse.Multiply(y, y), Sse.Multiply(z, z)))
        let s = Sse.Divide(Vector128.Create(32767.0f), Sse.Sqrt(ll))

        // rounded signed float->int
        let xr = Sse2.ConvertToVector128Int32(Sse.Multiply(x, s))
        let yr = Sse2.ConvertToVector128Int32(Sse.Multiply(y, s))
        let zr = Sse2.ConvertToVector128Int32(Sse.Multiply(z, s))

        // mix x/z and y/0 to make 16-bit unpack easier
        let xzr = Sse2.Or(Sse2.And(xr, Vector128.Create(0xffff)), Sse2.ShiftLeftLogical(zr, 16uy))
        let y0r = Sse2.And(yr, Vector128.Create(0xffff))

        // pack x/y/z using 16-bit unpacks; note that this has 0 where we should have .w
        let res_0 = Sse2.UnpackLow(xzr.AsInt16(), y0r.AsInt16()).AsInt32()
        let res_1 = Sse2.UnpackHigh(xzr.AsInt16(), y0r.AsInt16()).AsInt32()

        // patch in .w
        // _mm_set_epi32(0xffff0000, 0, 0xffff0000, 0) -> maskw
        let maskw = Vector128.Create(0, int 0xffff0000, 0, int 0xffff0000)
        let res_0 = Sse2.Or(res_0, Sse2.And(n4_0.AsInt32(), maskw))
        let res_1 = Sse2.Or(res_1, Sse2.And(n4_1.AsInt32(), maskw))

        Sse2.Store(NPtr.cast<int16, int> (NPtr.add data ((i + 0) * 4)), res_0)
        Sse2.Store(NPtr.cast<int16, int> (NPtr.add data ((i + 2) * 4)), res_1)
        i <- i + 4

let private decodeFilterQuatSimd_SSE (data: nativeptr<int16>) (count: int) =
    let scale = 32767.0f / sqrt 2.0f

    let mutable i = 0
    while i < count do
        let q4_0 = Sse.LoadVector128(NPtr.cast<int16, float32> (NPtr.add data ((i + 0) * 4)))
        let q4_1 = Sse.LoadVector128(NPtr.cast<int16, float32> (NPtr.add data ((i + 2) * 4)))

        // gather both x/y 16-bit pairs in each 32-bit lane
        let q4_xy = Sse.Shuffle(q4_0, q4_1, 0x88uy).AsInt32() // _MM_SHUFFLE(2,0,2,0)
        let q4_zc = Sse.Shuffle(q4_0, q4_1, 0xDDuy).AsInt32() // _MM_SHUFFLE(3,1,3,1)

        // sign-extends each of x,y in [x y] with arithmetic shifts
        let xf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(q4_xy, 16uy), 16uy)
        let yf = Sse2.ShiftRightArithmetic(q4_xy, 16uy)
        let zf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(q4_zc, 16uy), 16uy)
        let cf = Sse2.ShiftRightArithmetic(q4_zc, 16uy)

        // get a floating-point scaler using zc with bottom 2 bits set to 1
        let sf = Sse2.Or(cf, Vector128.Create(3))
        let s = Sse2.ConvertToVector128Single(sf)

        // convert x/y/z to floating point
        let x = Sse2.ConvertToVector128Single(xf)
        let y = Sse2.ConvertToVector128Single(yf)
        let z = Sse2.ConvertToVector128Single(zf)

        // reconstruct w as a square root; we clamp to 0.f to avoid NaN
        let ws = Sse.Multiply(s, Sse.Add(s, s)) // s*2s instead of 2*(s*s) to work around clang bug
        let ww = Sse.Subtract(ws, Sse.Add(Sse.Multiply(x, x), Sse.Add(Sse.Multiply(y, y), Sse.Multiply(z, z))))
        let w = Sse.Sqrt(Sse.Max(ww, Vector128.Create(0.0f)))

        // compute final scale
        let ss = Sse.Divide(Vector128.Create(scale), s)

        // rounded signed float->int
        let xr = Sse2.ConvertToVector128Int32(Sse.Multiply(x, ss))
        let yr = Sse2.ConvertToVector128Int32(Sse.Multiply(y, ss))
        let zr = Sse2.ConvertToVector128Int32(Sse.Multiply(z, ss))
        let wr = Sse2.ConvertToVector128Int32(Sse.Multiply(w, ss))

        // mix x/z and w/y to make 16-bit unpack easier
        let xzr = Sse2.Or(Sse2.And(xr, Vector128.Create(0xffff)), Sse2.ShiftLeftLogical(zr, 16uy))
        let wyr = Sse2.Or(Sse2.And(wr, Vector128.Create(0xffff)), Sse2.ShiftLeftLogical(yr, 16uy))

        // pack x/y/z/w using 16-bit unpacks; we pack wxyz by default (for qc=0)
        let res_0 = Sse2.UnpackLow(wyr.AsInt16(), xzr.AsInt16())
        let res_1 = Sse2.UnpackHigh(wyr.AsInt16(), xzr.AsInt16())

        // store results to stack so that we can rotate using scalar instructions
        let res: uint64[] = Array.zeroCreate 4
        use pinRes = fixed res
        Sse2.Store(NPtr.cast<uint64, int16> pinRes, res_0)
        Sse2.Store(NPtr.cast<uint64, int16> (NPtr.add pinRes 2), res_1)

        // rotate and store
        let out = NPtr.cast<int16, uint64> (NPtr.add data (i * 4))

        NPtr.set out 0 (rotateleft64 res.[0] (int (NPtr.get data ((i + 0) * 4 + 3)) <<< 4))
        NPtr.set out 1 (rotateleft64 res.[1] (int (NPtr.get data ((i + 1) * 4 + 3)) <<< 4))
        NPtr.set out 2 (rotateleft64 res.[2] (int (NPtr.get data ((i + 2) * 4 + 3)) <<< 4))
        NPtr.set out 3 (rotateleft64 res.[3] (int (NPtr.get data ((i + 3) * 4 + 3)) <<< 4))
        i <- i + 4

let private decodeFilterExpSimd_SSE (data: nativeptr<uint32>) (count: int) =
    let mutable i = 0
    while i < count do
        let v = Sse2.LoadVector128(NPtr.cast<uint32, int> (NPtr.add data i))

        // decode exponent into 2^x directly
        let ef = Sse2.ShiftRightArithmetic(v, 24uy)
        let es = Sse2.ShiftLeftLogical(Sse2.Add(ef, Vector128.Create(127)), 23uy)

        // decode 24-bit mantissa into floating-point value
        let mf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(v, 8uy), 8uy)
        let m = Sse2.ConvertToVector128Single(mf)

        let r = Sse.Multiply(es.AsSingle(), m)

        Sse.Store(NPtr.cast<uint32, float32> (NPtr.add data i), r)
        i <- i + 4

let private decodeFilterColorSimd8_SSE (data: nativeptr<byte>) (count: int) =
    let mutable i = 0
    while i < count do
        let c4 = Sse2.LoadVector128(NPtr.cast<byte, int> (NPtr.add data (i * 4)))

        // unpack y/co/cg/a (co/cg are sign extended with arithmetic shifts)
        let yf = Sse2.And(c4, Vector128.Create(0xff))
        let cof = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(c4, 16uy), 24uy)
        let cgf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(c4, 8uy), 24uy)
        let af = Sse2.ShiftRightLogical(c4, 24uy)

        // recover scale from alpha high bit
        let mutable as_ = af
        as_ <- Sse2.Or(as_, Sse2.ShiftRightLogical(as_, 1uy))
        as_ <- Sse2.Or(as_, Sse2.ShiftRightLogical(as_, 2uy))
        as_ <- Sse2.Or(as_, Sse2.ShiftRightLogical(as_, 4uy))

        // expand alpha by one bit to match other components
        let af = Sse2.Or(Sse2.And(Sse2.ShiftLeftLogical(af, 1uy), as_), Sse2.And(af, Vector128.Create(1)))

        // compute scaling factor
        let ss = Sse.Multiply(Vector128.Create(255.0f), Sse.Reciprocal(Sse2.ConvertToVector128Single(as_)))

        // convert to RGB in fixed point
        let rf = Sse2.Add(yf, Sse2.Subtract(cof, cgf))
        let gf = Sse2.Add(yf, cgf)
        let bf = Sse2.Subtract(yf, Sse2.Add(cof, cgf))

        // rounded signed float->int
        let rr = Sse2.ConvertToVector128Int32(Sse.Multiply(Sse2.ConvertToVector128Single(rf), ss))
        let gr = Sse2.ConvertToVector128Int32(Sse.Multiply(Sse2.ConvertToVector128Single(gf), ss))
        let br = Sse2.ConvertToVector128Int32(Sse.Multiply(Sse2.ConvertToVector128Single(bf), ss))
        let ar = Sse2.ConvertToVector128Int32(Sse.Multiply(Sse2.ConvertToVector128Single(af), ss))

        // repack rgba into final value
        let mutable res = rr
        res <- Sse2.Or(res, Sse2.ShiftLeftLogical(gr, 8uy))
        res <- Sse2.Or(res, Sse2.ShiftLeftLogical(br, 16uy))
        res <- Sse2.Or(res, Sse2.ShiftLeftLogical(ar, 24uy))

        Sse2.Store(NPtr.cast<byte, int> (NPtr.add data (i * 4)), res)
        i <- i + 4

let private decodeFilterColorSimd16_SSE (data: nativeptr<uint16>) (count: int) =
    let mutable i = 0
    while i < count do
        let c4_0 = Sse2.LoadVector128(NPtr.cast<uint16, int> (NPtr.add data ((i + 0) * 4)))
        let c4_1 = Sse2.LoadVector128(NPtr.cast<uint16, int> (NPtr.add data ((i + 2) * 4)))

        // gather both y/co 16-bit pairs in each 32-bit lane
        let c4_yco = Sse.Shuffle(c4_0.AsSingle(), c4_1.AsSingle(), 0x88uy).AsInt32() // _MM_SHUFFLE(2,0,2,0)
        let c4_cga = Sse.Shuffle(c4_0.AsSingle(), c4_1.AsSingle(), 0xDDuy).AsInt32() // _MM_SHUFFLE(3,1,3,1)

        // unpack y/co/cg/a components (co/cg are sign extended with arithmetic shifts)
        let yf = Sse2.And(c4_yco, Vector128.Create(0xffff))
        let cof = Sse2.ShiftRightArithmetic(c4_yco, 16uy)
        let cgf = Sse2.ShiftRightArithmetic(Sse2.ShiftLeftLogical(c4_cga, 16uy), 16uy)
        let af = Sse2.ShiftRightLogical(c4_cga, 16uy)

        // recover scale from alpha high bit
        let mutable as_ = af
        as_ <- Sse2.Or(as_, Sse2.ShiftRightLogical(as_, 1uy))
        as_ <- Sse2.Or(as_, Sse2.ShiftRightLogical(as_, 2uy))
        as_ <- Sse2.Or(as_, Sse2.ShiftRightLogical(as_, 4uy))
        as_ <- Sse2.Or(as_, Sse2.ShiftRightLogical(as_, 8uy))

        // expand alpha by one bit to match other components
        let af = Sse2.Or(Sse2.And(Sse2.ShiftLeftLogical(af, 1uy), as_), Sse2.And(af, Vector128.Create(1)))

        // compute scaling factor
        let ss = Sse.Divide(Vector128.Create(65535.0f), Sse2.ConvertToVector128Single(as_))

        // convert to RGB in fixed point
        let rf = Sse2.Add(yf, Sse2.Subtract(cof, cgf))
        let gf = Sse2.Add(yf, cgf)
        let bf = Sse2.Subtract(yf, Sse2.Add(cof, cgf))

        // rounded signed float->int
        let rr = Sse2.ConvertToVector128Int32(Sse.Multiply(Sse2.ConvertToVector128Single(rf), ss))
        let gr = Sse2.ConvertToVector128Int32(Sse.Multiply(Sse2.ConvertToVector128Single(gf), ss))
        let br = Sse2.ConvertToVector128Int32(Sse.Multiply(Sse2.ConvertToVector128Single(bf), ss))
        let ar = Sse2.ConvertToVector128Int32(Sse.Multiply(Sse2.ConvertToVector128Single(af), ss))

        // mix r/b and g/a to make 16-bit unpack easier
        let rbr = Sse2.Or(Sse2.And(rr, Vector128.Create(0xffff)), Sse2.ShiftLeftLogical(br, 16uy))
        let gar = Sse2.Or(Sse2.And(gr, Vector128.Create(0xffff)), Sse2.ShiftLeftLogical(ar, 16uy))

        // pack r/g/b/a using 16-bit unpacks
        let res_0 = Sse2.UnpackLow(rbr.AsInt16(), gar.AsInt16())
        let res_1 = Sse2.UnpackHigh(rbr.AsInt16(), gar.AsInt16())

        Sse2.Store(NPtr.cast<uint16, int16> (NPtr.add data ((i + 0) * 4)), res_0)
        Sse2.Store(NPtr.cast<uint16, int16> (NPtr.add data ((i + 2) * 4)), res_1)
        i <- i + 4

// ---- NEON SIMD implementations ----

let private decodeFilterOctSimd8_NEON (data: nativeptr<sbyte>) (count: int) =
    let sign = Vector128.Create(int 0x80000000)

    let mutable i = 0
    while i < count do
        let n4 = AdvSimd.LoadVector128(NPtr.cast<sbyte, int> (NPtr.add data (i * 4)))

        // sign-extends each of x,y in [x y ? ?] with arithmetic shifts
        let xf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(n4.AsUInt32(), 24uy).AsInt32(), Vector128.Create(-24))
        let yf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(n4.AsUInt32(), 16uy).AsInt32(), Vector128.Create(-24))

        // unpack z
        let zf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(n4.AsUInt32(), 8uy).AsInt32(), Vector128.Create(-24))

        // convert x and y to floats and reconstruct z
        let x = AdvSimd.ConvertToSingle(xf)
        let y = AdvSimd.ConvertToSingle(yf)
        let z = AdvSimd.Subtract(AdvSimd.ConvertToSingle(zf), AdvSimd.Add(AdvSimd.Abs(x), AdvSimd.Abs(y)))

        // fixup octahedral coordinates for z<0
        let t = AdvSimd.Min(z, Vector128.Create(0.0f))

        let x = AdvSimd.Add(x, AdvSimd.Xor(t.AsUInt32(), AdvSimd.And(x.AsUInt32(), sign.AsUInt32())).AsSingle())
        let y = AdvSimd.Add(y, AdvSimd.Xor(t.AsUInt32(), AdvSimd.And(y.AsUInt32(), sign.AsUInt32())).AsSingle())

        // compute normal length & scale
        let ll = AdvSimd.Add(AdvSimd.Multiply(x, x), AdvSimd.Add(AdvSimd.Multiply(y, y), AdvSimd.Multiply(z, z)))
        let rl = AdvSimd.ReciprocalSquareRootEstimate(ll)
        let s = AdvSimd.Multiply(Vector128.Create(127.0f), rl)

        // fast rounded signed float->int: addition triggers renormalization after which mantissa stores the integer value
        // note: the result is offset by 0x4B40_0000, but we only need the low 8 bits so we can omit the subtraction
        let fsnap = Vector128.Create(float32 (3 <<< 22))

        let xr = AdvSimd.FusedMultiplyAdd(fsnap, x, s).AsInt32()
        let yr = AdvSimd.FusedMultiplyAdd(fsnap, y, s).AsInt32()
        let zr = AdvSimd.FusedMultiplyAdd(fsnap, z, s).AsInt32()

        // combine xr/yr/zr into final value
        // vsliq_n_s32(xr, vsliq_n_s32(yr, zr, 8), 8) is: shift-left and insert
        let yzr = AdvSimd.ShiftLeftAndInsert(yr.AsUInt32(), zr.AsUInt32(), 8uy)
        let res = AdvSimd.ShiftLeftAndInsert(xr.AsUInt32(), yzr, 8uy)

        // vbslq_s32(vdupq_n_u32(0xff000000), n4, res) - bitwise select: mask? n4 : res
        let res = AdvSimd.BitwiseSelect(Vector128.Create(0xff000000u), n4.AsUInt32(), res).AsInt32()

        let res128 : Vector128<int> = res
        AdvSimd.Store(NPtr.cast<sbyte, int> (NPtr.add data (i * 4)), res128)
        i <- i + 4

let private decodeFilterOctSimd16_NEON (data: nativeptr<int16>) (count: int) =
    let sign = Vector128.Create(int 0x80000000)

    let mutable i = 0
    while i < count do
        let n4_0 = AdvSimd.LoadVector128(NPtr.cast<int16, int> (NPtr.add data ((i + 0) * 4)))
        let n4_1 = AdvSimd.LoadVector128(NPtr.cast<int16, int> (NPtr.add data ((i + 2) * 4)))

        // gather both x/y 16-bit pairs in each 32-bit lane: vuzpq_s32(n4_0, n4_1).val[0]
        // UnzipEven gives the even-indexed elements
        let n4 = AdvSimd.Arm64.UnzipEven(n4_0, n4_1)

        // sign-extends each of x,y in [x y] with arithmetic shifts
        let xf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(n4.AsUInt32(), 16uy).AsInt32(), Vector128.Create(-16))
        let yf = AdvSimd.ShiftArithmetic(n4, Vector128.Create(-16))

        // unpack z; note that z is unsigned so we don't need to sign extend it
        let z4 = AdvSimd.Arm64.UnzipOdd(n4_0, n4_1)
        let zf = AdvSimd.And(z4, Vector128.Create(0x7fff))

        // convert x and y to floats and reconstruct z
        let x = AdvSimd.ConvertToSingle(xf)
        let y = AdvSimd.ConvertToSingle(yf)
        let z = AdvSimd.Subtract(AdvSimd.ConvertToSingle(zf), AdvSimd.Add(AdvSimd.Abs(x), AdvSimd.Abs(y)))

        // fixup octahedral coordinates for z<0
        let t = AdvSimd.Min(z, Vector128.Create(0.0f))

        let x = AdvSimd.Add(x, AdvSimd.Xor(t.AsUInt32(), AdvSimd.And(x.AsUInt32(), sign.AsUInt32())).AsSingle())
        let y = AdvSimd.Add(y, AdvSimd.Xor(t.AsUInt32(), AdvSimd.And(y.AsUInt32(), sign.AsUInt32())).AsSingle())

        // compute normal length & scale
        let ll = AdvSimd.Add(AdvSimd.Multiply(x, x), AdvSimd.Add(AdvSimd.Multiply(y, y), AdvSimd.Multiply(z, z)))
        let s : Vector128<float32> = AdvSimd.Arm64.Divide(Vector128.Create(32767.0f), AdvSimd.Arm64.Sqrt(ll))

        // fast rounded signed float->int
        let fsnap = Vector128.Create(float32 (3 <<< 22))

        let xr = AdvSimd.FusedMultiplyAdd(fsnap, x, s).AsInt32()
        let yr = AdvSimd.FusedMultiplyAdd(fsnap, y, s).AsInt32()
        let zr = AdvSimd.FusedMultiplyAdd(fsnap, z, s).AsInt32()

        // mix x/z and y/0 to make 16-bit unpack easier
        let xzr = AdvSimd.ShiftLeftAndInsert(xr.AsUInt32(), zr.AsUInt32(), 16uy).AsInt32()
        let y0r = AdvSimd.And(yr, Vector128.Create(0xffff))

        // pack x/y/z using 16-bit unpacks; vzipq_s16 val[0] / val[1]
        let res_0 : Vector128<int32> = AdvSimd.Arm64.ZipLow(xzr.AsInt16(), y0r.AsInt16()).AsInt32()
        let res_1 : Vector128<int32> = AdvSimd.Arm64.ZipHigh(xzr.AsInt16(), y0r.AsInt16()).AsInt32()

        // patch in .w - vbslq_s32 with mask 0xffff000000000000 per 64-bit lane
        let maskw = Vector128.Create(0xffff000000000000UL).AsUInt32()
        let res_0 = AdvSimd.BitwiseSelect(maskw, n4_0.AsUInt32(), res_0.AsUInt32()).AsInt32()
        let res_1 = AdvSimd.BitwiseSelect(maskw, n4_1.AsUInt32(), res_1.AsUInt32()).AsInt32()

        let r0 : Vector128<int> = res_0
        let r1 : Vector128<int> = res_1
        AdvSimd.Store(NPtr.cast<int16, int> (NPtr.add data ((i + 0) * 4)), r0)
        AdvSimd.Store(NPtr.cast<int16, int> (NPtr.add data ((i + 2) * 4)), r1)
        i <- i + 4

let private decodeFilterQuatSimd_NEON (data: nativeptr<int16>) (count: int) =
    let scale = 32767.0f / sqrt 2.0f

    let mutable i = 0
    while i < count do
        let q4_0 = AdvSimd.LoadVector128(NPtr.cast<int16, int> (NPtr.add data ((i + 0) * 4)))
        let q4_1 = AdvSimd.LoadVector128(NPtr.cast<int16, int> (NPtr.add data ((i + 2) * 4)))

        // gather both x/y 16-bit pairs in each 32-bit lane
        let q4_xy = AdvSimd.Arm64.UnzipEven(q4_0, q4_1)
        let q4_zc = AdvSimd.Arm64.UnzipOdd(q4_0, q4_1)

        // sign-extends
        let xf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(q4_xy.AsUInt32(), 16uy).AsInt32(), Vector128.Create(-16))
        let yf = AdvSimd.ShiftArithmetic(q4_xy, Vector128.Create(-16))
        let zf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(q4_zc.AsUInt32(), 16uy).AsInt32(), Vector128.Create(-16))
        let cf = AdvSimd.ShiftArithmetic(q4_zc, Vector128.Create(-16))

        // get a floating-point scaler using zc with bottom 2 bits set to 1
        let sf = AdvSimd.Or(cf, Vector128.Create(3))
        let s = AdvSimd.ConvertToSingle(sf)

        // convert x/y/z to floating point
        let x = AdvSimd.ConvertToSingle(xf)
        let y = AdvSimd.ConvertToSingle(yf)
        let z = AdvSimd.ConvertToSingle(zf)

        // reconstruct w as a square root
        let ws = AdvSimd.Multiply(s, s)
        let ww = AdvSimd.Subtract(AdvSimd.Add(ws, ws), AdvSimd.Add(AdvSimd.Multiply(x, x), AdvSimd.Add(AdvSimd.Multiply(y, y), AdvSimd.Multiply(z, z))))
        let w : Vector128<float32> = AdvSimd.Arm64.Sqrt(AdvSimd.Max(ww, Vector128.Create(0.0f)))

        // compute final scale
        let ss : Vector128<float32> = AdvSimd.Arm64.Divide(Vector128.Create(scale), s)

        // fast rounded signed float->int
        let fsnap = Vector128.Create(float32 (3 <<< 22))

        let xr = AdvSimd.FusedMultiplyAdd(fsnap, x, ss).AsInt32()
        let yr = AdvSimd.FusedMultiplyAdd(fsnap, y, ss).AsInt32()
        let zr = AdvSimd.FusedMultiplyAdd(fsnap, z, ss).AsInt32()
        let wr = AdvSimd.FusedMultiplyAdd(fsnap, w, ss).AsInt32()

        // mix x/z and w/y to make 16-bit unpack easier
        let xzr = AdvSimd.ShiftLeftAndInsert(xr.AsUInt32(), zr.AsUInt32(), 16uy).AsInt32()
        let wyr = AdvSimd.ShiftLeftAndInsert(wr.AsUInt32(), yr.AsUInt32(), 16uy).AsInt32()

        // pack x/y/z/w using 16-bit unpacks; we pack wxyz by default (for qc=0)
        let res_0 : Vector128<uint64> = AdvSimd.Arm64.ZipLow(wyr.AsInt16(), xzr.AsInt16()).AsUInt64()
        let res_1 : Vector128<uint64> = AdvSimd.Arm64.ZipHigh(wyr.AsInt16(), xzr.AsInt16()).AsUInt64()

        // store results to stack so that we can rotate using scalar instructions
        let res: uint64[] = Array.zeroCreate 4
        use pinRes = fixed res
        let r0v : Vector128<uint64> = res_0
        let r1v : Vector128<uint64> = res_1
        AdvSimd.Store(pinRes, r0v)
        AdvSimd.Store(NPtr.add pinRes 2, r1v)

        // rotate and store
        let out = NPtr.cast<int16, uint64> (NPtr.add data (i * 4))

        NPtr.set out 0 (rotateleft64 res.[0] (int (NPtr.get data ((i + 0) * 4 + 3)) <<< 4))
        NPtr.set out 1 (rotateleft64 res.[1] (int (NPtr.get data ((i + 1) * 4 + 3)) <<< 4))
        NPtr.set out 2 (rotateleft64 res.[2] (int (NPtr.get data ((i + 2) * 4 + 3)) <<< 4))
        NPtr.set out 3 (rotateleft64 res.[3] (int (NPtr.get data ((i + 3) * 4 + 3)) <<< 4))
        i <- i + 4

let private decodeFilterExpSimd_NEON (data: nativeptr<uint32>) (count: int) =
    let mutable i = 0
    while i < count do
        let v = AdvSimd.LoadVector128(NPtr.cast<uint32, int> (NPtr.add data i))

        // decode exponent into 2^x directly
        let ef = AdvSimd.ShiftArithmetic(v, Vector128.Create(-24))
        let es = AdvSimd.ShiftLeftLogical(AdvSimd.Add(ef, Vector128.Create(127)).AsUInt32(), 23uy).AsInt32()

        // decode 24-bit mantissa into floating-point value
        let mf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(v.AsUInt32(), 8uy).AsInt32(), Vector128.Create(-8))
        let m = AdvSimd.ConvertToSingle(mf)

        let r = AdvSimd.Multiply(es.AsSingle(), m)

        let rv : Vector128<float32> = r
        AdvSimd.Store(NPtr.cast<uint32, float32> (NPtr.add data i), rv)
        i <- i + 4

let private decodeFilterColorSimd8_NEON (data: nativeptr<byte>) (count: int) =
    let mutable i = 0
    while i < count do
        let c4 = AdvSimd.LoadVector128(NPtr.cast<byte, int> (NPtr.add data (i * 4)))

        // unpack y/co/cg/a (co/cg are sign extended with arithmetic shifts)
        let yf = AdvSimd.And(c4, Vector128.Create(0xff))
        let cof = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(c4.AsUInt32(), 16uy).AsInt32(), Vector128.Create(-24))
        let cgf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(c4.AsUInt32(), 8uy).AsInt32(), Vector128.Create(-24))
        let af = AdvSimd.ShiftRightLogical(c4.AsUInt32(), 24uy).AsInt32()

        // recover scale from alpha high bit
        let mutable as_ = af
        as_ <- AdvSimd.Or(as_, AdvSimd.ShiftArithmetic(as_, Vector128.Create(-1)))
        as_ <- AdvSimd.Or(as_, AdvSimd.ShiftArithmetic(as_, Vector128.Create(-2)))
        as_ <- AdvSimd.Or(as_, AdvSimd.ShiftArithmetic(as_, Vector128.Create(-4)))

        // expand alpha by one bit to match other components
        let af = AdvSimd.Or(AdvSimd.And(AdvSimd.ShiftLeftLogical(af.AsUInt32(), 1uy).AsInt32(), as_), AdvSimd.And(af, Vector128.Create(1)))

        // compute scaling factor
        let ss = AdvSimd.Multiply(Vector128.Create(255.0f), AdvSimd.ReciprocalEstimate(AdvSimd.ConvertToSingle(as_)))

        // convert to RGB in fixed point
        let rf = AdvSimd.Add(yf, AdvSimd.Subtract(cof, cgf))
        let gf = AdvSimd.Add(yf, cgf)
        let bf = AdvSimd.Subtract(yf, AdvSimd.Add(cof, cgf))

        // fast rounded signed float->int
        let fsnap = Vector128.Create(float32 (3 <<< 22))

        let rr = AdvSimd.FusedMultiplyAdd(fsnap, AdvSimd.ConvertToSingle(rf), ss).AsInt32()
        let gr = AdvSimd.FusedMultiplyAdd(fsnap, AdvSimd.ConvertToSingle(gf), ss).AsInt32()
        let br = AdvSimd.FusedMultiplyAdd(fsnap, AdvSimd.ConvertToSingle(bf), ss).AsInt32()
        let ar = AdvSimd.FusedMultiplyAdd(fsnap, AdvSimd.ConvertToSingle(af), ss).AsInt32()

        // repack rgba into final value
        // vsliq_n_s32(rr, vsliq_n_s32(gr, vsliq_n_s32(br, ar, 8), 8), 8)
        let bra = AdvSimd.ShiftLeftAndInsert(br.AsUInt32(), ar.AsUInt32(), 8uy)
        let gbra = AdvSimd.ShiftLeftAndInsert(gr.AsUInt32(), bra, 8uy)
        let res = AdvSimd.ShiftLeftAndInsert(rr.AsUInt32(), gbra, 8uy).AsInt32()

        let res128 : Vector128<int> = res
        AdvSimd.Store(NPtr.cast<byte, int> (NPtr.add data (i * 4)), res128)
        i <- i + 4

let private decodeFilterColorSimd16_NEON (data: nativeptr<uint16>) (count: int) =
    let mutable i = 0
    while i < count do
        let c4_0 = AdvSimd.LoadVector128(NPtr.cast<uint16, int> (NPtr.add data ((i + 0) * 4)))
        let c4_1 = AdvSimd.LoadVector128(NPtr.cast<uint16, int> (NPtr.add data ((i + 2) * 4)))

        // gather both y/co 16-bit pairs in each 32-bit lane
        let c4_yco = AdvSimd.Arm64.UnzipEven(c4_0, c4_1)
        let c4_cga = AdvSimd.Arm64.UnzipOdd(c4_0, c4_1)

        // unpack y/co/cg/a components
        let yf = AdvSimd.And(c4_yco, Vector128.Create(0xffff))
        let cof = AdvSimd.ShiftArithmetic(c4_yco, Vector128.Create(-16))
        let cgf = AdvSimd.ShiftArithmetic(AdvSimd.ShiftLeftLogical(c4_cga.AsUInt32(), 16uy).AsInt32(), Vector128.Create(-16))
        let af = AdvSimd.ShiftRightLogical(c4_cga.AsUInt32(), 16uy).AsInt32()

        // recover scale from alpha high bit
        let mutable as_ = af
        as_ <- AdvSimd.Or(as_, AdvSimd.ShiftArithmetic(as_, Vector128.Create(-1)))
        as_ <- AdvSimd.Or(as_, AdvSimd.ShiftArithmetic(as_, Vector128.Create(-2)))
        as_ <- AdvSimd.Or(as_, AdvSimd.ShiftArithmetic(as_, Vector128.Create(-4)))
        as_ <- AdvSimd.Or(as_, AdvSimd.ShiftArithmetic(as_, Vector128.Create(-8)))

        // expand alpha by one bit to match other components
        let af = AdvSimd.Or(AdvSimd.And(AdvSimd.ShiftLeftLogical(af.AsUInt32(), 1uy).AsInt32(), as_), AdvSimd.And(af, Vector128.Create(1)))

        // compute scaling factor
        let ss : Vector128<float32> = AdvSimd.Arm64.Divide(Vector128.Create(65535.0f), AdvSimd.ConvertToSingle(as_))

        // convert to RGB in fixed point
        let rf = AdvSimd.Add(yf, AdvSimd.Subtract(cof, cgf))
        let gf = AdvSimd.Add(yf, cgf)
        let bf = AdvSimd.Subtract(yf, AdvSimd.Add(cof, cgf))

        // fast rounded signed float->int
        let fsnap = Vector128.Create(float32 (3 <<< 22))

        let rr = AdvSimd.FusedMultiplyAdd(fsnap, AdvSimd.ConvertToSingle(rf), ss).AsInt32()
        let gr = AdvSimd.FusedMultiplyAdd(fsnap, AdvSimd.ConvertToSingle(gf), ss).AsInt32()
        let br = AdvSimd.FusedMultiplyAdd(fsnap, AdvSimd.ConvertToSingle(bf), ss).AsInt32()
        let ar = AdvSimd.FusedMultiplyAdd(fsnap, AdvSimd.ConvertToSingle(af), ss).AsInt32()

        // mix r/b and g/a to make 16-bit unpack easier
        let rbr = AdvSimd.ShiftLeftAndInsert(rr.AsUInt32(), br.AsUInt32(), 16uy).AsInt32()
        let gar = AdvSimd.ShiftLeftAndInsert(gr.AsUInt32(), ar.AsUInt32(), 16uy).AsInt32()

        // pack r/g/b/a using 16-bit unpacks
        let res_0 : Vector128<int32> = AdvSimd.Arm64.ZipLow(rbr.AsInt16(), gar.AsInt16()).AsInt32()
        let res_1 : Vector128<int32> = AdvSimd.Arm64.ZipHigh(rbr.AsInt16(), gar.AsInt16()).AsInt32()

        let r0 : Vector128<int> = res_0
        let r1 : Vector128<int> = res_1
        AdvSimd.Store(NPtr.cast<uint16, int> (NPtr.add data ((i + 0) * 4)), r0)
        AdvSimd.Store(NPtr.cast<uint16, int> (NPtr.add data ((i + 2) * 4)), r1)
        i <- i + 4

// ---- helper functions for encode ----

// optimized variant of frexp
let inline private optlog2 (v: float32) : int =
    let ui = Unsafe.BitCast<float32, uint32>(v)
    // +1 accounts for implicit 1. in mantissa; denormalized numbers will end up clamped to min_exp by calling code
    if v = 0.0f then 0 else int ((ui >>> 23) &&& 0xffu) - 127 + 1

// optimized variant of ldexp
let inline private optexp2 (e: int) : float32 =
    let ui = uint32 (e + 127) <<< 23
    Unsafe.BitCast<uint32, float32>(ui)

// ---- public API functions ----

let meshopt_decodeFilterOct (buffer: voidptr) (count: int) (stride: int) =
    assert (stride = 4 || stride = 8)

    if Sse2.IsSupported then
        if stride = 4 then
            dispatchSimd decodeFilterOctSimd8_SSE decodeFilterOctScalar8 (NPtr.ofVoid<sbyte> buffer) count 4
        else
            dispatchSimd decodeFilterOctSimd16_SSE decodeFilterOctScalar16 (NPtr.ofVoid<int16> buffer) count 4
    elif AdvSimd.Arm64.IsSupported then
        if stride = 4 then
            dispatchSimd decodeFilterOctSimd8_NEON decodeFilterOctScalar8 (NPtr.ofVoid<sbyte> buffer) count 4
        else
            dispatchSimd decodeFilterOctSimd16_NEON decodeFilterOctScalar16 (NPtr.ofVoid<int16> buffer) count 4
    else
        if stride = 4 then
            decodeFilterOctScalar8 (NPtr.ofVoid<sbyte> buffer) count
        else
            decodeFilterOctScalar16 (NPtr.ofVoid<int16> buffer) count

let meshopt_decodeFilterQuat (buffer: voidptr) (count: int) (stride: int) =
    assert (stride = 8)
    ignore stride

    if Sse2.IsSupported then
        dispatchSimd decodeFilterQuatSimd_SSE decodeFilterQuatScalar (NPtr.ofVoid<int16> buffer) count 4
    elif AdvSimd.Arm64.IsSupported then
        dispatchSimd decodeFilterQuatSimd_NEON decodeFilterQuatScalar (NPtr.ofVoid<int16> buffer) count 4
    else
        decodeFilterQuatScalar (NPtr.ofVoid<int16> buffer) count

let meshopt_decodeFilterExp (buffer: voidptr) (count: int) (stride: int) =
    assert (stride > 0 && stride % 4 = 0)

    if Sse2.IsSupported then
        dispatchSimd decodeFilterExpSimd_SSE decodeFilterExpScalar (NPtr.ofVoid<uint32> buffer) (count * (stride / 4)) 1
    elif AdvSimd.Arm64.IsSupported then
        dispatchSimd decodeFilterExpSimd_NEON decodeFilterExpScalar (NPtr.ofVoid<uint32> buffer) (count * (stride / 4)) 1
    else
        decodeFilterExpScalar (NPtr.ofVoid<uint32> buffer) (count * (stride / 4))

let meshopt_decodeFilterColor (buffer: voidptr) (count: int) (stride: int) =
    assert (stride = 4 || stride = 8)

    if Sse2.IsSupported then
        if stride = 4 then
            dispatchSimd decodeFilterColorSimd8_SSE decodeFilterColorScalar8 (NPtr.ofVoid<byte> buffer) count 4
        else
            dispatchSimd decodeFilterColorSimd16_SSE decodeFilterColorScalar16 (NPtr.ofVoid<uint16> buffer) count 4
    elif AdvSimd.Arm64.IsSupported then
        if stride = 4 then
            dispatchSimd decodeFilterColorSimd8_NEON decodeFilterColorScalar8 (NPtr.ofVoid<byte> buffer) count 4
        else
            dispatchSimd decodeFilterColorSimd16_NEON decodeFilterColorScalar16 (NPtr.ofVoid<uint16> buffer) count 4
    else
        if stride = 4 then
            decodeFilterColorScalar8 (NPtr.ofVoid<byte> buffer) count
        else
            decodeFilterColorScalar16 (NPtr.ofVoid<uint16> buffer) count

let meshopt_encodeFilterOct (destination: voidptr) (count: int) (stride: int) (bits: int) (data: nativeptr<float32>) =
    assert (stride = 4 || stride = 8)
    assert (bits >= 2 && bits <= 16)

    let d8 = NPtr.ofVoid<sbyte> destination
    let d16 = NPtr.ofVoid<int16> destination

    let bytebits = int (stride * 2)

    for i = 0 to count - 1 do
        let n = NPtr.add data (i * 4)

        // octahedral encoding of a unit vector
        let nx = NPtr.get n 0
        let ny = NPtr.get n 1
        let nz = NPtr.get n 2
        let nw = NPtr.get n 3
        let nl = abs nx + abs ny + abs nz
        let ns = if nl = 0.0f then 0.0f else 1.0f / nl

        let nx = nx * ns
        let ny = ny * ns

        let u = if nz >= 0.0f then nx else (1.0f - abs ny) * (if nx >= 0.0f then 1.0f else -1.0f)
        let v = if nz >= 0.0f then ny else (1.0f - abs nx) * (if ny >= 0.0f then 1.0f else -1.0f)

        let fu = meshopt_quantizeSnorm u bits
        let fv = meshopt_quantizeSnorm v bits
        let fo = meshopt_quantizeSnorm 1.0f bits
        let fw = meshopt_quantizeSnorm nw bytebits

        if stride = 4 then
            NPtr.set d8 (i * 4 + 0) (sbyte fu)
            NPtr.set d8 (i * 4 + 1) (sbyte fv)
            NPtr.set d8 (i * 4 + 2) (sbyte fo)
            NPtr.set d8 (i * 4 + 3) (sbyte fw)
        else
            NPtr.set d16 (i * 4 + 0) (int16 fu)
            NPtr.set d16 (i * 4 + 1) (int16 fv)
            NPtr.set d16 (i * 4 + 2) (int16 fo)
            NPtr.set d16 (i * 4 + 3) (int16 fw)

let meshopt_encodeFilterQuat (destination_: voidptr) (count: int) (stride: int) (bits: int) (data: nativeptr<float32>) =
    assert (stride = 8)
    assert (bits >= 4 && bits <= 16)
    ignore stride

    let destination = NPtr.ofVoid<int16> destination_

    let scaler = sqrt 2.0f

    for i = 0 to count - 1 do
        let q = NPtr.add data (i * 4)
        let d = NPtr.add destination (i * 4)

        // establish maximum quaternion component
        let mutable qc = 0
        qc <- if abs (NPtr.get q 1) > abs (NPtr.get q qc) then 1 else qc
        qc <- if abs (NPtr.get q 2) > abs (NPtr.get q qc) then 2 else qc
        qc <- if abs (NPtr.get q 3) > abs (NPtr.get q qc) then 3 else qc

        // we use double-cover properties to discard the sign
        let sign = if NPtr.get q qc < 0.0f then -1.0f else 1.0f

        // note: we always encode a cyclical swizzle to be able to recover the order via rotation
        NPtr.set d 0 (int16 (meshopt_quantizeSnorm (NPtr.get q ((qc + 1) &&& 3) * scaler * sign) bits))
        NPtr.set d 1 (int16 (meshopt_quantizeSnorm (NPtr.get q ((qc + 2) &&& 3) * scaler * sign) bits))
        NPtr.set d 2 (int16 (meshopt_quantizeSnorm (NPtr.get q ((qc + 3) &&& 3) * scaler * sign) bits))
        NPtr.set d 3 (int16 ((meshopt_quantizeSnorm 1.0f bits &&& ~~~3) ||| qc))

let meshopt_encodeFilterExp (destination_: voidptr) (count: int) (stride: int) (bits: int) (data: nativeptr<float32>) (mode: meshopt_EncodeExpMode) =
    assert (stride > 0 && stride % 4 = 0 && stride <= 256)
    assert (bits >= 1 && bits <= 24)

    let destination = NPtr.ofVoid<uint32> destination_
    let stride_float = stride / sizeof<float32>

    let component_exp: int[] = Array.zeroCreate 64
    assert (stride_float <= component_exp.Length)

    let min_exp = -100

    if mode = meshopt_EncodeExpMode.SharedComponent then
        for j = 0 to stride_float - 1 do
            component_exp.[j] <- min_exp

        for i = 0 to count - 1 do
            let v = NPtr.add data (i * stride_float)

            // use maximum exponent to encode values; this guarantees that mantissa is [-1, 1]
            for j = 0 to stride_float - 1 do
                let e = optlog2 (NPtr.get v j)
                component_exp.[j] <- if component_exp.[j] < e then e else component_exp.[j]

    for i = 0 to count - 1 do
        let v = NPtr.add data (i * stride_float)
        let d = NPtr.add destination (i * stride_float)

        let mutable vector_exp = min_exp

        if mode = meshopt_EncodeExpMode.SharedVector then
            // use maximum exponent to encode values; this guarantees that mantissa is [-1, 1]
            for j = 0 to stride_float - 1 do
                let e = optlog2 (NPtr.get v j)
                vector_exp <- if vector_exp < e then e else vector_exp
        elif mode = meshopt_EncodeExpMode.Separate then
            for j = 0 to stride_float - 1 do
                let e = optlog2 (NPtr.get v j)
                component_exp.[j] <- if min_exp < e then e else min_exp
        elif mode = meshopt_EncodeExpMode.Clamped then
            for j = 0 to stride_float - 1 do
                let e = optlog2 (NPtr.get v j)
                component_exp.[j] <- if 0 < e then e else 0
        else
            // the code below assumes component_exp is initialized outside of the loop
            assert (mode = meshopt_EncodeExpMode.SharedComponent)

        for j = 0 to stride_float - 1 do
            let exp = if mode = meshopt_EncodeExpMode.SharedVector then vector_exp else component_exp.[j]

            // note that we additionally scale the mantissa to make it a K-bit signed integer (K-1 bits for magnitude)
            let exp = exp - (bits - 1)

            // compute renormalized rounded mantissa for each component
            let mmask = (1 <<< 24) - 1
            let m = int (NPtr.get v j * optexp2 (-exp) + (if NPtr.get v j >= 0.0f then 0.5f else -0.5f))

            NPtr.set d j (uint32 (m &&& mmask) ||| (uint32 exp <<< 24))

let meshopt_encodeFilterColor (destination: voidptr) (count: int) (stride: int) (bits: int) (data: nativeptr<float32>) =
    assert (stride = 4 || stride = 8)
    assert (bits >= 2 && bits <= 16)

    let d8 = NPtr.ofVoid<byte> destination
    let d16 = NPtr.ofVoid<uint16> destination

    for i = 0 to count - 1 do
        let c = NPtr.add data (i * 4)

        let fr = meshopt_quantizeUnorm (NPtr.get c 0) bits
        let fg = meshopt_quantizeUnorm (NPtr.get c 1) bits
        let fb = meshopt_quantizeUnorm (NPtr.get c 2) bits

        // YCoCg-R encoding with truncated Co/Cg ensures that decoding can be done using integers
        let fco = (fr - fb) / 2
        let tmp = fb + fco
        let fcg = (fg - tmp) / 2
        let fy = tmp + fcg

        // validate that R/G/B can be reconstructed with K bit integers
        assert (uint32 ((fy + fco - fcg) ||| (fy + fcg) ||| (fy - fco - fcg)) < (1u <<< bits))

        // alpha: K-1-bit encoding with high bit set to 1
        let fa = (meshopt_quantizeUnorm (NPtr.get c 3) bits >>> 1) ||| (1 <<< (bits - 1))

        if stride = 4 then
            NPtr.set d8 (i * 4 + 0) (byte fy)
            NPtr.set d8 (i * 4 + 1) (byte fco)
            NPtr.set d8 (i * 4 + 2) (byte fcg)
            NPtr.set d8 (i * 4 + 3) (byte fa)
        else
            NPtr.set d16 (i * 4 + 0) (uint16 fy)
            NPtr.set d16 (i * 4 + 1) (uint16 fco)
            NPtr.set d16 (i * 4 + 2) (uint16 fcg)
            NPtr.set d16 (i * 4 + 3) (uint16 fa)
