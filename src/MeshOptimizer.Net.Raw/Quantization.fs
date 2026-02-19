// This file is part of MeshOptimizer.Net; see meshoptimizer.h for version/license details
module MeshOptimizer.Net.Quantization

open System.Runtime.CompilerServices
open System.Runtime.InteropServices

/// Quantize a float into half-precision (IEEE-754 fp16) floating point value
/// Generates +-inf for overflow, preserves NaN, flushes denormals to zero, rounds to nearest
/// Representable magnitude range: [6e-5; 65504]
/// Maximum relative reconstruction error: 5e-4
let meshopt_quantizeHalf (v: float32) : uint16 =
    let ui = Unsafe.BitCast<float32, uint32>(v)

    let s = int ((ui >>> 16) &&& 0x8000u)
    let em = int (ui &&& 0x7fffffffu)

    // bias exponent and round to nearest; 112 is relative exponent bias (127-15)
    let mutable h = (em - (112 <<< 23) + (1 <<< 12)) >>> 13

    // underflow: flush to zero; 113 encodes exponent -14
    h <- if em < (113 <<< 23) then 0 else h

    // overflow: infinity; 143 encodes exponent 16
    h <- if em >= (143 <<< 23) then 0x7c00 else h

    // NaN; note that we convert all types of NaN to qNaN
    h <- if em > (255 <<< 23) then 0x7e00 else h

    uint16 (s ||| h)

/// Quantize a float into a floating point value with a limited number of significant mantissa bits,
/// preserving the IEEE-754 fp32 binary representation
/// Preserves infinities/NaN, flushes denormals to zero, rounds to nearest
/// Assumes N is in a valid mantissa precision range, which is 1..23
let meshopt_quantizeFloat (v: float32) (N: int) : float32 =
    assert (N >= 0 && N <= 23)

    let mutable ui = Unsafe.BitCast<float32, uint32>(v)

    let mask = uint32 ((1 <<< (23 - N)) - 1)
    let round = uint32 ((1 <<< (23 - N)) >>> 1)

    let e = ui &&& 0x7f800000u
    let rui = (ui + round) &&& (~~~mask)

    // round all numbers except inf/nan; this is important to make sure nan doesn't overflow into -0
    ui <- if e = 0x7f800000u then ui else rui

    // flush denormals to zero
    ui <- if e = 0u then 0u else ui

    Unsafe.BitCast<uint32, float32>(ui)

/// Reverse quantization of a half-precision (IEEE-754 fp16) floating point value
/// Preserves Inf/NaN, flushes denormals to zero
let meshopt_dequantizeHalf (h: uint16) : float32 =
    let s = uint32 (uint32 (h &&& 0x8000us) <<< 16)
    let em = int (h &&& 0x7fffus)

    // bias exponent and pad mantissa with 0; 112 is relative exponent bias (127-15)
    let mutable r = (em + (112 <<< 10)) <<< 13

    // denormal: flush to zero
    r <- if em < (1 <<< 10) then 0 else r

    // infinity/NaN; note that we preserve NaN payload as a byproduct of unifying inf/nan cases
    // 112 is an exponent bias fixup; since we already applied it once, applying it twice converts 31 to 255
    r <- r + (if em >= (31 <<< 10) then (112 <<< 23) else 0)

    Unsafe.BitCast<uint32, float32>(s ||| uint32 r)

/// Quantize a float in [0..1] range into an N-bit fixed point unorm value
/// Assumes reconstruction function (q / (2^N-1)), which is the case for fixed-function normalized fixed point conversion
/// Maximum reconstruction error: 1/2^(N+1)
let meshopt_quantizeUnorm (v: float32) (N: int) : int =
    let scale = float32 ((1 <<< N) - 1)

    let v = if v >= 0.0f then v else 0.0f
    let v = if v <= 1.0f then v else 1.0f

    int (v * scale + 0.5f)

/// Quantize a float in [-1..1] range into an N-bit fixed point snorm value
/// Assumes reconstruction function (q / (2^(N-1)-1)), which is the case for fixed-function normalized fixed point conversion
/// Maximum reconstruction error: 1/2^N
let meshopt_quantizeSnorm (v: float32) (N: int) : int =
    let scale = float32 ((1 <<< (N - 1)) - 1)

    let round = if v >= 0.0f then 0.5f else -0.5f

    let v = if v >= -1.0f then v else -1.0f
    let v = if v <= 1.0f then v else 1.0f

    int (v * scale + round)
