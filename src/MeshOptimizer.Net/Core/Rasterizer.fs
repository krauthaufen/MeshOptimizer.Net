// This file is part of MeshOptimizer.Net; see meshoptimizer.h for version/license details
module MeshOptimizer.Net.Rasterizer

// This work is based on:
// Nicolas Capens. Advanced Rasterization. 2004

open System
open MeshOptimizer.Net
open MeshOptimizer.Net.Allocator

[<Literal>]
let private kViewport = 256

[<Sealed>]
type private OverdrawBuffer() =
    let z = Array.zeroCreate<float32>(kViewport * kViewport * 2)
    let overdraw = Array.zeroCreate<uint32>(kViewport * kViewport * 2)

    member _.Clear() =
        Array.Clear(z, 0, z.Length)
        Array.Clear(overdraw, 0, overdraw.Length)

    member _.GetZ(y, x, s) = z.[(y * kViewport + x) * 2 + s]
    member _.SetZ(y, x, s, v) = z.[(y * kViewport + x) * 2 + s] <- v
    member _.GetOverdraw(y, x, s) = overdraw.[(y * kViewport + x) * 2 + s]
    member _.SetOverdraw(y, x, s, v) = overdraw.[(y * kViewport + x) * 2 + s] <- v

let private computeDepthGradients (x1: float32) (y1: float32) (z1: float32) (x2: float32) (y2: float32) (z2: float32) (x3: float32) (y3: float32) (z3: float32) =
    let det = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
    let invdet = if det = 0.0f then 0.0f else 1.0f / det

    let dzdx = ((z2 - z1) * (y3 - y1) - (y2 - y1) * (z3 - z1)) * invdet
    let dzdy = ((x2 - x1) * (z3 - z1) - (z2 - z1) * (x3 - x1)) * invdet

    struct (det, dzdx, dzdy)

let private rasterize (buffer: OverdrawBuffer) (v1x: float32) (v1y: float32) (v1z: float32) (v2x: float32) (v2y: float32) (v2z: float32) (v3x: float32) (v3y: float32) (v3z: float32) =
    let struct (det, DZx0, DZy0) = computeDepthGradients v1x v1y v1z v2x v2y v2z v3x v3y v3z
    let mutable DZx = DZx0
    let mutable DZy = DZy0
    let sign = if det > 0.0f then 1 else 0

    let mutable v2x = v2x
    let mutable v2y = v2y
    let mutable v3x = v3x
    let mutable v3y = v3y
    let mutable v1z = v1z

    if sign <> 0 then
        let t = v2x in v2x <- v3x; v3x <- t
        let t = v2y in v2y <- v3y; v3y <- t
        v1z <- float32 kViewport - v1z
        DZx <- -DZx
        DZy <- -DZy

    let X1 = int (16.0f * v1x + 0.5f)
    let X2 = int (16.0f * v2x + 0.5f)
    let X3 = int (16.0f * v3x + 0.5f)

    let Y1 = int (16.0f * v1y + 0.5f)
    let Y2 = int (16.0f * v2y + 0.5f)
    let Y3 = int (16.0f * v3y + 0.5f)

    let mutable minx = if X1 < X2 then X1 else X2
    minx <- if minx < X3 then minx else X3
    minx <- (minx + 7) >>> 4
    minx <- if minx < 0 then 0 else minx

    let mutable miny = if Y1 < Y2 then Y1 else Y2
    miny <- if miny < Y3 then miny else Y3
    miny <- (miny + 7) >>> 4
    miny <- if miny < 0 then 0 else miny

    let mutable maxx = if X1 > X2 then X1 else X2
    maxx <- if maxx > X3 then maxx else X3
    maxx <- (maxx + 7) >>> 4
    maxx <- if maxx > kViewport then kViewport else maxx

    let mutable maxy = if Y1 > Y2 then Y1 else Y2
    maxy <- if maxy > Y3 then maxy else Y3
    maxy <- (maxy + 7) >>> 4
    maxy <- if maxy > kViewport then kViewport else maxy

    let DX12 = X1 - X2
    let DX23 = X2 - X3
    let DX31 = X3 - X1

    let DY12 = Y1 - Y2
    let DY23 = Y2 - Y3
    let DY31 = Y3 - Y1

    let TL1 = if DY12 < 0 || (DY12 = 0 && DX12 > 0) then 1 else 0
    let TL2 = if DY23 < 0 || (DY23 = 0 && DX23 > 0) then 1 else 0
    let TL3 = if DY31 < 0 || (DY31 = 0 && DX31 > 0) then 1 else 0

    let FX = (minx <<< 4) + 8
    let FY = (miny <<< 4) + 8
    let mutable CY1 = DX12 * (FY - Y1) - DY12 * (FX - X1) + TL1 - 1
    let mutable CY2 = DX23 * (FY - Y2) - DY23 * (FX - X2) + TL2 - 1
    let mutable CY3 = DX31 * (FY - Y3) - DY31 * (FX - X3) + TL3 - 1
    let mutable ZY = v1z + (DZx * float32 (FX - X1) + DZy * float32 (FY - Y1)) * (1.0f / 16.0f)

    for y = miny to maxy - 1 do
        let mutable CX1 = CY1
        let mutable CX2 = CY2
        let mutable CX3 = CY3
        let mutable ZX = ZY

        for x = minx to maxx - 1 do
            if (CX1 ||| CX2 ||| CX3) >= 0 then
                if ZX >= buffer.GetZ(y, x, sign) then
                    buffer.SetZ(y, x, sign, ZX)
                    buffer.SetOverdraw(y, x, sign, buffer.GetOverdraw(y, x, sign) + 1u)

            CX1 <- CX1 - int (uint32 DY12 <<< 4)
            CX2 <- CX2 - int (uint32 DY23 <<< 4)
            CX3 <- CX3 - int (uint32 DY31 <<< 4)
            ZX <- ZX + DZx

        CY1 <- CY1 + int (uint32 DX12 <<< 4)
        CY2 <- CY2 + int (uint32 DX23 <<< 4)
        CY3 <- CY3 + int (uint32 DX31 <<< 4)
        ZY <- ZY + DZy

let private transformTriangles (triangles: float32[]) (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) : float32 =
    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    let mutable minv0 = Single.MaxValue
    let mutable minv1 = Single.MaxValue
    let mutable minv2 = Single.MaxValue
    let mutable maxv0 = -Single.MaxValue
    let mutable maxv1 = -Single.MaxValue
    let mutable maxv2 = -Single.MaxValue

    for i = 0 to vertex_count - 1 do
        let vbase = i * vertex_stride_float
        let vj0 = NPtr.get vertex_positions (vbase + 0)
        let vj1 = NPtr.get vertex_positions (vbase + 1)
        let vj2 = NPtr.get vertex_positions (vbase + 2)

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

    let scale = float32 kViewport / extent

    for i = 0 to index_count - 1 do
        let index = int (NPtr.get indices i)
        assert (index < vertex_count)
        let vbase = index * vertex_stride_float

        triangles.[i * 3 + 0] <- (NPtr.get vertex_positions (vbase + 0) - minv0) * scale
        triangles.[i * 3 + 1] <- (NPtr.get vertex_positions (vbase + 1) - minv1) * scale
        triangles.[i * 3 + 2] <- (NPtr.get vertex_positions (vbase + 2) - minv2) * scale

    extent

let private rasterizeTriangles (buffer: OverdrawBuffer) (triangles: float32[]) (index_count: int) (axis: int) =
    let mutable i = 0
    while i < index_count do
        let vn0 = i * 3
        let vn1 = (i + 1) * 3
        let vn2 = (i + 2) * 3

        match axis with
        | 0 ->
            rasterize buffer triangles.[vn0+2] triangles.[vn0+1] triangles.[vn0+0]
                               triangles.[vn1+2] triangles.[vn1+1] triangles.[vn1+0]
                               triangles.[vn2+2] triangles.[vn2+1] triangles.[vn2+0]
        | 1 ->
            rasterize buffer triangles.[vn0+0] triangles.[vn0+2] triangles.[vn0+1]
                               triangles.[vn1+0] triangles.[vn1+2] triangles.[vn1+1]
                               triangles.[vn2+0] triangles.[vn2+2] triangles.[vn2+1]
        | 2 ->
            rasterize buffer triangles.[vn0+1] triangles.[vn0+0] triangles.[vn0+2]
                               triangles.[vn1+1] triangles.[vn1+0] triangles.[vn1+2]
                               triangles.[vn2+1] triangles.[vn2+0] triangles.[vn2+2]
        | _ -> ()

        i <- i + 3

let meshopt_analyzeOverdraw (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) : meshopt_OverdrawStatistics =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    let mutable result = meshopt_OverdrawStatistics()

    let triangles = Array.zeroCreate<float32>(index_count * 3)
    transformTriangles triangles indices index_count vertex_positions vertex_count vertex_positions_stride |> ignore

    let buffer = OverdrawBuffer()

    for axis = 0 to 2 do
        buffer.Clear()
        rasterizeTriangles buffer triangles index_count axis

        for y = 0 to kViewport - 1 do
            for x = 0 to kViewport - 1 do
                for s = 0 to 1 do
                    let od = buffer.GetOverdraw(y, x, s)
                    result.pixels_covered <- result.pixels_covered + (if od > 0u then 1u else 0u)
                    result.pixels_shaded <- result.pixels_shaded + od

    result.overdraw <- if result.pixels_covered <> 0u then float32 result.pixels_shaded / float32 result.pixels_covered else 0.0f

    result

let meshopt_analyzeCoverage (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) : meshopt_CoverageStatistics =
    assert (index_count % 3 = 0)
    assert (vertex_positions_stride >= 12 && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)

    let mutable result = meshopt_CoverageStatistics()

    let triangles = Array.zeroCreate<float32>(index_count * 3)
    let extent = transformTriangles triangles indices index_count vertex_positions vertex_count vertex_positions_stride

    let buffer = OverdrawBuffer()

    for axis = 0 to 2 do
        buffer.Clear()
        rasterizeTriangles buffer triangles index_count axis

        let mutable covered = 0u

        for y = 0 to kViewport - 1 do
            for x = 0 to kViewport - 1 do
                if (buffer.GetOverdraw(y, x, 0) ||| buffer.GetOverdraw(y, x, 1)) > 0u then
                    covered <- covered + 1u

        let cov = float32 covered / float32 (kViewport * kViewport)
        match axis with
        | 0 -> result.coverage_0 <- cov
        | 1 -> result.coverage_1 <- cov
        | 2 -> result.coverage_2 <- cov
        | _ -> ()

    result.extent <- extent

    result
