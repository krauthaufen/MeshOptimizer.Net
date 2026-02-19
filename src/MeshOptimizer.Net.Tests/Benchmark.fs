// Performance benchmark: F# port vs C++ reference (via P/Invoke)
// Uses BenchmarkDotNet for statistically rigorous measurements
// Run with: LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project MeshOptimizer.Net.Tests -- --bench
module MeshOptimizer.Net.Tests.Benchmark

open System
open System.Runtime.InteropServices
open BenchmarkDotNet.Attributes
open BenchmarkDotNet.Columns
open BenchmarkDotNet.Configs
open BenchmarkDotNet.Diagnosers
open BenchmarkDotNet.Jobs
open MeshOptimizer.Net
open MeshOptimizer.Net.Tests.ObjLoader

#nowarn "9"

module private NPtr =
    open FSharp.NativeInterop
    let inline ofNI<'T when 'T : unmanaged> (n: nativeint) : nativeptr<'T> = NativePtr.ofNativeInt n

let inline private pinArray (arr: 'T[]) (f: nativeint -> 'R) : 'R =
    let h = GCHandle.Alloc(arr, GCHandleType.Pinned)
    try f (h.AddrOfPinnedObject())
    finally h.Free()

// ---- Grid mesh generator ----

let generateGrid (n: int) : ObjMesh =
    let vertices = Array.init (n * n) (fun i ->
        let x = i % n
        let y = i / n
        let s = 1.0f / float32 (n - 1)
        Vertex(float32 x * s, 0.0f, float32 y * s,
               0.0f, 1.0f, 0.0f,
               float32 x * s, float32 y * s))

    let indices = Array.zeroCreate<uint32> ((n - 1) * (n - 1) * 6)
    let mutable idx = 0
    for y = 0 to n - 2 do
        for x = 0 to n - 2 do
            let tl = uint32 (y * n + x)
            let tr = tl + 1u
            let bl = uint32 ((y + 1) * n + x)
            let br = bl + 1u
            indices.[idx]   <- tl
            indices.[idx+1] <- bl
            indices.[idx+2] <- tr
            indices.[idx+3] <- tr
            indices.[idx+4] <- bl
            indices.[idx+5] <- br
            idx <- idx + 6

    { Vertices = vertices; Indices = indices }

// ---- Shared setup state ----

type BenchState = {
    Mesh: ObjMesh
    IC: int
    VC: int
    VS: int
    CacheOptIndices: uint32[]
    IdxEncBuf: byte[]
    IdxEncSize: int
    VtxEncBuf: byte[]
    VtxEncSize: int
}

let createBenchState (gridSize: int) =
    Native.meshopt_encodeIndexVersion(1)
    MeshOptimizer.Net.IndexCodec.meshopt_encodeIndexVersion(1)
    Native.meshopt_encodeVertexVersion(0)
    MeshOptimizer.Net.VertexCodec.meshopt_encodeVertexVersion(0)

    let mesh = generateGrid gridSize
    let ic = mesh.Indices.Length
    let vc = mesh.Vertices.Length
    let vs = vertexSize

    let cacheOpt = Array.zeroCreate<uint32> ic
    pinArray mesh.Indices (fun src ->
        pinArray cacheOpt (fun d ->
            Native.meshopt_optimizeVertexCache(d, src, unativeint ic, unativeint vc)))

    let idxBound = MeshOptimizer.Net.IndexCodec.meshopt_encodeIndexBufferBound ic vc
    let idxBuf = Array.zeroCreate<byte> idxBound
    let idxSize =
        pinArray mesh.Indices (fun ip ->
            pinArray idxBuf (fun bp ->
                int (Native.meshopt_encodeIndexBuffer(bp, unativeint idxBound, ip, unativeint ic))))

    let vtxBound = MeshOptimizer.Net.VertexCodec.meshopt_encodeVertexBufferBound vc vs
    let vtxBuf = Array.zeroCreate<byte> vtxBound
    let vtxSize =
        pinArray mesh.Vertices (fun vp ->
            pinArray vtxBuf (fun bp ->
                int (Native.meshopt_encodeVertexBuffer(bp, unativeint vtxBound, vp, unativeint vc, unativeint vs))))

    { Mesh = mesh; IC = ic; VC = vc; VS = vs
      CacheOptIndices = cacheOpt
      IdxEncBuf = idxBuf; IdxEncSize = idxSize
      VtxEncBuf = vtxBuf; VtxEncSize = vtxSize }

// ---- Benchmark classes ----

[<MemoryDiagnoser>]
type VertexCacheBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let dst = Array.zeroCreate<uint32> this.S.IC
        pinArray this.S.Mesh.Indices (fun src ->
            pinArray dst (fun d ->
                Native.meshopt_optimizeVertexCache(d, src, unativeint this.S.IC, unativeint this.S.VC)))

    [<Benchmark>]
    member this.FSharp() =
        let dst = Array.zeroCreate<uint32> this.S.IC
        pinArray this.S.Mesh.Indices (fun src ->
            pinArray dst (fun d ->
                MeshOptimizer.Net.VCacheOptimizer.meshopt_optimizeVertexCache
                    (NPtr.ofNI d) (NPtr.ofNI src) this.S.IC this.S.VC))


[<MemoryDiagnoser>]
type OverdrawBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let dst = Array.zeroCreate<uint32> this.S.IC
        pinArray this.S.CacheOptIndices (fun src ->
            pinArray this.S.Mesh.Vertices (fun vp ->
                pinArray dst (fun d ->
                    Native.meshopt_optimizeOverdraw(d, src, unativeint this.S.IC, vp, unativeint this.S.VC, unativeint this.S.VS, 1.05f))))

    [<Benchmark>]
    member this.FSharp() =
        let dst = Array.zeroCreate<uint32> this.S.IC
        pinArray this.S.CacheOptIndices (fun src ->
            pinArray this.S.Mesh.Vertices (fun vp ->
                pinArray dst (fun d ->
                    MeshOptimizer.Net.OverdrawOptimizer.meshopt_optimizeOverdraw
                        (NPtr.ofNI d) (NPtr.ofNI src) this.S.IC (NPtr.ofNI<float32> vp) this.S.VC this.S.VS 1.05f)))


[<MemoryDiagnoser>]
type VertexFetchBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let idxCopy = Array.copy this.S.Mesh.Indices
        let dstV = Array.zeroCreate<Vertex> this.S.VC
        pinArray idxCopy (fun ip ->
            pinArray dstV (fun dp ->
                pinArray this.S.Mesh.Vertices (fun sp ->
                    Native.meshopt_optimizeVertexFetch(dp, ip, unativeint this.S.IC, sp, unativeint this.S.VC, unativeint this.S.VS) |> ignore)))

    [<Benchmark>]
    member this.FSharp() =
        let idxCopy = Array.copy this.S.Mesh.Indices
        let dstV = Array.zeroCreate<Vertex> this.S.VC
        pinArray idxCopy (fun ip ->
            pinArray dstV (fun dp ->
                pinArray this.S.Mesh.Vertices (fun sp ->
                    MeshOptimizer.Net.VFetchOptimizer.meshopt_optimizeVertexFetch
                        dp (NPtr.ofNI ip) this.S.IC sp this.S.VC this.S.VS |> ignore)))


[<MemoryDiagnoser>]
type SimplifyBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let targetCount = this.S.IC / 2
        let dst = Array.zeroCreate<uint32> this.S.IC
        let err = Array.zeroCreate<float32> 1
        pinArray this.S.Mesh.Indices (fun ip ->
            pinArray this.S.Mesh.Vertices (fun vp ->
                pinArray dst (fun dp ->
                    pinArray err (fun ep ->
                        Native.meshopt_simplify(dp, ip, unativeint this.S.IC, vp, unativeint this.S.VC, unativeint this.S.VS, unativeint targetCount, 0.01f, 0u, ep) |> ignore))))

    [<Benchmark>]
    member this.FSharp() =
        let targetCount = this.S.IC / 2
        let dst = Array.zeroCreate<uint32> this.S.IC
        let err = Array.zeroCreate<float32> 1
        pinArray this.S.Mesh.Indices (fun ip ->
            pinArray this.S.Mesh.Vertices (fun vp ->
                pinArray dst (fun dp ->
                    pinArray err (fun ep ->
                        MeshOptimizer.Net.Simplifier.meshopt_simplify
                            (NPtr.ofNI dp) (NPtr.ofNI ip) this.S.IC (NPtr.ofNI<float32> vp) this.S.VC this.S.VS targetCount 0.01f 0u (NPtr.ofNI ep) |> ignore))))


[<MemoryDiagnoser>]
type EncodeIndexBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let bound = MeshOptimizer.Net.IndexCodec.meshopt_encodeIndexBufferBound this.S.IC this.S.VC
        let buf = Array.zeroCreate<byte> bound
        pinArray this.S.Mesh.Indices (fun ip ->
            pinArray buf (fun bp ->
                Native.meshopt_encodeIndexBuffer(bp, unativeint bound, ip, unativeint this.S.IC) |> ignore))

    [<Benchmark>]
    member this.FSharp() =
        let bound = MeshOptimizer.Net.IndexCodec.meshopt_encodeIndexBufferBound this.S.IC this.S.VC
        let buf = Array.zeroCreate<byte> bound
        pinArray this.S.Mesh.Indices (fun ip ->
            pinArray buf (fun bp ->
                MeshOptimizer.Net.IndexCodec.meshopt_encodeIndexBuffer
                    (NPtr.ofNI bp) bound (NPtr.ofNI ip) this.S.IC |> ignore))


[<MemoryDiagnoser>]
type DecodeIndexBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let dst = Array.zeroCreate<uint32> this.S.IC
        pinArray this.S.IdxEncBuf (fun bp ->
            pinArray dst (fun dp ->
                Native.meshopt_decodeIndexBuffer(dp, unativeint this.S.IC, unativeint 4, bp, unativeint this.S.IdxEncSize) |> ignore))

    [<Benchmark>]
    member this.FSharp() =
        let dst = Array.zeroCreate<uint32> this.S.IC
        pinArray this.S.IdxEncBuf (fun bp ->
            pinArray dst (fun dp ->
                MeshOptimizer.Net.IndexCodec.meshopt_decodeIndexBuffer
                    dp this.S.IC 4 (NPtr.ofNI bp) this.S.IdxEncSize |> ignore))


[<MemoryDiagnoser>]
type EncodeVertexBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let bound = MeshOptimizer.Net.VertexCodec.meshopt_encodeVertexBufferBound this.S.VC this.S.VS
        let buf = Array.zeroCreate<byte> bound
        pinArray this.S.Mesh.Vertices (fun vp ->
            pinArray buf (fun bp ->
                Native.meshopt_encodeVertexBuffer(bp, unativeint bound, vp, unativeint this.S.VC, unativeint this.S.VS) |> ignore))

    [<Benchmark>]
    member this.FSharp() =
        let bound = MeshOptimizer.Net.VertexCodec.meshopt_encodeVertexBufferBound this.S.VC this.S.VS
        let buf = Array.zeroCreate<byte> bound
        pinArray this.S.Mesh.Vertices (fun vp ->
            pinArray buf (fun bp ->
                MeshOptimizer.Net.VertexCodec.meshopt_encodeVertexBuffer
                    (NPtr.ofNI bp) bound vp this.S.VC this.S.VS |> ignore))


[<MemoryDiagnoser>]
type DecodeVertexBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let dst = Array.zeroCreate<Vertex> this.S.VC
        pinArray this.S.VtxEncBuf (fun bp ->
            pinArray dst (fun dp ->
                Native.meshopt_decodeVertexBuffer(dp, unativeint this.S.VC, unativeint this.S.VS, bp, unativeint this.S.VtxEncSize) |> ignore))

    [<Benchmark>]
    member this.FSharp() =
        let dst = Array.zeroCreate<Vertex> this.S.VC
        pinArray this.S.VtxEncBuf (fun bp ->
            pinArray dst (fun dp ->
                MeshOptimizer.Net.VertexCodec.meshopt_decodeVertexBuffer
                    dp this.S.VC this.S.VS (NPtr.ofNI bp) this.S.VtxEncSize |> ignore))


[<MemoryDiagnoser>]
type BuildMeshletsBenchmark() =

    static let maxVerts = 64
    static let maxTris = 124

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set
    member val MaxMeshlets = 0 with get, set

    [<GlobalSetup>]
    member this.Setup() =
        this.S <- createBenchState this.GridSize
        this.MaxMeshlets <- MeshOptimizer.Net.Clusterizer.meshopt_buildMeshletsBound this.S.IC maxVerts maxTris

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let ms = Array.zeroCreate<meshopt_Meshlet> this.MaxMeshlets
        let mv = Array.zeroCreate<uint32> (this.MaxMeshlets * maxVerts)
        let mt = Array.zeroCreate<byte> (this.MaxMeshlets * maxTris * 3)
        pinArray this.S.Mesh.Indices (fun ip ->
            pinArray this.S.Mesh.Vertices (fun vp ->
                pinArray ms (fun mp ->
                    pinArray mv (fun mvp ->
                        pinArray mt (fun mtp ->
                            Native.meshopt_buildMeshlets(mp, mvp, mtp, ip, unativeint this.S.IC, vp, unativeint this.S.VC, unativeint this.S.VS, unativeint maxVerts, unativeint maxTris, 0.0f) |> ignore)))))

    [<Benchmark>]
    member this.FSharp() =
        let ms = Array.zeroCreate<meshopt_Meshlet> this.MaxMeshlets
        let mv = Array.zeroCreate<uint32> (this.MaxMeshlets * maxVerts)
        let mt = Array.zeroCreate<byte> (this.MaxMeshlets * maxTris * 3)
        pinArray this.S.Mesh.Indices (fun ip ->
            pinArray this.S.Mesh.Vertices (fun vp ->
                pinArray ms (fun mp ->
                    pinArray mv (fun mvp ->
                        pinArray mt (fun mtp ->
                            MeshOptimizer.Net.Clusterizer.meshopt_buildMeshlets
                                (NPtr.ofNI mp) (NPtr.ofNI mvp) (NPtr.ofNI mtp)
                                (NPtr.ofNI ip) this.S.IC (NPtr.ofNI<float32> vp) this.S.VC this.S.VS maxVerts maxTris 0.0f |> ignore)))))


[<MemoryDiagnoser>]
type StripifyBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set
    member val StripBound = 0 with get, set

    [<GlobalSetup>]
    member this.Setup() =
        this.S <- createBenchState this.GridSize
        this.StripBound <- MeshOptimizer.Net.Stripifier.meshopt_stripifyBound this.S.IC

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let dst = Array.zeroCreate<uint32> this.StripBound
        pinArray this.S.Mesh.Indices (fun ip ->
            pinArray dst (fun dp ->
                Native.meshopt_stripify(dp, ip, unativeint this.S.IC, unativeint this.S.VC, 0xFFFFFFFFu) |> ignore))

    [<Benchmark>]
    member this.FSharp() =
        let dst = Array.zeroCreate<uint32> this.StripBound
        pinArray this.S.Mesh.Indices (fun ip ->
            pinArray dst (fun dp ->
                MeshOptimizer.Net.Stripifier.meshopt_stripify
                    (NPtr.ofNI dp) (NPtr.ofNI ip) this.S.IC this.S.VC 0xFFFFFFFFu |> ignore))


[<MemoryDiagnoser>]
type SpatialSortBenchmark() =

    [<Params(32, 224, 1024)>]
    member val GridSize = 0 with get, set
    member val S = Unchecked.defaultof<BenchState> with get, set

    [<GlobalSetup>]
    member this.Setup() = this.S <- createBenchState this.GridSize

    [<Benchmark(Baseline = true)>]
    member this.Cpp() =
        let dst = Array.zeroCreate<uint32> this.S.VC
        pinArray this.S.Mesh.Vertices (fun vp ->
            pinArray dst (fun dp ->
                Native.meshopt_spatialSortRemap(dp, vp, unativeint this.S.VC, unativeint this.S.VS)))

    [<Benchmark>]
    member this.FSharp() =
        let dst = Array.zeroCreate<uint32> this.S.VC
        pinArray this.S.Mesh.Vertices (fun vp ->
            pinArray dst (fun dp ->
                MeshOptimizer.Net.SpatialOrder.meshopt_spatialSortRemap
                    (NPtr.ofNI dp) (NPtr.ofNI<float32> vp) this.S.VC this.S.VS))


// ---- Hardware counter config ----

type PerfConfig() =
    inherit ManualConfig()
    do
        base.AddJob(Job.Default) |> ignore
        base.AddDiagnoser(MemoryDiagnoser.Default) |> ignore
        base.AddColumn(StatisticColumn.Median) |> ignore
        base.AddHardwareCounters(
            HardwareCounter.BranchMispredictions,
            HardwareCounter.CacheMisses,
            HardwareCounter.InstructionRetired) |> ignore
