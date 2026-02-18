// Performance benchmark: F# port vs C++ reference (via P/Invoke)
// Run with: LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project MeshOptPort.Tests -- --bench
module MeshOptPort.Tests.Benchmark

open System
open System.Diagnostics
open System.Runtime.InteropServices
open MeshOptPort
open MeshOptPort.Tests.ObjLoader

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

// ---- Timing ----

let private median (values: float[]) =
    let sorted = Array.sort (Array.copy values)
    sorted.[sorted.Length / 2]

let private timeRuns (runs: int) (action: unit -> unit) : float =
    action() // warmup
    GC.Collect(2, GCCollectionMode.Forced, true)
    GC.WaitForPendingFinalizers()

    let times = Array.init runs (fun _ ->
        let sw = Stopwatch.StartNew()
        action()
        sw.Elapsed.TotalMilliseconds)

    median times

// ---- Benchmark runner ----

let runBenchmarks () =
    printfn "=== Performance Benchmark: F# Port vs C++ Reference ===\n"

    Native.meshopt_encodeIndexVersion(1)
    MeshOptPort.IndexCodec.meshopt_encodeIndexVersion(1)
    Native.meshopt_encodeVertexVersion(0)
    MeshOptPort.VertexCodec.meshopt_encodeVertexVersion(0)

    let sizes = [| "Small (1K tri)", 32, 50
                   "Medium (100K tri)", 224, 10
                   "Large (2M tri)", 1024, 3 |]

    for (label, gridSize, runs) in sizes do
        let mesh = generateGrid gridSize
        let vc = mesh.Vertices.Length
        let ic = mesh.Indices.Length
        let vs = vertexSize

        printfn "=== %s: %d verts, %d tris, %d indices ===" label vc (ic / 3) ic
        printfn "%-30s | %10s | %10s | %s" "Function" "C++ (ms)" "F# (ms)" "Ratio"
        printfn "%s+%s+%s+%s" (String('-', 31)) (String('-', 12)) (String('-', 12)) (String('-', 7))

        let bench name (cppAction: unit -> unit) (fsAction: unit -> unit) =
            let cppMs = timeRuns runs cppAction
            let fsMs = timeRuns runs fsAction
            let ratio = if cppMs > 0.001 then fsMs / cppMs else 0.0
            printfn "%-30s | %10.2f | %10.2f | %5.1fx" name cppMs fsMs ratio
            GC.Collect(2, GCCollectionMode.Forced, true)
            GC.WaitForPendingFinalizers()

        // 1. optimizeVertexCache
        let cacheOptResult = Array.zeroCreate<uint32> ic

        bench "optimizeVertexCache"
            (fun () ->
                let dst = Array.zeroCreate<uint32> ic
                pinArray mesh.Indices (fun src ->
                    pinArray dst (fun d ->
                        Native.meshopt_optimizeVertexCache(d, src, unativeint ic, unativeint vc))))
            (fun () ->
                let dst = Array.zeroCreate<uint32> ic
                pinArray mesh.Indices (fun src ->
                    pinArray dst (fun d ->
                        MeshOptPort.VCacheOptimizer.meshopt_optimizeVertexCache
                            (NPtr.ofNI d) (NPtr.ofNI src) ic vc)))

        // Produce cache-optimized indices for overdraw benchmark
        pinArray mesh.Indices (fun src ->
            pinArray cacheOptResult (fun d ->
                Native.meshopt_optimizeVertexCache(d, src, unativeint ic, unativeint vc)))

        // 2. optimizeOverdraw
        bench "optimizeOverdraw"
            (fun () ->
                let dst = Array.zeroCreate<uint32> ic
                pinArray cacheOptResult (fun src ->
                    pinArray mesh.Vertices (fun vp ->
                        pinArray dst (fun d ->
                            Native.meshopt_optimizeOverdraw(d, src, unativeint ic, vp, unativeint vc, unativeint vs, 1.05f)))))
            (fun () ->
                let dst = Array.zeroCreate<uint32> ic
                pinArray cacheOptResult (fun src ->
                    pinArray mesh.Vertices (fun vp ->
                        pinArray dst (fun d ->
                            MeshOptPort.OverdrawOptimizer.meshopt_optimizeOverdraw
                                (NPtr.ofNI d) (NPtr.ofNI src) ic (NPtr.ofNI<float32> vp) vc vs 1.05f))))

        // 3. optimizeVertexFetch
        bench "optimizeVertexFetch"
            (fun () ->
                let idxCopy = Array.copy mesh.Indices
                let dstV = Array.zeroCreate<Vertex> vc
                pinArray idxCopy (fun ip ->
                    pinArray dstV (fun dp ->
                        pinArray mesh.Vertices (fun sp ->
                            Native.meshopt_optimizeVertexFetch(dp, ip, unativeint ic, sp, unativeint vc, unativeint vs) |> ignore))))
            (fun () ->
                let idxCopy = Array.copy mesh.Indices
                let dstV = Array.zeroCreate<Vertex> vc
                pinArray idxCopy (fun ip ->
                    pinArray dstV (fun dp ->
                        pinArray mesh.Vertices (fun sp ->
                            MeshOptPort.VFetchOptimizer.meshopt_optimizeVertexFetch
                                dp (NPtr.ofNI ip) ic sp vc vs |> ignore))))

        // 4. simplify (50% target)
        let targetCount = ic / 2
        bench "simplify"
            (fun () ->
                let dst = Array.zeroCreate<uint32> ic
                let err = Array.zeroCreate<float32> 1
                pinArray mesh.Indices (fun ip ->
                    pinArray mesh.Vertices (fun vp ->
                        pinArray dst (fun dp ->
                            pinArray err (fun ep ->
                                Native.meshopt_simplify(dp, ip, unativeint ic, vp, unativeint vc, unativeint vs, unativeint targetCount, 0.01f, 0u, ep) |> ignore)))))
            (fun () ->
                let dst = Array.zeroCreate<uint32> ic
                let err = Array.zeroCreate<float32> 1
                pinArray mesh.Indices (fun ip ->
                    pinArray mesh.Vertices (fun vp ->
                        pinArray dst (fun dp ->
                            pinArray err (fun ep ->
                                MeshOptPort.Simplifier.meshopt_simplify
                                    (NPtr.ofNI dp) (NPtr.ofNI ip) ic (NPtr.ofNI<float32> vp) vc vs targetCount 0.01f 0u (NPtr.ofNI ep) |> ignore)))))

        // 5. encodeIndexBuffer
        let idxBound = MeshOptPort.IndexCodec.meshopt_encodeIndexBufferBound ic vc
        bench "encodeIndexBuffer"
            (fun () ->
                let buf = Array.zeroCreate<byte> idxBound
                pinArray mesh.Indices (fun ip ->
                    pinArray buf (fun bp ->
                        Native.meshopt_encodeIndexBuffer(bp, unativeint idxBound, ip, unativeint ic) |> ignore)))
            (fun () ->
                let buf = Array.zeroCreate<byte> idxBound
                pinArray mesh.Indices (fun ip ->
                    pinArray buf (fun bp ->
                        MeshOptPort.IndexCodec.meshopt_encodeIndexBuffer
                            (NPtr.ofNI bp) idxBound (NPtr.ofNI ip) ic |> ignore)))

        // 6. decodeIndexBuffer (encode first, then time decode only)
        let idxEncBuf = Array.zeroCreate<byte> idxBound
        let idxEncSize =
            pinArray mesh.Indices (fun ip ->
                pinArray idxEncBuf (fun bp ->
                    int (Native.meshopt_encodeIndexBuffer(bp, unativeint idxBound, ip, unativeint ic))))

        bench "decodeIndexBuffer"
            (fun () ->
                let dst = Array.zeroCreate<uint32> ic
                pinArray idxEncBuf (fun bp ->
                    pinArray dst (fun dp ->
                        Native.meshopt_decodeIndexBuffer(dp, unativeint ic, unativeint 4, bp, unativeint idxEncSize) |> ignore)))
            (fun () ->
                let dst = Array.zeroCreate<uint32> ic
                pinArray idxEncBuf (fun bp ->
                    pinArray dst (fun dp ->
                        MeshOptPort.IndexCodec.meshopt_decodeIndexBuffer
                            dp ic 4 (NPtr.ofNI bp) idxEncSize |> ignore)))

        // 7. encodeVertexBuffer
        let vtxBound = MeshOptPort.VertexCodec.meshopt_encodeVertexBufferBound vc vs
        bench "encodeVertexBuffer"
            (fun () ->
                let buf = Array.zeroCreate<byte> vtxBound
                pinArray mesh.Vertices (fun vp ->
                    pinArray buf (fun bp ->
                        Native.meshopt_encodeVertexBuffer(bp, unativeint vtxBound, vp, unativeint vc, unativeint vs) |> ignore)))
            (fun () ->
                let buf = Array.zeroCreate<byte> vtxBound
                pinArray mesh.Vertices (fun vp ->
                    pinArray buf (fun bp ->
                        MeshOptPort.VertexCodec.meshopt_encodeVertexBuffer
                            (NPtr.ofNI bp) vtxBound vp vc vs |> ignore)))

        // 8. decodeVertexBuffer (encode first, then time decode only)
        let vtxEncBuf = Array.zeroCreate<byte> vtxBound
        let vtxEncSize =
            pinArray mesh.Vertices (fun vp ->
                pinArray vtxEncBuf (fun bp ->
                    int (Native.meshopt_encodeVertexBuffer(bp, unativeint vtxBound, vp, unativeint vc, unativeint vs))))

        bench "decodeVertexBuffer"
            (fun () ->
                let dst = Array.zeroCreate<Vertex> vc
                pinArray vtxEncBuf (fun bp ->
                    pinArray dst (fun dp ->
                        Native.meshopt_decodeVertexBuffer(dp, unativeint vc, unativeint vs, bp, unativeint vtxEncSize) |> ignore)))
            (fun () ->
                let dst = Array.zeroCreate<Vertex> vc
                pinArray vtxEncBuf (fun bp ->
                    pinArray dst (fun dp ->
                        MeshOptPort.VertexCodec.meshopt_decodeVertexBuffer
                            dp vc vs (NPtr.ofNI bp) vtxEncSize |> ignore)))

        // 9. buildMeshlets
        let maxVerts = 64
        let maxTris = 124
        let maxMeshlets = MeshOptPort.Clusterizer.meshopt_buildMeshletsBound ic maxVerts maxTris

        bench "buildMeshlets"
            (fun () ->
                let ms = Array.zeroCreate<meshopt_Meshlet> maxMeshlets
                let mv = Array.zeroCreate<uint32> (maxMeshlets * maxVerts)
                let mt = Array.zeroCreate<byte> (maxMeshlets * maxTris * 3)
                pinArray mesh.Indices (fun ip ->
                    pinArray mesh.Vertices (fun vp ->
                        pinArray ms (fun mp ->
                            pinArray mv (fun mvp ->
                                pinArray mt (fun mtp ->
                                    Native.meshopt_buildMeshlets(mp, mvp, mtp, ip, unativeint ic, vp, unativeint vc, unativeint vs, unativeint maxVerts, unativeint maxTris, 0.0f) |> ignore))))))
            (fun () ->
                let ms = Array.zeroCreate<meshopt_Meshlet> maxMeshlets
                let mv = Array.zeroCreate<uint32> (maxMeshlets * maxVerts)
                let mt = Array.zeroCreate<byte> (maxMeshlets * maxTris * 3)
                pinArray mesh.Indices (fun ip ->
                    pinArray mesh.Vertices (fun vp ->
                        pinArray ms (fun mp ->
                            pinArray mv (fun mvp ->
                                pinArray mt (fun mtp ->
                                    MeshOptPort.Clusterizer.meshopt_buildMeshlets
                                        (NPtr.ofNI mp) (NPtr.ofNI mvp) (NPtr.ofNI mtp)
                                        (NPtr.ofNI ip) ic (NPtr.ofNI<float32> vp) vc vs maxVerts maxTris 0.0f |> ignore))))))

        // 10. stripify
        let stripBound = MeshOptPort.Stripifier.meshopt_stripifyBound ic

        bench "stripify"
            (fun () ->
                let dst = Array.zeroCreate<uint32> stripBound
                pinArray mesh.Indices (fun ip ->
                    pinArray dst (fun dp ->
                        Native.meshopt_stripify(dp, ip, unativeint ic, unativeint vc, 0xFFFFFFFFu) |> ignore)))
            (fun () ->
                let dst = Array.zeroCreate<uint32> stripBound
                pinArray mesh.Indices (fun ip ->
                    pinArray dst (fun dp ->
                        MeshOptPort.Stripifier.meshopt_stripify
                            (NPtr.ofNI dp) (NPtr.ofNI ip) ic vc 0xFFFFFFFFu |> ignore)))

        // 11. spatialSortRemap
        bench "spatialSortRemap"
            (fun () ->
                let dst = Array.zeroCreate<uint32> vc
                pinArray mesh.Vertices (fun vp ->
                    pinArray dst (fun dp ->
                        Native.meshopt_spatialSortRemap(dp, vp, unativeint vc, unativeint vs))))
            (fun () ->
                let dst = Array.zeroCreate<uint32> vc
                pinArray mesh.Vertices (fun vp ->
                    pinArray dst (fun dp ->
                        MeshOptPort.SpatialOrder.meshopt_spatialSortRemap
                            (NPtr.ofNI dp) (NPtr.ofNI<float32> vp) vc vs)))

        printfn ""
