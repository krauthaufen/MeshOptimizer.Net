// Test harness: compares C++ meshoptimizer (via P/Invoke) against F# port
// Run with: LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run --project MeshOptPort.Tests
module MeshOptPort.Tests.Program

open System
open System.Reflection
open System.Runtime.InteropServices
open System.Runtime.CompilerServices
open BenchmarkDotNet.Running
open MeshOptPort
open MeshOptPort.Tests.ObjLoader

#nowarn "9" // native interop

module NPtr =
    open FSharp.NativeInterop
    let inline get (p: nativeptr<'T>) (i: int) = NativePtr.get p i
    let inline set (p: nativeptr<'T>) (i: int) (v: 'T) = NativePtr.set p i v
    let inline add (p: nativeptr<'T>) (i: int) = NativePtr.add p i
    let inline toNI (p: nativeptr<'T>) = NativePtr.toNativeInt p
    let inline ofNI<'T when 'T : unmanaged> (n: nativeint) : nativeptr<'T> = NativePtr.ofNativeInt n
    let inline toVoid (p: nativeptr<'T>) = NativePtr.toVoidPtr p
    let inline ofVoid<'T when 'T : unmanaged> (p: voidptr) : nativeptr<'T> = NativePtr.ofVoidPtr p

// ---- Test infrastructure ----

let mutable passCount = 0
let mutable failCount = 0

let pass (name: string) =
    passCount <- passCount + 1
    printfn "  PASS  %s" name

let fail (name: string) (detail: string) =
    failCount <- failCount + 1
    printfn "  FAIL  %s: %s" name detail

let compareArraysExact (name: string) (a: 'T[]) (b: 'T[]) =
    if a.Length <> b.Length then
        fail name (sprintf "length mismatch: %d vs %d" a.Length b.Length)
    else
        let mutable diff = -1
        for i = 0 to a.Length - 1 do
            if not (obj.Equals(a.[i], b.[i])) && diff = -1 then
                diff <- i
        if diff >= 0 then
            fail name (sprintf "first diff at [%d]: %A vs %A" diff a.[diff] b.[diff])
        else
            pass name

let compareFloat (name: string) (a: float32) (b: float32) (eps: float32) =
    if abs (a - b) <= eps then pass name
    else fail name (sprintf "%.8f vs %.8f (diff %.8f)" a b (abs (a - b)))

// ---- Pin helper ----

/// Pin a managed array and get its nativeint, call f with it
let inline pinArray (arr: 'T[]) (f: nativeint -> 'R) : 'R =
    let h = GCHandle.Alloc(arr, GCHandleType.Pinned)
    try f (h.AddrOfPinnedObject())
    finally h.Free()

let inline pinArray2 (a1: 'T1[]) (a2: 'T2[]) (f: nativeint -> nativeint -> 'R) : 'R =
    let h1 = GCHandle.Alloc(a1, GCHandleType.Pinned)
    let h2 = GCHandle.Alloc(a2, GCHandleType.Pinned)
    try f (h1.AddrOfPinnedObject()) (h2.AddrOfPinnedObject())
    finally h2.Free(); h1.Free()

// ---- Tests ----

let testQuantization () =
    printfn "\n=== Quantization ==="

    // Test quantizeHalf / dequantizeHalf
    let testValues = [| 0.0f; 1.0f; -1.0f; 0.5f; 65504.0f; 6e-5f; Single.PositiveInfinity; Single.NaN |]
    let mutable allPass = true
    for v in testValues do
        let cppH = Native.meshopt_quantizeHalf(v)
        let fsH = MeshOptPort.Quantization.meshopt_quantizeHalf v
        if cppH <> fsH then
            fail "quantizeHalf" (sprintf "v=%.6g: C++=%d F#=%d" v cppH fsH)
            allPass <- false
    if allPass then pass "quantizeHalf (8 values)"

    let mutable allPass2 = true
    for h in [| 0us; 0x3C00us; 0xBC00us; 0x3800us; 0x7BFFus; 0x0400us; 0x7C00us; 0x7E00us |] do
        let cppF = Native.meshopt_dequantizeHalf(h)
        let fsF = MeshOptPort.Quantization.meshopt_dequantizeHalf h
        let cppBits = Unsafe.BitCast<float32, uint32>(cppF)
        let fsBits = Unsafe.BitCast<float32, uint32>(fsF)
        if cppBits <> fsBits then
            fail "dequantizeHalf" (sprintf "h=%d: C++=0x%08X F#=0x%08X" h cppBits fsBits)
            allPass2 <- false
    if allPass2 then pass "dequantizeHalf (8 values)"

let testVertexRemap (mesh: ObjMesh) (verticesPtr: nativeint) (indicesPtr: nativeint) =
    printfn "\n=== Vertex Remap (generateVertexRemap) ==="

    let vc = mesh.Vertices.Length
    let ic = mesh.Indices.Length
    let vs = vertexSize

    let cppRemap = Array.zeroCreate<uint32> vc
    let fsRemap = Array.zeroCreate<uint32> vc

    let cppUnique =
        pinArray cppRemap (fun remapPtr ->
            int (Native.meshopt_generateVertexRemap(remapPtr, indicesPtr, unativeint ic, verticesPtr, unativeint vc, unativeint vs)))

    let fsUnique =
        pinArray fsRemap (fun remapPtr ->
            MeshOptPort.IndexGenerator.meshopt_generateVertexRemap
                (NPtr.ofNI remapPtr) (NPtr.ofNI indicesPtr) ic (verticesPtr) vc vs)

    if cppUnique <> fsUnique then
        fail "generateVertexRemap count" (sprintf "C++=%d F#=%d" cppUnique fsUnique)
    else
        pass "generateVertexRemap count"

    compareArraysExact "generateVertexRemap remap" cppRemap fsRemap

    cppRemap // return for further use

let testOptimizeVertexCache (indices: uint32[]) (vertexCount: int) =
    printfn "\n=== Vertex Cache Optimization ==="

    let ic = indices.Length
    let cppDst = Array.zeroCreate<uint32> ic
    let fsDst = Array.zeroCreate<uint32> ic

    pinArray indices (fun srcPtr ->
        pinArray cppDst (fun cppPtr ->
            Native.meshopt_optimizeVertexCache(cppPtr, srcPtr, unativeint ic, unativeint vertexCount))
        pinArray fsDst (fun fsPtr ->
            MeshOptPort.VCacheOptimizer.meshopt_optimizeVertexCache
                (NPtr.ofNI fsPtr) (NPtr.ofNI srcPtr) ic vertexCount))

    compareArraysExact "optimizeVertexCache" cppDst fsDst
    cppDst // return optimized indices for further tests

let testOptimizeOverdraw (cacheOptIndices: uint32[]) (mesh: ObjMesh) =
    printfn "\n=== Overdraw Optimization ==="

    let ic = cacheOptIndices.Length
    let vc = mesh.Vertices.Length
    let stride = vertexSize // positions at offset 0, stride = 32 bytes
    let cppDst = Array.zeroCreate<uint32> ic
    let fsDst = Array.zeroCreate<uint32> ic

    pinArray cacheOptIndices (fun srcPtr ->
        pinArray (mesh.Vertices) (fun vPtr ->
            pinArray cppDst (fun cppPtr ->
                Native.meshopt_optimizeOverdraw(cppPtr, srcPtr, unativeint ic, vPtr, unativeint vc, unativeint stride, 1.05f))
            pinArray fsDst (fun fsPtr ->
                MeshOptPort.OverdrawOptimizer.meshopt_optimizeOverdraw
                    (NPtr.ofNI fsPtr) (NPtr.ofNI srcPtr) ic (NPtr.ofNI<float32> vPtr) vc stride 1.05f)))

    compareArraysExact "optimizeOverdraw" cppDst fsDst
    cppDst

let testOptimizeVertexFetch (indices: uint32[]) (mesh: ObjMesh) =
    printfn "\n=== Vertex Fetch Optimization ==="

    let ic = indices.Length
    let vc = mesh.Vertices.Length
    let vs = vertexSize

    // Need mutable copies since optimizeVertexFetch modifies indices in-place
    let cppIndices = Array.copy indices
    let fsIndices = Array.copy indices
    let cppVerts = Array.zeroCreate<Vertex> vc
    let fsVerts = Array.zeroCreate<Vertex> vc

    let cppUnique =
        pinArray cppIndices (fun idxPtr ->
            pinArray cppVerts (fun dstPtr ->
                pinArray (mesh.Vertices) (fun srcPtr ->
                    int (Native.meshopt_optimizeVertexFetch(dstPtr, idxPtr, unativeint ic, srcPtr, unativeint vc, unativeint vs)))))

    let fsUnique =
        pinArray fsIndices (fun idxPtr ->
            pinArray fsVerts (fun dstPtr ->
                pinArray (mesh.Vertices) (fun srcPtr ->
                    MeshOptPort.VFetchOptimizer.meshopt_optimizeVertexFetch
                        dstPtr (NPtr.ofNI idxPtr) ic srcPtr vc vs)))

    if cppUnique <> fsUnique then
        fail "optimizeVertexFetch count" (sprintf "C++=%d F#=%d" cppUnique fsUnique)
    else
        pass "optimizeVertexFetch count"

    compareArraysExact "optimizeVertexFetch indices" cppIndices fsIndices
    compareArraysExact "optimizeVertexFetch vertices" cppVerts fsVerts

let testAnalysis (indices: uint32[]) (mesh: ObjMesh) =
    printfn "\n=== Analysis ==="

    let ic = indices.Length
    let vc = mesh.Vertices.Length
    let vs = vertexSize

    pinArray indices (fun idxPtr ->
        // Vertex cache stats
        let cppStats = Native.meshopt_analyzeVertexCache(idxPtr, unativeint ic, unativeint vc, 16u, 0u, 0u)
        let fsStats =
            MeshOptPort.IndexAnalyzer.meshopt_analyzeVertexCache
                (NPtr.ofNI idxPtr) ic vc 16u 0u 0u
        compareFloat "analyzeVertexCache ACMR" cppStats.acmr fsStats.acmr 1e-6f
        compareFloat "analyzeVertexCache ATVR" cppStats.atvr fsStats.atvr 1e-6f

        // Vertex fetch stats
        let cppFetch = Native.meshopt_analyzeVertexFetch(idxPtr, unativeint ic, unativeint vc, unativeint vs)
        let fsFetch =
            MeshOptPort.IndexAnalyzer.meshopt_analyzeVertexFetch
                (NPtr.ofNI idxPtr) ic vc vs
        compareFloat "analyzeVertexFetch overfetch" cppFetch.overfetch fsFetch.overfetch 1e-6f

        // Overdraw analysis
        pinArray (mesh.Vertices) (fun vPtr ->
            let cppOd = Native.meshopt_analyzeOverdraw(idxPtr, unativeint ic, vPtr, unativeint vc, unativeint vertexSize)
            let fsOd =
                MeshOptPort.Rasterizer.meshopt_analyzeOverdraw
                    (NPtr.ofNI idxPtr) ic (NPtr.ofNI<float32> vPtr) vc vertexSize
            compareFloat "analyzeOverdraw overdraw" cppOd.overdraw fsOd.overdraw 1e-6f))

let testIndexCodec (indices: uint32[]) (vertexCount: int) =
    printfn "\n=== Index Buffer Codec ==="

    let ic = indices.Length

    // Ensure both use version 1
    Native.meshopt_encodeIndexVersion(1)
    MeshOptPort.IndexCodec.meshopt_encodeIndexVersion(1)

    let boundSize = int (Native.meshopt_encodeIndexBufferBound(unativeint ic, unativeint vertexCount))
    let fsBoundSize = MeshOptPort.IndexCodec.meshopt_encodeIndexBufferBound ic vertexCount

    if boundSize <> fsBoundSize then
        fail "encodeIndexBufferBound" (sprintf "C++=%d F#=%d" boundSize fsBoundSize)
    else
        pass "encodeIndexBufferBound"

    let cppBuf = Array.zeroCreate<byte> boundSize
    let fsBuf = Array.zeroCreate<byte> boundSize

    let cppEncSize =
        pinArray indices (fun idxPtr ->
            pinArray cppBuf (fun bufPtr ->
                int (Native.meshopt_encodeIndexBuffer(bufPtr, unativeint boundSize, idxPtr, unativeint ic))))

    let fsEncSize =
        pinArray indices (fun idxPtr ->
            pinArray fsBuf (fun bufPtr ->
                MeshOptPort.IndexCodec.meshopt_encodeIndexBuffer
                    (NPtr.ofNI bufPtr) boundSize (NPtr.ofNI idxPtr) ic))

    if cppEncSize <> fsEncSize then
        fail "encodeIndexBuffer size" (sprintf "C++=%d F#=%d" cppEncSize fsEncSize)
    else
        pass "encodeIndexBuffer size"

    // Compare encoded bytes
    let cppEncBytes = cppBuf.[..cppEncSize-1]
    let fsEncBytes = fsBuf.[..fsEncSize-1]
    compareArraysExact "encodeIndexBuffer data" cppEncBytes fsEncBytes

    // Decode with C++ (reference)
    let cppDecoded = Array.zeroCreate<uint32> ic
    pinArray cppBuf (fun bufPtr ->
        pinArray cppDecoded (fun dstPtr ->
            let rc = Native.meshopt_decodeIndexBuffer(dstPtr, unativeint ic, unativeint 4, bufPtr, unativeint cppEncSize)
            if rc <> 0 then fail "decodeIndexBuffer(C++ ref)" (sprintf "error code %d" rc)))

    // Decode with F#
    let fsDecoded = Array.zeroCreate<uint32> ic
    pinArray cppBuf (fun bufPtr ->
        pinArray fsDecoded (fun dstPtr ->
            let rc = MeshOptPort.IndexCodec.meshopt_decodeIndexBuffer
                        dstPtr ic 4 (NPtr.ofNI bufPtr) cppEncSize
            if rc <> 0 then fail "decodeIndexBuffer(F#)" (sprintf "error code %d" rc)))

    // Compare F# decode vs C++ decode (encoder may rotate triangles)
    compareArraysExact "decodeIndexBuffer F# vs C++" cppDecoded fsDecoded

let testVertexCodec (mesh: ObjMesh) =
    printfn "\n=== Vertex Buffer Codec ==="

    let vc = mesh.Vertices.Length
    let vs = vertexSize

    // Ensure both use version 0 for deterministic comparison
    Native.meshopt_encodeVertexVersion(0)
    MeshOptPort.VertexCodec.meshopt_encodeVertexVersion(0)

    let boundSize = int (Native.meshopt_encodeVertexBufferBound(unativeint vc, unativeint vs))
    let fsBoundSize = MeshOptPort.VertexCodec.meshopt_encodeVertexBufferBound vc vs

    if boundSize <> fsBoundSize then
        fail "encodeVertexBufferBound" (sprintf "C++=%d F#=%d" boundSize fsBoundSize)
    else
        pass "encodeVertexBufferBound"

    let cppBuf = Array.zeroCreate<byte> boundSize
    let fsBuf = Array.zeroCreate<byte> boundSize

    let cppEncSize =
        pinArray (mesh.Vertices) (fun vPtr ->
            pinArray cppBuf (fun bufPtr ->
                int (Native.meshopt_encodeVertexBuffer(bufPtr, unativeint boundSize, vPtr, unativeint vc, unativeint vs))))

    let fsEncSize =
        pinArray (mesh.Vertices) (fun vPtr ->
            pinArray fsBuf (fun bufPtr ->
                MeshOptPort.VertexCodec.meshopt_encodeVertexBuffer
                    (NPtr.ofNI bufPtr) boundSize vPtr vc vs))

    if cppEncSize <> fsEncSize then
        fail "encodeVertexBuffer size" (sprintf "C++=%d F#=%d" cppEncSize fsEncSize)
    else
        pass "encodeVertexBuffer size"

    let cppEncBytes = cppBuf.[..cppEncSize-1]
    let fsEncBytes = fsBuf.[..fsEncSize-1]
    compareArraysExact "encodeVertexBuffer data" cppEncBytes fsEncBytes

    // Cross-decode: C++ encoded → F# decode
    let fsDecoded = Array.zeroCreate<Vertex> vc
    pinArray cppBuf (fun bufPtr ->
        pinArray fsDecoded (fun dstPtr ->
            let rc = MeshOptPort.VertexCodec.meshopt_decodeVertexBuffer
                        dstPtr vc vs (NPtr.ofNI bufPtr) cppEncSize
            if rc <> 0 then fail "decodeVertexBuffer(C++→F#)" (sprintf "error code %d" rc)))
    compareArraysExact "decodeVertexBuffer C++→F#" mesh.Vertices fsDecoded

    // Cross-decode: F# encoded → C++ decode
    let cppDecoded = Array.zeroCreate<Vertex> vc
    pinArray fsBuf (fun bufPtr ->
        pinArray cppDecoded (fun dstPtr ->
            let rc = Native.meshopt_decodeVertexBuffer(dstPtr, unativeint vc, unativeint vs, bufPtr, unativeint fsEncSize)
            if rc <> 0 then fail "decodeVertexBuffer(F#→C++)" (sprintf "error code %d" rc)))
    compareArraysExact "decodeVertexBuffer F#→C++" mesh.Vertices cppDecoded

let testSimplify (indices: uint32[]) (mesh: ObjMesh) =
    printfn "\n=== Simplification ==="

    let ic = indices.Length
    let vc = mesh.Vertices.Length
    let stride = vertexSize
    let targetCount = ic / 2

    let cppDst = Array.zeroCreate<uint32> ic
    let fsDst = Array.zeroCreate<uint32> ic
    let cppErr = Array.zeroCreate<float32> 1
    let fsErr = Array.zeroCreate<float32> 1

    let cppResult =
        pinArray indices (fun idxPtr ->
            pinArray (mesh.Vertices) (fun vPtr ->
                pinArray cppDst (fun dstPtr ->
                    pinArray cppErr (fun errPtr ->
                        int (Native.meshopt_simplify(dstPtr, idxPtr, unativeint ic, vPtr, unativeint vc, unativeint stride, unativeint targetCount, 0.01f, 0u, errPtr))))))

    let fsResult =
        pinArray indices (fun idxPtr ->
            pinArray (mesh.Vertices) (fun vPtr ->
                pinArray fsDst (fun dstPtr ->
                    pinArray fsErr (fun errPtr ->
                        MeshOptPort.Simplifier.meshopt_simplify
                            (NPtr.ofNI dstPtr) (NPtr.ofNI idxPtr) ic (NPtr.ofNI<float32> vPtr) vc stride targetCount 0.01f 0u (NPtr.ofNI errPtr)))))

    if cppResult <> fsResult then
        fail "simplify result count" (sprintf "C++=%d F#=%d" cppResult fsResult)
    else
        pass (sprintf "simplify result count (%d)" cppResult)

    // Compare indices (only the valid portion)
    if cppResult > 0 && fsResult > 0 then
        let cmpLen = min cppResult fsResult
        let cppSlice = cppDst.[..cmpLen-1]
        let fsSlice = fsDst.[..cmpLen-1]
        compareArraysExact "simplify indices" cppSlice fsSlice

    compareFloat "simplify error" cppErr.[0] fsErr.[0] 1e-6f

    // simplifyScale
    pinArray (mesh.Vertices) (fun vPtr ->
        let cppScale = Native.meshopt_simplifyScale(vPtr, unativeint vc, unativeint stride)
        let fsScale = MeshOptPort.Simplifier.meshopt_simplifyScale (NPtr.ofNI<float32> vPtr) vc stride
        compareFloat "simplifyScale" cppScale fsScale 1e-6f)

    // simplifySloppy
    let cppDst2 = Array.zeroCreate<uint32> ic
    let fsDst2 = Array.zeroCreate<uint32> ic
    let cppErr2 = Array.zeroCreate<float32> 1
    let fsErr2 = Array.zeroCreate<float32> 1

    let cppResult2 =
        pinArray indices (fun idxPtr ->
            pinArray (mesh.Vertices) (fun vPtr ->
                pinArray cppDst2 (fun dstPtr ->
                    pinArray cppErr2 (fun errPtr ->
                        int (Native.meshopt_simplifySloppy(dstPtr, idxPtr, unativeint ic, vPtr, unativeint vc, unativeint stride, nativeint 0, unativeint targetCount, 0.01f, errPtr))))))

    let fsResult2 =
        pinArray indices (fun idxPtr ->
            pinArray (mesh.Vertices) (fun vPtr ->
                pinArray fsDst2 (fun dstPtr ->
                    pinArray fsErr2 (fun errPtr ->
                        MeshOptPort.Simplifier.meshopt_simplifySloppy
                            (NPtr.ofNI dstPtr) (NPtr.ofNI idxPtr) ic (NPtr.ofNI<float32> vPtr) vc stride (NPtr.ofNI<byte> (nativeint 0)) targetCount 0.01f (NPtr.ofNI errPtr)))))

    if cppResult2 <> fsResult2 then
        fail "simplifySloppy result count" (sprintf "C++=%d F#=%d" cppResult2 fsResult2)
    else
        pass (sprintf "simplifySloppy result count (%d)" cppResult2)

    if cppResult2 > 0 && fsResult2 > 0 then
        let cmpLen = min cppResult2 fsResult2
        let cppSlice2 = cppDst2.[..cmpLen-1]
        let fsSlice2 = fsDst2.[..cmpLen-1]
        compareArraysExact "simplifySloppy indices" cppSlice2 fsSlice2
        compareFloat "simplifySloppy error" cppErr2.[0] fsErr2.[0] 1e-6f

let testStripify (indices: uint32[]) (vertexCount: int) =
    printfn "\n=== Stripify ==="

    let ic = indices.Length
    let boundSize = int (Native.meshopt_stripifyBound(unativeint ic))
    let fsBoundSize = MeshOptPort.Stripifier.meshopt_stripifyBound ic

    if boundSize <> fsBoundSize then
        fail "stripifyBound" (sprintf "C++=%d F#=%d" boundSize fsBoundSize)
    else
        pass "stripifyBound"

    let cppStrip = Array.zeroCreate<uint32> boundSize
    let fsStrip = Array.zeroCreate<uint32> boundSize

    let cppStripCount =
        pinArray indices (fun idxPtr ->
            pinArray cppStrip (fun dstPtr ->
                int (Native.meshopt_stripify(dstPtr, idxPtr, unativeint ic, unativeint vertexCount, 0xFFFFFFFFu))))

    let fsStripCount =
        pinArray indices (fun idxPtr ->
            pinArray fsStrip (fun dstPtr ->
                MeshOptPort.Stripifier.meshopt_stripify
                    (NPtr.ofNI dstPtr) (NPtr.ofNI idxPtr) ic vertexCount 0xFFFFFFFFu))

    if cppStripCount <> fsStripCount then
        fail "stripify count" (sprintf "C++=%d F#=%d" cppStripCount fsStripCount)
    else
        pass (sprintf "stripify count (%d)" cppStripCount)

    if cppStripCount > 0 then
        let cppSlice = cppStrip.[..cppStripCount-1]
        let fsSlice = fsStrip.[..fsStripCount-1]
        compareArraysExact "stripify data" cppSlice fsSlice

    // Unstripify the strip back
    if cppStripCount > 0 then
        let unstripBound = int (Native.meshopt_unstripifyBound(unativeint cppStripCount))
        let fsUnstripBound = MeshOptPort.Stripifier.meshopt_unstripifyBound cppStripCount
        if unstripBound <> fsUnstripBound then
            fail "unstripifyBound" (sprintf "C++=%d F#=%d" unstripBound fsUnstripBound)
        else
            pass "unstripifyBound"

        let cppUnstrip = Array.zeroCreate<uint32> unstripBound
        let fsUnstrip = Array.zeroCreate<uint32> unstripBound
        let cppSlice = cppStrip.[..cppStripCount-1]

        let cppUnstripCount =
            pinArray cppSlice (fun srcPtr ->
                pinArray cppUnstrip (fun dstPtr ->
                    int (Native.meshopt_unstripify(dstPtr, srcPtr, unativeint cppStripCount, 0xFFFFFFFFu))))

        let fsUnstripCount =
            pinArray cppSlice (fun srcPtr ->
                pinArray fsUnstrip (fun dstPtr ->
                    MeshOptPort.Stripifier.meshopt_unstripify
                        (NPtr.ofNI dstPtr) (NPtr.ofNI srcPtr) cppStripCount 0xFFFFFFFFu))

        if cppUnstripCount <> fsUnstripCount then
            fail "unstripify count" (sprintf "C++=%d F#=%d" cppUnstripCount fsUnstripCount)
        else
            pass (sprintf "unstripify count (%d)" cppUnstripCount)

        if cppUnstripCount > 0 then
            let cppUS = cppUnstrip.[..cppUnstripCount-1]
            let fsUS = fsUnstrip.[..fsUnstripCount-1]
            compareArraysExact "unstripify data" cppUS fsUS

let testSpatialSort (mesh: ObjMesh) =
    printfn "\n=== Spatial Sort ==="

    let vc = mesh.Vertices.Length
    let stride = vertexSize

    let cppRemap = Array.zeroCreate<uint32> vc
    let fsRemap = Array.zeroCreate<uint32> vc

    pinArray (mesh.Vertices) (fun vPtr ->
        pinArray cppRemap (fun cppPtr ->
            Native.meshopt_spatialSortRemap(cppPtr, vPtr, unativeint vc, unativeint stride))
        pinArray fsRemap (fun fsPtr ->
            MeshOptPort.SpatialOrder.meshopt_spatialSortRemap
                (NPtr.ofNI fsPtr) (NPtr.ofNI<float32> vPtr) vc stride))

    compareArraysExact "spatialSortRemap" cppRemap fsRemap

let testMeshlets (indices: uint32[]) (mesh: ObjMesh) =
    printfn "\n=== Meshlets ==="

    let ic = indices.Length
    let vc = mesh.Vertices.Length
    let stride = vertexSize
    let maxVerts = 64
    let maxTris = 124

    let maxMeshlets = int (Native.meshopt_buildMeshletsBound(unativeint ic, unativeint maxVerts, unativeint maxTris))
    let fsBoundMeshlets = MeshOptPort.Clusterizer.meshopt_buildMeshletsBound ic maxVerts maxTris

    if maxMeshlets <> fsBoundMeshlets then
        fail "buildMeshletsBound" (sprintf "C++=%d F#=%d" maxMeshlets fsBoundMeshlets)
    else
        pass "buildMeshletsBound"

    let cppMeshlets = Array.zeroCreate<meshopt_Meshlet> maxMeshlets
    let fsMeshlets = Array.zeroCreate<meshopt_Meshlet> maxMeshlets
    let cppMV = Array.zeroCreate<uint32> ic
    let fsMV = Array.zeroCreate<uint32> ic
    let cppMT = Array.zeroCreate<byte> ic
    let fsMT = Array.zeroCreate<byte> ic

    let cppMeshletCount =
        pinArray indices (fun idxPtr ->
            pinArray (mesh.Vertices) (fun vPtr ->
                pinArray cppMeshlets (fun mPtr ->
                    pinArray cppMV (fun mvPtr ->
                        pinArray cppMT (fun mtPtr ->
                            int (Native.meshopt_buildMeshlets(mPtr, mvPtr, mtPtr, idxPtr, unativeint ic, vPtr, unativeint vc, unativeint stride, unativeint maxVerts, unativeint maxTris, 0.0f)))))))

    let fsMeshletCount =
        pinArray indices (fun idxPtr ->
            pinArray (mesh.Vertices) (fun vPtr ->
                pinArray fsMeshlets (fun mPtr ->
                    pinArray fsMV (fun mvPtr ->
                        pinArray fsMT (fun mtPtr ->
                            MeshOptPort.Clusterizer.meshopt_buildMeshlets
                                (NPtr.ofNI mPtr) (NPtr.ofNI mvPtr) (NPtr.ofNI mtPtr)
                                (NPtr.ofNI idxPtr) ic (NPtr.ofNI<float32> vPtr) vc stride maxVerts maxTris 0.0f)))))

    if cppMeshletCount <> fsMeshletCount then
        fail "buildMeshlets count" (sprintf "C++=%d F#=%d" cppMeshletCount fsMeshletCount)
    else
        pass (sprintf "buildMeshlets count (%d)" cppMeshletCount)

    if cppMeshletCount > 0 then
        // Compare meshlet structures
        let cppMS = cppMeshlets.[..cppMeshletCount-1]
        let fsMS = fsMeshlets.[..fsMeshletCount-1]
        compareArraysExact "buildMeshlets meshlets" cppMS fsMS

        // Compare total triangle counts
        let cppTotalTris = cppMS |> Array.sumBy (fun m -> int m.triangle_count)
        let fsTotalTris = fsMS |> Array.sumBy (fun m -> int m.triangle_count)
        if cppTotalTris <> fsTotalTris then
            fail "buildMeshlets total triangles" (sprintf "C++=%d F#=%d" cppTotalTris fsTotalTris)
        else
            pass (sprintf "buildMeshlets total triangles (%d)" cppTotalTris)

        // Test computeMeshletBounds on first meshlet
        let m = cppMS.[0]
        let mvSlice = cppMV.[int m.vertex_offset .. int m.vertex_offset + int m.vertex_count - 1]
        let mtSlice = cppMT.[int m.triangle_offset .. int m.triangle_offset + int m.triangle_count * 3 - 1]
        pinArray mvSlice (fun mvPtr ->
            pinArray mtSlice (fun mtPtr ->
                pinArray (mesh.Vertices) (fun vPtr ->
                    let cppBounds = Native.meshopt_computeMeshletBounds(mvPtr, mtPtr, unativeint (int m.triangle_count), vPtr, unativeint vc, unativeint stride)
                    let fsBounds = MeshOptPort.Clusterizer.meshopt_computeMeshletBounds (NPtr.ofNI mvPtr) (NPtr.ofNI mtPtr) (int m.triangle_count) (NPtr.ofNI<float32> vPtr) vc stride
                    compareFloat "meshletBounds center[0]" cppBounds.center_0 fsBounds.center_0 1e-4f
                    compareFloat "meshletBounds center[1]" cppBounds.center_1 fsBounds.center_1 1e-4f
                    compareFloat "meshletBounds center[2]" cppBounds.center_2 fsBounds.center_2 1e-4f
                    compareFloat "meshletBounds radius" cppBounds.radius fsBounds.radius 1e-4f
                    compareFloat "meshletBounds cone_cutoff" cppBounds.cone_cutoff fsBounds.cone_cutoff 1e-4f)))

let testIndexSequenceCodec (indices: uint32[]) (vertexCount: int) =
    printfn "\n=== Index Sequence Codec ==="

    let ic = indices.Length

    let boundSize = int (Native.meshopt_encodeIndexSequenceBound(unativeint ic, unativeint vertexCount))
    let fsBoundSize = MeshOptPort.IndexCodec.meshopt_encodeIndexSequenceBound ic vertexCount

    if boundSize <> fsBoundSize then
        fail "encodeIndexSequenceBound" (sprintf "C++=%d F#=%d" boundSize fsBoundSize)
    else
        pass "encodeIndexSequenceBound"

    let cppBuf = Array.zeroCreate<byte> boundSize
    let fsBuf = Array.zeroCreate<byte> boundSize

    let cppEncSize =
        pinArray indices (fun idxPtr ->
            pinArray cppBuf (fun bufPtr ->
                int (Native.meshopt_encodeIndexSequence(bufPtr, unativeint boundSize, idxPtr, unativeint ic))))

    let fsEncSize =
        pinArray indices (fun idxPtr ->
            pinArray fsBuf (fun bufPtr ->
                MeshOptPort.IndexCodec.meshopt_encodeIndexSequence
                    (NPtr.ofNI bufPtr) boundSize (NPtr.ofNI idxPtr) ic))

    if cppEncSize <> fsEncSize then
        fail "encodeIndexSequence size" (sprintf "C++=%d F#=%d" cppEncSize fsEncSize)
    else
        pass "encodeIndexSequence size"

    let cppEncBytes = cppBuf.[..cppEncSize-1]
    let fsEncBytes = fsBuf.[..fsEncSize-1]
    compareArraysExact "encodeIndexSequence data" cppEncBytes fsEncBytes

    // Cross-decode
    let fsDecoded = Array.zeroCreate<uint32> ic
    pinArray cppBuf (fun bufPtr ->
        pinArray fsDecoded (fun dstPtr ->
            let rc = MeshOptPort.IndexCodec.meshopt_decodeIndexSequence dstPtr ic 4 (NPtr.ofNI bufPtr) cppEncSize
            if rc <> 0 then fail "decodeIndexSequence(C++→F#)" (sprintf "error code %d" rc)))
    compareArraysExact "decodeIndexSequence C++→F#" indices fsDecoded

[<EntryPoint>]
let main argv =
    if argv |> Array.exists (fun a -> a = "--bench") then
        let useCounters = argv |> Array.exists (fun a -> a = "--counters")
        // Pass remaining args to BDN (strip --bench and --counters)
        let bdnArgs =
            argv
            |> Array.filter (fun a -> a <> "--bench" && a <> "--counters")
        let asm = Assembly.GetExecutingAssembly()
        if useCounters then
            BenchmarkSwitcher.FromAssembly(asm).Run(bdnArgs, Benchmark.PerfConfig()) |> ignore
        else
            BenchmarkSwitcher.FromAssembly(asm).Run(bdnArgs) |> ignore
        0
    else

    let objPath =
        let nonFlags = argv |> Array.filter (fun a -> not (a.StartsWith("--")))
        if nonFlags.Length > 0 then nonFlags.[0]
        else "/tmp/meshoptimizer/demo/pirate.obj"

    printfn "Loading %s..." objPath
    let mesh = loadObj objPath
    printfn "Loaded: %d vertices, %d indices (%d triangles)"
        mesh.Vertices.Length mesh.Indices.Length (mesh.Indices.Length / 3)

    // Pin the base arrays for the duration of tests
    let vHandle = GCHandle.Alloc(mesh.Vertices, GCHandleType.Pinned)
    let iHandle = GCHandle.Alloc(mesh.Indices, GCHandleType.Pinned)
    let verticesPtr = vHandle.AddrOfPinnedObject()
    let indicesPtr = iHandle.AddrOfPinnedObject()

    try
        testQuantization ()
        let _remap = testVertexRemap mesh verticesPtr indicesPtr
        let cacheOpt = testOptimizeVertexCache mesh.Indices mesh.Vertices.Length
        let overdrawOpt = testOptimizeOverdraw cacheOpt mesh
        testOptimizeVertexFetch overdrawOpt mesh
        testAnalysis mesh.Indices mesh
        testIndexCodec mesh.Indices mesh.Vertices.Length
        testIndexSequenceCodec mesh.Indices mesh.Vertices.Length
        testVertexCodec mesh
        testSimplify mesh.Indices mesh
        testStripify cacheOpt mesh.Vertices.Length
        testSpatialSort mesh
        testMeshlets mesh.Indices mesh

        printfn "\n========================================="
        printfn "Results: %d PASS, %d FAIL" passCount failCount
        printfn "========================================="
    finally
        iHandle.Free()
        vHandle.Free()

    if failCount > 0 then 1 else 0
