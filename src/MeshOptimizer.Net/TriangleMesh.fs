namespace MeshOptimizer.Net

type VertexAttribute =
    {
        Data        : Mem<float32>
        Dimension   : int
        Stride      : int
    }

type TriangleMesh =
    {
        Indices : Mem<uint32>
        
        Vertices : Mem<float32>
        VertexStride : int
        
        Attributes : Map<string, VertexAttribute>
    }
    
    member internal x.RealVertexStride =
        if x.VertexStride = 0 then 3
        else x.VertexStride
    
    member x.TriangleCount =
        if isNull x.Indices then
            if isNull x.Vertices then 0
            else (x.Vertices.Length / x.RealVertexStride) / 3
        else
            x.Indices.Length / 3
    

module TriangleMesh =
    
    type private MemBuilder() =
        member inline x.Bind(m : Mem<'a>, [<InlineIfLambda>] f : nativeptr<'a> -> 'r) = Mem.pin m f
        member inline x.Delay([<InlineIfLambda>] action : unit -> 'r) = action()
        member inline x.Zero() = ()
        member inline x.Return(v : 'r) = v
            
    let private mem = MemBuilder()
    
    
    let simplify (targetError : float) (targetIndexCount : int) (mesh : TriangleMesh) =
        
        
        let triangleCount = mesh.TriangleCount
        let indexCount = triangleCount * 3
        
        let dst = Array.zeroCreate<uint32> indexCount
        let dstMem = Mem.Create dst
        
        let outError = Array.zeroCreate<float32> 1
        use outErrorPtr = fixed outError
        
        mem {
            let! dstPtr = dstMem
            let! idxPtr = mesh.Indices
            let! vertPtr = mesh.Vertices
            
            let resultIndexCount = 
                Simplifier.meshopt_simplify
                    dstPtr
                    idxPtr
                    indexCount
                    vertPtr
                    (mesh.Vertices.Length / mesh.RealVertexStride)
                    (sizeof<float32> * mesh.RealVertexStride)
                    targetIndexCount
                    (float32 targetError)
                    0u
                    outErrorPtr
            
            let outIndexArray = dstMem.[..resultIndexCount - 1].ToArray()
            
            let err = float outError.[0]
            
            return err, {
                Indices = Mem.Create outIndexArray
                Vertices = mesh.Vertices
                VertexStride = mesh.VertexStride
                Attributes = mesh.Attributes
            }
        }
    



