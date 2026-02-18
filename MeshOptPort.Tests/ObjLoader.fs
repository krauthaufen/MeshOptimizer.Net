// Minimal OBJ loader for test meshes
// Parses v/vn/vt/f lines, triangulates quads, produces interleaved vertex + index buffers
module MeshOptPort.Tests.ObjLoader

open System
open System.IO
open System.Collections.Generic
open System.Runtime.InteropServices

/// Interleaved vertex: position (3) + normal (3) + texcoord (2) = 8 floats = 32 bytes
[<Struct; StructLayout(LayoutKind.Sequential)>]
type Vertex =
    val mutable px: float32
    val mutable py: float32
    val mutable pz: float32
    val mutable nx: float32
    val mutable ny: float32
    val mutable nz: float32
    val mutable tx: float32
    val mutable ty: float32

    new(px, py, pz, nx, ny, nz, tx, ty) =
        { px = px; py = py; pz = pz; nx = nx; ny = ny; nz = nz; tx = tx; ty = ty }

let vertexSize = sizeof<Vertex> // 32 bytes

type ObjMesh = {
    Vertices: Vertex[]
    Indices: uint32[]
}

/// Parse an OBJ file into deduplicated vertex + index buffers
let loadObj (path: string) : ObjMesh =
    let positions = ResizeArray<struct(float32 * float32 * float32)>()
    let normals = ResizeArray<struct(float32 * float32 * float32)>()
    let texcoords = ResizeArray<struct(float32 * float32)>()

    // Map from (posIdx, normIdx, texIdx) -> unique vertex index
    let vertexMap = Dictionary<struct(int * int * int), int>()
    let vertices = ResizeArray<Vertex>()
    let indices = ResizeArray<uint32>()

    let parseFloat (s: string) = Single.Parse(s, Globalization.CultureInfo.InvariantCulture)

    let parseFaceVertex (s: string) =
        let parts = s.Split('/')
        let vi = Int32.Parse(parts.[0]) - 1
        let ti = if parts.Length > 1 && parts.[1] <> "" then Int32.Parse(parts.[1]) - 1 else 0
        let ni = if parts.Length > 2 && parts.[2] <> "" then Int32.Parse(parts.[2]) - 1 else 0
        struct(vi, ni, ti)

    let getOrAddVertex (key: struct(int * int * int)) =
        match vertexMap.TryGetValue(key) with
        | true, idx -> uint32 idx
        | false, _ ->
            let struct(vi, ni, ti) = key
            let struct(px, py, pz) = if vi < positions.Count then positions.[vi] else struct(0.0f, 0.0f, 0.0f)
            let struct(nx, ny, nz) = if ni < normals.Count then normals.[ni] else struct(0.0f, 0.0f, 1.0f)
            let struct(tx, ty) = if ti < texcoords.Count then texcoords.[ti] else struct(0.0f, 0.0f)
            let idx = vertices.Count
            vertices.Add(Vertex(px, py, pz, nx, ny, nz, tx, ty))
            vertexMap.[key] <- idx
            uint32 idx

    for line in File.ReadLines(path) do
        let line = line.Trim()
        if line.Length > 0 && line.[0] <> '#' then
            let parts = line.Split([| ' '; '\t' |], StringSplitOptions.RemoveEmptyEntries)
            if parts.Length >= 2 then
                match parts.[0] with
                | "v" when parts.Length >= 4 ->
                    positions.Add(struct(parseFloat parts.[1], parseFloat parts.[2], parseFloat parts.[3]))
                | "vn" when parts.Length >= 4 ->
                    normals.Add(struct(parseFloat parts.[1], parseFloat parts.[2], parseFloat parts.[3]))
                | "vt" when parts.Length >= 3 ->
                    texcoords.Add(struct(parseFloat parts.[1], parseFloat parts.[2]))
                | "f" when parts.Length >= 4 ->
                    // Triangulate: fan from first vertex
                    let faceVerts = Array.init (parts.Length - 1) (fun i -> parseFaceVertex parts.[i + 1])
                    for i = 1 to faceVerts.Length - 2 do
                        indices.Add(getOrAddVertex faceVerts.[0])
                        indices.Add(getOrAddVertex faceVerts.[i])
                        indices.Add(getOrAddVertex faceVerts.[i + 1])
                | _ -> ()

    { Vertices = vertices.ToArray(); Indices = indices.ToArray() }
