// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
namespace MeshOptPort

open System
open System.Runtime.InteropServices

/// Version macro; major * 1000 + minor * 10 + patch
[<AutoOpen>]
module Version =
    [<Literal>]
    let MESHOPTIMIZER_VERSION = 1000 // 1.0

/// Vertex attribute stream
/// Each element takes size bytes, beginning at data, with stride controlling the spacing between successive elements (stride >= size).
[<Struct; StructLayout(LayoutKind.Sequential)>]
type meshopt_Stream =
    val mutable data: nativeint
    val mutable size: unativeint
    val mutable stride: unativeint

/// Vertex transform cache statistics
[<Struct; StructLayout(LayoutKind.Sequential)>]
type meshopt_VertexCacheStatistics =
    val mutable vertices_transformed: uint32
    val mutable warps_executed: uint32
    /// transformed vertices / triangle count; best case 0.5, worst case 3.0, optimum depends on topology
    val mutable acmr: float32
    /// transformed vertices / vertex count; best case 1.0, worst case 6.0, optimum is 1.0 (each vertex is transformed once)
    val mutable atvr: float32

/// Vertex fetch cache statistics
[<Struct; StructLayout(LayoutKind.Sequential)>]
type meshopt_VertexFetchStatistics =
    val mutable bytes_fetched: uint32
    /// fetched bytes / vertex buffer size; best case 1.0 (each byte is fetched once)
    val mutable overfetch: float32

/// Overdraw statistics
[<Struct; StructLayout(LayoutKind.Sequential)>]
type meshopt_OverdrawStatistics =
    val mutable pixels_covered: uint32
    val mutable pixels_shaded: uint32
    /// shaded pixels / covered pixels; best case 1.0
    val mutable overdraw: float32

/// Coverage statistics
[<Struct; StructLayout(LayoutKind.Sequential)>]
type meshopt_CoverageStatistics =
    val mutable coverage_0: float32
    val mutable coverage_1: float32
    val mutable coverage_2: float32
    /// viewport size in mesh coordinates
    val mutable extent: float32

/// Meshlet is a small mesh cluster (subset) that consists of:
/// - triangles, an 8-bit micro triangle (index) buffer, that for each triangle specifies three local vertices to use;
/// - vertices, a 32-bit vertex indirection buffer, that for each local vertex specifies which mesh vertex to fetch vertex attributes from.
[<Struct; StructLayout(LayoutKind.Sequential)>]
type meshopt_Meshlet =
    /// offset within meshlet_vertices array
    val mutable vertex_offset: uint32
    /// offset within meshlet_triangles array
    val mutable triangle_offset: uint32
    /// number of vertices used in the meshlet
    val mutable vertex_count: uint32
    /// number of triangles used in the meshlet
    val mutable triangle_count: uint32

/// Cluster bounds for frustum, backface and occlusion culling
[<Struct; StructLayout(LayoutKind.Sequential)>]
type meshopt_Bounds =
    // bounding sphere
    val mutable center_0: float32
    val mutable center_1: float32
    val mutable center_2: float32
    val mutable radius: float32
    // normal cone
    val mutable cone_apex_0: float32
    val mutable cone_apex_1: float32
    val mutable cone_apex_2: float32
    val mutable cone_axis_0: float32
    val mutable cone_axis_1: float32
    val mutable cone_axis_2: float32
    /// cos(angle/2)
    val mutable cone_cutoff: float32
    // normal cone axis and cutoff, stored in 8-bit SNORM format; decode using x/127.0
    val mutable cone_axis_s8_0: sbyte
    val mutable cone_axis_s8_1: sbyte
    val mutable cone_axis_s8_2: sbyte
    val mutable cone_cutoff_s8: sbyte

/// Exponent encoding mode for meshopt_encodeFilterExp
type meshopt_EncodeExpMode =
    /// Use separate values for each component (maximum quality)
    | Separate = 0
    /// Use shared value for all components of each vector (better compression)
    | SharedVector = 1
    /// Use shared value for each component of all vectors (best compression)
    | SharedComponent = 2
    /// Use separate values for each component, but clamp to 0 (good quality if very small values are not important)
    | Clamped = 3

/// Simplification option flags
[<Flags>]
type meshopt_SimplifyOptions =
    | None = 0
    /// Do not move vertices that are located on the topological border
    | LockBorder = 1
    /// Improve simplification performance assuming input indices are a sparse subset of the mesh
    | Sparse = 2
    /// Treat error limit and resulting error as absolute instead of relative to mesh extents
    | ErrorAbsolute = 4
    /// Remove disconnected parts of the mesh during simplification incrementally
    | Prune = 8
    /// Produce more regular triangle sizes and shapes during simplification
    | Regularize = 16
    /// Allow collapses across attribute discontinuities
    | Permissive = 32

/// Simplification vertex flags/locks
[<Flags>]
type meshopt_SimplifyVertexFlags =
    | None = 0
    /// Do not move this vertex
    | Lock = 1
    /// Protect attribute discontinuity at this vertex
    | Protect = 2
