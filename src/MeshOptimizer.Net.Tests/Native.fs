// P/Invoke bindings for C++ libmeshoptimizer.so
// Used to compare C++ reference output against F# port
module MeshOptimizer.Net.Tests.Native

open System.Runtime.InteropServices
open MeshOptimizer.Net

// Note: C size_t = unativeint on .NET; C unsigned int = uint32; C void* = nativeint; C float* = nativeint

// ---- Index Generator ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_generateVertexRemap(nativeint destination, nativeint indices, unativeint index_count, nativeint vertices, unativeint vertex_count, unativeint vertex_size)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern void meshopt_remapVertexBuffer(nativeint destination, nativeint vertices, unativeint vertex_count, unativeint vertex_size, nativeint remap)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern void meshopt_remapIndexBuffer(nativeint destination, nativeint indices, unativeint index_count, nativeint remap)

// ---- Vertex Cache Optimizer ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern void meshopt_optimizeVertexCache(nativeint destination, nativeint indices, unativeint index_count, unativeint vertex_count)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern void meshopt_optimizeVertexCacheStrip(nativeint destination, nativeint indices, unativeint index_count, unativeint vertex_count)

// ---- Overdraw Optimizer ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern void meshopt_optimizeOverdraw(nativeint destination, nativeint indices, unativeint index_count, nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride, float32 threshold)

// ---- Vertex Fetch Optimizer ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_optimizeVertexFetch(nativeint destination, nativeint indices, unativeint index_count, nativeint vertices, unativeint vertex_count, unativeint vertex_size)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_optimizeVertexFetchRemap(nativeint destination, nativeint indices, unativeint index_count, unativeint vertex_count)

// ---- Index Buffer Codec ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_encodeIndexBuffer(nativeint buffer, unativeint buffer_size, nativeint indices, unativeint index_count)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_encodeIndexBufferBound(unativeint index_count, unativeint vertex_count)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern int meshopt_decodeIndexBuffer(nativeint destination, unativeint index_count, unativeint index_size, nativeint buffer, unativeint buffer_size)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern void meshopt_encodeIndexVersion(int version)

// ---- Index Sequence Codec ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_encodeIndexSequence(nativeint buffer, unativeint buffer_size, nativeint indices, unativeint index_count)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_encodeIndexSequenceBound(unativeint index_count, unativeint vertex_count)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern int meshopt_decodeIndexSequence(nativeint destination, unativeint index_count, unativeint index_size, nativeint buffer, unativeint buffer_size)

// ---- Vertex Buffer Codec ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_encodeVertexBuffer(nativeint buffer, unativeint buffer_size, nativeint vertices, unativeint vertex_count, unativeint vertex_size)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_encodeVertexBufferBound(unativeint vertex_count, unativeint vertex_size)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern int meshopt_decodeVertexBuffer(nativeint destination, unativeint vertex_count, unativeint vertex_size, nativeint buffer, unativeint buffer_size)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern void meshopt_encodeVertexVersion(int version)

// ---- Analysis ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern meshopt_VertexCacheStatistics meshopt_analyzeVertexCache(nativeint indices, unativeint index_count, unativeint vertex_count, uint32 cache_size, uint32 warp_size, uint32 primgroup_size)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern meshopt_VertexFetchStatistics meshopt_analyzeVertexFetch(nativeint indices, unativeint index_count, unativeint vertex_count, unativeint vertex_size)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern meshopt_OverdrawStatistics meshopt_analyzeOverdraw(nativeint indices, unativeint index_count, nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride)

// ---- Simplification ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_simplify(nativeint destination, nativeint indices, unativeint index_count, nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride, unativeint target_index_count, float32 target_error, uint32 options, nativeint result_error)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_simplifySloppy(nativeint destination, nativeint indices, unativeint index_count, nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride, nativeint vertex_lock, unativeint target_index_count, float32 target_error, nativeint result_error)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern float32 meshopt_simplifyScale(nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride)

// ---- Stripifier ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_stripify(nativeint destination, nativeint indices, unativeint index_count, unativeint vertex_count, uint32 restart_index)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_stripifyBound(unativeint index_count)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_unstripify(nativeint destination, nativeint indices, unativeint index_count, uint32 restart_index)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_unstripifyBound(unativeint index_count)

// ---- Spatial Sort ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern void meshopt_spatialSortRemap(nativeint destination, nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride)

// ---- Meshlet Building ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_buildMeshlets(nativeint meshlets, nativeint meshlet_vertices, nativeint meshlet_triangles, nativeint indices, unativeint index_count, nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride, unativeint max_vertices, unativeint max_triangles, float32 cone_weight)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern unativeint meshopt_buildMeshletsBound(unativeint index_count, unativeint max_vertices, unativeint max_triangles)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern meshopt_Bounds meshopt_computeClusterBounds(nativeint indices, unativeint index_count, nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern meshopt_Bounds meshopt_computeMeshletBounds(nativeint meshlet_vertices, nativeint meshlet_triangles, unativeint triangle_count, nativeint vertex_positions, unativeint vertex_count, unativeint vertex_positions_stride)

// ---- Quantization ----

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern uint16 meshopt_quantizeHalf(float32 v)

[<DllImport("meshoptimizer", CallingConvention = CallingConvention.Cdecl)>]
extern float32 meshopt_dequantizeHalf(uint16 h)
