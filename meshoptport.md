# MeshOptPort — meshoptimizer F# Port

F# port of the [meshoptimizer](https://github.com/zeux/meshoptimizer) C/C++ library. Located at `~/MeshOptPort/`.

## Overview

Line-by-line translation of all 17 meshoptimizer source files to F#. Uses `nativeptr<T>` for C pointers, `System.Runtime.Intrinsics` for SIMD, and mirrors the C code structure closely.

- **19 F# source files, 11,590 lines**
- Target: `net8.0`, AllowUnsafeBlocks, CheckOverflow=false
- No external dependencies

## File Mapping

| C++ Source | F# File | Lines | Description |
|------------|---------|-------|-------------|
| meshoptimizer.h (types) | Types.fs | 127 | Structs, enums |
| — | NativeUtil.fs | 29 | `NPtr` helper module (AutoOpen) |
| allocator.cpp | Allocator.fs | 65 | RAII allocator (IDisposable) |
| quantization.cpp | Quantization.fs | 94 | Half-float, quantize/dequantize |
| vfetchoptimizer.cpp | VFetchOptimizer.fs | 68 | Vertex fetch optimization |
| indexanalyzer.cpp | IndexAnalyzer.fs | 116 | Cache/fetch/overdraw analysis |
| stripifier.cpp | Stripifier.fs | 227 | Triangle strip generation |
| rasterizer.cpp | Rasterizer.fs | 249 | Software rasterizer |
| vcacheoptimizer.cpp | VCacheOptimizer.fs | 427 | Vertex cache optimization |
| spatialorder.cpp | SpatialOrder.fs | 293 | Spatial sorting (Morton codes) |
| overdrawoptimizer.cpp | OverdrawOptimizer.fs | 301 | Overdraw optimization |
| partition.cpp | Partition.fs | 628 | Cluster partitioning |
| indexcodec.cpp | IndexCodec.fs | 688 | Index buffer codec |
| indexgenerator.cpp | IndexGenerator.fs | 632 | Vertex remap/dedup |
| vertexcodec.cpp | VertexCodec.fs | 1426 | Vertex buffer codec (SSE/NEON) |
| vertexfilter.cpp | VertexFilter.fs | 1032 | Vertex filters (Oct/Quat/Exp/Color) |
| meshletcodec.cpp | MeshletCodec.fs | 1223 | Meshlet codec (SSE4.1/NEON) |
| clusterizer.cpp | Clusterizer.fs | 1739 | Meshlet building + bounds |
| simplifier.cpp | Simplifier.fs | 2221 | Mesh simplification (QEM) |

## Translation Patterns

### Pointers

All C pointers become `nativeptr<T>`. The `NPtr` module (AutoOpen) wraps `NativePtr`:

```fsharp
NPtr.get ptr i        // ptr[i]
NPtr.set ptr i v      // ptr[i] = v
NPtr.add ptr n        // ptr + n
NPtr.cast<T,U> ptr    // (U*)ptr
NPtr.memcpy dst src n // memcpy
NPtr.memset ptr v n   // memset
NPtr.toNI ptr         // (nativeint)ptr
NPtr.ofNI<T> ni       // (T*)ni
```

### Memory Management

`meshopt_Allocator` implements `IDisposable` — tracks up to 24 allocations, frees on dispose:

```fsharp
use allocator = new meshopt_Allocator()
let buffer = allocator.allocate<uint32>(count)
// auto-freed when allocator is disposed
```

### SIMD

Runtime detection with scalar fallback:

```fsharp
if Sse41.IsSupported then
    // SSE path
elif AdvSimd.Arm64.IsSupported then
    // NEON path
else
    // scalar fallback
```

Maps `_mm_*` to `Sse*.Method()`, `v*q_*` to `AdvSimd*.Method()`. WASM SIMD paths skipped.

### Operators

| C/C++ | F# |
|-------|-----|
| `>>` (right shift) | `>>>` |
| `<<` (left shift) | `<<<` |
| `^` (XOR) | `^^^` |
| `&` (AND) | `&&&` |
| `\|` (OR) | `\|\|\|` |
| `~` (NOT) | `~~~` |

### Key Restrictions

- **byref in closures**: F# forbids `byref<T>` in closures/nested functions. Solution: inline dispatch directly at call sites, or pass structs by value and return new structs.
- **Struct default ctors with nativeptr fields**: Use `Unchecked.defaultof<T>` instead.
- **`let inline private`**: keyword order matters (not `let private inline`).

## What's Skipped

- WASM SIMD paths
- AVX512 paths
- TRACE/debug logging
- C++ template specializations (expanded to explicit typed functions)

## Building

```bash
cd ~/MeshOptPort
dotnet build MeshOptPort/MeshOptPort.fsproj
```

## Status

All files compile with 0 errors, 0 warnings. Not yet tested against the C++ reference implementation.
