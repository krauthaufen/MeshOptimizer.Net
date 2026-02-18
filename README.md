# MeshOptPort

F# port of [meshoptimizer](https://github.com/zeux/meshoptimizer), a C/C++ library for mesh processing (vertex cache optimization, simplification, encoding, meshlet building, and more).

Line-by-line translation of all 17 source files — 19 F# files, ~11,600 lines. Uses `nativeptr<T>` for pointers, `System.Runtime.Intrinsics` for SIMD (SSE4.1/NEON with scalar fallback), and mirrors the C code structure closely. No external dependencies.

## Status

- All 49 correctness tests pass (byte-exact match against C++ `libmeshoptimizer.so`)
- Performance within 1–2x of C++ for most algorithms, 3–5x for codec functions (C++ uses hand-tuned SIMD)
- See [BENCHMARK.md](BENCHMARK.md) for detailed numbers

## Building

```bash
dotnet build -c Release
```

## Testing

Tests compare F# output against the C++ reference via P/Invoke. Requires building the C++ shared library first:

```bash
# Build C++ reference
cd /tmp && git clone https://github.com/zeux/meshoptimizer
cd meshoptimizer && mkdir build && cd build
cmake .. -DMESHOPT_BUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Run correctness tests
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests

# Run performance benchmarks
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench
```

## Project Structure

```
src/
  MeshOptPort/            # F# library
  MeshOptPort.Tests/      # Correctness tests + benchmarks
```

## File Mapping

| C++ Source | F# File | Description |
|------------|---------|-------------|
| meshoptimizer.h (types) | Types.fs | Structs, enums |
| — | NativeUtil.fs | `NPtr` helper module |
| allocator.cpp | Allocator.fs | RAII allocator (IDisposable) |
| quantization.cpp | Quantization.fs | Half-float, quantize/dequantize |
| vfetchoptimizer.cpp | VFetchOptimizer.fs | Vertex fetch optimization |
| indexanalyzer.cpp | IndexAnalyzer.fs | Cache/fetch/overdraw analysis |
| stripifier.cpp | Stripifier.fs | Triangle strip generation |
| rasterizer.cpp | Rasterizer.fs | Software rasterizer |
| vcacheoptimizer.cpp | VCacheOptimizer.fs | Vertex cache optimization |
| spatialorder.cpp | SpatialOrder.fs | Spatial sorting (Morton codes) |
| overdrawoptimizer.cpp | OverdrawOptimizer.fs | Overdraw optimization |
| partition.cpp | Partition.fs | Cluster partitioning |
| indexcodec.cpp | IndexCodec.fs | Index buffer codec |
| indexgenerator.cpp | IndexGenerator.fs | Vertex remap/dedup |
| vertexcodec.cpp | VertexCodec.fs | Vertex buffer codec (SSE/NEON) |
| vertexfilter.cpp | VertexFilter.fs | Vertex filters (Oct/Quat/Exp/Color) |
| meshletcodec.cpp | MeshletCodec.fs | Meshlet codec (SSE4.1/NEON) |
| clusterizer.cpp | Clusterizer.fs | Meshlet building + bounds |
| simplifier.cpp | Simplifier.fs | Mesh simplification (QEM) |

## What's Skipped

- WASM SIMD paths
- AVX512 paths
- TRACE/debug logging
- C++ template specializations (expanded to explicit typed functions)
