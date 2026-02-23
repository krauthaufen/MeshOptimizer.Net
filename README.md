# MeshOptimizerDotNet

F# port of [meshoptimizer](https://github.com/zeux/meshoptimizer), a C/C++ library for mesh processing (vertex cache optimization, simplification, encoding, meshlet building, and more).

Line-by-line translation of all 17 source files — 19 F# files, ~11,600 lines. Uses `nativeptr<T>` for pointers, `System.Runtime.Intrinsics` for SIMD (SSE4.1/NEON with scalar fallback), and mirrors the C code structure closely. No external dependencies.

**Based on meshoptimizer [`d033a68`](https://github.com/zeux/meshoptimizer/commit/d033a68) (v1.0.1+109, 2025).** To sync with a newer version, diff the C++ sources against this commit and apply corresponding changes to the F# files (see [file mapping](#file-mapping) below).

## Status

All 49 correctness tests pass (byte-exact match against C++ `libmeshoptimizer.so`). See [BENCHMARK.md](BENCHMARK.md) for detailed numbers.

### Performance vs C++ (GridSize=1024, ~2M triangles)

| Function | C++ | F# | Ratio |
|----------|----:|---:|------:|
| optimizeOverdraw | 21,846 μs | 25,537 μs | 1.17 |
| optimizeVertexCache | 164,546 μs | 215,398 μs | 1.31 |
| spatialSortRemap | 24,267 μs | 33,309 μs | 1.37 |
| buildMeshlets | 699,026 μs | 995,270 μs | 1.42 |
| optimizeVertexFetch | 7,340 μs | 10,705 μs | 1.46 |
| decodeIndexBuffer | 5,583 μs | 8,436 μs | 1.51 |
| simplify | 250,538 μs | 498,074 μs | 1.99 |
| stripify | 16,710 μs | 34,984 μs | 2.09 |
| decodeVertexBuffer | 8,713 μs | 18,694 μs | 2.15 |
| encodeIndexBuffer | 11,713 μs | 25,206 μs | 2.15 |
| encodeVertexBuffer | 31,146 μs | 79,236 μs | 2.54 |

BenchmarkDotNet, AMD Ryzen 7 7700X, .NET 8, RyuJIT AVX-512.

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
cmake .. -DMESHOPT_BUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-ffp-contract=off"
make -j$(nproc)

# Run correctness tests
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptimizerDotNet.Tests

# Run performance benchmarks (BenchmarkDotNet)
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptimizerDotNet.Tests -- --bench

# Run a single benchmark
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptimizerDotNet.Tests -- --bench --filter *Simplify*
```

### FMA contraction and simplifier tests

The C++ shared library must be built with `-ffp-contract=off` for all 49 tests to pass. Without this flag, the simplifier tests (`simplify indices`, `simplify error`, `simplifySloppy indices`) will fail on ARM64 due to FMA (fused multiply-add) contraction.

On ARM64, clang with `-O2`/`-O3` fuses `a * b + c` into a single FMA instruction that rounds once, while .NET's RyuJIT emits separate multiply and add instructions that round twice. The tiny floating-point differences in quadric error computation cause different edge collapse ordering in the simplifier, which cascades into different output indices. The results are equally valid simplifications — just not byte-identical.

On x86-64, this is typically not an issue since clang doesn't fuse FP operations by default unless explicitly enabled (`-mfma -ffp-contract=fast`).

## Project Structure

```
src/
  MeshOptimizerDotNet/            # F# library
  MeshOptimizerDotNet.Tests/      # Correctness tests + benchmarks
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
