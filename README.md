# MeshOptimizer.Net

F# port of [meshoptimizer](https://github.com/zeux/meshoptimizer), a C/C++ library for mesh processing (vertex cache optimization, simplification, encoding, meshlet building, and more).

Line-by-line translation of all 17 source files — 19 F# files, ~11,600 lines. Uses `nativeptr<T>` for pointers, `System.Runtime.Intrinsics` for SIMD (SSE4.1/NEON with scalar fallback), and mirrors the C code structure closely. No external dependencies.

**Based on meshoptimizer [`d033a68`](https://github.com/zeux/meshoptimizer/commit/d033a68) (v1.0.1+109, 2025).** To sync with a newer version, diff the C++ sources against this commit and apply corresponding changes to the F# files (see [file mapping](#file-mapping) below).

## Status

- All 49 correctness tests pass (byte-exact match against C++ `libmeshoptimizer.so`)
- Performance within 1.1–2.5x of C++ across all functions (BenchmarkDotNet, Ryzen 7 7700X)
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
cmake .. -DMESHOPT_BUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-ffp-contract=off"
make -j$(nproc)

# Run correctness tests
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptimizer.Net.Tests

# Run performance benchmarks (BenchmarkDotNet)
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptimizer.Net.Tests -- --bench

# Run a single benchmark
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptimizer.Net.Tests -- --bench --filter *Simplify*
```

### FMA contraction and simplifier tests

The C++ shared library must be built with `-ffp-contract=off` for all 49 tests to pass. Without this flag, the simplifier tests (`simplify indices`, `simplify error`, `simplifySloppy indices`) will fail on ARM64 due to FMA (fused multiply-add) contraction.

On ARM64, clang with `-O2`/`-O3` fuses `a * b + c` into a single FMA instruction that rounds once, while .NET's RyuJIT emits separate multiply and add instructions that round twice. The tiny floating-point differences in quadric error computation cause different edge collapse ordering in the simplifier, which cascades into different output indices. The results are equally valid simplifications — just not byte-identical.

On x86-64, this is typically not an issue since clang doesn't fuse FP operations by default unless explicitly enabled (`-mfma -ffp-contract=fast`).

## Project Structure

```
src/
  MeshOptimizer.Net/            # F# library
  MeshOptimizer.Net.Tests/      # Correctness tests + benchmarks
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
