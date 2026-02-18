# Performance Benchmark: F# Port vs C++ Reference

## Methodology

Uses [BenchmarkDotNet](https://benchmarkdotnet.org/) for statistically rigorous measurements with confidence intervals, outlier detection, and memory allocation tracking. Each function has a dedicated benchmark class with `Cpp` (baseline, via P/Invoke) and `FSharp` methods.

**Mesh generation:** Flat grid meshes with interleaved position/normal/texcoord vertices (32 bytes each). Three parameterized sizes (`GridSize` = 32, 224, 1024):

| Size   | Grid      | Vertices    | Triangles   | Indices     |
|--------|-----------|-------------|-------------|-------------|
| Small  | 32x32     | 1,024       | 1,922       | 5,766       |
| Medium | 224x224   | 50,176      | 99,458      | 298,374     |
| Large  | 1024x1024 | 1,048,576   | 2,093,058   | 6,279,174   |

**Timing:** BenchmarkDotNet auto-selects iteration count per operation speed, reports mean with 99.9% confidence interval, and detects outliers. `[<MemoryDiagnoser>]` tracks managed allocations.

**Hardware counters:** Optional `--counters` flag enables `perf`-based counters (branch mispredictions, cache misses, instructions retired). Requires Linux `perf` and `kernel.perf_event_paranoid ≤ 1`.

**Build:** `dotnet run -c Release` (.NET 8, RyuJIT). CPU: AMD Ryzen 9 7940HS capped at 3 GHz.

## Results

### Small (1K triangles)

| Function                | C++ (ms) | F# (ms) | Ratio |
|-------------------------|----------|---------|-------|
| optimizeVertexCache     |     0.15 |    0.25 |  1.7x |
| optimizeOverdraw        |     0.04 |    0.10 |  2.7x |
| optimizeVertexFetch     |     0.03 |    0.03 |  1.3x |
| simplify                |     0.20 |    1.39 |  7.0x |
| encodeIndexBuffer       |     0.01 |    0.41 | 27.4x |
| decodeIndexBuffer       |     0.01 |    0.06 |  7.8x |
| encodeVertexBuffer      |     0.04 |    1.03 | 23.2x |
| decodeVertexBuffer      |     0.01 |    0.19 | 14.9x |
| buildMeshlets           |     0.52 |    3.87 |  7.4x |
| stripify                |     0.03 |    0.09 |  3.6x |
| spatialSortRemap        |     0.02 |    0.11 |  6.7x |

Small-mesh ratios are inflated by fixed overhead (JIT, P/Invoke, GC pinning) that dominates sub-millisecond timings.

### Medium (100K triangles)

| Function                | C++ (ms) | F# (ms) | Ratio |
|-------------------------|----------|---------|-------|
| optimizeVertexCache     |     8.12 |   13.05 |  1.6x |
| optimizeOverdraw        |     1.89 |    1.98 |  1.0x |
| optimizeVertexFetch     |     1.32 |    1.82 |  1.4x |
| simplify                |    15.09 |   33.22 |  2.2x |
| encodeIndexBuffer       |     0.75 |    4.40 |  5.9x |
| decodeIndexBuffer       |     0.84 |    2.10 |  2.5x |
| encodeVertexBuffer      |     2.14 |    5.20 |  2.4x |
| decodeVertexBuffer      |     1.00 |    1.86 |  1.9x |
| buildMeshlets           |    42.00 |   64.99 |  1.5x |
| stripify                |     1.29 |    2.38 |  1.8x |
| spatialSortRemap        |     0.83 |    1.51 |  1.8x |

### Large (2M triangles)

| Function                | C++ (ms) | F# (ms) | Ratio |
|-------------------------|----------|---------|-------|
| optimizeVertexCache     |   301.33 |  391.72 |  1.3x |
| optimizeOverdraw        |    62.25 |   61.56 |  1.0x |
| optimizeVertexFetch     |    26.43 |   17.50 |  0.7x |
| simplify                |   650.92 | 1135.92 |  1.7x |
| encodeIndexBuffer       |    16.33 |   89.99 |  5.5x |
| decodeIndexBuffer       |     8.78 |   36.48 |  4.2x |
| encodeVertexBuffer      |    45.14 |  119.05 |  2.6x |
| decodeVertexBuffer      |    13.84 |   28.36 |  2.0x |
| buildMeshlets           |  1180.67 | 1602.75 |  1.4x |
| stripify                |    26.19 |   50.99 |  1.9x |
| spatialSortRemap        |    41.60 |   49.12 |  1.2x |

Large-mesh ratios are the most representative since computation dominates overhead.

## Summary

At production scale (Large mesh), the F# port falls into three performance tiers:

| Tier          | Ratio  | Functions |
|---------------|--------|-----------|
| At parity     | 0.7-1.2x | optimizeOverdraw, optimizeVertexFetch, spatialSortRemap |
| Close         | 1.3-1.9x | optimizeVertexCache, buildMeshlets, simplify, stripify, decodeVertexBuffer |
| Codec gap     | 2.6-5.5x | encodeIndexBuffer, decodeIndexBuffer, encodeVertexBuffer |

The codec functions show the largest gap because C++ meshoptimizer uses hand-tuned SIMD intrinsics for byte-level encoding/decoding. The algorithmic functions (optimization, simplification, clusterization) are within 2x, and some match or beat C++ thanks to .NET's efficient array access and RyuJIT optimizations.

## Running

```bash
# Run all benchmarks
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench

# Run a single function
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench --filter *VertexCache*

# With hardware performance counters (needs perf + paranoid ≤ 1)
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench --counters

# Short run (fewer iterations, faster)
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench --job short
```
