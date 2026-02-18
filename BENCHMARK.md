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

**Build:** `dotnet run -c Release` (.NET 8, RyuJIT AVX-512). CPU: AMD Ryzen 7 7700X.

## Results

### optimizeVertexCache

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **82.92 μs** | **0.801 μs** | **0.749 μs** | **1.00** |
| FSharp | 32 | 122.27 μs | 2.214 μs | 2.071 μs | 1.47 |
| **Cpp** | **224** | **4,795 μs** | **58.81 μs** | **55.01 μs** | **1.00** |
| FSharp | 224 | 7,222 μs | 108.3 μs | 101.3 μs | 1.51 |
| **Cpp** | **1024** | **164,546 μs** | **2,229 μs** | **1,976 μs** | **1.00** |
| FSharp | 1024 | 215,398 μs | 4,091 μs | 10,920 μs | 1.31 |

### optimizeOverdraw

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **18.25 μs** | **0.166 μs** | **0.156 μs** | **1.00** |
| FSharp | 32 | 20.99 μs | 0.120 μs | 0.112 μs | 1.15 |
| **Cpp** | **224** | **937.6 μs** | **12.80 μs** | **11.98 μs** | **1.00** |
| FSharp | 224 | 1,052 μs | 9.677 μs | 9.052 μs | 1.12 |
| **Cpp** | **1024** | **21,846 μs** | **337.3 μs** | **315.6 μs** | **1.00** |
| FSharp | 1024 | 25,537 μs | 229.8 μs | 191.9 μs | 1.17 |

### optimizeVertexFetch

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **5.341 μs** | **0.103 μs** | **0.106 μs** | **1.00** |
| FSharp | 32 | 10.03 μs | 0.077 μs | 0.072 μs | 1.88 |
| **Cpp** | **224** | **861.9 μs** | **6.562 μs** | **6.138 μs** | **1.00** |
| FSharp | 224 | 1,038 μs | 20.61 μs | 35.01 μs | 1.20 |
| **Cpp** | **1024** | **7,340 μs** | **39.53 μs** | **36.98 μs** | **1.00** |
| FSharp | 1024 | 10,705 μs | 73.72 μs | 68.96 μs | 1.46 |

### simplify

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **133.8 μs** | **0.94 μs** | **0.88 μs** | **1.00** |
| FSharp | 32 | 319.2 μs | 1.40 μs | 1.31 μs | 2.39 |
| **Cpp** | **224** | **10,180 μs** | **108.0 μs** | **101.0 μs** | **1.00** |
| FSharp | 224 | 20,070 μs | 224.6 μs | 210.1 μs | 1.97 |
| **Cpp** | **1024** | **250,538 μs** | **2,592 μs** | **2,424 μs** | **1.00** |
| FSharp | 1024 | 498,074 μs | 9,872 μs | 19,254 μs | 1.99 |

### encodeIndexBuffer

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **11.62 μs** | **0.228 μs** | **0.213 μs** | **1.00** |
| FSharp | 32 | 24.24 μs | 0.400 μs | 0.374 μs | 2.09 |
| **Cpp** | **224** | **616.0 μs** | **10.66 μs** | **9.975 μs** | **1.00** |
| FSharp | 224 | 1,271 μs | 15.15 μs | 14.18 μs | 2.06 |
| **Cpp** | **1024** | **11,713 μs** | **175.6 μs** | **164.2 μs** | **1.00** |
| FSharp | 1024 | 25,206 μs | 495.0 μs | 570.1 μs | 2.15 |

### decodeIndexBuffer

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **5.126 μs** | **0.081 μs** | **0.076 μs** | **1.00** |
| FSharp | 32 | 8.093 μs | 0.161 μs | 0.151 μs | 1.58 |
| **Cpp** | **224** | **284.7 μs** | **3.742 μs** | **3.501 μs** | **1.00** |
| FSharp | 224 | 419.4 μs | 3.801 μs | 3.555 μs | 1.47 |
| **Cpp** | **1024** | **5,583 μs** | **21.87 μs** | **20.46 μs** | **1.00** |
| FSharp | 1024 | 8,436 μs | 159.4 μs | 149.1 μs | 1.51 |

### encodeVertexBuffer

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **32.82 μs** | **0.615 μs** | **0.604 μs** | **1.00** |
| FSharp | 32 | 73.22 μs | 1.394 μs | 1.491 μs | 2.23 |
| **Cpp** | **224** | **1,561 μs** | **14.44 μs** | **12.80 μs** | **1.00** |
| FSharp | 224 | 3,494 μs | 56.77 μs | 53.10 μs | 2.24 |
| **Cpp** | **1024** | **31,146 μs** | **268.9 μs** | **251.6 μs** | **1.00** |
| FSharp | 1024 | 79,236 μs | 756.3 μs | 707.5 μs | 2.54 |

### decodeVertexBuffer

| Method | GridSize | Mean | Error | StdDev | Ratio | Alloc Ratio |
|--------|----------|-----:|------:|-------:|------:|------------:|
| **Cpp** | **32** | **7.492 μs** | **0.131 μs** | **0.122 μs** | **1.00** | **1.00** |
| FSharp | 32 | 18.96 μs | 0.366 μs | 0.548 μs | 2.53 | 2.52 |
| **Cpp** | **224** | **372.1 μs** | **6.479 μs** | **5.743 μs** | **1.00** | **1.00** |
| FSharp | 224 | 1,175 μs | 22.06 μs | 20.64 μs | 3.16 | 2.33 |
| **Cpp** | **1024** | **8,713 μs** | **90.62 μs** | **84.76 μs** | **1.00** | **1.00** |
| FSharp | 1024 | 18,694 μs | 143.8 μs | 127.4 μs | 2.15 | 2.35 |

### buildMeshlets

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **380.8 μs** | **5.39 μs** | **5.05 μs** | **1.00** |
| FSharp | 32 | 682.4 μs | 5.16 μs | 4.58 μs | 1.79 |
| **Cpp** | **224** | **31,516 μs** | **218.0 μs** | **203.9 μs** | **1.00** |
| FSharp | 224 | 42,403 μs | 383.5 μs | 358.7 μs | 1.35 |
| **Cpp** | **1024** | **699,026 μs** | **6,167 μs** | **5,769 μs** | **1.00** |
| FSharp | 1024 | 995,270 μs | 8,637 μs | 8,079 μs | 1.42 |

### stripify

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **16.50 μs** | **0.087 μs** | **0.077 μs** | **1.00** |
| FSharp | 32 | 33.05 μs | 0.375 μs | 0.333 μs | 2.00 |
| **Cpp** | **224** | **852.7 μs** | **9.325 μs** | **8.722 μs** | **1.00** |
| FSharp | 224 | 1,683 μs | 11.59 μs | 10.84 μs | 1.97 |
| **Cpp** | **1024** | **16,710 μs** | **91.74 μs** | **85.82 μs** | **1.00** |
| FSharp | 1024 | 34,984 μs | 357.0 μs | 333.9 μs | 2.09 |

### spatialSortRemap

| Method | GridSize | Mean | Error | StdDev | Ratio |
|--------|----------|-----:|------:|-------:|------:|
| **Cpp** | **32** | **11.69 μs** | **0.100 μs** | **0.093 μs** | **1.00** |
| FSharp | 32 | 22.17 μs | 0.196 μs | 0.183 μs | 1.90 |
| **Cpp** | **224** | **544.7 μs** | **5.611 μs** | **5.248 μs** | **1.00** |
| FSharp | 224 | 1,004 μs | 10.09 μs | 8.946 μs | 1.84 |
| **Cpp** | **1024** | **24,267 μs** | **202.5 μs** | **189.4 μs** | **1.00** |
| FSharp | 1024 | 33,309 μs | 226.8 μs | 201.1 μs | 1.37 |

## Summary

At production scale (GridSize=1024, ~2M triangles), the F# port falls into three performance tiers:

| Tier          | Ratio    | Functions |
|---------------|----------|-----------|
| Near parity   | 1.1–1.5x | optimizeOverdraw, optimizeVertexCache, spatialSortRemap, buildMeshlets, optimizeVertexFetch, decodeIndexBuffer |
| Moderate gap  | 1.5–2.2x | simplify, stripify, encodeIndexBuffer, decodeVertexBuffer |
| Codec gap     | 2.5x     | encodeVertexBuffer |

Compared to the old hand-rolled Stopwatch benchmarks, BenchmarkDotNet reveals tighter ratios across the board — several functions previously reported at 4–5x are actually within 1.5–2.2x with proper warmup and statistical analysis. The vertex codec encode remains the widest gap due to C++ hand-tuned SIMD byte manipulation.

**Environment:** AMD Ryzen 7 7700X (8C/16T), .NET 8.0.22, RyuJIT AVX-512F+CD+BW+DQ+VL+VBMI, Manjaro Linux.

## Running

```bash
# Run all benchmarks
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench --filter "*.Benchmark*"

# Run a single function
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench --filter *VertexCache*

# With hardware performance counters (needs perf + paranoid ≤ 1)
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench --counters

# Short run (fewer iterations, faster)
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench --job short
```
