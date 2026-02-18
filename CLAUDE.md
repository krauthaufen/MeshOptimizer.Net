# MeshOptPort

F# port of [meshoptimizer](https://github.com/zeux/meshoptimizer) (C/C++). Line-by-line translation, not an idiomatic rewrite.

## Build & Test

```bash
dotnet build -c Release

# Correctness (needs C++ shared lib)
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests

# Benchmark (BenchmarkDotNet)
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run -c Release --project src/MeshOptPort.Tests -- --bench
# Single function: --bench --filter *VertexCache*
# Hardware counters: --bench --counters
```

## Architecture

Two projects in `src/`:

- **MeshOptPort** — the library. 19 F# files, ~11,600 lines. No dependencies. Targets net8.0.
- **MeshOptPort.Tests** — correctness tests (49 tests comparing F# vs C++ P/Invoke) and performance benchmarks.

Each C++ `.cpp` file maps to one F# `.fs` file (see README for full table). The compile order in the `.fsproj` matters — F# requires files to be ordered by dependency.

## Translation Patterns

### Pointers

All C pointers become `nativeptr<T>`. The `NPtr` module (AutoOpen in NativeUtil.fs) wraps `FSharp.NativeInterop.NativePtr`:

```
NPtr.get ptr i       // ptr[i]
NPtr.set ptr i v     // ptr[i] = v
NPtr.add ptr n       // ptr + n
NPtr.cast<T,U> ptr   // (U*)ptr
NPtr.memcpy dst src n
NPtr.memset ptr v n
```

Void pointers use `nativeint`. Conversion: `NPtr.toNI` / `NPtr.ofNI<T>`.

### Memory Management

`meshopt_Allocator` (Allocator.fs) implements `IDisposable` — tracks up to 24 native allocations, frees on dispose. Mirrors the C++ RAII pattern:

```fsharp
use allocator = new meshopt_Allocator()
let buf = allocator.allocate<uint32>(count)  // freed when allocator is disposed
```

### SIMD

Three files use `System.Runtime.Intrinsics`: VertexCodec.fs, VertexFilter.fs, MeshletCodec.fs (~480 intrinsic calls total). Runtime detection with scalar fallback:

```fsharp
if Ssse3.IsSupported then
    // SSE path using Sse2/Ssse3/Sse41 methods
elif AdvSimd.Arm64.IsSupported then
    // NEON path using AdvSimd methods
else
    // scalar fallback
```

WASM SIMD and AVX512 paths from C++ are not ported.

### Mutable State

The C++ code uses heavy mutation. The port preserves this directly with `let mutable` locals and `byref<T>` parameters for functions that update caller state:

```fsharp
let private pushEdgeFifo (fifo: uint32[]) (a: uint32) (b: uint32) (offset: byref<int>) =
    fifo.[offset * 2] <- a
    fifo.[offset * 2 + 1] <- b
    offset <- (offset + 1) &&& 15
```

### F#-Specific Constraints

- **byref in closures**: F# forbids `byref<T>` captures in closures/nested functions. Workaround: inline the logic at call sites or pass by value and return updated state.
- **Struct initialization with pointer fields**: Use `Unchecked.defaultof<T>` (22 occurrences across Simplifier, IndexGenerator, Clusterizer).
- **Keyword order**: `let inline private` (not `let private inline`).
- **Bitwise operators**: `>>>` `<<<` `^^^` `&&&` `|||` `~~~` instead of C's `>> << ^ & | ~`.

### Types (Types.fs)

All public structs use `[<Struct; StructLayout(LayoutKind.Sequential)>]` for P/Invoke compatibility. Key types: `meshopt_Meshlet`, `meshopt_Bounds`, `meshopt_Stream`, stats structs, simplify option flags.

## What's Not Ported

- WASM SIMD paths
- AVX512 paths
- TRACE/debug logging macros
- C++ template specializations (expanded to explicit typed functions)

## Performance Notes

See BENCHMARK.md for full results. At scale (2M triangles):

- Most algorithms: 1–2x of C++ (overdraw/vertex fetch at parity)
- Codec functions: 3–5x slower (C++ uses hand-tuned SIMD byte manipulation)
- Simplifier: ~1.7x (dominated by QEM math, not memory access)

The codec gap is the main optimization opportunity. The algorithmic functions benefit from .NET's efficient bounds-checked array access and RyuJIT.
