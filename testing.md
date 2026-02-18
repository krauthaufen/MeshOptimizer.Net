# MeshOptPort Testing Plan

Test the F# port against the C++ reference implementation by running both on the same mesh data and comparing results.

## Approach

1. Build C++ meshoptimizer as `libmeshoptimizer.so`
2. Create F# test project with P/Invoke bindings to the C++ library
3. Load test mesh (pirate.obj from meshoptimizer demo)
4. Call both implementations with identical inputs, compare outputs

## Setup

### Build C++ shared library

```bash
cd /tmp/meshoptimizer
mkdir -p build && cd build
cmake .. -DMESHOPT_BUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
# → /tmp/meshoptimizer/build/libmeshoptimizer.so
```

### Create test project

```bash
cd ~/MeshOptPort
dotnet new console -lang F# -n MeshOptPort.Tests
dotnet add MeshOptPort.Tests reference MeshOptPort/MeshOptPort.fsproj
```

### Test files

| File | Purpose |
|------|---------|
| `Native.fs` | P/Invoke bindings for libmeshoptimizer.so |
| `ObjLoader.fs` | Minimal OBJ parser |
| `Program.fs` | Test harness — calls C++ and F#, compares |

### Test mesh

`/tmp/meshoptimizer/demo/pirate.obj` — ~1800 vertices, real-world model.

Vertex layout: `px, py, pz, nx, ny, nz, tx, ty` (32 bytes, 8 floats)

## Functions to Test

### Exact match expected

**Quantization:**
- `meshopt_quantizeHalf` / `meshopt_dequantizeHalf`
- `meshopt_quantizeUnorm` / `meshopt_quantizeSnorm`

**Codec round-trips (encode one side → decode other):**
- `meshopt_encodeIndexBuffer` → `meshopt_decodeIndexBuffer`
- `meshopt_encodeVertexBuffer` → `meshopt_decodeVertexBuffer`

**Optimization (deterministic, same input → same output):**
- `meshopt_generateVertexRemap` — compare remap tables
- `meshopt_optimizeVertexCache` — compare output indices
- `meshopt_optimizeOverdraw` — compare output indices
- `meshopt_optimizeVertexFetch` — compare output + remap
- `meshopt_stripify` / `meshopt_unstripify`
- `meshopt_spatialSortRemap`

### Float epsilon comparison

**Analysis stats:**
- `meshopt_analyzeVertexCache` — ACMR, ATVR
- `meshopt_analyzeVertexFetch` — overfetch

**Bounds:**
- `meshopt_computeMeshletBounds` — center, radius, cone

### Structural comparison

**Simplification (index count must match, error within epsilon):**
- `meshopt_simplify`
- `meshopt_simplifySloppy`

**Meshlets (count must match, total triangles must match):**
- `meshopt_buildMeshlets`

## Comparison Strategy

```
For each test:
  1. Allocate identical input buffers
  2. Call C++ function via P/Invoke
  3. Call F# function directly
  4. Compare outputs:
     - Byte arrays: memcmp
     - Float values: |a - b| < 1e-6
     - Counts: exact match
  5. Print PASS/FAIL with details
```

## Running

```bash
LD_LIBRARY_PATH=/tmp/meshoptimizer/build dotnet run --project MeshOptPort.Tests
```
