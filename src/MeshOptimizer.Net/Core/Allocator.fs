// This file is part of MeshOptimizer.Net; see meshoptimizer.h for version/license details
module MeshOptimizer.Net.Allocator

#nowarn "9"

open System
open System.Runtime.InteropServices

/// Allocation function signature: takes size in bytes, returns pointer
type AllocateFunc = delegate of unativeint -> nativeint

/// Deallocation function signature: takes pointer
type DeallocateFunc = delegate of nativeint -> unit

let mutable private allocateFunc: AllocateFunc =
    AllocateFunc(fun size -> Marshal.AllocHGlobal(nativeint size))

let mutable private deallocateFunc: DeallocateFunc =
    DeallocateFunc(fun ptr -> Marshal.FreeHGlobal(ptr))

/// Set allocation callbacks
/// These callbacks will be used instead of the default for all temporary allocations in the library.
let meshopt_setAllocator (allocate: AllocateFunc) (deallocate: DeallocateFunc) =
    allocateFunc <- allocate
    deallocateFunc <- deallocate

/// Allocate memory of the given size in bytes
let allocate (size: unativeint) : nativeint =
    allocateFunc.Invoke(size)

/// Deallocate memory previously allocated with allocate
let deallocate (ptr: nativeint) : unit =
    deallocateFunc.Invoke(ptr)

/// RAII-style allocator that tracks allocations and frees them on dispose.
/// Mirrors the C++ meshopt_Allocator class.
[<Sealed>]
type meshopt_Allocator() =
    let blocks = Array.zeroCreate<nativeint> 24
    let mutable count = 0

    /// Allocate an array of 'count' elements of the given size
    member _.allocate<'T when 'T : unmanaged>(size: int) : nativeptr<'T> =
        assert (count < blocks.Length)
        let byteSize =
            if uint64 size > uint64 UInt64.MaxValue / uint64 sizeof<'T> then
                unativeint UInt64.MaxValue
            else
                unativeint (size * sizeof<'T>)
        let ptr = allocate byteSize
        blocks.[count] <- ptr
        count <- count + 1
        NativeInterop.NativePtr.ofNativeInt<'T> ptr

    /// Deallocate a previously allocated pointer (must be in LIFO order)
    member _.deallocate(ptr: nativeint) : unit =
        assert (count > 0 && blocks.[count - 1] = ptr)
        deallocate ptr
        count <- count - 1

    interface IDisposable with
        member _.Dispose() =
            for i = count - 1 downto 0 do
                deallocate blocks.[i]
            count <- 0
