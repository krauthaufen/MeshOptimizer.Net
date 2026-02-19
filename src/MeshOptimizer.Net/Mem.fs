namespace MeshOptimizer.Net

open System
open System.Threading
open System.Runtime.InteropServices
open Microsoft.FSharp.NativeInterop

#nowarn "9"

/// Abstract base class representing a block of memory. This is the core abstraction used by the MeshOptimizer functions.
[<AbstractClass>]
type Mem() =
    abstract member VoidPointer : nativeint
    abstract member SizeInBytes : uint64
    abstract member Acquire : unit -> unit
    abstract member Release : unit -> unit

/// Generic version of Mem that provides typed access to the memory. The type parameter 'a must be unmanaged to ensure it can be used in native code.
[<AbstractClass>]
type Mem<'a when 'a : unmanaged>() =
    inherit Mem()
    abstract member Pointer : nativeptr<'a>
    abstract member Length : int

    override x.VoidPointer = NativePtr.toNativeInt x.Pointer
    override x.SizeInBytes = uint64 x.Length * uint64 sizeof<'a>

    /// Creates a sub-region of the memory. The caller must ensure the offset and count are within bounds.
    member x.Sub(offset : int, count : int) =
        if offset < 0 then raise <| ArgumentOutOfRangeException(nameof offset, "Offset must be non-negative")
        if count < 0 then raise <| ArgumentOutOfRangeException(nameof count, "Count must be non-negative")
        if offset + count > x.Length then raise <| ArgumentOutOfRangeException("Offset and count exceed memory length")
        { new Mem<'a>() with
            member _.Pointer = NativePtr.add x.Pointer offset
            member _.Length = count
            member _.Acquire () = x.Acquire()
            member _.Release () = x.Release()
        }
    
    /// Creates a slice of the memory from min to max (inclusive). The caller must ensure the indices are within bounds.
    member x.GetSlice(min : option<int>, max : option<int>) : Mem<'a> =
        let start = defaultArg min 0
        let len =
            match max with
            | Some max -> 1 + max - start
            | None -> x.Length - start
        x.Sub(start, len)

module Mem =
    /// Creates a Mem instance from a managed array. The memory will be pinned while acquired.
    let ofArray (arr : 'a[]) : Mem<'a> =
        let gc = ref Unchecked.defaultof<GCHandle>
        let cnt = ref 0
        { new Mem<'a>() with
            member _.Pointer =
                lock gc (fun () ->
                    if cnt.Value <= 0 then raise <| InvalidOperationException("Memory not acquired")
                    gc.Value.AddrOfPinnedObject() |> NativePtr.ofNativeInt
                )
                
            member _.Length = arr.Length
            member _.Acquire () =
                lock gc (fun () ->
                    if cnt.Value = 0 then
                        gc.Value <- GCHandle.Alloc(arr, GCHandleType.Pinned)
                    cnt.Value <- cnt.Value + 1
                )
            member _.Release () =
                lock gc (fun () ->
                    if cnt.Value <= 0 then raise <| InvalidOperationException("Memory not acquired")
                    cnt.Value <- cnt.Value - 1
                    if cnt.Value = 0 then
                        gc.Value.Free()
                        gc.Value <- Unchecked.defaultof<GCHandle>
                )
        }

    /// Creates a Mem instance from an unmanaged pointer. The caller is responsible for ensuring the memory is valid and properly released.
    let ofNativePtr (ptr : nativeptr<'a>) (length : int) : Mem<'a> =
        { new Mem<'a>() with
            member _.Pointer = ptr
            member _.Length = length
            member _.Acquire () = ()
            member _.Release () = ()
        }

    /// Creates a Mem instance from a Memory<'a>. The memory will be pinned while acquired.
    let ofMemory (mem : Memory<'a>) : Mem<'a> =
        let handle = ref Unchecked.defaultof<Buffers.MemoryHandle>
        let cnt = ref 0
        { new Mem<'a>() with
            member _.Pointer =
                lock handle (fun () ->
                    if cnt.Value <= 0 then raise <| InvalidOperationException("Memory not acquired")
                    handle.Value.Pointer |> NativePtr.ofVoidPtr
                )
                
            member _.Length = mem.Length
            member _.Acquire () =
                lock handle (fun () ->
                    if cnt.Value = 0 then
                        handle.Value <- mem.Pin()
                    cnt.Value <- cnt.Value + 1
                )
            member _.Release () =
                lock handle (fun () ->
                    if cnt.Value <= 0 then raise <| InvalidOperationException("Memory not acquired")
                    cnt.Value <- cnt.Value - 1
                    if cnt.Value = 0 then
                        handle.Value.Dispose()
                        handle.Value <- Unchecked.defaultof<_>
                )
        }
    
    /// Creates a Mem instance that allocates unmanaged memory. The memory will be allocated on the first Acquire call and freed on the last Release call. The caller must ensure the memory is properly released.
    let temp<'a when 'a : unmanaged> (count : int) : Mem<'a> =
        let ptr = ref 0n
        let cnt = ref 0
        { new Mem<'a>() with
            member x.Length = count
            member x.Pointer =
                if cnt.Value <= 0 then raise <| InvalidOperationException("Memory not acquired")
                NativePtr.ofNativeInt ptr.Value
            member x.Acquire() =
                lock ptr (fun () ->
                    if cnt.Value = 0 then
                        let size = nativeint count * nativeint sizeof<'a>
                        ptr.Value <- Marshal.AllocHGlobal(size)
                    cnt.Value <- cnt.Value + 1
                )
            member x.Release() =
                lock ptr (fun () ->
                    if cnt.Value <= 0 then raise <| InvalidOperationException("Memory not acquired")
                    cnt.Value <- cnt.Value - 1
                    if cnt.Value = 0 then
                        Marshal.FreeHGlobal(ptr.Value)
                        ptr.Value <- 0n
                )
                
        }
    
    /// Casts a Mem instance to a different element type. The caller must ensure the memory layout is compatible.
    let cast<'a when 'a : unmanaged> (mem : Mem) : Mem<'a> =
        let cnt = int (mem.SizeInBytes / uint64 sizeof<'a>)
        { new Mem<'a>() with
            member _.Pointer = NativePtr.ofNativeInt mem.VoidPointer
            member _.Length = cnt
            member _.Acquire () = mem.Acquire()
            member _.Release () = mem.Release()
        }

    
    let ofRef (r : ref<'a>) =
        { new Mem<'a>() with
            member _.Pointer = &&r.contents
            member _.Length = 1
            member _.Acquire () = ()
            member _.Release () = ()
        }
    
    /// Pins the memory and executes the given action with a pointer to the memory. The memory will be released after the action completes, even if an exception occurs.
    let inline pin (mem : Mem<'a>) ([<InlineIfLambda>] action : nativeptr<'a> -> 'b) : 'b =
        mem.Acquire()
        try action mem.Pointer
        finally mem.Release()
