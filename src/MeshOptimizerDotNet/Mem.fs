namespace MeshOptimizerDotNet

open System
open System.Threading
open System.Runtime.InteropServices
open System.Runtime.CompilerServices
open Microsoft.FSharp.NativeInterop

#nowarn "9"

/// Abstract base class representing a block of memory. This is the core abstraction used by the MeshOptimizer functions.
[<AbstractClass; AllowNullLiteral>]
type Mem() =
    abstract member VoidPointer : nativeint
    abstract member SizeInBytes : uint64
    abstract member Acquire : unit -> unit
    abstract member Release : unit -> unit


/// Generic version of Mem that provides typed access to the memory. The type parameter 'a must be unmanaged to ensure it can be used in native code.
and [<AbstractClass; AllowNullLiteral>]
    Mem<'a when 'a : unmanaged>() =
    inherit Mem()
    
    static let nullPtr =
        { new Mem<'a>() with
            member _.Pointer = NativePtr.ofNativeInt 0n
            member _.Length = 0
            member _.Acquire () = ()
            member _.Release () = ()
        }
    
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

    member x.CopyTo(dst : 'a[], [<DefaultParameterValue(0); OptionalArgument>] dstOffset : int) =
        if dstOffset < 0 then raise <| ArgumentOutOfRangeException(nameof dstOffset, "Destination offset must be non-negative")
        if dstOffset + x.Length > dst.Length then raise <| ArgumentOutOfRangeException("Destination offset and memory length exceed destination array length")
        
        x.Acquire()
        try
            let src = System.Span<'a>(NativePtr.toVoidPtr x.Pointer, x.Length)
            let dst = System.Span<'a>(dst, dstOffset, x.Length)
            src.CopyTo dst
        finally
            x.Release()

    member x.CopyTo(dst : Mem<'a>) =
        if dst.Length < x.Length then raise <| ArgumentOutOfRangeException("Destination memory is too small to hold source memory")
        
        x.Acquire()
        dst.Acquire()
        try
            let src = System.Span<'a>(NativePtr.toVoidPtr x.Pointer, x.Length)
            let dst = System.Span<'a>(NativePtr.toVoidPtr dst.Pointer, dst.Length)
            src.CopyTo dst
        finally
            dst.Release()
            x.Release()

    member x.CopyTo(dst : nativeptr<'a>) =
        x.Acquire()
        try
            let src = System.Span<'a>(NativePtr.toVoidPtr x.Pointer, x.Length)
            let dst = System.Span<'a>(NativePtr.toVoidPtr dst, x.Length)
            src.CopyTo dst
        finally
            x.Release()
            
    member x.CopyTo(dst : System.Memory<'a>) =
        use h = dst.Pin()
        x.CopyTo(NativePtr.ofVoidPtr h.Pointer)

    member x.CopyTo(dst : System.Span<'a>) =
        x.Acquire()
        try
            let src = System.Span<'a>(NativePtr.toVoidPtr x.Pointer, x.Length)
            src.CopyTo dst
        finally
            x.Release()

    member x.ToArray() =
        let arr = Array.zeroCreate<'a> x.Length
        x.CopyTo arr
        arr
    
    static member Null = nullPtr
    
    static member Create(arr : 'a[]) =
        if isNull arr then
            Mem<'a>.Null
        else
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

    static member Create (mem : System.Memory<'a>) =
        if mem.IsEmpty then
            Mem<'a>.Null
        else
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

    static member Create(ptr : nativeptr<'a>, length : int) =
        { new Mem<'a>() with
            member _.Pointer = ptr
            member _.Length = length
            member _.Acquire () = ()
            member _.Release () = ()
        }


[<AbstractClass; Sealed>]
type MemExtensions private() =
    [<Extension>]
    static member Cast<'a when 'a : unmanaged>(x : Mem) : Mem<'a> =
        if isNull x then
            Mem<'a>.Null
        else
            let cnt = int (x.SizeInBytes / uint64 sizeof<'a>)
            { new Mem<'a>() with
                member _.Pointer = NativePtr.ofNativeInt x.VoidPointer
                member _.Length = cnt
                member _.Acquire () = x.Acquire()
                member _.Release () = x.Release()
            }

    [<Extension>]
    static member Pin(this : Mem<'a>, action : System.Action<nativeptr<'a>>) : unit =
        if isNull this then
            action.Invoke (NativePtr.ofNativeInt 0n)
        else
            this.Acquire()
            try action.Invoke this.Pointer
            finally this.Release()


module Mem =
    
    /// Creates a Mem instance from a managed array. The memory will be pinned while acquired.
    let inline ofArray (arr : 'a[]) = Mem<'a>.Create arr

    /// Creates a Mem instance from an unmanaged pointer. The caller is responsible for ensuring the memory is valid and properly released.
    let inline ofNativePtr (ptr : nativeptr<'a>) (length : int) = Mem<'a>.Create(ptr, length)

    /// Creates a Mem instance from a Memory<'a>. The memory will be pinned while acquired.
    let inline ofMemory (mem : Memory<'a>) = Mem<'a>.Create mem
    
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
    let inline cast<'a when 'a : unmanaged> (mem : Mem) = mem.Cast<'a>()

    /// Pins the memory and executes the given action with a pointer to the memory. The memory will be released after the action completes, even if an exception occurs.
    let inline pin (mem : Mem<'a>) ([<InlineIfLambda>] action : nativeptr<'a> -> 'b) : 'b =
        if isNull mem then
            action (NativePtr.ofNativeInt 0n)
        else
            mem.Acquire()
            try action mem.Pointer
            finally mem.Release()
