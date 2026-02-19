// Utility helpers for native pointer operations used throughout the port
[<AutoOpen>]
module MeshOptimizer.Net.NativeUtil

open System.Runtime.InteropServices

module NPtr =
    open FSharp.NativeInterop

    let inline get (p: nativeptr<'T>) (i: int) = NativePtr.get p i
    let inline set (p: nativeptr<'T>) (i: int) (v: 'T) = NativePtr.set p i v
    let inline add (p: nativeptr<'T>) (i: int) = NativePtr.add p i
    let inline toNI (p: nativeptr<'T>) = NativePtr.toNativeInt p
    let inline ofNI<'T when 'T : unmanaged> (n: nativeint) : nativeptr<'T> = NativePtr.ofNativeInt n
    let inline toVoid (p: nativeptr<'T>) = NativePtr.toVoidPtr p
    let inline ofVoid<'T when 'T : unmanaged> (p: voidptr) : nativeptr<'T> = NativePtr.ofVoidPtr p

    /// memset equivalent: fill byte count starting at ptr with value
    let inline memset (p: nativeptr<'T>) (value: byte) (byteCount: int) =
        let bp : nativeptr<byte> = ofNI (toNI p)
        NativePtr.initBlock bp value (uint32 byteCount)

    /// memcpy equivalent
    let inline memcpy (dst: nativeptr<'T>) (src: nativeptr<'U>) (byteCount: int) =
        System.Buffer.MemoryCopy(NativePtr.toVoidPtr src, NativePtr.toVoidPtr dst, int64 byteCount, int64 byteCount)

    /// Cast nativeptr from one type to another
    let inline cast<'T, 'U when 'T : unmanaged and 'U : unmanaged> (p: nativeptr<'T>) : nativeptr<'U> =
        ofNI (toNI p)
