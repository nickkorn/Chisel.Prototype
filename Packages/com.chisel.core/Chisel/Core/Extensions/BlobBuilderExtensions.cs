using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using UnityEngine;

namespace Chisel.Core
{
    public static class BlobBuilderExtensions
    {
        public static unsafe BlobBuilderArray<T> Construct<T>(this BlobBuilder builder, ref BlobArray<T> blobArray, NativeList<T> data) where T : unmanaged
        {
            var blobBuilderArray = builder.Allocate(ref blobArray, data.Length);
            UnsafeUtility.MemCpy(blobBuilderArray.GetUnsafePtr(), data.GetUnsafeReadOnlyPtr(), blobBuilderArray.Length * sizeof(T));
            return blobBuilderArray;
        }
    }
}
