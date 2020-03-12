using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using UnityEngine;

namespace Chisel.Core
{
    public static class NativeListExtensions
    {
        [BurstDiscard] static void LogRangeError() { Debug.LogError("Invalid range used in RemoveRange"); }

        public unsafe static void RemoveRange<T>(this NativeList<T> list, int index, int count) where T : unmanaged
        {
            if (count == 0)
                return;
            if (index < 0 || index + count > list.Length)
            {
                LogRangeError();
                return;
            }
            if (index == 0 && count == list.Length)
            {
                list.Clear();
                return;
            }

            if (index + count < list.Length)
            {
                var listPtr = (T*)list.GetUnsafePtr();
                int size = sizeof(T);
                UnsafeUtility.MemMove(listPtr + index, listPtr + (index + count), (list.Length - (index + count)) * size);
            }
            list.Resize(list.Length - count, NativeArrayOptions.ClearMemory);
        }

        public unsafe static void RemoveAt<T>(this NativeList<T> list, int index) where T : unmanaged
        {
            RemoveRange(list, index, 1);
        }

        public unsafe static NativeList<T> ToNativeList<T>(this List<T> list, Allocator allocator) where T : unmanaged
        {
            var nativeList = new NativeList<T>(list.Count, allocator);
            for (int i = 0; i < list.Count; i++)
                nativeList.Add(list[i]);
            return nativeList;
        }
    }
}
