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
    public static unsafe class NativeListExtensions
    {
        [BurstDiscard] static void LogRangeError() { Debug.LogError("Invalid range used in RemoveRange"); }

        public static bool Contains<T>(this NativeList<T> array, T value)
            where T : struct
        {
            for (int i = 0; i < array.Length; i++)
            {
                if (EqualityComparer<T>.Default.Equals(array[i], value))
                    return true;
            }
            return false;
        }
        public static bool Contains<T>(this NativeListArray<T>.NativeList array, T value)
            where T : struct
        {
            for (int i = 0; i < array.Length; i++)
            {
                if (EqualityComparer<T>.Default.Equals(array[i], value))
                    return true;
            }
            return false;
        }

        public static void RemoveRange<T>(this NativeList<T> list, int index, int count) where T : unmanaged
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

        public static void RemoveAt<T>(this NativeList<T> list, int index) where T : unmanaged
        {
            RemoveRange(list, index, 1);
        }

        
        public static void RemoveRange<T>(this NativeListArray<T>.NativeList list, int index, int count) where T : unmanaged
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(list.m_Safety);
#endif
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

        public static void RemoveAt<T>(this NativeListArray<T>.NativeList list, int index) where T : unmanaged
        {
            RemoveRange(list, index, 1);
        }
        public static void Remove<T>(this NativeListArray<T>.NativeList list, T item) where T : unmanaged
        {
            for (int index = 0; index < list.Length; index++)
            {
                if (EqualityComparer<T>.Default.Equals(list[index], item))
                {
                    RemoveRange(list, index, 1);
                    return;
                }
            }
        }
        public static void Remove<T>(this NativeList<T> list, T item) where T : unmanaged
        {
            for (int index = 0; index < list.Length; index++)
            {
                if (EqualityComparer<T>.Default.Equals(list[index], item))
                {
                    RemoveRange(list, index, 1);
                    return;
                }
            }
        }

        public static void* GetUnsafePtr<T>(this NativeListArray<T>.NativeList list) where T : struct
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(list.m_Safety);
#endif
            return list.m_ListData->Ptr;
        }

        public static NativeList<T> ToNativeList<T>(this List<T> list, Allocator allocator) where T : unmanaged
        {
            var nativeList = new NativeList<T>(list.Count, allocator);
            for (int i = 0; i < list.Count; i++)
                nativeList.Add(list[i]);
            return nativeList;
        }
    }
}
