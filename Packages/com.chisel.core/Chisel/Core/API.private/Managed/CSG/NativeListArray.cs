using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using Unity.Burst;
using Unity.Burst.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
    [DebuggerDisplay("Length = {Length}, Capacity = {Capacity}, IsCreated = {IsCreated}")]
    public unsafe struct UnsafeListArray : IDisposable
    {
        [NativeDisableUnsafePtrRestriction]
        public UnsafeList** Ptr;

        public int Length;
        public int InitialListCapacity;

        public Allocator Allocator;

        public static UnsafeListArray* Create(Allocator allocator)
        {
            UnsafeListArray* arrayData = (UnsafeListArray*)UnsafeUtility.Malloc(UnsafeUtility.SizeOf<UnsafeListArray>(), UnsafeUtility.AlignOf<UnsafeListArray>(), allocator);
            UnsafeUtility.MemClear(arrayData, UnsafeUtility.SizeOf<UnsafeListArray>());

            arrayData->Allocator = allocator;
            arrayData->Length = 0;

            return arrayData;
        }

        public void Allocate(int length)
        {
            CheckAlreadyAllocated(Length);
            var bytesToMalloc = sizeof(UnsafeList*) * length;
            Ptr = (UnsafeList**)UnsafeUtility.Malloc(bytesToMalloc, UnsafeUtility.AlignOf<long>(), Allocator);
            UnsafeUtility.MemClear(Ptr, bytesToMalloc);
            Length = length;
        }

        public UnsafeList* InitializeIndex(int index, int sizeOf, int alignOf, NativeArrayOptions options = NativeArrayOptions.UninitializedMemory)
        {
            CheckIndexInRange(index, Length);
            var ptr = UnsafeList.Create(sizeOf, alignOf, InitialListCapacity, Allocator, options);
            Ptr[index] = ptr;
            return ptr;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckAlreadyAllocated(int length)
        {
            if (length > 0)
                throw new IndexOutOfRangeException($"NativeListArray already allocated.");
        }


        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckIndexInRange(int value, int length)
        {
            if (value < 0)
                throw new IndexOutOfRangeException($"Value {value} must be positive.");

            if ((uint)value >= (uint)length)
                throw new IndexOutOfRangeException($"Value {value} is out of range in NativeList of '{length}' Length.");
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        internal static void NullCheck(void* arrayData)
        {
            if (arrayData == null)
            {
                throw new Exception("UnsafeListArray has yet to be created or has been destroyed!");
            }
        }

        public static void Destroy(UnsafeListArray* arrayData)
        {
            NullCheck(arrayData);
            var allocator = arrayData->Allocator;
            arrayData->Dispose();
            UnsafeUtility.Free(arrayData, allocator);
        }

        public bool IsCreated => Ptr != null;

        internal static bool ShouldDeallocate(Allocator allocator)
        {
            // Allocator.Invalid == container is not initialized.
            // Allocator.None    == container is initialized, but container doesn't own data.
            return allocator > Allocator.None;
        }

        public void Dispose()
        {
            if (ShouldDeallocate(Allocator))
            {
                for (int i = 0; i < Length; i++)
                {
                    if (Ptr[i] != null)
                        Ptr[i]->Dispose();
                }
                UnsafeUtility.Free(Ptr, Allocator);
                Allocator = Allocator.Invalid;
            }

            Ptr = null;
            Length = 0;
        }

        [BurstCompile]
        internal unsafe struct UnsafeDisposeJob : IJob
        {
            [NativeDisableUnsafePtrRestriction]
            public UnsafeList** Ptr;
            public int Length;
            public Allocator Allocator;

            public void Execute()
            {
                for (int i = 0; i < Length; i++)
                    Ptr[i]->Dispose();
                UnsafeUtility.Free(Ptr, Allocator);
            }
        }

        public JobHandle Dispose(JobHandle inputDeps)
        {
            if (ShouldDeallocate(Allocator))
            {
                var jobHandle = new UnsafeDisposeJob { Ptr = Ptr, Length = Length, Allocator = Allocator }.Schedule(inputDeps);

                Ptr = null;
                Allocator = Allocator.Invalid;

                return jobHandle;
            }

            Ptr = null;

            return inputDeps;
        }

        public void Clear()
        {
            Length = 0;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        static private void CheckAllocator(Allocator a)
        {
            if (a <= Allocator.None)
            {
                throw new Exception("UnsafeListArray is not initialized, it must be initialized with allocator before use.");
            }
        }

        public int IndexOf<T>(T value) where T : unmanaged, IEquatable<T>
        {
            return NativeArrayExtensions.IndexOf<T, T>(Ptr, Length, value);
        }

        public bool Contains<T>(T value) where T : unmanaged, IEquatable<T>
        {
            return IndexOf(value) != -1;
        }
    }
    
    [NativeContainer]
    [StructLayout(LayoutKind.Sequential)]
    [DebuggerDisplay("Length = {Length}")]
    public unsafe struct NativeListArray<T> : IDisposable
        where T : struct
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        internal AtomicSafetyHandle m_Safety;

        [NativeSetClassTypeToNullOnSchedule]
        DisposeSentinel m_DisposeSentinel;
#endif

        [NativeDisableUnsafePtrRestriction]
        internal UnsafeListArray* m_Array;
        
        public int Length { [return: AssumeRange(0, int.MaxValue)] get { return m_Array->Length; } }
        public bool IsCreated => m_Array != null;

        public NativeListArray(Allocator allocator)
            : this(1, allocator, 2)
        {
        }

        public NativeListArray(int initialListCapacity, Allocator allocator)
            : this(initialListCapacity, allocator, 2)
        {
        }

        NativeListArray(int initialListCapacity, Allocator allocator, int disposeSentinelStackDepth)
        {
            var totalSize = UnsafeUtility.SizeOf<T>() * (long)initialListCapacity;
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            // Native allocation is only valid for Temp, Job and Persistent.
            if (allocator <= Allocator.None)
                throw new ArgumentException("Allocator must be Temp, TempJob or Persistent", nameof(allocator));
            if (initialListCapacity < 0)
                throw new ArgumentOutOfRangeException(nameof(initialListCapacity), "InitialListCapacity must be >= 0");


            CollectionHelper.CheckIsUnmanaged<T>();

            if (totalSize > int.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(initialListCapacity), $"InitialListCapacity * sizeof(T) cannot exceed {int.MaxValue} bytes");

            DisposeSentinel.Create(out m_Safety, out m_DisposeSentinel, disposeSentinelStackDepth, allocator);
#endif
            m_Array = UnsafeListArray.Create(allocator);
            m_Array->InitialListCapacity = initialListCapacity;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.SetBumpSecondaryVersionOnScheduleWrite(m_Safety, true);
#endif
        }

        public void Allocate(int length)
        {
            if (length < 0)
                throw new ArgumentOutOfRangeException(nameof(length), "Length must be >= 0");
            m_Array->Allocate(length);
        }

        [return: AssumeRange(0, int.MaxValue)]
        internal static int AssumePositive(int x)
        {
            return x;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckSufficientCapacity(int capacity, int length)
        {
            if (capacity < length)
            {
                throw new Exception($"Length {length} exceeds capacity Capacity {capacity}");
            }
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckAllocated(UnsafeList* listData)
        {
            if (listData == null)
                throw new Exception($"Expected {nameof(listData)} to be allocated.");
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckIndexInRange(int value, int length)
        {
            if (value < 0)
                throw new IndexOutOfRangeException($"Value {value} must be positive.");

            if ((uint)value >= (uint)length)
                throw new IndexOutOfRangeException($"Value {value} is out of range in NativeList of '{length}' Length.");
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckCapacityInRange(int value, int length)
        {
            if (value < 0)
                throw new ArgumentOutOfRangeException($"Value {value} must be positive.");

            if ((uint)value < (uint)length)
                throw new ArgumentOutOfRangeException($"Value {value} is out of range in NativeList of '{length}' Length.");
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckArgInRange(int value, int length)
        {
            if (value < 0)
                throw new ArgumentOutOfRangeException($"Value {value} must be positive.");

            if ((uint)value >= (uint)length)
                throw new ArgumentOutOfRangeException($"Value {value} is out of range in NativeList of '{length}' Length.");
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckArgPositive(int value)
        {
            if (value < 0)
                throw new ArgumentOutOfRangeException($"Value {value} must be positive.");
        }
            
        public NativeList this[int index]
        {
            get
            {
                CheckArgPositive(index);
                var positiveIndex = AssumePositive(index);
                CheckArgInRange(positiveIndex, m_Array->Length);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);

                var ptr = m_Array->Ptr[positiveIndex];
                if (ptr == null)
                    ptr = m_Array->InitializeIndex(index, UnsafeUtility.SizeOf<T>(), UnsafeUtility.AlignOf<T>());
                return new NativeList(m_Array->Ptr[positiveIndex], ref m_Safety);
#else
                return new NativeList(m_Array->Ptr[positiveIndex]);
#endif
            }
        }

        public void Dispose()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            DisposeSentinel.Dispose(ref m_Safety, ref m_DisposeSentinel);
#endif
            UnsafeListArray.Destroy(m_Array);
            m_Array = null;
        }
        
        [NativeContainer]
        public unsafe struct NativeList
        {
            [NativeDisableUnsafePtrRestriction]
            public UnsafeList* m_ListData;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            internal AtomicSafetyHandle m_Safety;

            public unsafe NativeList(UnsafeList* listData, ref AtomicSafetyHandle safety)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                CheckAllocated(listData);
#endif
                m_ListData = listData;
                m_Safety = safety;
            }
#else
            public unsafe NativeList(UnsafeList* listData)
            {
                m_ListData = listData;
            }
#endif

            public T this[int index]
            {
                get
                {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                    AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
                    CheckIndexInRange(index, m_ListData->Length);
#endif
                    return UnsafeUtility.ReadArrayElement<T>(m_ListData->Ptr, AssumePositive(index));
                }
                set
                {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                    AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
                    CheckIndexInRange(index, m_ListData->Length);
#endif
                    UnsafeUtility.WriteArrayElement(m_ListData->Ptr, AssumePositive(index), value);
                }
            }

            public int Length
            {
                get
                {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                    AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                    return AssumePositive(m_ListData->Length);
                }
            }

            public int Capacity
            {
                get
                {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                    AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                    return AssumePositive(m_ListData->Capacity);
                }

                set
                {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                    AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
                    CheckCapacityInRange(value, m_ListData->Length);
#endif
                    m_ListData->SetCapacity<T>(value);
                }
            }

            public void AddNoResize(T value)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
                m_ListData->AddNoResize(value);
            }

            public void AddRangeNoResize(void* ptr, int length)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
                CheckArgPositive(length);
                m_ListData->AddRangeNoResize<T>(ptr, length);
            }

            public void AddRangeNoResize(NativeList<T> list)
            {
                AddRangeNoResize(list.GetUnsafePtr(), list.Length);
            }

            public void Add(T value)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
#endif
                m_ListData->Add(value);
            }
            public void AddRange(NativeArray<T> elements)
            {
                AddRange(elements.GetUnsafeReadOnlyPtr(), elements.Length);
            }
            public unsafe void AddRange(void* elements, int count)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
                CheckArgPositive(count);
#endif
                m_ListData->AddRange<T>(elements, AssumePositive(count));
            }
            public void RemoveAtSwapBack(int index)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
#endif
                CheckArgInRange(index, Length);
                m_ListData->RemoveAtSwapBack<T>(AssumePositive(index));
            }

            public void Clear()
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
#endif
                m_ListData->Clear();
            }

            public static implicit operator NativeArray<T>(NativeList nativeList)
            {
                return nativeList.AsArray();
            }

            public NativeArray<T> AsArray()
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckGetSecondaryDataPointerAndThrow(m_Safety);
                var arraySafety = m_Safety;
                AtomicSafetyHandle.UseSecondaryVersion(ref arraySafety);
#endif
                var array = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<T>(m_ListData->Ptr, m_ListData->Length, Allocator.None);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
                NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref array, arraySafety);
#endif
                return array;
            }

            public T[] ToArray()
            {
                return AsArray().ToArray();
            }

            public NativeArray<T> ToArray(Allocator allocator)
            {
                var result = new NativeArray<T>(Length, allocator, NativeArrayOptions.UninitializedMemory);
                result.CopyFrom(this);
                return result;
            }

            public void CopyFrom(T[] array)
            {
                Resize(array.Length, NativeArrayOptions.UninitializedMemory);
                NativeArray<T> na = AsArray();
                na.CopyFrom(array);
            }

            public void Resize(int length, NativeArrayOptions options)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                m_ListData->Resize(UnsafeUtility.SizeOf<T>(), UnsafeUtility.AlignOf<T>(), length, options);
            }

            public void ResizeUninitialized(int length)
            {
                Resize(length, NativeArrayOptions.UninitializedMemory);
            }
        }
    }
}



namespace Chisel.Core.LowLevel.Unsafe
{
    public unsafe static class NativeArrayListUnsafeUtility
    {
        public static void* GetUnsafePtr<T>(this NativeListArray<T>.NativeList list) where T : struct
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(list.m_Safety);
#endif
            return list.m_ListData->Ptr;
        }

        public static unsafe void* GetUnsafeReadOnlyPtr<T>(this NativeListArray<T>.NativeList list) where T : struct
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(list.m_Safety);
#endif
            return list.m_ListData->Ptr;
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        public static AtomicSafetyHandle GetAtomicSafetyHandle<T>(ref NativeListArray<T>.NativeList list) where T : struct
        {
            return list.m_Safety;
        }
#endif

        public static void* GetInternalListDataPtrUnchecked<T>(ref NativeListArray<T>.NativeList list) where T : struct
        {
            return list.m_ListData;
        }


        public static void AddRange<T>(this NativeList<T> dst, in NativeListArray<T>.NativeList list) where T : struct
        {
            var offset = dst.Length;
            dst.ResizeUninitialized(offset + list.Length);
            for (int i = 0; i < list.Length; i++)
                dst[offset + i] = list[i];
        }
    }
}
