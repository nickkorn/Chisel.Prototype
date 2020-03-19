using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION

    internal static class VertexSoupUtility
    {
        // TODO: measure the hash function and see how well it works
        const long kHashMagicValue = (long)1099511628211ul;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetHash(int3 index)
        {
            var hashCode = (uint)((index.y ^ ((index.x ^ index.z) * kHashMagicValue)) * kHashMagicValue);
            var hashIndex = ((int)(hashCode % VertexSoup.kHashTableSize)) + 1;
            return hashIndex;
        }
        
        // Add but make the assumption we're not growing any list
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static ushort AddNoResize(ushort* hashTable, UnsafeList* chainedIndices, UnsafeList* vertices, float3 vertex)
        {
            var centerIndex = new int3((int)(vertex.x / VertexSoup.kCellSize), (int)(vertex.y / VertexSoup.kCellSize), (int)(vertex.z / VertexSoup.kCellSize));
            var offsets = stackalloc int3[]
            {
                new int3(-1, -1, -1), new int3(-1, -1,  0), new int3(-1, -1, +1),
                new int3(-1,  0, -1), new int3(-1,  0,  0), new int3(-1,  0, +1),                
                new int3(-1, +1, -1), new int3(-1, +1,  0), new int3(-1, +1, +1),

                new int3( 0, -1, -1), new int3( 0, -1,  0), new int3( 0, -1, +1),
                new int3( 0,  0, -1), new int3( 0,  0,  0), new int3( 0,  0, +1),                
                new int3( 0, +1, -1), new int3( 0, +1,  0), new int3( 0, +1, +1),

                new int3(+1, -1, -1), new int3(+1, -1,  0), new int3(+1, -1, +1),
                new int3(+1,  0, -1), new int3(+1,  0,  0), new int3(+1,  0, +1),                
                new int3(+1, +1, -1), new int3(+1, +1,  0), new int3(+1, +1, +1)
            };
            for (int i = 0; i < 3 * 3 * 3; i++)
            {
                var index = centerIndex + offsets[i];
                var chainIndex = ((int)hashTable[GetHash(index)]) - 1;
                {
                    ushort closestVertexIndex = ushort.MaxValue;
                    float closestDistance = CSGManagerPerformCSG.kSqrMergeEpsilon;
                    while (chainIndex != -1)
                    {
                        var element         = UnsafeUtility.ReadArrayElement<VertexSoup.ChainedIndex>(chainedIndices->Ptr, chainIndex);
                        var nextChainIndex  = ((int)element.nextChainIndex) -1;
                        var sqrDistance     = math.lengthsq(element.vertex - vertex);
                        if (sqrDistance < closestDistance)
                        {
                            closestVertexIndex = element.vertexIndex;
                            closestDistance = sqrDistance;
                        }
                        chainIndex = nextChainIndex;
                    }
                    if (closestVertexIndex != ushort.MaxValue)
                        return closestVertexIndex;
                }
            }

            // Add Unique vertex
            {
                var vertexIndex = (ushort)vertices->Length;
                vertices->AddNoResize(vertex);

                var hashCode        = GetHash(centerIndex);
                var prevChainIndex  = hashTable[hashCode];
                var newChainIndex   = chainedIndices->Length;
                var newChainedIndex = new VertexSoup.ChainedIndex() { vertexIndex = vertexIndex, nextChainIndex = (ushort)prevChainIndex, vertex = vertex };
                chainedIndices->AddNoResize(newChainedIndex);
                hashTable[(int)hashCode] = (ushort)(newChainIndex + 1);
                return vertexIndex;
            }
        }
    }


    // TODO: make this safely writable in parallel / maybe check out NativeMultiHashMap? 
    //       write multiple vertices -> combine? but what about indices? seperate vertex generation from getting indices?
    [NativeContainer]
    public unsafe struct VertexSoup : IDisposable
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        internal AtomicSafetyHandle m_Safety;

        [NativeSetClassTypeToNullOnSchedule]
        DisposeSentinel m_DisposeSentinel;
#endif
        [NativeDisableUnsafePtrRestriction] internal UnsafeList*    m_Vertices;
        [NativeDisableUnsafePtrRestriction] internal UnsafeList*    m_ChainedIndices;
        [NativeDisableUnsafePtrRestriction] internal void*          m_HashTable;

        public const int        kMaxVertexCount = 65000;
        internal const uint     kHashTableSize  = 509u;
        internal const float    kCellSize       = CSGManagerPerformCSG.kDistanceEpsilon * 2;

        internal struct ChainedIndex
        {
            public ushort   vertexIndex;
            public ushort   nextChainIndex;
            public float3   vertex;
        }

        // Keep track of where the memory for this was allocated
        Allocator m_AllocatorLabel;

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckIndexInRange(int value, int length)
        {
            if (value < 0)
                throw new IndexOutOfRangeException($"Value {value} must be positive.");

            if ((uint)value >= (uint)length)
                throw new IndexOutOfRangeException($"Value {value} is out of range in NativeList of '{length}' Length.");
        }

        /// <summary>
        /// Retrieve a member of the contaner by index.
        /// </summary>
        /// <param name="index">The zero-based index into the list.</param>
        /// <value>The list item at the specified index.</value>
        /// <exception cref="IndexOutOfRangeException">Thrown if index is negative or >= to <see cref="Length"/>.</exception>
        public float3 this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
                CheckIndexInRange(index, m_Vertices->Length);
#endif
                return UnsafeUtility.ReadArrayElement<float3>(m_Vertices->Ptr, index);
            }
            set
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
                CheckIndexInRange(index, m_Vertices->Length);
#endif
                UnsafeUtility.WriteArrayElement(m_Vertices->Ptr, index, value);
            }
        }

        public float3* GetUnsafeReadOnlyPtr()
        {
            return (float3*)(m_Vertices->Ptr);
        }

        public int VertexCount
        {
            get
            {
                return m_Vertices->Length;
            }
        }

        /// <summary>
        /// The current number of items in the list.
        /// </summary>
        /// <value>The item count.</value>
        public int Length
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return m_Vertices->Length;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public VertexSoup(int minCapacity, Allocator allocator = Allocator.Persistent)
            : this(minCapacity, minCapacity, allocator)
        {
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        VertexSoup(int vertexCapacity, int chainedIndicesCapacity, Allocator allocator = Allocator.Persistent)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            DisposeSentinel.Create(out m_Safety, out m_DisposeSentinel, 0, allocator);
#endif
            m_AllocatorLabel = allocator;
            var hashTableMemSize = (ushort)(kHashTableSize + 1) * UnsafeUtility.SizeOf<ushort>();
            m_HashTable = UnsafeUtility.Malloc(hashTableMemSize, UnsafeUtility.AlignOf<ushort>(), m_AllocatorLabel);
            UnsafeUtility.MemClear(m_HashTable, hashTableMemSize);

            m_Vertices          = UnsafeList.Create(UnsafeUtility.SizeOf<float3>(), UnsafeUtility.AlignOf<float3>(), vertexCapacity, allocator);
            m_ChainedIndices    = UnsafeList.Create(UnsafeUtility.SizeOf<ChainedIndex>(), UnsafeUtility.AlignOf<ChainedIndex>(), chainedIndicesCapacity, allocator);
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckAllocated(VertexSoup otherSoup)
        {
            if (otherSoup.IsCreated)
                throw new ArgumentException($"Value {otherSoup} is not allocated.");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public VertexSoup(VertexSoup otherSoup, Allocator allocator = Allocator.Persistent)
            : this((otherSoup.m_Vertices != null) ? otherSoup.m_Vertices->Length : 1, (otherSoup.m_ChainedIndices != null) ? otherSoup.m_ChainedIndices->Length : 1, allocator)
        {
            CheckAllocated(otherSoup);
            m_ChainedIndices->AddRangeNoResize<ChainedIndex>(*otherSoup.m_ChainedIndices);
            m_Vertices->AddRangeNoResize<float3>(*otherSoup.m_Vertices);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public VertexSoup(ref BlobArray<float3> uniqueVertices, Allocator allocator = Allocator.Persistent)
            : this(uniqueVertices.Length, allocator)
        {
            // Add Unique vertex
            for (int i = 0; i < uniqueVertices.Length; i++)
            {
                var vertex = uniqueVertices[i];
                var vertexIndex = (ushort)m_Vertices->Length;
                m_Vertices->AddNoResize(vertex);

                var centerIndex = new int3((int)(vertex.x / kCellSize), (int)(vertex.y / kCellSize), (int)(vertex.z / kCellSize));
                var hashCode = VertexSoupUtility.GetHash(centerIndex);
                var prevChainIndex = ((ushort*)m_HashTable)[hashCode];
                var newChainIndex = m_ChainedIndices->Length;
                var newChainedIndex = new ChainedIndex() { vertexIndex = vertexIndex, nextChainIndex = (ushort)prevChainIndex, vertex = vertex };
                m_ChainedIndices->AddNoResize(newChainedIndex);
                ((ushort*)m_HashTable)[(int)hashCode] = (ushort)(newChainIndex + 1);
            }
        }

        // ensure we have at least this many extra vertices in capacity
        public void Reserve(int extraIndices)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
            var requiredVertices = extraIndices + m_Vertices->Length;
            if (m_Vertices->Capacity < requiredVertices)
                m_Vertices->SetCapacity<float3>(requiredVertices);

            var requiredIndices = extraIndices + m_ChainedIndices->Length;
            if (m_ChainedIndices->Capacity < requiredIndices)
                m_ChainedIndices->SetCapacity<ChainedIndex>(requiredIndices);
        }

        public bool IsCreated => m_Vertices != null && m_ChainedIndices != null && m_ChainedIndices != null;
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            DisposeSentinel.Dispose(ref m_Safety, ref m_DisposeSentinel);
#endif
            UnsafeList.Destroy(m_Vertices);
            m_Vertices = null;
            UnsafeList.Destroy(m_ChainedIndices);
            m_ChainedIndices = null;
            UnsafeUtility.Free(m_HashTable, m_AllocatorLabel);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ushort AddNoResize(float3 vertex)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
            return VertexSoupUtility.AddNoResize((ushort*)m_HashTable, m_ChainedIndices, m_Vertices, vertex);
        }
    }
#endif
}
