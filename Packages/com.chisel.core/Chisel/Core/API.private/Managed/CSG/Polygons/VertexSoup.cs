using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
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
        public unsafe static ushort AddNoResize(ref NativeArray<ushort> hashTable, ref NativeList<VertexSoup.ChainedIndex> chainedIndices, ref NativeList<float3> vertices, float3 vertex)
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
                        var nextChainIndex  = ((int)chainedIndices[chainIndex].nextChainIndex) - 1;
                        var sqrDistance     = math.lengthsq(chainedIndices[chainIndex].vertex - vertex);
                        if (sqrDistance < closestDistance)
                        {
                            closestVertexIndex = chainedIndices[chainIndex].vertexIndex;
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
                var vertexIndex = (ushort)vertices.Length;
                vertices.AddNoResize(vertex);

                var hashCode        = GetHash(centerIndex);
                var prevChainIndex  = hashTable[hashCode];
                var newChainIndex   = chainedIndices.Length;
                var newChainedIndex = new VertexSoup.ChainedIndex() { vertexIndex = vertexIndex, nextChainIndex = (ushort)prevChainIndex, vertex = vertex };
                chainedIndices.AddNoResize(newChainedIndex);
                hashTable[(int)hashCode] = (ushort)(newChainIndex + 1);
                return vertexIndex;
            }
        }
    }

    public struct VertexSoupReader
    {
        [ReadOnly] public NativeList<float3> vertices;
    }

    // TODO: make this safely writable in parallel / maybe check out NativeMultiHashMap? 
    //       write multiple vertices -> combine? but what about indices? seperate vertex generation from getting indices?
    public struct VertexSoup : IDisposable
    {
        public const int        kMaxVertexCount = 65000;
        internal const uint     kHashTableSize  = 509u;
        internal const float    kCellSize       = CSGManagerPerformCSG.kDistanceEpsilon * 2;

        internal struct ChainedIndex
        {
            public ushort   vertexIndex;
            public ushort   nextChainIndex;
            public float3   vertex;
        }

        [NativeDisableContainerSafetyRestriction] NativeList<float3>        vertices;
        [NativeDisableContainerSafetyRestriction] NativeList<ChainedIndex>  chainedIndices;
        [NativeDisableContainerSafetyRestriction] NativeArray<ushort>       hashTable;

        public VertexSoupReader AsReader() { return new VertexSoupReader() { vertices = vertices }; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Initialize(int minCapacity, Allocator allocator = Allocator.Persistent)
        {
            if (hashTable.IsCreated) hashTable.Dispose();
            hashTable = new NativeArray<ushort>((ushort)(kHashTableSize + 1), allocator, NativeArrayOptions.ClearMemory);
            
            if (chainedIndices.IsCreated) chainedIndices.Dispose();
            chainedIndices = new NativeList<ChainedIndex>(minCapacity, allocator);

            if (vertices.IsCreated) vertices.Dispose();
            vertices = new NativeList<float3>(minCapacity, allocator);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Initialize(VertexSoup otherSoup, Allocator allocator = Allocator.Persistent)
        {
            if (hashTable.IsCreated) hashTable.Dispose();
            hashTable = new NativeArray<ushort>((ushort)(kHashTableSize + 1), allocator, NativeArrayOptions.ClearMemory);

            if (chainedIndices.IsCreated) chainedIndices.Dispose();
            chainedIndices = new NativeList<ChainedIndex>(otherSoup.chainedIndices.Length, allocator);
            chainedIndices.AddRangeNoResize(otherSoup.chainedIndices);

            if (vertices.IsCreated) vertices.Dispose();
            vertices = new NativeList<float3>(otherSoup.vertices.Length, allocator);
            vertices.AddRangeNoResize(otherSoup.vertices);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Initialize(ref BlobArray<float3> uniqueVertices, Allocator allocator = Allocator.Persistent)
        {
            if (hashTable.IsCreated) hashTable.Dispose();
            hashTable = new NativeArray<ushort>((ushort)(kHashTableSize + 1), allocator, NativeArrayOptions.ClearMemory);

            if (chainedIndices.IsCreated) chainedIndices.Dispose();
            chainedIndices = new NativeList<ChainedIndex>(uniqueVertices.Length, allocator);

            if (vertices.IsCreated) vertices.Dispose();
            vertices = new NativeList<float3>(uniqueVertices.Length, allocator);


            // Add Unique vertex
            for (int i=0;i<uniqueVertices.Length;i++)
            {
                var vertex      = uniqueVertices[i];
                var vertexIndex = (ushort)vertices.Length;
                vertices.AddNoResize(vertex);

                var centerIndex         = new int3((int)(vertex.x / kCellSize), (int)(vertex.y / kCellSize), (int)(vertex.z / kCellSize));
                var hashCode            = VertexSoupUtility.GetHash(centerIndex);
                var prevChainIndex      = hashTable[hashCode];
                var newChainIndex       = chainedIndices.Length;
                var newChainedIndex     = new ChainedIndex() { vertexIndex = vertexIndex, nextChainIndex = (ushort)prevChainIndex, vertex = vertex };
                chainedIndices.AddNoResize(newChainedIndex);
                hashTable[(int)hashCode] = (ushort)(newChainIndex + 1);
            }
        }

        // ensure we have at least this many extra vertices in capacity
        public void Reserve(int extraIndices)
        {
            var requiredVertices = extraIndices + vertices.Length;
            if (vertices.Capacity < requiredVertices)
                vertices.Capacity = requiredVertices;

            var requiredIndices = extraIndices + chainedIndices.Length;
            if (chainedIndices.Capacity < requiredIndices)
                chainedIndices.Capacity = requiredIndices;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose()
        {
            if (chainedIndices.IsCreated)
                chainedIndices.Dispose();
            if (vertices.IsCreated)
                vertices.Dispose();
            if (hashTable.IsCreated)
                hashTable.Dispose();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ushort AddNoResize(float3 vertex)
        {
            return VertexSoupUtility.AddNoResize(ref hashTable, ref chainedIndices, ref vertices, vertex);
        }
    }
#endif
}
