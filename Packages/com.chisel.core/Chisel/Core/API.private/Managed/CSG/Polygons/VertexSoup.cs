﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    public sealed class VertexSoup
    {
        struct ChainedIndex
        {
            public ushort vertexIndex;
            public int nextChainIndex;
        }

        public List<float3> vertices = new List<float3>();
        public NativeArray<float3> vertexArray;
        List<ChainedIndex> chainedIndices = new List<ChainedIndex>();
        int[] hashTable = new int[(int)kHashTableSize];

        // TODO: measure the hash function and see how well it works
        const long  kHashMagicValue = (long)1099511628211ul;
        const uint  kHashTableSize  = 6529u;

        const float kCellSize       = CSGManagerPerformCSG.kDistanceEpsilon * 2;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static uint GetHash(int x, int y, int z) 
        {
            return (uint)((y ^ ((x ^ z) * kHashMagicValue)) * kHashMagicValue); 
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear(int minCapacity)
        {
            if (vertices.Capacity < minCapacity)
                vertices.Capacity = minCapacity;
            if (chainedIndices.Capacity < minCapacity)
                chainedIndices.Capacity = minCapacity;
            vertices.Clear();
            chainedIndices.Clear();
            for (int i = 0; i < kHashTableSize; i++)
                hashTable[i] = -1;
            if (vertexArray.IsCreated)
                vertexArray.Dispose();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void CleanUp()
        {
            if (vertexArray.IsCreated)
                vertexArray.Dispose();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ushort AddUnique(float3 vertex, int mx, int my, int mz)
        {
            var vertexIndex = (ushort)vertices.Count;
            vertices.Add(vertex);
            {
                var hashCode = GetHash(mx, my, mz) % kHashTableSize;
                var prevChainIndex = hashTable[hashCode];
                var newChainIndex = chainedIndices.Count;
                var newChainedIndex = new ChainedIndex() { vertexIndex = vertexIndex, nextChainIndex = prevChainIndex };
                chainedIndices.Add(newChainedIndex);
                hashTable[hashCode] = newChainIndex;
            }

            return vertexIndex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ushort Add(float3 vertex)
        {
            var mx = (int)(vertex.x / kCellSize);
            var my = (int)(vertex.y / kCellSize);
            var mz = (int)(vertex.z / kCellSize);

            for (var x = mx - 1; x <= mx + 1; x++)
            {
                for (var y = my - 1; y <= my + 1; y++)
                {
                    {
                        var chainIndex = hashTable[GetHash(x, y, mz - 1) % kHashTableSize];
                        {
                            while (chainIndex != -1)
                            {
                                var vertexIndex = chainedIndices[chainIndex].vertexIndex;
                                chainIndex = chainedIndices[chainIndex].nextChainIndex;
                                var sqrDistance = math.lengthsq(vertices[vertexIndex] - vertex);
                                if (sqrDistance < CSGManagerPerformCSG.kSqrMergeEpsilon)
                                    return vertexIndex; 
                            }
                        }
                    }

                    {
                        var chainIndex = hashTable[GetHash(x, y, mz) % kHashTableSize];
                        {
                            while (chainIndex != -1)
                            {
                                var vertexIndex = chainedIndices[chainIndex].vertexIndex;
                                chainIndex = chainedIndices[chainIndex].nextChainIndex;
                                var sqrDistance = math.lengthsq(vertices[vertexIndex] - vertex);
                                if (sqrDistance < CSGManagerPerformCSG.kSqrMergeEpsilon)
                                    return vertexIndex;
                            }
                        }
                    }

                    {
                        var chainIndex = hashTable[GetHash(x, y, mz + 1) % kHashTableSize];
                        {
                            while (chainIndex != -1)
                            {
                                var vertexIndex = chainedIndices[chainIndex].vertexIndex;
                                chainIndex = chainedIndices[chainIndex].nextChainIndex;
                                var sqrDistance = math.lengthsq(vertices[vertexIndex] - vertex);
                                if (sqrDistance < CSGManagerPerformCSG.kSqrMergeEpsilon)
                                    return vertexIndex;
                            }
                        }
                    }
                }
            }

            return AddUnique(vertex, mx, my, mz);
        }
    }
#endif
}