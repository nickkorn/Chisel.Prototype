using System;
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
    public struct VertexSoup
    {
        public const int    kMaxVertexCount = 65000;
        const uint          kHashTableSize  = 6529u;

        struct ChainedIndex
        {
            public ushort   vertexIndex;
            public int      nextChainIndex;
        }

        public NativeList<float3> vertices;

        NativeList<ChainedIndex> chainedIndices;
        NativeArray<int> hashTable;

        // TODO: measure the hash function and see how well it works
        const long  kHashMagicValue = (long)1099511628211ul;

        const float kCellSize       = CSGManagerPerformCSG.kDistanceEpsilon * 2;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int GetHash(int x, int y, int z) 
        {
            var hashCode = //math.hash(new int3(x, y, z));// 
                            (uint)((y ^ ((x ^ z) * kHashMagicValue)) * kHashMagicValue);
            var hashIndex = ((int)(hashCode % kHashTableSize)) + 1;
            return hashIndex;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Initialize(int minCapacity)
        {
            if (hashTable.IsCreated) hashTable.Dispose();
            hashTable = new NativeArray<int>((int)(kHashTableSize + 1), Allocator.Persistent, NativeArrayOptions.ClearMemory);

            if (chainedIndices.IsCreated) chainedIndices.Dispose();
            chainedIndices = new NativeList<ChainedIndex>(minCapacity, Allocator.Persistent);

            if (vertices.IsCreated) vertices.Dispose();
            vertices = new NativeList<float3>(minCapacity, Allocator.Persistent);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void Clear()
        {
            if (chainedIndices.IsCreated)
                chainedIndices.Dispose();
            if (vertices.IsCreated)
                vertices.Dispose();
            if (hashTable.IsCreated)
                hashTable.Dispose();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ushort AddUnique(float3 vertex, int mx, int my, int mz)
        {
            var vertexIndex = (ushort)vertices.Length;
            vertices.Add(vertex);
            {
                var hashCode = GetHash(mx, my, mz);
                var prevChainIndex = hashTable[hashCode];
                var newChainIndex = chainedIndices.Length;
                var newChainedIndex = new ChainedIndex() { vertexIndex = vertexIndex, nextChainIndex = prevChainIndex };
                chainedIndices.Add(newChainedIndex);
                hashTable[(int)hashCode] = newChainIndex;
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
                        var chainIndex = hashTable[GetHash(x, y, mz - 1)];
                        {
                            ushort closestIndex = ushort.MaxValue;
                            float closestDistance = CSGManagerPerformCSG.kSqrMergeEpsilon;
                            while (chainIndex != 0)
                            {
                                var vertexIndex = chainedIndices[chainIndex].vertexIndex;
                                chainIndex = chainedIndices[chainIndex].nextChainIndex;
                                var sqrDistance = math.lengthsq(vertices[vertexIndex] - vertex);
                                if (sqrDistance < closestDistance)
                                {
                                    closestIndex = vertexIndex;
                                    closestDistance = sqrDistance;
                                }
                            }
                            if (closestIndex != ushort.MaxValue)
                                return closestIndex;
                        }
                    }

                    {
                        var chainIndex = hashTable[GetHash(x, y, mz)];
                        {
                            ushort closestIndex = ushort.MaxValue;
                            float closestDistance = CSGManagerPerformCSG.kSqrMergeEpsilon;
                            while (chainIndex != 0)
                            {
                                var vertexIndex = chainedIndices[chainIndex].vertexIndex;
                                chainIndex = chainedIndices[chainIndex].nextChainIndex;
                                var sqrDistance = math.lengthsq(vertices[vertexIndex] - vertex);
                                if (sqrDistance < closestDistance)
                                {
                                    closestIndex = vertexIndex;
                                    closestDistance = sqrDistance;
                                }
                            }
                            if (closestIndex != ushort.MaxValue)
                                return closestIndex;
                        }
                    }

                    {
                        var chainIndex = hashTable[GetHash(x, y, mz + 1)];
                        {
                            ushort closestIndex = ushort.MaxValue;
                            float closestDistance = CSGManagerPerformCSG.kSqrMergeEpsilon;
                            while (chainIndex != 0)
                            {
                                var vertexIndex = chainedIndices[chainIndex].vertexIndex;
                                chainIndex = chainedIndices[chainIndex].nextChainIndex;
                                var sqrDistance = math.lengthsq(vertices[vertexIndex] - vertex);
                                if (sqrDistance < closestDistance)
                                {
                                    closestIndex = vertexIndex;
                                    closestDistance = sqrDistance;
                                }
                            }
                            if (closestIndex != ushort.MaxValue)
                                return closestIndex;
                        }
                    }
                }
            }

            return AddUnique(vertex, mx, my, mz);
        }
    }
#endif
}
