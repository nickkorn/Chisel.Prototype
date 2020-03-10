using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;
using System.Runtime.CompilerServices;
using Unity.Entities;

namespace Chisel.Core
{
    [BurstCompile(Debug = false)]
    unsafe struct CategorizeEdgesJob : IJob
    {
        [ReadOnly] public VertexSoup vertexSoup;

        [ReadOnly] public NativeArray<Edge> edges1;
        [ReadOnly] public NativeArray<Edge> edges2;

        [ReadOnly] public BlobAssetReference<BrushWorldPlanes> brushWorldPlanes1;
        [ReadOnly] public BlobAssetReference<BrushWorldPlanes> brushWorldPlanes2;

        [NativeDisableUnsafePtrRestriction] [ReadOnly] public bool* destroyed1;
        [NativeDisableUnsafePtrRestriction] [ReadOnly] public bool* destroyed2;

        [ReadOnly] public EdgeCategory good1;
        [ReadOnly] public EdgeCategory good2;
        [ReadOnly] public EdgeCategory good3;
        [ReadOnly] public EdgeCategory good4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOf(Edge* edgesPtr, int edgesLength, Edge edge, out bool inverted)
        {
            for (int e = 0; e < edgesLength; e++)
            {
                if (edgesPtr[e].index1 == edge.index1 && edgesPtr[e].index2 == edge.index2) { inverted = false; return e; }
                if (edgesPtr[e].index1 == edge.index2 && edgesPtr[e].index2 == edge.index1) { inverted = true; return e; }
            }
            inverted = false;
            return -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static EdgeCategory CategorizeEdge(ref BlobArray<float4> worldPlanes, in VertexSoup vertexSoup, Edge* edgesPtr, int edgesLength, Edge edge)
        {
            // TODO: use something more clever than looping through all edges
            if (IndexOf(edgesPtr, edgesLength, edge, out bool inverted) != -1)
                return (inverted) ? EdgeCategory.ReverseAligned : EdgeCategory.Aligned;
            var vertices = vertexSoup.vertices;
            var midPoint = (vertices[edge.index1] + vertices[edge.index2]) * 0.5f;

            if (CSGManagerPerformCSG.IsOutsidePlanes(ref worldPlanes, new float4(midPoint, 1)))
                return EdgeCategory.Outside;
            return EdgeCategory.Inside;
        }

        public void Execute()
        {
            var edges1Ptr = (Edge*)edges1.GetUnsafeReadOnlyPtr();
            var edges1Length = edges1.Length;

            var edges2Ptr = (Edge*)edges2.GetUnsafeReadOnlyPtr();
            var edges2Length = edges2.Length;

            var categories1 = new NativeArray<EdgeCategory>(edges1Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var categories2 = new NativeArray<EdgeCategory>(edges2Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int index = 0; index < edges1Length; index++)
            {
                if (!destroyed1[index])
                {
                    categories1[index] = CategorizeEdge(ref brushWorldPlanes2.Value.worldPlanes, vertexSoup, edges2Ptr, edges2Length, edges1Ptr[index]);
                }
            }

            for (int index = 0; index < edges2Length; index++)
            {
                if (!destroyed2[index])
                {
                    categories2[index] = CategorizeEdge(ref brushWorldPlanes1.Value.worldPlanes, vertexSoup, edges1Ptr, edges1Length, edges2Ptr[index]);
                }
            }

            for (int index = 0; index < edges1Length; index++)
            {
                if (!destroyed1[index])
                {
                    var category = categories1[index];
                    if (category == good1 || category == good2)
                        continue;
                    destroyed1[index] = true;
                }
            }

            for (int index = 0; index < edges2Length; index++)
            {
                if (!destroyed2[index])
                {
                    var category = categories2[index];
                    if (category == good3 || category == good4)
                        continue;
                    destroyed2[index] = true;
                }
            }
            categories1.Dispose();
            categories2.Dispose();
        }
    }
}
