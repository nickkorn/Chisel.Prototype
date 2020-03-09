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
    [BurstCompile]
    unsafe struct CategorizeEdgesJob : IJob
    {
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kPlaneDistanceEpsilon;

        [ReadOnly] public VertexSoup                    vertexSoup;
        [ReadOnly] public NativeArray<Edge>             edges1;
        [ReadOnly] public NativeArray<Edge>             edges2;
        [ReadOnly] public BlobAssetReference<BrushWorldPlanes> brushWorldPlanes;
        [ReadOnly] public EdgeCategory good1;
        [ReadOnly] public EdgeCategory good2;

        [NativeDisableUnsafePtrRestriction] public bool* destroyed1;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(Edge* edgesPtr, int edgesLength, Edge edge, out bool inverted)
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
        public EdgeCategory CategorizeEdge(Edge* edgesPtr, int edgesLength, Edge edge)
        {
            // TODO: use something more clever than looping through all edges
            if (IndexOf(edgesPtr, edgesLength, edge, out bool inverted) != -1)
                return (inverted) ? EdgeCategory.ReverseAligned : EdgeCategory.Aligned;
            var vertices = vertexSoup.vertices;
            var midPoint = (vertices[edge.index1] + vertices[edge.index2]) * 0.5f;

            if (CSGManagerPerformCSG.IsOutsidePlanes(ref brushWorldPlanes.Value.worldPlanes, new float4(midPoint, 1)))
                return EdgeCategory.Outside;
            return EdgeCategory.Inside;
        }

        public void Execute()
        {
            var edgesPtr    = (Edge*)edges2.GetUnsafeReadOnlyPtr();
            var edgesLength = edges2.Length;
            for (int index = 0; index < edges1.Length; index++)
            {
                if (!destroyed1[index])
                {
                    var category = CategorizeEdge(edgesPtr, edgesLength, edges1[index]);
                    if (category == good1 || category == good2)
                        continue;
                    destroyed1[index] = true;
                }
            }
        }
    }
}
