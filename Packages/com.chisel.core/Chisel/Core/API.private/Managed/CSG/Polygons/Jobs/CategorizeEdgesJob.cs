#define IS_PARALLEL
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
    public struct LoopSegment
    {
        public int edgeOffset;
        public int edgeLength;
        public int planesOffset;
        public int planesLength;
    }

    static unsafe class BooleanEdgesUtility
    {
        const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOf(NativeList<Edge> edges, int edgesOffset, int edgesLength, Edge edge, out bool inverted)
        {
            for (int e = edgesOffset; e < edgesOffset + edgesLength; e++)
            {
                if (edges[e].index1 == edge.index1 && edges[e].index2 == edge.index2) { inverted = false; return e; }
                if (edges[e].index1 == edge.index2 && edges[e].index2 == edge.index1) { inverted = true; return e; }
            }
            inverted = false;
            return -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe bool IsOutsidePlanes(NativeList<float4> planes, int planesOffset, int planesLength, float4 localVertex)
        {
            var planePtr = (float4*)planes.GetUnsafeReadOnlyPtr();
            for (int n = 0; n < planesLength; n++)
            {
                var distance = math.dot(planePtr[planesOffset + n], localVertex);

                // will be 'false' when distance is NaN or Infinity
                if (!(distance <= kEpsilon))
                    return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static EdgeCategory CategorizeEdge(in Edge edge, in NativeList<float4> planes, in NativeList<Edge> edges, in LoopSegment segment, in NativeList<float3> vertices)
        {
            // TODO: use something more clever than looping through all edges
            if (IndexOf(edges, segment.edgeOffset, segment.edgeLength, edge, out bool inverted) != -1)
                return (inverted) ? EdgeCategory.ReverseAligned : EdgeCategory.Aligned;
            var midPoint = (vertices[edge.index1] + vertices[edge.index2]) * 0.5f;

            if (IsOutsidePlanes(planes, segment.planesOffset, segment.planesLength, new float4(midPoint, 1)))
                return EdgeCategory.Outside;
            return EdgeCategory.Inside;
        }

    }

    [BurstCompile(Debug = false)]
    unsafe struct SubtractEdgesJob : IJobParallelFor
    {
        [ReadOnly] public int                       segmentIndex;

        [ReadOnly] public NativeList<float3>        vertices;
        [ReadOnly] public NativeList<Edge>          allEdges;
        [ReadOnly] public NativeList<float4>        allWorldPlanes;
        [ReadOnly] public NativeList<LoopSegment>   allSegments;       

        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<byte>        destroyedEdges;

        public void Execute(int index)
        {
            var segment1 = allSegments[segmentIndex];
            var segment2 = allSegments[index];

            if (segment1.edgeLength == 0 ||
                segment2.edgeLength == 0)
                return;

            for (int e = 0; e < segment1.edgeLength; e++)
            {
                var category = BooleanEdgesUtility.CategorizeEdge(allEdges[segment1.edgeOffset + e], allWorldPlanes, allEdges, segment2, vertices);
                if (category == EdgeCategory.Outside || category == EdgeCategory.Aligned)
                    continue;
                destroyedEdges[segment1.edgeOffset + e] = 1;
            }

            for (int e = 0; e < segment2.edgeLength; e++)
            {
                var category = BooleanEdgesUtility.CategorizeEdge(allEdges[segment2.edgeOffset + e], allWorldPlanes, allEdges, segment1, vertices);
                if (category == EdgeCategory.Inside)
                    continue;
                destroyedEdges[segment2.edgeOffset + e] = 1;
            }
        }
    }
    
    [BurstCompile(Debug = false)]
#if IS_PARALLEL
    unsafe struct MergeEdgesJob : IJobParallelFor
#else
    unsafe struct MergeEdgesJob : IJob
#endif
    {
        [ReadOnly] public int segmentCount;

        [ReadOnly] public NativeList<float3>        vertices;
        [ReadOnly] public NativeList<Edge>          allEdges;
        [ReadOnly] public NativeList<float4>        allWorldPlanes;
        [ReadOnly] public NativeList<LoopSegment>   allSegments;

        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<byte>        destroyedEdges;

#if IS_PARALLEL
        public void Execute(int index)
        {
            var arrayIndex = GeometryMath.GetTriangleArrayIndex(index, segmentCount);
            var segmentIndex1 = arrayIndex.x;
            var segmentIndex2 = arrayIndex.y;
            {
                {
#else
        public void Execute()
        {
            for (int segmentIndex1 = 0; segmentIndex1 < segmentCount; segmentIndex1++)
            {
                for (int segmentIndex2 = segmentIndex1 + 1; segmentIndex2 < segmentCount; segmentIndex2++)
                {
#endif
                    var segment1 = allSegments[segmentIndex1];
                    var segment2 = allSegments[segmentIndex2];
                    if (segment1.edgeLength > 0 && segment2.edgeLength > 0)
                    {
                        for (int e = 0; e < segment1.edgeLength; e++)
                        {
                            var category = BooleanEdgesUtility.CategorizeEdge(allEdges[segment1.edgeOffset + e], allWorldPlanes, allEdges, segment2, vertices);
                            if (category == EdgeCategory.Outside ||
                                category == EdgeCategory.Aligned)
                                continue;
                            destroyedEdges[segment1.edgeOffset + e] = 1;
                        }

                        for (int e = 0; e < segment2.edgeLength; e++)
                        {
                            var category = BooleanEdgesUtility.CategorizeEdge(allEdges[segment2.edgeOffset + e], allWorldPlanes, allEdges, segment1, vertices);
                            if (category == EdgeCategory.Outside)
                                continue;
                            destroyedEdges[segment2.edgeOffset + e] = 1;
                        }
                    }
                }
            }
        }
    }
}
