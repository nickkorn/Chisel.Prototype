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
    public enum OperationResult : byte
    {
        Fail,
        Cut,
        Outside,
        Polygon1InsidePolygon2,
        Polygon2InsidePolygon1,
        Overlapping
    }

    static unsafe class BooleanEdgesUtility
    {
        const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOf(NativeArray<Edge> edges, int edgesOffset, int edgesLength, Edge edge, out bool inverted)
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
        public static int IndexOf(NativeArray<Edge> edges, Edge edge, out bool inverted)
        {
            for (int e = 0; e < edges.Length; e++)
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
        public static unsafe bool IsOutsidePlanes(ref BlobArray<float4> planes, float4 localVertex)
        {
            for (int n = 0; n < planes.Length; n++)
            {
                var distance = math.dot(planes[n], localVertex);

                // will be 'false' when distance is NaN or Infinity
                if (!(distance <= kEpsilon))
                    return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static EdgeCategory CategorizeEdge(Edge edge, in NativeList<float4> planes, in NativeArray<Edge> edges, in LoopSegment segment, in VertexSoup vertices)
        {
            // TODO: use something more clever than looping through all edges
            if (IndexOf(edges, segment.edgeOffset, segment.edgeLength, edge, out bool inverted) != -1)
                return (inverted) ? EdgeCategory.ReverseAligned : EdgeCategory.Aligned;
            var midPoint = (vertices[edge.index1] + vertices[edge.index2]) * 0.5f;

            if (IsOutsidePlanes(planes, segment.planesOffset, segment.planesLength, new float4(midPoint, 1)))
                return EdgeCategory.Outside;
            return EdgeCategory.Inside;
        }

        internal static EdgeCategory CategorizeEdge(Edge edge, ref BlobArray<float4> planes, in NativeArray<Edge> edges, in VertexSoup vertices)
        {
            // TODO: use something more clever than looping through all edges
            if (IndexOf(edges, edge, out bool inverted) != -1)
                return (inverted) ? EdgeCategory.ReverseAligned : EdgeCategory.Aligned;
            var midPoint = (vertices[edge.index1] + vertices[edge.index2]) * 0.5f;

            if (IsOutsidePlanes(ref planes, new float4(midPoint, 1)))
                return EdgeCategory.Outside;
            return EdgeCategory.Inside;
        }


        // Note: Assumes polygons are convex
        public unsafe static bool AreLoopsOverlapping(Loop polygon1, Loop polygon2)
        {
            if (!polygon1.edges.IsCreated ||
                !polygon2.edges.IsCreated)
                return false;
            if (polygon1.edges.Length < 3 ||
                polygon2.edges.Length < 3)
                return false;

            if (polygon1.edges.Length != polygon2.edges.Length)
                return false;

            for (int i = 0; i < polygon1.edges.Length; i++)
            {
                if (IndexOf(polygon2.edges, polygon1.edges[i], out bool inverted) == -1)
                    return false;
            }
            return true;
        }
    }

    // Note: Assumes polygons are convex
    [BurstCompile(CompileSynchronously = true)]
    unsafe struct SubtractEdgesJob : IJobParallelFor
    {
        [ReadOnly] public int                       segmentIndex;

        [ReadOnly] public VertexSoup                vertices;
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

    // Note: Assumes polygons are convex
    [BurstCompile(CompileSynchronously = true)]
#if IS_PARALLEL
    unsafe struct MergeEdgesJob : IJobParallelFor
#else
    unsafe struct MergeEdgesJob : IJob
#endif
    {
        [ReadOnly] public int segmentCount;

        [ReadOnly] public VertexSoup                vertices;
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


    // Note: Assumes polygons are convex
    [BurstCompile(CompileSynchronously = true)]
    unsafe struct IntersectEdgesJob : IJob
    {
        [ReadOnly] public VertexSoup                 vertices;
        [ReadOnly] public NativeArray<Edge>          edges1;
        [ReadOnly] public NativeArray<Edge>          edges2;
        [ReadOnly] public BlobAssetReference<BrushWorldPlanes> worldPlanes1;
        [ReadOnly] public BlobAssetReference<BrushWorldPlanes> worldPlanes2;

        [NativeDisableUnsafePtrRestriction]
        [WriteOnly] public OperationResult* result; // FIXME: SHOULD NOT DO THIS
        [WriteOnly] public NativeList<Edge> outEdges;

        public void Execute()
        {
            if (edges1.Length == 0 ||
                edges2.Length == 0)
            {
                *result = OperationResult.Outside;
                return;
            }

            int inside2 = 0, outside2 = 0;
            var categories2 = new NativeArray<EdgeCategory>(edges2.Length, Allocator.Temp);
            for (int e = 0; e < edges2.Length; e++)
            {
                var category = BooleanEdgesUtility.CategorizeEdge(edges2[e], ref worldPlanes1.Value.worldPlanes, edges1, vertices);
                categories2[e] = category;
                if (category == EdgeCategory.Inside) inside2++;
                else if (category == EdgeCategory.Outside) outside2++;
            }
            var aligned2 = edges2.Length - (inside2 + outside2);

            int inside1 = 0, outside1 = 0;
            var categories1 = new NativeArray<EdgeCategory>(edges1.Length, Allocator.Temp);
            for (int e = 0; e < edges1.Length; e++)
            {
                var category = BooleanEdgesUtility.CategorizeEdge(edges1[e], ref worldPlanes2.Value.worldPlanes, edges2, vertices);
                categories1[e] = category;
                if (category == EdgeCategory.Inside) inside1++;
                else if (category == EdgeCategory.Outside) outside1++;
            }
            var aligned1 = edges1.Length - (inside1 + outside1);

            // Completely aligned
            if ((outside1 + inside1) == 0 &&
                (outside2 + inside2) == 0)
            {
                *result = OperationResult.Overlapping;
            } else
            // polygon1 edges Completely inside polygon2
            if (inside1 == 0 && outside2 == 0)
            {
                outEdges.AddRange(edges2);
                *result = OperationResult.Polygon2InsidePolygon1;
            } else
            // polygon2 edges Completely inside polygon1
            if (outside1 == 0 && inside2 == 0)
            {
                // polygon1 Completely inside polygon2
                outEdges.AddRange(edges1);
                *result = OperationResult.Polygon1InsidePolygon2;
            }  else
            // Completely outside
            if ((inside1 + aligned1) == 0 && (aligned2 + inside2) == 0)
            {
                *result = OperationResult.Outside;
            } else
            {
                int outEdgesLength = 0; // Can't read from outEdges.Length since it's marked as WriteOnly
                for (int e = 0; e < edges1.Length; e++)
                {
                    var category = categories1[e];
                    if (category == EdgeCategory.Inside)
                    {
                        outEdges.Add(edges1[e]);
                        outEdgesLength++;
                    }
                }

                for (int e = 0; e < edges2.Length; e++)
                {
                    var category = categories2[e];
                    if (category != EdgeCategory.Outside)
                    {
                        outEdges.Add(edges2[e]);
                        outEdgesLength++;
                    }
                }

                if (outEdgesLength < 3)
                    *result = OperationResult.Outside;
                else
                    *result = OperationResult.Cut;
            }

            categories1.Dispose();
            categories2.Dispose();
        }
    }

}
