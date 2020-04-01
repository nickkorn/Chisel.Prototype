using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Entities;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    [BurstCompile(Debug = false)]
    public struct FindLoopPlaneIntersectionsJob : IJob
    {
        public const int kMaxVertexCount    = short.MaxValue;
        const float kVertexEqualEpsilonSqr  = (float)CSGManagerPerformCSG.kEpsilonSqr;
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kDistanceEpsilon;
             
        [ReadOnly] public NativeList<float4>    allWorldSpacePlanes;
        [ReadOnly] public int2                  otherPlanesSegment;
        [ReadOnly] public int2                  selfPlanesSegment;

        [NativeDisableContainerSafetyRestriction]
        public VertexSoup                       vertexSoup; // <-- TODO: we're reading AND writing to the same NativeList!?!?!
        public NativeList<Edge>                 edges;

        // TODO: find a way to share found intersections between loops, to avoid accuracy issues
        public unsafe void Execute()
        {
            if (edges.Length < 3)
                return;
            
            var inputEdgesLength    = edges.Length;
            var inputEdges          = (Edge*)UnsafeUtility.Malloc(edges.Length * sizeof(Edge), 4, Allocator.TempJob);
            UnsafeUtility.MemCpyReplicate(inputEdges, edges.GetUnsafePtr(), sizeof(Edge) * edges.Length, 1);
            edges.Clear();

            var tempVertices = stackalloc ushort[] { 0, 0, 0, 0 };

            var allWorldSpacePlanePtr   = (float4*)allWorldSpacePlanes.GetUnsafeReadOnlyPtr();
            var otherPlanesNativePtr    = allWorldSpacePlanePtr + otherPlanesSegment.x;
            var selfPlanesNativePtr     = allWorldSpacePlanePtr + selfPlanesSegment.x;

            var otherPlaneCount = otherPlanesSegment.y;
            var selfPlaneCount  = selfPlanesSegment.y;

            vertexSoup.Reserve(otherPlaneCount); // ensure we have at least this many extra vertices in capacity

            // TODO: Optimize the hell out of this
            for (int e = 0; e < inputEdgesLength; e++)
            {
                var vertexIndex0 = inputEdges[e].index1;
                var vertexIndex1 = inputEdges[e].index2;

                var vertex0 = vertexSoup[vertexIndex0];
                var vertex1 = vertexSoup[vertexIndex1];

                var vertex0w = new float4(vertex0, 1);
                var vertex1w = new float4(vertex1, 1);

                var foundVertices = 0;

                for (int p = 0; p < otherPlaneCount; p++)
                {
                    var otherPlane = otherPlanesNativePtr[p];

                    var distance0 = math.dot(otherPlane, vertex0w);
                    var distance1 = math.dot(otherPlane, vertex1w);

                    if (distance0 < 0)
                    {
                        if (distance1 <=  kPlaneDistanceEpsilon ||
                            distance0 >= -kPlaneDistanceEpsilon) continue;
                    } else
                    {
                        if (distance1 >= -kPlaneDistanceEpsilon ||
                            distance0 <=  kPlaneDistanceEpsilon) continue;
                    }

                    float3 newVertex;

                    // Ensure we always do the intersection calculations in the exact same 
                    // direction across a plane to increase floating point consistency
                    if (distance0 > 0)
                    {
                        var length = distance0 - distance1;
                        var delta = distance0 / length;
                        if (delta <= 0 || delta >= 1)
                            continue;
                        var vector = vertex0 - vertex1;
                        newVertex = vertex0 - (vector * delta);
                    } else
                    {
                        var length = distance1 - distance0;
                        var delta = distance1 / length;
                        if (delta <= 0 || delta >= 1)
                            continue;
                        var vector = vertex1 - vertex0;
                        newVertex = vertex1 - (vector * delta);
                    }

                    // Check if the new vertex is identical to one of our existing vertices
                    if (math.lengthsq(vertex0 - newVertex) <= kVertexEqualEpsilonSqr ||
                        math.lengthsq(vertex1 - newVertex) <= kVertexEqualEpsilonSqr)
                        continue;

                    var newVertexw = new float4(newVertex, 1);
                    for (int p2 = 0; p2 < otherPlaneCount; p2++)
                    {
                        otherPlane = otherPlanesNativePtr[p2];
                        var distance = math.dot(otherPlane, newVertexw);
                        if (distance > kPlaneDistanceEpsilon)
                            goto SkipEdge;
                    }
                    for (int p1 = 0; p1 < selfPlaneCount; p1++)
                    {
                        otherPlane = selfPlanesNativePtr[p1];
                        var distance = math.dot(otherPlane, newVertexw);
                        if (distance > kPlaneDistanceEpsilon)
                            goto SkipEdge;
                    }

                    var tempVertexIndex = vertexSoup.AddNoResize(newVertex);
                    if ((foundVertices == 0 || tempVertexIndex != tempVertices[1]) &&
                        vertexIndex0 != tempVertexIndex &&
                        vertexIndex1 != tempVertexIndex)
                    {
                        tempVertices[foundVertices + 1] = tempVertexIndex;
                        foundVertices++;

                        // It's impossible to have more than 2 intersections on a single edge when intersecting with a convex shape
                        if (foundVertices == 2)
                            break;
                    }
                    SkipEdge:
                    ;
                }

                if (foundVertices > 0)
                {
                    var tempVertexIndex0 = tempVertices[1];
                    var tempVertex0 = vertexSoup[tempVertexIndex0];
                    var tempVertexIndex1 = tempVertexIndex0;
                    if (foundVertices == 2)
                    {
                        tempVertexIndex1 = tempVertices[2];
                        var tempVertex1 = vertexSoup[tempVertexIndex1];
                        var dot0 = math.lengthsq(tempVertex0 - vertex1);
                        var dot1 = math.lengthsq(tempVertex1 - vertex1);
                        if (dot0 < dot1)
                        {
                            tempVertices[1] = tempVertexIndex1;
                            tempVertices[2] = tempVertexIndex0;
                        }
                    }
                    tempVertices[0] = vertexIndex0;
                    tempVertices[1 + foundVertices] = vertexIndex1;
                    for (int i = 1; i < 2 + foundVertices; i++)
                    {
                        if (tempVertices[i - 1] != tempVertices[i])
                            edges.Add(new Edge() { index1 = tempVertices[i - 1], index2 = tempVertices[i] });
                    }
                } else
                {
                    edges.Add(inputEdges[e]);
                }
            }

            UnsafeUtility.Free(inputEdges, Allocator.TempJob);
        }
    }

#endif
}
