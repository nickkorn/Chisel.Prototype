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
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    //[BurstCompile(Debug = false)]
    public struct FindLoopVertexOverlapsJob : IJob
    {
        public const int kMaxVertexCount = short.MaxValue;
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;

        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeArray<float4>   selfPlanes;
        [ReadOnly] public VertexSoup            vertexSoup;
        [ReadOnly] public NativeList<ushort>    otherIndices;
        [ReadOnly] public NativeList<Edge>      otherEdges;
        public NativeList<ushort>               indices;
        public NativeList<Edge>                 edges;


        // TODO: find a way to share found intersections between loops, to avoid accuracy issues
        public unsafe void ExecuteIndices()
        {
            if (indices.Length < 3 ||
                otherIndices.Length < 3)
                return;

            var vertices = vertexSoup.vertices;

            var otherVerticesLength = 0;
            var otherVertices       = (ushort*)UnsafeUtility.Malloc(otherIndices.Length * sizeof(ushort), 4, Allocator.TempJob);

            // TODO: use edges instead + 2 planes intersecting each edge
            for (int v = 0; v < otherIndices.Length; v++)
            {
                var vertexIndex = otherIndices[v];
                if (indices.Contains(vertexIndex))
                    continue;

                var vertex = vertices[vertexIndex];
                for (int p = 0; p < selfPlanes.Length; p++)
                {
                    var distance = math.dot(selfPlanes[p], new float4(vertex, 1));
                    if (distance > kPlaneDistanceEpsilon)
                        goto NextVertex;
                }
                // TODO: Check if vertex intersects at least 2 selfPlanes
                otherVertices[otherVerticesLength] = vertexIndex;
                otherVerticesLength++;
            NextVertex:
                ;
            }

            if (otherVerticesLength == 0)
            {
                UnsafeUtility.Free(otherVertices, Allocator.TempJob);
                return;
            }

            var inputIndicesLength = indices.Length;
            var inputIndices = (ushort*)UnsafeUtility.Malloc(indices.Length * sizeof(ushort), 4, Allocator.TempJob);
            UnsafeUtility.MemCpyReplicate(inputIndices, indices.GetUnsafePtr(), sizeof(ushort) * indices.Length, 1);

            indices.Clear();

            // TODO: Optimize the hell out of this
            float3 vertex1;
            float4 vertex1w;
            var vertexIndex0 = inputIndices[inputIndicesLength - 1];
            var vertex0 = vertices[vertexIndex0];
            var vertex0w = new float4(vertex0, 1);
            for (int v0 = 0; v0 < inputIndicesLength && otherVerticesLength > 0; v0++)
            {
                if (v0 > 0)
                    indices.Add(vertexIndex0);

                var vertexIndex1 = vertexIndex0;
                vertexIndex0 = inputIndices[v0];

                vertex1 = vertex0;
                vertex0 = vertices[vertexIndex0];

                vertex1w = vertex0w;
                vertex0w = new float4(vertex0, 1);

                var delta = (vertex1 - vertex0);
                var max = math.dot(vertex1 - vertex0, delta);
                var first = indices.Length;
                for (int v1 = otherVerticesLength - 1; v1 >= 0; v1--)
                {
                    var otherVertexIndex = otherVertices[v1];
                    var otherVertex = vertices[otherVertexIndex];
                    var dot = math.dot(otherVertex - vertex0, delta);
                    if (dot <= 0 || dot >= max)
                        continue;
                    if (!GeometryMath.IsPointOnLineSegment(otherVertex, vertex0, vertex1))
                        continue;

                    // Note: the otherVertices array cannot contain any indices that are part in 
                    //       the input indices, since we checked for that when we created it.
                    indices.Add(otherVertexIndex);

                    // TODO: figure out why removing vertices fails?
                    //if (v1 != otherVerticesLength - 1 && otherVerticesLength > 0)
                    //    otherVertices[v1] = otherVertices[otherVerticesLength - 1];
                    //otherVerticesLength--;
                }


                var last = indices.Length;
                if (last > first)
                {
                    float dot1, dot2;
                    for (int v1 = first; v1 < last - 1; v1++)
                    {
                        for (int v2 = first + 1; v2 < last; v2++)
                        {
                            var otherVertexIndex1 = indices[v1];
                            var otherVertexIndex2 = indices[v2];
                            var otherVertex1 = vertices[otherVertexIndex1];
                            var otherVertex2 = vertices[otherVertexIndex2];
                            dot1 = math.dot(delta, otherVertex1);
                            dot2 = math.dot(delta, otherVertex2);
                            if (dot1 < dot2)
                            {
                                indices[v1] = otherVertexIndex2;
                                indices[v2] = otherVertexIndex1;
                            }
                        }
                    }
                }
            }
            indices.Add(inputIndices[inputIndicesLength - 1]);

            UnsafeUtility.Free(inputIndices, Allocator.TempJob);
            UnsafeUtility.Free(otherVertices, Allocator.TempJob);
        }

        public unsafe void ExecuteEdges()
        {
            if (edges.Length < 3 ||
                otherEdges.Length < 3)
                return;

            var vertices = vertexSoup.vertices;

            var otherVerticesLength = 0;
            var otherVertices       = (ushort*)UnsafeUtility.Malloc(otherIndices.Length * sizeof(ushort), 4, Allocator.TempJob);

            // TODO: use edges instead + 2 planes intersecting each edge
            for (int v = 0; v < otherIndices.Length; v++)
            {
                var vertexIndex = otherIndices[v];
                for (int e = 0; e < edges.Length; e++)
                {
                    if (edges[e].index1 == vertexIndex ||
                        edges[e].index2 == vertexIndex)
                        goto NextVertex;
                }

                var vertex = vertices[vertexIndex];
                for (int p = 0; p < selfPlanes.Length; p++)
                {
                    var distance = math.dot(selfPlanes[p], new float4(vertex, 1));
                    if (distance > kPlaneDistanceEpsilon)
                        goto NextVertex;
                }
                // TODO: Check if vertex intersects at least 2 selfPlanes
                otherVertices[otherVerticesLength] = vertexIndex;
                otherVerticesLength++;
            NextVertex:
                ;
            }

            if (otherVerticesLength == 0)
            {
                UnsafeUtility.Free(otherVertices, Allocator.TempJob);
                return;
            }

            var tempList = new NativeList<ushort>(Allocator.Temp);
            {
                var tempListPtr = (ushort*)tempList.GetUnsafePtr();
                var inputEdgesLength = edges.Length;
                var inputEdges = (Edge*)UnsafeUtility.Malloc(edges.Length * sizeof(Edge), 4, Allocator.TempJob);
                UnsafeUtility.MemCpyReplicate(inputEdges, edges.GetUnsafePtr(), sizeof(Edge) * edges.Length, 1);
                edges.Clear();

                // TODO: Optimize the hell out of this
                for (int e = 0; e < inputEdgesLength && otherVerticesLength > 0; e++)
                {
                    var vertexIndex0 = inputEdges[e].index1;
                    var vertexIndex1 = inputEdges[e].index2;

                    var vertex0 = vertexSoup.vertices[vertexIndex0];
                    var vertex1 = vertexSoup.vertices[vertexIndex1];

                    var vertex0w = new float4(vertex0, 1);
                    var vertex1w = new float4(vertex1, 1);

                    tempList.Clear();
                    tempList.Add(vertexIndex0);

                    var delta = (vertex1 - vertex0);
                    var max = math.dot(vertex1 - vertex0, delta);
                    for (int v1 = otherVerticesLength - 1; v1 >= 0; v1--)
                    {
                        var otherVertexIndex = otherVertices[v1];
                        if (otherVertexIndex == vertexIndex0 ||
                            otherVertexIndex == vertexIndex1)
                            continue;
                        var otherVertex = vertices[otherVertexIndex];
                        var dot = math.dot(otherVertex - vertex0, delta);
                        if (dot <= 0 || dot >= max)
                            continue;
                        if (!GeometryMath.IsPointOnLineSegment(otherVertex, vertex0, vertex1))
                            continue;

                        // Note: the otherVertices array cannot contain any indices that are part in 
                        //       the input indices, since we checked for that when we created it.
                        tempList.Add(otherVertexIndex);

                        // TODO: figure out why removing vertices fails?
                        //if (v1 != otherVerticesLength - 1 && otherVerticesLength > 0)
                        //    otherVertices[v1] = otherVertices[otherVerticesLength - 1];
                        //otherVerticesLength--;
                    }
                    tempList.Add(vertexIndex1);

                    if (tempList.Length > 2)
                    {
                        float dot1, dot2;
                        var last = tempList.Length - 1;
                        for (int v1 = 1; v1 < last - 1; v1++)
                        {
                            for (int v2 = 2; v2 < last; v2++)
                            {
                                var otherVertexIndex1 = tempList[v1];
                                var otherVertexIndex2 = tempList[v2];
                                var otherVertex1 = vertices[otherVertexIndex1];
                                var otherVertex2 = vertices[otherVertexIndex2];
                                dot1 = math.dot(delta, otherVertex1);
                                dot2 = math.dot(delta, otherVertex2);
                                if (dot1 >= dot2)
                                {
                                    tempListPtr[v1] = otherVertexIndex2;
                                    tempListPtr[v2] = otherVertexIndex1;
                                }
                            }
                        }
                        for (int i = 1; i < tempList.Length; i++)
                        {
                            if (tempList[i - 1] != tempList[i])
                                edges.Add(new Edge() { index1 = tempList[i - 1], index2 = tempList[i] });
                        }
                    } else
                    {
                        edges.Add(inputEdges[e]);
                    }
                }
                /*
                for (int e = 0; e < edges.Length; e++)
                {
                    var edge = edges[e];
                    if (indices[indices.Length - 1] == edge.index1 &&
                        indices[0] == edge.index2)
                        continue;
                    for (int i = 1; i < indices.Length; i++)
                    {
                        if (indices[i - 1] == edge.index1 &&
                            indices[i] == edge.index2)
                            goto Found;
                    }
                    Debug.Log("Fail");
                Found:
                    ;
                }
                */
                UnsafeUtility.Free(inputEdges, Allocator.TempJob);
                UnsafeUtility.Free(otherVertices, Allocator.TempJob);
            }
            tempList.Dispose();
        }

        public void Execute()
        {
            ExecuteIndices();
            ExecuteEdges();
        }
    }

#endif
}
