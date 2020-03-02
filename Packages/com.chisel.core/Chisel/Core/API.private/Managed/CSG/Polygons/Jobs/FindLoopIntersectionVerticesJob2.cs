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
    [BurstCompile(FloatPrecision = FloatPrecision.Low, FloatMode = FloatMode.Fast, CompileSynchronously = true, Debug = false)]
    public struct FindLoopIntersectionVerticesJob2 : IJob
    {
        public const int kMaxVertexCount = short.MaxValue;

        [ReadOnly] public NativeArray<float3>   verticesInput;
        [ReadOnly] public NativeArray<float3>   otherVertices;
        [WriteOnly] public NativeArray<float3>  verticesOutput;
        public int vertexCount;
        public int otherVertexCount;


        public void FindIntersections(in VertexSoup vertexSoup, List<ushort> indices, List<ushort> otherIndices, NativeArray<float4> selfPlanes)
        {
            var vertices = vertexSoup.vertices;

            vertexCount = indices.Count;
            // TODO: use edges instead + 2 planes intersecting each edge
            for (int v = 0; v < indices.Count; v++)
                verticesInput[v] = vertices[indices[v]];

            const float kPlaneDistanceEpsilon = (float)CSGManagerPerformCSG.kDistanceEpsilon;

            otherVertexCount = 0;
            for (int v = 0; v < otherIndices.Count; v++)
            {
                if (indices.Contains(otherIndices[v]))
                    continue;
                var vertex = vertices[otherIndices[v]];
                foreach (var plane in selfPlanes)
                {
                    var distance = math.dot(plane, new float4(vertex, 1));
                    if (distance > kPlaneDistanceEpsilon)
                        goto NextVertex;
                }
                // TODO: Check if vertex intersects at least 2 selfPlanes
                otherVertices[otherVertexCount] = vertices[otherIndices[v]];
                otherVertexCount++;
            NextVertex:
                ;
            }
            Execute();
        }

        public void GetOutput(in VertexSoup vertexSoup, List<ushort> indices)
        {
            if (vertexCount <= indices.Count)
                return;

            indices.Clear();
            if (indices.Capacity < vertexCount)
                indices.Capacity = vertexCount;
            for (int n = 0; n < vertexCount; n++)
            {
                var worldVertex = verticesOutput[n];
                var vertexIndex = vertexSoup.Add(worldVertex);

                if (indices.Contains(vertexIndex))
                    continue;

                indices.Add(vertexIndex);
            }
        }


        // TODO: find a way to share found intersections between loops, to avoid accuracy issues
        public unsafe void Execute()
        {
            var tempVertices = stackalloc float3[] { float3.zero, float3.zero };
            var innerVertexCount = 0;

            var verticesSrcPtr       = (float3*)NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(verticesInput);
            var otherVerticesSrcPtr  = (float3*)NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(otherVertices);
            var verticesDstPtr       = (float3*)NativeArrayUnsafeUtility.GetUnsafePtr(verticesOutput);

            // TODO: Optimize the hell out of this
            float3 vertex1;
            float4 vertex1w;
            var vertex0 = verticesSrcPtr[vertexCount - 1];
            var vertex0w = new float4(vertex0, 1);
            for (int v0 = 0; v0 < vertexCount; v0++)
            {
                verticesDstPtr[innerVertexCount] = vertex0; innerVertexCount++;
                vertex1 = vertex0;
                vertex0 = verticesSrcPtr[v0];

                vertex1w = vertex0w;
                vertex0w = new float4(vertex0, 1);

                var delta = (vertex1 - vertex0);
                var max = math.dot(vertex1 - vertex0, delta);
                var foundVertices = 0;
                for (int v1 = 0; v1 < otherVertexCount; v1++)
                {
                    var otherVertex = otherVerticesSrcPtr[v1];
                    var dot = math.dot(otherVertex - vertex0, delta);
                    if (dot <= 0 || dot >= max) 
                        continue;
                    if (!GeometryMath.IsPointOnLineSegment(otherVertex, vertex0, vertex1))
                        continue;
                    verticesDstPtr[innerVertexCount] = otherVertex; innerVertexCount++;
                    foundVertices++;
                }
                

                if (foundVertices > 0)
                {
                    var first = innerVertexCount;
                    var last = innerVertexCount + foundVertices;
                    for (int v1 = first; v1 < last - 1; v1++)
                    {
                        var dot1 = math.dot(delta, verticesDstPtr[v1]);
                        for (int v2 = first + 1; v2 < last; v2++)
                        {
                            var dot2 = math.dot(delta, verticesDstPtr[v2]);
                            if (dot1 > dot2)
                            {
                                {
                                    var t = verticesDstPtr[v1];
                                    verticesDstPtr[v1] = verticesDstPtr[v2];
                                    verticesDstPtr[v2] = t;
                                }
                                {
                                    var t = dot1;
                                    dot1 = dot2;
                                    dot2 = t;
                                }
                            }
                        }
                    }
                }
            }
            this.vertexCount = innerVertexCount;
        }
    }

#endif
}
