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
    [BurstCompile]//Debug = false
    public struct FindLoopIntersectionVerticesJob2 : IJob
    {
        public const int kMaxVertexCount = short.MaxValue;
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;

        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<float3>    verticesInput;
        public NativeList<float3>               otherVertices;
        public NativeList<float3>               verticesOutput;


        public void FindIntersections(in VertexSoup vertexSoup, List<ushort> indices, List<ushort> otherIndices, NativeArray<float4> selfPlanes)
        {
            var vertices = vertexSoup.vertices;

            verticesInput.Clear();
            verticesInput.ResizeUninitialized(indices.Count);
            // TODO: use edges instead + 2 planes intersecting each edge
            for (int v = 0; v < indices.Count; v++)
                verticesInput[v] = vertices[indices[v]];


            otherVertices.Clear();
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
                otherVertices.Add(vertex);
            NextVertex:
                ;
            }
        }

        public void GetOutput(in VertexSoup vertexSoup, List<ushort> indices)
        {
            if (verticesOutput.Length <= indices.Count)
                return;

            indices.Clear();
            if (indices.Capacity < verticesOutput.Length)
                indices.Capacity = verticesOutput.Length;
            for (int n = 0; n < verticesOutput.Length; n++)
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
            // TODO: Optimize the hell out of this
            float3 vertex1;
            float4 vertex1w;
            var vertex0 = verticesInput[verticesInput.Length - 1];
            var vertex0w = new float4(vertex0, 1);
            for (int v0 = 0; v0 < verticesInput.Length; v0++)
            {
                verticesOutput.Add(vertex0); 
                vertex1 = vertex0;
                vertex0 = verticesInput[v0];

                vertex1w = vertex0w;
                vertex0w = new float4(vertex0, 1);

                var delta = (vertex1 - vertex0);
                var max = math.dot(vertex1 - vertex0, delta);
                var first = verticesOutput.Length;
                for (int v1 = otherVertices.Length - 1; v1 >= 0; v1--)
                {
                    var otherVertex = otherVertices[v1];
                    var dot = math.dot(otherVertex - vertex0, delta);
                    if (dot <= 0 || dot >= max)
                        continue;
                    if (!GeometryMath.IsPointOnLineSegment(otherVertex, vertex0, vertex1))
                        continue;
                    verticesOutput.Add(otherVertex);
                    otherVertices.RemoveAtSwapBack(v1);
                }


                var last = verticesOutput.Length;
                if (last > first)
                {
                    float dot1, dot2;
                    for (int v1 = first; v1 < last - 1; v1++)
                    {
                        for (int v2 = first + 1; v2 < last; v2++)
                        {
                            dot1 = math.dot(delta, verticesOutput[v1]);
                            dot2 = math.dot(delta, verticesOutput[v2]);
                            if (dot1 < dot2)
                            {
                                {
                                    var t = verticesOutput[v1];
                                    verticesOutput[v1] = verticesOutput[v2];
                                    verticesOutput[v2] = t;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

#endif
}
