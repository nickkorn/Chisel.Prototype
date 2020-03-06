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
    [BurstCompile]//FloatPrecision = FloatPrecision.Low, FloatMode = FloatMode.Fast, CompileSynchronously = true, Debug = false
    public struct FindLoopPlaneIntersectionsJob : IJob
    {
        public const int kMaxVertexCount    = short.MaxValue;
        const float kVertexEqualEpsilonSqr  = (float)CSGManagerPerformCSG.kEpsilonSqr;
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kDistanceEpsilon;
             

        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<float3>    verticesInput;
        [ReadOnly] public NativeArray<float4>   otherPlanesNative;
        [ReadOnly] public NativeArray<float4>   selfPlanesNative;
        [WriteOnly] public NativeList<float3>   verticesOutput;

        // TODO: find a way to share found intersections between loops, to avoid accuracy issues
        public unsafe void Execute()
        {
            const float kVertexEqualEpsilonSqr = (float)CSGManagerPerformCSG.kEpsilonSqr;
            const float kPlaneDistanceEpsilon = (float)CSGManagerPerformCSG.kDistanceEpsilon;
             
            var tempVertices = stackalloc float3[] { float3.zero, float3.zero };
            
            var otherPlanesNativePtr = (float4*)NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(otherPlanesNative);
            var selfPlanesNativePtr  = (float4*)NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(selfPlanesNative);

            var otherPlaneCount = otherPlanesNative.Length;
            var selfPlaneCount  = selfPlanesNative.Length;

            // TODO: Optimize the hell out of this
            float3 vertex1;
            float4 vertex1w;
            var vertex0 = verticesInput[verticesInput.Length - 1];
            float4 vertex0w = new float4(vertex0, 1);
            for (int v0 = 0; v0 < verticesInput.Length; v0++)
            {
                vertex1 = vertex0;
                vertex0 = verticesInput[v0];

                vertex1w = vertex0w;
                vertex0w = new float4(vertex0, 1);

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

                    tempVertices[foundVertices] = newVertex;
                    foundVertices++;

                    // It's impossible to have more than 2 intersections on a single edge when intersecting with a convex shape
                    if (foundVertices == 2)
                        break;
                    SkipEdge:
                    ;
                }

                if (foundVertices > 0)
                {
                    if (foundVertices == 2)
                    {
                        var dot0 = math.lengthsq(tempVertices[0] - vertex1);
                        var dot1 = math.lengthsq(tempVertices[1] - vertex1);
                        if (dot0 < dot1)
                        {
                            verticesOutput.Add(tempVertices[0]);
                            verticesOutput.Add(tempVertices[1]);
                        }
                        else
                        {
                            verticesOutput.Add(tempVertices[1]);
                            verticesOutput.Add(tempVertices[0]);
                        }
                    } else
                    {
                        verticesOutput.Add(tempVertices[0]);
                    }
                }
                verticesOutput.Add(vertex0);
            }
        }
    }

#endif
}
