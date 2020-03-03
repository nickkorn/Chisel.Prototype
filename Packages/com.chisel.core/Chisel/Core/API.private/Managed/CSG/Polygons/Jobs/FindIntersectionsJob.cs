﻿using System;
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

namespace Chisel.Core
{
    struct PlaneVertexIndexPair : IComparable<PlaneVertexIndexPair>
    {
        public ushort planeIndex;
        public ushort vertexIndex;

        public int CompareTo(PlaneVertexIndexPair other)
        {
            if (planeIndex < other.planeIndex)
                return -1;
            if (planeIndex > other.planeIndex)
                return 1;
            if (vertexIndex < other.vertexIndex)
                return -1;
            if (vertexIndex > other.vertexIndex)
                return 1;
            return 0;
        }
    }


    [BurstCompile]
    struct FindIntersectionsJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        [ReadOnly] public NativeArray<CSGManagerPerformCSG.PlanePair> usedPlanePairs;
        [ReadOnly] public NativeArray<float4> intersectingPlanes1;
        [ReadOnly] public NativeArray<float4> intersectingPlanes2;
        [ReadOnly] public float4x4 nodeToTreeSpaceMatrix1;

        [WriteOnly] public NativeStream.Writer  foundVertices;

            
        public void Execute(int index)
        {
            foundVertices.BeginForEachIndex(index);
            var plane2 = intersectingPlanes1[index];

            for (int i = 0; i < usedPlanePairs.Length; i++)
            {
                var planePair = usedPlanePairs[i];

                // should never happen
                if (planePair.P0 == planePair.P1)
                    continue;

                var plane0 = planePair.Plane0;
                var plane1 = planePair.Plane1;


                // FIXME: Fix situation when plane intersects edge but is not identical to either planes of the edge ...
                if (!(math.abs(plane0.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) > kNormalEpsilon) &&
                    !(math.abs(plane1.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) > kNormalEpsilon) &&

                    !(math.abs(plane0.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) < -kNormalEpsilon) &&
                    !(math.abs(plane1.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) < -kNormalEpsilon))
                {
                    var localVertex = new float4(PlaneExtensions.Intersection(plane2, plane0, plane1), 1);

                    // TODO: since we're using a pair in the outer loop, we could also determine which 
                    //       2 planes it intersects at both ends and just check those two planes ..

                    // NOTE: for brush2, the intersection will always be only on two planes
                    //       UNLESS it's a corner vertex along that edge (we can compare to the two vertices)
                    //       in which case we could use a pre-calculated list of planes ..
                    //       OR when the intersection is outside of the edge ..

                    if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes2, localVertex) &&
                        !CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes1, localVertex))
                    {
                        var worldVertex = math.mul(nodeToTreeSpaceMatrix1, localVertex).xyz;
                        foundVertices.Write(worldVertex);
                        foundVertices.Write(new int2(planePair.P0, planePair.P1));
                    }
                }
            }

            foundVertices.EndForEachIndex();
        }
    }

    [BurstCompile]
    struct InsertIntersectionVerticesJob : IJob
    {
        [ReadOnly] public NativeArray<int> intersectingPlaneIndices1;
        [ReadOnly] public NativeStream.Reader vertexReader;

        public VertexSoup brushVertices1;
        public VertexSoup brushVertices2;

        [WriteOnly] public NativeList<PlaneVertexIndexPair> foundIndices1;
        [WriteOnly] public NativeList<PlaneVertexIndexPair> foundIndices2;

        public void Execute()
        {
            int maxIndex = vertexReader.ForEachCount;

            int index = 0;
            vertexReader.BeginForEachIndex(index++);
            while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                vertexReader.BeginForEachIndex(index++);
            while (vertexReader.RemainingItemCount > 0)
            {
                var worldVertex = vertexReader.Read<float3>();
                var planePair   = vertexReader.Read<int2>();

                // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex
                var vertexIndex1 = brushVertices1.Add(worldVertex);
                var vertexIndex2 = brushVertices2.Add(worldVertex);

                foundIndices1.Add(new PlaneVertexIndexPair() { planeIndex = (ushort)intersectingPlaneIndices1[index - 1], vertexIndex = vertexIndex1 });
                foundIndices2.Add(new PlaneVertexIndexPair() { planeIndex = (ushort)planePair.x, vertexIndex = vertexIndex2 });
                foundIndices2.Add(new PlaneVertexIndexPair() { planeIndex = (ushort)planePair.y, vertexIndex = vertexIndex2 });

                while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                    vertexReader.BeginForEachIndex(index++);
            }
        }
    }

    [BurstCompile]
    struct SortLoopsJob : IJob
    {
        [ReadOnly] public NativeList<PlaneVertexIndexPair>  foundIndices;
        [ReadOnly] public NativeArray<CSGManagerPerformCSG.SurfaceInfo> surfaceCategory; // TODO: only use plane information here
        [ReadOnly] public VertexSoup                        vertexSoup;

        public NativeList<int2> sortedStack;

        //[WriteOnly]
        public NativeList<PlaneVertexIndexPair> uniqueIndices;
        //[WriteOnly]
        public NativeList<int>                  loopLengths;


        #region Sort
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        float3 FindPolygonCentroid(int offset, int indicesCount)
        {
            var centroid = float3.zero;
            var vertices = vertexSoup.vertices;
            for (int i = 0; i < indicesCount; i++, offset++)
                centroid += vertices[uniqueIndices[offset].vertexIndex];
            return centroid / indicesCount;
        }

        // TODO: sort by using plane information instead of unreliable floating point math ..
        // TODO: make this work on non-convex polygons
        void SortIndices(int offset, int indicesCount, float3 normal)
        {
            // There's no point in trying to sort a point or a line 
            if (indicesCount < 3)
                return;

            var vertices = vertexSoup.vertices;
            var indices = uniqueIndices;

            float3 tangentX, tangentY;
            if (normal.x > normal.y)
            {
                if (normal.x > normal.z)
                {
                    tangentX = math.cross(normal, new float3(0, 1, 0));
                    tangentY = math.cross(normal, tangentX);
                } else
                {
                    tangentX = math.cross(normal, new float3(0, 0, 1));
                    tangentY = math.cross(normal, tangentX);
                }
            } else
            {
                if (normal.y > normal.z)
                {
                    tangentX = math.cross(normal, new float3(1, 0, 0));
                    tangentY = math.cross(normal, tangentX);
                } else
                {
                    tangentX = math.cross(normal, new float3(0, 1, 0));
                    tangentY = math.cross(normal, tangentX);
                }
            }

            var centroid = FindPolygonCentroid(offset, indicesCount);
            var center = new float2(math.dot(tangentX, centroid), // distance in direction of tangentX
                                    math.dot(tangentY, centroid)); // distance in direction of tangentY

            sortedStack.Clear();
            sortedStack.Add(new int2(0, indicesCount - 1));
            while (sortedStack.Length > 0)
            {
                var top = sortedStack[sortedStack.Length - 1];
                sortedStack.Resize(sortedStack.Length - 1, NativeArrayOptions.UninitializedMemory);
                var l = top.x;
                var r = top.y;
                var left = l;
                var right = r;
                var va = (float3)vertices[indices[offset + (left + right) / 2].vertexIndex];
                while (true)
                {
                    var a_angle = math.atan2(math.dot(tangentX, va) - center.x, math.dot(tangentY, va) - center.y);

                    {
                        var vb = (float3)vertices[indices[offset + left].vertexIndex];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (b_angle > a_angle)
                        {
                            left++;
                            vb = (float3)vertices[indices[offset + left].vertexIndex];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    {
                        var vb = (float3)vertices[indices[offset + right].vertexIndex];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (a_angle > b_angle)
                        {
                            right--;
                            vb = (float3)vertices[indices[offset + right].vertexIndex];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    if (left <= right)
                    {
                        if (left != right)
                        {
                            var t = indices[offset + left];
                            indices[offset + left] = indices[offset + right];
                            indices[offset + right] = t;
                        }

                        left++;
                        right--;
                    }
                    if (left > right)
                        break;
                }
                if (l < right)
                {
                    sortedStack.Add(new int2(l, right));
                }
                if (left < r)
                {
                    sortedStack.Add(new int2(left, r));
                }
            }
        }
        #endregion


        public void Execute()
        {
            var previousPlaneIndex  = foundIndices[0].planeIndex;
            var previousVertexIndex = foundIndices[0].vertexIndex;
            uniqueIndices.Add(foundIndices[0]);
            var loopStart = 0;
            for (int i = 1; i < foundIndices.Length; i++)
            {
                var indices     = foundIndices[i];

                var planeIndex  = indices.planeIndex;
                var vertexIndex = indices.vertexIndex;

                // TODO: why do we have soooo many duplicates?
                if (planeIndex  == previousPlaneIndex &&
                    vertexIndex == previousVertexIndex)
                    continue;

                if (planeIndex != previousPlaneIndex)
                {
                    loopLengths.Add(uniqueIndices.Length - loopStart);
                    loopStart = uniqueIndices.Length;
                }

                uniqueIndices.Add(indices);
                previousVertexIndex = vertexIndex;
                previousPlaneIndex = planeIndex;
            }
            loopLengths.Add(uniqueIndices.Length - loopStart);

            // TODO: do in separate pass?
            int offset = 0;
            for (int n = 0; n < loopLengths.Length; n++)
            {
                var loopLength = loopLengths[n];
                var planeIndex = uniqueIndices[offset].planeIndex;
                SortIndices(offset, loopLength, surfaceCategory[planeIndex].worldPlane.normal);
                offset += loopLength;
            }
        }
    }

    // TODO: make burstable (somehow)
    struct CreateLoopsJob : IJob
    {
        [ReadOnly] public NativeList<PlaneVertexIndexPair>  foundIndices;
        [ReadOnly] public NativeList<int>                   loopLengths;
        [ReadOnly] public NativeArray<CSGManagerPerformCSG.SurfaceInfo> surfaceCategory;

        [ReadOnly] public CSGTreeBrush      brush;

        [WriteOnly] public SurfaceLoops     outputLoops;

        public void Execute()
        {
            int offset = 0;
            for (int n = 0; n < loopLengths.Length; n++)
            {
                var loopLength = loopLengths[n];
                if (loopLength > 2)
                {
                    var basePlaneIndex = foundIndices[offset].planeIndex;
                    var hole = new Loop()
                    {
                        info = new LoopInfo()
                        {
                            basePlaneIndex  = basePlaneIndex,
                            brush           = brush,
                            worldPlane      = surfaceCategory[basePlaneIndex].worldPlane,
                            right           = surfaceCategory[basePlaneIndex].right,
                            forward         = surfaceCategory[basePlaneIndex].forward,
                            layers          = surfaceCategory[basePlaneIndex].layers
                        },
                        convex = true,
                        interiorCategory = surfaceCategory[basePlaneIndex].category
                    };
                    hole.AddIndices(foundIndices, offset, loopLength);
                    outputLoops.surfaces[basePlaneIndex].Add(hole);
                } 
                offset += loopLength;
            }
        }
    }
}