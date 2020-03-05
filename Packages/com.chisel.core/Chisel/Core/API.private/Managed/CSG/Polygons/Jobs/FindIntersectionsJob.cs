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

namespace Chisel.Core
{
    public struct SurfaceInfo
    {
        public float4               worldPlane;
        public SurfaceLayers        layers;
        public int                  basePlaneIndex;
        public CSGTreeBrush         brush;
        public CategoryGroupIndex   interiorCategory;
    }

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

    struct PlaneIndexLengthPair : IComparable<PlaneIndexLengthPair>
    {
        public ushort planeIndex;
        public ushort polygonLength;

        public int CompareTo(PlaneIndexLengthPair other)
        {
            if (planeIndex < other.planeIndex)
                return -1;
            if (planeIndex > other.planeIndex)
                return 1;
            if (polygonLength < other.polygonLength)
                return -1;
            if (polygonLength > other.polygonLength)
                return 1;
            return 0;
        }
    }

    struct VertexAndPlanePair
    {
        public float3 vertex;
        public ushort plane0;
        public ushort plane1;
    }


    [BurstCompile]
    unsafe struct FindIntersectionsJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<CSGManagerPerformCSG.PlanePair> usedPlanePairs;
        [ReadOnly] public NativeList<float4>    intersectingPlanes1;
        [ReadOnly] public NativeList<float4>    intersectingPlanes2;
        [ReadOnly] public float4x4              nodeToTreeSpaceMatrix1;

        [WriteOnly] public NativeStream.Writer foundVertices;
            
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
                        foundVertices.Write(new VertexAndPlanePair()
                        {
                            vertex = worldVertex,
                            plane0 = (ushort)planePair.P0,
                            plane1 = (ushort)planePair.P1
                        });
                    }
                }
            } 

            foundVertices.EndForEachIndex();
        }
    }

    [BurstCompile]
    unsafe struct InsertIntersectionVerticesJob : IJob
    {
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<int> intersectingPlaneIndices1;
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
                var vertexAndPlanePair = vertexReader.Read<VertexAndPlanePair>();                
                var worldVertex = vertexAndPlanePair.vertex;
                var plane0      = vertexAndPlanePair.plane0;
                var plane1      = vertexAndPlanePair.plane1; 

                // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex
                var vertexIndex1 = brushVertices1.Add(worldVertex);
                var vertexIndex2 = brushVertices2.Add(worldVertex);

                foundIndices1.Add(new PlaneVertexIndexPair() { planeIndex = (ushort)intersectingPlaneIndices1[index - 1], vertexIndex = vertexIndex1 });
                foundIndices2.Add(new PlaneVertexIndexPair() { planeIndex = plane0, vertexIndex = vertexIndex2 });
                foundIndices2.Add(new PlaneVertexIndexPair() { planeIndex = plane1, vertexIndex = vertexIndex2 });

                while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                    vertexReader.BeginForEachIndex(index++);
            }
        }
    }

    [BurstCompile]
    unsafe struct SortLoopsJob : IJob
    {
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<PlaneVertexIndexPair>  foundIndices;
        [ReadOnly] public NativeArray<SurfaceInfo> surfaceCategory; // TODO: only use plane information here
        [ReadOnly] public VertexSoup                        vertexSoup;

        public NativeList<int2> sortedStack;

        [NativeDisableContainerSafetyRestriction]//[WriteOnly]
        public NativeList<ushort>               uniqueIndices;
        [NativeDisableContainerSafetyRestriction]//[WriteOnly]
        public NativeList<PlaneIndexLengthPair> loopLengths;


        #region Sort
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        float3 FindPolygonCentroid(ushort* indicesPtr, int offset, int indicesCount)
        {
            var centroid = float3.zero;
            var vertices = vertexSoup.vertices;
            for (int i = 0; i < indicesCount; i++, offset++)
                centroid += vertices[indicesPtr[offset]];
            return centroid / indicesCount;
        }

        // TODO: sort by using plane information instead of unreliable floating point math ..
        // TODO: make this work on non-convex polygons
        void SortIndices(ushort* indicesPtr, int offset, int indicesCount, float3 normal)
        {
            // There's no point in trying to sort a point or a line 
            if (indicesCount < 3)
                return;

            var vertices = vertexSoup.vertices;

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

            var centroid = FindPolygonCentroid(indicesPtr, offset, indicesCount);
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
                var va = (float3)vertices[indicesPtr[offset + (left + right) / 2]];
                while (true)
                {
                    var a_angle = math.atan2(math.dot(tangentX, va) - center.x, math.dot(tangentY, va) - center.y);

                    {
                        var vb = (float3)vertices[indicesPtr[offset + left]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (b_angle > a_angle)
                        {
                            left++;
                            vb = (float3)vertices[indicesPtr[offset + left]];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    {
                        var vb = (float3)vertices[indicesPtr[offset + right]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (a_angle > b_angle)
                        {
                            right--;
                            vb = (float3)vertices[indicesPtr[offset + right]];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    if (left <= right)
                    {
                        if (left != right)
                        {
                            var t = indicesPtr[offset + left];
                            indicesPtr[offset + left] = indicesPtr[offset + right];
                            indicesPtr[offset + right] = t;
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
            //NativeSortExtension.Sort(foundIndices); // <- we can't if it's readonly!

            var previousPlaneIndex  = foundIndices[0].planeIndex;
            var previousVertexIndex = foundIndices[0].vertexIndex;
            uniqueIndices.Add(previousVertexIndex);
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
                    loopLengths.Add(new PlaneIndexLengthPair() { polygonLength = (ushort)(uniqueIndices.Length - loopStart), planeIndex = previousPlaneIndex });
                    loopStart = uniqueIndices.Length;
                }

                uniqueIndices.Add(vertexIndex);
                previousVertexIndex = vertexIndex;
                previousPlaneIndex = planeIndex;
            }
            loopLengths.Add(new PlaneIndexLengthPair() { polygonLength = (ushort)(uniqueIndices.Length - loopStart), planeIndex = previousPlaneIndex });

            // TODO: do in separate pass?

            var indicesPtr = (ushort*)uniqueIndices.GetUnsafeReadOnlyPtr();

            int offset = 0;
            for (int n = 0; n < loopLengths.Length; n++)
            {
                var loopLength = loopLengths[n];
                var planeIndex = loopLength.planeIndex;
                SortIndices(indicesPtr, offset, loopLength.polygonLength, surfaceCategory[planeIndex].worldPlane.xyz);
                offset += loopLength.polygonLength;
            }
        }
    }

    // TODO: make burstable (somehow)
    unsafe struct CreateLoopsJob : IJob
    {
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<ushort>                uniqueIndices;
        [ReadOnly] public NativeList<PlaneIndexLengthPair>  loopLengths;
        [ReadOnly] public NativeArray<SurfaceInfo>          surfaceCategories;

        [ReadOnly] public CSGTreeBrush      brush;

        [WriteOnly] public List<Loop>[]     outputSurfaces;

        public void Execute()
        {
            int offset = 0;

            var indicesPtr              = (ushort*)uniqueIndices.GetUnsafeReadOnlyPtr();
            var loopLengthsPtr          = (PlaneIndexLengthPair*)loopLengths.GetUnsafeReadOnlyPtr();
            var loopLengthsLength       = loopLengths.Length;
            var surfaceCategoriesPtr    = (SurfaceInfo*)surfaceCategories.GetUnsafeReadOnlyPtr();
            
            for (int n = 0; n < loopLengthsLength; n++)
            {
                var planeIndexLength = loopLengthsPtr[n];
                var loopLength = planeIndexLength.polygonLength;
                if (loopLength > 2)
                {
                    var basePlaneIndex  = planeIndexLength.planeIndex;
                    var surfaceCategory = surfaceCategoriesPtr[basePlaneIndex];
                    var hole = new Loop()
                    {
                        info = surfaceCategory
                    };
                    hole.SetIndices(indicesPtr, offset, loopLength);
                    outputSurfaces[basePlaneIndex].Add(hole);
                } 
                offset += loopLength;
            }
        }
    }
}