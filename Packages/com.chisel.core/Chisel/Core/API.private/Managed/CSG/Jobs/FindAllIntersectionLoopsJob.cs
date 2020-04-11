using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Chisel.Core.LowLevel.Unsafe;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION    
    public struct BrushSurfacePair : IEquatable<BrushSurfacePair>
    {
        public int brushNodeIndex0;
        public int brushNodeIndex1;
        public int basePlaneIndex;

        #region Equals
        public bool Equals(BrushSurfacePair other)
        {
            return (brushNodeIndex0 == other.brushNodeIndex0 && brushNodeIndex1 == other.brushNodeIndex1 && basePlaneIndex == other.basePlaneIndex);
        }

        public override bool Equals(object obj)
        {
            if (!(obj is BrushSurfacePair))
                return false;
            return base.Equals((BrushSurfacePair)obj);
        }
        #endregion

        public override int GetHashCode()
        {
            return (int)math.hash(new int3(brushNodeIndex0, brushNodeIndex1, basePlaneIndex));
        }
    }


    [BurstCompile(CompileSynchronously = true)]
    public unsafe struct FindAllIntersectionLoopsJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kDistanceEpsilon        = CSGManagerPerformCSG.kDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>> brushWorldPlanes;
        [NoAlias, ReadOnly] public NativeArray<BlobAssetReference<BrushPairIntersection>> intersectingBrushes;

        [NoAlias, WriteOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>.ParallelWriter outputSurfaces;


        struct PlaneIndexPairSort : IComparer<PlaneVertexIndexPair>
        {
            public int Compare(PlaneVertexIndexPair x, PlaneVertexIndexPair y)
            {
                if (x.planeIndex > y.planeIndex)
                    return 1;
                if (x.planeIndex == y.planeIndex)
                {
                    if (x.vertexIndex == y.vertexIndex)
                        return 0;
                    if (x.vertexIndex < y.vertexIndex)
                        return 1;
                }
                return -1;
            }
        }

        struct PlaneVertexIndexPair
        {
            public ushort planeIndex;
            public ushort vertexIndex;
        }

        struct PlaneIndexOffsetLength
        {
            public ushort length;
            public ushort offset;
            public ushort planeIndex;
        }

        #region Sort
        static float3 FindPolygonCentroid(float3* vertices, ushort* indices, int offset, int indicesCount)
        {
            var centroid = float3.zero;
            for (int i = 0; i < indicesCount; i++, offset++)
                centroid += vertices[indices[offset]];
            return centroid / indicesCount;
        }

        // TODO: sort by using plane information instead of unreliable floating point math ..
        unsafe static void SortIndices(float3* vertices, int2* sortedStack, ushort* indices, int offset, int indicesCount, float3 normal)
        {
            // There's no point in trying to sort a point or a line 
            if (indicesCount < 3)
                return;


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

            var centroid = FindPolygonCentroid(vertices, indices, offset, indicesCount);
            var center = new float2(math.dot(tangentX, centroid), // distance in direction of tangentX
                                    math.dot(tangentY, centroid)); // distance in direction of tangentY


            var sortedStackLength = 1;
            sortedStack[0] = new int2(0, indicesCount - 1);
            while (sortedStackLength > 0)
            {
                var top = sortedStack[sortedStackLength - 1];
                sortedStackLength--;
                var l = top.x;
                var r = top.y;
                var left = l;
                var right = r;
                var va = vertices[indices[offset + (left + right) / 2]];
                while (true)
                {
                    var a_angle = math.atan2(math.dot(tangentX, va) - center.x, math.dot(tangentY, va) - center.y);

                    {
                        var vb = vertices[indices[offset + left]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (b_angle > a_angle)
                        {
                            left++;
                            vb = vertices[indices[offset + left]];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    {
                        var vb = vertices[indices[offset + right]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (a_angle > b_angle)
                        {
                            right--;
                            vb = vertices[indices[offset + right]];
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
                    sortedStack[sortedStackLength] = new int2(l, right);
                    sortedStackLength++;
                }
                if (left < r)
                {
                    sortedStack[sortedStackLength] = new int2(left, r);
                    sortedStackLength++;
                }
            }
        }
        #endregion

        static void FindInsideVertices(ref BlobArray<float3>    usedVertices0,  
                                       ref BlobArray<int>       intersectingPlaneIndices0, 
                                       ref BlobArray<float4>    intersectingPlanes0, 
                                       ref BlobArray<float4>    intersectingPlanes1,
                                       float4x4                 nodeToTreeSpaceMatrix0,
                                       float4x4                 vertexToLocal0,
                                       VertexSoup               vertexSoup0,
                                       PlaneVertexIndexPair*    foundIndices0,
                                       ref int                  foundIndices0Length)
        {
            for (int j = 0; j < usedVertices0.Length; j++)
            {
                var brushVertex1 = usedVertices0[j];
                var localVertex1 = math.mul(vertexToLocal0, new float4(brushVertex1, 1));
                if (CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes1, localVertex1))
                    continue;
                    
                var worldVertex = math.mul(nodeToTreeSpaceMatrix0, new float4(brushVertex1, 1));
                var worldVertexIndex = -1;

                // TODO: optimize this, we already know these vertices are ON the planes of this brush, just not which: this can be precalculated
                for (int i = 0; i < intersectingPlaneIndices0.Length; i++)
                {
                    // only use a plane when the vertex is ON it
                    var distance = math.dot(intersectingPlanes0[i], localVertex1);
                    if (distance >= -kPlaneDistanceEpsilon && distance <= kPlaneDistanceEpsilon) // Note: this is false on NaN/Infinity, so don't invert
                    {
                        var planeIndex = intersectingPlaneIndices0[i];
                        if (worldVertexIndex == -1)
                            worldVertexIndex = vertexSoup0.AddNoResize(worldVertex.xyz);
                        foundIndices0[foundIndices0Length] = new PlaneVertexIndexPair() { planeIndex = (ushort)planeIndex, vertexIndex = (ushort)worldVertexIndex };
                        foundIndices0Length++;
                    }
                }
            }
        }
        
        static void FindIntersectionVertices(ref BlobArray<float4>      intersectingPlanes0,
                                             ref BlobArray<float4>      intersectingPlanes1,
                                             ref BlobArray<PlanePair>   usedPlanePairs1,
                                             ref BlobArray<int>         intersectingPlaneIndices0,
                                             float4x4                   nodeToTreeSpaceMatrix0,
                                             VertexSoup                 vertexSoup0,
                                             VertexSoup                 vertexSoup1,
                                             PlaneVertexIndexPair*      foundIndices0,
                                             ref int                    foundIndices0Length,
                                             PlaneVertexIndexPair*      foundIndices1,
                                             ref int                    foundIndices1Length)
        {
            for (int i = 0; i < usedPlanePairs1.Length; i++)
            {
                var planePair   = usedPlanePairs1[i];
                var edgeVertex0 = planePair.edgeVertex0;
                var edgeVertex1 = planePair.edgeVertex1;
                var plane0      = planePair.plane0;
                var plane1      = planePair.plane1;

                for (int j = 0; j < intersectingPlanes0.Length; j++)
                {
                    var plane2 = intersectingPlanes0[j];
                    if (math.abs(math.dot(plane2, edgeVertex0)) < kDistanceEpsilon &&
                        math.abs(math.dot(plane2, edgeVertex1)) < kDistanceEpsilon)
                    {
                        // plane2 is aligned with edge, so we skip it
                        continue;
                    }

                    var localVertex = new float4(PlaneExtensions.Intersection(plane2, plane0, plane1), 1);

                    // TODO: since we're using a pair in the outer loop, we could also determine which 
                    //       2 planes it intersects at both ends and just check those two planes ..

                    // NOTE: for brush2, the intersection will always be only on two planes
                    //       UNLESS it's a corner vertex along that edge (we can compare to the two vertices)
                    //       in which case we could use a pre-calculated list of planes ..
                    //       OR when the intersection is outside of the edge ..


                    if (!CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes0, localVertex) &&
                        !CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes1, localVertex))
                    {
                        var planeIndex0 = (ushort)planePair.planeIndex0;
                        var planeIndex1 = (ushort)planePair.planeIndex1;
                        var planeIndex2 = (ushort)intersectingPlaneIndices0[j];

                        // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex 
                        //       to ensure they are identical
                        var worldVertex = math.mul(nodeToTreeSpaceMatrix0, localVertex).xyz;
                        var vertexIndex1 = vertexSoup0.AddNoResize(worldVertex);
                        var vertexIndex2 = vertexSoup1.AddNoResize(worldVertex);

                        foundIndices0[foundIndices0Length] = new PlaneVertexIndexPair() { planeIndex = planeIndex2, vertexIndex = vertexIndex1 };
                        foundIndices0Length++;

                        foundIndices1[foundIndices1Length] = new PlaneVertexIndexPair() { planeIndex = planeIndex0, vertexIndex = vertexIndex2 };
                        foundIndices1Length++;

                        foundIndices1[foundIndices1Length] = new PlaneVertexIndexPair() { planeIndex = planeIndex1, vertexIndex = vertexIndex2 };
                        foundIndices1Length++;
                    }
                }
            }
        }


        static void GenerateLoop(int                                    brushNodeIndex0,
                                 int                                    brushNodeIndex1,
                                 ref BlobArray<SurfaceInfo>             surfaceInfos,
                                 BlobAssetReference<BrushWorldPlanes>   brushWorldPlanes,
                                 PlaneVertexIndexPair*                  foundIndices0,
                                 ref int                                foundIndices0Length,
                                 VertexSoup                             vertexSoup0,
                                 NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>.ParallelWriter outputSurfaces)
        {
            var planeIndexOffsetsLength = 0;
            var planeIndexOffsets       = stackalloc PlaneIndexOffsetLength[foundIndices0Length];
            var uniqueIndicesLength     = 0;
            var uniqueIndices           = stackalloc ushort[foundIndices0Length];
                
                
            var sortedFoundIndices = foundIndices0;
            var sorter = new PlaneIndexPairSort();
            NativeSortExtension.Sort<PlaneVertexIndexPair, PlaneIndexPairSort>(sortedFoundIndices, foundIndices0Length, sorter);


            // Now that our indices are sorted by planeIndex, we can segment them by start/end offset
            var previousPlaneIndex  = sortedFoundIndices[0].planeIndex;
            var previousVertexIndex = sortedFoundIndices[0].vertexIndex;
            uniqueIndices[uniqueIndicesLength] = previousVertexIndex;
            uniqueIndicesLength++;
            var loopStart = 0;
            for (int i = 1; i < foundIndices0Length; i++)
            {
                var indices     = sortedFoundIndices[i];

                var planeIndex  = indices.planeIndex;
                var vertexIndex = indices.vertexIndex;

                // TODO: why do we have soooo many duplicates sometimes?
                if (planeIndex  == previousPlaneIndex &&
                    vertexIndex == previousVertexIndex)
                    continue;

                if (planeIndex != previousPlaneIndex)
                {
                    var currLength = (uniqueIndicesLength - loopStart);
                    if (currLength > 2)
                    {
                        planeIndexOffsets[planeIndexOffsetsLength] = new PlaneIndexOffsetLength()
                        {
                            length = (ushort)currLength,
                            offset = (ushort)loopStart,
                            planeIndex = previousPlaneIndex
                        };
                        planeIndexOffsetsLength++;
                    }
                    loopStart = uniqueIndicesLength;
                }

                uniqueIndices[uniqueIndicesLength] = vertexIndex;
                uniqueIndicesLength++;
                previousVertexIndex = vertexIndex;
                previousPlaneIndex = planeIndex;
            }
            {
                var currLength = (uniqueIndicesLength - loopStart);
                if (currLength > 2)
                {
                    planeIndexOffsets[planeIndexOffsetsLength] = new PlaneIndexOffsetLength()
                    {
                        length = (ushort)currLength,
                        offset = (ushort)loopStart,
                        planeIndex = previousPlaneIndex
                    };
                    planeIndexOffsetsLength++;
                }
            }

            var maxLength = 0;
            for (int i = 0; i < planeIndexOffsetsLength; i++)
                maxLength = math.max(maxLength, planeIndexOffsets[i].length);

            // For each segment, we now sort our vertices within each segment, 
            // making the assumption that they are convex
            var sortedStack = stackalloc int2[maxLength * 2];
            var vertices    = vertexSoup0.GetUnsafeReadOnlyPtr();
            for (int n = planeIndexOffsetsLength - 1; n >= 0; n--)
            {
                var planeIndexOffset    = planeIndexOffsets[n];
                var length              = planeIndexOffset.length;
                var offset              = planeIndexOffset.offset;
                var planeIndex          = planeIndexOffset.planeIndex;
                    
                // TODO: use plane information instead
                SortIndices(vertices, sortedStack, uniqueIndices, offset, length, brushWorldPlanes.Value.worldPlanes[planeIndex].xyz);
            }

            for (int j = 0; j < planeIndexOffsetsLength; j++)
            { 
                var planeIndexLength    = planeIndexOffsets[j];
                var offset              = planeIndexLength.offset;
                var loopLength          = planeIndexLength.length;
                var basePlaneIndex      = planeIndexLength.planeIndex;
                var surfaceInfo         = surfaceInfos[basePlaneIndex];

                var builder = new BlobBuilder(Allocator.Temp);
                ref var root = ref builder.ConstructRoot<BrushIntersectionLoop>();
                root.surfaceInfo = surfaceInfo;
                var dstVertices = builder.Allocate(ref root.loopVertices, loopLength);
                var srcVertices = vertexSoup0.GetUnsafeReadOnlyPtr();
                for (int d = 0; d < loopLength; d++)
                    dstVertices[d] = srcVertices[uniqueIndices[offset + d]];

                var outputSurface = builder.CreateBlobAssetReference<BrushIntersectionLoop>(Allocator.Persistent);
                builder.Dispose();

                outputSurfaces.TryAdd(new BrushSurfacePair()
                {
                    brushNodeIndex0 = brushNodeIndex0,
                    brushNodeIndex1 = brushNodeIndex1,
                    basePlaneIndex = basePlaneIndex
                }, outputSurface);
            }
        }

        public void Execute(int index)
        {
            if (index >= intersectingBrushes.Length)
                return;

            var intersectionAsset               = intersectingBrushes[index];
            ref var intersection                = ref intersectionAsset.Value;
            ref var brushPairIntersection0      = ref intersection.brushes[0];
            ref var brushPairIntersection1      = ref intersection.brushes[1];
            var brushNodeIndex0                 = brushPairIntersection0.brushNodeIndex;
            var brushNodeIndex1                 = brushPairIntersection1.brushNodeIndex;

            int insideVerticesStream0Capacity   = math.max(1, brushPairIntersection0.usedVertices.Length);
            int insideVerticesStream1Capacity   = math.max(1, brushPairIntersection1.usedVertices.Length);
            int intersectionStream0Capacity     = math.max(1, brushPairIntersection1.usedPlanePairs.Length) * brushPairIntersection0.localSpacePlanes0.Length;
            int intersectionStream1Capacity     = math.max(1, brushPairIntersection0.usedPlanePairs.Length) * brushPairIntersection1.localSpacePlanes0.Length;
            int foundIndices0Capacity           = intersectionStream0Capacity + (2 * intersectionStream1Capacity) + (brushPairIntersection0.localSpacePlanes0.Length * insideVerticesStream0Capacity);
            int foundIndices1Capacity           = intersectionStream1Capacity + (2 * intersectionStream0Capacity) + (brushPairIntersection1.localSpacePlanes0.Length * insideVerticesStream1Capacity);

            var foundIndices0           = stackalloc PlaneVertexIndexPair[foundIndices0Capacity];
            var foundIndices1           = stackalloc PlaneVertexIndexPair[foundIndices1Capacity];
            var foundIndices0Length     = 0;
            var foundIndices1Length     = 0;
            //var foundIndices0   = new NativeList<PlaneVertexIndexPair>(foundIndices0Capacity, Allocator.Temp);
            //var foundIndices1   = new NativeList<PlaneVertexIndexPair>(foundIndices1Capacity, Allocator.Temp);

            // TODO: fill them with original brush vertices so that they're always snapped to these
            var vertexSoup0     = new VertexSoup(foundIndices0Capacity, Allocator.Temp);
            var vertexSoup1     = new VertexSoup(foundIndices1Capacity, Allocator.Temp);

            // First find vertices from other brush that are inside the other brush, so that any vertex we 
            // find during the intersection part will be snapped to those vertices and not the other way around

            // TODO: when all vertices of a polygon are inside the other brush, don't bother intersecting it.
            //       same when two planes overlap each other ...

            // Now find all the intersection vertices
            if (intersection.type == IntersectionType.Intersection)
            { 
                if (brushPairIntersection1.usedPlanePairs.Length > 0)
                {
                    FindIntersectionVertices(ref intersectionAsset.Value.brushes[0].localSpacePlanes0,
                                             ref intersectionAsset.Value.brushes[1].localSpacePlanes0,
                                             ref intersectionAsset.Value.brushes[1].usedPlanePairs,
                                             ref intersectionAsset.Value.brushes[0].localSpacePlaneIndices0,
                                             intersectionAsset.Value.brushes[0].nodeToTreeSpace,
                                             vertexSoup0,
                                             vertexSoup1,
                                             foundIndices0, ref foundIndices0Length,
                                             foundIndices1, ref foundIndices1Length);
                }

                if (brushPairIntersection0.usedPlanePairs.Length > 0)
                {
                    FindIntersectionVertices(ref intersectionAsset.Value.brushes[1].localSpacePlanes0,
                                             ref intersectionAsset.Value.brushes[0].localSpacePlanes0,
                                             ref intersectionAsset.Value.brushes[0].usedPlanePairs,
                                             ref intersectionAsset.Value.brushes[1].localSpacePlaneIndices0,
                                             intersectionAsset.Value.brushes[0].nodeToTreeSpace,
                                             vertexSoup1,
                                             vertexSoup0,
                                             foundIndices1, ref foundIndices1Length,
                                             foundIndices0, ref foundIndices0Length);
                }
            }

            // Find all vertices of brush0 that are inside brush1, and put their intersections into the appropriate loops
            if (foundIndices0Length > 0 &&
                brushPairIntersection0.usedVertices.Length > 0)
            {
                FindInsideVertices(ref intersectionAsset.Value.brushes[0].usedVertices,
                                   ref intersectionAsset.Value.brushes[0].localSpacePlaneIndices0,
                                   ref intersectionAsset.Value.brushes[0].localSpacePlanes0,
                                   ref intersectionAsset.Value.brushes[1].localSpacePlanes0,
                                   intersectionAsset.Value.brushes[0].nodeToTreeSpace,
                                   float4x4.identity,
                                   vertexSoup0,
                                   foundIndices0, ref foundIndices0Length);
            }

            // Find all vertices of brush1 that are inside brush0, and put their intersections into the appropriate loops
            if (foundIndices1Length > 0 && 
                brushPairIntersection1.usedVertices.Length > 0)
            {
                FindInsideVertices(ref intersectionAsset.Value.brushes[1].usedVertices,
                                   ref intersectionAsset.Value.brushes[1].localSpacePlaneIndices0,
                                   ref intersectionAsset.Value.brushes[1].localSpacePlanes0,
                                   ref intersectionAsset.Value.brushes[0].localSpacePlanes0,
                                   intersectionAsset.Value.brushes[1].nodeToTreeSpace,
                                   intersectionAsset.Value.brushes[1].toOtherBrushSpace,
                                   vertexSoup1,
                                   foundIndices1, ref foundIndices1Length);
            }


            if (foundIndices0Length >= 3)
            {
                GenerateLoop(brushNodeIndex0,
                             brushNodeIndex1,
                             ref intersectionAsset.Value.brushes[0].surfaceInfos,
                             brushWorldPlanes[brushNodeIndex0],
                             foundIndices0, ref foundIndices0Length,
                             vertexSoup0,
                             outputSurfaces);
            }

            if (foundIndices1Length >= 3)
            {
                GenerateLoop(brushNodeIndex1,
                             brushNodeIndex0,
                             ref intersectionAsset.Value.brushes[1].surfaceInfos,
                             brushWorldPlanes[brushNodeIndex1],
                             foundIndices1, ref foundIndices1Length,
                             vertexSoup1,
                             outputSurfaces);
            }

            //foundIndices0.Dispose();
            //foundIndices1.Dispose();

            vertexSoup0.Dispose();
            vertexSoup1.Dispose();

            intersectionAsset.Dispose();
        }
    }
#endif
}
