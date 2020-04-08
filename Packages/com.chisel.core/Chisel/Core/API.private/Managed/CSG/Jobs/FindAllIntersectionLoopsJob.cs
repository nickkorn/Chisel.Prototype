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
    [BurstCompile(CompileSynchronously = true)]
    public unsafe struct FindAllIntersectionLoopsJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kDistanceEpsilon        = CSGManagerPerformCSG.kDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        [NoAlias,ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>  brushWorldPlanes;
        [NoAlias,ReadOnly] public NativeArray<BlobAssetReference<BrushPairIntersection>>    intersectingBrushes;

        [NoAlias,WriteOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>.ParallelWriter outputSurfaces;

        // TODO: move all the code that used to be in separate jobs here, instead of calling "Execute" on all of these

        struct LocalWorldPair
        {
            public float4 localVertex1;
            public float4 worldVertex;
        }
        
        #region Sort
        static float3 FindPolygonCentroid(float3* vertices, NativeList<ushort> indices, int offset, int indicesCount)
        {
            var centroid = float3.zero;
            for (int i = 0; i < indicesCount; i++, offset++)
                centroid += vertices[indices[offset]];
            return centroid / indicesCount;
        }

        // TODO: sort by using plane information instead of unreliable floating point math ..
        unsafe void SortIndices(float3* vertices, int2* sortedStack, NativeList<ushort> indices, int offset, int indicesCount, float3 normal)
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
                    
            // TODO: allocate per intersection, perform all calculations/sorts, THEN create ALL surface-loops and assign indices
            var foundIndices0   = new NativeList<PlaneVertexIndexPair>(foundIndices0Capacity, Allocator.Temp);
            var foundIndices1   = new NativeList<PlaneVertexIndexPair>(foundIndices1Capacity, Allocator.Temp);

            // TODO: fill them with original brush vertices so that they're always snapped to these
            var vertexSoup0     = new VertexSoup(foundIndices0Capacity, Allocator.Temp);
            var vertexSoup1     = new VertexSoup(foundIndices1Capacity, Allocator.Temp);

            // First find vertices from other brush that are inside the other brush, so that any vertex we 
            // find during the intersection part will be snapped to those vertices and not the other way around

            // TODO: when all vertices of a polygon are inside the other brush, don't bother intersecting it.
            //       same when two planes overlap each other ...

            // Find all vertices of brush1 that are inside brush2, and put their intersections into the appropriate loops
            if (brushPairIntersection0.usedVertices.Length > 0)
            {
                var insideVerticesStream0           = new NativeList<LocalWorldPair>(insideVerticesStream0Capacity, Allocator.Temp);
                ref var usedVertices0               = ref intersectionAsset.Value.brushes[0].usedVertices;
                ref var intersectingPlaneIndices0   = ref intersectionAsset.Value.brushes[0].localSpacePlaneIndices0;
                ref var intersectingPlanes1         = ref intersectionAsset.Value.brushes[1].localSpacePlanes0;
                ref var intersectingPlanes0         = ref intersectionAsset.Value.brushes[0].localSpacePlanes0;
                ref var allVertices0                = ref intersectionAsset.Value.brushes[0].blobMesh.Value.vertices;

                var nodeToTreeSpaceMatrix0  = intersectionAsset.Value.brushes[0].transformation.nodeToTree;
                var vertexToLocal0          = float4x4.identity;
                for (int j = 0; j < usedVertices0.Length; j++)
                {
                    var vertexIndex1 = usedVertices0[j];
                    var brushVertex1 = new float4(allVertices0[vertexIndex1], 1);
                    var localVertex1 = math.mul(vertexToLocal0, brushVertex1);
                    if (!CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes1, localVertex1))
                    { 
                        var worldVertex = math.mul(nodeToTreeSpaceMatrix0, brushVertex1);
                        insideVerticesStream0.AddNoResize(new LocalWorldPair() { localVertex1 = localVertex1, worldVertex = worldVertex });
                    }
                }
                for (int j = 0; j < insideVerticesStream0.Length; j++)
                {
                    var localVertex1    = insideVerticesStream0[j].localVertex1;
                    var worldVertex     = insideVerticesStream0[j].worldVertex;

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
                            foundIndices0.AddNoResize(new PlaneVertexIndexPair() { planeIndex = (ushort)planeIndex, vertexIndex = (ushort)worldVertexIndex });
                        }
                    }
                }
                insideVerticesStream0.Dispose();
            }

            if (brushPairIntersection1.usedVertices.Length > 0)
            {
                var insideVerticesStream1 = new NativeList<LocalWorldPair>(insideVerticesStream1Capacity, Allocator.Temp);
                ref var intersectingPlanes0         = ref intersectionAsset.Value.brushes[0].localSpacePlanes0;
                ref var intersectingPlanes1         = ref intersectionAsset.Value.brushes[1].localSpacePlanes0;
                ref var intersectingPlaneIndices1   = ref intersectionAsset.Value.brushes[1].localSpacePlaneIndices0;
                ref var allVertices0                = ref intersectionAsset.Value.brushes[1].blobMesh.Value.vertices;
                ref var usedVertices1               = ref intersectionAsset.Value.brushes[1].usedVertices;
                var nodeToTreeSpaceMatrix1          = intersectionAsset.Value.brushes[1].transformation.nodeToTree;
                var vertexToLocal0                  = intersectionAsset.Value.brushes[1].toOtherBrushSpace;
                for (int j = 0; j < usedVertices1.Length; j++)
                {
                    var vertexIndex1 = usedVertices1[j];
                    var brushVertex1 = new float4(allVertices0[vertexIndex1], 1);
                    var localVertex1 = math.mul(vertexToLocal0, brushVertex1);
                    if (!CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes0, localVertex1))
                    { 
                        var worldVertex = math.mul(nodeToTreeSpaceMatrix1, brushVertex1);
                        insideVerticesStream1.AddNoResize(new LocalWorldPair() { localVertex1 = localVertex1, worldVertex = worldVertex });
                    }
                }
                for (int j = 0; j < insideVerticesStream1.Length; j++)
                { 
                    var localVertex1    = insideVerticesStream1[j].localVertex1;
                    var worldVertex     = insideVerticesStream1[j].worldVertex;

                    var worldVertexIndex = -1;
                    // TODO: optimize this, we already know these vertices are ON the planes of this brush, just not which: this can be precalculated
                    for (int i = 0; i < intersectingPlaneIndices1.Length; i++)
                    {
                        // only use a plane when the vertex is ON it
                        var distance = math.dot(intersectingPlanes1[i], localVertex1);
                        if (distance >= -kPlaneDistanceEpsilon && distance <= kPlaneDistanceEpsilon) // Note: this is false on NaN/Infinity, so don't invert
                        {
                            var planeIndex = intersectingPlaneIndices1[i];
                            if (worldVertexIndex == -1)
                                worldVertexIndex = vertexSoup1.AddNoResize(worldVertex.xyz);
                            foundIndices1.AddNoResize(new PlaneVertexIndexPair() { planeIndex = (ushort)planeIndex, vertexIndex = (ushort)worldVertexIndex });
                        }
                    }
                }
                insideVerticesStream1.Dispose();
            }

            // Now find all the intersection vertices
            if (intersection.type == IntersectionType.Intersection &&
                brushPairIntersection1.usedPlanePairs.Length > 0)
            {
                // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                ref var intersectingPlanes0         = ref intersectionAsset.Value.brushes[0].localSpacePlanes0;
                ref var intersectingPlanes1         = ref intersectionAsset.Value.brushes[1].localSpacePlanes0;
                ref var usedPlanePairs1             = ref intersectionAsset.Value.brushes[1].usedPlanePairs;
                ref var intersectingPlaneIndices0   = ref intersectionAsset.Value.brushes[0].localSpacePlaneIndices0;
                var nodeToTreeSpaceMatrix0          = intersectionAsset.Value.brushes[0].transformation.nodeToTree;
                for (int j = 0; j < intersectingPlanes0.Length; j++)
                {

                    var plane2 = intersectingPlanes0[j];

                    for (int i = 0; i < usedPlanePairs1.Length; i++)
                    {
                        var planePair = usedPlanePairs1[i];

                        var plane0 = planePair.plane0;
                        var plane1 = planePair.plane1;

                        // FIXME: Fix situation when plane intersects edge but is not identical to either planes of the edge ...
                        if (!(math.abs(plane0.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) > kNormalEpsilon) &&
                            !(math.abs(plane1.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) > kNormalEpsilon) &&

                            !(math.abs(plane0.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) < -kNormalEpsilon) &&
                            !(math.abs(plane1.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) < -kNormalEpsilon))
                        {

#if true
                            var edgeVertex0 = planePair.edgeVertex0;
                            var edgeVertex1 = planePair.edgeVertex1;

                            // Fixes situation when plane intersects edge but is not identical to either planes of the edge ...
                            if (math.abs(math.dot(plane2, edgeVertex0)) < kDistanceEpsilon)
                            {
                                if (math.abs(math.dot(plane2, edgeVertex1)) < kDistanceEpsilon)
                                {
                                    // plane2 is aligned with edge, so we ignore this
                                    continue;
                                }
                                /*
                                // plane2 goes straight through vertex0, don't need to calculate intersection
                                // but actually make sure it's not outside the brush that plane2 belongs to
                                if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes0, edgeVertex0))
                                {
                                    var worldVertex = math.mul(nodeToTreeSpaceMatrix0, edgeVertex0).xyz;
                                    foundVertices.Write(new VertexAndPlanePair()
                                    {
                                        vertex = worldVertex,
                                        plane0 = (ushort)planePair.planeIndex0,
                                        plane1 = (ushort)planePair.planeIndex1
                                    });
                                }*/
                            }/* else
                            if (math.abs(math.dot(plane2, edgeVertex1)) < kPlaneDistanceEpsilon)
                            {
                                // plane2 goes straight through vertex1, don't need to calculate intersection
                                // but actually make sure it's not outside the brush that plane2 belongs to
                                if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes0, edgeVertex1))
                                {
                                    var worldVertex = math.mul(nodeToTreeSpaceMatrix0, edgeVertex1).xyz;
                                    foundVertices.Write(new VertexAndPlanePair()
                                    {
                                        vertex = worldVertex,
                                        plane0 = (ushort)planePair.planeIndex0,
                                        plane1 = (ushort)planePair.planeIndex1
                                    });
                                }
                            }*/
#endif

                            var localVertex = new float4(PlaneExtensions.Intersection(plane2, plane0, plane1), 1);

                            // TODO: since we're using a pair in the outer loop, we could also determine which 
                            //       2 planes it intersects at both ends and just check those two planes ..

                            // NOTE: for brush2, the intersection will always be only on two planes
                            //       UNLESS it's a corner vertex along that edge (we can compare to the two vertices)
                            //       in which case we could use a pre-calculated list of planes ..
                            //       OR when the intersection is outside of the edge ..

                            var worldVertex = math.mul(nodeToTreeSpaceMatrix0, localVertex).xyz;

                            if (!CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes1, localVertex) &&
                                !CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes0, localVertex))
                            {
                                var planeIndex0 = (ushort)planePair.planeIndex0;
                                var planeIndex1 = (ushort)planePair.planeIndex1;
                                var planeIndex2 = (ushort)intersectingPlaneIndices0[j];

                                // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex 
                                //       to ensure they are identical
                                var vertexIndex1 = vertexSoup0.AddNoResize(worldVertex);
                                worldVertex = vertexSoup0[vertexIndex1];
                                var vertexIndex2 = vertexSoup1.AddNoResize(worldVertex);

                                foundIndices0.AddNoResize(new PlaneVertexIndexPair() { planeIndex = planeIndex2, vertexIndex = vertexIndex1 });
                                foundIndices1.AddNoResize(new PlaneVertexIndexPair() { planeIndex = planeIndex0, vertexIndex = vertexIndex2 });
                                foundIndices1.AddNoResize(new PlaneVertexIndexPair() { planeIndex = planeIndex1, vertexIndex = vertexIndex2 });
                            }
                        }
                    }
                }
            }

            if (intersection.type == IntersectionType.Intersection &&
                brushPairIntersection0.usedPlanePairs.Length > 0)
            {
                // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                ref var intersectingPlanes0         = ref intersectionAsset.Value.brushes[1].localSpacePlanes0;
                ref var intersectingPlanes1         = ref intersectionAsset.Value.brushes[0].localSpacePlanes0;
                ref var usedPlanePairs1             = ref intersectionAsset.Value.brushes[0].usedPlanePairs;
                ref var intersectingPlaneIndices0   = ref intersectionAsset.Value.brushes[1].localSpacePlaneIndices0;
                var nodeToTreeSpaceMatrix0          = intersectionAsset.Value.brushes[0].transformation.nodeToTree;
                for (int j = 0; j < intersectingPlanes0.Length; j++)
                {

                    var plane2 = intersectingPlanes0[j];

                    for (int i = 0; i < usedPlanePairs1.Length; i++)
                    {
                        var planePair = usedPlanePairs1[i];

                        var plane0 = planePair.plane0;
                        var plane1 = planePair.plane1;

                        // FIXME: Fix situation when plane intersects edge but is not identical to either planes of the edge ...
                        if (!(math.abs(plane0.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) > kNormalEpsilon) &&
                            !(math.abs(plane1.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) > kNormalEpsilon) &&

                            !(math.abs(plane0.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) < -kNormalEpsilon) &&
                            !(math.abs(plane1.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) < -kNormalEpsilon))
                        {
#if true
                            var edgeVertex0 = planePair.edgeVertex0;
                            var edgeVertex1 = planePair.edgeVertex1;

                            // Fixes situation when plane intersects edge but is not identical to either planes of the edge ...
                            if (math.abs(math.dot(plane2, edgeVertex0)) < kDistanceEpsilon)
                            {
                                if (math.abs(math.dot(plane2, edgeVertex1)) < kDistanceEpsilon)
                                {
                                    // plane2 is aligned with edge, so we ignore this
                                    continue;
                                }
                                /*
                                // plane2 goes straight through vertex0, don't need to calculate intersection
                                // but actually make sure it's not outside the brush that plane2 belongs to
                                if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes0, edgeVertex0))
                                {
                                    var worldVertex = math.mul(nodeToTreeSpaceMatrix0, edgeVertex0).xyz;
                                    foundVertices.Write(new VertexAndPlanePair()
                                    {
                                        vertex = worldVertex,
                                        plane0 = (ushort)planePair.planeIndex0,
                                        plane1 = (ushort)planePair.planeIndex1
                                    });
                                }*/
                            }/* else
                            if (math.abs(math.dot(plane2, edgeVertex1)) < kPlaneDistanceEpsilon)
                            {
                                // plane2 goes straight through vertex1, don't need to calculate intersection
                                // but actually make sure it's not outside the brush that plane2 belongs to
                                if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes0, edgeVertex1))
                                {
                                    var worldVertex = math.mul(nodeToTreeSpaceMatrix0, edgeVertex1).xyz;
                                    foundVertices.Write(new VertexAndPlanePair()
                                    {
                                        vertex = worldVertex,
                                        plane0 = (ushort)planePair.planeIndex0,
                                        plane1 = (ushort)planePair.planeIndex1
                                    });
                                }
                            }*/
#endif

                            var localVertex = new float4(PlaneExtensions.Intersection(plane2, plane0, plane1), 1);

                            // TODO: since we're using a pair in the outer loop, we could also determine which 
                            //       2 planes it intersects at both ends and just check those two planes ..

                            // NOTE: for brush2, the intersection will always be only on two planes
                            //       UNLESS it's a corner vertex along that edge (we can compare to the two vertices)
                            //       in which case we could use a pre-calculated list of planes ..
                            //       OR when the intersection is outside of the edge ..

                            var worldVertex = math.mul(nodeToTreeSpaceMatrix0, localVertex).xyz;

                            if (!CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes1, localVertex) &&
                                !CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes0, localVertex))
                            {
                                var planeIndex0 = (ushort)planePair.planeIndex0;
                                var planeIndex1 = (ushort)planePair.planeIndex1;
                                var planeIndex2 = (ushort)intersectingPlaneIndices0[j];


                                // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex 
                                //       to ensure they are identical
                                var vertexIndex2 = vertexSoup0.AddNoResize(worldVertex);
                                worldVertex = vertexSoup0[vertexIndex2];
                                var vertexIndex1 = vertexSoup1.AddNoResize(worldVertex);

                                foundIndices1.AddNoResize(new PlaneVertexIndexPair() { planeIndex = planeIndex2, vertexIndex = vertexIndex1 });
                                foundIndices0.AddNoResize(new PlaneVertexIndexPair() { planeIndex = planeIndex0, vertexIndex = vertexIndex2 });
                                foundIndices0.AddNoResize(new PlaneVertexIndexPair() { planeIndex = planeIndex1, vertexIndex = vertexIndex2 });
                            }
                        }
                    }
                }
            }

            if (foundIndices0.Length >= 3)
            {
                var planeIndexOffsets0      = new NativeList<PlaneIndexOffsetLength>(foundIndices0.Length, Allocator.Temp);
                var uniqueIndices0          = new NativeList<ushort>(foundIndices0.Length, Allocator.Temp);
                var sortedFoundIndicesLength = foundIndices0.Length;

                // Unity doesn't like it if we sort on a AsDeferredArray, 
                // but doesn't like it if we sort the list, or it cast to an array either .. :(
                var memorySize          = sortedFoundIndicesLength * sizeof(PlaneVertexIndexPair);
                var sortedFoundIndices  = stackalloc PlaneVertexIndexPair[sortedFoundIndicesLength];
                UnsafeUtility.MemCpy(sortedFoundIndices, foundIndices0.GetUnsafeReadOnlyPtr(), memorySize);
                for (int i = 0; i < sortedFoundIndicesLength - 1; i++)
                {
                    for (int j = i + 1; j < sortedFoundIndicesLength; j++)
                    {
                        if (sortedFoundIndices[i].planeIndex < sortedFoundIndices[j].planeIndex)
                            continue;
                        if (sortedFoundIndices[i].planeIndex > sortedFoundIndices[j].planeIndex ||
                            sortedFoundIndices[i].vertexIndex > sortedFoundIndices[j].vertexIndex)
                        {
                            var t = sortedFoundIndices[i];
                            sortedFoundIndices[i] = sortedFoundIndices[j];
                            sortedFoundIndices[j] = t;
                        }
                    }
                }

                // Now that our indices are sorted by planeIndex, we can segment them by start/end offset
                var previousPlaneIndex  = sortedFoundIndices[0].planeIndex;
                var previousVertexIndex = sortedFoundIndices[0].vertexIndex;
                var uniqueIndicesLength = 0; // Cannot 'read' from uniqueIndices.Length here while writing
                uniqueIndices0.AddNoResize(previousVertexIndex);
                uniqueIndicesLength++;
                var loopStart = 0;
                for (int i = 1; i < sortedFoundIndicesLength; i++)
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
                            planeIndexOffsets0.AddNoResize(new PlaneIndexOffsetLength()
                            {
                                length = (ushort)currLength,
                                offset = (ushort)loopStart,
                                planeIndex = previousPlaneIndex
                            });
                        }
                        loopStart = uniqueIndicesLength;
                    }

                    uniqueIndices0.AddNoResize(vertexIndex);
                    uniqueIndicesLength++;
                    previousVertexIndex = vertexIndex;
                    previousPlaneIndex = planeIndex;
                }
                {
                    var currLength = (uniqueIndicesLength - loopStart);
                    if (currLength > 2)
                    {
                        planeIndexOffsets0.AddNoResize(new PlaneIndexOffsetLength()
                        {
                            length = (ushort)currLength,
                            offset = (ushort)loopStart,
                            planeIndex = previousPlaneIndex
                        });
                    }
                }

                var maxLength = 0;
                for (int i = 0; i < planeIndexOffsets0.Length; i++)
                    maxLength = math.max(maxLength, planeIndexOffsets0[i].length);

                var brushWorldPlanes = this.brushWorldPlanes[intersectionAsset.Value.brushes[0].brushNodeIndex];

                // For each segment, we now sort our vertices within each segment, 
                // making the assumption that they are convex
                var sortedStack = stackalloc int2[maxLength * 2];
                var vertices    = vertexSoup0.GetUnsafeReadOnlyPtr();
                for (int n = planeIndexOffsets0.Length - 1; n >= 0; n--)
                {
                    var planeIndexOffset    = planeIndexOffsets0[n];
                    var length              = planeIndexOffset.length;
                    var offset              = planeIndexOffset.offset;
                    var planeIndex          = planeIndexOffset.planeIndex;
                    
                    // TODO: use plane information instead
                    SortIndices(vertices, sortedStack, uniqueIndices0, offset, length, brushWorldPlanes.Value.worldPlanes[planeIndex].xyz);
                }

                ref var surfaceInfos = ref intersectionAsset.Value.brushes[0].surfaceInfos;
                var planeIndexOffsetsLength = planeIndexOffsets0.Length;

                for (int j = 0; j < planeIndexOffsets0.Length; j++)
                { 
                    var planeIndexLength    = planeIndexOffsets0[j];
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
                        dstVertices[d] = srcVertices[uniqueIndices0[offset + d]];

                    var outputSurface = builder.CreateBlobAssetReference<BrushIntersectionLoop>(Allocator.Persistent);
                    builder.Dispose();

                    outputSurfaces.TryAdd(new BrushSurfacePair()
                    {
                        brushNodeIndex0 = brushNodeIndex0,
                        brushNodeIndex1 = brushNodeIndex1,
                        basePlaneIndex = basePlaneIndex
                    }, outputSurface);
                }
                planeIndexOffsets0.Dispose();
                uniqueIndices0.Dispose();
            }

            if (foundIndices1.Length >= 3)
            {
                var planeIndexOffsets1          = new NativeList<PlaneIndexOffsetLength>(foundIndices1.Length, Allocator.Temp);
                var uniqueIndices1              = new NativeList<ushort>(foundIndices1.Length, Allocator.Temp);
                var sortedFoundIndicesLength    = foundIndices1.Length;

                // Unity doesn't like it if we sort on a AsDeferredArray, 
                // but doesn't like it if we sort the list, or it cast to an array either .. :(
                var memorySize = sortedFoundIndicesLength * sizeof(PlaneVertexIndexPair);
                var sortedFoundIndices = stackalloc PlaneVertexIndexPair[sortedFoundIndicesLength];
                UnsafeUtility.MemCpy(sortedFoundIndices, foundIndices1.GetUnsafeReadOnlyPtr(), memorySize);
                for (int i = 0; i < sortedFoundIndicesLength - 1; i++)
                {
                    for (int j = i + 1; j < sortedFoundIndicesLength; j++)
                    {
                        if (sortedFoundIndices[i].planeIndex < sortedFoundIndices[j].planeIndex)
                            continue;
                        if (sortedFoundIndices[i].planeIndex > sortedFoundIndices[j].planeIndex ||
                            sortedFoundIndices[i].vertexIndex > sortedFoundIndices[j].vertexIndex)
                        {
                            var t = sortedFoundIndices[i];
                            sortedFoundIndices[i] = sortedFoundIndices[j];
                            sortedFoundIndices[j] = t;
                        }
                    }
                }

                // Now that our indices are sorted by planeIndex, we can segment them by start/end offset
                var previousPlaneIndex  = sortedFoundIndices[0].planeIndex;
                var previousVertexIndex = sortedFoundIndices[0].vertexIndex;
                var uniqueIndicesLength = 0; // Cannot 'read' from uniqueIndices.Length here while writing
                uniqueIndices1.AddNoResize(previousVertexIndex);
                uniqueIndicesLength++;
                var loopStart = 0;
                for (int i = 1; i < sortedFoundIndicesLength; i++)
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
                            planeIndexOffsets1.AddNoResize(new PlaneIndexOffsetLength()
                            {
                                length = (ushort)currLength,
                                offset = (ushort)loopStart,
                                planeIndex = previousPlaneIndex
                            });
                        }
                        loopStart = uniqueIndicesLength;
                    }

                    uniqueIndices1.AddNoResize(vertexIndex);
                    uniqueIndicesLength++;
                    previousVertexIndex = vertexIndex;
                    previousPlaneIndex = planeIndex;
                }
                {
                    var currLength = (uniqueIndicesLength - loopStart);
                    if (currLength > 2)
                    {
                        planeIndexOffsets1.AddNoResize(new PlaneIndexOffsetLength()
                        {
                            length = (ushort)currLength,
                            offset = (ushort)loopStart,
                            planeIndex = previousPlaneIndex
                        });
                    }
                }
                var maxLength = 0;
                for (int i = 0; i < planeIndexOffsets1.Length; i++)
                    maxLength = math.max(maxLength, planeIndexOffsets1[i].length);

                var brushWorldPlanes = this.brushWorldPlanes[intersectionAsset.Value.brushes[1].brushNodeIndex];

                // For each segment, we now sort our vertices within each segment, 
                // making the assumption that they are convex
                var sortedStack = stackalloc int2[maxLength * 2];
                var vertices    = vertexSoup1.GetUnsafeReadOnlyPtr();
                for (int n = planeIndexOffsets1.Length - 1; n >= 0; n--)
                {
                    var planeIndexOffset    = planeIndexOffsets1[n];
                    var length              = planeIndexOffset.length;
                    var offset              = planeIndexOffset.offset;
                    var planeIndex          = planeIndexOffset.planeIndex;
                    
                    // TODO: use plane information instead
                    SortIndices(vertices, sortedStack, uniqueIndices1, offset, length, brushWorldPlanes.Value.worldPlanes[planeIndex].xyz);
                }

                ref var surfaceInfos = ref intersectionAsset.Value.brushes[1].surfaceInfos;
                var planeIndexOffsetsLength = planeIndexOffsets1.Length;

                for (int j = 0; j < planeIndexOffsets1.Length; j++)
                { 
                    var planeIndexLength    = planeIndexOffsets1[j];
                    var offset              = planeIndexLength.offset;
                    var loopLength          = planeIndexLength.length;
                    var basePlaneIndex      = planeIndexLength.planeIndex;
                    var surfaceInfo         = surfaceInfos[basePlaneIndex];

                    var builder = new BlobBuilder(Allocator.Temp);
                    ref var root = ref builder.ConstructRoot<BrushIntersectionLoop>();
                    root.surfaceInfo = surfaceInfo;
                    var dstVertices = builder.Allocate(ref root.loopVertices, loopLength);
                    var srcVertices = vertexSoup1.GetUnsafeReadOnlyPtr();
                    for (int d = 0; d < loopLength; d++)
                        dstVertices[d] = srcVertices[uniqueIndices1[offset + d]];

                    var outputSurface = builder.CreateBlobAssetReference<BrushIntersectionLoop>(Allocator.Persistent);
                    builder.Dispose();

                    outputSurfaces.TryAdd(new BrushSurfacePair()
                    {
                        brushNodeIndex0 = brushNodeIndex1,
                        brushNodeIndex1 = brushNodeIndex0,
                        basePlaneIndex = basePlaneIndex
                    }, outputSurface);
                }
                planeIndexOffsets1.Dispose();
                uniqueIndices1.Dispose();
            }

            foundIndices0.Dispose();
            foundIndices1.Dispose();

            vertexSoup0.Dispose();
            vertexSoup1.Dispose();

            intersectionAsset.Dispose();
        }
    }
#endif
}
