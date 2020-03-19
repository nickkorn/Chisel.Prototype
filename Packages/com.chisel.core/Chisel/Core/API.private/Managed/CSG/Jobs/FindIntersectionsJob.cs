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
    public struct SurfaceInfo
    {
        public float4               worldPlane;
        public SurfaceLayers        layers;
        public int                  basePlaneIndex;
        public int                  brushNodeIndex;
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

    struct PlaneIndexOffsetLength : IComparable<PlaneIndexOffsetLength>
    {
        public ushort length;
        public ushort offset;
        public ushort planeIndex;

        public int CompareTo(PlaneIndexOffsetLength other)
        {
            if (planeIndex < other.planeIndex)
                return -1;
            if (planeIndex > other.planeIndex)
                return 1;
            if (offset < other.offset)
                return -1;
            if (offset > other.offset)
                return 1;
            if (length < other.length)
                return -1;
            if (length > other.length)
                return 1;
            return 0;
        }
    }

    struct VertexAndPlanePair
    {
        public float3 vertex;
        public ushort plane0;
        public ushort plane1;
        public ushort plane2;
    }
    
    public struct PlanePair
    {
        public float4 plane0;
        public float4 plane1;
        //public double4 N0;
        //public double4 N1;
        public float4 edgeVertex0;
        public float4 edgeVertex1;
        public int planeIndex0;
        public int planeIndex1;
    }
    
    [BurstCompile(Debug = false)]
    unsafe struct FindIntersectionsJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kDistanceEpsilon        = CSGManagerPerformCSG.kDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        [ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [ReadOnly] public int                   intersectionPlaneIndex0;
        [ReadOnly] public int                   intersectionPlaneIndex1;
        [ReadOnly] public int                   usedPlanePairIndex1;

        [WriteOnly] public NativeList<VertexAndPlanePair>.ParallelWriter foundVertices;

        public void Execute(int index)
        {
            ref var intersectingPlanes0         = ref intersection.Value.brushes[intersectionPlaneIndex0].localSpacePlanes0;
            ref var intersectingPlanes1         = ref intersection.Value.brushes[intersectionPlaneIndex1].localSpacePlanes0;
            ref var usedPlanePairs1             = ref intersection.Value.brushes[usedPlanePairIndex1].usedPlanePairs;
            ref var intersectingPlaneIndices0   = ref intersection.Value.brushes[intersectionPlaneIndex0].localSpacePlaneIndices0;
            var nodeToTreeSpaceMatrix0          = intersection.Value.brushes[0].transformation.nodeToTree;

            var plane2 = intersectingPlanes0[index];

            for (int i = 0; i < usedPlanePairs1.Length; i++)
            {
                var planePair   = usedPlanePairs1[i];

                var plane0      = planePair.plane0;
                var plane1      = planePair.plane1;

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
                        foundVertices.AddNoResize(new VertexAndPlanePair()
                        {
                            vertex = worldVertex,
                            plane0 = (ushort)planePair.planeIndex0,
                            plane1 = (ushort)planePair.planeIndex1,
                            plane2 = (ushort)intersectingPlaneIndices0[index]
                        });
                    }
                }
            }
        }
    }

    [BurstCompile(Debug = false)]
    unsafe struct InsertIntersectionVerticesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<VertexAndPlanePair> vertexReader;
        [ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [ReadOnly] public int intersectionPlaneIndex0;
        
        public VertexSoup brushVertices0;
        public VertexSoup brushVertices1;

        [WriteOnly] public NativeList<PlaneVertexIndexPair>.ParallelWriter outputIndices0;
        [WriteOnly] public NativeList<PlaneVertexIndexPair>.ParallelWriter outputIndices1;

        public void Execute(int index) 
        {
            var worldVertex = vertexReader[index].vertex;
            var plane0      = vertexReader[index].plane0;
            var plane1      = vertexReader[index].plane1; 
            var plane2      = vertexReader[index].plane2;

            // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex 
            //       to ensure they are identical
            var vertexIndex1 = brushVertices0.AddNoResize(worldVertex);
            var vertexIndex2 = brushVertices1.AddNoResize(worldVertex);

            outputIndices0.AddNoResize(new PlaneVertexIndexPair() { planeIndex = plane2, vertexIndex = vertexIndex1 });
            outputIndices1.AddNoResize(new PlaneVertexIndexPair() { planeIndex = plane0, vertexIndex = vertexIndex2 });
            outputIndices1.AddNoResize(new PlaneVertexIndexPair() { planeIndex = plane1, vertexIndex = vertexIndex2 });
        }
    }

    [BurstCompile(Debug = false)]
    unsafe struct SortLoopsJob : IJob
    {
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>> allBrushWorldPlanes;
        [ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [ReadOnly] public int                       brushNodeIndex;
        [ReadOnly] public NativeList<float3>        vertexSoup;

        // Cannot be ReadOnly because we sort it
        //[ReadOnly] 
        public NativeList<PlaneVertexIndexPair>     foundIndices;

        // Cannot be WriteOnly because we sort segments after we insert them
        //[WriteOnly]
        public NativeList<ushort>                   uniqueIndices;
        //[WriteOnly]
        public NativeList<PlaneIndexOffsetLength>   planeIndexOffsets;


        #region Sort
        static float3 FindPolygonCentroid(float3* vertices, ushort* indicesPtr, int offset, int indicesCount)
        {
            var centroid = float3.zero;
            for (int i = 0; i < indicesCount; i++, offset++)
                centroid += vertices[indicesPtr[offset]];
            return centroid / indicesCount;
        }

        // TODO: sort by using plane information instead of unreliable floating point math ..
        unsafe void SortIndices(int2* sortedStack, ushort* indicesPtr, int offset, int indicesCount, float3 normal)
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

            var verticesPtr = (float3*)vertexSoup.GetUnsafeReadOnlyPtr();
            var centroid = FindPolygonCentroid(verticesPtr, indicesPtr, offset, indicesCount);
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
                var va = (float3)verticesPtr[indicesPtr[offset + (left + right) / 2]];
                while (true)
                {
                    var a_angle = math.atan2(math.dot(tangentX, va) - center.x, math.dot(tangentY, va) - center.y);

                    {
                        var vb = (float3)verticesPtr[indicesPtr[offset + left]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (b_angle > a_angle)
                        {
                            left++;
                            vb = (float3)verticesPtr[indicesPtr[offset + left]];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    {
                        var vb = (float3)verticesPtr[indicesPtr[offset + right]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (a_angle > b_angle)
                        {
                            right--;
                            vb = (float3)verticesPtr[indicesPtr[offset + right]];
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


        public void Execute()
        {
            BlobAssetReference<BrushWorldPlanes> brushWorldPlanes = allBrushWorldPlanes[intersection.Value.brushes[brushNodeIndex].brushNodeIndex];
            for (int i = 0; i < foundIndices.Length - 1; i++)
            {
                for (int j = i + 1; j < foundIndices.Length; j++)
                {
                    if (foundIndices[i].planeIndex < foundIndices[j].planeIndex)
                        continue;
                    if (foundIndices[i].planeIndex > foundIndices[j].planeIndex ||
                        foundIndices[i].vertexIndex > foundIndices[j].vertexIndex)
                    {
                        var t = foundIndices[i];
                        foundIndices[i] = foundIndices[j];
                        foundIndices[j] = t;
                    }
                }
            }
                //NativeSortExtension.Sort(foundIndices); // <- we can only do this if it's not readonly!

            // Now that our indices are sorted by planeIndex, we can segment them by start/end offset
            var previousPlaneIndex  = foundIndices[0].planeIndex;
            var previousVertexIndex = foundIndices[0].vertexIndex;
            uniqueIndices.AddNoResize(previousVertexIndex);
            var loopStart = 0;
            var maxLength = 0;
            for (int i = 1; i < foundIndices.Length; i++)
            {
                var indices     = foundIndices[i];

                var planeIndex  = indices.planeIndex;
                var vertexIndex = indices.vertexIndex;

                // TODO: why do we have soooo many duplicates sometimes?
                if (planeIndex  == previousPlaneIndex &&
                    vertexIndex == previousVertexIndex)
                    continue;

                if (planeIndex != previousPlaneIndex)
                {
                    var currLength = (uniqueIndices.Length - loopStart);
                    if (currLength > 2)
                    {
                        planeIndexOffsets.AddNoResize(new PlaneIndexOffsetLength()
                        {
                            length = (ushort)currLength,
                            offset = (ushort)loopStart,
                            planeIndex = previousPlaneIndex
                        });
                        maxLength = math.max(maxLength, currLength);
                    }
                    loopStart = uniqueIndices.Length;
                }

                uniqueIndices.AddNoResize(vertexIndex);
                previousVertexIndex = vertexIndex;
                previousPlaneIndex = planeIndex;
            }
            {
                var currLength = (uniqueIndices.Length - loopStart);
                if (currLength > 2)
                {
                    planeIndexOffsets.AddNoResize(new PlaneIndexOffsetLength()
                    {
                        length = (ushort)currLength,
                        offset = (ushort)loopStart,
                        planeIndex = previousPlaneIndex
                    });
                    maxLength = math.max(maxLength, currLength);
                }
            }


            // TODO: do in separate pass?

            
            // For each segment, we now sort our vertices within each segment, 
            // making the assumption that they are convex
            var indicesPtr = (ushort*)uniqueIndices.GetUnsafePtr();
            var sortedStack = (int2*)UnsafeUtility.Malloc(maxLength * 2 * sizeof(int2), 4, Allocator.Temp);
            for (int n = planeIndexOffsets.Length - 1; n >= 0; n--)
            {
                var planeIndexOffset    = planeIndexOffsets[n];
                var length              = planeIndexOffset.length;
                var offset              = planeIndexOffset.offset;
                var planeIndex          = planeIndexOffset.planeIndex;
                // TODO: use plane information instead
                SortIndices(sortedStack, indicesPtr, offset, length, brushWorldPlanes.Value.worldPlanes[planeIndex].xyz);
            }
            UnsafeUtility.Free(sortedStack, Allocator.Temp);
        }
    }

    internal struct BrushSurfacePair : IEquatable<BrushSurfacePair>
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

    // TODO: merge with SortLoopsJob
    [BurstCompile]
    unsafe struct CreateLoopsJob : IJobParallelFor
    {
        [ReadOnly] public int                                   brushIndex0;
        [ReadOnly] public int                                   brushIndex1;
        [ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [ReadOnly] public int                                   intersectionSurfaceIndex;
        [ReadOnly] public NativeList<float3>                    srcVertices;
        [ReadOnly] public NativeList<ushort>                    uniqueIndices;
        [ReadOnly] public NativeList<PlaneIndexOffsetLength>    planeIndexOffsets;

        [WriteOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>.ParallelWriter outputSurfaces;

        public void Execute(int index)
        {
            if (uniqueIndices.Length < 3)
                return;

            ref var surfaceInfos = ref intersection.Value.brushes[intersectionSurfaceIndex].surfaceInfos;

            var indicesPtr              = (ushort*)uniqueIndices.GetUnsafeReadOnlyPtr();
            var planeIndexOffsetsPtr    = (PlaneIndexOffsetLength*)planeIndexOffsets.GetUnsafeReadOnlyPtr();
            var planeIndexOffsetsLength = planeIndexOffsets.Length;

            //for (int index = 0; index < planeIndexOffsetsLength; index++)
            {
                var planeIndexLength    = planeIndexOffsetsPtr[index];
                var offset              = planeIndexLength.offset;
                var loopLength          = planeIndexLength.length;
                var basePlaneIndex      = planeIndexLength.planeIndex;
                var surfaceInfo         = surfaceInfos[basePlaneIndex];

                var builder = new BlobBuilder(Allocator.Temp);
                ref var root = ref builder.ConstructRoot<BrushIntersectionLoop>();
                root.surfaceInfo = surfaceInfo;
                var dstVertices = builder.Allocate(ref root.loopVertices, loopLength);
                for (int j = 0; j < loopLength; j++)
                    dstVertices[j] = srcVertices[indicesPtr[offset + j]];

                var outputSurface = builder.CreateBlobAssetReference<BrushIntersectionLoop>(Allocator.Persistent);
                builder.Dispose();

                outputSurfaces.TryAdd(new BrushSurfacePair()
                {
                    brushNodeIndex0 = brushIndex0,
                    brushNodeIndex1 = brushIndex1,
                    basePlaneIndex = basePlaneIndex
                }, outputSurface);
            }
        }
    }
}