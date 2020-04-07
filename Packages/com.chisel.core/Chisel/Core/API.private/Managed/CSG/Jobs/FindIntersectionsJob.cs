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
            if (planeIndex > other.planeIndex)
                return 1;
            if (planeIndex == other.planeIndex)
            {
                if (vertexIndex == other.vertexIndex)
                    return 0;
                if (vertexIndex < other.vertexIndex)
                    return 1;
            }
            return -1;
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
    
    [BurstCompile(CompileSynchronously = true)]
    unsafe struct FindIntersectionsJob : IJob// IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kDistanceEpsilon        = CSGManagerPerformCSG.kDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        [NoAlias, ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [NoAlias, ReadOnly] public int                   intersectionPlaneIndex0;
        [NoAlias, ReadOnly] public int                   intersectionPlaneIndex1;
        [NoAlias, ReadOnly] public int                   usedPlanePairIndex1;

        [NoAlias, WriteOnly] public NativeList<VertexAndPlanePair>           foundVertices;

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

        public void Execute()
        {
            ref var intersectingPlanes0 = ref intersection.Value.brushes[intersectionPlaneIndex0].localSpacePlanes0;
            for (int index = 0;index< intersectingPlanes0.Length;index++)
            {
                Execute(index);
            }
        }
    }

    [BurstCompile(CompileSynchronously = true)]
    unsafe struct InsertIntersectionVerticesJob : IJob
    {
        [NoAlias, ReadOnly] public NativeArray<VertexAndPlanePair> vertexReader;
        [NoAlias, ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [NoAlias, ReadOnly] public int intersectionPlaneIndex0;

        [NoAlias, WriteOnly] public VertexSoup brushVertices0;
        [NoAlias, WriteOnly] public VertexSoup brushVertices1;

        [NoAlias, WriteOnly] public NativeList<PlaneVertexIndexPair> outputIndices0;
        [NoAlias, WriteOnly] public NativeList<PlaneVertexIndexPair> outputIndices1;

        public void Execute() 
        {
            for (int index = 0; index < vertexReader.Length; index++)
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
    }

    [BurstCompile(CompileSynchronously = true)]
    unsafe struct CombineLoopIndicesJob : IJob
    {
        [NoAlias, ReadOnly] public NativeArray<PlaneVertexIndexPair>     foundIndices;
        [NoAlias, WriteOnly] public NativeList<ushort>                   uniqueIndices;
        [NoAlias, WriteOnly] public NativeList<PlaneIndexOffsetLength>   planeIndexOffsets;

        public void Execute()
        {
            if (foundIndices.Length < 3)
                return;

            var sortedFoundIndicesLength = foundIndices.Length;

            // Unity doesn't like it if we sort on a AsDeferredArray, 
            // but doesn't like it if we sort the list, or it cast to an array either .. :(
            var memorySize = sortedFoundIndicesLength * sizeof(PlaneVertexIndexPair);
            var sortedFoundIndices = stackalloc PlaneVertexIndexPair[sortedFoundIndicesLength];// (PlaneVertexIndexPair*)UnsafeUtility.Malloc(memorySize, 4, Allocator.Temp);
            UnsafeUtility.MemCpy(sortedFoundIndices, foundIndices.GetUnsafeReadOnlyPtr(), memorySize);
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
            uniqueIndices.AddNoResize(previousVertexIndex);
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
                        planeIndexOffsets.AddNoResize(new PlaneIndexOffsetLength()
                        {
                            length = (ushort)currLength,
                            offset = (ushort)loopStart,
                            planeIndex = previousPlaneIndex
                        });
                    }
                    loopStart = uniqueIndicesLength;
                }

                uniqueIndices.AddNoResize(vertexIndex);
                uniqueIndicesLength++;
                previousVertexIndex = vertexIndex;
                previousPlaneIndex = planeIndex;
            }
            {
                var currLength = (uniqueIndicesLength - loopStart);
                if (currLength > 2)
                {
                    planeIndexOffsets.AddNoResize(new PlaneIndexOffsetLength()
                    {
                        length = (ushort)currLength,
                        offset = (ushort)loopStart,
                        planeIndex = previousPlaneIndex
                    });
                }
            }
            //UnsafeUtility.Free(sortedFoundIndices, Allocator.Temp);
        }
    }
    
    [BurstCompile(CompileSynchronously = true)]
    unsafe struct SortLoopsJob : IJob
    {
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>> allBrushWorldPlanes;
        [NoAlias, ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [NoAlias, ReadOnly] public int                       brushNodeIndex;
        [NoAlias, ReadOnly] public VertexSoup                soup;

        // Cannot be WriteOnly because we sort segments after we insert them
        public NativeList<ushort>                   uniqueIndices;
        public NativeList<PlaneIndexOffsetLength>   planeIndexOffsets;


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


        public void Execute()
        {
            if (uniqueIndices.Length < 3)
                return;


            var maxLength = 0;
            for (int i = 0; i < planeIndexOffsets.Length; i++)
                maxLength = math.max(maxLength, planeIndexOffsets[i].length);

            var brushWorldPlanes = allBrushWorldPlanes[intersection.Value.brushes[brushNodeIndex].brushNodeIndex];

            // For each segment, we now sort our vertices within each segment, 
            // making the assumption that they are convex
            var sortedStack = stackalloc int2[maxLength * 2];// (int2*)UnsafeUtility.Malloc(maxLength * 2 * sizeof(int2), 4, Allocator.Temp);
            var vertices    = soup.GetUnsafeReadOnlyPtr();
            for (int n = planeIndexOffsets.Length - 1; n >= 0; n--)
            {
                var planeIndexOffset    = planeIndexOffsets[n];
                var length              = planeIndexOffset.length;
                var offset              = planeIndexOffset.offset;
                var planeIndex          = planeIndexOffset.planeIndex;
                // TODO: use plane information instead
                SortIndices(vertices, sortedStack, uniqueIndices, offset, length, brushWorldPlanes.Value.worldPlanes[planeIndex].xyz);
            }
            //UnsafeUtility.Free(sortedStack, Allocator.Temp);
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
    [BurstCompile(CompileSynchronously = true)]
    unsafe struct CreateLoopsJob //: IJob
    {
        [NoAlias, ReadOnly] public int                                   brushIndex0;
        [NoAlias, ReadOnly] public int                                   brushIndex1;
        [NoAlias, ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [NoAlias, ReadOnly] public int                                   intersectionSurfaceIndex;
        [NoAlias, ReadOnly] public VertexSoup                            vertexSoup;
        [NoAlias, ReadOnly] public NativeList<ushort>                    uniqueIndices;
        [NoAlias, ReadOnly] public NativeList<PlaneIndexOffsetLength>    planeIndexOffsets;

#if false
        [NoAlias, WriteOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>.ParallelWriter outputSurfaces;

        public void Execute()
        {
            if (uniqueIndices.Length < 3)
                return;

            for (int index = 0; index < planeIndexOffsets.Length; index++)
            { 
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
                    var srcVertices = vertexSoup.GetUnsafeReadOnlyPtr();
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

#else
        public void Execute(ref NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>.ParallelWriter outputSurfaces)
        {
            if (uniqueIndices.Length < 3)
                return;

            for (int index = 0; index < planeIndexOffsets.Length; index++)
            { 
                ref var surfaceInfos = ref intersection.Value.brushes[intersectionSurfaceIndex].surfaceInfos;

                var planeIndexOffsetsLength = planeIndexOffsets.Length;

                //for (int index = 0; index < planeIndexOffsetsLength; index++)
                {
                    var planeIndexLength    = planeIndexOffsets[index];
                    var offset              = planeIndexLength.offset;
                    var loopLength          = planeIndexLength.length;
                    var basePlaneIndex      = planeIndexLength.planeIndex;
                    var surfaceInfo         = surfaceInfos[basePlaneIndex];

                    var builder = new BlobBuilder(Allocator.Temp);
                    ref var root = ref builder.ConstructRoot<BrushIntersectionLoop>();
                    root.surfaceInfo = surfaceInfo;
                    var dstVertices = builder.Allocate(ref root.loopVertices, loopLength);
                    var srcVertices = vertexSoup.GetUnsafeReadOnlyPtr();
                    for (int j = 0; j < loopLength; j++)
                        dstVertices[j] = srcVertices[uniqueIndices[offset + j]];

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
#endif
    }
}