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
    [BurstCompile]
    struct FindBrushPairsJob : IJob
    {
        public struct Empty { }

        [ReadOnly] public NativeArray<int>       treeBrushes;
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>> brushesTouchedByBrushes;
        [WriteOnly] public NativeList<BrushPair> brushPairs;

        public void Execute()
        {
            var empty = new Empty();
            var brushPairMap = new NativeHashMap<BrushPair, FindBrushPairsJob.Empty>(GeometryMath.GetTriangleArraySize(treeBrushes.Length), Allocator.Temp);
            for (int b0 = 0; b0 < treeBrushes.Length; b0++)
            {
                var brushNodeID0     = treeBrushes[b0];
                var brushNodeIndex0  = brushNodeID0 - 1;
                if (!brushesTouchedByBrushes.TryGetValue(brushNodeIndex0, out BlobAssetReference<BrushesTouchedByBrush> brushesTouchedByBrush))
                    continue;
                    
                ref var intersections = ref brushesTouchedByBrush.Value.brushIntersections;
                if (intersections.Length == 0)
                    continue;

                // Find all intersections between brushes
                for (int i = 0; i < intersections.Length; i++)
                {
                    var intersection    = intersections[i];
                    var brushNodeIndex1 = intersection.nodeIndex;

                    var reverseOrder    = brushNodeIndex0 > brushNodeIndex1; // ensures we do calculations exactly the same for each brush pair
                    var type            = intersection.type;

                    var brushPair       = new BrushPair()
                    {
                        type            = type,
                        brushNodeIndex0 = brushNodeIndex0,
                        brushNodeIndex1 = brushNodeIndex1
                    };

                    if (reverseOrder)
                        brushPair.Flip();

                    if (brushPairMap.TryAdd(brushPair, empty))
                    {
                        brushPairs.Add(brushPair);
                    }
                }
            }
            brushPairMap.Dispose();
        }
    }

    [BurstCompile]
    struct PrepareBrushPairIntersectionsJob : IJob
    {
        const float kDistanceEpsilon        = CSGManagerPerformCSG.kDistanceEpsilon;
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        public struct Empty { }

        [ReadOnly] public NativeArray<BrushPair> brushPairs;

        [ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>         brushMeshBlobLookup;
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<NodeTransformations>>   transformations;

        [WriteOnly] public NativeList<BlobAssetReference<BrushPairIntersection>>.ParallelWriter intersectingBrushes;

        // TODO: turn into job
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void GetIntersectingPlanes(ref BlobArray<float4> localPlanes, ref BlobArray<float3> vertices, Bounds selfBounds, float4x4 treeToNodeSpaceInverseTransposed, NativeList<int> intersectingPlanes)
        {
            //using (new ProfileSample("GetIntersectingPlanes"))
            {
                var min = (float3)selfBounds.min;
                var max = (float3)selfBounds.max;

                var verticesLength = vertices.Length;
                var verticesPtr = (float3*)vertices.GetUnsafePtr();

                for (int i = 0; i < localPlanes.Length; i++)
                {
                    // bring plane into local space of mesh, the same space as the bounds of the mesh

                    var localPlane = localPlanes[i];

                    // note: a transpose is part of this transformation
                    var transformedPlane = math.mul(treeToNodeSpaceInverseTransposed, localPlane);
                    //var normal            = transformedPlane.xyz; // only need the signs, so don't care about normalization
                    //transformedPlane /= math.length(normal);      // we don't have to normalize the plane

                    var corner = new float4((transformedPlane.x < 0) ? max.x : min.x,
                                            (transformedPlane.y < 0) ? max.y : min.y,
                                            (transformedPlane.z < 0) ? max.z : min.z,
                                            1.0f);
                    float forward = math.dot(transformedPlane, corner);
                    if (forward > kDistanceEpsilon) // closest point is outside
                    {
                        intersectingPlanes.Clear();
                        return;
                    }

                    // do a bounds check
                    corner = new float4((transformedPlane.x >= 0) ? max.x : min.x,
                                        (transformedPlane.y >= 0) ? max.y : min.y,
                                        (transformedPlane.z >= 0) ? max.z : min.z,
                                        1.0f);
                    float backward = math.dot(transformedPlane, corner);
                    if (backward < -kDistanceEpsilon) // closest point is inside
                        continue;

                    float minDistance = float.PositiveInfinity;
                    float maxDistance = float.NegativeInfinity;
                    int onCount = 0;
                    for (int v = 0; v < verticesLength; v++)
                    {
                        float distance = math.dot(transformedPlane, new float4(verticesPtr[v], 1));
                        minDistance = math.min(distance, minDistance);
                        maxDistance = math.max(distance, maxDistance);
                        onCount += (distance >= -kDistanceEpsilon && distance <= kDistanceEpsilon) ? 1 : 0;
                    }

                    // if all vertices are 'inside' this plane, then we're not truly intersecting with it
                    if ((minDistance > kDistanceEpsilon || maxDistance < -kDistanceEpsilon))
                        continue;

                    intersectingPlanes.Add(i);
                }
            }
        }
        
        // TODO: turn into job
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void FindPlanePairs(NativeList<int>       usedVertices,
                                          NativeList<PlanePair> usedPlanePairs, 
                                          NativeList<int>       intersectingPlanes,
                                          float4*               localSpacePlanesPtr,
                                          float4x4              vertexTransform,
                                          ref BrushMeshBlob     mesh)
        {
            //using (new ProfileSample("FindPlanePairs"))
            {
                // TODO: this can be partially stored in brushmesh 
                // TODO: optimize
                usedPlanePairs.Clear();

                var halfEdgesLength = mesh.halfEdges.Length;
                var halfEdgesPtr = (BrushMesh.HalfEdge*)mesh.halfEdges.GetUnsafePtr();

                //var edgeVertexPlanePairLength   = mesh.edgeVertexPlanePair.Length;
                //var edgeVertexPlanePairsPtr     = (BrushMeshBlob.EdgeVertexPlanePair*)mesh.edgeVertexPlanePair.GetUnsafePtr();
                var halfEdgePolygonIndicesPtr = (int*)mesh.halfEdgePolygonIndices.GetUnsafePtr();

                var intersectingPlanesLength = intersectingPlanes.Length;
                var intersectingPlanesPtr = (int*)intersectingPlanes.GetUnsafeReadOnlyPtr();

                var vertexUsed = stackalloc byte[mesh.vertices.Length];
                var planeAvailable = stackalloc byte[mesh.localPlanes.Length];
                {
                    {
                        for (int p = 0; p < intersectingPlanesLength; p++)
                        {
                            planeAvailable[intersectingPlanesPtr[p]] = 1;
                        }
                    }

                    for (int e = 0; e < halfEdgesLength; e++)
                    {
                        var twinIndex = halfEdgesPtr[e].twinIndex;
                        if (twinIndex < e)
                            continue;

                        //var planeIndex0 = edgeVertexPlanePairsPtr[e].planeIndex0;
                        //var planeIndex1 = edgeVertexPlanePairsPtr[e].planeIndex1;

                        var planeIndex0 = halfEdgePolygonIndicesPtr[e];
                        var planeIndex1 = halfEdgePolygonIndicesPtr[twinIndex];

                        Debug.Assert(planeIndex0 != planeIndex1);

                        if (planeAvailable[planeIndex0] == 0 ||
                            planeAvailable[planeIndex1] == 0)
                            continue;

                        var plane0 = localSpacePlanesPtr[planeIndex0];
                        var plane1 = localSpacePlanesPtr[planeIndex1];
                        //var vertexIndex0 = edgeVertexPlanePairsPtr[e].vertexIndex0;
                        //var vertexIndex1 = edgeVertexPlanePairsPtr[e].vertexIndex1;

                        var vertexIndex0 = halfEdgesPtr[e].vertexIndex;
                        var vertexIndex1 = halfEdgesPtr[twinIndex].vertexIndex;

                        var vertex0 = math.mul(vertexTransform, new float4(mesh.vertices[vertexIndex0], 1));
                        var vertex1 = math.mul(vertexTransform, new float4(mesh.vertices[vertexIndex1], 1));

                        //PlaneExtensions.IntersectionFirst(localSpacePlanes1[pI0], localSpacePlanes1[pI1], out double4 N0, out double4 N1);
                        if (vertexUsed[vertexIndex0] == 0) { vertexUsed[vertexIndex0] = 1; usedVertices.Add(vertexIndex0); }
                        if (vertexUsed[vertexIndex1] == 0) { vertexUsed[vertexIndex1] = 1; usedVertices.Add(vertexIndex1); }
                        usedPlanePairs.Add(new PlanePair()
                        {
                            plane0 = plane0,
                            plane1 = plane1,
                            edgeVertex0 = vertex0,
                            edgeVertex1 = vertex1,
                            planeIndex0 = planeIndex0,
                            planeIndex1 = planeIndex1
                        });
                    }
                }
            }
        }

        public unsafe void Execute()
        {
            var intersectingPlaneIndices0   = new NativeList<int>(Allocator.Temp);
            var intersectingPlaneIndices1   = new NativeList<int>(Allocator.Temp);
            var intersectingPlanes0         = new NativeList<float4>(Allocator.Temp);
            var intersectingPlanes1         = new NativeList<float4>(Allocator.Temp);

            var usedPlanePairs0             = new NativeList<PlanePair>(Allocator.Temp);
            var usedPlanePairs1             = new NativeList<PlanePair>(Allocator.Temp);

            var surfaceInfos0            = new NativeList<SurfaceInfo>(Allocator.Temp);
            var surfaceInfos1            = new NativeList<SurfaceInfo>(Allocator.Temp);

            var usedVertices0               = new NativeList<int>(Allocator.Temp);
            var usedVertices1               = new NativeList<int>(Allocator.Temp);
            for (int index = 0; index < brushPairs.Length; index++)
            {
                var brushPair       = brushPairs[index];
                var brushNodeIndex0 = brushPair.brushNodeIndex0;
                var brushNodeIndex1 = brushPair.brushNodeIndex1;

                if (!brushMeshBlobLookup.TryGetValue(brushNodeIndex0, out BlobAssetReference<BrushMeshBlob> blobMesh0) ||
                    !brushMeshBlobLookup.TryGetValue(brushNodeIndex1, out BlobAssetReference<BrushMeshBlob> blobMesh1))
                    continue;

                var transformations0 = transformations[brushNodeIndex0];
                var transformations1 = transformations[brushNodeIndex1];

                var node1ToNode0            = math.mul(transformations0.Value.treeToNode, transformations1.Value.nodeToTree);
                var node0ToNode1            = math.mul(transformations1.Value.treeToNode, transformations0.Value.nodeToTree);
                var inversedNode1ToNode0    = math.transpose(node0ToNode1);
                var inversedNode0ToNode1    = math.transpose(node1ToNode0);

                ref var mesh0 = ref blobMesh0.Value;
                ref var mesh1 = ref blobMesh1.Value;

                intersectingPlaneIndices0.Clear();
                intersectingPlaneIndices1.Clear();

                var type = brushPair.type;
                if (type == IntersectionType.Intersection)
                {
                    GetIntersectingPlanes(ref mesh0.localPlanes, ref mesh1.vertices, mesh1.localBounds, inversedNode0ToNode1, intersectingPlaneIndices0);
                    GetIntersectingPlanes(ref mesh1.localPlanes, ref mesh0.vertices, mesh0.localBounds, inversedNode1ToNode0, intersectingPlaneIndices1);
                } else
                if (type == IntersectionType.AInsideB ||
                    type == IntersectionType.BInsideA)
                {
                    intersectingPlaneIndices0.ResizeUninitialized(mesh0.localPlanes.Length);
                    intersectingPlaneIndices1.ResizeUninitialized(mesh1.localPlanes.Length);
                    for (int i = 0; i < intersectingPlaneIndices0.Length; i++) intersectingPlaneIndices0[i] = i;
                    for (int i = 0; i < intersectingPlaneIndices1.Length; i++) intersectingPlaneIndices1[i] = i;
                } else
                    continue;

                if (intersectingPlaneIndices0.Length == 0 ||
                    intersectingPlaneIndices1.Length == 0)
                    continue;

                var inverseNodeToTreeSpaceMatrix0 = math.transpose(transformations0.Value.treeToNode);
                var inverseNodeToTreeSpaceMatrix1 = math.transpose(transformations1.Value.treeToNode);

                surfaceInfos0.Clear();
                surfaceInfos1.Clear();
                surfaceInfos0.Resize(mesh0.localPlanes.Length, NativeArrayOptions.ClearMemory); // all set to Inside (0)
                surfaceInfos1.Resize(mesh1.localPlanes.Length, NativeArrayOptions.ClearMemory); // all set to Inside (0)
                for (int i = 0; i < surfaceInfos0.Length; i++)
                {
                    var localPlaneVector = mesh0.localPlanes[i];
                    var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix0, localPlaneVector);
                    var length = math.length(worldPlaneVector.xyz);
                    worldPlaneVector /= length;

                    surfaceInfos0[i] = new SurfaceInfo()
                    {
                        interiorCategory    = (CategoryGroupIndex)CategoryIndex.Inside,
                        basePlaneIndex      = i,
                        brushNodeIndex      = brushNodeIndex1,
                        worldPlane          = worldPlaneVector,
                        layers              = mesh0.polygons[i].layerDefinition
                    };
                }
                for (int i = 0; i < surfaceInfos1.Length; i++)
                {
                    var localPlaneVector = mesh1.localPlanes[i];
                    var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix1, localPlaneVector);
                    var length = math.length(worldPlaneVector.xyz);
                    worldPlaneVector /= length;

                    surfaceInfos1[i] = new SurfaceInfo()
                    {
                        interiorCategory    = (CategoryGroupIndex)CategoryIndex.Inside,
                        basePlaneIndex      = i,
                        brushNodeIndex      = brushNodeIndex0,
                        worldPlane          = worldPlaneVector,
                        layers              = mesh1.polygons[i].layerDefinition
                    };
                }


                // TODO: we don't actually use ALL of these planes .. Optimize this
                var localSpacePlanes0Length = mesh0.localPlanes.Length;
                var localSpacePlanes0 = stackalloc float4[localSpacePlanes0Length];
                for (int p = 0; p < localSpacePlanes0Length; p++)
                    localSpacePlanes0[p] = mesh0.localPlanes[p];

                // TODO: we don't actually use ALL of these planes .. Optimize this
                var localSpacePlanes1Length = mesh1.localPlanes.Length;
                var localSpacePlanes1 = stackalloc float4[localSpacePlanes1Length];
                for (int p = 0; p < localSpacePlanes1Length; p++)
                {
                    var transformedPlane = math.mul(inversedNode1ToNode0, mesh1.localPlanes[p]);
                    localSpacePlanes1[p] = transformedPlane / math.length(transformedPlane.xyz);
                }

                if (type != IntersectionType.Intersection)
                {
                    intersectingPlaneIndices0.ResizeUninitialized(mesh0.localPlanes.Length);
                    for (int i = 0; i < intersectingPlaneIndices0.Length; i++) intersectingPlaneIndices0[i] = i;
                    intersectingPlaneIndices1.ResizeUninitialized(mesh1.localPlanes.Length);
                    for (int i = 0; i < intersectingPlaneIndices1.Length; i++) intersectingPlaneIndices1[i] = i;

                    usedVertices0.Clear();
                    usedVertices1.Clear();
                    usedVertices0.ResizeUninitialized(mesh0.vertices.Length);
                    usedVertices1.ResizeUninitialized(mesh1.vertices.Length);
                    for (int i = 0; i < usedVertices0.Length; i++) usedVertices0[i] = i;
                    for (int i = 0; i < usedVertices1.Length; i++) usedVertices1[i] = i;

                    intersectingPlanes0.Clear();
                    intersectingPlanes0.AddRange(localSpacePlanes0, localSpacePlanes0Length);
                    intersectingPlanes1.Clear();
                    intersectingPlanes1.AddRange(localSpacePlanes1, localSpacePlanes1Length);

                    usedPlanePairs0.Clear();
                    usedPlanePairs1.Clear();
                } else
                {
                    intersectingPlanes0.Clear();
                    intersectingPlanes0.ResizeUninitialized(intersectingPlaneIndices0.Length);
                    for (int i = 0; i < intersectingPlaneIndices0.Length; i++)
                        intersectingPlanes0[i] = localSpacePlanes0[intersectingPlaneIndices0[i]];

                    intersectingPlanes1.Clear();
                    intersectingPlanes1.ResizeUninitialized(intersectingPlaneIndices1.Length);
                    for (int i = 0; i < intersectingPlaneIndices1.Length; i++)
                        intersectingPlanes1[i] = localSpacePlanes1[intersectingPlaneIndices1[i]];

                    usedPlanePairs0.Clear();
                    usedPlanePairs1.Clear();
                    usedVertices0.Clear();
                    usedVertices1.Clear();
                    FindPlanePairs(usedVertices0, usedPlanePairs0, intersectingPlaneIndices0, localSpacePlanes0, float4x4.identity, ref mesh0);
                    FindPlanePairs(usedVertices1, usedPlanePairs1, intersectingPlaneIndices1, localSpacePlanes1, node1ToNode0, ref mesh1);

                    // decide which planes of brush1 align with brush2
                    // TODO: optimize
                    // TODO: should do this as a separate pass

                    for (int i1 = 0; i1 < intersectingPlaneIndices0.Length; i1++)
                    {
                        var p1          = intersectingPlaneIndices0[i1];
                        var localPlane1 = localSpacePlanes0[p1];
                        for (int i2 = 0; i2 < intersectingPlaneIndices1.Length; i2++)
                        {
                            var p2          = intersectingPlaneIndices1[i2];
                            var localPlane2 = localSpacePlanes1[p2];
                            if (math.abs(localPlane1.w - localPlane2.w) >= kPlaneDistanceEpsilon ||
                                math.dot(localPlane1.xyz, localPlane2.xyz) < kNormalEpsilon)
                            {
                                localPlane2 = -localPlane2;
                                if (math.abs(localPlane1.w - localPlane2.w) >= kPlaneDistanceEpsilon ||
                                    math.dot(localPlane1.xyz, localPlane2.xyz) < kNormalEpsilon)
                                    continue;

                                var surfaceInfo0 = surfaceInfos0[p1];
                                surfaceInfo0.interiorCategory = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
                                surfaceInfos0[p1] = surfaceInfo0;
                                var surfaceInfo1 = surfaceInfos1[p2];
                                surfaceInfo1.interiorCategory = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
                                surfaceInfos1[p2] = surfaceInfo1;
                            } else
                            {
                                var surfaceInfo0 = surfaceInfos0[p1];
                                surfaceInfo0.interiorCategory = (CategoryGroupIndex)CategoryIndex.Aligned;
                                surfaceInfos0[p1] = surfaceInfo0;
                                var surfaceInfo1 = surfaceInfos1[p2];
                                surfaceInfo1.interiorCategory = (CategoryGroupIndex)CategoryIndex.Aligned;
                                surfaceInfos1[p2] = surfaceInfo1;
                            }
                        }
                    }
                }



                var builder = new BlobBuilder(Allocator.Temp);
                ref var root = ref builder.ConstructRoot<BrushPairIntersection>();
                root.type = type;
                var brushIntersections = builder.Allocate(ref root.brushes, 2);
                brushIntersections[0] = new BrushIntersectionInfo()
                {
                    brushNodeIndex      = brushNodeIndex0,
                    blobMesh            = blobMesh0,
                    transformation      = transformations0.Value,
                    toOtherBrushSpace   = node0ToNode1
                };
                brushIntersections[1] = new BrushIntersectionInfo()
                {
                    brushNodeIndex      = brushNodeIndex1,
                    blobMesh            = blobMesh1,
                    transformation      = transformations1.Value,
                    toOtherBrushSpace   = node1ToNode0
                };
                builder.Construct(ref brushIntersections[0].localSpacePlaneIndices0,    intersectingPlaneIndices0);
                builder.Construct(ref brushIntersections[0].localSpacePlanes0,          intersectingPlanes0);
                builder.Construct(ref brushIntersections[0].usedPlanePairs,             usedPlanePairs0);
                builder.Construct(ref brushIntersections[0].usedVertices,               usedVertices0);
                builder.Construct(ref brushIntersections[0].surfaceInfos,               surfaceInfos0);

                builder.Construct(ref brushIntersections[1].localSpacePlaneIndices0,    intersectingPlaneIndices1);
                builder.Construct(ref brushIntersections[1].localSpacePlanes0,          intersectingPlanes1);
                builder.Construct(ref brushIntersections[1].usedPlanePairs,             usedPlanePairs1);
                builder.Construct(ref brushIntersections[1].usedVertices,               usedVertices1);
                builder.Construct(ref brushIntersections[1].surfaceInfos,               surfaceInfos1);
                var result = builder.CreateBlobAssetReference<BrushPairIntersection>(Allocator.Persistent);
                builder.Dispose();

                intersectingBrushes.AddNoResize(result);
            }
            intersectingPlaneIndices0.Dispose();
            intersectingPlaneIndices1.Dispose();
            intersectingPlanes0.Dispose();
            intersectingPlanes1.Dispose();
            usedPlanePairs0.Dispose();
            usedPlanePairs1.Dispose();
            surfaceInfos0.Dispose();
            surfaceInfos1.Dispose();
            usedVertices0.Dispose();
            usedVertices1.Dispose();
        }
    }
}
