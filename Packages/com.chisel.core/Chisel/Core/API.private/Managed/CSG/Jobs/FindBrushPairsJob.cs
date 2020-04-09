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
    public struct PlanePair
    {
        public float4 plane0;
        public float4 plane1;
        public float4 edgeVertex0;
        public float4 edgeVertex1;
        public int planeIndex0;
        public int planeIndex1;
    }

    [BurstCompile(CompileSynchronously = true)]
    struct FindBrushPairsJob : IJob
    {
        public struct Empty { }

        [NoAlias, ReadOnly] public int maxPairs;
        [NoAlias, ReadOnly] public NativeArray<int>      treeBrushIndices;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>> brushesTouchedByBrushes;
        [NoAlias, WriteOnly] public NativeList<BrushPair> uniqueBrushPairs;

        public void Execute()
        {
            var brushPairMap = new NativeHashMap<BrushPair, FindBrushPairsJob.Empty>(maxPairs, Allocator.Temp);
            var empty = new Empty();
            for (int b0 = 0; b0 < treeBrushIndices.Length; b0++)
            {
                var brushNodeIndex0         = treeBrushIndices[b0];
                //var brushesTouchedByBrush = touchedBrushesByTreeBrushes[b0];
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

                    var brushPair       = new BrushPair()
                    {
                        type            = intersection.type,
                        brushNodeIndex0 = brushNodeIndex0,
                        brushNodeIndex1 = brushNodeIndex1
                    };

                    if (brushNodeIndex0 > brushNodeIndex1) // ensures we do calculations exactly the same for each brush pair
                        brushPair.Flip();

                    if (brushPairMap.TryAdd(brushPair, empty))
                    {
                        uniqueBrushPairs.AddNoResize(brushPair);
                    }
                }
            }
            brushPairMap.Dispose();
        }
    }

    [BurstCompile(CompileSynchronously = true)]
    struct DisposeBrushPairsJob : IJobParallelFor
    {
        [NoAlias, ReadOnly] public NativeList<BlobAssetReference<BrushPairIntersection>> intersectingBrushes;

        public void Execute(int index)
        {
            if (index >= intersectingBrushes.Length ||
                intersectingBrushes.Length == 0)
                return;

            intersectingBrushes[index].Dispose();
        }
    }

    [BurstCompile(CompileSynchronously = true)]
    struct PrepareBrushPairIntersectionsJob : IJobParallelFor
    {
        const float kDistanceEpsilon        = CSGManagerPerformCSG.kDistanceEpsilon;
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        [NoAlias, ReadOnly] public NativeArray<BrushPair>                                                uniqueBrushPairs;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>                 brushMeshBlobLookup;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<NodeTransformations>>           transformations;
        [NoAlias, WriteOnly] public NativeList<BlobAssetReference<BrushPairIntersection>>.ParallelWriter intersectingBrushes;

        // TODO: turn into job
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void GetIntersectingPlanes(ref BlobArray<float4> localPlanes, ref BlobArray<float3> vertices, Bounds selfBounds, float4x4 treeToNodeSpaceInverseTransposed, int* intersectingPlanesPtr, out int intersectingPlaneLength)
        {
            var min = (float3)selfBounds.min;
            var max = (float3)selfBounds.max;

            intersectingPlaneLength = 0;
            var verticesLength = vertices.Length;
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
                    intersectingPlaneLength = 0;
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
                    float distance = math.dot(transformedPlane, new float4(vertices[v], 1));
                    minDistance = math.min(distance, minDistance);
                    maxDistance = math.max(distance, maxDistance);
                    onCount += (distance >= -kDistanceEpsilon && distance <= kDistanceEpsilon) ? 1 : 0;
                }

                // if all vertices are 'inside' this plane, then we're not truly intersecting with it
                if ((minDistance > kDistanceEpsilon || maxDistance < -kDistanceEpsilon))
                    continue;

                intersectingPlanesPtr[intersectingPlaneLength] = i;
                intersectingPlaneLength++;
            }
        }
        
        // TODO: turn into job
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void FindPlanePairs(ref BrushMeshBlob         mesh,
                                          ref BlobBuilderArray<int> intersectingPlanes,
                                          float4*                   localSpacePlanesPtr,
                                          int*                      vertexUsedPtr,
                                          float4x4                  vertexTransform,
                                          PlanePair*                usedPlanePairsPtr,
                                          out int                   usedPlanePairsLength,
                                          out int                   usedVerticesLength)
        {
            //using (new ProfileSample("FindPlanePairs"))
            {
                // TODO: this can be partially stored in brushmesh 
                // TODO: optimize
                
                ref var halfEdgePolygonIndices  = ref mesh.halfEdgePolygonIndices;
                ref var halfEdges               = ref mesh.halfEdges;

                usedVerticesLength = 0;
                usedPlanePairsLength = 0;
                var planeAvailable  = stackalloc byte[mesh.localPlanes.Length];
                {
                    {
                        for (int p = 0; p < intersectingPlanes.Length; p++)
                        {
                            planeAvailable[intersectingPlanes[p]] = 1;
                        }
                    }

                    for (int e = 0; e < halfEdges.Length; e++)
                    {
                        var twinIndex = halfEdges[e].twinIndex;
                        if (twinIndex < e)
                            continue;

                        var planeIndex0 = halfEdgePolygonIndices[e];
                        var planeIndex1 = halfEdgePolygonIndices[twinIndex];

                        //Debug.Assert(planeIndex0 != planeIndex1);

                        if (planeAvailable[planeIndex0] == 0 ||
                            planeAvailable[planeIndex1] == 0)
                            continue;

                        var plane0 = localSpacePlanesPtr[planeIndex0];
                        var plane1 = localSpacePlanesPtr[planeIndex1];

                        var vertexIndex0 = halfEdges[e].vertexIndex;
                        var vertexIndex1 = halfEdges[twinIndex].vertexIndex;

                        var vertex0 = math.mul(vertexTransform, new float4(mesh.vertices[vertexIndex0], 1));
                        var vertex1 = math.mul(vertexTransform, new float4(mesh.vertices[vertexIndex1], 1));

                        if (vertexUsedPtr[vertexIndex0] == 0) { vertexUsedPtr[vertexIndex0] = vertexIndex0 + 1; usedVerticesLength++; }
                        if (vertexUsedPtr[vertexIndex1] == 0) { vertexUsedPtr[vertexIndex1] = vertexIndex1 + 1; usedVerticesLength++; }
                        usedPlanePairsPtr[usedPlanePairsLength] = new PlanePair()
                        {
                            plane0 = plane0,
                            plane1 = plane1,
                            edgeVertex0 = vertex0,
                            edgeVertex1 = vertex1,
                            planeIndex0 = planeIndex0,
                            planeIndex1 = planeIndex1
                        };
                        usedPlanePairsLength++;
                    }
                }
            }
        }

        public unsafe void Execute(int index)
        {
            if (index >= uniqueBrushPairs.Length)
                return;

            var brushPair       = uniqueBrushPairs[index];
            var brushNodeIndex0 = brushPair.brushNodeIndex0;
            var brushNodeIndex1 = brushPair.brushNodeIndex1;

            if (!brushMeshBlobLookup.TryGetValue(brushNodeIndex0, out BlobAssetReference<BrushMeshBlob> blobMesh0) ||
                !brushMeshBlobLookup.TryGetValue(brushNodeIndex1, out BlobAssetReference<BrushMeshBlob> blobMesh1))
                //continue;
                return;

            var type = brushPair.type;
            if (type != IntersectionType.Intersection &&
                type != IntersectionType.AInsideB &&
                type != IntersectionType.BInsideA)
                return;


            var transformations0 = transformations[brushNodeIndex0];
            var transformations1 = transformations[brushNodeIndex1];

            var node1ToNode0            = math.mul(transformations0.Value.treeToNode, transformations1.Value.nodeToTree);
            var node0ToNode1            = math.mul(transformations1.Value.treeToNode, transformations0.Value.nodeToTree);
            var inversedNode1ToNode0    = math.transpose(node0ToNode1);
            var inversedNode0ToNode1    = math.transpose(node1ToNode0);

            ref var mesh0 = ref blobMesh0.Value;
            ref var mesh1 = ref blobMesh1.Value;

            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<BrushPairIntersection>();
            root.type = type;

            var brushIntersections = builder.Allocate(ref root.brushes, 2);
            brushIntersections[0] = new BrushIntersectionInfo()
            {
                brushNodeIndex      = brushNodeIndex0,
                nodeToTreeSpace    = transformations0.Value.nodeToTree,
                toOtherBrushSpace   = node0ToNode1
            };
            brushIntersections[1] = new BrushIntersectionInfo()
            {
                brushNodeIndex      = brushNodeIndex1,
                nodeToTreeSpace    = transformations1.Value.nodeToTree,
                toOtherBrushSpace   = node1ToNode0
            };

            BlobBuilderArray<int> intersectingPlaneIndices0, intersectingPlaneIndices1;
            if (type == IntersectionType.Intersection)
            {
                {
                    int intersectingPlanesLength = 0;
                    var intersectingPlanesPtr = stackalloc int[mesh0.localPlanes.Length];
                    GetIntersectingPlanes(ref mesh0.localPlanes, ref mesh1.vertices, mesh1.localBounds, inversedNode0ToNode1, intersectingPlanesPtr, out intersectingPlanesLength);
                    if (intersectingPlanesLength == 0) { builder.Dispose(); return; }
                    intersectingPlaneIndices0 = builder.Construct(ref brushIntersections[0].localSpacePlaneIndices0, intersectingPlanesPtr, intersectingPlanesLength);
                }

                {
                    int intersectingPlanesLength = 0;
                    var intersectingPlanesPtr = stackalloc int[mesh1.localPlanes.Length];
                    GetIntersectingPlanes(ref mesh1.localPlanes, ref mesh0.vertices, mesh0.localBounds, inversedNode1ToNode0, intersectingPlanesPtr, out intersectingPlanesLength);
                    if (intersectingPlanesLength == 0) { builder.Dispose(); return; }
                    intersectingPlaneIndices1 = builder.Construct(ref brushIntersections[1].localSpacePlaneIndices0, intersectingPlanesPtr, intersectingPlanesLength);
                }
            } else
            //if (type == IntersectionType.AInsideB || type == IntersectionType.BInsideA)
            {
                intersectingPlaneIndices0 = builder.Allocate(ref brushIntersections[0].localSpacePlaneIndices0, mesh0.localPlanes.Length);
                intersectingPlaneIndices1 = builder.Allocate(ref brushIntersections[1].localSpacePlaneIndices0, mesh1.localPlanes.Length);
                for (int i = 0; i < intersectingPlaneIndices0.Length; i++) intersectingPlaneIndices0[i] = i;
                for (int i = 0; i < intersectingPlaneIndices1.Length; i++) intersectingPlaneIndices1[i] = i;
            }
            

            var inverseNodeToTreeSpaceMatrix0 = math.transpose(transformations0.Value.treeToNode);
            var inverseNodeToTreeSpaceMatrix1 = math.transpose(transformations1.Value.treeToNode);


            var surfaceInfos0 = builder.Allocate(ref brushIntersections[0].surfaceInfos, mesh0.localPlanes.Length);
            var surfaceInfos1 = builder.Allocate(ref brushIntersections[1].surfaceInfos, mesh1.localPlanes.Length);
            for (int i = 0; i < surfaceInfos0.Length; i++)
            {
                surfaceInfos0[i] = new SurfaceInfo()
                {
                    interiorCategory    = (CategoryGroupIndex)CategoryIndex.Inside,
                    basePlaneIndex      = (ushort)i,
                    brushNodeIndex      = brushNodeIndex1
                };
            }
            for (int i = 0; i < surfaceInfos1.Length; i++)
            {
                surfaceInfos1[i] = new SurfaceInfo()
                {
                    interiorCategory    = (CategoryGroupIndex)CategoryIndex.Inside,
                    basePlaneIndex      = (ushort)i,
                    brushNodeIndex      = brushNodeIndex0
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
                var intersectingPlanes0 = builder.Construct(ref brushIntersections[0].localSpacePlanes0, localSpacePlanes0, localSpacePlanes0Length);
                var intersectingPlanes1 = builder.Construct(ref brushIntersections[1].localSpacePlanes0, localSpacePlanes1, localSpacePlanes1Length);

                builder.Allocate(ref brushIntersections[0].usedPlanePairs, 0);
                builder.Allocate(ref brushIntersections[1].usedPlanePairs, 0);

                var usedVertices0 = builder.Allocate(ref brushIntersections[0].usedVertices, mesh0.vertices.Length);
                var usedVertices1 = builder.Allocate(ref brushIntersections[1].usedVertices, mesh1.vertices.Length);
                for (int i = 0; i < usedVertices0.Length; i++) usedVertices0[i] = i;
                for (int i = 0; i < usedVertices1.Length; i++) usedVertices1[i] = i;
            } else
            {
                var intersectingPlanes0 = builder.Allocate(ref brushIntersections[0].localSpacePlanes0, intersectingPlaneIndices0.Length);
                var intersectingPlanes1 = builder.Allocate(ref brushIntersections[1].localSpacePlanes0, intersectingPlaneIndices1.Length);
                for (int i = 0; i < intersectingPlaneIndices0.Length; i++)
                    intersectingPlanes0[i] = localSpacePlanes0[intersectingPlaneIndices0[i]];
                for (int i = 0; i < intersectingPlaneIndices1.Length; i++)
                    intersectingPlanes1[i] = localSpacePlanes1[intersectingPlaneIndices1[i]];

                {
                    int usedVerticesLength;
                    var vertexUsedPtr = stackalloc int[mesh0.vertices.Length];
                    {
                        var usedPlanePairsPtr = stackalloc PlanePair[mesh0.halfEdges.Length];
                        FindPlanePairs(ref mesh0, ref intersectingPlaneIndices0, localSpacePlanes0, vertexUsedPtr, float4x4.identity, usedPlanePairsPtr, out int usedPlanePairsLength, out usedVerticesLength);
                        builder.Construct(ref brushIntersections[0].usedPlanePairs, usedPlanePairsPtr, usedPlanePairsLength);
                    }
                    var usedVertices = builder.Allocate(ref brushIntersections[0].usedVertices, usedVerticesLength);
                    if (usedVerticesLength > 0)
                    {
                        for (int i = 0, n = 0; i < mesh0.vertices.Length; i++)
                        {
                            if (vertexUsedPtr[i] == 0)
                                continue;
                            usedVertices[n] = mesh0.vertices[vertexUsedPtr[i] - 1];
                            n++;
                        }
                    }
                }
                {
                    int usedVerticesLength;
                    var vertexUsedPtr = stackalloc int[mesh1.vertices.Length];
                    {
                        var usedPlanePairsPtr = stackalloc PlanePair[mesh1.halfEdges.Length];
                        FindPlanePairs(ref mesh1, ref intersectingPlaneIndices1, localSpacePlanes1, vertexUsedPtr, node1ToNode0, usedPlanePairsPtr, out int usedPlanePairsLength, out usedVerticesLength);
                        builder.Construct(ref brushIntersections[1].usedPlanePairs, usedPlanePairsPtr, usedPlanePairsLength);
                    }
                    var usedVertices = builder.Allocate(ref brushIntersections[1].usedVertices, usedVerticesLength);
                    if (usedVerticesLength > 0)
                    {
                        for (int i = 0, n = 0; i < mesh1.vertices.Length; i++)
                        {
                            if (vertexUsedPtr[i] == 0)
                                continue;
                            usedVertices[n] = mesh1.vertices[vertexUsedPtr[i] - 1];
                            n++;
                        }
                    }
                }



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

            var result = builder.CreateBlobAssetReference<BrushPairIntersection>(Allocator.Persistent);
            builder.Dispose();

            intersectingBrushes.AddNoResize(result);
        }
    }
}
