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

        public unsafe void Execute()
        {
            var intersectingPlaneIndices0 = new NativeList<int>(Allocator.Temp);
            var intersectingPlaneIndices1 = new NativeList<int>(Allocator.Temp);
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

                var node1ToNode0 = math.mul(transformations0.Value.treeToNode, transformations1.Value.nodeToTree);
                var node0ToNode1 = math.mul(transformations1.Value.treeToNode, transformations0.Value.nodeToTree);

                ref var mesh0 = ref blobMesh0.Value;
                ref var mesh1 = ref blobMesh1.Value;

                intersectingPlaneIndices0.Clear();
                intersectingPlaneIndices1.Clear();
                var type = brushPair.type;
                if (type == IntersectionType.Intersection)
                {
                    var inversedNode1ToNode0 = math.transpose(node0ToNode1);
                    var inversedNode0ToNode1 = math.transpose(node1ToNode0);
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
                }

                if (intersectingPlaneIndices0.Length == 0 ||
                    intersectingPlaneIndices1.Length == 0)
                    continue;
                /*
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
                */

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
                builder.Construct(ref brushIntersections[0].localSpacePlanes0, intersectingPlaneIndices0);
                builder.Construct(ref brushIntersections[1].localSpacePlanes0, intersectingPlaneIndices1);
                var result = builder.CreateBlobAssetReference<BrushPairIntersection>(Allocator.Persistent);
                builder.Dispose();

                //var inverseNodeToTreeSpaceMatrix0   = math.transpose(transformations0.Value.treeToNode);
                //var inverseNodeToTreeSpaceMatrix1   = math.transpose(transformations1.Value.treeToNode);

                intersectingBrushes.AddNoResize(result);


                // TODO: put this in a burstable data structure
                //var loops01 = new SurfaceLoops(blobMesh0.Value.localPlanes.Length);
                //var loops10 = new SurfaceLoops(blobMesh1.Value.localPlanes.Length);

                //var brushOutputLoops0 = CSGManager.GetBrushInfo(brushNodeID0).brushOutputLoops;
                //var brushOutputLoops1 = CSGManager.GetBrushInfo(brushNodeID1).brushOutputLoops;
                //var vertexSoup0 = brushOutputLoops0.vertexSoup;
                //var vertexSoup1 = brushOutputLoops1.vertexSoup;
                /*
                var transformations0 = transformations[brushNodeIndex0];
                var transformations1 = transformations[brushNodeIndex1];


                using (var sharedPlaneData = new SharedPlaneData(brushNodeIndex0, blobMesh0, transformations0,
                                                                    brushNodeIndex1, blobMesh1, transformations1,
                                                                    type, Allocator.TempJob))
                {
                    sharedPlaneData.Run();
                    if (sharedPlaneData.intersectingPlanes0.Length == 0 && sharedPlaneData.intersectingPlanes1.Length == 0)
                    {
                        //loops01.Dispose();
                        //loops10.Dispose();
                        continue;
                    }
                }
                */
            }
            intersectingPlaneIndices0.Dispose();
            intersectingPlaneIndices1.Dispose();
        }
    }
}
