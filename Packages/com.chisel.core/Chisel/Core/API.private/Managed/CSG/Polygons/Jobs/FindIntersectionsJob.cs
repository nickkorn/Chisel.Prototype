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
    unsafe struct SharedPlaneData : IJob, IDisposable
    {
        const float kDistanceEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kNormalEpsilon = CSGManagerPerformCSG.kNormalEpsilon;

        public CSGTreeBrush             treeBrush0;
        public CSGTreeBrush             treeBrush1;
        public IntersectionType         intersectionType;

        public BlobAssetReference<BrushMeshBlob> blobMesh0;
        public BlobAssetReference<BrushMeshBlob> blobMesh1;

        public float4x4                 nodeToTreeSpaceMatrix0;
        public float4x4                 treeToNodeSpaceMatrix0;
        public float4x4                 nodeToTreeSpaceMatrix1;
        public float4x4                 treeToNodeSpaceMatrix1;
        public float4x4                 node1ToNode0;
            
        public NativeList<int>          intersectingPlaneIndices0;
        public NativeList<int>          intersectingPlaneIndices1;

        public NativeList<float4>       intersectingPlanes0;
        public NativeList<float4>       intersectingPlanes1;

        public NativeList<PlanePair>    usedPlanePairs0;
        public NativeList<PlanePair>    usedPlanePairs1;

        public NativeList<int>          usedVertices0;
        public NativeList<int>          usedVertices1;

        public NativeArray<SurfaceInfo> surfaceCategory0;
        public NativeArray<SurfaceInfo> surfaceCategory1;

        public VertexSoup               vertexSoup0;
        public VertexSoup               vertexSoup1;
            

        public SharedPlaneData(CSGManager.BrushInfo brushInfo0, CSGManager.BrushInfo brushInfo1, CSGTreeBrush brush0, BlobAssetReference<BrushMeshBlob> blobMesh0, CSGTreeBrush brush1, BlobAssetReference<BrushMeshBlob> blobMesh1, IntersectionType intersectionType, Allocator allocator)
        {
            using (new ProfileSample("SharedPlaneData_Constructor"))
            {
                var nodeToTreeSpaceMatrix0 = (float4x4)brush0.NodeToTreeSpaceMatrix;
                var treeToNodeSpaceMatrix0 = (float4x4)brush0.TreeToNodeSpaceMatrix;
                var nodeToTreeSpaceMatrix1 = (float4x4)brush1.NodeToTreeSpaceMatrix;
                var treeToNodeSpaceMatrix1 = (float4x4)brush1.TreeToNodeSpaceMatrix;

                ref var mesh0 = ref blobMesh0.Value;
                ref var mesh1 = ref blobMesh1.Value;

                this.treeBrush0                  = brush0;
                this.treeBrush1                  = brush1;
                this.blobMesh0                   = blobMesh0;
                this.blobMesh1                   = blobMesh1;

                this.vertexSoup0                 = brushInfo0.brushOutputLoops.vertexSoup;
                this.vertexSoup1                 = brushInfo1.brushOutputLoops.vertexSoup;

                this.nodeToTreeSpaceMatrix0      = nodeToTreeSpaceMatrix0;
                this.treeToNodeSpaceMatrix0      = treeToNodeSpaceMatrix0;
                this.nodeToTreeSpaceMatrix1      = nodeToTreeSpaceMatrix1;
                this.treeToNodeSpaceMatrix1      = treeToNodeSpaceMatrix1;
                this.node1ToNode0                = math.mul(treeToNodeSpaceMatrix0, nodeToTreeSpaceMatrix1);

                this.intersectionType            = intersectionType;

                this.intersectingPlaneIndices0   = new NativeList<int>(mesh0.localPlanes.Length, allocator);
                this.intersectingPlaneIndices1   = new NativeList<int>(mesh1.localPlanes.Length, allocator);

                this.usedPlanePairs0             = new NativeList<PlanePair>(mesh0.halfEdges.Length, allocator);
                this.usedPlanePairs1             = new NativeList<PlanePair>(mesh1.halfEdges.Length, allocator);

                this.surfaceCategory0            = new NativeArray<SurfaceInfo>(mesh0.localPlanes.Length, allocator, NativeArrayOptions.ClearMemory); // all set to Inside (0)
                this.surfaceCategory1            = new NativeArray<SurfaceInfo>(mesh1.localPlanes.Length, allocator, NativeArrayOptions.ClearMemory); // all set to Inside (0)

                this.usedVertices0               = new NativeList<int>(mesh0.vertices.Length, allocator);
                this.usedVertices1               = new NativeList<int>(mesh1.vertices.Length, allocator);

                this.intersectingPlanes0         = new NativeList<float4>(mesh0.localPlanes.Length, allocator);
                this.intersectingPlanes1         = new NativeList<float4>(mesh1.localPlanes.Length, allocator);
            }
        }

        public void Dispose()
        {
            if (intersectingPlaneIndices0.IsCreated) intersectingPlaneIndices0.Dispose();
            if (intersectingPlaneIndices1.IsCreated) intersectingPlaneIndices1.Dispose();

            if (intersectingPlanes0.IsCreated) intersectingPlanes0.Dispose();
            if (intersectingPlanes1.IsCreated) intersectingPlanes1.Dispose();

            if (usedPlanePairs0.IsCreated) usedPlanePairs0.Dispose();
            if (usedPlanePairs1.IsCreated) usedPlanePairs1.Dispose();

            if (usedVertices0.IsCreated) usedVertices0.Dispose();
            if (usedVertices1.IsCreated) usedVertices1.Dispose();

            if (surfaceCategory0.IsCreated) surfaceCategory0.Dispose();
            if (surfaceCategory1.IsCreated) surfaceCategory1.Dispose();
        }

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

        public void Execute()
        {
            ref var mesh0 = ref blobMesh0.Value;
            ref var mesh1 = ref blobMesh1.Value;

            var node0ToNode1 = math.mul(treeToNodeSpaceMatrix1, nodeToTreeSpaceMatrix0);
            var inversedNode1ToNode0 = math.transpose(node0ToNode1);

            var inversedNode0ToNode1 = math.transpose(node1ToNode0);
            var inverseNodeToTreeSpaceMatrix0 = math.transpose(treeToNodeSpaceMatrix0);
            var inverseNodeToTreeSpaceMatrix1 = math.transpose(treeToNodeSpaceMatrix1);

            for (int i = 0; i < surfaceCategory0.Length; i++)
            {
                var localPlaneVector = mesh0.localPlanes[i];
                var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix0, localPlaneVector);
                var length = math.length(worldPlaneVector.xyz);
                worldPlaneVector /= length;

                surfaceCategory0[i] = new SurfaceInfo()
                {
                    interiorCategory = (CategoryGroupIndex)CategoryIndex.Inside,
                    basePlaneIndex = i,
                    brush = treeBrush1,
                    worldPlane = worldPlaneVector,
                    layers = mesh0.polygons[i].layerDefinition
                };
            }
            for (int i = 0; i < surfaceCategory1.Length; i++)
            {
                var localPlaneVector = mesh1.localPlanes[i];
                var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix1, localPlaneVector);
                var length = math.length(worldPlaneVector.xyz);
                worldPlaneVector /= length;

                surfaceCategory1[i] = new SurfaceInfo()
                {
                    interiorCategory = (CategoryGroupIndex)CategoryIndex.Inside,
                    basePlaneIndex = i,
                    brush = treeBrush0,
                    worldPlane = worldPlaneVector,
                    layers = mesh1.polygons[i].layerDefinition
                };
            }

            if (intersectionType == IntersectionType.Intersection)
            {
                GetIntersectingPlanes(ref mesh0.localPlanes, ref mesh1.vertices, mesh1.localBounds, inversedNode0ToNode1, intersectingPlaneIndices0);
                GetIntersectingPlanes(ref mesh1.localPlanes, ref mesh0.vertices, mesh0.localBounds, inversedNode1ToNode0, intersectingPlaneIndices1);
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

            if (intersectingPlaneIndices0.Length == 0 ||
                intersectingPlaneIndices1.Length == 0)
            {
                if (intersectionType == IntersectionType.Intersection)
                {
#if UNITY_EDITOR
                    //Debug.Assert(intersectingPlaneIndices1.Length == 0 && intersectingPlaneIndices0.Length == 0, $"Expected intersection, but no intersection found between {brush0.NodeID} & {brush1.NodeID}");
                    //Debug.LogError($"{brush0.NodeID}", UnityEditor.EditorUtility.InstanceIDToObject(brush0.UserID));
                    //Debug.LogError($"{brush1.NodeID}", UnityEditor.EditorUtility.InstanceIDToObject(brush1.UserID));
#endif
                    intersectionType = IntersectionType.NoIntersection;
                }

                intersectingPlaneIndices0.ResizeUninitialized(mesh0.localPlanes.Length);
                intersectingPlaneIndices1.ResizeUninitialized(mesh1.localPlanes.Length);

                for (int i = 0; i < intersectingPlaneIndices0.Length; i++) intersectingPlaneIndices0[i] = i;
                for (int i = 0; i < intersectingPlaneIndices1.Length; i++) intersectingPlaneIndices1[i] = i;

                usedVertices0.ResizeUninitialized(mesh0.vertices.Length);
                usedVertices1.ResizeUninitialized(mesh1.vertices.Length);
                for (int i = 0; i < usedVertices0.Length; i++) usedVertices0[i] = i;
                for (int i = 0; i < usedVertices1.Length; i++) usedVertices1[i] = i;

                intersectingPlanes0.AddRange(localSpacePlanes0, localSpacePlanes0Length);
                intersectingPlanes1.AddRange(localSpacePlanes1, localSpacePlanes1Length);

                return;
            }


            intersectingPlanes0.ResizeUninitialized(intersectingPlaneIndices0.Length);
            intersectingPlanes1.ResizeUninitialized(intersectingPlaneIndices1.Length);
            for (int i = 0; i < intersectingPlaneIndices0.Length; i++)
                intersectingPlanes0[i] = localSpacePlanes0[intersectingPlaneIndices0[i]];

            for (int i = 0; i < intersectingPlaneIndices1.Length; i++)
                intersectingPlanes1[i] = localSpacePlanes1[intersectingPlaneIndices1[i]];


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

                        var surfaceInfo0 = surfaceCategory0[p1];
                        surfaceInfo0.interiorCategory = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
                        surfaceCategory0[p1] = surfaceInfo0;
                        var surfaceInfo1 = surfaceCategory1[p2];
                        surfaceInfo1.interiorCategory = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
                        surfaceCategory1[p2] = surfaceInfo1;
                    } else
                    {
                        var surfaceInfo0 = surfaceCategory0[p1];
                        surfaceInfo0.interiorCategory = (CategoryGroupIndex)CategoryIndex.Aligned;
                        surfaceCategory0[p1] = surfaceInfo0;
                        var surfaceInfo1 = surfaceCategory1[p2];
                        surfaceInfo1.interiorCategory = (CategoryGroupIndex)CategoryIndex.Aligned;
                        surfaceCategory1[p2] = surfaceInfo1;
                    }
                }
            }
        }
    }


    [BurstCompile(Debug = false)]
    unsafe struct FindIntersectionsJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kDistanceEpsilon        = CSGManagerPerformCSG.kDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<PlanePair> usedPlanePairs1;
        [ReadOnly] public VertexSoup            vertexSoup1;
        [ReadOnly] public NativeList<float4>    intersectingPlanes0;
        [ReadOnly] public NativeList<float4>    intersectingPlanes1;
        [ReadOnly] public float4x4              nodeToTreeSpaceMatrix0;

        [WriteOnly] public NativeStream.Writer  foundVertices;
            
        public void Execute(int index)
        {
            foundVertices.BeginForEachIndex(index);
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

                    if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes1, localVertex) &&
                        !CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes0, localVertex))
                    {
                        foundVertices.Write(new VertexAndPlanePair()
                        {
                            vertex = worldVertex,
                            plane0 = (ushort)planePair.planeIndex0,
                            plane1 = (ushort)planePair.planeIndex1
                        });
                    }
                }
            }

            foundVertices.EndForEachIndex();
        }
    }

    [BurstCompile(Debug = false)]
    unsafe struct InsertIntersectionVerticesJob : IJob
    {
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeStream.Reader vertexReader;
        [ReadOnly] public NativeList<int> intersectingPlaneIndices0;
        
        public VertexSoup brushVertices0;
        public VertexSoup brushVertices1;

        [WriteOnly] public NativeList<PlaneVertexIndexPair> outputIndices0;
        [WriteOnly] public NativeList<PlaneVertexIndexPair> outputIndices1;

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
                var vertexIndex1 = brushVertices0.Add(worldVertex);
                var vertexIndex2 = brushVertices1.Add(worldVertex);

                outputIndices0.Add(new PlaneVertexIndexPair() { planeIndex = (ushort)intersectingPlaneIndices0[index - 1], vertexIndex = vertexIndex1 });
                outputIndices1.Add(new PlaneVertexIndexPair() { planeIndex = plane0, vertexIndex = vertexIndex2 });
                outputIndices1.Add(new PlaneVertexIndexPair() { planeIndex = plane1, vertexIndex = vertexIndex2 });

                while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                    vertexReader.BeginForEachIndex(index++);
            }
        }
    }

    [BurstCompile(Debug = false)]
    unsafe struct SortLoopsJob : IJob
    {
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeArray<SurfaceInfo>  surfaceCategory; // TODO: only use plane information here
        [ReadOnly] public VertexSoup                vertexSoup;

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
        unsafe void SortIndices(ushort* indicesPtr, int offset, int indicesCount, float3 normal)
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

            var vertices = (float3*)vertexSoup.vertices.GetUnsafePtr();
            var centroid = FindPolygonCentroid(vertices, indicesPtr, offset, indicesCount);
            var center = new float2(math.dot(tangentX, centroid), // distance in direction of tangentX
                                    math.dot(tangentY, centroid)); // distance in direction of tangentY


            var sortedStack = (int2*)UnsafeUtility.Malloc(indicesCount * 2 * sizeof(int2), 4, Allocator.TempJob);
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
                    sortedStack[sortedStackLength] = new int2(l, right);
                    sortedStackLength++;
                }
                if (left < r)
                {
                    sortedStack[sortedStackLength] = new int2(left, r);
                    sortedStackLength++;
                }
            }
            UnsafeUtility.Free(sortedStack, Allocator.TempJob);
        }
#endregion


        public void Execute()
        {
            if (foundIndices.Length < 3)
            {
                foundIndices.Clear();
                return;
            }
            NativeSortExtension.Sort(foundIndices); // <- we can only do this if it's readonly!

            // Now that our indices are sorted by planeIndex, we can segment them by start/end offset
            var previousPlaneIndex  = foundIndices[0].planeIndex;
            var previousVertexIndex = foundIndices[0].vertexIndex;
            uniqueIndices.Add(previousVertexIndex);
            var loopStart = 0;
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
                    planeIndexOffsets.Add(new PlaneIndexOffsetLength()
                    {
                        offset = (ushort)loopStart,
                        length = (ushort)(uniqueIndices.Length - loopStart),
                        planeIndex = previousPlaneIndex
                    });
                    loopStart = uniqueIndices.Length;
                }

                uniqueIndices.Add(vertexIndex);
                previousVertexIndex = vertexIndex;
                previousPlaneIndex = planeIndex;
            }
            planeIndexOffsets.Add(new PlaneIndexOffsetLength()
            {
                length = (ushort)(uniqueIndices.Length - loopStart),
                offset = (ushort)loopStart,
                planeIndex = previousPlaneIndex
            });


            // TODO: do in separate pass?

            
            // For each segment, we now sort our vertices within each segment, 
            // making the assumption that they are convex
            var indicesPtr = (ushort*)uniqueIndices.GetUnsafeReadOnlyPtr();
            for (int n = planeIndexOffsets.Length - 1; n >= 0; n--)
            {
                var planeIndexOffset    = planeIndexOffsets[n];
                var length              = planeIndexOffset.length;
                if (length <= 2)
                {
                    planeIndexOffsets.RemoveAtSwapBack(n);
                    continue;
                }

                var offset      = planeIndexOffset.offset;
                var planeIndex  = planeIndexOffset.planeIndex;
                // TODO: use plane information instead
                SortIndices(indicesPtr, offset, length, surfaceCategory[planeIndex].worldPlane.xyz);
            }
        }
    }

    // TODO: make burstable (somehow)
    unsafe struct CreateLoopsJob : IJob
    {
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<ushort>                    uniqueIndices;
        [ReadOnly] public NativeList<PlaneIndexOffsetLength>    planeIndexOffsets;
        [ReadOnly] public NativeArray<SurfaceInfo>              surfaceCategories;

        [WriteOnly] public List<Loop>[]     outputSurfaces;

        public void Execute()
        {
            if (uniqueIndices.Length < 3)
                return;
            
            var indicesPtr              = (ushort*)uniqueIndices.GetUnsafeReadOnlyPtr();
            var planeIndexOffsetsPtr    = (PlaneIndexOffsetLength*)planeIndexOffsets.GetUnsafeReadOnlyPtr();
            var planeIndexOffsetsLength = planeIndexOffsets.Length;
            var surfaceCategoriesPtr    = (SurfaceInfo*)surfaceCategories.GetUnsafeReadOnlyPtr();
            
            for (int n = 0; n < planeIndexOffsetsLength; n++)
            {
                var planeIndexLength    = planeIndexOffsetsPtr[n];
                var offset              = planeIndexLength.offset;
                var loopLength          = planeIndexLength.length;
                var basePlaneIndex      = planeIndexLength.planeIndex;
                var surfaceCategory     = surfaceCategoriesPtr[basePlaneIndex];
                outputSurfaces[basePlaneIndex].Add(new Loop(surfaceCategory, indicesPtr, offset, loopLength));
            }
        }
    }
}