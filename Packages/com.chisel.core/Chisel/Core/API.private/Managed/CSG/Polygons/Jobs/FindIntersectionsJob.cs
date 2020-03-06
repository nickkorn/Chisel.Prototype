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
        public float4 Plane0;
        public float4 Plane1;
        //public double4 N0;
        //public double4 N1;
        public int P0;
        public int P1;
    }
    
    [BurstCompile]
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

            this.intersectingPlaneIndices0   = new NativeList<int>(mesh0.planes.Length, allocator);
            this.intersectingPlaneIndices1   = new NativeList<int>(mesh1.planes.Length, allocator);

            this.usedPlanePairs0             = new NativeList<PlanePair>(mesh0.halfEdges.Length, allocator);
            this.usedPlanePairs1             = new NativeList<PlanePair>(mesh1.halfEdges.Length, allocator);

            this.surfaceCategory0            = new NativeArray<SurfaceInfo>(mesh0.planes.Length, allocator, NativeArrayOptions.ClearMemory); // all set to Inside (0)
            this.surfaceCategory1            = new NativeArray<SurfaceInfo>(mesh1.planes.Length, allocator, NativeArrayOptions.ClearMemory); // all set to Inside (0)

            this.usedVertices0               = new NativeList<int>(mesh0.vertices.Length, allocator);
            this.usedVertices1               = new NativeList<int>(mesh1.vertices.Length, allocator);

            this.intersectingPlanes0         = new NativeList<float4>(mesh0.planes.Length, allocator);
            this.intersectingPlanes1         = new NativeList<float4>(mesh1.planes.Length, allocator);
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
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void GetIntersectingPlanes(ref BlobArray<float4> localPlanes, ref BlobArray<float3> vertices, Bounds selfBounds, float4x4 treeToNodeSpaceInverseTransposed, NativeList<int> intersectingPlanes)
        {
            var min = (float3)selfBounds.min;
            var max = (float3)selfBounds.max;

            var verticesLength  = vertices.Length;
            var verticesPtr     = (float3*)vertices.GetUnsafePtr();

            for (int i = 0; i < localPlanes.Length; i++)
            {
                // bring plane into local space of mesh, the same space as the bounds of the mesh
                
                var localPlane          = localPlanes[i];

                // note: a transpose is part of this transformation
                var transformedPlane    = math.mul(treeToNodeSpaceInverseTransposed, localPlane);
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
                if ((minDistance > kDistanceEpsilon || maxDistance < -kDistanceEpsilon) 
                    || 
                    ((maxDistance >= -kDistanceEpsilon && maxDistance <= kDistanceEpsilon) &&
                     (onCount == 1 || onCount == 2)) // If we have a polygon intersecting with a plane, we use it
                    )
                    continue;

                intersectingPlanes.Add(i);
            }
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void FindPlanePairs(NativeList<int>       usedVertices,
                                          NativeList<PlanePair> usedPlanePairs, 
                                          NativeList<int>       intersectingPlanes,
                                          float4*               localSpacePlanesPtr,
                                          ref BrushMeshBlob     mesh)
        {
            // TODO: this can be partially stored in brushmesh 
            // TODO: optimize
            usedPlanePairs.Clear();

            var halfEdgesLength             = mesh.halfEdges.Length;
            var halfEdgesPtr                = (BrushMesh.HalfEdge*)mesh.halfEdges.GetUnsafePtr();
            var halfEdgePolygonIndicesPtr   = (int*)mesh.halfEdgePolygonIndices.GetUnsafePtr();

            var intersectingPlanesLength    = intersectingPlanes.Length;
            var intersectingPlanesPtr       = (int*)intersectingPlanes.GetUnsafeReadOnlyPtr();

            var vertexUsed = stackalloc byte[mesh.vertices.Length];
            var planeAvailable = stackalloc byte[mesh.planes.Length];
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

                    var pI0 = halfEdgePolygonIndicesPtr[e];
                    var pI1 = halfEdgePolygonIndicesPtr[twinIndex];

                    if (planeAvailable[pI0] == 0 ||
                        planeAvailable[pI1] == 0)
                        continue;

                    var vI0 = halfEdgesPtr[e].vertexIndex;
                    var vI1 = halfEdgesPtr[twinIndex].vertexIndex;

                    //PlaneExtensions.IntersectionFirst(localSpacePlanes1[pI0], localSpacePlanes1[pI1], out double4 N0, out double4 N1);
                    if (vertexUsed[vI0] == 0) { vertexUsed[vI0] = 1; usedVertices.Add(vI0); }
                    if (vertexUsed[vI1] == 0) { vertexUsed[vI1] = 1; usedVertices.Add(vI1); }
                    usedPlanePairs.Add(new PlanePair()
                    {
                        Plane0 = localSpacePlanesPtr[pI0], 
                        Plane1 = localSpacePlanesPtr[pI1],
                        P0 = pI0,
                        P1 = pI1
                    });
                }
            }
        }

        // TODO: get rid of references to mesh0/mesh1 and make this a job
        public void Execute()
        {
            ref var mesh0 = ref blobMesh0.Value;
            ref var mesh1 = ref blobMesh1.Value;

            var node0ToNode1                    = math.mul(treeToNodeSpaceMatrix1, nodeToTreeSpaceMatrix0);
            var inversedNode1ToNode0            = math.transpose(node0ToNode1);

            var inversedNode0ToNode1            = math.transpose(node1ToNode0);
            var inverseNodeToTreeSpaceMatrix0   = math.transpose(treeToNodeSpaceMatrix0);
            var inverseNodeToTreeSpaceMatrix1   = math.transpose(treeToNodeSpaceMatrix1);

            for (int i = 0; i < surfaceCategory0.Length; i++)
            {
                var localPlaneVector = mesh0.planes[i];
                var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix0, localPlaneVector);
                var length = math.length(worldPlaneVector);
                worldPlaneVector /= length;

                surfaceCategory0[i] = new SurfaceInfo()
                {
                    interiorCategory    = (CategoryGroupIndex)CategoryIndex.Inside,
                    basePlaneIndex      = i,
                    brush               = treeBrush1,
                    worldPlane          = worldPlaneVector,
                    layers              = mesh0.polygons[i].layerDefinition
                };
            }
            for (int i = 0; i < surfaceCategory1.Length; i++)
            {
                var localPlaneVector = mesh1.planes[i];
                var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix1, localPlaneVector);
                var length = math.length(worldPlaneVector);
                worldPlaneVector /= length;

                surfaceCategory1[i] = new SurfaceInfo()
                {
                    interiorCategory    = (CategoryGroupIndex)CategoryIndex.Inside,
                    basePlaneIndex      = i,
                    brush               = treeBrush0,
                    worldPlane          = worldPlaneVector,
                    layers              = mesh1.polygons[i].layerDefinition 
                };
            }

            if (intersectionType == IntersectionType.Intersection)
            {
                GetIntersectingPlanes(ref mesh1.planes, ref mesh0.vertices, mesh0.localBounds, inversedNode1ToNode0, intersectingPlaneIndices1);
                GetIntersectingPlanes(ref mesh0.planes, ref mesh1.vertices, mesh1.localBounds, inversedNode0ToNode1, intersectingPlaneIndices0);
            }


            // TODO: we don't actually use ALL of these planes .. Optimize this
            var localSpacePlanes0Length = mesh0.planes.Length;
            var localSpacePlanes0 = stackalloc float4[localSpacePlanes0Length];
            for (int p = 0; p < localSpacePlanes0Length; p++)
                localSpacePlanes0[p] = mesh0.planes[p];

            // TODO: we don't actually use ALL of these planes .. Optimize this
            var localSpacePlanes1Length = mesh1.planes.Length;
            var localSpacePlanes1 = stackalloc float4[localSpacePlanes1Length];
            for (int p = 0; p < localSpacePlanes1Length; p++)
            {
                var transformedPlane = math.mul(inversedNode1ToNode0, mesh1.planes[p]);
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

                intersectingPlaneIndices0.ResizeUninitialized(mesh0.planes.Length);
                intersectingPlaneIndices1.ResizeUninitialized(mesh1.planes.Length);

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

            FindPlanePairs(usedVertices1, usedPlanePairs1, intersectingPlaneIndices1, localSpacePlanes1, ref mesh1);
            FindPlanePairs(usedVertices0, usedPlanePairs0, intersectingPlaneIndices0, localSpacePlanes0, ref mesh0);

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


    [BurstCompile]
    unsafe struct FindIntersectionsJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<PlanePair> usedPlanePairs;
        [ReadOnly] public NativeList<float4>    intersectingPlanes0;
        [ReadOnly] public NativeList<float4>    intersectingPlanes1;
        [ReadOnly] public float4x4              nodeToTreeSpaceMatrix0;

        [WriteOnly] public NativeStream.Writer  foundVertices;
            
        public void Execute(int index)
        {
            foundVertices.BeginForEachIndex(index);
            var plane2 = intersectingPlanes0[index];

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

                    if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes1, localVertex) &&
                        !CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes0, localVertex))
                    {
                        var worldVertex = math.mul(nodeToTreeSpaceMatrix0, localVertex).xyz;
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

    [BurstCompile]
    unsafe struct SortLoopsJob : IJob
    {
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<PlaneVertexIndexPair>  foundIndices;
        [ReadOnly] public NativeArray<SurfaceInfo>          surfaceCategory; // TODO: only use plane information here
        [ReadOnly] public VertexSoup                        vertexSoup;

        public NativeList<int2> sortedStack;

        //[WriteOnly]
        public NativeList<ushort>                   uniqueIndices;
        //[WriteOnly]
        public NativeList<PlaneIndexOffsetLength>   planeIndexOffsets;


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
                var planeIndex = planeIndexOffset.planeIndex;
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