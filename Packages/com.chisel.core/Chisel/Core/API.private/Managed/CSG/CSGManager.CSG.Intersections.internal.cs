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
#if USE_MANAGED_CSG_IMPLEMENTATION
    static partial class CSGManagerPerformCSG
    {
        #region Helper methods

        #region TransformByTransposedInversedMatrix

        // TODO: Optimize
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [BurstCompile]
        static unsafe void TransformByTransposedInversedMatrix(float4* outputPlanes, float4* surfaces, int length, float4x4 nodeToTreeSpaceInversed)
        {
            for (int p = 0; p < length; p++)
            {
                var planeVector = math.mul(nodeToTreeSpaceInversed, surfaces[p]);
                outputPlanes[p] = planeVector / math.length(planeVector.xyz);
            }
        }

        #endregion

        #region SortIndices

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float3 FindPolygonCentroid(List<ushort> indices, NativeList<float3> vertices)
        {
            var centroid = float3.zero;
            for (int i = 0; i < indices.Count; i++)
                centroid += vertices[indices[i]];
            return centroid / indices.Count;
        }

        static List<int2> s_SortStack = new List<int2>();
        // TODO: sort by using plane information instead of unreliable floating point math ..
        // TODO: make this work on non-convex polygons
        static void SortIndices(List<ushort> indices, NativeList<float3> vertices, float3 normal)
        {
            // There's no point in trying to sort a point or a line 
            if (indices.Count < 3)
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

            var centroid = FindPolygonCentroid(indices, vertices);
            var center = new float2(math.dot(tangentX, centroid), // distance in direction of tangentX
                                    math.dot(tangentY, centroid)); // distance in direction of tangentY
#if true
            s_SortStack.Clear();
            s_SortStack.Add(new int2(0, indices.Count - 1));
            while (s_SortStack.Count > 0)
            {
                var top = s_SortStack[s_SortStack.Count - 1];
                s_SortStack.RemoveAt(s_SortStack.Count - 1);
                var l = top.x;
                var r = top.y;
                var left = l;
                var right = r;
                var va = (float3)vertices[indices[(left + right) / 2]];
                while (true)
                {
                    var a_angle = math.atan2(math.dot(tangentX, va) - center.x, math.dot(tangentY, va) - center.y);

                    {
                        var vb = (float3)vertices[indices[left]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (b_angle > a_angle)
                        {
                            left++;
                            vb = (float3)vertices[indices[left]];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    {
                        var vb = (float3)vertices[indices[right]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (a_angle > b_angle)
                        {
                            right--;
                            vb = (float3)vertices[indices[right]];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    if (left <= right)
                    {
                        if (left != right)
                        {
                            var t = indices[left];
                            indices[left] = indices[right];
                            indices[right] = t;
                        }

                        left++;
                        right--;
                    }
                    if (left > right)
                        break;
                }
                if (l < right)
                {
                    s_SortStack.Add(new int2(l, right));
                    //SortIndices(indices, vertices, tangentX, tangentY, center, l, right);
                }
                if (left < r)
                {
                    s_SortStack.Add(new int2(left, r));
                    //SortIndices(indices, vertices, tangentX, tangentY, center, left, r);
                }
            }
            //SortIndices(indices, vertices, tangentX, tangentY, center, 0, indices.Count - 1);
#else
            // sort vertices according to their angle relative to the centroid on plane defined by tangentX/tangentY
            vertices.Sort(delegate (Vector3 a, Vector3 b) 
            {
                var ax = math.dot(tangentX, a) - center.x;  // distance in direction of tangentX
                var ay = math.dot(tangentY, a) - center.y;  // distance in direction of tangentY
                var bx = math.dot(tangentX, b) - center.x;  // distance in direction of tangentX
                var by = math.dot(tangentY, b) - center.y;  // distance in direction of tangentY

                var a1 = math.atan2(ax, ay); // angle between ax/ay and cx/cy
                var a2 = math.atan2(bx, by); // angle between bx/by and cx/cy
                return (int)math.sign(a2 - a1);
            });
#endif
        }

        #endregion

        #region IsOutsidePlanes

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe bool IsOutsidePlanes(float4* planes, int length, float4 localVertex)
        {
            const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;
            for (int n = 0; n < length; n++)
            {
                var distance = math.dot(planes[n], localVertex);
                
                // will be 'false' when distance is NaN or Infinity
                if (!(distance <= kEpsilon))
                    return true;
            }
            return false;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe bool IsOutsidePlanes(NativeArray<float4> planes, float4 localVertex)
        {
            const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;
            for (int n = 0; n < planes.Length; n++)
            {
                var distance = math.dot(planes[n], localVertex);

                // will be 'false' when distance is NaN or Infinity
                if (!(distance <= kEpsilon))
                    return true;
            }
            return false;
        }
        #endregion
        
        #endregion
        
        #region FindLoopOverlapIntersections
        
        internal struct BrushData
        {
            public BlobAssetReference<BrushMeshBlob>    meshBlob;
            public float4x4                             treeToNodeSpaceMatrix;
            public float4x4                             nodeToTreeSpaceMatrix;
            public int2                                 planeSegment;

            public BrushLoops               outputLoops;
            public List<int>[]              intersectionSurfaces;
            public IntersectionLoop[]       basePolygonLoops;
            public static readonly BrushData Default = new BrushData()
            {
                meshBlob                = BlobAssetReference<BrushMeshBlob>.Null,
                treeToNodeSpaceMatrix   = float4x4.identity,
                nodeToTreeSpaceMatrix   = float4x4.identity
            };
        }
        
        internal unsafe class Temporaries : IDisposable
        {
            public NativeList<float4>               allWorldSpacePlanes;
            public NativeList<int2>                 brushPlaneSegments;

            public List<BrushData>                  brushDataList;
            public List<int>                        treeBrushes;
            public HashSet<BrushBrushIntersection>  intersectingBrushes = new HashSet<BrushBrushIntersection>();
            public List<IntersectionLoop>           allIntersectionLoops;
            public Temporaries(List<int> treeBrushes)
            {
                this.treeBrushes            = treeBrushes;
                this.allWorldSpacePlanes    = new NativeList<float4>(4, Allocator.Persistent);
                this.brushPlaneSegments     = new NativeList<int2>(treeBrushes.Count, Allocator.Persistent);
                
                this.brushDataList          = new List<BrushData>();
                this.allIntersectionLoops   = new List<IntersectionLoop>();
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static unsafe void TransformByTransposedInversedMatrix(float4* planes, int length, float4x4 nodeToTreeSpaceInversed)
            {
                for (int p = 0; p < length; p++)
                {
                    var planeVector = math.mul(nodeToTreeSpaceInversed, planes[p]);
                    planes[p] = planeVector / math.length(planeVector.xyz);
                }
            }

            public void Dispose()
            {
                if (allWorldSpacePlanes.IsCreated)
                    allWorldSpacePlanes.Dispose();

                if (brushPlaneSegments.IsCreated)
                    brushPlaneSegments.Dispose();

                if (this.allIntersectionLoops != null)
                {
                    foreach (var intersection in this.allIntersectionLoops)
                        intersection.indices.Dispose();
                    this.allIntersectionLoops.Clear();
                }
                brushDataList.Clear();
            }

            public void Init()
            {
                int totalPlanes = 0;

                intersectingBrushes.Clear();
                for (int b0 = 0; b0 < treeBrushes.Count; b0++)
                {
                    var brushInfo = CSGManager.GetBrushInfo(treeBrushes[b0]);
                    var intersections = brushInfo.brushBrushIntersections;
                    if (intersections.Count == 0)
                    {
                        brushDataList.Add(BrushData.Default);
                        continue;
                    }

                    // Find all intersections between brushes
                    foreach (var intersection in intersections)
                    {
                        Debug.Assert(intersection.brushNodeID0 != intersection.brushNodeID1);
                        var reverseOrder    = intersection.brushNodeID0 > intersection.brushNodeID1; // ensures we do calculations exactly the same for each brush pair
                        var type            = intersection.type;

                        CSGTreeBrush brush0, brush1;
                        if (reverseOrder)
                        {
                            if (type == IntersectionType.AInsideB) type = IntersectionType.BInsideA;
                            else if (type == IntersectionType.BInsideA) type = IntersectionType.AInsideB;
                            brush0.brushNodeID = intersection.brushNodeID1;
                            brush1.brushNodeID = intersection.brushNodeID0;
                        } else
                        {
                            brush0.brushNodeID = intersection.brushNodeID0;
                            brush1.brushNodeID = intersection.brushNodeID1;
                        }

                        intersectingBrushes.Add(intersection); // uses hashset to ensure this is unique
                    }
                    
                    var intersectingBrush   = new CSGTreeBrush() { brushNodeID = treeBrushes[b0] };
                    var newBrushData = new BrushData()
                    {
                        meshBlob                = BrushMeshManager.GetBrushMeshBlob(intersectingBrush.BrushMesh.BrushMeshID),
                        treeToNodeSpaceMatrix   = intersectingBrush.TreeToNodeSpaceMatrix,
                        nodeToTreeSpaceMatrix   = intersectingBrush.NodeToTreeSpaceMatrix,

                        outputLoops             = brushInfo.brushOutputLoops
                    };
                    totalPlanes += newBrushData.meshBlob.Value.planes.Length;
                    brushDataList.Add(newBrushData);
                }

                allWorldSpacePlanes.Capacity = totalPlanes;
            }

            public void Execute()
            {
                var indexLookup = new Dictionary<int, int>();
                for (int b0 = 0; b0 < treeBrushes.Count; b0++)
                {
                    indexLookup[treeBrushes[b0]] = b0;
                    var newBrushData = brushDataList[b0];
                    if (newBrushData.meshBlob == BlobAssetReference<BrushMeshBlob>.Null)
                        continue;

                    var treeToNodeSpaceTransposed = math.transpose(newBrushData.treeToNodeSpaceMatrix);
                    var meshPlanes          = (float4*)newBrushData.meshBlob.Value.planes.GetUnsafePtr();
                    var startIndex          = allWorldSpacePlanes.Length;
                    allWorldSpacePlanes.AddRange(meshPlanes, newBrushData.meshBlob.Value.planes.Length);
                    var segment             = new int2(startIndex, allWorldSpacePlanes.Length - startIndex);
                    brushPlaneSegments.Add(segment);
                    var worldSpacePlanesPtr = ((float4*)allWorldSpacePlanes.GetUnsafePtr()) + segment.x;
                    TransformByTransposedInversedMatrix(worldSpacePlanesPtr, segment.y, treeToNodeSpaceTransposed);
                    newBrushData.planeSegment = segment;
                }

                for (int b0 = 0; b0 < treeBrushes.Count; b0++)
                {
                    var newBrushData = brushDataList[b0];
                    if (newBrushData.meshBlob == BlobAssetReference<BrushMeshBlob>.Null)
                        continue;

                    var basePolygons = newBrushData.outputLoops.basePolygons;

                    newBrushData.basePolygonLoops       = new IntersectionLoop[basePolygons.Count];
                    newBrushData.intersectionSurfaces   = new List<int>[newBrushData.meshBlob.Value.planes.Length];
                    for (int i = 0; i < newBrushData.intersectionSurfaces.Length; i++)
                        newBrushData.intersectionSurfaces[i] = new List<int>(16);

                    for (int s = 0; s < newBrushData.basePolygonLoops.Length; s++)
                    {
                        var loop = basePolygons[s];
                        var intersectionLoop = new IntersectionLoop()
                        {
                            segment         = newBrushData.planeSegment,
                            indices         = new NativeList<ushort>(loop.indices.Count, Allocator.Persistent),
                            surfaceIndex    = s,
                            brushNodeID     = treeBrushes[b0]
                        };

                        for (int i = 0; i < loop.indices.Count; i++)
                            intersectionLoop.indices.Add(loop.indices[i]);

                        this.allIntersectionLoops.Add(intersectionLoop);
                        newBrushData.basePolygonLoops[s] = intersectionLoop;
                    }
                }
            }
        }

        internal unsafe static void FindAllIntersectionLoops(List<int> treeBrushes)
        {
            // TODO: - calculate all data per brush here (currently unused)
            //         and use this later on, hopefully making it possible to remove all intermediates
            //       - also get rid of all SurfaceLoops etc. in the middle, only store at the end
            //       - finally instead of storing stuff in SurfaceLoops at the end, store as blob
            using (var temporaries = new Temporaries(treeBrushes))
            {
                temporaries.Init();
                temporaries.Execute();

                // Create unique loops between brush intersections
                UnityEngine.Profiling.Profiler.BeginSample("CreateIntersectionLoops");
                try
                {
                    foreach (var intersection in temporaries.intersectingBrushes)
                    {
                        var type = intersection.type;
                        var brush0 = new CSGTreeBrush() { brushNodeID = intersection.brushNodeID0 };
                        var brush1 = new CSGTreeBrush() { brushNodeID = intersection.brushNodeID1 };
                        var blobMesh0 = BrushMeshManager.GetBrushMeshBlob(brush0.BrushMesh.BrushMeshID);
                        var blobMesh1 = BrushMeshManager.GetBrushMeshBlob(brush1.BrushMesh.BrushMeshID);
                        if (!blobMesh0.IsCreated || !blobMesh1.IsCreated) continue;
                        if (blobMesh0.Value.IsEmpty() || blobMesh1.Value.IsEmpty()) continue;

                        var brushInfo0 = CSGManager.GetBrushInfo(brush0.brushNodeID);
                        var brushInfo1 = CSGManager.GetBrushInfo(brush1.brushNodeID);
                        // TODO: get all the above from "temporaries"

                        // TODO: put this in a burstable data structure
                        var loops01 = new SurfaceLoops(blobMesh0.Value.planes.Length);
                        var loops10 = new SurfaceLoops(blobMesh1.Value.planes.Length);


                        using (var sharedPlaneData = new SharedPlaneData(brushInfo0, brushInfo1, brush0, blobMesh0, brush1, blobMesh1, type, Allocator.TempJob))
                        {
                            sharedPlaneData.Run();
                            if (sharedPlaneData.intersectingPlanes0.Length == 0 && sharedPlaneData.intersectingPlanes1.Length == 0)
                                continue;

                            // chain each subsection together, then chain subsections together
                            var handleDep = new JobHandle();

                            // TODO: allocate per intersection, perform all calculations/sorts, THEN create ALL surface-loops and assign indices
                            using (var insideVerticesStream0    = new NativeStream(math.max(1, sharedPlaneData.usedVertices0.Length), Allocator.TempJob))
                            using (var insideVerticesStream1    = new NativeStream(math.max(1, sharedPlaneData.usedVertices1.Length), Allocator.TempJob))
                            using (var intersectionStream0      = new NativeStream(sharedPlaneData.intersectingPlaneIndices0.Length, Allocator.TempJob))
                            using (var intersectionStream1      = new NativeStream(sharedPlaneData.intersectingPlaneIndices1.Length, Allocator.TempJob))
                            using (var foundIndices0            = new NativeList<PlaneVertexIndexPair>(sharedPlaneData.intersectingPlanes0.Length, Allocator.TempJob))
                            using (var foundIndices1            = new NativeList<PlaneVertexIndexPair>(sharedPlaneData.intersectingPlanes1.Length, Allocator.TempJob))
                            using (var planeIndexOffsets0       = new NativeList<PlaneIndexOffsetLength>(sharedPlaneData.surfaceCategory0.Length, Allocator.TempJob))
                            using (var planeIndexOffsets1       = new NativeList<PlaneIndexOffsetLength>(sharedPlaneData.surfaceCategory1.Length, Allocator.TempJob))
                            using (var uniqueIndices0           = new NativeList<ushort>(sharedPlaneData.intersectingPlaneIndices0.Length, Allocator.TempJob))
                            using (var uniqueIndices1           = new NativeList<ushort>(sharedPlaneData.intersectingPlaneIndices1.Length, Allocator.TempJob))
                            {
                                // First find vertices from other brush that are inside the other brush, so that any vertex we 
                                // find during the intersection part will be snapped to those vertices and not the other way around

                                // TODO: when all vertices of a polygon are inside the other brush, don't bother intersecting it.
                                //       same when two planes overlap each other ...

                                // Find all vertices of brush1 that are inside brush2, and put their intersections into the appropriate loops
                                if (sharedPlaneData.usedVertices0.Length > 0)
                                {
                                    var findInsideVerticesJob = new FindInsideVerticesJob
                                    {
                                        intersectingPlanes1         = sharedPlaneData.intersectingPlanes1,
                                        usedVertices0               = sharedPlaneData.usedVertices0,
                                        allVertices0                = (float3*)sharedPlaneData.blobMesh0.Value.vertices.GetUnsafePtr(),
                                        nodeToTreeSpaceMatrix       = sharedPlaneData.nodeToTreeSpaceMatrix0,
                                        vertexToLocal0              = float4x4.identity,
                                        vertexWriter                = insideVerticesStream0.AsWriter()
                                    };
                                    findInsideVerticesJob.Run(sharedPlaneData.usedVertices0.Length);
                                    var insertInsideVerticesJob = new InsertInsideVerticesJob
                                    {
                                        vertexReader                = insideVerticesStream0.AsReader(),
                                        intersectingPlanes          = sharedPlaneData.intersectingPlanes0,
                                        intersectingPlaneIndices    = sharedPlaneData.intersectingPlaneIndices0,
                                        brushVertices               = sharedPlaneData.vertexSoup0,
                                        outputIndices               = foundIndices0
                                    };
                                    insertInsideVerticesJob.Run();
                                }
                                if (sharedPlaneData.usedVertices1.Length > 0)
                                {
                                    var findInsideVerticesJob = new FindInsideVerticesJob
                                    {
                                        intersectingPlanes1         = sharedPlaneData.intersectingPlanes0,
                                        usedVertices0               = sharedPlaneData.usedVertices1,
                                        allVertices0                = (float3*)sharedPlaneData.blobMesh1.Value.vertices.GetUnsafePtr(),
                                        nodeToTreeSpaceMatrix       = sharedPlaneData.nodeToTreeSpaceMatrix1,
                                        vertexToLocal0              = sharedPlaneData.node1ToNode0,
                                        vertexWriter                = insideVerticesStream1.AsWriter()
                                    };
                                    findInsideVerticesJob.Run(sharedPlaneData.usedVertices1.Length);
                                    var insertInsideVerticesJob = new InsertInsideVerticesJob
                                    {
                                        vertexReader                = insideVerticesStream1.AsReader(),
                                        intersectingPlanes          = sharedPlaneData.intersectingPlanes1,
                                        intersectingPlaneIndices    = sharedPlaneData.intersectingPlaneIndices1,
                                        brushVertices               = sharedPlaneData.vertexSoup1,
                                        outputIndices               = foundIndices1
                                    };
                                    insertInsideVerticesJob.Run();
                                }

                                // Now find all the intersection vertices
                                if (sharedPlaneData.intersectionType == IntersectionType.Intersection)
                                {
                                    if (sharedPlaneData.usedPlanePairs1.Length > 0)
                                    {
                                        // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                                        var findIntersectionsJob = new FindIntersectionsJob
                                        {
                                            usedPlanePairs          = sharedPlaneData.usedPlanePairs1,
                                            intersectingPlanes0     = sharedPlaneData.intersectingPlanes0, 
                                            intersectingPlanes1     = sharedPlaneData.intersectingPlanes1,
                                            nodeToTreeSpaceMatrix0  = sharedPlaneData.nodeToTreeSpaceMatrix0,
                                            foundVertices           = intersectionStream0.AsWriter()
                                        };
                                        findIntersectionsJob.Run(sharedPlaneData.intersectingPlanes0.Length);
                                        var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                                        {
                                            vertexReader                = intersectionStream0.AsReader(),
                                            intersectingPlaneIndices0   = sharedPlaneData.intersectingPlaneIndices0,
                                            brushVertices0              = sharedPlaneData.vertexSoup0,
                                            brushVertices1              = sharedPlaneData.vertexSoup1,
                                            outputIndices0              = foundIndices0,
                                            outputIndices1              = foundIndices1
                                        };
                                        insertIntersectionVerticesJob.Run();
                                    }

                                    if (sharedPlaneData.usedPlanePairs0.Length > 0)
                                    {
                                        // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                                        var findIntersectionsJob = new FindIntersectionsJob
                                        {
                                            usedPlanePairs          = sharedPlaneData.usedPlanePairs0,
                                            intersectingPlanes0     = sharedPlaneData.intersectingPlanes1, 
                                            intersectingPlanes1     = sharedPlaneData.intersectingPlanes0,
                                            nodeToTreeSpaceMatrix0  = sharedPlaneData.nodeToTreeSpaceMatrix0,
                                            foundVertices           = intersectionStream1.AsWriter()
                                        };
                                        findIntersectionsJob.Run(sharedPlaneData.intersectingPlanes1.Length);
                                        var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                                        {
                                            vertexReader                = intersectionStream1.AsReader(),
                                            intersectingPlaneIndices0   = sharedPlaneData.intersectingPlaneIndices1,
                                            brushVertices0              = sharedPlaneData.vertexSoup1,
                                            brushVertices1              = sharedPlaneData.vertexSoup0,
                                            outputIndices0              = foundIndices1,
                                            outputIndices1              = foundIndices0
                                        };
                                        insertIntersectionVerticesJob.Run();
                                    }
                                }


                                {
                                    var sortLoopsJob0 = new SortLoopsJob
                                    {
                                        surfaceCategory     = sharedPlaneData.surfaceCategory0,
                                        vertexSoup          = sharedPlaneData.vertexSoup0,
                                        foundIndices        = foundIndices0,
                                        uniqueIndices       = uniqueIndices0,
                                        planeIndexOffsets   = planeIndexOffsets0
                                    };
                                    sortLoopsJob0.Run();

                                    // TODO: make this burstable
                                    var createLoopsJob0 = new CreateLoopsJob
                                    {
                                        uniqueIndices       = uniqueIndices0,
                                        planeIndexOffsets   = planeIndexOffsets0,
                                        surfaceCategories   = sharedPlaneData.surfaceCategory0,
                                        outputSurfaces      = loops01.surfaces
                                    };
                                    createLoopsJob0.Execute();
                                }
                                
                                {
                                    var sortLoopsJob1 = new SortLoopsJob
                                    {
                                        surfaceCategory     = sharedPlaneData.surfaceCategory1,
                                        vertexSoup          = sharedPlaneData.vertexSoup1,
                                        foundIndices        = foundIndices1,
                                        uniqueIndices       = uniqueIndices1,
                                        planeIndexOffsets   = planeIndexOffsets1
                                    };
                                    sortLoopsJob1.Run();

                                    // TODO: make this burstable
                                    var createLoopsJob1 = new CreateLoopsJob
                                    {
                                        uniqueIndices       = uniqueIndices1,
                                        planeIndexOffsets   = planeIndexOffsets1,
                                        surfaceCategories   = sharedPlaneData.surfaceCategory1,
                                        outputSurfaces      = loops10.surfaces
                                    };
                                    createLoopsJob1.Execute();
                                }
                            }
                        }

                        // TODO: this can probably be done using a list instead of a dictionary
                        brushInfo0.brushOutputLoops.intersectionSurfaceLoops[brush1.brushNodeID] = loops01;
                        brushInfo1.brushOutputLoops.intersectionSurfaceLoops[brush0.brushNodeID] = loops10;
                    }
                } finally { UnityEngine.Profiling.Profiler.EndSample(); }

                UnityEngine.Profiling.Profiler.BeginSample("FindLoopOverlapIntersections");
                try
                {
                    for (int b0 = 0; b0 < treeBrushes.Count; b0++)
                    {
                        var brush0NodeID    = treeBrushes[b0];
                        var brush0          = new CSGTreeBrush() { brushNodeID = brush0NodeID };
                        var outputLoops     = CSGManager.GetBrushInfo(brush0NodeID).brushOutputLoops;

                        if (outputLoops.intersectionSurfaceLoops.Count == 0)
                            continue;

                        var meshBlob = BrushMeshManager.GetBrushMeshBlob(brush0.BrushMesh.BrushMeshID);
                        Debug.Assert(meshBlob != null);

                        if (meshBlob.Value.planes.Length == 0)
                            continue;

                        var vertexSoup = outputLoops.vertexSoup;

                        using (var intersectionData = new OverlapIntersectionData(brush0, meshBlob, outputLoops.intersectionSurfaceLoops, outputLoops.basePolygons))
                        {
                            intersectionData.Execute();

                            UnityEngine.Profiling.Profiler.BeginSample("Find intersection-loop/intersection-loop intersections");
                            try
                            {
                                for (int s = 0; s < intersectionData.intersectionSurfaces.Length; s++)
                                {
                                    var intersectionSurfaceList = intersectionData.intersectionSurfaces[s];
                                    for (int l0 = intersectionSurfaceList.Count - 1; l0 >= 0; l0--)
                                    {
                                        var intersectionSurface0 = intersectionData.allIntersectionLoops[intersectionSurfaceList[l0]];
                                        var indices              = intersectionSurface0.indices;

                                        for (int l1 = 0; l1 < intersectionSurfaceList.Count; l1++)
                                        {
                                            if (l0 == l1)
                                                continue;

                                            var intersectionJob = new FindLoopPlaneIntersectionsJob()
                                            {
                                                allWorldSpacePlanes = intersectionData.allWorldSpacePlanes,
                                                otherPlanesSegment  = intersectionData.allIntersectionLoops[intersectionSurfaceList[l1]].segment,
                                                selfPlanesSegment   = intersectionSurface0.segment,
                                                vertexSoup          = vertexSoup,
                                                indices             = indices
                                            };
                                            intersectionJob.Run();

                                            // TODO: merge these so that intersections will be identical on both loops (without using math, use logic)
                                            // TODO: make sure that intersections between loops will be identical on OTHER brushes (without using math, use logic)
                                        }
                                    }
                                }
                            }
                            finally { UnityEngine.Profiling.Profiler.EndSample(); }


                            UnityEngine.Profiling.Profiler.BeginSample("Find base-loop/intersection-loop intersections");
                            try
                            {
                                // TODO: should only intersect with all brushes that each particular basepolygon intersects with
                                //       but also need adjency information between basePolygons to ensure that intersections exist on 
                                //       both sides of each edge on a brush. 
                                for (int b = 0; b < intersectionData.basePolygonLoops.Length; b++)
                                {
                                    var indices = intersectionData.basePolygonLoops[b].indices;
                                    foreach (var brushPlaneSegment in intersectionData.brushPlaneSegments)
                                    {
                                        var intersectionJob = new FindLoopPlaneIntersectionsJob()
                                        {
                                            allWorldSpacePlanes = intersectionData.allWorldSpacePlanes,
                                            otherPlanesSegment  = brushPlaneSegment,
                                            selfPlanesSegment   = intersectionData.worldSpacePlanes0Segment,
                                            vertexSoup          = vertexSoup,
                                            indices             = indices
                                        };
                                        intersectionJob.Run();
                                    }
                                }
                            }
                            finally { UnityEngine.Profiling.Profiler.EndSample(); }

                            UnityEngine.Profiling.Profiler.BeginSample("Find intersection-loop/base-loop intersections");
                            try
                            {
                                for (int s = 0; s < intersectionData.intersectionSurfaces.Length; s++)
                                {
                                    var intersectionSurfaceList = intersectionData.intersectionSurfaces[s];
                                    if (intersectionSurfaceList.Count == 0)
                                        continue;
                                
                                    var basePolygonIndices = intersectionData.basePolygonLoops[s].indices;
                                    for (int l0 = 0; l0 < intersectionSurfaceList.Count; l0++)
                                    {
                                        var intersectionSurface = intersectionData.allIntersectionLoops[intersectionSurfaceList[l0]];
                                        var intersectionJob2 = new FindLoopVertexOverlapsJob
                                        {
                                            selfPlanes      = intersectionData.GetPlanes(intersectionSurface.segment),
                                            otherIndices    = basePolygonIndices,
                                            vertexSoup      = vertexSoup,
                                            indices         = intersectionSurface.indices
                                        };
                                        intersectionJob2.Run();
                                    }
                                }
                            }
                            finally { UnityEngine.Profiling.Profiler.EndSample(); }

                            UnityEngine.Profiling.Profiler.BeginSample("Cleanup");
                            try
                            {
                                for (int i = intersectionData.allIntersectionLoops.Count - 1; i >= 0; i--)
                                {
                                    var indices = intersectionData.allIntersectionLoops[i].indices;
                                    var removeIdenticalIndicesJob = new RemoveIdenticalIndicesJob { indices = indices };
                                    removeIdenticalIndicesJob.Run();
                                }

                                for (int i = intersectionData.basePolygonLoops.Length - 1; i >= 0; i--)
                                {
                                    var indices = intersectionData.basePolygonLoops[i].indices;
                                    var removeIdenticalIndicesJob = new RemoveIdenticalIndicesJob { indices = indices };
                                    removeIdenticalIndicesJob.Run();
                                }

                                // TODO: eventually merge indices across multiple loops when vertices are identical

                                intersectionData.StoreOutput(outputLoops.intersectionSurfaceLoops, outputLoops.intersectionLoops, outputLoops.basePolygons);
                            }
                            finally { UnityEngine.Profiling.Profiler.EndSample(); }
                        }
                    }
                }
                finally { UnityEngine.Profiling.Profiler.EndSample(); }
            }
        }        
        #endregion
    }
#endif
}
