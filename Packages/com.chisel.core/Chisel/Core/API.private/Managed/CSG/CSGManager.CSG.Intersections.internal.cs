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

        static readonly HashSet<BrushBrushIntersection> s_IntersectingBrushes = new HashSet<BrushBrushIntersection>();
        

        internal unsafe static void FindAllIntersectionLoops(List<int> treeBrushes)
        {
            // Find all intersections between brushes
            s_IntersectingBrushes.Clear();
            for (int b0 = 0; b0 < treeBrushes.Count; b0++)
            {
                var brushInfo = CSGManager.GetBrushInfo(treeBrushes[b0]);
                var intersections = brushInfo.brushBrushIntersections;
                if (intersections.Count == 0)
                    continue;

                foreach (var intersection in intersections)
                {
                    s_IntersectingBrushes.Add(intersection); // uses hashset to ensure this is unique
                }
            }

            // Create unique loops between brush intersections
            UnityEngine.Profiling.Profiler.BeginSample("CreateIntersectionLoops");
            try
            {
                foreach (var intersection in s_IntersectingBrushes)
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
                    }
                    else
                    {
                        brush0.brushNodeID = intersection.brushNodeID0;
                        brush1.brushNodeID = intersection.brushNodeID1;
                    }

                    var blobMesh0 = BrushMeshManager.GetBrushMeshBlob(brush0.BrushMesh.BrushMeshID);
                    var blobMesh1 = BrushMeshManager.GetBrushMeshBlob(brush1.BrushMesh.BrushMeshID);
                    if (!blobMesh0.IsCreated || !blobMesh1.IsCreated) { Debug.Log("mesh1 == null || mesh2 == null"); continue; }
                    if (blobMesh0.Value.IsEmpty() || blobMesh1.Value.IsEmpty()) continue;

                    var brushInfo0 = CSGManager.GetBrushInfo(brush0.brushNodeID);
                    var brushInfo1 = CSGManager.GetBrushInfo(brush1.brushNodeID);

                    var loops01 = new SurfaceLoops(blobMesh0.Value.planes.Length);
                    var loops10 = new SurfaceLoops(blobMesh1.Value.planes.Length);

                    // TODO: this can probably be done using a list instead of a dictionary
                    brushInfo0.brushOutputLoops.intersectionSurfaceLoops[brush1.brushNodeID] = loops01;
                    brushInfo1.brushOutputLoops.intersectionSurfaceLoops[brush0.brushNodeID] = loops10;

                    // TODO: put BrushMesh into a blobasset for efficiency, put a lot more precalculated stuff in there as well

                    using (var sharedPlaneData = new SharedPlaneData(brushInfo0, brushInfo1, brush0, blobMesh0, brush1, blobMesh1, type, Allocator.TempJob))
                    {
                        sharedPlaneData.Run();
                        if (sharedPlaneData.intersectingPlanes0.Length == 0 && sharedPlaneData.intersectingPlanes1.Length == 0)
                            continue;

                        var handleDep = new JobHandle();

                        using (var foundIndices0 = new NativeList<PlaneVertexIndexPair>(sharedPlaneData.intersectingPlanes0.Length, Allocator.TempJob))
                        using (var foundIndices1 = new NativeList<PlaneVertexIndexPair>(sharedPlaneData.intersectingPlanes1.Length, Allocator.TempJob))
                        {
                            // First find vertices from other brush that are inside the other brush, so that any vertex we 
                            // find during the intersection part will be snapped to those vertices and not the other way around

                            // TODO: when all vertices of a polygon are inside the other brush, don't bother intersecting it.
                            //       same when two planes overlap each other ...

                            // Find all vertices of brush1 that are inside brush2, and put their intersections into the appropriate loops
                            if (sharedPlaneData.usedVertices0.Length > 0)
                            {
                                using (var foundVertices0 = new NativeStream(math.max(1, sharedPlaneData.usedVertices0.Length), Allocator.TempJob))
                                {
                                    var findInsideVerticesJob = new FindInsideVerticesJob
                                    {
                                        intersectingPlanes1         = sharedPlaneData.intersectingPlanes1,
                                        usedVertices0               = sharedPlaneData.usedVertices0,
                                        allVertices0                = (float3*)sharedPlaneData.blobMesh0.Value.vertices.GetUnsafePtr(),
                                        nodeToTreeSpaceMatrix       = sharedPlaneData.nodeToTreeSpaceMatrix0,
                                        vertexToLocal0              = float4x4.identity,
                                        vertexWriter                = foundVertices0.AsWriter()
                                    };
                                    findInsideVerticesJob.Run(sharedPlaneData.usedVertices0.Length);
                                    var insertInsideVerticesJob = new InsertInsideVerticesJob
                                    {
                                        vertexReader                = foundVertices0.AsReader(),
                                        intersectingPlanes          = sharedPlaneData.intersectingPlanes0,
                                        intersectingPlaneIndices    = sharedPlaneData.intersectingPlaneIndices0,
                                        brushVertices               = sharedPlaneData.vertexSoup0,
                                        outputIndices               = foundIndices0
                                    };
                                    insertInsideVerticesJob.Run();
                                }
                            }
                            if (sharedPlaneData.usedVertices1.Length > 0)
                            {
                                using (var foundVertices1 = new NativeStream(math.max(1, sharedPlaneData.usedVertices1.Length), Allocator.TempJob))
                                {
                                    var findInsideVerticesJob = new FindInsideVerticesJob
                                    {
                                        intersectingPlanes1         = sharedPlaneData.intersectingPlanes0,
                                        usedVertices0               = sharedPlaneData.usedVertices1,
                                        allVertices0                = (float3*)sharedPlaneData.blobMesh1.Value.vertices.GetUnsafePtr(),
                                        nodeToTreeSpaceMatrix       = sharedPlaneData.nodeToTreeSpaceMatrix1,
                                        vertexToLocal0              = sharedPlaneData.node1ToNode0,
                                        vertexWriter                = foundVertices1.AsWriter()
                                    };
                                    findInsideVerticesJob.Run(sharedPlaneData.usedVertices1.Length);
                                    var insertInsideVerticesJob = new InsertInsideVerticesJob
                                    {
                                        vertexReader                = foundVertices1.AsReader(),
                                        intersectingPlanes          = sharedPlaneData.intersectingPlanes1,
                                        intersectingPlaneIndices    = sharedPlaneData.intersectingPlaneIndices1,
                                        brushVertices               = sharedPlaneData.vertexSoup1,
                                        outputIndices               = foundIndices1
                                    };
                                    insertInsideVerticesJob.Run();
                                }
                            }

                            // Now find all the intersection vertices
                            if (sharedPlaneData.intersectionType == IntersectionType.Intersection)
                            {
                                if (sharedPlaneData.usedPlanePairs1.Length > 0)
                                {
                                    using (var foundVertices = new NativeStream(sharedPlaneData.intersectingPlaneIndices0.Length, Allocator.TempJob))
                                    {
                                        // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                                        var findIntersectionsJob = new FindIntersectionsJob
                                        {
                                            usedPlanePairs          = sharedPlaneData.usedPlanePairs1,
                                            intersectingPlanes0     = sharedPlaneData.intersectingPlanes0, 
                                            intersectingPlanes1     = sharedPlaneData.intersectingPlanes1,
                                            nodeToTreeSpaceMatrix0  = sharedPlaneData.nodeToTreeSpaceMatrix0,
                                            foundVertices           = foundVertices.AsWriter()
                                        };
                                        findIntersectionsJob.Run(sharedPlaneData.intersectingPlanes0.Length);
                                        var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                                        {
                                            vertexReader                = foundVertices.AsReader(),
                                            intersectingPlaneIndices0   = sharedPlaneData.intersectingPlaneIndices0,
                                            brushVertices0              = sharedPlaneData.vertexSoup0,
                                            brushVertices1              = sharedPlaneData.vertexSoup1,
                                            outputIndices0              = foundIndices0,
                                            outputIndices1              = foundIndices1
                                        };
                                        insertIntersectionVerticesJob.Run();
                                    }
                                }

                                if (sharedPlaneData.usedPlanePairs0.Length > 0)
                                {
                                    using (var foundVertices = new NativeStream(sharedPlaneData.intersectingPlaneIndices1.Length, Allocator.TempJob))
                                    {
                                        // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                                        var findIntersectionsJob = new FindIntersectionsJob
                                        {
                                            usedPlanePairs          = sharedPlaneData.usedPlanePairs0,
                                            intersectingPlanes0     = sharedPlaneData.intersectingPlanes1, 
                                            intersectingPlanes1     = sharedPlaneData.intersectingPlanes0,
                                            nodeToTreeSpaceMatrix0  = sharedPlaneData.nodeToTreeSpaceMatrix0,
                                            foundVertices           = foundVertices.AsWriter()
                                        };
                                        findIntersectionsJob.Run(sharedPlaneData.intersectingPlanes1.Length);
                                        var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                                        {
                                            vertexReader                = foundVertices.AsReader(),
                                            intersectingPlaneIndices0   = sharedPlaneData.intersectingPlaneIndices1,
                                            brushVertices0              = sharedPlaneData.vertexSoup1,
                                            brushVertices1              = sharedPlaneData.vertexSoup0,
                                            outputIndices0              = foundIndices1,
                                            outputIndices1              = foundIndices0
                                        };
                                        insertIntersectionVerticesJob.Run();
                                    }
                                }
                            }


                            if (foundIndices0.Length > 0)
                            {
                                using (var uniqueIndices = new NativeList<ushort>(foundIndices0.Length, Allocator.TempJob))
                                using (var planeIndexOffsets = new NativeList<PlaneIndexOffsetLength>(sharedPlaneData.surfaceCategory0.Length, Allocator.TempJob))
                                using (var sortedStack = new NativeList<int2>(32, Allocator.TempJob))
                                {
                                    NativeSortExtension.SortJob(foundIndices0.AsArray()).Complete();
                                    var sortLoopsJob0 = new SortLoopsJob
                                    {
                                        foundIndices        = foundIndices0,
                                        uniqueIndices       = uniqueIndices,
                                        surfaceCategory     = sharedPlaneData.surfaceCategory0,
                                        sortedStack         = sortedStack,
                                        planeIndexOffsets   = planeIndexOffsets,
                                        vertexSoup          = sharedPlaneData.vertexSoup0
                                    };
                                    sortLoopsJob0.Run();

                                    // TODO: make this burstable
                                    var createLoopsJob0 = new CreateLoopsJob
                                    {
                                        uniqueIndices       = uniqueIndices,
                                        planeIndexOffsets   = planeIndexOffsets,
                                        surfaceCategories   = sharedPlaneData.surfaceCategory0,
                                        outputSurfaces      = loops01.surfaces
                                    };
                                    createLoopsJob0.Execute();
                                }
                            }

                            if (foundIndices1.Length > 0)
                            {
                                NativeSortExtension.SortJob(foundIndices1.AsArray()).Complete();
                                using (var uniqueIndices = new NativeList<ushort>(foundIndices1.Length, Allocator.TempJob))
                                using (var planeIndexOffsets = new NativeList<PlaneIndexOffsetLength>(sharedPlaneData.surfaceCategory1.Length, Allocator.TempJob))
                                using (var sortedStack = new NativeList<int2>(32, Allocator.TempJob))
                                {
                                    var sortLoopsJob1 = new SortLoopsJob
                                    {
                                        foundIndices        = foundIndices1,
                                        uniqueIndices       = uniqueIndices,
                                        surfaceCategory     = sharedPlaneData.surfaceCategory1,
                                        sortedStack         = sortedStack,
                                        planeIndexOffsets   = planeIndexOffsets,
                                        vertexSoup          = sharedPlaneData.vertexSoup1
                                    };
                                    sortLoopsJob1.Run();

                                    // TODO: make this burstable
                                    var createLoopsJob1 = new CreateLoopsJob
                                    {
                                        uniqueIndices       = uniqueIndices,
                                        planeIndexOffsets   = planeIndexOffsets,
                                        surfaceCategories   = sharedPlaneData.surfaceCategory1,
                                        outputSurfaces      = loops10.surfaces
                                    };
                                    createLoopsJob1.Execute();
                                }
                            }
                        }
                    }
                }
            } finally { UnityEngine.Profiling.Profiler.EndSample(); }

            UnityEngine.Profiling.Profiler.BeginSample("FindLoopOverlapIntersections");
            try
            {
                for (int b0 = 0; b0 < treeBrushes.Count; b0++)
                {
                    var brush0NodeID    = treeBrushes[b0];
                    var outputLoops     = CSGManager.GetBrushInfo(brush0NodeID).brushOutputLoops;

                    if (outputLoops.intersectionSurfaceLoops.Count == 0)
                        continue;

                    var meshBlob = BrushMeshManager.GetBrushMeshBlob(outputLoops.brush.BrushMesh.BrushMeshID);
                    Debug.Assert(meshBlob != null);

                    if (meshBlob.Value.planes.Length == 0)
                        continue;

                    using (var intersectionData = new OverlapIntersectionData(outputLoops.brush, meshBlob, outputLoops.intersectionSurfaceLoops, outputLoops.basePolygons))
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
                                            vertexSoup          = outputLoops.vertexSoup,
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
                                        vertexSoup          = outputLoops.vertexSoup,
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
                                        vertexSoup      = outputLoops.vertexSoup,
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
                            // TODO: handle this more carefully to avoid gaps
                            for (int i = intersectionData.allIntersectionLoops.Count - 1; i >= 0; i--)
                            {
                                var indices = intersectionData.allIntersectionLoops[i].indices;

                                var removeIdenticalIndicesJob = new RemoveIdenticalIndicesJob
                                {
                                    indices = indices
                                };
                                // TODO: eventually actually use jobs
                                removeIdenticalIndicesJob.Execute();
                            }

                            for (int i = intersectionData.basePolygonLoops.Length - 1; i >= 0; i--)
                            {
                                var indices = intersectionData.basePolygonLoops[i].indices;

                                var removeIdenticalIndicesJob = new RemoveIdenticalIndicesJob
                                {
                                    indices = indices
                                };
                                // TODO: eventually actually use jobs
                                removeIdenticalIndicesJob.Execute();
                            }

                            intersectionData.StoreOutput();

                            // TODO: eventually merge indices across multiple loops when vertices are identical


                            //outputLoops.vertexSoup.vertexArray.Clear();
                            //for (int i = 0; i < outputLoops.vertexSoup.vertexArray.Count; i++)
                            //    outputLoops.vertexSoup.vertexArray.Add(outputLoops.vertexSoup.vertices[i]);

                            for (int i = 0; i < outputLoops.basePolygons.Count; i++)
                            {
                                var basePolygon = outputLoops.basePolygons[i];
                                basePolygon.edges.Clear();
                                basePolygon.AddEdges(basePolygon.indices);
                            }

                            foreach (var pair in outputLoops.intersectionSurfaceLoops)
                            {
                                var surfaceLoops = pair.Value.surfaces;
                                if (surfaceLoops == null)
                                    continue;
                                for (int i = 0; i < surfaceLoops.Length; i++)
                                {
                                    var loops = surfaceLoops[i];
                                    for (int s = 0; s < loops.Count; s++)
                                    {
                                        var intersectionLoop = loops[s];
                                        intersectionLoop.edges.Clear();
                                        intersectionLoop.AddEdges(intersectionLoop.indices);
                                    }
                                }
                            }


                            foreach (var pair in outputLoops.intersectionSurfaceLoops)
                            {
                                var surfaceLoops = pair.Value.surfaces;
                                if (surfaceLoops == null)
                                {
                                    outputLoops.intersectionLoops[pair.Key] = null;
                                }
                                else
                                {
                                    var loops = new Loop[surfaceLoops.Length];
                                    for (int i = 0; i < surfaceLoops.Length; i++)
                                    {
                                        var surfaceLoop = surfaceLoops[i];
                                        if (surfaceLoop == null ||
                                            surfaceLoop.Count == 0 ||
                                            surfaceLoop[0] == null ||
                                            !surfaceLoop[0].Valid)
                                        {
                                            loops[i] = null;
                                            continue;
                                        }

                                        Debug.Assert(surfaceLoop[0].Valid && (int)surfaceLoop[0].info.interiorCategory < CategoryRoutingRow.Length);

                                        loops[i] = surfaceLoop[0];
                                    }
                                    outputLoops.intersectionLoops[pair.Key] = loops;
                                }
                            }
                        }
                        finally { UnityEngine.Profiling.Profiler.EndSample(); }
                    }
                }
            }
            finally { UnityEngine.Profiling.Profiler.EndSample(); }
        }        
        #endregion
    }
#endif
}
