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
        
        #region IsOutsidePlanes

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe bool IsOutsidePlanes(NativeArray<float4> planes, float4 localVertex)
        {
            const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;
            var planePtr = (float4*)planes.GetUnsafeReadOnlyPtr();
            for (int n = 0; n < planes.Length; n++)
            {
                var distance = math.dot(planePtr[n], localVertex);

                // will be 'false' when distance is NaN or Infinity
                if (!(distance <= kEpsilon))
                    return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe bool IsOutsidePlanes(ref BlobArray<float4> planes, float4 localVertex)
        {
            const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;
            var planePtr = (float4*)planes.GetUnsafePtr();
            for (int n = 0; n < planes.Length; n++)
            {
                var distance = math.dot(planePtr[n], localVertex);

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
            public BlobAssetReference<BrushMeshBlob>    blobMesh;
            public float4x4                             treeToNodeSpaceMatrix;
            public float4x4                             nodeToTreeSpaceMatrix;
            public int2                                 planeSegment;

            public BrushLoops               outputLoops;
            public List<int>[]              intersectionSurfaces;
            public IntersectionLoop[]       basePolygonLoops;
            public static readonly BrushData Default = new BrushData()
            {
                blobMesh                = BlobAssetReference<BrushMeshBlob>.Null,
                treeToNodeSpaceMatrix   = float4x4.identity,
                nodeToTreeSpaceMatrix   = float4x4.identity
            };
        }
        
        internal unsafe class Temporaries : IDisposable
        {
            public NativeList<float4>               allWorldSpacePlanes;
            public NativeList<int2>                 brushPlaneSegments;

            public List<BrushData>                  brushDataList;
            public NativeArray<int>                 treeBrushes;
            public HashSet<BrushBrushIntersection>  intersectingBrushes = new HashSet<BrushBrushIntersection>();
            public List<IntersectionLoop>           allIntersectionLoops;
            public Temporaries(NativeArray<int> treeBrushes)
            {
                this.treeBrushes            = treeBrushes;
                this.allWorldSpacePlanes    = new NativeList<float4>(4, Allocator.Persistent);
                this.brushPlaneSegments     = new NativeList<int2>(treeBrushes.Length, Allocator.Persistent);
                
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
                    this.allIntersectionLoops.Clear();
                brushDataList.Clear();
            }

            public void Init(NativeArray<int> brushMeshInstanceIDs)
            {
                int totalPlanes = 0;

                ref var brushMeshBlobs          = ref ChiselLookup.Value.brushMeshBlobs;
                ref var brushesTouchedByBrushes = ref ChiselLookup.Value.brushesTouchedByBrushes;
                ref var transformations         = ref ChiselLookup.Value.transformations;

                intersectingBrushes.Clear();
                for (int b0 = 0; b0 < treeBrushes.Length; b0++)
                {
                    var brushNodeID0     = treeBrushes[b0];
                    var brushNodeIndex0  = brushNodeID0 - 1;
                    if (!brushesTouchedByBrushes.TryGetValue(brushNodeIndex0, out BlobAssetReference<BrushesTouchedByBrush> brushesTouchedByBrush))
                    {
                        brushDataList.Add(BrushData.Default);
                        continue;
                    }

                    ref var intersections = ref brushesTouchedByBrush.Value.brushIntersections;
                    if (intersections.Length == 0)
                    {
                        brushDataList.Add(BrushData.Default);
                        continue;
                    }

                    var brushMeshID0        = brushMeshInstanceIDs[b0];
                    var blobMesh0           = brushMeshBlobs[brushMeshID0 - 1];

                    if (!blobMesh0.IsCreated || blobMesh0.Value.IsEmpty()) continue;

                    // Find all intersections between brushes
                    for (int i = 0; i < intersections.Length; i++)
                    {
                        var intersection = intersections[i];
                        var brushNodeIndex1 = intersection.nodeIndex;

                        var reverseOrder = brushNodeIndex0 > brushNodeIndex1; // ensures we do calculations exactly the same for each brush pair
                        var type = intersection.type;

                        // TODO: get rid of this
                        var brushMeshID1 = new CSGTreeBrush() { brushNodeID = brushNodeIndex1 + 1 }.BrushMesh.BrushMeshID;
                        var blobMesh1 = brushMeshBlobs[brushMeshID1 - 1];
                        if (!blobMesh1.IsCreated || blobMesh1.Value.IsEmpty()) continue;

                        BlobAssetReference<BrushMeshBlob> foundBlobMesh0, foundBlobMesh1;
                        int foundBrushNodeID0, foundBrushNodeID1;
                        if (reverseOrder)
                        {
                            if (type == IntersectionType.AInsideB) type = IntersectionType.BInsideA;
                            else if (type == IntersectionType.BInsideA) type = IntersectionType.AInsideB;
                            foundBrushNodeID0 = brushNodeIndex1 + 1;
                            foundBrushNodeID1 = brushNodeIndex0 + 1;

                            foundBlobMesh0 = blobMesh1;
                            foundBlobMesh1 = blobMesh0;
                        }  else
                        {
                            foundBrushNodeID0 = brushNodeIndex0 + 1;
                            foundBrushNodeID1 = brushNodeIndex1 + 1;

                            foundBlobMesh0 = blobMesh0;
                            foundBlobMesh1 = blobMesh1;
                        }

                        intersectingBrushes.Add(new BrushBrushIntersection()
                        {
                            brushNodeID0 = foundBrushNodeID0,
                            brushNodeID1 = foundBrushNodeID1,
                            blobMesh0    = foundBlobMesh0,
                            blobMesh1    = foundBlobMesh1,
                            type         = type,
                        }); // uses hashset to ensure this is unique
                    }

                    var transform           = transformations[brushNodeID0 - 1];
                    var newBrushData        = new BrushData()
                    {
                        blobMesh                = blobMesh0,
                        treeToNodeSpaceMatrix   = transform.Value.treeToNode,
                        nodeToTreeSpaceMatrix   = transform.Value.nodeToTree,
                        outputLoops             = CSGManager.GetBrushInfo(brushNodeID0).brushOutputLoops
                    };
                    totalPlanes += newBrushData.blobMesh.Value.localPlanes.Length;
                    brushDataList.Add(newBrushData);
                }

                allWorldSpacePlanes.Capacity = totalPlanes;
            }

            public void Execute()
            {
                var indexLookup = new Dictionary<int, int>();
                for (int b0 = 0; b0 < treeBrushes.Length; b0++)
                {
                    indexLookup[treeBrushes[b0]] = b0;
                    var newBrushData = brushDataList[b0];
                    if (newBrushData.blobMesh == BlobAssetReference<BrushMeshBlob>.Null)
                        continue;

                    var treeToNodeSpaceTransposed = math.transpose(newBrushData.treeToNodeSpaceMatrix);
                    var meshPlanes          = (float4*)newBrushData.blobMesh.Value.localPlanes.GetUnsafePtr();
                    var startIndex          = allWorldSpacePlanes.Length;
                    allWorldSpacePlanes.AddRange(meshPlanes, newBrushData.blobMesh.Value.localPlanes.Length);
                    var segment             = new int2(startIndex, allWorldSpacePlanes.Length - startIndex);
                    brushPlaneSegments.Add(segment);
                    var worldSpacePlanesPtr = ((float4*)allWorldSpacePlanes.GetUnsafePtr()) + segment.x;
                    TransformByTransposedInversedMatrix(worldSpacePlanesPtr, segment.y, treeToNodeSpaceTransposed);
                    newBrushData.planeSegment = segment;
                }

                for (int b0 = 0; b0 < treeBrushes.Length; b0++)
                    {
                        var newBrushData = brushDataList[b0];
                        if (newBrushData.blobMesh == BlobAssetReference<BrushMeshBlob>.Null)
                            continue;

                        var basePolygons = newBrushData.outputLoops.basePolygons;

                        newBrushData.basePolygonLoops       = new IntersectionLoop[basePolygons.Count];
                        newBrushData.intersectionSurfaces   = new List<int>[newBrushData.blobMesh.Value.localPlanes.Length];
                        for (int i = 0; i < newBrushData.intersectionSurfaces.Length; i++)
                            newBrushData.intersectionSurfaces[i] = new List<int>(16);

                        for (int s = 0; s < newBrushData.basePolygonLoops.Length; s++)
                        {
                            var loop = basePolygons[s];
                            var intersectionLoop = new IntersectionLoop()
                            {
                                segment         = newBrushData.planeSegment,
                                edges           = loop.edges,
                                surfaceIndex    = s,
                                brushNodeID     = treeBrushes[b0]
                            };

                            this.allIntersectionLoops.Add(intersectionLoop);
                            newBrushData.basePolygonLoops[s] = intersectionLoop;
                        }
                    }
            }
        }

        static List<Loop[]> sIntersectionLoops = new List<Loop[]>();
        internal unsafe static void FindAllIntersectionLoops(NativeArray<int> treeBrushes, NativeArray<int> brushMeshInstanceIDs)
        {
            ref var brushMeshBlobs = ref ChiselLookup.Value.brushMeshBlobs;
            ref var transformations = ref ChiselLookup.Value.transformations;

            // TODO: - calculate all data per brush here (currently unused)
            //         and use this later on, hopefully making it possible to remove all intermediates
            //       - also get rid of all SurfaceLoops etc. in the middle, only store at the end
            //       - finally instead of storing stuff in SurfaceLoops at the end, store as blob
            using (var temporaries = new Temporaries(treeBrushes))
            {
                temporaries.Init(brushMeshInstanceIDs);
                temporaries.Execute();

                // Create unique loops between brush intersections
                UnityEngine.Profiling.Profiler.BeginSample("CreateIntersectionLoops");
                try
                {
                    foreach (var intersection in temporaries.intersectingBrushes)
                    {
                        var type = intersection.type;
                        var brushNodeID0 = intersection.brushNodeID0;
                        var brushNodeID1 = intersection.brushNodeID1;
                        var blobMesh0 = intersection.blobMesh0;
                        var blobMesh1 = intersection.blobMesh1;

                        // TODO: put this in a burstable data structure
                        var loops01 = new SurfaceLoops(blobMesh0.Value.localPlanes.Length);
                        var loops10 = new SurfaceLoops(blobMesh1.Value.localPlanes.Length);

                        var brushOutputLoops0 = CSGManager.GetBrushInfo(brushNodeID0).brushOutputLoops;
                        var brushOutputLoops1 = CSGManager.GetBrushInfo(brushNodeID1).brushOutputLoops;
                        var vertexSoup0 = brushOutputLoops0.vertexSoup;
                        var vertexSoup1 = brushOutputLoops1.vertexSoup;

                        var transformations0 = transformations[brushNodeID0 - 1];
                        var transformations1 = transformations[brushNodeID1 - 1];


                        using (var sharedPlaneData = new SharedPlaneData(vertexSoup0, brushNodeID0, blobMesh0, transformations0,
                                                                         vertexSoup1, brushNodeID1, blobMesh1, transformations1,
                                                                         type, Allocator.TempJob))
                        {
                            sharedPlaneData.Run();
                            if (sharedPlaneData.intersectingPlanes0.Length == 0 && sharedPlaneData.intersectingPlanes1.Length == 0)
                            {
                                loops01.Dispose();
                                loops10.Dispose();
                                continue;
                            }

                            // chain each subsection together, then chain subsections together
                            //var handleDep = new JobHandle();

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
                                            usedPlanePairs1         = sharedPlaneData.usedPlanePairs1,
                                            vertexSoup1             = sharedPlaneData.vertexSoup1,
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
                                            usedPlanePairs1          = sharedPlaneData.usedPlanePairs0,
                                            vertexSoup1              = sharedPlaneData.vertexSoup0,
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
                        {
                            if (brushOutputLoops0.intersectionSurfaceLoops.TryGetValue(brushNodeID1, out SurfaceLoops oldLoops1))
                                oldLoops1.Dispose();
                            if (brushOutputLoops0.intersectionSurfaceLoops.TryGetValue(brushNodeID0, out SurfaceLoops oldLoops0))
                                oldLoops0.Dispose();
                        }

                        brushOutputLoops0.intersectionSurfaceLoops[brushNodeID1] = loops01;
                        brushOutputLoops1.intersectionSurfaceLoops[brushNodeID0] = loops10;
                    }
                }
                finally { UnityEngine.Profiling.Profiler.EndSample(); }

                UnityEngine.Profiling.Profiler.BeginSample("FindLoopOverlapIntersections");
                try
                {
                    for (int b0 = 0; b0 < treeBrushes.Length; b0++)
                    {
                        var brush0NodeID    = treeBrushes[b0];
                        var outputLoops     = CSGManager.GetBrushInfo(brush0NodeID).brushOutputLoops;

                        if (outputLoops.intersectionSurfaceLoops.Count == 0)
                            continue;

                        var brushMeshID = brushMeshInstanceIDs[b0];
                        var meshBlob    = brushMeshBlobs[brushMeshID - 1];

                        if (meshBlob.Value.localPlanes.Length == 0)
                            continue;

                        var vertexSoup  = outputLoops.vertexSoup;
                        var transform   = transformations[brush0NodeID - 1];

                        using (var intersectionData = new OverlapIntersectionData(brush0NodeID, transform, meshBlob, outputLoops.intersectionSurfaceLoops, outputLoops.basePolygons))
                        {
                            intersectionData.Execute();

                            for (int s = 0; s < intersectionData.intersectionSurfaces.Length; s++)
                            {
                                var intersectionSurfaceList = intersectionData.intersectionSurfaces[s];
                                for (int l0 = intersectionSurfaceList.Count - 1; l0 >= 0; l0--)
                                {
                                    var intersectionSurface0 = intersectionData.allIntersectionLoops[intersectionSurfaceList[l0]];
                                    var edges                = intersectionSurface0.edges;
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
                                            edges               = edges
                                        };
                                        intersectionJob.Run();

                                        // TODO: merge these so that intersections will be identical on both loops (without using math, use logic)
                                        // TODO: make sure that intersections between loops will be identical on OTHER brushes (without using math, use logic)
                                    }
                                }
                            }

                            // TODO: should only intersect with all brushes that each particular basepolygon intersects with
                            //       but also need adjency information between basePolygons to ensure that intersections exist on 
                            //       both sides of each edge on a brush. 
                            for (int b = 0; b < intersectionData.basePolygonLoops.Length; b++)
                            {
                                var edges   = intersectionData.basePolygonLoops[b].edges;
                                foreach (var brushPlaneSegment in intersectionData.brushPlaneSegments)
                                {
                                    var intersectionJob = new FindLoopPlaneIntersectionsJob()
                                    {
                                        allWorldSpacePlanes = intersectionData.allWorldSpacePlanes,
                                        otherPlanesSegment  = brushPlaneSegment,
                                        selfPlanesSegment   = intersectionData.worldSpacePlanes0Segment,
                                        vertexSoup          = vertexSoup,
                                        edges               = edges
                                    };
                                    intersectionJob.Run();
                                }
                            }
                            

                            for (int s = 0; s < intersectionData.intersectionSurfaces.Length; s++)
                            {
                                var intersectionSurfaceList = intersectionData.intersectionSurfaces[s];
                                if (intersectionSurfaceList.Count == 0)
                                    continue;
                                
                                var basePolygonEdges    = intersectionData.basePolygonLoops[s].edges;
                                for (int l0 = 0; l0 < intersectionSurfaceList.Count; l0++)
                                {
                                    var intersectionSurface = intersectionData.allIntersectionLoops[intersectionSurfaceList[l0]];
                                    var intersectionJob2 = new FindLoopVertexOverlapsJob
                                    {
                                        selfPlanes      = intersectionData.GetPlanes(intersectionSurface.segment),
                                        otherEdges      = basePolygonEdges,
                                        vertexSoup      = vertexSoup,
                                        edges           = intersectionSurface.edges
                                    };
                                    intersectionJob2.Run();
                                }
                            }

                            for (int i = intersectionData.allIntersectionLoops.Count - 1; i >= 0; i--)
                            {
                                // TODO: might not be necessary
                                var edges = intersectionData.allIntersectionLoops[i].edges;
                                var removeIdenticalIndicesEdgesJob = new RemoveIdenticalIndicesEdgesJob { edges = edges };
                                removeIdenticalIndicesEdgesJob.Run();
                            }

                            for (int i = intersectionData.basePolygonLoops.Length - 1; i >= 0; i--)
                            {
                                // TODO: might not be necessary
                                var edges = intersectionData.basePolygonLoops[i].edges;
                                var removeIdenticalIndicesEdgesJob = new RemoveIdenticalIndicesEdgesJob { edges = edges };
                                removeIdenticalIndicesEdgesJob.Run();
                            }

                            // TODO: eventually merge indices across multiple loops when vertices are identical

                            intersectionData.StoreOutput(outputLoops.intersectionSurfaceLoops, outputLoops.intersectionLoopLookup, outputLoops.basePolygons);
                        }
                    }
                }
                finally { UnityEngine.Profiling.Profiler.EndSample(); }
            }


            for (int b = 0; b < treeBrushes.Length; b++)
            {
                var brushNodeID = treeBrushes[b];

                if (!ChiselLookup.Value.routingTableLookup.TryGetValue(brushNodeID - 1, out BlobAssetReference<RoutingTable> routingTable))
                    continue;

                var brushOutputLoops = CSGManager.GetBrushInfo(brushNodeID).brushOutputLoops;
                ref var nodes = ref routingTable.Value.nodes;
                sIntersectionLoops.Clear();
                for (int i = 0; i < nodes.Length; i++)
                {
                    var cutting_node_id = nodes[i];
                    // Get the intersection loops between the two brushes on every surface of the brush we're performing CSG on
                    if (!brushOutputLoops.intersectionLoopLookup.TryGetValue(cutting_node_id, out Loop[] cuttingNodeIntersectionLoops))
                        cuttingNodeIntersectionLoops = null;
                    sIntersectionLoops.Add(cuttingNodeIntersectionLoops);
                }
                brushOutputLoops.intersectionLoops = sIntersectionLoops.ToArray();
            }
        }        
        #endregion
    }
#endif
}
