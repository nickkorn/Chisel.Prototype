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
        
        static List<Loop[]> sIntersectionLoops = new List<Loop[]>();
        // Create unique loops between brush intersections
        internal unsafe static void FindAllIntersectionLoops(NativeArray<int> treeBrushes, NativeHashMap<int, BlobAssetReference<BrushMeshBlob>> brushMeshLookup)
        {
            // TODO: get rid of this, create blobasset as an output
            for (int b = 0; b < treeBrushes.Length; b++)
            {
                var brushNodeID = treeBrushes[b];
                var result      = ChiselLookup.Value.basePolygons[brushNodeID - 1];
                var vertexSoup = new VertexSoup();
                vertexSoup.Initialize(ref result.Value.vertices);
                ChiselLookup.Value.vertexSoups.Add(brushNodeID - 1, vertexSoup);
            }

            // TODO: - calculate all data per brush here (currently unused)
            //         and use this later on, hopefully making it possible to remove all intermediates
            //       - also get rid of all SurfaceLoops etc. in the middle, only store at the end
            //       - finally instead of storing stuff in SurfaceLoops at the end, store as blob
            using (var outputSurfaces       = new NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>(65500, Allocator.TempJob))
            using (var intersectingBrushes  = new NativeList<BlobAssetReference<BrushPairIntersection>>(GeometryMath.GetTriangleArraySize(treeBrushes.Length), Allocator.TempJob))
            {
                using (var brushPairs = new NativeList<BrushPair>(GeometryMath.GetTriangleArraySize(treeBrushes.Length), Allocator.TempJob))
                {
                    var findBrushPairsJob = new FindBrushPairsJob
                    {
                        treeBrushes             = treeBrushes,
                        brushesTouchedByBrushes = ChiselLookup.Value.brushesTouchedByBrushes,                        
                        brushPairs              = brushPairs
                    };
                    findBrushPairsJob.Run();

                    var prepareBrushPairIntersectionsJob = new PrepareBrushPairIntersectionsJob
                    {
                        brushPairs              = brushPairs,
                        brushMeshBlobLookup     = brushMeshLookup,
                        transformations         = ChiselLookup.Value.transformations,
                        intersectingBrushes     = intersectingBrushes.AsParallelWriter()
                    };
                    prepareBrushPairIntersectionsJob.Run();
                }
                
                foreach (var intersection in intersectingBrushes)
                {
                    var type            = intersection.Value.type;
                    var brushNodeIndex0 = intersection.Value.brushes[0].brushNodeIndex;
                    var brushNodeIndex1 = intersection.Value.brushes[1].brushNodeIndex;

                    // chain each subsection together, then chain subsections together
                    //var handleDep = new JobHandle();
                            
                    // TODO: allocate per intersection, perform all calculations/sorts, THEN create ALL surface-loops and assign indices
                    using (var planeIndexOffsets0       = new NativeList<PlaneIndexOffsetLength>(intersection.Value.brushes[0].surfaceInfos.Length, Allocator.TempJob))
                    using (var planeIndexOffsets1       = new NativeList<PlaneIndexOffsetLength>(intersection.Value.brushes[1].surfaceInfos.Length, Allocator.TempJob))
                    using (var foundIndices0            = new NativeList<PlaneVertexIndexPair>(intersection.Value.brushes[0].localSpacePlanes0.Length, Allocator.TempJob))
                    using (var foundIndices1            = new NativeList<PlaneVertexIndexPair>(intersection.Value.brushes[1].localSpacePlanes0.Length, Allocator.TempJob))
                    using (var uniqueIndices0           = new NativeList<ushort>(intersection.Value.brushes[0].localSpacePlaneIndices0.Length, Allocator.TempJob))
                    using (var uniqueIndices1           = new NativeList<ushort>(intersection.Value.brushes[1].localSpacePlaneIndices0.Length, Allocator.TempJob))
                    using (var insideVerticesStream0    = new NativeStream(math.max(1, intersection.Value.brushes[0].usedVertices.Length), Allocator.TempJob))
                    using (var insideVerticesStream1    = new NativeStream(math.max(1, intersection.Value.brushes[1].usedVertices.Length), Allocator.TempJob))
                    using (var intersectionStream0      = new NativeStream(intersection.Value.brushes[0].localSpacePlaneIndices0.Length, Allocator.TempJob))
                    using (var intersectionStream1      = new NativeStream(intersection.Value.brushes[1].localSpacePlaneIndices0.Length, Allocator.TempJob))
                    {
                        var vertexSoup0 = ChiselLookup.Value.vertexSoups[brushNodeIndex0];
                        var vertexSoup1 = ChiselLookup.Value.vertexSoups[brushNodeIndex1];

                        // First find vertices from other brush that are inside the other brush, so that any vertex we 
                        // find during the intersection part will be snapped to those vertices and not the other way around

                        // TODO: when all vertices of a polygon are inside the other brush, don't bother intersecting it.
                        //       same when two planes overlap each other ...

                        // Find all vertices of brush1 that are inside brush2, and put their intersections into the appropriate loops
                        if (intersection.Value.brushes[0].usedVertices.Length > 0)
                        {
                            var findInsideVerticesJob = new FindInsideVerticesJob
                            {
                                intersection                = intersection,
                                intersectionPlaneIndex1     = 1,
                                usedVerticesIndex0          = 0,
                                vertexWriter                = insideVerticesStream0.AsWriter()
                            };
                            findInsideVerticesJob.Run(intersection.Value.brushes[0].usedVertices.Length);
                            var insertInsideVerticesJob = new InsertInsideVerticesJob
                            {
                                vertexReader                = insideVerticesStream0.AsReader(),
                                intersection                = intersection,
                                intersectionPlaneIndex      = 0,
                                brushVertices               = vertexSoup0,
                                outputIndices               = foundIndices0
                            };
                            insertInsideVerticesJob.Run();
                        }
                        if (intersection.Value.brushes[1].usedVertices.Length > 0)
                        {
                            var findInsideVerticesJob = new FindInsideVerticesJob
                            {
                                intersection                = intersection,
                                intersectionPlaneIndex1     = 0,
                                usedVerticesIndex0          = 1,
                                vertexWriter                = insideVerticesStream1.AsWriter()
                            };
                            findInsideVerticesJob.Run(intersection.Value.brushes[1].usedVertices.Length);
                            var insertInsideVerticesJob = new InsertInsideVerticesJob
                            {
                                vertexReader                = insideVerticesStream1.AsReader(),
                                intersection                = intersection,
                                intersectionPlaneIndex      = 1,
                                brushVertices               = vertexSoup1,
                                outputIndices               = foundIndices1
                            }; 
                            insertInsideVerticesJob.Run();
                        }

                        // Now find all the intersection vertices
                        if (intersection.Value.type == IntersectionType.Intersection)
                        {
                            if (intersection.Value.brushes[1].usedPlanePairs.Length > 0)
                            {
                                // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                                var findIntersectionsJob = new FindIntersectionsJob
                                {
                                    vertexSoup1                 = vertexSoup1,
                                    intersection                = intersection,
                                    intersectionPlaneIndex0     = 0,
                                    intersectionPlaneIndex1     = 1,
                                    usedPlanePairIndex1         = 1,
                                    foundVertices               = intersectionStream0.AsWriter()
                                };
                                findIntersectionsJob.Run(intersection.Value.brushes[0].localSpacePlanes0.Length);
                                var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                                {
                                    vertexReader                = intersectionStream0.AsReader(),
                                    intersection                = intersection,
                                    intersectionPlaneIndex0     = 0,
                                    brushVertices0              = vertexSoup0,
                                    brushVertices1              = vertexSoup1,
                                    outputIndices0              = foundIndices0,
                                    outputIndices1              = foundIndices1
                                };
                                insertIntersectionVerticesJob.Run();
                            }

                            if (intersection.Value.brushes[0].usedPlanePairs.Length > 0)
                            {
                                // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                                var findIntersectionsJob = new FindIntersectionsJob
                                {
                                    vertexSoup1                 = vertexSoup0,
                                    intersection                = intersection,
                                    intersectionPlaneIndex0     = 1,
                                    intersectionPlaneIndex1     = 0,
                                    usedPlanePairIndex1         = 0,
                                    foundVertices               = intersectionStream1.AsWriter()
                                };
                                findIntersectionsJob.Run(intersection.Value.brushes[1].localSpacePlanes0.Length);
                                var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                                {
                                    vertexReader                = intersectionStream1.AsReader(),
                                    intersection                = intersection,
                                    intersectionPlaneIndex0     = 1,
                                    brushVertices0              = vertexSoup1,
                                    brushVertices1              = vertexSoup0,
                                    outputIndices0              = foundIndices1,
                                    outputIndices1              = foundIndices0
                                };
                                insertIntersectionVerticesJob.Run();
                            }
                        }

                        {
                            var sortLoopsJob0 = new SortLoopsJob
                            {
                                brushWorldPlanes            = ChiselLookup.Value.brushWorldPlanes[intersection.Value.brushes[0].brushNodeIndex],
                                vertexSoup                  = vertexSoup0,
                                foundIndices                = foundIndices0,
                                uniqueIndices               = uniqueIndices0,
                                planeIndexOffsets           = planeIndexOffsets0
                            };
                            sortLoopsJob0.Run();

                            var createLoopsJob0 = new CreateLoopsJob
                            {
                                brushIndex0                 = brushNodeIndex0,
                                brushIndex1                 = brushNodeIndex1,
                                intersection                = intersection,
                                intersectionSurfaceIndex    = 0,
                                vertexSoup                  = vertexSoup0,
                                uniqueIndices               = uniqueIndices0,
                                planeIndexOffsets           = planeIndexOffsets0,
                                outputSurfaces              = outputSurfaces.AsParallelWriter()
                            };
                            createLoopsJob0.Run(planeIndexOffsets0.Length);
                        }

                        {
                            var sortLoopsJob1 = new SortLoopsJob
                            {
                                brushWorldPlanes            = ChiselLookup.Value.brushWorldPlanes[intersection.Value.brushes[1].brushNodeIndex],
                                vertexSoup                  = vertexSoup1,
                                foundIndices                = foundIndices1,
                                uniqueIndices               = uniqueIndices1,
                                planeIndexOffsets           = planeIndexOffsets1
                            };
                            sortLoopsJob1.Run();

                            var createLoopsJob1 = new CreateLoopsJob
                            {
                                brushIndex0                 = brushNodeIndex1,
                                brushIndex1                 = brushNodeIndex0,
                                intersection                = intersection,
                                intersectionSurfaceIndex    = 1,
                                vertexSoup                  = vertexSoup1,
                                uniqueIndices               = uniqueIndices1,
                                planeIndexOffsets           = planeIndexOffsets1,
                                outputSurfaces              = outputSurfaces.AsParallelWriter(),
                            };
                            createLoopsJob1.Run(planeIndexOffsets1.Length);
                        }
                    }
                }
                foreach (var intersection in intersectingBrushes)
                    intersection.Dispose();

                // TODO: store somewhere else instead

                using (var outputSurfaceKeys = outputSurfaces.GetKeyArray(Allocator.Temp))
                {
                    // TODO: get rid of this, create blobasset as an output
                    for (int b = 0; b < treeBrushes.Length; b++)
                    {
                        var brushNodeID = treeBrushes[b];
                        var result = ChiselLookup.Value.basePolygons[brushNodeID - 1];
                        var outputLoops = CSGManager.GetBrushInfo(brushNodeID).brushOutputLoops;
                        outputLoops.Dispose();

                        outputLoops.basePolygons = new List<Loop>(result.Value.surfaces.Length);
                        for (int p = 0; p < result.Value.surfaces.Length; p++)
                        {
                            ref var input = ref result.Value.surfaces[p];
                            var surfacePolygon = new Loop()
                            {
                                info = input.surfaceInfo,
                                holes = new List<Loop>()
                            };
                            for (int e = input.startEdgeIndex; e < input.endEdgeIndex; e++)
                                surfacePolygon.edges.Add(result.Value.edges[e]);
                            outputLoops.basePolygons.Add(surfacePolygon);
                        }
                    }
                    for (int b = 0; b < treeBrushes.Length; b++)
                    {
                        var brushNodeID = treeBrushes[b];
                        var brushOutputLoops = CSGManager.GetBrushInfo(brushNodeID).brushOutputLoops;
                        foreach (var item in brushOutputLoops.intersectionSurfaceLoops)
                            item.Value.Dispose();
                        brushOutputLoops.intersectionSurfaceLoops.Clear();
                    }
                    foreach (var item in outputSurfaceKeys)
                    {
                        var brushNodeIndex0     = item.brushNodeIndex0;
                        var brushNodeID0        = brushNodeIndex0 + 1;

                        var brushNodeIndex1     = item.brushNodeIndex1;
                        var brushNodeID1        = brushNodeIndex1 + 1;

                        var brushOutputLoops0   = CSGManager.GetBrushInfo(brushNodeID0).brushOutputLoops;
                        if (!brushOutputLoops0.intersectionSurfaceLoops.TryGetValue(brushNodeID1, out SurfaceLoops surfaceLoops))
                        {
                            surfaceLoops = new SurfaceLoops(brushOutputLoops0.basePolygons.Count);
                            brushOutputLoops0.intersectionSurfaceLoops[brushNodeID1] = surfaceLoops;
                        }

                        var vertexSoup          = ChiselLookup.Value.vertexSoups[brushNodeIndex0];
                        var outputSurface       = outputSurfaces[item];
                        surfaceLoops.surfaces[item.basePlaneIndex].Add(new Loop(outputSurface, vertexSoup));
                    }
                }
            }
        }


        internal unsafe static void FindLoopOverlapIntersections(NativeArray<int> treeBrushes, NativeArray<int> brushMeshInstanceIDs)
        {
            ref var brushMeshBlobs = ref ChiselLookup.Value.brushMeshBlobs;
            ref var transformations = ref ChiselLookup.Value.transformations;

            for (int b0 = 0; b0 < treeBrushes.Length; b0++)
            {
                var brush0NodeID = treeBrushes[b0];
                var outputLoops = CSGManager.GetBrushInfo(brush0NodeID).brushOutputLoops;

                if (outputLoops.intersectionSurfaceLoops.Count == 0)
                    continue;

                var brushMeshID = brushMeshInstanceIDs[b0];
                var meshBlob = brushMeshBlobs[brushMeshID - 1];

                if (meshBlob.Value.localPlanes.Length == 0)
                    continue;

                var vertexSoup = ChiselLookup.Value.vertexSoups[brush0NodeID - 1];
                var transform = transformations[brush0NodeID - 1];

                using (var intersectionData = new OverlapIntersectionData(brush0NodeID, transform, meshBlob, outputLoops.intersectionSurfaceLoops, outputLoops.basePolygons))
                {
                    intersectionData.Execute();

                    for (int s = 0; s < intersectionData.intersectionSurfaces.Length; s++)
                    {
                        var intersectionSurfaceList = intersectionData.intersectionSurfaces[s];
                        for (int l0 = intersectionSurfaceList.Count - 1; l0 >= 0; l0--)
                        {
                            var intersectionSurface0 = intersectionData.allIntersectionLoops[intersectionSurfaceList[l0]];
                            var edges = intersectionSurface0.edges;
                            for (int l1 = 0; l1 < intersectionSurfaceList.Count; l1++)
                            {
                                if (l0 == l1)
                                    continue;

                                var intersectionJob = new FindLoopPlaneIntersectionsJob()
                                {
                                    allWorldSpacePlanes = intersectionData.allWorldSpacePlanes,
                                    otherPlanesSegment = intersectionData.allIntersectionLoops[intersectionSurfaceList[l1]].segment,
                                    selfPlanesSegment = intersectionSurface0.segment,
                                    vertexSoup = vertexSoup,
                                    edges = edges
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
                        var edges = intersectionData.basePolygonLoops[b].edges;
                        foreach (var brushPlaneSegment in intersectionData.brushPlaneSegments)
                        {
                            var intersectionJob = new FindLoopPlaneIntersectionsJob()
                            {
                                allWorldSpacePlanes = intersectionData.allWorldSpacePlanes,
                                otherPlanesSegment = brushPlaneSegment,
                                selfPlanesSegment = intersectionData.worldSpacePlanes0Segment,
                                vertexSoup = vertexSoup,
                                edges = edges
                            };
                            intersectionJob.Run();
                        }
                    }


                    for (int s = 0; s < intersectionData.intersectionSurfaces.Length; s++)
                    {
                        var intersectionSurfaceList = intersectionData.intersectionSurfaces[s];
                        if (intersectionSurfaceList.Count == 0)
                            continue;

                        var basePolygonEdges = intersectionData.basePolygonLoops[s].edges;
                        for (int l0 = 0; l0 < intersectionSurfaceList.Count; l0++)
                        {
                            var intersectionSurface = intersectionData.allIntersectionLoops[intersectionSurfaceList[l0]];
                            var intersectionJob2 = new FindLoopVertexOverlapsJob
                            {
                                selfPlanes = intersectionData.GetPlanes(intersectionSurface.segment),
                                otherEdges = basePolygonEdges,
                                vertexSoup = vertexSoup,
                                edges = intersectionSurface.edges
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
