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
using UnityEngine.Profiling;

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


        [BurstCompile]
        public unsafe struct FindAllIntersectionLoopsJob : IJobParallelFor
        {
            [ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>  brushWorldPlanes;
            [ReadOnly] public NativeArray<BlobAssetReference<BrushPairIntersection>>    intersectingBrushes;

            [WriteOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>.ParallelWriter outputSurfaces;

            // TODO: move all the code that used to be in separate jobs here, instead of calling "Execute" on all of these

            public void Execute(int index)
            {
                if (index >= intersectingBrushes.Length)
                    return;

                var intersectionAsset               = intersectingBrushes[index];
                ref var intersection                = ref intersectionAsset.Value;
                ref var brushPairIntersection0      = ref intersection.brushes[0];
                ref var brushPairIntersection1      = ref intersection.brushes[1];
                var brushNodeIndex0                 = brushPairIntersection0.brushNodeIndex;
                var brushNodeIndex1                 = brushPairIntersection1.brushNodeIndex;

                int insideVerticesStream0Capacity   = math.max(1, brushPairIntersection0.usedVertices.Length);
                int insideVerticesStream1Capacity   = math.max(1, brushPairIntersection1.usedVertices.Length);
                int intersectionStream0Capacity     = math.max(1, brushPairIntersection1.usedPlanePairs.Length) * brushPairIntersection0.localSpacePlanes0.Length;
                int intersectionStream1Capacity     = math.max(1, brushPairIntersection0.usedPlanePairs.Length) * brushPairIntersection1.localSpacePlanes0.Length;
                int foundIndices0Capacity           = intersectionStream0Capacity + (2 * intersectionStream1Capacity) + (brushPairIntersection0.localSpacePlanes0.Length * insideVerticesStream0Capacity);
                int foundIndices1Capacity           = intersectionStream1Capacity + (2 * intersectionStream0Capacity) + (brushPairIntersection1.localSpacePlanes0.Length * insideVerticesStream1Capacity);
                    
                // TODO: allocate per intersection, perform all calculations/sorts, THEN create ALL surface-loops and assign indices
                var foundIndices0   = new NativeList<PlaneVertexIndexPair>(foundIndices0Capacity, Allocator.Temp);
                var foundIndices1   = new NativeList<PlaneVertexIndexPair>(foundIndices1Capacity, Allocator.Temp);

                // TODO: fill them with original brush vertices so that they're always snapped to these
                var vertexSoup0     = new VertexSoup(foundIndices0Capacity, Allocator.Temp);
                var vertexSoup1     = new VertexSoup(foundIndices1Capacity, Allocator.Temp);

                // First find vertices from other brush that are inside the other brush, so that any vertex we 
                // find during the intersection part will be snapped to those vertices and not the other way around

                // TODO: when all vertices of a polygon are inside the other brush, don't bother intersecting it.
                //       same when two planes overlap each other ...

                // Find all vertices of brush1 that are inside brush2, and put their intersections into the appropriate loops
                if (brushPairIntersection0.usedVertices.Length > 0)
                {
                    var insideVerticesStream0 = new NativeList<LocalWorldPair>(insideVerticesStream0Capacity, Allocator.Temp);
                    var findInsideVerticesJob = new FindInsideVerticesJob
                    {
                        intersection                = intersectionAsset,
                        intersectionPlaneIndex1     = 1,
                        usedVerticesIndex0          = 0,
                        vertexWriter                = insideVerticesStream0
                    };
                    findInsideVerticesJob.Execute();
                    if (insideVerticesStream0.Length > 0)
                    { 
                        var insertInsideVerticesJob = new InsertInsideVerticesJob
                        {
                            vertexReader            = insideVerticesStream0,
                            intersection            = intersectionAsset,
                            intersectionPlaneIndex  = 0,
                            brushVertices           = vertexSoup0,
                            outputIndices           = foundIndices0
                        };
                        insertInsideVerticesJob.Execute();
                    }
                    insideVerticesStream0.Dispose();
                }

                if (brushPairIntersection1.usedVertices.Length > 0)
                {
                    var insideVerticesStream1 = new NativeList<LocalWorldPair>(insideVerticesStream1Capacity, Allocator.Temp);
                    var findInsideVerticesJob = new FindInsideVerticesJob
                    {
                        intersection                = intersectionAsset,
                        intersectionPlaneIndex1     = 0,
                        usedVerticesIndex0          = 1,
                        vertexWriter                = insideVerticesStream1
                    };
                    findInsideVerticesJob.Execute();
                    if (insideVerticesStream1.Length > 0)
                    {
                        var insertInsideVerticesJob = new InsertInsideVerticesJob
                        {
                            vertexReader            = insideVerticesStream1,
                            intersection            = intersectionAsset,
                            intersectionPlaneIndex  = 1,
                            brushVertices           = vertexSoup1,
                            outputIndices           = foundIndices1
                        };
                        insertInsideVerticesJob.Execute();
                    }
                    insideVerticesStream1.Dispose();
                }

                // Now find all the intersection vertices
                if (intersection.type == IntersectionType.Intersection &&
                    brushPairIntersection1.usedPlanePairs.Length > 0)
                {
                    var intersectionStream0 = new NativeList<VertexAndPlanePair>(intersectionStream0Capacity, Allocator.Temp);

                    // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                    var findIntersectionsJob = new FindIntersectionsJob
                    {
                        intersection                = intersectionAsset,
                        intersectionPlaneIndex0     = 0,
                        intersectionPlaneIndex1     = 1,
                        usedPlanePairIndex1         = 1,
                        foundVertices               = intersectionStream0
                    };
                    findIntersectionsJob.Execute();
                    if (intersectionStream0.Length > 0)
                    { 
                        var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                        {
                            vertexReader                = intersectionStream0,
                            intersection                = intersectionAsset,
                            intersectionPlaneIndex0     = 0,
                            brushVertices0              = vertexSoup0,
                            brushVertices1              = vertexSoup1,
                            outputIndices0              = foundIndices0,
                            outputIndices1              = foundIndices1
                        };
                        insertIntersectionVerticesJob.Execute();
                    }
                    intersectionStream0.Dispose();
                }

                if (intersection.type == IntersectionType.Intersection &&
                    brushPairIntersection0.usedPlanePairs.Length > 0)
                {
                    var intersectionStream1 = new NativeList<VertexAndPlanePair>(intersectionStream1Capacity, Allocator.Temp);

                    // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                    var findIntersectionsJob = new FindIntersectionsJob
                    {
                        intersection                = intersectionAsset,
                        intersectionPlaneIndex0     = 1,
                        intersectionPlaneIndex1     = 0,
                        usedPlanePairIndex1         = 0,
                        foundVertices               = intersectionStream1
                    };
                    findIntersectionsJob.Execute();
                    if (intersectionStream1.Length > 0)
                    {
                        var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                        {
                            vertexReader                = intersectionStream1,
                            intersection                = intersectionAsset,
                            intersectionPlaneIndex0     = 1,
                            brushVertices0              = vertexSoup1,
                            brushVertices1              = vertexSoup0,
                            outputIndices0              = foundIndices1,
                            outputIndices1              = foundIndices0
                        };
                        insertIntersectionVerticesJob.Execute();
                    }
                    intersectionStream1.Dispose();
                }

                if (foundIndices0.Length > 0)
                {
                    var planeIndexOffsets0      = new NativeList<PlaneIndexOffsetLength>(foundIndices0.Length, Allocator.Temp);
                    var uniqueIndices0          = new NativeList<ushort>(foundIndices0.Length, Allocator.Temp);
                    var combineLoopIndicesJob0  = new CombineLoopIndicesJob
                    {
                        foundIndices                = foundIndices0,
                        uniqueIndices               = uniqueIndices0,
                        planeIndexOffsets           = planeIndexOffsets0
                    };
                    combineLoopIndicesJob0.Execute();
                    var sortLoopsJob0           = new SortLoopsJob
                    {
                        allBrushWorldPlanes         = brushWorldPlanes,
                        intersection                = intersectionAsset,
                        brushNodeIndex              = 0,
                        soup                        = vertexSoup0,
                        uniqueIndices               = uniqueIndices0,
                        planeIndexOffsets           = planeIndexOffsets0
                    };
                    sortLoopsJob0.Execute();
                    var createLoopsJob0         = new CreateLoopsJob
                    {
                        brushIndex0                 = brushNodeIndex0,
                        brushIndex1                 = brushNodeIndex1,
                        intersection                = intersectionAsset,
                        intersectionSurfaceIndex    = 0,
                        vertexSoup                  = vertexSoup0,
                        uniqueIndices               = uniqueIndices0,
                        planeIndexOffsets           = planeIndexOffsets0,
                        //outputSurfaces            = outputSurfaces
                    };
                    createLoopsJob0.Execute(ref outputSurfaces);
                    planeIndexOffsets0.Dispose();
                    uniqueIndices0.Dispose();
                }

                if (foundIndices1.Length > 0)
                {
                    var planeIndexOffsets1      = new NativeList<PlaneIndexOffsetLength>(foundIndices1.Length, Allocator.Temp);
                    var uniqueIndices1          = new NativeList<ushort>(foundIndices1.Length, Allocator.Temp);
                    var combineLoopIndicesJob1  = new CombineLoopIndicesJob
                    {
                        foundIndices                = foundIndices1,
                        uniqueIndices               = uniqueIndices1,
                        planeIndexOffsets           = planeIndexOffsets1
                    };
                    combineLoopIndicesJob1.Execute();
                    var sortLoopsJob1           = new SortLoopsJob
                    {
                        allBrushWorldPlanes         = brushWorldPlanes,
                        intersection                = intersectionAsset,
                        brushNodeIndex              = 1,
                        soup                        = vertexSoup1,
                        uniqueIndices               = uniqueIndices1,
                        planeIndexOffsets           = planeIndexOffsets1
                    };
                    sortLoopsJob1.Execute();
                    var createLoopsJob1         = new CreateLoopsJob
                    {
                        brushIndex0                 = brushNodeIndex1,
                        brushIndex1                 = brushNodeIndex0,
                        intersection                = intersectionAsset,
                        intersectionSurfaceIndex    = 1,
                        vertexSoup                  = vertexSoup1,
                        uniqueIndices               = uniqueIndices1,
                        planeIndexOffsets           = planeIndexOffsets1,
                        //outputSurfaces            = outputSurfaces
                    };
                    createLoopsJob1.Execute(ref outputSurfaces);
                    planeIndexOffsets1.Dispose();
                    uniqueIndices1.Dispose();
                }

                foundIndices0.Dispose();
                foundIndices1.Dispose();

                vertexSoup0.Dispose();
                vertexSoup1.Dispose();

                intersectionAsset.Dispose();
            }
        }


        #region FindLoopOverlapIntersections
        

        static List<Loop[]> sIntersectionLoops = new List<Loop[]>();
        
        // TODO: convert all managed code to unmanaged code, convert data at end to something all subsequent phases can use (until those are rewritten)
        internal unsafe static void FindLoopOverlapIntersections(ref ChiselLookup.Data chiselLookupValues, NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>> outputSurfaces, NativeArray<int> treeBrushes, NativeArray<int> brushMeshInstanceIDs)
        {
            ref var brushMeshBlobs = ref chiselLookupValues.brushMeshBlobs;
            ref var transformations = ref chiselLookupValues.transformations;

            Profiler.BeginSample("InitVertexSoup");
            // TODO: get rid of this somehow
            for (int b = 0; b < treeBrushes.Length; b++)
            {
                var brushNodeID     = treeBrushes[b];
                var brushNodeIndex  = brushNodeID - 1;
                var result          = chiselLookupValues.basePolygons[brushNodeIndex];
                var vertexSoup = new VertexSoup(ref result.Value.vertices);
                chiselLookupValues.vertexSoups.Add(brushNodeIndex, vertexSoup);
            }
            Profiler.EndSample();

            Profiler.BeginSample("CopyToLoops");
            // TODO: get rid of this, create blobassets as output
            using (var outputSurfaceKeys = outputSurfaces.GetKeyArray(Allocator.Temp))
            {
                for (int b = 0; b < treeBrushes.Length; b++)
                {
                    var brushNodeID     = treeBrushes[b];
                    var brushNodeIndex  = brushNodeID - 1;
                    var result          = chiselLookupValues.basePolygons[brushNodeIndex];
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

                    var vertexSoup          = chiselLookupValues.vertexSoups[brushNodeIndex0];
                    var outputSurface       = outputSurfaces[item];
                    surfaceLoops.surfaces[item.basePlaneIndex].Add(new Loop(outputSurface, vertexSoup));
                }
            }
            Profiler.EndSample();

            for (int b0 = 0; b0 < treeBrushes.Length; b0++)
            {
                var brush0NodeID    = treeBrushes[b0];
                var brush0NodeIndex = brush0NodeID - 1;
                var outputLoops = CSGManager.GetBrushInfo(brush0NodeID).brushOutputLoops;

                if (outputLoops.intersectionSurfaceLoops.Count == 0)
                    continue;

                var brushMeshID = brushMeshInstanceIDs[b0];
                var meshBlob = brushMeshBlobs[brushMeshID - 1];

                if (meshBlob.Value.localPlanes.Length == 0)
                    continue;

                var vertexSoup = chiselLookupValues.vertexSoups[brush0NodeIndex];
                var transform = transformations[brush0NodeIndex];

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

                                var intersectionJob = new FindLoopPlaneIntersectionsJob()// TODO: we're reading AND writing to the same NativeList!?!?!
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
                            var intersectionJob = new FindLoopPlaneIntersectionsJob()// TODO: we're reading AND writing to the same NativeList!?!?!
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
                                selfPlanes  = intersectionData.GetPlanes(intersectionSurface.segment),
                                vertexSoup    = vertexSoup,
                                otherEdges  = basePolygonEdges,
                                edges       = intersectionSurface.edges
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
                var brushNodeID     = treeBrushes[b];
                var brushNodeIndex  = brushNodeID - 1;

                if (!chiselLookupValues.routingTableLookup.TryGetValue(brushNodeIndex, out BlobAssetReference<RoutingTable> routingTable))
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
