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
        
        #region IsOutsidePlanes

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


        [BurstCompile(CompileSynchronously = true)]
        public unsafe struct FindAllIntersectionLoopsJob : IJobParallelFor
        {
            [NoAlias,ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>  brushWorldPlanes;
            [NoAlias,ReadOnly] public NativeArray<BlobAssetReference<BrushPairIntersection>>    intersectingBrushes;

            [NoAlias,WriteOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>.ParallelWriter outputSurfaces;

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

        static Dictionary<int, Loop[]> sIntersectionLoopLookup = new Dictionary<int, Loop[]>();

        // TODO: convert all managed code to unmanaged code, convert data at end to something all subsequent phases can use (until those are rewritten)
        internal unsafe static void FindLoopOverlapIntersections(ref ChiselLookup.Data chiselLookupValues, NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>> intersectionLoopBlobs, NativeArray<int> treeBrushesArray, NativeArray<int> brushMeshInstanceIDs)
        {
            ref var brushWorldPlaneBlobs        = ref chiselLookupValues.brushWorldPlanes;
            ref var basePolygonBlobs            = ref chiselLookupValues.basePolygons;
            ref var routingTableLookup          = ref chiselLookupValues.routingTableLookup;
            ref var vertexSoups                 = ref chiselLookupValues.vertexSoups;

            var intersectionLoopBlobsKeys   = intersectionLoopBlobs.GetKeyArray(Allocator.TempJob);
            if (intersectionLoopBlobsKeys.Length > 0)
            { 
                for (int index = 0; index < treeBrushesArray.Length; index++)
                {
                    var brushNodeID         = treeBrushesArray[index];
                    var brushNodeIndex      = brushNodeID - 1;

                    if (!routingTableLookup.TryGetValue(brushNodeIndex, out BlobAssetReference<RoutingTable> routingTable))
                        continue;

                    var basePolygonBlob         = basePolygonBlobs[brushNodeIndex];
                    var surfaceCount            = basePolygonBlob.Value.surfaces.Length;

                    var brushSurfaceInfos       = new NativeList<SurfaceInfo>(intersectionLoopBlobsKeys.Length, Allocator.TempJob);                    
                    var intersectionEdges       = new NativeListArray<Edge>(intersectionLoopBlobsKeys.Length, 16, Allocator.TempJob);
                    var basePolygonEdges        = new NativeListArray<Edge>(surfaceCount, 16, Allocator.TempJob);
                    var vertexSoup              = new VertexSoup(basePolygonBlob.Value.vertices.Length + 16, Allocator.TempJob);

                    vertexSoups[brushNodeIndex] = vertexSoup;
                    var findLoopOverlapIntersectionsJob = new FindLoopOverlapIntersectionsJob
                    {
                        brushNodeIndex              = brushNodeIndex,
                        
                        intersectionLoopBlobs       = intersectionLoopBlobs,
                        intersectionLoopBlobsKeys   = intersectionLoopBlobsKeys,
                        
                        brushWorldPlanes            = brushWorldPlaneBlobs,
                        basePolygonBlob             = basePolygonBlob,
                        
                        vertexSoup                  = vertexSoup,
                        intersectionEdges           = intersectionEdges,
                        basePolygonEdges            = basePolygonEdges,
                        brushSurfaceInfos           = brushSurfaceInfos
                    };
                    findLoopOverlapIntersectionsJob.Run();


                    //***GET RID OF THIS***//
                    var outputLoops = CSGManager.GetBrushInfo(brushNodeID).brushOutputLoops;/*!!*/
                    outputLoops.Dispose();/*!!*/

                    {
                        var basePolygons = new List<Loop>(basePolygonBlob.Value.surfaces.Length);    /*OUTPUT*/
                        for (int s = 0; s < basePolygonBlob.Value.surfaces.Length; s++)
                        {
                            ref var input = ref basePolygonBlob.Value.surfaces[s];

                            var loop = new Loop()/*!!*/
                            {
                                info = input.surfaceInfo,
                                holes = new List<Loop>()/*!!*/
                            };

                            var edges = basePolygonEdges[s];
                            for (int e = 0; e < edges.Length; e++)
                                loop.edges.Add(edges[e]);
                            basePolygons.Add(loop);
                        }
                        outputLoops.basePolygons = basePolygons;/*!!*/ /*OUTPUT*/
                    }

                    var routingTableNodes = routingTable.Value.nodes.ToArray();

                    // Note: those above are only the INTERSECTING surfaces, below ALL surfaces
                    sIntersectionLoopLookup.Clear();
                    for (int i = 0; i < brushSurfaceInfos.Length; i++)
                    {
                        var surfaceInfo = brushSurfaceInfos[i];
                        var brushNodeIndex1 = surfaceInfo.brushNodeIndex;
                        var brushNodeID1 = brushNodeIndex1 + 1;

                        if (!routingTableNodes.Contains(brushNodeID1))
                            continue;

                        if (!sIntersectionLoopLookup.TryGetValue(brushNodeID1, out Loop[] loops))
                        {
                            loops = new Loop[surfaceCount];
                            sIntersectionLoopLookup[brushNodeID1] = loops;
                        }

                        loops[surfaceInfo.basePlaneIndex] = new Loop(intersectionEdges[i], surfaceInfo);
                    }


                    //**********
                    //**********
                    //**********
                    //**********
                    sIntersectionLoops.Clear(); // <-   this can be static. We know routingTableNodes.Length + surfaceCount, 
                                                //      need remap from index into sIntersectionLoops for nodeID
                                                //      also: this could be a continuous array with list of offsets
                                                //      -> this can be a NativeListArray of edges
                    //**********
                    //**********
                    //**********
                    //**********
                    for (int i = 0; i < routingTableNodes.Length; i++)
                    {
                        var cutting_node_id = routingTableNodes[i];
                        // Get the intersection loops between the two brushes on every surface of the brush we're performing CSG on
                        if (sIntersectionLoopLookup.TryGetValue(cutting_node_id, out Loop[] cuttingNodeIntersectionLoops) &&
                            cuttingNodeIntersectionLoops.Length > 0)
                            sIntersectionLoops.Add(cuttingNodeIntersectionLoops);
                        else
                            sIntersectionLoops.Add(null);
                    }
                    outputLoops.intersectionLoops = sIntersectionLoops.ToArray();
                    //***GET RID OF THIS***//

                    intersectionEdges.Dispose();
                    basePolygonEdges.Dispose();
                    brushSurfaceInfos.Dispose();
                }
            }
            intersectionLoopBlobsKeys.Dispose();
        }
#endregion
    }
#endif
}
