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

    internal struct IntersectionLoop
    {
        public int                  surfaceIndex;
        public int                  brushNodeIndex;
        public NativeList<Edge>     edges;
        //public BlobAssetReference<BrushIntersectionLoop> loop;
    }

    internal unsafe struct FindLoopOverlapIntersectionsJob : IJobParallelFor
    {
        [NoAlias, ReadOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>> intersectionLoopBlobs;
        [NoAlias, ReadOnly] public NativeArray<BrushSurfacePair>                                 intersectionLoopBlobsKeys;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>>      basePolygonBlobs;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>      brushWorldPlanes;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>         brushMeshBlobs;
        [NoAlias, ReadOnly] public NativeArray<int>                                              treeBrushes;
        [NoAlias, ReadOnly] public NativeArray<int>                                              brushMeshInstanceIDs;

        public Dictionary<int, VertexSoup>                                  vertexSoups; /*!!*/

        struct Empty { };

        public unsafe void Execute(int index)
        {
            if (intersectionLoopBlobsKeys.Length == 0)
                return;


            var brushNodeID0        = treeBrushes[index];
            var brushNodeIndex0     = brushNodeID0 - 1;

            var basePolygonBlob     = basePolygonBlobs[brushNodeIndex0];
            var surfaceCount        = basePolygonBlob.Value.surfaces.Length;
            if (surfaceCount == 0)
                return;

            var uniqueBrushIndices  = new NativeHashMap<int, Empty>(intersectionLoopBlobsKeys.Length, Allocator.TempJob);

            // TODO: get rid of this somehow
            foreach (var item in intersectionLoopBlobsKeys)
            {
                if (item.brushNodeIndex0 != brushNodeIndex0)
                    continue;
                    
                uniqueBrushIndices[item.brushNodeIndex1] = new Empty();
            }

            var uniqueBrushIndexCount = uniqueBrushIndices.Count();
            if (uniqueBrushIndexCount == 0)
            {
                // If we don't have any intersection loops, just convert basePolygonBlob to loops and be done
                // TODO: should do this per surface!
                uniqueBrushIndices.Dispose();
                
                var basePolygons = new List<Loop>(basePolygonBlob.Value.surfaces.Length);/*!!*/
                for (int p = 0; p < basePolygonBlob.Value.surfaces.Length; p++)
                {
                    ref var input = ref basePolygonBlob.Value.surfaces[p];
                    var surfacePolygon = new Loop()/*!!*/
                    {
                        info = input.surfaceInfo,
                        holes = new List<Loop>()/*!!*/
                    };
                    for (int e = input.startEdgeIndex; e < input.endEdgeIndex; e++)
                        surfacePolygon.edges.Add(basePolygonBlob.Value.edges[e]);
                    basePolygons.Add(surfacePolygon);
                }


                //***GET RID OF THIS***//
                var outputLoops         = CSGManager.GetBrushInfo(brushNodeID0).brushOutputLoops;/*!!*/
                outputLoops.Dispose();/*!!*/
                outputLoops.basePolygons    = basePolygons;/*!!*/ /*OUTPUT*/
                var vertexSoup          = new VertexSoup(ref basePolygonBlob.Value.vertices); /*OUTPUT*/
                vertexSoups.Add(brushNodeIndex0, vertexSoup);/*!!*/  /*OUTPUT*/
                //***GET RID OF THIS***//
                return;
            }



            {
                var intersectingUniqueBrushIndices = uniqueBrushIndices.GetKeyArray(Allocator.Temp);
                uniqueBrushIndices.Dispose();

                var vertexSoup                  = new VertexSoup(ref basePolygonBlob.Value.vertices);       /*OUTPUT*/

                //***GET RID OF THIS***//
                var surfaceLoops                = new SurfaceLoops(surfaceCount);                           /*!!*/
                var intersectionSurfaceSegments = new int2[surfaceCount];                                   /*!!*/
                var allIntersectionLoops        = new List<IntersectionLoop>();                             /*!!*/
                //***GET RID OF THIS***//


                var basePolygonEdges = new NativeListArray<Edge>(surfaceCount, 16, Allocator.TempJob);
                
                {
                    {
                        for (int s = 0; s < basePolygonBlob.Value.surfaces.Length; s++)
                        {
                            ref var input = ref basePolygonBlob.Value.surfaces[s];

                            var edges = basePolygonEdges[s];
                            edges.Capacity = input.endEdgeIndex - input.startEdgeIndex;
                            for (int e = input.startEdgeIndex; e < input.endEdgeIndex; e++)
                                edges.Add(basePolygonBlob.Value.edges[e]);
                        }

                        // TODO: get rid of this somehow
                        foreach (var item in intersectionLoopBlobsKeys)
                        {
                            if (item.brushNodeIndex0 != brushNodeIndex0)
                                continue;

                            var outputSurface   = intersectionLoopBlobs[item];
                            var newLoop         = new Loop(outputSurface, vertexSoup);/*!!*/ /* vertexSoup -> OUTPUT*/
                            surfaceLoops.surfaces[item.basePlaneIndex].Add(newLoop);
                        }

                        for (int s = 0; s < surfaceLoops.surfaces.Length; s++)
                        {
                            var startIndex          = allIntersectionLoops.Count;
                            var loops               = surfaceLoops.surfaces[s];
                            if (loops == null ||
                                loops.Count == 0)
                            {
                                intersectionSurfaceSegments[s] = new int2(startIndex, 0);
                                continue;
                            }

                            intersectionSurfaceSegments[s] = new int2(startIndex, loops.Count);
                            for (int l = 0; l < loops.Count; l++)
                            {
                                var loop                = loops[l];
                                var brushNodeIndex1     = loop.info.brushNodeIndex;

                                // At this point it should not be possible to have more loop from the same intersecting brush since 
                                // we only support convex brushes. 
                                // Later on, however, when we start intersecting loops with each other, we can end up with multiple fragments.

                                var intersectionLoop = new IntersectionLoop()
                                {
                                    surfaceIndex    = s,
                                    brushNodeIndex  = brushNodeIndex1,
                                    edges           = loop.edges,
                                    //loop          = intersectionLoopBlobs[new BrushSurfacePair { basePlaneIndex =s, brushNodeIndex0 = brushNodeIndex0, brushNodeIndex1 = brushNodeIndex1 }]
                                }; 

                                // We add the intersection loop of this particular brush with our own brush
                                allIntersectionLoops.Add(intersectionLoop);
                            }
                        }
                    }

                    for (int s = 0; s < surfaceCount; s++)
                    {
                        var intersectionSurfaceCount    = intersectionSurfaceSegments[s].y;
                        var intersectionSurfaceOffset   = intersectionSurfaceSegments[s].x;
                        for (int l0 = intersectionSurfaceCount - 1; l0 >= 0; l0--)
                        {
                            var intersectionSurface0 = allIntersectionLoops[intersectionSurfaceOffset + l0];
                            var edges = intersectionSurface0.edges;
                            for (int l1 = 0; l1 < intersectionSurfaceCount; l1++)
                            {
                                if (l0 == l1)
                                    continue;

                                // TODO: use intersectionLoopBlobs/basePolygonBlobs as input + planes, output new blob
                                var intersectionJob = new FindLoopPlaneIntersectionsJob()// TODO: we're reading AND writing to the same NativeList!?!?!
                                {
                                    brushWorldPlanes    = brushWorldPlanes,
                                    otherBrushNodeIndex = allIntersectionLoops[intersectionSurfaceOffset + l1].brushNodeIndex,
                                    selfBrushNodeIndex  = intersectionSurface0.brushNodeIndex,
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
                    for (int b = 0; b < basePolygonEdges.Length; b++)
                    {
                        var edges = basePolygonEdges[b];
                        for (int i = 0; i < intersectingUniqueBrushIndices.Length; i++)
                        {
                            // TODO: use intersectionLoopBlobs/basePolygonBlobs as input + planes, output new blob
                            var intersectionJob = new FindBasePolygonPlaneIntersectionsJob()// TODO: we're reading AND writing to the same NativeList!?!?!
                            {
                                brushWorldPlanes    = brushWorldPlanes,
                                otherBrushNodeIndex = intersectingUniqueBrushIndices[i],
                                selfBrushNodeIndex  = brushNodeIndex0,
                                vertexSoup          = vertexSoup,
                                edges               = edges
                            };
                            intersectionJob.Run();
                        }
                    }

                    for (int s = 0; s < surfaceCount; s++)
                    {
                        var intersectionSurfaceCount    = intersectionSurfaceSegments[s].y;
                        var intersectionSurfaceOffset   = intersectionSurfaceSegments[s].x;
                        if (intersectionSurfaceCount == 0)
                            continue;

                        var edges = basePolygonEdges[s];
                        for (int l0 = 0; l0 < intersectionSurfaceCount; l0++)
                        {
                            var intersectionLoop = allIntersectionLoops[intersectionSurfaceOffset + l0];
                            // TODO: use intersectionLoopBlobs/basePolygonBlobs as input + planes, output new blob
                            var intersectionJob2 = new FindLoopVertexOverlapsJob
                            {
                                brushWorldPlanes    = brushWorldPlanes,
                                selfBrushNodeIndex  = intersectionLoop.brushNodeIndex,
                                vertexSoup          = vertexSoup,
                                otherEdges          = edges,
                                edges               = intersectionLoop.edges
                            };
                            intersectionJob2.Run();
                        }
                    } 

                    for (int i = allIntersectionLoops.Count - 1; i >= 0; i--)
                    {
                        // TODO: might not be necessary
                        var edges = allIntersectionLoops[i].edges;
                        var removeIdenticalIndicesEdgesJob = new RemoveIdenticalIndicesEdgesJob { edges = edges };
                        removeIdenticalIndicesEdgesJob.Run();
                    }

                    for (int i = basePolygonEdges.Length - 1; i >= 0; i--)
                    {
                        // TODO: might not be necessary
                        var edges = basePolygonEdges[i];
                        var removeIdenticalIndicesEdgesJob = new RemoveIdenticalIndicesEdgesJob2 { edges = edges };
                        removeIdenticalIndicesEdgesJob.Run();
                    }

                    // TODO: merge indices across multiple loops when vertices are identical

                    //intersectionData.StoreOutput
                    {

                        //***GET RID OF THIS***//
                        var outputLoops             = CSGManager.GetBrushInfo(brushNodeID0).brushOutputLoops;/*!!*/
                        outputLoops.Dispose();/*!!*/

                        {
                            var basePolygons = new List<Loop>(basePolygonBlob.Value.surfaces.Length);    /*OUTPUT*/
                            for (int s = 0; s < basePolygonBlob.Value.surfaces.Length; s++)
                            {
                                ref var input = ref basePolygonBlob.Value.surfaces[s];

                                var loop = new Loop()/*!!*/
                                {
                                    info    = input.surfaceInfo,
                                    holes   = new List<Loop>()/*!!*/
                                };

                                var edges = basePolygonEdges[s];
                                for (int e = 0; e < edges.Length; e++)
                                    loop.edges.Add(edges[e]);
                                basePolygons.Add(loop);
                            }
                            outputLoops.basePolygons    = basePolygons;/*!!*/ /*OUTPUT*/
                        }
                        var intersectionLoopLookup  = outputLoops.intersectionLoopLookup;/*!!*/ /*OUTPUT*/
                        vertexSoups.Add(brushNodeIndex0, vertexSoup);/*!!*/  /*OUTPUT*/
                        //***GET RID OF THIS***//

                        // Note: those above are only the INTERSECTING surfaces, below ALL surfaces
                        for (int k = 0; k < intersectingUniqueBrushIndices.Length; k++)
                        {
                            var brushNodeIndex  = intersectingUniqueBrushIndices[k];
                            var brushNodeID     = brushNodeIndex + 1;

                            Loop[] loops = null;/*!!*/
                            for (int s = 0; s < surfaceLoops.surfaces.Length; s++)
                            {
                                var intersectionLoops = surfaceLoops.surfaces[s];
                                if (intersectionLoops == null || intersectionLoops.Count == 0)
                                    continue;

                                for (int i = 0; i < intersectionLoops.Count; i++)
                                {
                                    if (intersectionLoops[i].info.brushNodeIndex == brushNodeIndex)
                                    {
                                        if (loops == null)
                                            loops = new Loop[surfaceLoops.surfaces.Length];

                                        loops[s] = intersectionLoops[i];
                                        break;
                                    }
                                }                                    
                            }
                            intersectionLoopLookup[brushNodeID] = loops;
                        }
                    }
                }
                intersectingUniqueBrushIndices.Dispose();
                basePolygonEdges.Dispose();
            }
        }


        public void Execute()
        {
            for (int index = 0; index < treeBrushes.Length; index++)
            {
                Execute(index);
            }
        }

    }
#endif
}
