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

    internal unsafe struct FindLoopOverlapIntersectionsJob : IJobParallelFor
    {
        [NoAlias, ReadOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>> intersectionLoopBlobs;
        [NoAlias, ReadOnly] public NativeArray<BrushSurfacePair>                                 intersectionLoopBlobsKeys;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>>      basePolygonBlobs;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>      brushWorldPlanes;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>         brushMeshBlobs;
        [NoAlias, ReadOnly] public NativeArray<int>                                              treeBrushes;
        [NoAlias, ReadOnly] public NativeArray<int>                                              brushMeshInstanceIDs;

        // TODO: get rid of this somehow
        public Dictionary<int, VertexSoup>                                  vertexSoups; /*!!*/

        struct Empty { };

        struct SortByBasePlaneIndex : IComparer<BlobAssetReference<BrushIntersectionLoop>>
        {
            public int Compare(BlobAssetReference<BrushIntersectionLoop> x, BlobAssetReference<BrushIntersectionLoop> y)
            {
                if (!x.IsCreated) return y.IsCreated ? 1 : 0;
                if (!y.IsCreated) return -1;

                var diff = x.Value.surfaceInfo.basePlaneIndex - y.Value.surfaceInfo.basePlaneIndex;
                if (diff != 0)
                    return diff;

                diff = x.Value.surfaceInfo.brushNodeIndex - y.Value.surfaceInfo.brushNodeIndex;
                if (diff != 0)
                    return diff;

                return 0;
            }
        }

        public unsafe void CopyFrom(NativeListArray<Edge>.NativeList dst, BlobAssetReference<BrushIntersectionLoop> brushIntersectionLoop, VertexSoup vertexSoup)
        {
            ref var vertices = ref brushIntersectionLoop.Value.loopVertices;
            var srcIndices = new ushort[vertices.Length];
            vertexSoup.Reserve(srcIndices.Length);
            for (int j = 0; j < srcIndices.Length; j++)
                srcIndices[j] = vertexSoup.AddNoResize(vertices[j]);

            var surfaceInfo = brushIntersectionLoop.Value.surfaceInfo;
            {
                dst.Capacity = srcIndices.Length;
                for (int j = 1; j < srcIndices.Length; j++)
                {
                    dst.Add(new Edge() { index1 = srcIndices[j - 1], index2 = srcIndices[j] });
                }
                dst.Add(new Edge() { index1 = srcIndices[srcIndices.Length - 1], index2 = srcIndices[0] });
            }
        }

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
            for (int k = 0; k < intersectionLoopBlobsKeys.Length; k++)
            {
                var item = intersectionLoopBlobsKeys[k];
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

                var brushIntersectionLoops      = new NativeList<BlobAssetReference<BrushIntersectionLoop>>(intersectionLoopBlobsKeys.Length, Allocator.TempJob);
                var vertexSoup                  = new VertexSoup(ref basePolygonBlob.Value.vertices);       /*OUTPUT*/
                var intersectionSurfaceSegments = new NativeArray<int2>(surfaceCount, Allocator.TempJob);
                var basePolygonEdges            = new NativeListArray<Edge>(surfaceCount, 16, Allocator.TempJob);
                NativeListArray<Edge>   intersectionEdges;
                NativeArray<int>        intersectionBrushIndices;
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

                        for (int k = 0; k < intersectionLoopBlobsKeys.Length; k++)
                        {
                            var item = intersectionLoopBlobsKeys[k];

                            // TODO: get rid of this somehow
                            if (item.brushNodeIndex0 != brushNodeIndex0)
                                continue;

                            var outputSurface = intersectionLoopBlobs[item];
                            brushIntersectionLoops.Add(outputSurface);
                        }

                        brushIntersectionLoops.Sort(new SortByBasePlaneIndex());

                        intersectionEdges           = new NativeListArray<Edge>(brushIntersectionLoops.Length, 16, Allocator.TempJob);
                        intersectionBrushIndices    = new NativeArray<int>(brushIntersectionLoops.Length, Allocator.TempJob);

                        { 
                            int prevBasePlaneIndex = 0;
                            int startIndex = 0;
                            for (int l = 0; l < brushIntersectionLoops.Length; l++)
                            {
                                var brushIntersectionLoop = brushIntersectionLoops[l];
                                ref var surfaceInfo = ref brushIntersectionLoop.Value.surfaceInfo;
                                var basePlaneIndex = surfaceInfo.basePlaneIndex;
                                if (prevBasePlaneIndex != basePlaneIndex)
                                {
                                    intersectionSurfaceSegments[prevBasePlaneIndex] = new int2(startIndex, l - startIndex);
                                    startIndex = l;
                                    for (int s = prevBasePlaneIndex + 1; s < basePlaneIndex; s++)
                                        intersectionSurfaceSegments[s] = new int2(startIndex, 0);
                                    prevBasePlaneIndex = basePlaneIndex;
                                }
                                intersectionBrushIndices[l] = surfaceInfo.brushNodeIndex;
                                CopyFrom(intersectionEdges[l], brushIntersectionLoop, vertexSoup);
                            }
                            {
                                intersectionSurfaceSegments[prevBasePlaneIndex] = new int2(startIndex, brushIntersectionLoops.Length - startIndex);
                                startIndex = brushIntersectionLoops.Length;
                                for (int s = prevBasePlaneIndex + 1; s < surfaceCount; s++)
                                    intersectionSurfaceSegments[s] = new int2(startIndex, 0);
                            }
                        }
                    }

                    for (int s = 0; s < surfaceCount; s++)
                    {
                        var intersectionSurfaceCount    = intersectionSurfaceSegments[s].y;
                        var intersectionSurfaceOffset   = intersectionSurfaceSegments[s].x;
                        for (int l0 = intersectionSurfaceCount - 1; l0 >= 0; l0--)
                        {
                            var intersectionBrushIndex0 = intersectionBrushIndices[intersectionSurfaceOffset + l0];
                            var edges = intersectionEdges[intersectionSurfaceOffset + l0];
                            for (int l1 = 0; l1 < intersectionSurfaceCount; l1++)
                            {
                                if (l0 == l1)
                                    continue;

                                var intersectionBrushIndex1 = intersectionBrushIndices[intersectionSurfaceOffset + l1];

                                var intersectionJob = new FindLoopPlaneIntersectionsJob()
                                {
                                    brushWorldPlanes    = brushWorldPlanes,
                                    otherBrushNodeIndex = intersectionBrushIndex1,
                                    selfBrushNodeIndex  = intersectionBrushIndex0,
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
                            var intersectionJob = new FindBasePolygonPlaneIntersectionsJob()
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

                        var bp_edges = basePolygonEdges[s];
                        for (int l0 = 0; l0 < intersectionSurfaceCount; l0++)
                        {
                            var intersectionBrushIndex  = intersectionBrushIndices[intersectionSurfaceOffset + l0];
                            var in_edges                = intersectionEdges[intersectionSurfaceOffset + l0];
                            var intersectionJob2 = new FindLoopVertexOverlapsJob
                            {
                                brushWorldPlanes    = brushWorldPlanes,
                                selfBrushNodeIndex  = intersectionBrushIndex,
                                vertexSoup          = vertexSoup,
                                otherEdges          = bp_edges,
                                edges               = in_edges
                            };
                            intersectionJob2.Run();
                        }
                    } 

                    for (int i = intersectionEdges.Length - 1; i >= 0; i--)
                    {
                        // TODO: might not be necessary
                        var edges = intersectionEdges[i];
                        var removeIdenticalIndicesEdgesJob = new RemoveIdenticalIndicesEdgesJob2 { edges = edges };
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
                        vertexSoups.Add(brushNodeIndex0, vertexSoup);/*!!*/  /*OUTPUT*/

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

                        // Note: those above are only the INTERSECTING surfaces, below ALL surfaces
                        var intersectionLoopLookup  = outputLoops.intersectionLoopLookup;/*!!*/ /*OUTPUT*/
                        for (int i = 0; i < brushIntersectionLoops.Length; i++)
                        {
                            var outputSurface = brushIntersectionLoops[i];
                            ref var surfaceInfo = ref outputSurface.Value.surfaceInfo;
                            var newLoop = new Loop(intersectionEdges[i], outputSurface.Value.surfaceInfo);/*!!*/ /* vertexSoup -> OUTPUT*/

                            var brushNodeIndex  = surfaceInfo.brushNodeIndex;
                            var brushNodeID     = brushNodeIndex + 1;

                            if (!intersectionLoopLookup.TryGetValue(brushNodeID, out Loop[] loops))
                            {
                                loops = new Loop[surfaceCount];
                                intersectionLoopLookup[brushNodeID] = loops;
                            }

                            loops[outputSurface.Value.surfaceInfo.basePlaneIndex] = newLoop;
                        }
                        //***GET RID OF THIS***//
                    }
                }
                intersectingUniqueBrushIndices.Dispose();
                basePolygonEdges.Dispose();
                intersectionEdges.Dispose();
                intersectionBrushIndices.Dispose();
                brushIntersectionLoops.Dispose();
                intersectionSurfaceSegments.Dispose();
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
