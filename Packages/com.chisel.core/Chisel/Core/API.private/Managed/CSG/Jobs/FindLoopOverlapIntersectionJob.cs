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

    [BurstCompile(CompileSynchronously = true)]
    internal unsafe struct FindLoopOverlapIntersectionsJob : IJob
    { 
        [NoAlias, ReadOnly] public int                                                          brushNodeIndex;
        [NoAlias, ReadOnly] public NativeArray<BrushSurfacePair>                                intersectionLoopBlobsKeys;
        [NoAlias, ReadOnly] public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>> intersectionLoopBlobs;
        [NoAlias, ReadOnly] public BlobAssetReference<BasePolygonsBlob>                         basePolygonBlob;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>     brushWorldPlanes;
        

        [NoAlias] public VertexSoup                 vertexSoup;
        [NoAlias] public NativeListArray<Edge>      intersectionEdges;
        [NoAlias] public NativeListArray<Edge>      basePolygonEdges;
        [NoAlias] public NativeList<SurfaceInfo>    brushSurfaceInfos;

        static unsafe void CopyFrom(NativeListArray<Edge>.NativeList dst, BlobAssetReference<BrushIntersectionLoop> brushIntersectionLoop, VertexSoup vertexSoup)
        {
            ref var vertices = ref brushIntersectionLoop.Value.loopVertices;
            var srcIndices = stackalloc ushort[vertices.Length];
            vertexSoup.Reserve(vertices.Length);
            for (int j = 0; j < vertices.Length; j++)
                srcIndices[j] = vertexSoup.AddNoResize(vertices[j]);

            {
                dst.Capacity = vertices.Length;
                for (int j = 1; j < vertices.Length; j++)
                {
                    dst.Add(new Edge() { index1 = srcIndices[j - 1], index2 = srcIndices[j] });
                }
                dst.Add(new Edge() { index1 = srcIndices[vertices.Length - 1], index2 = srcIndices[0] });
            }
        }

        public struct Empty { };

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

        public unsafe void Execute()
        {
            var surfaceCount        = basePolygonBlob.Value.surfaces.Length;
            if (surfaceCount == 0)
                return;


            var brushIntersectionLoops      = new NativeList<BlobAssetReference<BrushIntersectionLoop>>(intersectionLoopBlobsKeys.Length, Allocator.Temp);
            var uniqueBrushIndicesHashMap   = new NativeHashMap<int, Empty>(intersectionLoopBlobsKeys.Length, Allocator.Temp);
            for (int k = 0; k < intersectionLoopBlobsKeys.Length; k++)
            {
                var item = intersectionLoopBlobsKeys[k];

                // TODO: get rid of this somehow
                if (item.brushNodeIndex0 != brushNodeIndex)
                    continue;

                var outputSurface = intersectionLoopBlobs[item];
                uniqueBrushIndicesHashMap.TryAdd(item.brushNodeIndex1, new Empty());
                brushIntersectionLoops.AddNoResize(outputSurface); /*OUTPUT*/
            }

            var uniqueBrushIndices = uniqueBrushIndicesHashMap.GetKeyArray(Allocator.Temp);
            uniqueBrushIndicesHashMap.Dispose();

            vertexSoup.AddUniqueVertices(ref basePolygonBlob.Value.vertices); /*OUTPUT*/

            var uniqueBrushIndexCount = uniqueBrushIndices.Length;
            if (uniqueBrushIndexCount == 0)
            {
                // If we don't have any intersection loops, just convert basePolygonBlob to loops and be done
                // TODO: should do this per surface!

                for (int s = 0; s < basePolygonBlob.Value.surfaces.Length; s++)
                {
                    ref var input = ref basePolygonBlob.Value.surfaces[s];

                    var edges = basePolygonEdges[s];
                    edges.Capacity = input.endEdgeIndex - input.startEdgeIndex;
                    for (int e = input.startEdgeIndex; e < input.endEdgeIndex; e++)
                        edges.Add(basePolygonBlob.Value.edges[e]);
                }

                for (int k = 0; k < brushIntersectionLoops.Length; k++)
                    brushSurfaceInfos.AddNoResize(brushIntersectionLoops[k].Value.surfaceInfo); /*OUTPUT*/

                uniqueBrushIndices.Dispose();
                brushIntersectionLoops.Dispose();
                return;
            }

            brushIntersectionLoops.Sort(new SortByBasePlaneIndex());
            for (int k = 0; k < brushIntersectionLoops.Length; k++)
                brushSurfaceInfos.AddNoResize(brushIntersectionLoops[k].Value.surfaceInfo); /*OUTPUT*/

            var intersectionSurfaceSegments = new NativeArray<int2>(surfaceCount, Allocator.Temp);
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
                        var intersectionBrushIndex0 = brushSurfaceInfos[intersectionSurfaceOffset + l0].brushNodeIndex;
                        var edges = intersectionEdges[intersectionSurfaceOffset + l0];
                        for (int l1 = 0; l1 < intersectionSurfaceCount; l1++)
                        {
                            if (l0 == l1)
                                continue;

                            var intersectionBrushIndex1 = brushSurfaceInfos[intersectionSurfaceOffset + l1].brushNodeIndex;

                            var intersectionJob = new FindLoopPlaneIntersectionsJob()
                            {
                                brushWorldPlanes    = brushWorldPlanes,
                                otherBrushNodeIndex = intersectionBrushIndex1,
                                selfBrushNodeIndex  = intersectionBrushIndex0,
                                vertexSoup          = vertexSoup,
                                edges               = edges
                            };
                            intersectionJob.Execute();

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
                    for (int i = 0; i < uniqueBrushIndices.Length; i++)
                    {
                        var intersectionJob = new FindBasePolygonPlaneIntersectionsJob()
                        {
                            brushWorldPlanes    = brushWorldPlanes,
                            otherBrushNodeIndex = uniqueBrushIndices[i],
                            selfBrushNodeIndex  = brushNodeIndex,
                            vertexSoup          = vertexSoup,
                            edges               = edges
                        };
                        intersectionJob.Execute();
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
                        var intersectionBrushIndex  = brushSurfaceInfos[intersectionSurfaceOffset + l0].brushNodeIndex;
                        var in_edges                = intersectionEdges[intersectionSurfaceOffset + l0];
                        var intersectionJob2 = new FindLoopVertexOverlapsJob
                        {
                            brushWorldPlanes    = brushWorldPlanes,
                            selfBrushNodeIndex  = intersectionBrushIndex,
                            vertexSoup          = vertexSoup,
                            otherEdges          = bp_edges,
                            edges               = in_edges
                        };
                        intersectionJob2.Execute();
                    }
                } 

                for (int i = 0; i < brushIntersectionLoops.Length; i++)
                {
                    // TODO: might not be necessary
                    var edges = intersectionEdges[i];
                    var removeIdenticalIndicesEdgesJob = new RemoveIdenticalIndicesEdgesJob { edges = edges };
                    removeIdenticalIndicesEdgesJob.Execute();
                }

                for (int i = 0; i < basePolygonEdges.Length; i++)
                {
                    // TODO: might not be necessary
                    var edges = basePolygonEdges[i];
                    var removeIdenticalIndicesEdgesJob = new RemoveIdenticalIndicesEdgesJob { edges = edges };
                    removeIdenticalIndicesEdgesJob.Execute();
                }


                // TODO: merge indices across multiple loops when vertices are identical
            }
            intersectionSurfaceSegments.Dispose();
            brushIntersectionLoops.Dispose();
            uniqueBrushIndices.Dispose();
        }
        
    }
#endif
}
