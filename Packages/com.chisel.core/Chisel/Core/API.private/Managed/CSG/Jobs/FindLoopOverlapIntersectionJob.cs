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
    internal unsafe struct FindLoopOverlapIntersectionsJob : IJobParallelFor
    { 
        [NoAlias, ReadOnly] public NativeArray<int>                                             treeBrushIndices;
        [NoAlias, ReadOnly] public NativeArray<BlobAssetReference<BrushIntersectionLoops>>      intersectionLoopBlobs;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>>     basePolygonBlobs;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>     brushWorldPlanes;
        
        //[NoAlias] public VertexSoup                         vertexSoup;
        //[NoAlias] public NativeListArray<Edge>              basePolygonEdges;
        //[NoAlias] public NativeList<SurfaceInfo>            basePolygonSurfaceInfos;
        //[NoAlias] public NativeListArray<Edge>              intersectionEdges;
        //[NoAlias] public NativeList<SurfaceInfo>            intersectionSurfaceInfos;

        [NoAlias, WriteOnly] public NativeStream.Writer     output;

        public struct Empty { };
        /*
        struct SortByBasePlaneIndex : IComparer<BlobAssetReference<BrushIntersectionLoop>>
        {
            public int Compare(BlobAssetReference<BrushIntersectionLoop> x, BlobAssetReference<BrushIntersectionLoop> y)
            {
                if (!x.IsCreated) return y.IsCreated ? 1 : 0;
                if (!y.IsCreated) return -1;

                ref var vx = ref x.Value;
                ref var vy = ref y.Value;

                var diff = vx.surfaceInfo.basePlaneIndex - vy.surfaceInfo.basePlaneIndex;
                if (diff != 0)
                    return diff;

                diff = vx.surfaceInfo.brushNodeIndex - vy.surfaceInfo.brushNodeIndex;
                if (diff != 0)
                    return diff;

                return 0;
            }
        }
        */
        public int CompareSortByBasePlaneIndex(int2 x, int2 y)
        {
            ref var vx = ref intersectionLoopBlobs[x.x].Value.loops[x.y];
            ref var vy = ref intersectionLoopBlobs[y.x].Value.loops[y.y];

            var diff = vx.surfaceInfo.basePlaneIndex - vy.surfaceInfo.basePlaneIndex;
            if (diff != 0)
                return diff;

            diff = vx.surfaceInfo.brushNodeIndex - vy.surfaceInfo.brushNodeIndex;
            if (diff != 0)
                return diff;
            return 0;
        }

        static unsafe void CopyFrom(NativeListArray<Edge>.NativeList dst, ref BrushIntersectionLoop brushIntersectionLoop, VertexSoup vertexSoup)
        {
            ref var vertices = ref brushIntersectionLoop.loopVertices;
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

        public unsafe void Execute(int index)
        {
            var brushNodeIndex      = treeBrushIndices[index];

            var basePolygonBlob     = basePolygonBlobs[brushNodeIndex];
            ref var basePolygonBlobValue = ref basePolygonBlob.Value;

            var surfaceCount        = basePolygonBlobValue.surfaces.Length;
            if (surfaceCount == 0)
                return;
            
            var basePolygonSurfaceInfos     = new NativeList<SurfaceInfo>(0, Allocator.Temp);
            var basePolygonEdges            = new NativeListArray<Edge>(0, Allocator.Temp);
            var intersectionSurfaceInfos    = new NativeList<SurfaceInfo>(0, Allocator.Temp);
            var intersectionEdges           = new NativeListArray<Edge>(0, Allocator.Temp);
            var vertexSoup                  = new VertexSoup(2048, Allocator.Temp);

            basePolygonEdges.ResizeExact(surfaceCount);
            basePolygonSurfaceInfos.ResizeUninitialized(surfaceCount);



            // ***********************
            // TODO: get rid of this somehow
            var brushIntersectionLoops      = new NativeList<int2>(intersectionLoopBlobs.Length, Allocator.Temp);
            var uniqueBrushIndicesHashMap   = new NativeHashMap<int, Empty>(intersectionLoopBlobs.Length, Allocator.Temp);
            for (int k = 0; k < intersectionLoopBlobs.Length; k++)
            {
                ref var loops           = ref intersectionLoopBlobs[k].Value.loops;
                for (int n = 0; n < loops.Length; n++)
                { 
                    ref var outputSurface   = ref loops[n];
                    ref var pair            = ref outputSurface.pair;

                    // TODO: get rid of this somehow
                    if (pair.brushNodeIndex0 != brushNodeIndex)
                        continue;

                    uniqueBrushIndicesHashMap.TryAdd(pair.brushNodeIndex1, new Empty());
                    brushIntersectionLoops.AddNoResize(new int2(k, n)); /*OUTPUT*/
                }
            }
            // ***********************



            var uniqueBrushIndices = uniqueBrushIndicesHashMap.GetKeyArray(Allocator.Temp);
            uniqueBrushIndicesHashMap.Dispose();

            vertexSoup.AddUniqueVertices(ref basePolygonBlobValue.vertices); /*OUTPUT*/

            var uniqueBrushIndexCount = uniqueBrushIndices.Length;
            if (uniqueBrushIndexCount == 0)
            {
                // If we don't have any intersection loops, just convert basePolygonBlob to loops and be done
                // TODO: should do this per surface!

                for (int s = 0; s < basePolygonBlobValue.surfaces.Length; s++)
                {
                    ref var input = ref basePolygonBlobValue.surfaces[s];

                    var edges = basePolygonEdges[s];
                    edges.Capacity = input.endEdgeIndex - input.startEdgeIndex;
                    for (int e = input.startEdgeIndex; e < input.endEdgeIndex; e++)
                        edges.Add(basePolygonBlobValue.edges[e]);

                    basePolygonSurfaceInfos[s] = basePolygonBlobValue.surfaces[s].surfaceInfo;
                }

                uniqueBrushIndices.Dispose();
                brushIntersectionLoops.Dispose();
                return;
            }

            intersectionEdges.ResizeExact(brushIntersectionLoops.Length);
            for (int i = 0; i < brushIntersectionLoops.Length - 1; i++)
            {
                for (int j = i + 1; j < brushIntersectionLoops.Length; j++)
                {
                    if (CompareSortByBasePlaneIndex(brushIntersectionLoops[i], brushIntersectionLoops[j]) > 0)
                    {
                        var t = brushIntersectionLoops[i];
                        brushIntersectionLoops[i] = brushIntersectionLoops[j];
                        brushIntersectionLoops[j] = t;
                    }
                }
            }

            var intersectionSurfaceSegments = stackalloc int2[surfaceCount];
            {
                {
                    for (int s = 0; s < basePolygonBlobValue.surfaces.Length; s++)
                    {
                        ref var input = ref basePolygonBlobValue.surfaces[s];

                        var edges = basePolygonEdges[s];
                        edges.Capacity = input.endEdgeIndex - input.startEdgeIndex;
                        for (int e = input.startEdgeIndex; e < input.endEdgeIndex; e++)
                            edges.Add(basePolygonBlobValue.edges[e]);

                        basePolygonSurfaceInfos[s] = basePolygonBlobValue.surfaces[s].surfaceInfo;
                    }

                    { 
                        int prevBasePlaneIndex = 0;
                        int startIndex = 0;
                        for (int l = 0; l < brushIntersectionLoops.Length; l++)
                        {
                            var brushIntersectionIndex      = brushIntersectionLoops[l];
                            ref var brushIntersectionLoop   = ref intersectionLoopBlobs[brushIntersectionIndex.x].Value.loops[brushIntersectionIndex.y];
                            ref var surfaceInfo             = ref brushIntersectionLoop.surfaceInfo;
                            var basePlaneIndex = surfaceInfo.basePlaneIndex;
                            if (prevBasePlaneIndex != basePlaneIndex)
                            {
                                intersectionSurfaceSegments[prevBasePlaneIndex] = new int2(startIndex, l - startIndex);
                                startIndex = l;
                                for (int s = prevBasePlaneIndex + 1; s < basePlaneIndex; s++)
                                    intersectionSurfaceSegments[s] = new int2(startIndex, 0);
                                prevBasePlaneIndex = basePlaneIndex;
                            }
                            CopyFrom(intersectionEdges[l], ref brushIntersectionLoop, vertexSoup);
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
                        var intersectionIndex0      = brushIntersectionLoops[intersectionSurfaceOffset + l0];
                        ref var intersection0       = ref intersectionLoopBlobs[intersectionIndex0.x].Value.loops[intersectionIndex0.y];
                        var intersectionBrushIndex0 = intersection0.surfaceInfo.brushNodeIndex;
                        var edges = intersectionEdges[intersectionSurfaceOffset + l0];
                        for (int l1 = 0; l1 < intersectionSurfaceCount; l1++)
                        {
                            if (l0 == l1)
                                continue;
                            
                            var intersectionIndex1      = brushIntersectionLoops[intersectionSurfaceOffset + l1];
                            ref var intersection1       = ref intersectionLoopBlobs[intersectionIndex1.x].Value.loops[intersectionIndex1.y];
                            var intersectionBrushIndex1 = intersection1.surfaceInfo.brushNodeIndex;

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
                        var intersectionIndex       = brushIntersectionLoops[intersectionSurfaceOffset + l0];
                        ref var intersection        = ref intersectionLoopBlobs[intersectionIndex.x].Value.loops[intersectionIndex.y];
                        var intersectionBrushIndex  = intersection.surfaceInfo.brushNodeIndex;
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

                for (int i = 0; i < intersectionEdges.Length; i++)
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

            intersectionSurfaceInfos.Capacity = brushIntersectionLoops.Length;
            for (int k = 0; k < brushIntersectionLoops.Length; k++)
            {
                var intersectionIndex = brushIntersectionLoops[k];
                ref var intersection = ref intersectionLoopBlobs[intersectionIndex.x].Value.loops[intersectionIndex.y];
                intersectionSurfaceInfos.AddNoResize(intersection.surfaceInfo); /*OUTPUT*/
            }


            output.BeginForEachIndex(index);
            output.Write(brushNodeIndex);
            output.Write(surfaceCount);
            output.Write(vertexSoup.Length);
            for (int l = 0; l < vertexSoup.Length; l++)
                output.Write(vertexSoup[l]);
            
            output.Write(basePolygonEdges.Length);
            for (int l = 0; l < basePolygonEdges.Length; l++)
            {
                output.Write(basePolygonSurfaceInfos[l]);
                var edges = basePolygonEdges[l].AsArray();
                output.Write(edges.Length);
                for (int e = 0; e < edges.Length; e++)
                    output.Write(edges[e]);
            }

            output.Write(intersectionEdges.Length);
            for (int l = 0; l < intersectionEdges.Length; l++)
            {
                output.Write(intersectionSurfaceInfos[l]);
                var edges = intersectionEdges[l].AsArray();
                output.Write(edges.Length);
                for (int e = 0; e < edges.Length; e++)
                    output.Write(edges[e]);
            }
            output.EndForEachIndex();


            //intersectionSurfaceSegments.Dispose();
            brushIntersectionLoops.Dispose();
            uniqueBrushIndices.Dispose();
        }
        
    }
#endif
}
