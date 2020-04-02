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
        public int2                 segment;
        public NativeList<Edge>     edges;
    }

    internal struct IntersectionBasePolygon
    {
        public int                  surfaceIndex;
        public int                  brushNodeIndex;
        public NativeList<Edge>     edges;
        public int2                 segment;
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

        public NativeArray<float4> GetPlanes(in NativeList<float4> allWorldSpacePlanes, int2 segment)
        {
            return allWorldSpacePlanes.AsArray().GetSubArray(segment.x, segment.y);
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

            var uniqueBrushIndices  = new NativeHashMap<int, int2>(intersectionLoopBlobsKeys.Length, Allocator.TempJob);

            // TODO: get rid of this somehow
            foreach (var item in intersectionLoopBlobsKeys)
            {
                if (item.brushNodeIndex0 != brushNodeIndex0)
                    continue;
                    
                uniqueBrushIndices[item.brushNodeIndex1] = new int2();
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

                var vertexSoup          = new VertexSoup(ref basePolygonBlob.Value.vertices);

                var surfaceLoops        = new SurfaceLoops(surfaceCount);/*!!*/

                // TODO: get rid of this somehow
                foreach (var item in intersectionLoopBlobsKeys)
                {
                    if (item.brushNodeIndex0 != brushNodeIndex0)
                        continue;
                    
                    var outputSurface   = intersectionLoopBlobs[item];
                    var newLoop         = new Loop(outputSurface, vertexSoup);/*!!*/
                    surfaceLoops.surfaces[item.basePlaneIndex].Add(newLoop);
                }

                var intersectionSurfaces        = new List<int>[surfaceCount]; /*!!*/
                for (int i = 0; i < intersectionSurfaces.Length; i++)
                    intersectionSurfaces[i]     = new List<int>(16);/*!!*/

                var basePolygonLoops            = new IntersectionBasePolygon[surfaceCount];/*!!*/
                var allIntersectionLoops        = new List<IntersectionLoop>();/*!!*/

                var allWorldSpacePlanes         = new NativeList<float4>(surfaceCount, Allocator.Persistent);
                int2 worldSpacePlanes0Segment;
                    
                {
                    {
                        var uniqueBrushIndicesKeys = uniqueBrushIndices.GetKeyArray(Allocator.Temp);
                        {
                            ref var meshPlanes  = ref brushWorldPlanes[brushNodeIndex0].Value.worldPlanes;
                            var startIndex      = allWorldSpacePlanes.Length;
                            allWorldSpacePlanes.AddRange(meshPlanes.GetUnsafePtr(), meshPlanes.Length);
                            worldSpacePlanes0Segment = new int2(startIndex, allWorldSpacePlanes.Length - startIndex);
                            uniqueBrushIndices[brushNodeIndex0] = worldSpacePlanes0Segment;
                        }

                        for (int i = 0; i < uniqueBrushIndicesKeys.Length; i++)
                        {
                            var brushNodeIndex  = uniqueBrushIndicesKeys[i];
                            ref var meshPlanes  = ref brushWorldPlanes[brushNodeIndex].Value.worldPlanes;
                            var startIndex      = allWorldSpacePlanes.Length;
                            allWorldSpacePlanes.AddRange(meshPlanes.GetUnsafePtr(), meshPlanes.Length);
                            var segment = new int2(startIndex, allWorldSpacePlanes.Length - startIndex);
                            uniqueBrushIndices[brushNodeIndex] = segment;
                        }
                        uniqueBrushIndicesKeys.Dispose();

                        for (int s = 0; s < basePolygons.Count; s++)
                        {
                            var loop = basePolygons[s];
                            var intersectionLoop = new IntersectionBasePolygon()
                            {
                                segment         = worldSpacePlanes0Segment,
                                edges           = loop.edges,
                                surfaceIndex    = s,
                                brushNodeIndex  = brushNodeIndex0
                            };

                            basePolygonLoops[s] = intersectionLoop;                
                        }

                        for (int s = 0; s < surfaceLoops.surfaces.Length; s++)
                        {
                            var intersectionSurface = intersectionSurfaces[s];
                            var loops               = surfaceLoops.surfaces[s];
                            if (loops == null ||
                                loops.Count == 0)
                            {
                                continue;
                            }

                            for (int l = 0; l < loops.Count; l++)
                            {
                                var loop                = loops[l];
                                var brushNodeIndex1     = loop.info.brushNodeIndex;

                                // At this point it should not be possible to have more loop from the same intersecting brush since 
                                // we only support convex brushes. 
                                // Later on, however, when we start intersecting loops with each other, we can end up with multiple fragments.

                                var segment          = uniqueBrushIndices[brushNodeIndex1];
                                var intersectionLoop = new IntersectionLoop()
                                {
                                    segment         = segment,
                                    edges           = loop.edges,
                                    surfaceIndex    = s,
                                    brushNodeIndex  = brushNodeIndex1
                                }; 

                                // We add the intersection loop of this particular brush with our own brush
                                intersectionSurface.Add(allIntersectionLoops.Count);
                                allIntersectionLoops.Add(intersectionLoop);
                            }
                        }
                    }

                    for (int s = 0; s < intersectionSurfaces.Length; s++)
                    {
                        var intersectionSurfaceList = intersectionSurfaces[s];
                        for (int l0 = intersectionSurfaceList.Count - 1; l0 >= 0; l0--)
                        {
                            var intersectionSurface0 = allIntersectionLoops[intersectionSurfaceList[l0]];
                            var edges = intersectionSurface0.edges;
                            for (int l1 = 0; l1 < intersectionSurfaceList.Count; l1++)
                            {
                                if (l0 == l1)
                                    continue;

                                // TODO: use intersectionLoopBlobs/basePolygonBlobs as input + planes, output new blob
                                var intersectionJob = new FindLoopPlaneIntersectionsJob()// TODO: we're reading AND writing to the same NativeList!?!?!
                                {
                                    // TODO: use worldplaneblobs instead, remove all need to create allWorldSpacePlanes/segments
                                    allWorldSpacePlanes = allWorldSpacePlanes,
                                    otherPlanesSegment  = allIntersectionLoops[intersectionSurfaceList[l1]].segment,
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
                    var brushPlaneSegments = uniqueBrushIndices.GetValueArray(Allocator.Temp);
                    for (int b = 0; b < basePolygonLoops.Length; b++)
                    {
                        var edges = basePolygonLoops[b].edges;
                        foreach (var brushPlaneSegment in brushPlaneSegments)
                        {
                            // TODO: use intersectionLoopBlobs/basePolygonBlobs as input + planes, output new blob
                            var intersectionJob = new FindBasePolygonPlaneIntersectionsJob()// TODO: we're reading AND writing to the same NativeList!?!?!
                            {
                                allWorldSpacePlanes = allWorldSpacePlanes,
                                otherPlanesSegment  = brushPlaneSegment,
                                selfPlanesSegment   = worldSpacePlanes0Segment,
                                vertexSoup          = vertexSoup,
                                edges               = edges
                            };
                            intersectionJob.Run();
                        }
                    }
                    brushPlaneSegments.Dispose();

                    for (int s = 0; s < surfaceCount; s++)
                    {
                        var intersectionSurfaceList = intersectionSurfaces[s];
                        if (intersectionSurfaceList.Count == 0)
                            continue;

                        var basePolygonEdges = basePolygonLoops[s].edges;
                        for (int l0 = 0; l0 < intersectionSurfaceList.Count; l0++)
                        {
                            var intersectionLoop = allIntersectionLoops[intersectionSurfaceList[l0]];
                            // TODO: use intersectionLoopBlobs/basePolygonBlobs as input + planes, output new blob
                            var intersectionJob2 = new FindLoopVertexOverlapsJob
                            {
                                brushWorldPlanes    = brushWorldPlanes,
                                selfBrushNodeIndex  = intersectionLoop.brushNodeIndex,
                                vertexSoup          = vertexSoup,
                                otherEdges          = basePolygonEdges,
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

                    for (int i = basePolygonLoops.Length - 1; i >= 0; i--)
                    {
                        // TODO: might not be necessary
                        var edges = basePolygonLoops[i].edges;
                        var removeIdenticalIndicesEdgesJob = new RemoveIdenticalIndicesEdgesJob { edges = edges };
                        removeIdenticalIndicesEdgesJob.Run();
                    }

                    // TODO: merge indices across multiple loops when vertices are identical

                    //intersectionData.StoreOutput
                    {

                        //***GET RID OF THIS***//
                        var outputLoops             = CSGManager.GetBrushInfo(brushNodeID0).brushOutputLoops;/*!!*/
                        outputLoops.Dispose();/*!!*/
                        outputLoops.basePolygons    = basePolygons;/*!!*/ /*OUTPUT*/
                        var intersectionLoopLookup  = outputLoops.intersectionLoopLookup;/*!!*/ /*OUTPUT*/
                        vertexSoups.Add(brushNodeIndex0, vertexSoup);/*!!*/  /*OUTPUT*/
                        //***GET RID OF THIS***//

                        // Note: those above are only the INTERSECTING surfaces, below ALL surfaces
                        var uniqueBrushIndicesKeys = uniqueBrushIndices.GetKeyArray(Allocator.Temp);                        
                        for (int k = 0; k < uniqueBrushIndicesKeys.Length; k++)
                        {
                            var brushNodeIndex  = uniqueBrushIndicesKeys[k];
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
                        uniqueBrushIndicesKeys.Dispose();
                    }
                }
                allWorldSpacePlanes.Dispose();
            }
            uniqueBrushIndices.Dispose();
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
