using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Entities;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Profiler = UnityEngine.Profiling.Profiler;
using System.Runtime.CompilerServices;
using Unity.Burst;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
    [BurstCompile(CompileSynchronously = true)]
    unsafe struct PerformCSGJob : IJob
    {
        [NoAlias, ReadOnly] public int                                                              brushNodeIndex;
        [NoAlias, ReadOnly] public VertexSoup                                                       brushVertices;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<RoutingTable>>             routingTableLookup;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>         brushWorldPlanes;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>    brushesTouchedByBrushes;

        [NoAlias, ReadOnly] public NativeList<SurfaceInfo>  intersectionSurfaceInfos;
        [NoAlias, ReadOnly] public NativeListArray<Edge>    intersectionEdges;

        [NoAlias] public NativeList<SurfaceInfo>            basePolygonSurfaceInfos;    // <-- should be readonly?
        [NoAlias] public NativeListArray<Edge>              basePolygonEdges;           // <-- should be readonly?

        [NoAlias] public NativeListArray<int>               surfaceLoopIndices;
        [NoAlias] public NativeList<SurfaceInfo>            allInfos;
        [NoAlias] public NativeListArray<Edge>              allEdges;

        struct Empty {}

        [BurstDiscard]
        private static void NotUniqueEdgeException()
        {
            throw new Exception("Edge is not unique");
        }

        public static bool AddEdges(NativeListArray<Edge>.NativeList edges, NativeListArray<Edge>.NativeList addEdges, bool removeDuplicates = false)
        {
            if (addEdges.Length == 0)
                return false;

            var uniqueEdges = new NativeHashMap<Edge, Empty>(addEdges.Length, Allocator.Temp);
            uniqueEdges.Clear();
            for (int e = 0; e < addEdges.Length; e++)
            {
                if (!uniqueEdges.TryAdd(addEdges[e], new Empty()))
                    NotUniqueEdgeException();
            }

            if (removeDuplicates)
            {
                for (int e = edges.Length - 1; e >= 0; e--)
                {
                    if (uniqueEdges.Remove(edges[e]))
                    {
                        edges.RemoveAtSwapBack(e);
                    }
                }
            }

            bool duplicates = false;

            var values = uniqueEdges.GetKeyArray(Allocator.Temp);
            uniqueEdges.Dispose();
            for (int v= 0;v<values.Length;v++)
            {
                var index = IndexOf(edges, values[v], out bool _);
                if (index != -1)
                {
                    //Debug.Log($"Duplicate edge {inverted}  {values[v].index1}/{values[v].index2} {edges[index].index1}/{edges[index].index2}");
                    duplicates = true;
                    continue;
                }
                edges.Add(values[v]);
            }
            values.Dispose();

            return duplicates;
        }

        public static int IndexOf(NativeListArray<Edge>.NativeList edges, Edge edge, out bool inverted)
        {
            //var builder = new System.Text.StringBuilder();
            for (int e = 0; e < edges.Length; e++)
            {
                //builder.AppendLine($"{e}/{edges.Count}: {edges[e]} {edge}");
                if (edges[e].index1 == edge.index1 && edges[e].index2 == edge.index2) { inverted = false; return e; }
                if (edges[e].index1 == edge.index2 && edges[e].index2 == edge.index1) { inverted = true;  return e; }
            }
            //Debug.Log(builder.ToString());
            inverted = false;
            return -1;
        }

        public void IntersectLoopsJob(int surfaceLoopIndex, NativeListArray<Edge>.NativeList intersectionLoop, CategoryGroupIndex intersectionCategory, SurfaceInfo intersectionInfo, NativeListArray<int>.NativeList loopIndices, NativeListArray<int> holeIndices)
        {
            var currentLoopEdges    = allEdges[surfaceLoopIndex];
            var currentInfo         = allInfos[surfaceLoopIndex];
            var currentHoleIndices  = holeIndices[surfaceLoopIndex];

            // It might look like we could just set the interiorCategory of brush_intersection here, and let all other cut loops copy from it below,
            // but the same brush_intersection might be used by another categorized_loop and then we'd try to reroute it again, which wouldn't work
            //brush_intersection.interiorCategory = newHoleCategory;

            var outEdges = new NativeList<Edge>(math.max(intersectionLoop.Length, currentLoopEdges.Length), Allocator.Temp);

            var result = OperationResult.Fail;
            var intersectEdgesJob = new IntersectEdgesJob
            {
                vertices = brushVertices,
                edges1 = intersectionLoop,
                edges2 = currentLoopEdges,
                worldPlanes1 = brushWorldPlanes[intersectionInfo.brushNodeIndex],
                worldPlanes2 = brushWorldPlanes[currentInfo.brushNodeIndex],

                result = &result,
                outEdges = outEdges
            };
            intersectEdgesJob.Execute();

            // *somehow* put whats below in a job

            if (result == OperationResult.Outside ||
                result == OperationResult.Fail)
            {
                outEdges.Dispose();
            } else
            if (result == OperationResult.Polygon2InsidePolygon1)
            {
                // This new piece overrides the current loop
                currentLoopEdges.Clear();
                currentLoopEdges.AddRange(outEdges);
                currentInfo.interiorCategory = intersectionCategory;
                allInfos[surfaceLoopIndex] = currentInfo;
                outEdges.Dispose();
            } else
            if (result == OperationResult.Overlapping)
            {
                outEdges.Dispose();
                currentInfo.interiorCategory = intersectionCategory;
                allInfos[surfaceLoopIndex] = currentInfo;
            } else
            {
                // FIXME: when brush_intersection and categorized_loop are grazing each other, 
                //          technically we cut it but we shouldn't be creating it as a separate polygon + hole (bug7)

                // the output of cutting operations are both holes for the original polygon (categorized_loop)
                // and new polygons on the surface of the brush that need to be categorized
                intersectionInfo.interiorCategory = intersectionCategory;
                var intersectedHoleIndices = new NativeList<int>(Allocator.Temp);

                // the output of cutting operations are both holes for the original polygon (categorized_loop)
                // and new polygons on the surface of the brush that need to be categorized
                if (currentHoleIndices.Capacity < currentHoleIndices.Length + 1)
                    currentHoleIndices.Capacity = currentHoleIndices.Length + 1;

                if (currentHoleIndices.Length > 0)
                {
                    ref var brushIntersections = ref brushesTouchedByBrushes[currentInfo.brushNodeIndex].Value.brushIntersections;
                    for (int h = 0; h < currentHoleIndices.Length; h++)
                    {
                        // Need to make a copy so we can edit it without causing side effects
                        var holeIndex = currentHoleIndices[h];
                        var holeEdges = allEdges[holeIndex];
                        if (holeEdges.Length < 3)
                            continue;

                        var holeInfo        = allInfos[holeIndex];
                        var holeBrushNodeID = holeInfo.brushNodeIndex + 1;

                        // TODO: Optimize and make this a BlobAsset that's created in a pass,
                        //       this BlobAsset must make it easy to quickly determine if two
                        //       brushes intersect. Use this for both routing table generation 
                        //       and loop merging.
                        bool touches = false;
                        for (int t = 0; t < brushIntersections.Length; t++)
                        {
                            if (brushIntersections[t].nodeIndex == holeBrushNodeID)
                            {
                                touches = true;
                                break;
                            }
                        }

                        // But only if they touch
                        if (touches)
                        {
                            intersectedHoleIndices.Add(allEdges.Length);
                            holeIndices.Add();
                            allInfos.Add(holeInfo);
                            allEdges.Add(holeEdges);
                        }
                    }
                }

                // TODO: Separate loop "shapes" from category/loop-hole hierarchy, 
                //       so we can simply assign the same shape to a hole and loop without 
                //       needing to copy data we can create a new shape when we modify it.

                // this loop is a hole 
                currentHoleIndices.Add(allEdges.Length);
                holeIndices.Add();
                allInfos.Add(intersectionInfo);
                allEdges.Add(outEdges);

                // but also a polygon on its own
                loopIndices.Add(allEdges.Length);
                holeIndices.Add(intersectedHoleIndices);
                allInfos.Add(intersectionInfo);
                allEdges.Add(outEdges);

                intersectedHoleIndices.Dispose();
                outEdges.Dispose();
            }
        }

        public void RemoveEmptyLoopsJob(NativeListArray<int>.NativeList loopIndices, NativeListArray<int> holeIndices)
        {
            for (int l = loopIndices.Length - 1; l >= 0; l--)
            {
                var loopIndex = loopIndices[l];
                var loopEdges = allEdges[loopIndex];
                if (loopEdges.Length < 3)
                {
                    loopIndices.RemoveAtSwapBack(l);
                    break;
                }

                var holeIndicesList = holeIndices[loopIndex];
                for (int h = holeIndicesList.Length - 1; h >= 0; h--)
                {
                    var holeIndex = holeIndicesList[h];
                    var holeEdges = allEdges[holeIndex];
                    if (holeEdges.Length < 3)
                    {
                        holeIndicesList.RemoveAtSwapBack(h);
                        break;
                    }
                }
            }
        }

        public void CleanUpJob(in NativeListArray<int>.NativeList loopIndices, NativeListArray<int> holeIndices)
        {
            for (int l = loopIndices.Length - 1; l >= 0; l--)
            {
                var baseloopIndex   = loopIndices[l];
                var baseLoopEdges   = allEdges[baseloopIndex];
                if (baseLoopEdges.Length < 3)
                {
                    baseLoopEdges.Clear();
                    continue;
                }

                var baseLoopNormal = CSGManagerPerformCSG.CalculatePlaneEdges(baseLoopEdges, brushVertices);
                if (math.all(baseLoopNormal == float3.zero))
                {
                    baseLoopEdges.Clear();
                    continue;
                }

                var holeIndicesList = holeIndices[baseloopIndex];
                if (holeIndicesList.Length == 0)
                    continue;

                var baseLoopInfo    = allInfos[baseloopIndex];
                
                for (int h = holeIndicesList.Length - 1; h >= 0; h--)
                {
                    var holeIndex   = holeIndicesList[h];
                    var holeEdges   = allEdges[holeIndex];
                    if (holeEdges.Length < 3)
                    {
                        holeIndicesList.RemoveAtSwapBack(h);
                        continue;
                    }
                    var holeNormal = CSGManagerPerformCSG.CalculatePlaneEdges(holeEdges, brushVertices);
                    if (math.all(holeNormal == float3.zero))
                    {
                        holeIndicesList.RemoveAtSwapBack(h);
                        continue;
                    }
                }
                if (holeIndicesList.Length == 0)
                    continue;

                var allWorldPlanes   = new NativeList<float4>(Allocator.Temp);
                var allSegments      = new NativeList<LoopSegment>(Allocator.Temp);
                var allCombinedEdges = new NativeList<Edge>(Allocator.Temp);
                {
                    int edgeOffset = 0;
                    int planeOffset = 0;
                    for (int h = 0; h < holeIndicesList.Length; h++)
                    {
                        var holeIndex   = holeIndicesList[h];
                        var holeInfo    = allInfos[holeIndex];
                        var holeEdges   = allEdges[holeIndex];

                        // TODO: figure out why sometimes polygons are flipped around, and try to fix this at the source
                        var holeNormal  = CSGManagerPerformCSG.CalculatePlaneEdges(holeEdges, brushVertices);
                        if (math.dot(holeNormal, baseLoopNormal) > 0)
                        {
                            for (int n = 0; n < holeEdges.Length; n++)
                            {
                                var holeEdge = holeEdges[n];
                                var i1 = holeEdge.index1;
                                var i2 = holeEdge.index2;
                                holeEdge.index1 = i2;
                                holeEdge.index2 = i1;
                                holeEdges[n] = holeEdge;
                            }
                        }

                        ref var worldPlanes = ref brushWorldPlanes[holeInfo.brushNodeIndex].Value.worldPlanes;
                        
                        var edgesLength     = holeEdges.Length;
                        var planesLength    = worldPlanes.Length;

                        allSegments.Add(new LoopSegment()
                        {
                            edgeOffset      = edgeOffset,
                            edgeLength      = edgesLength,
                            planesOffset    = planeOffset,
                            planesLength    = planesLength
                        });

                        allCombinedEdges.AddRange(holeEdges);

                        // TODO: ideally we'd only use the planes that intersect our edges
                        allWorldPlanes.AddRange(worldPlanes.GetUnsafePtr(), planesLength);

                        edgeOffset += edgesLength;
                        planeOffset += planesLength;
                    }
                    if (baseLoopEdges.Length > 0)
                    {
                        ref var worldPlanes = ref brushWorldPlanes[baseLoopInfo.brushNodeIndex].Value.worldPlanes;

                        var planesLength    = worldPlanes.Length;
                        var edgesLength     = baseLoopEdges.Length;

                        allSegments.Add(new LoopSegment()
                        {
                            edgeOffset      = edgeOffset,
                            edgeLength      = edgesLength,
                            planesOffset    = planeOffset,
                            planesLength    = planesLength
                        });

                        allCombinedEdges.AddRange(baseLoopEdges);

                        // TODO: ideally we'd only use the planes that intersect our edges
                        allWorldPlanes.AddRange(worldPlanes.GetUnsafePtr(), planesLength);

                        edgeOffset += edgesLength;
                        planeOffset += planesLength;
                    }

                    var destroyedEdges = new NativeArray<byte>(edgeOffset, Allocator.Temp);
                    {
                        {
                            var subtractEdgesJob = new SubtractEdgesJob()
                            {
                                vertices            = brushVertices,
                                segmentIndex        = holeIndicesList.Length,
                                destroyedEdges      = destroyedEdges,
                                allWorldPlanes      = allWorldPlanes,
                                allSegments         = allSegments,
                                allEdges            = allCombinedEdges
                            };
                            for (int j=0;j< holeIndicesList.Length;j++)
                                subtractEdgesJob.Execute(j);
                        }

                        // TODO: optimize, keep track which holes (potentially) intersect
                        // TODO: create our own bounds data structure that doesn't use stupid slow properties for everything
                        {
                            var mergeEdgesJob = new MergeEdgesJob()
                            {
                                vertices            = brushVertices,
                                segmentCount        = holeIndicesList.Length,
                                destroyedEdges      = destroyedEdges,
                                allWorldPlanes      = allWorldPlanes,
                                allSegments         = allSegments,
                                allEdges            = allCombinedEdges
                            };
                            for (int j = 0, length = GeometryMath.GetTriangleArraySize(holeIndicesList.Length); j < length; j++)
                                mergeEdgesJob.Execute(j);
                        }

                        {
                            var segment = allSegments[holeIndicesList.Length];
                            for (int e = baseLoopEdges.Length - 1; e >= 0; e--)
                            {
                                if (destroyedEdges[segment.edgeOffset + e] == 0)
                                    continue;
                                baseLoopEdges.RemoveAtSwapBack(e);
                            }
                        }

                        for (int h1 = holeIndicesList.Length - 1; h1 >= 0; h1--)
                        {
                            var holeIndex1  = holeIndicesList[h1];
                            var holeEdges1  = allEdges[holeIndex1];
                            var segment     = allSegments[h1];
                            for (int e = holeEdges1.Length - 1; e >= 0; e--)
                            {
                                if (destroyedEdges[segment.edgeOffset + e] == 0)
                                    continue;
                                holeEdges1.RemoveAtSwapBack(e);
                            }
                        }
                    }
                    destroyedEdges.Dispose();
                }
                allWorldPlanes  .Dispose();
                allSegments     .Dispose();
                allCombinedEdges.Dispose();

                for (int h = holeIndicesList.Length - 1; h >= 0; h--)
                {
                    var holeIndex   = holeIndicesList[h];
                    var holeEdges   = allEdges[holeIndex];
                    // Note: can have duplicate edges when multiple holes share an edge
                    //          (only edges between holes and base-loop are guaranteed to not be duplciate)
                    AddEdges(baseLoopEdges, holeEdges, removeDuplicates: true);
                }

                holeIndicesList.Clear();
            }

            // TODO: remove the need for this
            for (int l = loopIndices.Length - 1; l >= 0; l--)
            {
                var baseloopIndex   = loopIndices[l];
                var baseLoopEdges   = allEdges[baseloopIndex];
                if (baseLoopEdges.Length < 3)
                {
                    loopIndices.RemoveAtSwapBack(l);
                    continue;
                }
            }
        }

        public void Execute()
        {
            if (basePolygonSurfaceInfos.Length == 0)
                return;


            if (!routingTableLookup.TryGetValue(brushNodeIndex, out BlobAssetReference<RoutingTable> routingTable))
            {
                //Debug.LogError("No routing table found");
                return;
            }

            ref var nodes                   = ref routingTable.Value.nodes;
            ref var routingLookups          = ref routingTable.Value.routingLookups;
                
            var surfaceCount                = basePolygonEdges.Length;

            NativeListArray<Edge>       intersectionLoops;
            NativeArray<SurfaceInfo>    intersectionSurfaceInfo;
            {
                ref var routingTableNodes = ref routingTable.Value.nodes;

                var nodeIDtoIndex = new NativeHashMap<int, int>(routingTableNodes.Length, Allocator.Temp);
                for (int i = 0; i < routingTableNodes.Length; i++)
                    nodeIDtoIndex[routingTableNodes[i]] = i;

                // TODO: Sort the brushSurfaceInfos/intersectionEdges based on nodeIDtoIndex[surfaceInfo.brushNodeID], 
                //       have a sequential list of all data. 
                //       Have segment list to determine which part of array belong to which brushNodeID
                //       Don't need bottom part, can determine this in Job

                intersectionLoops           = new NativeListArray<Edge>(16, Allocator.Temp);
                intersectionSurfaceInfo     = new NativeArray<SurfaceInfo>(routingTableNodes.Length * surfaceCount, Allocator.Temp);
                intersectionLoops.ResizeExact(routingTableNodes.Length * surfaceCount);
                for (int i = 0; i < intersectionSurfaceInfos.Length; i++)
                {
                    var surfaceInfo = intersectionSurfaceInfos[i];
                    var brushNodeIndex1 = surfaceInfo.brushNodeIndex;
                    var brushNodeID1 = brushNodeIndex1 + 1;

                    if (!nodeIDtoIndex.TryGetValue(brushNodeID1, out int brushIndex))
                        continue;

                    int offset = (brushIndex * surfaceCount) + surfaceInfo.basePlaneIndex;
                    intersectionLoops[offset].AddRange(intersectionEdges[i]);
                    intersectionSurfaceInfo[offset] = surfaceInfo;
                }

                nodeIDtoIndex.Dispose();
            }


            var holeIndices = new NativeListArray<int>(surfaceCount, Allocator.Temp);
            surfaceLoopIndices.ResizeExact(surfaceCount);
            for (int s = 0; s < surfaceCount; s++)
            {
                var info = basePolygonSurfaceInfos[s];
                info.interiorCategory = CategoryGroupIndex.First; // TODO: make sure that it's always set to "First" so we don't need to do this

                var loopIndices = surfaceLoopIndices[s];
                loopIndices.Add(allEdges.Length);
                holeIndices.Add();
                allInfos.Add(info);
                allEdges.Add(basePolygonEdges[s]);
            }

            for (int i = 0, offset = 0; i < routingLookups.Length; i++)
            {
                ref var routingLookup = ref routingLookups[i];
                for (int surfaceIndex = 0; surfaceIndex < surfaceCount; surfaceIndex++, offset++)
                {
                    var loopIndices         = surfaceLoopIndices[surfaceIndex];
                    var intersectionLoop    = intersectionLoops[offset];
                    var intersectionInfo    = intersectionSurfaceInfo[offset];
                    for (int l = loopIndices.Length - 1; l >= 0; l--)
                    {
                        var surfaceLoopIndex = loopIndices[l];
                        var surfaceLoopEdges = allEdges[surfaceLoopIndex];
                        if (surfaceLoopEdges.Length < 3)
                            continue;

                        var surfaceLoopInfo = allInfos[surfaceLoopIndex];

                        if (!routingLookup.TryGetRoute(routingTable, surfaceLoopInfo.interiorCategory, out CategoryRoutingRow routingRow))
                        {
                            Debug.Assert(false, "Could not find route");
                            continue;
                        }

                        bool overlap = intersectionLoop.Length != 0 &&
                                        BooleanEdgesUtility.AreLoopsOverlapping(surfaceLoopEdges, intersectionLoop);

                        // Lookup categorization lookup between original surface & other surface ...
                        if (overlap)
                        {
                            // If we overlap don't bother with creating a new polygon + hole and reuse existing one
                            surfaceLoopInfo.interiorCategory = routingRow[intersectionInfo.interiorCategory];
                            allInfos[surfaceLoopIndex] = surfaceLoopInfo;
                            continue;
                        } else
                        {
                            surfaceLoopInfo.interiorCategory = routingRow[CategoryIndex.Outside];
                            allInfos[surfaceLoopIndex] = surfaceLoopInfo;
                        }

                        // Add all holes that share the same plane to the polygon
                        if (intersectionLoop.Length != 0)
                        {
                            // Categorize between original surface & intersection
                            var intersectionCategory = routingRow[intersectionInfo.interiorCategory];

                            // If the intersection polygon would get the same category, we don't need to do a pointless intersection
                            if (intersectionCategory == surfaceLoopInfo.interiorCategory)
                                continue;

                            IntersectLoopsJob(surfaceLoopIndex, intersectionLoop, intersectionCategory, intersectionInfo, loopIndices, holeIndices);
                        }
                    }

                    // TODO: remove the need for this (check on insertion)
                    RemoveEmptyLoopsJob(loopIndices, holeIndices);
                }
            }

            intersectionLoops.Dispose();
            intersectionSurfaceInfo.Dispose();

            for (int surfaceIndex = 0; surfaceIndex < surfaceLoopIndices.Length; surfaceIndex++)
            {
                var loopIndices = surfaceLoopIndices[surfaceIndex];
                CleanUpJob(in loopIndices, holeIndices);
            }

            holeIndices.Dispose();
        }
    }
}
