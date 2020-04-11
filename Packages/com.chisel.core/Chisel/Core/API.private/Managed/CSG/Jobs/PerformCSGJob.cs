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
        //const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;

        [NoAlias, ReadOnly] public int                                                              brushNodeIndex;
        //[NoAlias, ReadOnly] public NativeArray<int>                                               treeBrushNodeIndices;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<RoutingTable>>             routingTableLookup;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>         brushWorldPlanes;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>    brushesTouchedByBrushes;

        [NoAlias, ReadOnly] public VertexSoup brushVertices; 
        [NoAlias, ReadOnly] public NativeList<SurfaceInfo>  intersectionSurfaceInfos;
        [NoAlias, ReadOnly] public NativeListArray<Edge>    intersectionEdges;

        [NoAlias, ReadOnly] public int                      surfaceCount;
        [NoAlias, ReadOnly] public NativeList<SurfaceInfo>  basePolygonSurfaceInfos;
        [NoAlias, ReadOnly] public NativeListArray<Edge>    basePolygonEdges;

        [NoAlias] public NativeListArray<int>               surfaceLoopIndices;
        [NoAlias] public NativeList<SurfaceInfo>            allInfos;
        [NoAlias] public NativeListArray<Edge>              allEdges;

        struct Empty {}

        [BurstDiscard]
        private static void NotUniqueEdgeException() 
        {
            throw new Exception("Edge is not unique");
        }

        static bool AddEdges(NativeListArray<Edge>.NativeList edges, NativeListArray<Edge>.NativeList addEdges, bool removeDuplicates = false)
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
            for (int v = 0; v < values.Length;v++)
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

        static int IndexOf(NativeListArray<Edge>.NativeList edges, Edge edge, out bool inverted)
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
        
        void IntersectLoopsJob(NativeList<SurfaceInfo>           allInfos,
                               NativeListArray<Edge>             allEdges,
                               in VertexSoup                     brushVertices,
                               int                               surfaceLoopIndex, 
                               NativeListArray<Edge>.NativeList  intersectionLoop, 
                               CategoryGroupIndex                intersectionCategory, 
                               SurfaceInfo                       intersectionInfo, 
                               NativeListArray<int>.NativeList   loopIndices, 
                               NativeListArray<int>              holeIndices)
        {
            var currentLoopEdges    = allEdges[surfaceLoopIndex];
            var currentInfo         = allInfos[surfaceLoopIndex];
            var currentHoleIndices  = holeIndices[surfaceLoopIndex];

            // It might look like we could just set the interiorCategory of brush_intersection here, and let all other cut loops copy from it below,
            // but the same brush_intersection might be used by another categorized_loop and then we'd try to reroute it again, which wouldn't work
            //brush_intersection.interiorCategory = newHoleCategory;

            if (intersectionLoop.Length == 0 ||
                currentLoopEdges.Length == 0)
                return;

            var maxLength       = math.max(intersectionLoop.Length, currentLoopEdges.Length);
            if (maxLength < 3)
                return;

            int inside2 = 0, outside2 = 0;
            var categories2     = stackalloc EdgeCategory[currentLoopEdges.Length];
            var worldPlanes1    = brushWorldPlanes[intersectionInfo.brushNodeIndex];
            for (int e = 0; e < currentLoopEdges.Length; e++)
            {
                var category = BooleanEdgesUtility.CategorizeEdge(currentLoopEdges[e], ref worldPlanes1.Value.worldPlanes, intersectionLoop, brushVertices);
                categories2[e] = category;
                if      (category == EdgeCategory.Inside) inside2++;
                else if (category == EdgeCategory.Outside) outside2++;
            }
            var aligned2 = currentLoopEdges.Length - (inside2 + outside2);

            int inside1 = 0, outside1 = 0;
            var categories1     = stackalloc EdgeCategory[intersectionLoop.Length];
            var worldPlanes2    = brushWorldPlanes[currentInfo.brushNodeIndex];
            for (int e = 0; e < intersectionLoop.Length; e++)
            {
                var category = BooleanEdgesUtility.CategorizeEdge(intersectionLoop[e], ref worldPlanes2.Value.worldPlanes, currentLoopEdges, brushVertices);
                categories1[e] = category;
                if      (category == EdgeCategory.Inside) inside1++;
                else if (category == EdgeCategory.Outside) outside1++;
            }
            var aligned1 = intersectionLoop.Length - (inside1 + outside1);

            // Completely outside
            if ((inside1 + aligned1) == 0 && (aligned2 + inside2) == 0)
                return;

            if ((inside1 + (inside2 + aligned2)) < 3)
                return;

            // Completely aligned
            if (((outside1 + inside1) == 0 && (outside2 + inside2) == 0) ||
                // polygon1 edges Completely inside polygon2
                (inside1 == 0 && outside2 == 0))
            {
                // New polygon overrides the existing polygon
                currentInfo.interiorCategory = intersectionCategory;
                allInfos[surfaceLoopIndex] = currentInfo;
                return; 
            }
            

            var outEdges        = stackalloc Edge[maxLength];
            var outEdgesLength  = 0;

            // polygon2 edges Completely inside polygon1
            if (outside1 == 0 && inside2 == 0)
            {
                // polygon1 Completely inside polygon2
                for (int n = 0; n < intersectionLoop.Length; n++)
                {
                    outEdges[outEdgesLength] = intersectionLoop[n];
                    outEdgesLength++;
                }
                //OperationResult.Polygon1InsidePolygon2;
            } else
            {
                //int outEdgesLength = 0; // Can't read from outEdges.Length since it's marked as WriteOnly
                for (int e = 0; e < intersectionLoop.Length; e++)
                {
                    var category = categories1[e];
                    if (category == EdgeCategory.Inside)
                    {
                        outEdges[outEdgesLength] = intersectionLoop[e];
                        outEdgesLength++;
                    }
                }

                for (int e = 0; e < currentLoopEdges.Length; e++)
                {
                    var category = categories2[e];
                    if (category != EdgeCategory.Outside)
                    {
                        outEdges[outEdgesLength] = currentLoopEdges[e];
                        outEdgesLength++;
                    }
                }
                //OperationResult.Cut;
            }

            if (outEdgesLength < 3)
                return;

            // FIXME: when brush_intersection and categorized_loop are grazing each other, 
            //          technically we cut it but we shouldn't be creating it as a separate polygon + hole (bug7)

            // the output of cutting operations are both holes for the original polygon (categorized_loop)
            // and new polygons on the surface of the brush that need to be categorized
            intersectionInfo.interiorCategory = intersectionCategory;


            // Figure out why this is seemingly not necessary?
            /*
            var intersectedHoleIndices = stackalloc int[currentHoleIndices.Length];
            var intersectedHoleIndicesLength = 0;

            // the output of cutting operations are both holes for the original polygon (categorized_loop)
            // and new polygons on the surface of the brush that need to be categorized
            if (currentHoleIndices.Capacity < currentHoleIndices.Length + 1)
                currentHoleIndices.Capacity = currentHoleIndices.Length + 1;
            
            if (currentHoleIndices.Length > 0)
            {
                ref var brushesTouchedByBrush   = ref brushesTouchedByBrushes[currentInfo.brushNodeIndex].Value;
                ref var brushIntersections      = ref brushesTouchedByBrushes[currentInfo.brushNodeIndex].Value.brushIntersections;
                for (int h = 0; h < currentHoleIndices.Length; h++)
                {
                    // Need to make a copy so we can edit it without causing side effects
                    var holeIndex = currentHoleIndices[h];
                    var holeEdges = allEdges[holeIndex];
                    if (holeEdges.Length < 3)
                        continue;

                    var holeInfo            = allInfos[holeIndex];
                    var holeBrushNodeIndex  = holeInfo.brushNodeIndex;

                    bool touches = brushesTouchedByBrush.Get(holeBrushNodeIndex) != IntersectionType.NoIntersection;
                    
                    // Only add if they touch
                    if (touches)
                    {
                        intersectedHoleIndices[intersectedHoleIndicesLength] = allEdges.Length;
                        intersectedHoleIndicesLength++;
                        holeIndices.Add();
                        allInfos.Add(holeInfo);
                        allEdges.Add(holeEdges);
                    }
                }
            }
            */

            // This loop is a hole 
            currentHoleIndices.Add(allEdges.Length);
            holeIndices.Add();
            allInfos.Add(intersectionInfo);
            allEdges.Add(outEdges, outEdgesLength);

            // But also a polygon on its own
            loopIndices.Add(allEdges.Length);
            holeIndices.Add();// intersectedHoleIndices, intersectedHoleIndicesLength);
            allInfos.Add(intersectionInfo);
            allEdges.Add(outEdges, outEdgesLength);
        }

        void CleanUp(in NativeList<SurfaceInfo> allInfos, NativeListArray<Edge> allEdges, in VertexSoup brushVertices, in NativeListArray<int>.NativeList loopIndices, NativeListArray<int> holeIndices)
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

                        ref var worldPlanes = ref brushWorldPlanes[allInfos[holeIndex].brushNodeIndex].Value.worldPlanes;
                        
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
                        ref var worldPlanes = ref brushWorldPlanes[allInfos[baseloopIndex].brushNodeIndex].Value.worldPlanes;

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

                    var destroyedEdges = stackalloc byte[edgeOffset];
                    {
                        {
                            var segment1 = allSegments[holeIndicesList.Length];
                            for (int j = 0; j < holeIndicesList.Length; j++)
                            {
                                var segment2 = allSegments[j];

                                if (segment1.edgeLength == 0 ||
                                    segment2.edgeLength == 0)
                                    continue;

                                for (int e = 0; e < segment1.edgeLength; e++)
                                {
                                    var category = BooleanEdgesUtility.CategorizeEdge(allCombinedEdges[segment1.edgeOffset + e], allWorldPlanes, allCombinedEdges, segment2, brushVertices);
                                    if (category == EdgeCategory.Outside || category == EdgeCategory.Aligned)
                                        continue;
                                    destroyedEdges[segment1.edgeOffset + e] = 1;
                                }

                                for (int e = 0; e < segment2.edgeLength; e++)
                                {
                                    var category = BooleanEdgesUtility.CategorizeEdge(allCombinedEdges[segment2.edgeOffset + e], allWorldPlanes, allCombinedEdges, segment1, brushVertices);
                                    if (category == EdgeCategory.Inside)
                                        continue;
                                    destroyedEdges[segment2.edgeOffset + e] = 1;
                                }
                            }
                        }

                        // TODO: optimize, keep track which holes (potentially) intersect
                        // TODO: create our own bounds data structure that doesn't use stupid slow properties for everything
                        {
                            for (int j = 0, length = GeometryMath.GetTriangleArraySize(holeIndicesList.Length); j < length; j++)
                            {
                                var arrayIndex = GeometryMath.GetTriangleArrayIndex(j, holeIndicesList.Length);
                                var segmentIndex1 = arrayIndex.x;
                                var segmentIndex2 = arrayIndex.y;
                                var segment1 = allSegments[segmentIndex1];
                                var segment2 = allSegments[segmentIndex2];
                                if (segment1.edgeLength > 0 && segment2.edgeLength > 0)
                                {
                                    for (int e = 0; e < segment1.edgeLength; e++)
                                    {
                                        var category = BooleanEdgesUtility.CategorizeEdge(allCombinedEdges[segment1.edgeOffset + e], allWorldPlanes, allCombinedEdges, segment2, brushVertices);
                                        if (category == EdgeCategory.Outside ||
                                            category == EdgeCategory.Aligned)
                                            continue;
                                        destroyedEdges[segment1.edgeOffset + e] = 1;
                                    }

                                    for (int e = 0; e < segment2.edgeLength; e++)
                                    {
                                        var category = BooleanEdgesUtility.CategorizeEdge(allCombinedEdges[segment2.edgeOffset + e], allWorldPlanes, allCombinedEdges, segment1, brushVertices);
                                        if (category == EdgeCategory.Outside)
                                            continue;
                                        destroyedEdges[segment2.edgeOffset + e] = 1;
                                    }
                                }
                            }
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
            //int brushNodeIndex = treeBrushNodeIndices[index];

            if (surfaceCount == 0)
                return;


            if (!routingTableLookup.TryGetValue(brushNodeIndex, out BlobAssetReference<RoutingTable> routingTableRef))
            {
                //Debug.LogError("No routing table found");
                return;
            }

            ref var nodes               = ref routingTableRef.Value.nodes;
            ref var routingLookups      = ref routingTableRef.Value.routingLookups;
            ref var routingTableNodes   = ref routingTableRef.Value.nodes;

            var intersectionSurfaceInfo = stackalloc SurfaceInfo[routingTableNodes.Length * surfaceCount]; ;
            var intersectionLoops       = new NativeListArray<Edge>(0, Allocator.Temp);
            intersectionLoops.ResizeExact(routingTableNodes.Length * surfaceCount);

            {
                int maxIndex = 0;
                for (int i = 0; i < routingTableNodes.Length; i++)
                    maxIndex = math.max(maxIndex, routingTableNodes[i] - 1);

                var nodeIDtoIndex = stackalloc int[maxIndex+1];
                for (int i = 0; i < routingTableNodes.Length; i++)
                    nodeIDtoIndex[routingTableNodes[i] - 1] = i + 1;

                // TODO: Sort the brushSurfaceInfos/intersectionEdges based on nodeIDtoIndex[surfaceInfo.brushNodeID], 
                //       have a sequential list of all data. 
                //       Have segment list to determine which part of array belong to which brushNodeID
                //       Don't need bottom part, can determine this in Job

                for (int i = 0; i < intersectionSurfaceInfos.Length; i++)
                {
                    var surfaceInfo     = intersectionSurfaceInfos[i];
                    var brushNodeIndex1 = surfaceInfo.brushNodeIndex;

                    var brushIndex = nodeIDtoIndex[brushNodeIndex1];
                    if (brushIndex == 0)
                        continue;
                    brushIndex--;

                    int offset = brushIndex + (surfaceInfo.basePlaneIndex * routingLookups.Length);
                    intersectionLoops[offset].AddRange(intersectionEdges[i]);
                    intersectionSurfaceInfo[offset] = surfaceInfo;
                }
            }


            var holeIndices = new NativeListArray<int>(routingLookups.Length * surfaceCount, Allocator.Temp);
            surfaceLoopIndices.ResizeExact(surfaceCount);

            ref var routingTable = ref routingTableRef.Value;
            for (int surfaceIndex = 0, offset = 0; surfaceIndex < surfaceCount; surfaceIndex++)
            {
                if (basePolygonEdges[surfaceIndex].Length < 3)
                    continue;

                var info = basePolygonSurfaceInfos[surfaceIndex];
                info.interiorCategory = CategoryGroupIndex.First; // TODO: make sure that it's always set to "First" so we don't need to do this

                var loopIndices = surfaceLoopIndices[surfaceIndex];
                loopIndices.Add(allEdges.Length);
                holeIndices.Add();
                allInfos.Add(info);
                allEdges.Add(basePolygonEdges[surfaceIndex]);


                for (int i = 0; i < routingLookups.Length; i++, offset++)
                {
                    ref var routingLookup   = ref routingLookups[i];
                    var intersectionLoop    = intersectionLoops[offset];
                    var intersectionInfo    = intersectionSurfaceInfo[offset];
                    for (int l = loopIndices.Length - 1; l >= 0; l--)
                    {
                        var surfaceLoopIndex = loopIndices[l];
                        var surfaceLoopEdges = allEdges[surfaceLoopIndex];
                        if (surfaceLoopEdges.Length < 3)
                            continue;

                        var surfaceLoopInfo = allInfos[surfaceLoopIndex];

                        // Lookup categorization between original surface & other surface ...
                        if (!routingLookup.TryGetRoute(ref routingTable, surfaceLoopInfo.interiorCategory, out CategoryRoutingRow routingRow))
                        {
                            Debug.Assert(false, "Could not find route");
                            continue;
                        }

                        bool overlap = intersectionLoop.Length != 0 &&
                                        BooleanEdgesUtility.AreLoopsOverlapping(surfaceLoopEdges, intersectionLoop);

                        if (overlap)
                        {
                            // If we overlap don't bother with creating a new polygon & hole and just reuse existing polygon + replace category
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

                            IntersectLoopsJob(allInfos, allEdges, brushVertices,
                                              surfaceLoopIndex, 
                                              intersectionLoop, 
                                              intersectionCategory, 
                                              intersectionInfo, 
                                              loopIndices, 
                                              holeIndices);
                        }
                    }
                }
                CleanUp(in allInfos, allEdges, in brushVertices, in loopIndices, holeIndices);
            }

            intersectionLoops.Dispose();
            holeIndices.Dispose();
        }
    }
}
