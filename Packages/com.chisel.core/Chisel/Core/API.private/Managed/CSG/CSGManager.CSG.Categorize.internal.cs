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
using System.Globalization;
using System.Runtime.CompilerServices;
using Unity.Entities;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    static partial class CSGManagerPerformCSG
    {
        #region PerformBooleanOperation

        public const double kEpsilon = 0.00001;
        public const double kEpsilon2 = 0.001;
        public const double kEpsilonSqr = kEpsilon * kEpsilon;
        #endregion

        #region PerformCSG


        //*
        internal static void Dump(System.Text.StringBuilder builder, Loop categorized_loop, in VertexSoup soup, Quaternion rotation)
        {
            var vertices = soup.vertices;
            //builder.AppendLine($"loop ({categorized_loop.indices.Count}):");
            //builder.AppendLine($"loop {categorized_loop.loopIndex}:");
            /*
            for (int i = 0; i < categorized_loop.indices.Count; i++)
            {
                if (i > 0)
                    builder.Append(",");
                var index = categorized_loop.indices[i];

                builder.Append($"{index}");
            }
            builder.AppendLine();

            for (int i = 0; i < categorized_loop.indices.Count; i++)
            {
                var index = categorized_loop.indices[i];
                var vertex = ((float3)(rotation * vertices[index])).xy;

                builder.Append($"({Convert.ToString((Decimal)vertex.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex.y, CultureInfo.InvariantCulture)})");
                builder.Append(", ");
            }
            if (categorized_loop.indices.Count > 0)
            { 
                var index = categorized_loop.indices[0];
                var vertex = ((float3)(rotation * vertices[index])).xy;
                builder.Append($"({Convert.ToString((Decimal)vertex.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex.y, CultureInfo.InvariantCulture)})");
            }
            builder.AppendLine("             ");
            */

            for (int i = 0; i < categorized_loop.edges.Length; i++)
            {
                var edge = categorized_loop.edges[i];
                var index1 = edge.index1;
                var index2 = edge.index2;
                var vertex1 = ((float3)(rotation * vertices[index1])).xy;
                var vertex2 = ((float3)(rotation * vertices[index2])).xy;

                builder.Append($"({Convert.ToString((Decimal)vertex1.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex1.y, CultureInfo.InvariantCulture)}), ");
                builder.AppendLine($"({Convert.ToString((Decimal)vertex2.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex2.y, CultureInfo.InvariantCulture)})             ");
            }
        }
        //*/


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void RemoveEmptyLoops(List<Loop> loopsOnBrushSurface)
        {
            for (int l = loopsOnBrushSurface.Count - 1; l >= 0; l--)
            {
                if (!loopsOnBrushSurface[l].Valid)
                {
                    loopsOnBrushSurface[l].Dispose();
                    loopsOnBrushSurface.RemoveAt(l);
                    break;
                }

                var holes = loopsOnBrushSurface[l].holes;
                for (int h = holes.Count - 1; h >= 0; h--)
                {
                    if (!holes[h].Valid)
                    {
                        holes[h].Dispose();
                        holes.RemoveAt(h);
                        break;
                    }
                }
            }
        }

        internal static unsafe void Intersect(in VertexSoup brushVertices, List<Loop> loopsOnBrushSurface, Loop surfaceLoop, Loop intersectionLoop, CategoryGroupIndex newHoleCategory)
        {
            // It might look like we could just set the interiorCategory of brush_intersection here, and let all other cut loops copy from it below,
            // but the same brush_intersection might be used by another categorized_loop and then we'd try to reroute it again, which wouldn't work
            //brush_intersection.interiorCategory = newHoleCategory;

            var result = OperationResult.Fail;
            var intersectEdgesJob = new IntersectEdgesJob()
            {
                vertices        = brushVertices.vertices,
                edges1          = intersectionLoop.edges,
                edges2          = surfaceLoop.edges,
                worldPlanes1    = ChiselLookup.Value.brushWorldPlanes[intersectionLoop.info.brush.brushNodeID - 1],
                worldPlanes2    = ChiselLookup.Value.brushWorldPlanes[surfaceLoop.info.brush.brushNodeID - 1],

                result          = &result,
                outEdges        = new NativeList<Edge>(math.max(intersectionLoop.edges.Length, surfaceLoop.edges.Length), Allocator.Persistent)
            };
            intersectEdgesJob.Run();

            // *somehow* put whats below in a job

            if (result == OperationResult.Outside ||
                result == OperationResult.Fail)
            {
                intersectEdgesJob.outEdges.Dispose();
            } else
            if (result == OperationResult.Polygon2InsidePolygon1)
            {
                // This new piece overrides the current loop
                surfaceLoop.edges.Clear();
                surfaceLoop.edges.AddRange(intersectEdgesJob.outEdges);
                surfaceLoop.info.interiorCategory = newHoleCategory;
                intersectEdgesJob.outEdges.Dispose();
            } else
            if (result == OperationResult.Overlapping)
            {
                intersectEdgesJob.outEdges.Dispose();
                surfaceLoop.info.interiorCategory = newHoleCategory;
            } else
            { 
                // FIXME: when brush_intersection and categorized_loop are grazing each other, 
                //          technically we cut it but we shouldn't be creating it as a separate polygon + hole (bug7)

                // the output of cutting operations are both holes for the original polygon (categorized_loop)
                // and new polygons on the surface of the brush that need to be categorized
                var intersectedLoop = new Loop(intersectEdgesJob.outEdges, intersectionLoop.info);
                intersectedLoop.info.interiorCategory = newHoleCategory;

                // the output of cutting operations are both holes for the original polygon (categorized_loop)
                // and new polygons on the surface of the brush that need to be categorized
                if (surfaceLoop.holes.Capacity < surfaceLoop.holes.Count + 1)
                    surfaceLoop.holes.Capacity = surfaceLoop.holes.Count + 1;

                if (surfaceLoop.holes.Count > 0)
                {
                    ref var brushIntersections = ref ChiselLookup.Value.brushesTouchedByBrushes[surfaceLoop.info.brush.brushNodeID - 1].Value.brushIntersections;
                    for (int h = 0; h < surfaceLoop.holes.Count; h++)
                    {
                        // Need to make a copy so we can edit it without causing side effects
                        var hole = surfaceLoop.holes[h];
                        if (!hole.Valid)
                            continue;

                        var holeBrushNodeID = hole.info.brush.brushNodeID;

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
                            var copyPolygon = new Loop(hole);
                            intersectedLoop.holes.Add(copyPolygon);
                        }
                    }
                }

                // TODO: Separate loop "shapes" from category/loop-hole hierarchy, 
                //       so we can simply assign the same shape to a hole and loop without 
                //       needing to copy data we can create a new shape when we modify it.

                surfaceLoop.holes.Add(new Loop(intersectedLoop, newHoleCategory));  // but it is also a hole for our polygon
                loopsOnBrushSurface.Add(intersectedLoop);                           // this loop is a polygon on its own
            }
        }
        #endregion

        #region CleanUp

        // Clean up, after performing CSG
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void CleanUp(in VertexSoup soup, List<Loop>[] surfaceLoops)
        {
            for (int surfaceIndex = 0; surfaceIndex < surfaceLoops.Length; surfaceIndex++)
                CleanUp(soup, surfaceLoops[surfaceIndex]);
        }

        internal unsafe static void CleanUp(in VertexSoup brushVertices, List<Loop> baseloops)
        {
            for (int l = baseloops.Count - 1; l >= 0; l--)
            {
                var baseloop = baseloops[l];
                if (baseloop.edges.Length < 3)
                {
                    baseloop.edges.Clear();
                    continue;
                }

                var baseLoopNormal = CalculatePlaneEdges(baseloop, brushVertices);
                if (math.all(baseLoopNormal == float3.zero))
                {
                    baseloop.edges.Clear();
                    continue;
                }

                var holes = baseloop.holes;
                if (holes.Count == 0)
                    continue;
                
                for (int h = holes.Count - 1; h >= 0; h--)
                {
                    var hole = holes[h];
                    if (hole.edges.Length < 3)
                    {
                        holes[h].Dispose();
                        holes.RemoveAt(h);
                        continue;
                    }
                    var holeNormal = CalculatePlaneEdges(hole, brushVertices);
                    if (math.all(holeNormal == float3.zero))
                    {
                        holes[h].Dispose();
                        holes.RemoveAt(h);
                        continue;
                    }
                }
                if (holes.Count == 0)
                    continue;

                using (var allWorldPlanes   = new NativeList<float4>(Allocator.TempJob))
                using (var allSegments      = new NativeList<LoopSegment>(Allocator.TempJob))
                using (var allEdges         = new NativeList<Edge>(Allocator.TempJob))
                {
                    int edgeOffset = 0;
                    int planeOffset = 0;
                    for (int h = 0; h < holes.Count; h++)
                    {
                        var hole = holes[h];

                        // TODO: figure out why sometimes polygons are flipped around, and try to fix this at the source
                        var holeEdges = hole.edges;
                        var holeNormal = CalculatePlaneEdges(hole, brushVertices);
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

                        ref var worldPlanes = ref ChiselLookup.Value.brushWorldPlanes[hole.info.brush.brushNodeID - 1].Value.worldPlanes;
                        
                        var edgesLength = hole.edges.Length;
                        var planesLength = worldPlanes.Length;

                        allSegments.Add(new LoopSegment()
                        {
                            edgeOffset      = edgeOffset,
                            edgeLength      = edgesLength,
                            planesOffset    = planeOffset,
                            planesLength    = planesLength
                        });

                        allEdges.AddRange(hole.edges);

                        // TODO: ideally we'd only use the planes that intersect our edges
                        allWorldPlanes.AddRange(worldPlanes.GetUnsafePtr(), planesLength);

                        edgeOffset += edgesLength;
                        planeOffset += planesLength;
                    }
                    if (baseloop.edges.Length > 0)
                    {
                        ref var worldPlanes = ref ChiselLookup.Value.brushWorldPlanes[baseloop.info.brush.brushNodeID - 1].Value.worldPlanes;

                        var planesLength    = worldPlanes.Length;
                        var edgesLength     = baseloop.edges.Length;

                        allSegments.Add(new LoopSegment()
                        {
                            edgeOffset      = edgeOffset,
                            edgeLength      = edgesLength,
                            planesOffset    = planeOffset,
                            planesLength    = planesLength
                        });

                        allEdges.AddRange(baseloop.edges);

                        // TODO: ideally we'd only use the planes that intersect our edges
                        allWorldPlanes.AddRange(worldPlanes.GetUnsafePtr(), planesLength);

                        edgeOffset += edgesLength;
                        planeOffset += planesLength;
                    }

                    using (var destroyedEdges = new NativeArray<byte>(edgeOffset, Allocator.TempJob))
                    {
                        {
                            var subtractEdgesJob = new SubtractEdgesJob()
                            {
                                vertices            = brushVertices.vertices,
                                segmentIndex        = holes.Count,
                                destroyedEdges      = destroyedEdges,
                                allWorldPlanes      = allWorldPlanes,
                                allSegments         = allSegments,
                                allEdges            = allEdges
                            };
                            subtractEdgesJob.Run(holes.Count);
                        }

                        // TODO: optimize, keep track which holes (potentially) intersect
                        // TODO: create our own bounds data structure that doesn't use stupid slow properties for everything
                        {
                            var mergeEdgesJob = new MergeEdgesJob()
                            {
                                vertices            = brushVertices.vertices,
                                segmentCount        = holes.Count,
                                destroyedEdges      = destroyedEdges,
                                allWorldPlanes      = allWorldPlanes,
                                allSegments         = allSegments,
                                allEdges            = allEdges
                            };
                            mergeEdgesJob.Run(GeometryMath.GetTriangleArraySize(holes.Count));
                        }

                        {
                            var segment = allSegments[holes.Count];
                            for (int e = baseloop.edges.Length - 1; e >= 0; e--)
                            {
                                if (destroyedEdges[segment.edgeOffset + e] == 0)
                                    continue;
                                baseloop.edges.RemoveAtSwapBack(e);
                            }
                        }

                        for (int h1 = holes.Count - 1; h1 >= 0; h1--)
                        {
                            var hole1 = holes[h1];
                            var segment = allSegments[h1];
                            for (int e = hole1.edges.Length - 1; e >= 0; e--)
                            {
                                if (destroyedEdges[segment.edgeOffset + e] == 0)
                                    continue;
                                hole1.edges.RemoveAtSwapBack(e);
                            }
                        }
                    }
                }

                for (int h = holes.Count - 1; h >= 0; h--)
                {
                    // Note: can have duplicate edges when multiple holes share an edge
                    //          (only edges between holes and base-loop are guaranteed to not be duplciate)
                    baseloop.AddEdges(holes[h].edges, removeDuplicates: true);
                }
                foreach(var hole in holes)
                    hole.Dispose();
                holes.Clear();
            }

            // TODO: remove the need for this
            for (int l = baseloops.Count - 1; l >= 0; l--)
            {
                var baseloop = baseloops[l];
                if (baseloop.edges.Length < 3)
                {
                    baseloops[l].Dispose();
                    baseloops.RemoveAt(l);
                    continue;
                }
            }
        }

        
        static float3 CalculatePlaneEdges(in Loop loop, in VertexSoup soup)
        {
            // Newell's algorithm to create a plane for concave polygons.
            // NOTE: doesn't work well for self-intersecting polygons
            var normal = Vector3.zero;
            var vertices = soup.vertices;
            for (int n = 0; n < loop.edges.Length; n++)
            {
                var edge = loop.edges[n];
                var prevVertex = vertices[edge.index1];
                var currVertex = vertices[edge.index2];
                normal.x = normal.x + ((prevVertex.y - currVertex.y) * (prevVertex.z + currVertex.z));
                normal.y = normal.y + ((prevVertex.z - currVertex.z) * (prevVertex.x + currVertex.x));
                normal.z = normal.z + ((prevVertex.x - currVertex.x) * (prevVertex.y + currVertex.y));
            }
            normal = normal.normalized;

            return normal;
        }

#endregion
    }
#endif
}
