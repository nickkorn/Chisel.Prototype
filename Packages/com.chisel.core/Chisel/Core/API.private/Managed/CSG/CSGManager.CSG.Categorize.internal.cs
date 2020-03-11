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
    // TODO:    perform CSG using edges instead of indices.
    //          we don't care about ordering since triangulation doesn't care about ordering
    //          so CSG would simply be determining if edges are inside/outside or ON the other polygon & categorize them
    //
    //          Intersecting -> needs to check Inside/Outside on both polygons -> if inputs are convex, output is convex too
    //          Subtraction/Addition -> only needs to check Inside/Outside on secondary polygon
    //          
    //          In first phase (intersecting) just work with convex polygons, in second phase, keep subtracting/adding 
    //          convex polygons to final polygon.
    //          => can use bag of unordered edges
#if USE_MANAGED_CSG_IMPLEMENTATION
    static partial class CSGManagerPerformCSG
    {
        #region PerformBooleanOperation
        public enum OperationResult
        {
            Fail,
            Cut,
            Outside,
            Polygon1InsidePolygon2,
            Polygon2InsidePolygon1
        }

        public const double kEpsilon = 0.00001;
        public const double kEpsilon2 = 0.001;
        public const double kEpsilonSqr = kEpsilon * kEpsilon;

        // Note: Assumes polygons are convex
        public unsafe static bool AreLoopsOverlapping(Loop polygon1, Loop polygon2)
        {
            if (polygon1.edges.Length < 3 ||
                polygon2.edges.Length < 3)
                return false;

            if (polygon1.edges.Length != polygon2.edges.Length)
                return false;

            for (int i = 0; i < polygon1.edges.Length; i++)
            {
                for (int j = 0; j < polygon2.edges.Length; j++)
                {
                    if ((polygon1.edges[i].index1 == polygon1.edges[j].index1 &&
                         polygon1.edges[i].index2 == polygon1.edges[j].index2) ||
                        (polygon1.edges[i].index1 == polygon1.edges[j].index2 &&
                         polygon1.edges[i].index2 == polygon1.edges[j].index1))
                        return false;
                }
            }
            return true;
        }

        // Note: Assumes polygons are convex
        public unsafe static OperationResult BooleanIntersectionOperation(in VertexSoup soup, Loop polygon1, Loop polygon2, List<Loop> resultLoops)
        {
            //if (AreLoopsOverlapping(polygon1, polygon2))
            //    return CSGManagerPerformCSG.OperationResult.Overlapping;

            var brush1 = polygon1.info.brush;


            ref var worldSpacePlanes1 = ref CSGManager.GetBrushInfoUnsafe(brush1.brushNodeID).brushWorldPlanes.Value.worldPlanes;

            var newPolygon = new Loop()
            {
                info = polygon1.info
            };


            var vertices = soup.vertices;
            for (int i = 0; i < polygon2.edges.Length; i++)
            {
                var edge = polygon2.edges[i];
                var worldVertex = new float4((vertices[edge.index1] + vertices[edge.index2]) * 0.5f, 1);
                if (IsOutsidePlanes(ref worldSpacePlanes1, worldVertex))
                    continue;
                //if (!newPolygon.indices.Contains(vertexIndex)) 
                    newPolygon.edges.Add(edge);
            }

            if (newPolygon.edges.Length == polygon2.edges.Length) // all vertices of polygon2 are inside polygon1
            {
                newPolygon.Dispose();
                return CSGManagerPerformCSG.OperationResult.Polygon2InsidePolygon1;
            }


            var brush2 = polygon2.info.brush;
            ref var worldSpacePlanes2 = ref CSGManager.GetBrushInfoUnsafe(brush2.brushNodeID).brushWorldPlanes.Value.worldPlanes;

            if (newPolygon.edges.Length == 0) // no vertex of polygon2 is inside polygon1
            {
                // polygon edges are not intersecting
                var edge = polygon1.edges[0];
                var worldVertex = new float4((vertices[edge.index1] + vertices[edge.index2]) * 0.5f, 1);
                if (IsOutsidePlanes(ref worldSpacePlanes2, worldVertex))
                {
                    newPolygon.Dispose();
                    // no vertex of polygon1 can be inside polygon2
                    return CSGManagerPerformCSG.OperationResult.Outside;
                }

                // all vertices of polygon1 must be inside polygon2
                {
                    newPolygon.Dispose();
                    resultLoops.Add(new Loop(polygon1));
                    return CSGManagerPerformCSG.OperationResult.Cut;
                    //return CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2;
                }
            } else
            {
                // check if all vertices of polygon1 are on or inside polygon2
                bool haveOutsideVertices = false;
                for (int i = 0; i < polygon1.edges.Length; i++)
                {
                    var edge = polygon1.edges[i];
                    var worldVertex = new float4((vertices[edge.index1] + vertices[edge.index2]) * 0.5f, 1);
                    if (!IsOutsidePlanes(ref worldSpacePlanes2, worldVertex))
                        continue;
                    haveOutsideVertices = true;
                    break;
                }
                if (!haveOutsideVertices)
                {
                    newPolygon.Dispose();
                    resultLoops.Add(new Loop(polygon1));
                    return CSGManagerPerformCSG.OperationResult.Cut;
                    //return CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2;
                }
            }

            // we might be missing vertices on polygon1 that are inside polygon2 (intersections with other loops)
            // TODO: optimize
            for (int i = 0; i < polygon1.edges.Length; i++)
            {
                var edge = polygon1.edges[i];
                var worldVertex = new float4((vertices[edge.index1] + vertices[edge.index2]) * 0.5f, 1);
                if (IsOutsidePlanes(ref worldSpacePlanes2, worldVertex))
                    continue;

                var edges = newPolygon.edges;
                for (int e = 0; e < edges.Length; e++)
                {
                    if (edges[e].index1 == edge.index1 &&
                        edges[e].index2 == edge.index2)
                        goto SkipEdge;
                }
                //if (!newPolygon.edges.Contains(edge))
                    newPolygon.edges.Add(edge);
                SkipEdge:
                ;
            }

            resultLoops.Add(newPolygon);
            return CSGManagerPerformCSG.OperationResult.Cut;
        }

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

        static readonly List<Loop> s_OverlappingArea = new List<Loop>();
        internal static void Intersect(in VertexSoup brushVertices, List<Loop> loopsOnBrushSurface, Loop surfaceLoop, Loop intersectionLoop, CategoryGroupIndex newHoleCategory)
        {
            // It might look like we could just set the interiorCategory of brush_intersection here, and let all other cut loops copy from it below,
            // but the same brush_intersection might be used by another categorized_loop and then we'd try to reroute it again, which wouldn't work
            //brush_intersection.interiorCategory = newHoleCategory;

            // TODO: input polygons are convex, and intersection of both polygons will always be convex.
            //       Make an intersection boolean operation that assumes everything is convex
            Debug.Assert(s_OverlappingArea.Count == 0);
            s_OverlappingArea.Clear();
            OperationResult result;
            result = CSGManagerPerformCSG.BooleanIntersectionOperation(brushVertices,
                                                                     intersectionLoop, surfaceLoop,
                                                                     s_OverlappingArea  // the output of cutting operations are both holes for the original polygon (categorized_loop)
                                                                                        // and new polygons on the surface of the brush that need to be categorized
                                                                     );
            Debug.Assert(s_OverlappingArea.Count <= 1);

            // FIXME: when brush_intersection and categorized_loop are grazing each other, 
            //          technically we cut it but we shouldn't be creating it as a separate polygon + hole (bug7)


            switch (result)
            {
                default:
                case CSGManagerPerformCSG.OperationResult.Fail:     
                case CSGManagerPerformCSG.OperationResult.Outside:
                    foreach (var loop in s_OverlappingArea)
                        loop.Dispose();
                    s_OverlappingArea.Clear();
                    return;

                case CSGManagerPerformCSG.OperationResult.Polygon2InsidePolygon1:
                {
                    // This new piece overrides the current loop
                    if (surfaceLoop.Valid)
                    {
                        var newPolygon = new Loop(surfaceLoop, newHoleCategory);
                        loopsOnBrushSurface.Add(newPolygon);
                    }
                    foreach (var loop in s_OverlappingArea)
                        loop.Dispose();
                    s_OverlappingArea.Clear();
                    surfaceLoop.ClearAllEdges();
                    return;
                }
                /*
                case CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2:
                {
                    var newPolygon = new Loop(intersectionLoop, newHoleCategory);
                    s_OverlappingArea.Add(newPolygon);
                    break;
                }*/

                case CSGManagerPerformCSG.OperationResult.Cut:
                    break;
            }

            Debug.Assert(s_OverlappingArea.Count == 1, s_OverlappingArea.Count);

            // the output of cutting operations are both holes for the original polygon (categorized_loop)
            // and new polygons on the surface of the brush that need to be categorized
            if (surfaceLoop.holes.Capacity < surfaceLoop.holes.Count + s_OverlappingArea.Count)
                surfaceLoop.holes.Capacity = surfaceLoop.holes.Count + s_OverlappingArea.Count;

            int o = 0;
            //for (int o = 0; o < s_OverlappingArea.Count; o++)
            {
                var overlappingLoop = s_OverlappingArea[o];
                if (!overlappingLoop.Valid)
                {
                    foreach (var loop in s_OverlappingArea)
                        loop.Dispose();
                    s_OverlappingArea.Clear();
                    return;
                }
                
                if (CSGManagerPerformCSG.AreLoopsOverlapping(surfaceLoop, overlappingLoop))
                {
                    foreach (var loop in s_OverlappingArea)
                        loop.Dispose();
                    surfaceLoop.info.interiorCategory = newHoleCategory;
                    s_OverlappingArea.Clear();
                    return;
                } 

                overlappingLoop.info.interiorCategory = newHoleCategory;

                for (int h = 0; h < surfaceLoop.holes.Count; h++)
                {
                    // Need to make a copy so we can edit it without causing side effects
                    var copyPolygon = new Loop(surfaceLoop.holes[h]);
                    overlappingLoop.holes.Add(copyPolygon);
                }

                var newPolygon = new Loop(overlappingLoop, newHoleCategory);
                surfaceLoop.holes.Add(newPolygon);              // but it is also a hole for our polygon
                loopsOnBrushSurface.Add(overlappingLoop);       // this loop is a polygon on its own
            }
            s_OverlappingArea.Clear();
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

                        ref var worldPlanes = ref CSGManager.GetBrushInfoUnsafe(hole.info.brush.brushNodeID).brushWorldPlanes.Value.worldPlanes;

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
                        ref var worldPlanes = ref CSGManager.GetBrushInfoUnsafe(baseloop.info.brush.brushNodeID).brushWorldPlanes.Value.worldPlanes;

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
