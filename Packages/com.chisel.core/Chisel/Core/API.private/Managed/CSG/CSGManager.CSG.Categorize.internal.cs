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
            if (polygon1.edges.Count < 3 ||
                polygon2.edges.Count < 3)
                return false;

            if (polygon1.edges.Count != polygon2.edges.Count)
                return false;

            for (int i = 0; i < polygon1.edges.Count; i++)
            {
                for (int j = 0; j < polygon2.edges.Count; j++)
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
            using (new ProfileSample("BooleanIntersectionOperation"))
            {
                Loop newPolygon = null;
                //if (AreLoopsOverlapping(polygon1, polygon2))
                //    return CSGManagerPerformCSG.OperationResult.Overlapping;

                var brush1 = polygon1.info.brush;
                var mesh1 = BrushMeshManager.GetBrushMesh(brush1.BrushMesh.BrushMeshID);
                var nodeToTreeSpaceInversed1 = brush1.TreeToNodeSpaceMatrix;
                var nodeToTreeSpace1 = brush1.NodeToTreeSpaceMatrix;

                var worldSpacePlanes1Length = mesh1.planes.Length;
                var worldSpacePlanes1 = stackalloc float4[worldSpacePlanes1Length];
                fixed (float4* mesh1Planes = &mesh1.planes[0])
                {
                    CSGManagerPerformCSG.TransformByTransposedInversedMatrix(worldSpacePlanes1, (float4*)mesh1Planes, mesh1.planes.Length, math.transpose(nodeToTreeSpaceInversed1));
                }

                newPolygon = new Loop()
                {
                    info = polygon1.info
                };


                var vertices = soup.vertices;
                for (int i = 0; i < polygon2.edges.Count; i++)
                {
                    var edge = polygon2.edges[i];
                    var worldVertex = new float4((vertices[edge.index1] + vertices[edge.index2]) * 0.5f, 1);
                    if (IsOutsidePlanes(worldSpacePlanes1, worldSpacePlanes1Length, worldVertex))
                        continue;
                    //if (!newPolygon.indices.Contains(vertexIndex)) 
                        newPolygon.edges.Add(edge);
                }

                if (newPolygon.edges.Count == polygon2.edges.Count) // all vertices of polygon2 are inside polygon1
                    return CSGManagerPerformCSG.OperationResult.Polygon2InsidePolygon1;


                var brush2 = polygon2.info.brush;
                var mesh2 = BrushMeshManager.GetBrushMesh(brush2.BrushMesh.BrushMeshID);
                var nodeToTreeSpaceInversed2 = brush2.TreeToNodeSpaceMatrix;
                var nodeToTreeSpace2 = brush2.NodeToTreeSpaceMatrix;

                var worldSpacePlanes2Length = mesh2.planes.Length;
                var worldSpacePlanes2 = stackalloc float4[mesh2.planes.Length];
                fixed (float4* mesh2Planes = &mesh2.planes[0])
                {
                    CSGManagerPerformCSG.TransformByTransposedInversedMatrix(worldSpacePlanes2, (float4*)mesh2Planes, mesh2.planes.Length, math.transpose(nodeToTreeSpaceInversed2));
                }

                if (newPolygon.edges.Count == 0) // no vertex of polygon2 is inside polygon1
                {
                    // polygon edges are not intersecting
                    var edge = polygon1.edges[0];
                    var worldVertex = new float4((vertices[edge.index1] + vertices[edge.index2]) * 0.5f, 1);
                    if (IsOutsidePlanes(worldSpacePlanes2, worldSpacePlanes2Length, worldVertex))
                        // no vertex of polygon1 can be inside polygon2
                        return CSGManagerPerformCSG.OperationResult.Outside;

                    // all vertices of polygon1 must be inside polygon2
                    {
                        resultLoops.Add(new Loop(polygon1));
                        return CSGManagerPerformCSG.OperationResult.Cut;
                        //return CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2;
                    }
                } else
                {
                    // check if all vertices of polygon1 are on or inside polygon2
                    bool haveOutsideVertices = false;
                    for (int i = 0; i < polygon1.edges.Count; i++)
                    {
                        var edge = polygon1.edges[i];
                        var worldVertex = new float4((vertices[edge.index1] + vertices[edge.index2]) * 0.5f, 1);
                        if (!IsOutsidePlanes(worldSpacePlanes2, worldSpacePlanes2Length, worldVertex))
                            continue;
                        haveOutsideVertices = true;
                        break;
                    }
                    if (!haveOutsideVertices)
                    {
                        resultLoops.Add(new Loop(polygon1));
                        return CSGManagerPerformCSG.OperationResult.Cut;
                        //return CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2;
                    }
                }

                // we might be missing vertices on polygon1 that are inside polygon2 (intersections with other loops)
                // TODO: optimize
                for (int i = 0; i < polygon1.edges.Count; i++)
                {
                    var edge = polygon1.edges[i];
                    var worldVertex = new float4((vertices[edge.index1] + vertices[edge.index2]) * 0.5f, 1);
                    if (IsOutsidePlanes(worldSpacePlanes2, worldSpacePlanes2Length, worldVertex))
                        continue;

                    var edges = newPolygon.edges;
                    for (int e=0;e<edges.Count;e++)
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

            for (int i = 0; i < categorized_loop.edges.Count; i++)
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
                    loopsOnBrushSurface.RemoveAt(l);
                    break;
                }

                var holes = loopsOnBrushSurface[l].holes;
                for (int h = holes.Count - 1; h >= 0; h--)
                {
                    if (!holes[h].Valid)
                    {
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
                    return;

                case CSGManagerPerformCSG.OperationResult.Polygon2InsidePolygon1:
                {
                    // This new piece overrides the current loop
                    if (surfaceLoop.Valid)
                    {
                        var newPolygon = new Loop(surfaceLoop, newHoleCategory);
                        loopsOnBrushSurface.Add(newPolygon);
                    }
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
                if (overlappingLoop.Valid)
                {
                    if (CSGManagerPerformCSG.AreLoopsOverlapping(surfaceLoop, overlappingLoop))
                    {
                        surfaceLoop.info.interiorCategory = newHoleCategory;
                    } else
                    { 
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
                }
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

        internal static void CleanUp(in VertexSoup brushVertices, List<Loop> baseloops)
        {
            for (int l = baseloops.Count - 1; l >= 0; l--)
            {
                var baseloop = baseloops[l];
                if (baseloop.edges.Count < 3)
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
                    if (hole.edges.Count < 3)
                    {
                        holes.RemoveAt(h);
                        continue;
                    }
                    var holeNormal = CalculatePlaneEdges(hole, brushVertices);
                    if (math.all(holeNormal == float3.zero))
                    {
                        holes.RemoveAt(h);
                        continue;
                    }
                }
                if (holes.Count == 0)
                    continue;

                /*
                // TODO: figure out why sometimes polygons are flipped around, and try to fix this at the source
                if (math.dot(baseLoopNormal, baseloop.info.worldPlane.xyz) < 0)
                {
                    var holeEdges = baseloop.edges;
                    for (int n = 0; n < holeEdges.Count; n++)
                    {
                        var holeEdge = holeEdges[n];
                        var i1 = holeEdge.index1;
                        var i2 = holeEdge.index2;
                        holeEdge.index1 = i2;
                        holeEdge.index2 = i1;
                        holeEdges[n] = holeEdge;
                    }
                    baseLoopNormal = baseloop.info.worldPlane.xyz;
                }
                */

                // TODO: figure out why sometimes polygons are flipped around, and try to fix this at the source
                for (int h = holes.Count - 1; h >= 0; h--)
                {
                    var holeEdges = holes[h].edges;
                    var holeNormal = CalculatePlaneEdges(holes[h], brushVertices);
                    if (math.dot(holeNormal, baseLoopNormal) > 0)
                    {
                        for (int n = 0; n < holeEdges.Count; n++)
                        {
                            var holeEdge = holeEdges[n];
                            var i1 = holeEdge.index1;
                            var i2 = holeEdge.index2;
                            holeEdge.index1 = i2;
                            holeEdge.index2 = i1;
                            holeEdges[n] = holeEdge;
                        }
                    }
                }

                {
                    var brushInfo1 = CSGManager.GetBrushInfoUnsafe(baseloop.info.brush.brushNodeID);
                    var worldPlanes1 = brushInfo1.brushWorldPlanes;
                    using (var loop1Edges = new NativeArray<Edge>(baseloop.edges.Count, Allocator.TempJob))
                    {
                        baseloop.destroyed = new bool[baseloop.edges.Count];
                        for (int h1 = holes.Count - 1; h1 >= 0; h1--)
                        {
                            var hole1 = holes[h1];
                            hole1.destroyed = new bool[hole1.edges.Count];
                            CSGManagerPerformCSG.Subtract(brushVertices, worldPlanes1, loop1Edges, baseloop, hole1);
                        }
                    }
                }

                // TODO: optimize, keep track which holes (potentially) intersect
                for (int h1 = holes.Count - 1; h1 >= 0; h1--)
                {
                    var hole1 = holes[h1];

                    var brush1NodeID = hole1.info.brush.brushNodeID;

                    // TODO: create our own data structure that doesn't use stupid slow properties for everything
                    //Bounds bounds1 = default;
                    //if (CSGManager.GetBrushBounds(brush1NodeID, ref bounds1))
                    {
                        var brushInfo1 = CSGManager.GetBrushInfoUnsafe(brush1NodeID);
                        var worldPlanes1 = brushInfo1.brushWorldPlanes;
                        using (var loop1Edges = new NativeArray<Edge>(hole1.edges.Count, Allocator.TempJob))
                        {
                            for (int h2 = holes.Count - 1; h2 > h1; h2--)
                            {
                                var hole2 = holes[h2];
                                var brush2NodeID = hole2.info.brush.brushNodeID;
                                //Bounds bounds2 = default;
                                //if (CSGManager.GetBrushBounds(brush1NodeID, ref bounds2) &&
                                //    bounds1.Intersects(bounds2))
                                {
                                    CSGManagerPerformCSG.Merge(brushVertices, worldPlanes1, loop1Edges, hole1, hole2);
                                }
                            }
                        }
                    }
                }
                for (int e = baseloop.destroyed.Length - 1; e >= 0; e--)
                {
                    if (!baseloop.destroyed[e])
                        continue;
                    baseloop.edges.RemoveAt(e);
                }
                baseloop.destroyed = null;

                for (int h1 = holes.Count - 1; h1 >= 0; h1--)
                {
                    var hole1 = holes[h1];
                    for (int e = hole1.destroyed.Length - 1; e >= 0; e--)
                    {
                        if (!hole1.destroyed[e])
                            continue;
                        hole1.edges.RemoveAt(e);
                    }
                    hole1.destroyed = null;
                }

                for (int h = holes.Count - 1; h >= 0; h--)
                {
                    // Note: can have duplicate edges when multiple holes share an edge
                    //          (only edges between holes and base-loop are guaranteed to not be duplciate)
                    baseloop.AddEdges(holes[h].edges, removeDuplicates: true);
                }
                holes.Clear();
            }

            // TODO: remove the need for this
            for (int l = baseloops.Count - 1; l >= 0; l--)
            {
                var baseloop = baseloops[l];
                if (baseloop.edges.Count < 3)
                {
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
            for (int n = 0; n < loop.edges.Count; n++)
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

        static unsafe void Subtract(in VertexSoup soup, BlobAssetReference<BrushWorldPlanes> worldPlanes1, NativeArray<Edge> loop1Edges, Loop loop1, Loop loop2)
        {
            if (loop1.edges.Count == 0 ||
                loop2.edges.Count == 0)
                return;

            var categories1 = new EdgeCategory[loop1.edges.Count];
            var categories2 = new EdgeCategory[loop2.edges.Count];

            var brushInfo2 = CSGManager.GetBrushInfoUnsafe(loop2.info.brush.brushNodeID);
            var worldPlanes2 = brushInfo2.brushWorldPlanes;

            // TODO: allocate/dispose arrays outside of outer loop, or make all edges always a NativeArray
            using (var loop2Edges = new NativeArray<Edge>(loop2.edges.Count, Allocator.TempJob))
            {
                {
                    NativeArray<Edge> temp = loop1Edges; // workaround for "CaN't EdIt DiSpOsAbLe VaRiAbLeS" nonsense
                    var edgePtr = (Edge*)temp.GetUnsafePtr();
                    for (int e = 0; e < loop1.edges.Count; e++)
                        edgePtr[e] = loop1.edges[e];
                }
                {
                    NativeArray<Edge> temp = loop2Edges; // workaround for "CaN't EdIt DiSpOsAbLe VaRiAbLeS" nonsense
                    var edgePtr = (Edge*)temp.GetUnsafePtr();
                    for (int e = 0; e < loop2.edges.Count; e++)
                        edgePtr[e] = loop2.edges[e];
                }
                // 1. there are edges with two identical vertex indices?
                // 2. all edges should overlap, but some aren't overlapping?

                // *------*......*
                // |      |      .
                // |      |      .
                // |      |      .
                // *------*......*

                // *------*
                // | *----*
                // | |    .
                // | *----*
                // *------*

                fixed (EdgeCategory* categories1Ptr = &categories1[0])
                fixed (EdgeCategory* categories2Ptr = &categories2[0])
                fixed (bool* loop1EdgesDestroyed = &loop1.destroyed[0])
                fixed (bool* loop2EdgesDestroyed = &loop2.destroyed[0])
                { 
                    var categorizeEdgesJob1 = new CategorizeEdgesJob()
                    {
                        vertexSoup          = soup,
                        edges1              = loop1Edges,
                        destroyed1          = loop1EdgesDestroyed,
                        categories1         = categories1Ptr,
                        brushWorldPlanes    = worldPlanes2,
                        edges2              = loop2Edges
                    };
                    categorizeEdgesJob1.Run();

                    var categorizeEdgesJob2 = new CategorizeEdgesJob()
                    {
                        vertexSoup          = soup,
                        edges1              = loop2Edges,
                        destroyed1          = loop2EdgesDestroyed,
                        categories1         = categories2Ptr,
                        brushWorldPlanes    = worldPlanes1,
                        edges2              = loop1Edges
                    };
                    categorizeEdgesJob2.Run();
                    
                    var setEdgeDestroyedJob1 = new SetEdgeDestroyedJob()
                    {
                        edgeCount       = loop1.edges.Count,
                        destroyed1      = loop1EdgesDestroyed,
                        categories1     = categories1Ptr,
                    
                        // TODO: reversed edges should cancel?
                        good1           = EdgeCategory.Outside,
                        good2           = EdgeCategory.Aligned
                    };
                    setEdgeDestroyedJob1.Run();

                    var setEdgeDestroyedJob2 = new SetEdgeDestroyedJob()
                    {
                        edgeCount       = loop2.edges.Count,
                        destroyed1      = loop2EdgesDestroyed,
                        categories1     = categories2Ptr,

                        good1           = EdgeCategory.Inside,
                        good2           = EdgeCategory.Inside
                    };
                    setEdgeDestroyedJob2.Run();
                }
            }
        }



        static unsafe void Merge(in VertexSoup soup, BlobAssetReference<BrushWorldPlanes> worldPlanes1, NativeArray<Edge> loop1Edges, Loop loop1, Loop loop2)
        {
            if (loop1.edges.Count == 0 ||
                loop2.edges.Count == 0)
                return;

            var categories1 = new EdgeCategory[loop1.edges.Count];
            var categories2 = new EdgeCategory[loop2.edges.Count];

            var brushInfo2 = CSGManager.GetBrushInfoUnsafe(loop2.info.brush.brushNodeID);
            var worldPlanes2 = brushInfo2.brushWorldPlanes;

            // TODO: allocate/dispose arrays outside of outer loop, or make all edges always a NativeArray
            using (var loop2Edges = new NativeArray<Edge>(loop2.edges.Count, Allocator.TempJob))
            {
                {
                    NativeArray<Edge> temp = loop1Edges; // workaround for "CaN't EdIt DiSpOsAbLe VaRiAbLeS" nonsense
                    var edgePtr = (Edge*)temp.GetUnsafePtr();
                    for (int e = 0; e < loop1.edges.Count; e++)
                        edgePtr[e] = loop1.edges[e];
                }
                {
                    NativeArray<Edge> temp = loop2Edges; // workaround for "CaN't EdIt DiSpOsAbLe VaRiAbLeS" nonsense
                    var edgePtr = (Edge*)temp.GetUnsafePtr();
                    for (int e = 0; e < loop2.edges.Count; e++)
                        edgePtr[e] = loop2.edges[e];
                }
                fixed (EdgeCategory* categories1Ptr = &categories1[0])
                fixed (EdgeCategory* categories2Ptr = &categories2[0])
                fixed (bool* loop1EdgesDestroyed = &loop1.destroyed[0])
                fixed (bool* loop2EdgesDestroyed = &loop2.destroyed[0])
                { 
                    var categorizeEdgesJob1 = new CategorizeEdgesJob()
                    {
                        vertexSoup          = soup,
                        edges1              = loop1Edges,
                        destroyed1          = loop1EdgesDestroyed,
                        categories1         = categories1Ptr,
                        brushWorldPlanes    = worldPlanes2,
                        edges2              = loop2Edges
                    };
                    categorizeEdgesJob1.Run();

                    var categorizeEdgesJob2 = new CategorizeEdgesJob()
                    {
                        vertexSoup          = soup,
                        edges1              = loop2Edges,
                        destroyed1          = loop2EdgesDestroyed,
                        categories1         = categories2Ptr,
                        brushWorldPlanes    = worldPlanes1,
                        edges2              = loop1Edges
                    };
                    categorizeEdgesJob2.Run();

                    var setEdgeDestroyedJob1 = new SetEdgeDestroyedJob()
                    {
                        edgeCount       = loop1.edges.Count,
                        destroyed1      = loop1EdgesDestroyed,
                        categories1     = categories1Ptr,

                        good1           = EdgeCategory.Outside,
                        good2           = EdgeCategory.Aligned
                    };
                    setEdgeDestroyedJob1.Run();




                    var setEdgeDestroyedJob2 = new SetEdgeDestroyedJob()
                    {
                        edgeCount       = loop2.edges.Count,
                        destroyed1      = loop2EdgesDestroyed,
                        categories1     = categories2Ptr,

                        good1           = EdgeCategory.Outside,
                        good2           = EdgeCategory.Outside
                    };
                    setEdgeDestroyedJob2.Run();
                }
            }
        }

        #endregion
    }
#endif
}
