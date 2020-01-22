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

        // Assumes polygons are convex
        public unsafe static bool AreLoopsOverlapping(Loop polygon1, Loop polygon2)
        {
            if (polygon1.indices.Count != polygon2.indices.Count)
                return false;


            Debug.Assert(polygon1.convex && polygon2.convex);
            if (!polygon1.convex || !polygon2.convex)
                return false;
            
            for (int i = 0; i < polygon1.indices.Count; i++)
            {
                if (polygon2.indices.IndexOf(polygon1.indices[i]) == -1)
                    return false;
            }
            return true;
        }

        public unsafe static OperationResult PerformBooleanIntersection(VertexSoup soup, Loop polygon1, Loop polygon2, List<Loop> resultLoops)
        {
            UnityEngine.Profiling.Profiler.BeginSample("PerformBooleanOperation");
            try
            {
                Loop newPolygon = null;
                Debug.Assert(polygon1.convex && polygon2.convex);
                if (!polygon1.convex || !polygon2.convex)
                    return OperationResult.Fail;
                
                //if (AreLoopsOverlapping(polygon1, polygon2))
                //    return CSGManagerPerformCSG.OperationResult.Overlapping;

                var brush1 = polygon1.info.brush;
                var mesh1 = BrushMeshManager.GetBrushMesh(brush1.BrushMesh.BrushMeshID);
                var nodeToTreeSpaceInversed1 = brush1.TreeToNodeSpaceMatrix;
                var nodeToTreeSpace1 = brush1.NodeToTreeSpaceMatrix;

                var worldSpacePlanes1Length = mesh1.surfaces.Length;
                var worldSpacePlanes1 = stackalloc float4[worldSpacePlanes1Length];
                fixed (BrushMesh.Surface* mesh1Surfaces = &mesh1.surfaces[0])
                {
                    CSGManagerPerformCSG.TransformByTransposedInversedMatrix(worldSpacePlanes1, (float4*)mesh1Surfaces, mesh1.surfaces.Length, math.transpose(nodeToTreeSpaceInversed1));
                }

                newPolygon = new Loop()
                {
                    info = polygon1.info,
                    interiorCategory = polygon1.interiorCategory,
                    convex = true
                };

                for (int i = 0; i < polygon2.indices.Count; i++)
                {
                    var vertexIndex = polygon2.indices[i];
                    var worldVertex = new float4(soup.vertices[vertexIndex], 1);
                    if (IsOutsidePlanes(worldSpacePlanes1, worldSpacePlanes1Length, worldVertex))
                        continue;
                    if (!newPolygon.indices.Contains(vertexIndex)) 
                        newPolygon.indices.Add(vertexIndex);
                }

                if (newPolygon.indices.Count == polygon2.indices.Count) // all vertices of polygon2 are inside polygon1
                    return CSGManagerPerformCSG.OperationResult.Polygon2InsidePolygon1;


                var brush2 = polygon2.info.brush;
                var mesh2 = BrushMeshManager.GetBrushMesh(brush2.BrushMesh.BrushMeshID);
                var nodeToTreeSpaceInversed2 = brush2.TreeToNodeSpaceMatrix;
                var nodeToTreeSpace2 = brush2.NodeToTreeSpaceMatrix;

                var worldSpacePlanes2Length = mesh2.surfaces.Length;
                var worldSpacePlanes2 = stackalloc float4[mesh2.surfaces.Length];
                fixed (BrushMesh.Surface* mesh2Surfaces = &mesh2.surfaces[0])
                {
                    CSGManagerPerformCSG.TransformByTransposedInversedMatrix(worldSpacePlanes2, (float4*)mesh2Surfaces, mesh2.surfaces.Length, math.transpose(nodeToTreeSpaceInversed2));
                }

                if (newPolygon.indices.Count == 0) // no vertex of polygon2 is inside polygon1
                {
                    // polygon edges are not intersecting
                    var vertexIndex = polygon1.indices[0];
                    var worldVertex = new float4(soup.vertices[vertexIndex], 1);
                    if (IsOutsidePlanes(worldSpacePlanes2, worldSpacePlanes2Length, worldVertex))
                        // no vertex of polygon1 can be inside polygon2
                        return CSGManagerPerformCSG.OperationResult.Outside;

                    // all vertices of polygon1 must be inside polygon2
                    return CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2;
                } else
                {
                    // check if all vertices of polygon1 are on or inside polygon2
                    bool haveOutsideVertices = false;
                    for (int i = 0; i < polygon1.indices.Count; i++)
                    {
                        var vertexIndex = polygon1.indices[i];
                        var worldVertex = new float4(soup.vertices[vertexIndex], 1);
                        if (!IsOutsidePlanes(worldSpacePlanes2, worldSpacePlanes2Length, worldVertex))
                            continue;
                        haveOutsideVertices = true;
                        break;
                    }
                    if (!haveOutsideVertices)
                        return CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2;
                }

                // we might be missing vertices on polygon1 that are inside polygon2 (intersections with other loops)
                // TODO: optimize
                for (int i = 0; i < polygon1.indices.Count; i++)
                {
                    var vertexIndex = polygon1.indices[i];
                    var worldVertex = new float4(soup.vertices[vertexIndex], 1);
                    if (IsOutsidePlanes(worldSpacePlanes2, worldSpacePlanes2Length, worldVertex))
                        continue;

                    if (!newPolygon.indices.Contains(vertexIndex))
                        newPolygon.indices.Add(vertexIndex);
                }
                // We need to resort the indices now
                // TODO: find a way to not have to do this
                SortIndices(newPolygon.indices, soup.vertices, newPolygon.info.worldPlane.normal);

                newPolygon.AddEdges(newPolygon.indices);
                resultLoops.Add(newPolygon);
                return CSGManagerPerformCSG.OperationResult.Cut;
            }
            finally { UnityEngine.Profiling.Profiler.EndSample(); }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe static bool IsPointInPolygon(float3 right, float3 forward, List<ushort> indices, VertexSoup soup, float3 point)
        {
            var px = math.dot(right, point);
            var py = math.dot(forward, point);

            float ix, iy, jx, jy;

            var verticesPtr = (float3*)NativeArrayUnsafeUtility.GetUnsafePtr(soup.vertexArray);

            var vert = verticesPtr[indices[indices.Count - 1]];
            ix = math.dot(right,   vert);
            iy = math.dot(forward, vert);

            bool result = false;
            for (int i = 0; i < indices.Count; i++)
            {
                jx = ix;
                jy = iy;

                vert = verticesPtr[indices[i]];
                ix = math.dot(right,   vert);
                iy = math.dot(forward, vert);

                if ((py >= iy && py < jy) ||
                    (py >= jy && py < iy))
                {
                    if (ix + (py - iy) / (jy - iy) * (jx - ix) < px)
                    {
                        result = !result;
                    }
                }
            }
            return result;
        }

        #endregion


        #region PerformCSG


        //*
        static void Dump(System.Text.StringBuilder builder, Loop categorized_loop, VertexSoup soup, Quaternion rotation)
        {
            //builder.AppendLine($"loop ({categorized_loop.indices.Count}):");
            //builder.AppendLine($"loop {categorized_loop.loopIndex}:");
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
                var vertex = ((float3)(rotation * soup.vertices[index])).xy;

                builder.Append($"({Convert.ToString((Decimal)vertex.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex.y, CultureInfo.InvariantCulture)})");
                builder.Append(", ");
            }
            { 
                var index = categorized_loop.indices[0];
                var vertex = ((float3)(rotation * soup.vertices[index])).xy;
                builder.Append($"({Convert.ToString((Decimal)vertex.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex.y, CultureInfo.InvariantCulture)})");
            }
            builder.AppendLine("             ");
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
        internal static void Intersect(VertexSoup brushVertices, List<Loop> loopsOnBrushSurface, Loop surfaceLoop, Loop intersectionLoop, CategoryGroupIndex newHoleCategory)
        {
            // It might look like we could just set the interiorCategory of brush_intersection here, and let all other cut loops copy from it below,
            // but the same brush_intersection might be used by another categorized_loop and then we'd try to reroute it again, which wouldn't work
            //brush_intersection.interiorCategory = newHoleCategory;

            // TODO: input polygons are convex, and intersection of both polygons will always be convex.
            //       Make an intersection boolean operation that assumes everything is convex
            s_OverlappingArea.Clear();
            OperationResult result;
            result = CSGManagerPerformCSG.PerformBooleanIntersection(brushVertices,
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
                        var newPolygon = new Loop(surfaceLoop) { interiorCategory = newHoleCategory };
                        loopsOnBrushSurface.Add(newPolygon);
                    }
                    surfaceLoop.ClearAllIndices();
                    return;
                }

                case CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2:
                {
                    var newPolygon = new Loop(intersectionLoop) { interiorCategory = newHoleCategory };
                    s_OverlappingArea.Add(newPolygon);
                    break;
                }

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
                        surfaceLoop.interiorCategory = newHoleCategory;
                    } else
                    { 
                        overlappingLoop.interiorCategory = newHoleCategory;
                        overlappingLoop.holes.AddRange(surfaceLoop.holes);
                        var newPolygon = new Loop(overlappingLoop) { interiorCategory = newHoleCategory };
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
        internal static void CleanUp(VertexSoup soup, List<Loop>[] surfaceLoops)
        {
            for (int surfaceIndex = 0; surfaceIndex < surfaceLoops.Length; surfaceIndex++)
                CleanUp(soup, surfaceLoops[surfaceIndex], surfaceIndex);
        }

        internal static void CleanUp(VertexSoup soup, List<Loop> baseloops, int surfaceIndex)
        {
            for (int l = baseloops.Count - 1; l >= 0; l--)
            {
                var baseloop = baseloops[l];
                if (baseloop.edges.Count < 3)
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
                }

                baseloop.destroyed = new bool[baseloop.edges.Count];
                for (int h1 = holes.Count - 1; h1 >= 0; h1--)
                {
                    var hole1 = holes[h1];
                    hole1.destroyed = new bool[hole1.edges.Count];
                    CSGManagerPerformCSG.Subtract(soup, baseloop, hole1);
                }
                
                // TODO: optimize, keep track which holes (potentially) intersect
                for (int h1 = holes.Count - 1; h1 >= 0; h1--)
                {
                    var hole1 = holes[h1];
                    for (int h2 = holes.Count - 1; h2 > h1; h2--)
                    {
                        var hole2 = holes[h2];
                        CSGManagerPerformCSG.Merge(soup, hole1, hole2);
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
                    var edges = holes[h].edges;

                    // Note: can have duplicate edges when multiple holes share an edge
                    //          (only edges between holes and base-loop are guaranteed to not be duplciate)
                    baseloop.AddEdges(edges, removeDuplicates: true);
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

                var interiorCategory = baseloop.interiorCategory - 1;
                baseloop.interiorCategory = interiorCategory;
            }
        }

        static void Subtract(VertexSoup soup, Loop loop1, Loop loop2)
        {
            if (loop1.edges.Count == 0 ||
                loop2.edges.Count == 0)
                return;

            var categories1 = new EdgeCategory[loop1.edges.Count];
            var categories2 = new EdgeCategory[loop2.edges.Count];


            // 1. there are edges with two identical vertex indices?
            // 2. all edges should overlap, but some aren't overlapping?

            // TODO: use something that assumes convexity instead,
            //          current method sometimes fails
            for (int e = 0; e < loop1.edges.Count; e++)
            {
                categories1[e] = loop2.CategorizeEdge(soup, loop1.edges[e]);
            }

            for (int e = 0; e < loop2.edges.Count; e++)
            {
                categories2[e] = loop1.CategorizeEdge(soup, loop2.edges[e]);
            }
            
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


            // TODO: reversed edges should cancel?
            for (int e = loop1.edges.Count - 1; e >= 0; e--)
            {
                if (categories1[e] == EdgeCategory.Outside
                    //|| categories1[e] == EdgeCategory.Aligned
                    //|| categories1[e] == EdgeCategory.ReverseAligned
                    )
                    continue;
                loop1.destroyed[e] = true;
            }

            for (int e = loop2.edges.Count - 1; e >= 0; e--)
            {
                if (categories2[e] == EdgeCategory.Inside
                    //|| categories1[e] == EdgeCategory.Outside
                    //|| categories1[e] == EdgeCategory.Aligned
                    //|| categories1[e] == EdgeCategory.ReverseAligned
                    )
                    continue;
                loop2.destroyed[e] = true;
            }
        }

        static void Merge(VertexSoup soup, Loop loop1, Loop loop2)
        {
            if (loop1.edges.Count == 0 ||
                loop2.edges.Count == 0)
                return;

            var categories1 = new EdgeCategory[loop1.edges.Count];
            var categories2 = new EdgeCategory[loop2.edges.Count];


            for (int e = 0; e < loop1.edges.Count; e++)
            {
                categories1[e] = loop2.CategorizeEdge(soup, loop1.edges[e]);
            }

            for (int e = 0; e < loop2.edges.Count; e++)
            {
                categories2[e] = loop1.CategorizeEdge(soup, loop2.edges[e]);
            }


            for (int e = loop1.edges.Count - 1; e >= 0; e--)
            {
                if (categories1[e] == EdgeCategory.Outside
                    //|| categories1[e] == EdgeCategory.ReverseAligned
                    || categories1[e] == EdgeCategory.Aligned
                    )
                    continue;
                loop1.destroyed[e] = true;
            }

            for (int e = loop2.edges.Count - 1; e >= 0; e--)
            {
                if (categories2[e] == EdgeCategory.Outside
                    //|| categories1[e] == EdgeCategory.ReverseAligned
                    //|| categories1[e] == EdgeCategory.Aligned
                    )
                    continue;
                loop2.destroyed[e] = true;
            }
        }

        #endregion
    }
#endif
}
