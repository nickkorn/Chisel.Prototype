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

        #region FindHoleDependencies

        struct LoopDependency
        {
            public int      startIndex;
            public int[]    dependencies;
        }

        static void FillDependencyList(List<int> dependencies, int baseIndex, HashSet<int> skipSet, int[][] intersections)
        {
            if (dependencies.Count == intersections.Length)
                return;

            var baseIntersections = intersections[baseIndex];
            if (baseIntersections != null)
            {
                var startIndex = dependencies.Count;
                for (int i = 0; i < baseIntersections.Length; i++)
                {
                    var loopIndex = baseIntersections[i];
                    if (!skipSet.Add(loopIndex))
                        continue;

                    dependencies.Add(loopIndex);
                }

                // TODO: make this non-recursive
                // We add the child dependencies later, to ensure the order of the entire list is correct
                var endIndex = dependencies.Count;
                for (int i = startIndex; i < endIndex; i++)
                    FillDependencyList(dependencies, dependencies[i], skipSet, intersections);
            }
        }

        static void FindHoleDependencies(VertexSoup soup, List<Loop> holes, List<LoopDependency> loopDependencies)
        {
            loopDependencies.Clear();

            // TODO: optimize
            // Finds all the intersections between all polygons
            int[][] intersections = new int[holes.Count][];
            for (int a = 0; a < holes.Count; a++)
            {
                if (!holes[a].Valid)
                {
                    Debug.Log($"hole[{a}] is not valid");
                    continue;
                }

                var foundIntersections = new List<int>();
                for (int b = 0; b < holes.Count; b++)
                {
                    if (a == b)
                        continue;

                    if (!holes[b].Valid)
                    {
                        Debug.Log($"hole[{b}] is not valid");
                        continue;
                    }

                    if (Intersects(holes[a], holes[b]) ||
                        Intersects(holes[b], holes[a]))
                    {
                        foundIntersections.Add(b);
                    } else
                    {
                        if (IsInside(soup, holes[a], holes[b]) ||
                            IsInside(soup, holes[b], holes[a]))
                        {
                            foundIntersections.Add(b);
                        }

                    }
                }
                intersections[a] = foundIntersections.ToArray();
            }

            // Creates a list of dependencies, in order
            var skipSet = new HashSet<int>();
            var dependencies = new List<int>();
            for (int a = 0; a < holes.Count; a++)
            {
                dependencies.Clear();

                skipSet.Clear();
                skipSet.Add(a);

                FillDependencyList(dependencies, a, skipSet, intersections);

                loopDependencies.Add(new LoopDependency()
                {
                    startIndex = 0,
                    dependencies = dependencies.ToArray()
                });
            }
        }

        #endregion


        public enum OperationResult
        {
            Fail,
            Cut,
            Outside,
            Polygon1InsidePolygon2,
            Polygon2InsidePolygon1,
            Overlapping
        }

        [Flags]
        internal enum PointFlags : byte
        {
            None = 0,

            On = 1,
            Inside = 2,
            Outside = 4,

            Inwards = 8,    // we reached this point going into the shape
            Outwards = 16,   // after this point we'll exit the shape

            Entering = 32,
            Removed = 64
        }

        public const double kEpsilon = 0.00001;
        public const double kEpsilon2 = 0.001;
        public const double kEpsilonSqr = kEpsilon * kEpsilon;

        public unsafe static OperationResult PerformBooleanOperation(VertexSoup soup, Loop polygon1, Loop polygon2, List<Loop> resultLoops, CSGOperationType operationType)//, bool debug = false)
        {
            UnityEngine.Profiling.Profiler.BeginSample("PerformBooleanOperation");
            try
            {
                Loop newPolygon = null;
                if (operationType == CSGOperationType.Intersecting)
                {
                    Debug.Assert(polygon1.convex && polygon2.convex);
                    if (polygon1.convex && polygon2.convex)
                    {
                        bool isOverlapping = true;
                        for (int i = 0; i < polygon1.indices.Count; i++)
                        {
                            if (polygon2.indices.IndexOf(polygon1.indices[i]) == -1)
                            {
                                isOverlapping = false;
                                break;
                            }
                        }
                        if (isOverlapping)
                            return CSGManagerPerformCSG.OperationResult.Overlapping;

                        var brush1 = polygon1.info.brush;
                        var mesh1 = BrushMeshManager.GetBrushMesh(brush1.BrushMesh.BrushMeshID);
                        var nodeToTreeSpaceInversed1 = brush1.TreeToNodeSpaceMatrix;
                        var nodeToTreeSpace1 = brush1.NodeToTreeSpaceMatrix;

                        var worldSpacePlanes1Length = mesh1.surfaces.Length;
                        var worldSpacePlanes1 = stackalloc float4[worldSpacePlanes1Length];
                        CSGManagerPerformCSG.GetBrushPlanes(worldSpacePlanes1, mesh1.surfaces, nodeToTreeSpace1);

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
                        CSGManagerPerformCSG.GetBrushPlanes(worldSpacePlanes2, mesh2.surfaces, nodeToTreeSpace2);

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
                }

                var result = PerformBooleanOperation(soup, polygon1.info.right, polygon1.info.forward, polygon1, polygon2, resultLoops, operationType);//, debug);

                foreach (var overlappingLoop in resultLoops)
                {
                    overlappingLoop.interiorCategory = polygon1.interiorCategory;
                    overlappingLoop.info = polygon1.info;
                }

                polygon1.edges.Clear();
                polygon1.AddEdges(polygon1.indices);

                polygon2.edges.Clear();
                polygon2.AddEdges(polygon2.indices);
                /*
                var builder = new System.Text.StringBuilder();
                Quaternion rotation = Quaternion.identity;
                if (newPolygon != null)
                {
                    rotation = Quaternion.FromToRotation(newPolygon.info.worldPlane.normal, Vector3.forward);
                
                    builder.AppendLine($"{result}");
                
                    builder.AppendLine("polygon1:");
                    Dump(builder, polygon1, soup, rotation);

                    builder.AppendLine("polygon2:");
                    Dump(builder, polygon2, soup, rotation);
                
                    builder.AppendLine();
                    builder.AppendLine("newloop:");
                    Dump(builder, newPolygon, soup, rotation);
                }
                */
                if (resultLoops != null)
                {
                    for (int n = resultLoops.Count - 1; n >= 0; n--)
                    {
                        if (!resultLoops[n].Valid)
                        {
                            resultLoops.RemoveAt(n);
                            continue;
                        }
                        resultLoops[n].edges.Clear();
                        resultLoops[n].AddEdges(resultLoops[n].indices);
                        /*
                        if (newPolygon != null)
                        {
                            builder.AppendLine($"resultLoops[{n}]:");
                            Dump(builder, resultLoops[n], soup, rotation);
                        }*/
                    }
                }
                /*
                if (newPolygon != null)
                    Debug.Log(builder.ToString());
                    */
                return result;
            }
            finally { UnityEngine.Profiling.Profiler.EndSample(); }
        }

        //  outside       touching/               overlapping    polygon inside 
        //                outside                                polygon
        //
        //  *--*  *--*    *--*--*     *--* *      *-----*        *-----*
        //  |  |  |  |    |  |  |     |  |/|      |     |        | *-* |
        //  *--*  *--*    *--*--*     |  *-*      |     |        | | | |
        //                            *--*        |     |        | *-* | 
        //                                        *-----*        *-----*
        //  polygon intersecting
        //
        //    *-*                                             *-*         *-*
        //  *-|-|-*     *---*-*     *-*-*-*     *-*-*-*     *-|-|-*     *-|-|-*
        //  | | | |     |  /| |     | | | |     | | | |     | | | |     | | | | 
        //  | *-* |     | *-* |     | *-* |     | | | |     | | | |     | | | |
        //  *-----*     *-----*     *-----*     *-*-*-*     *-*-*-*     *-|-|-*
        //                                                                *-*

        // TODO: find all intersection vertices before trying to do boolean operations, should get rid of a lot of precision issues
        //          snap vertices/edges to edges on other brushes?

        static readonly List<PointFlags> s_PointFlags1 = new List<PointFlags>();
        static readonly List<PointFlags> s_PointFlags2 = new List<PointFlags>();

        static readonly List<CategoryIndex> s_EdgeFlags1 = new List<CategoryIndex>();
        static readonly List<CategoryIndex> s_EdgeFlags2 = new List<CategoryIndex>();

        public static OperationResult PerformBooleanOperation(VertexSoup soup, Vector3 right, Vector3 forward, Loop polygon1, Loop polygon2, List<Loop> resultLoops, CSGOperationType operationType)
        {
            // TODO: store 'loop' as a list of edges (2 ushorts to vertices)
            //          -> do CSG on edges being inside/on/outside other brush, don't bother with ordering
            //          -> use these edges with triangulation code

            if (!polygon1.Valid ||
                !polygon2.Valid)
                return OperationResult.Outside;

            // Get list of all points from this polygon (including all intersection points with other polygon)
            List<ushort> indices1, indices2;

            indices1 = polygon1.indices;//.ToList();
            indices2 = polygon2.indices;//.ToList(); 

            if (indices1.Count < 3 ||
                indices2.Count < 3)
                return OperationResult.Outside;

            DetermineOnEdgeFlags(//soup.vertices, 
                                indices2, indices1, s_PointFlags1);
            DetermineOnEdgeFlags(//soup.vertices, 
                                indices1, indices2, s_PointFlags2);

            var point1CrossingToIndex2 = new List<int>();//[vertices1.Count];
            var point2CrossingToIndex1 = new List<int>();//[vertices2.Count];
            if (point1CrossingToIndex2.Capacity < indices1.Count)
                point1CrossingToIndex2.Capacity = indices1.Count;
            if (point2CrossingToIndex1.Capacity < indices2.Count)
                point2CrossingToIndex1.Capacity = indices2.Count;
            for (int i = 0; i < indices1.Count; i++) point1CrossingToIndex2.Add(-1);
            for (int i = 0; i < indices2.Count; i++) point2CrossingToIndex1.Add(-1);


            Debug.Assert(s_PointFlags1.Count == indices1.Count);
            Debug.Assert(s_PointFlags2.Count == indices2.Count);

            int onEdgeCount = 0;
            for (int currIndex2 = indices2.Count - 1,
                     nextIndex2 = 0;

                     nextIndex2 < indices2.Count;

                     currIndex2 = nextIndex2,
                     nextIndex2++)
            {
                var flags2 = s_PointFlags2[currIndex2];
                if ((flags2 & PointFlags.On) == PointFlags.None)
                    continue;

                var currVertexIndex2 = indices2[currIndex2];
                //var currVertex2 = soup.vertices[indices2[currIndex2]];
                for (int currIndex1 = indices1.Count - 1,
                         nextIndex1 = 0;

                         nextIndex1 < indices1.Count;

                         currIndex1 = nextIndex1,
                         nextIndex1++)
                {
                    var flags1 = s_PointFlags1[currIndex1];
                    if ((flags1 & PointFlags.On) == PointFlags.None)
                        continue;

                    var currVertexIndex1 = indices1[currIndex1];
                    //var currVertex1 = soup.vertices[indices1[currIndex1]];
                    //if (currVertex1.Equals(currVertex2, kEpsilon))
                    if (currVertexIndex1 == currVertexIndex2)
                    {
                        point1CrossingToIndex2[currIndex1] = currIndex2;
                        point2CrossingToIndex1[currIndex2] = currIndex1;
                        onEdgeCount++;
                        break;
                    }
                }
            }


            if (s_EdgeFlags1.Capacity < s_PointFlags1.Count)
                s_EdgeFlags1.Capacity = s_PointFlags1.Count;
            if (s_EdgeFlags2.Capacity < s_PointFlags2.Count)
                s_EdgeFlags2.Capacity = s_PointFlags2.Count;

            int insideCount1 = 0;
            int insideCount2 = 0;
            DetermineEdgeFlags(soup, s_EdgeFlags1, s_PointFlags1, indices1, point1CrossingToIndex2, right, forward, indices2, point2CrossingToIndex1, ref insideCount1);
            DetermineEdgeFlags(soup, s_EdgeFlags2, s_PointFlags2, indices2, point2CrossingToIndex1, right, forward, indices1, point1CrossingToIndex2, ref insideCount2);


            // TODO: optimize, simplify

            int onPointCount1 = 0;
            int onEdgeCount1 = 0;
            int insideEdgeCount1 = 0;
            int notInsideEdgeCount1 = 0;
            int notOutsideEdgeCount1 = 0;
            int outsideEdgeCount1 = 0;
            for (int i = 0; i < s_PointFlags1.Count; i++)
            {
                if ((s_PointFlags1[i] & PointFlags.On) != PointFlags.None)
                    onPointCount1++;

                if (s_EdgeFlags1[i] == CategoryIndex.Aligned || s_EdgeFlags1[i] == CategoryIndex.ReverseAligned)
                    onEdgeCount1++;

                if (s_EdgeFlags1[i] != CategoryIndex. Inside) notInsideEdgeCount1++;
                if (s_EdgeFlags1[i] == CategoryIndex. Inside) insideEdgeCount1++;
                if (s_EdgeFlags1[i] == CategoryIndex.Outside) outsideEdgeCount1++;
                if (s_EdgeFlags1[i] != CategoryIndex.Outside) notOutsideEdgeCount1++;
            }

            int onPointCount2 = 0;
            int onEdgeCount2 = 0;
            int insideEdgeCount2 = 0;
            int notInsideEdgeCount2 = 0;
            int notOutsideEdgeCount2 = 0;
            int outsideEdgeCount2 = 0;
            for (int i = 0; i < s_PointFlags2.Count; i++)
            {
                if ((s_PointFlags2[i] & PointFlags.On) != PointFlags.None)
                    onPointCount2++;

                if (s_EdgeFlags2[i] == CategoryIndex.Aligned || s_EdgeFlags2[i] == CategoryIndex.ReverseAligned)
                    onEdgeCount2++;

                if (s_EdgeFlags2[i] != CategoryIndex. Inside) notInsideEdgeCount2++;
                if (s_EdgeFlags2[i] == CategoryIndex. Inside) insideEdgeCount2++;
                if (s_EdgeFlags2[i] == CategoryIndex.Outside) outsideEdgeCount2++;
                if (s_EdgeFlags2[i] != CategoryIndex.Outside) notOutsideEdgeCount2++;
            }



            // If both polygons have all edges "on edge", then they must be overlapping
            if (indices1.Count == onEdgeCount1 && indices1.Count == onPointCount1 &&
                indices2.Count == onEdgeCount2 && indices2.Count == onPointCount2)
            {
                Debug.Assert(indices1.Count == indices2.Count);
                return OperationResult.Overlapping;
            }

            // If all edges of both polygons are outside or "on edge", then they must be outside each other
            // (since they can't be overlapping)
            if (notInsideEdgeCount1 == indices1.Count &&
                notInsideEdgeCount2 == indices2.Count)
            {
                if (operationType != CSGOperationType.Additive ||
                    (outsideEdgeCount1 == indices1.Count &&
                     outsideEdgeCount2 == indices2.Count))
                    return OperationResult.Outside;
            }

            // If all edges of one polygon is inside and the other is outside, then one must be inside the other
            if (outsideEdgeCount1 == indices1.Count && insideEdgeCount2 == indices2.Count) return OperationResult.Polygon2InsidePolygon1;
            if (outsideEdgeCount2 == indices2.Count && insideEdgeCount1 == indices1.Count) return OperationResult.Polygon1InsidePolygon2;

            // If one of the polygons has all it's edges "on edge", but not the other, then one of the polygons must be super thin
            if (operationType != CSGOperationType.Additive)
            {
                if (indices1.Count == onPointCount1 && notOutsideEdgeCount2 == 0) { return OperationResult.Outside; }
                if (indices2.Count == onPointCount2 && notOutsideEdgeCount1 == 0) { return OperationResult.Outside; }
            }


            // Since some of the edges must be intersecting or on/inside we have to cut
            int innerDirection = (operationType == CSGOperationType.Subtractive) ? -1 : 1;
            int outerDirection = 1;

            int crossingCount = 0;
            DeterminePolygonCrossOvers(operationType, s_EdgeFlags1, s_PointFlags1, point1CrossingToIndex2, ref crossingCount, primary: true);
            DeterminePolygonCrossOvers(operationType, s_EdgeFlags2, s_PointFlags2, point2CrossingToIndex1, ref crossingCount, primary: false);
            


            //*
            if (onEdgeCount == 0)
            {
                if (indices1.Count == insideCount1)
                {
                    bool found = true;
                    if (indices2.Count == insideCount1)
                    {
                        for (int i = 0; i < s_PointFlags1.Count; i++)
                        {
                            if ((s_PointFlags1[i] & PointFlags.On) == PointFlags.None &&
                                s_EdgeFlags1[i] != CategoryIndex.Inside)
                            {
                                found = false;
                                break;
                            }
                        }
                    }
                    if (found)
                        return OperationResult.Polygon1InsidePolygon2;
                }

                if (indices2.Count == insideCount2)
                {
                    bool found = true;
                    if (indices1.Count == insideCount2)
                    {
                        for (int i = 0; i < s_PointFlags2.Count; i++)
                        {
                            if ((s_PointFlags2[i] & PointFlags.On) == PointFlags.None &&
                                s_EdgeFlags2[i] != CategoryIndex.Inside)
                            {
                                found = false;
                                break;
                            }
                        }
                    }
                    if (found)
                        return OperationResult.Polygon2InsidePolygon1;
                }
            } else
            {
                if (indices1.Count <= onEdgeCount + insideCount1)
                {
                    bool found = true;
                    //if (vertices.Count <= onEdgeCount + insideCount1)
                    {
                        for (int i = 0; i < s_PointFlags1.Count; i++)
                        {
                            if ((s_PointFlags1[i] & PointFlags.On) == PointFlags.None &&
                                s_EdgeFlags1[i] != CategoryIndex.Inside)
                            {
                                found = false;
                                break;
                            }
                        }
                    }
                    if (found)
                        return OperationResult.Polygon1InsidePolygon2;
                }
            }


            resultLoops.Clear();
            if (crossingCount > 0)
            {/*
                if (polygon1.loopIndex == 121 &&
                    polygon2.loopIndex == 125)
                {
                    var builder = new System.Text.StringBuilder();
                    var rotation = Quaternion.FromToRotation(polygon1.info.worldPlane.normal, Vector3.forward);
                    Dump(builder, polygon1, soup, rotation);
                    Dump(builder, polygon2, soup, rotation);
                    Debug.Log(builder.ToString());
                }*/
                Combine(soup,
                        indices1, point1CrossingToIndex2, polygon1.loopIndex,
                        indices2, point2CrossingToIndex1, polygon2.loopIndex,
                        resultLoops, innerDirection, outerDirection, operationType);
            }


            if (resultLoops.Count != 0)
                return OperationResult.Cut;

            return OperationResult.Outside;
        }


        private static readonly bool[][,] IntersectingCrossOver  = new bool[][,]
        {
            new bool[,]
            //  From:               Reverse-
            //  Inside     Aligned  Aligned  Outside
            {                                          // To:
                { false,   true,    true,    true  },  // Inside
                { false,   false,   false,   true  },  // Aligned
                { false,   false,   false,   false },  // ReverseAligned
                { false,   false,   false,   false },  // Outside 
            },
            new bool[,]
            //  From:               Reverse-
            //  Inside     Aligned  Aligned  Outside
            {                                          // To:
                { false,   true,    true,    true  },  // Inside
                { false,   false,   false,   true  },  // Aligned
                { false,   false,   false,   false },  // ReverseAligned
                { false,   false,   false,   false },  // Outside 
            }
        };
        private static readonly bool[][,] AdditiveCrossOver      = new bool[][,]
        {
            new bool[,]
            //  From:               Reverse-
            //  Inside     Aligned  Aligned  Outside
            {                                          // To:
                { false,   false,   false,   false },  // Inside
                { true,    false,   true,    false },  // Aligned
                { false,   false,   false,   false },  // ReverseAligned
                { true,    false,   true,    false },  // Outside
            },
            new bool[,]
            //  From:               Reverse-
            //  Inside     Aligned  Aligned  Outside
            {                                          // To:
                { false,   false,   false,   false },  // Inside
                { true,    false,   true,    false },  // Aligned
                { false,   false,   false,   false },  // ReverseAligned
                { true,    false,   true,    false },  // Outside
            }
        };
        private static readonly bool[][,] SubtractiveCrossOver   = new bool[][,]
        {
            new bool[,]
            //  From:               Reverse-
            //  Inside     Aligned  Aligned  Outside
            {                                          // To:
                { false,   false,   false,   false },  // Inside
                { false,   false,   false,   false },  // Aligned
                { true,    true,    false,   false },  // ReverseAligned
                { false,   true,    false,   false },  // Outside
            },
            new bool[,]
            //  From:               Reverse-
            //  Inside     Aligned  Aligned  Outside
            {                                          // To:
                { false,   false,   false,   false },  // Inside
                { true,    true,    true,    false },  // Aligned
                { true,    true,    true,    false },  // ReverseAligned
                { false,   true,    true,    false },  // Outside
            }
        };

        private static readonly bool[][][,] EdgeOperations =
        {
            AdditiveCrossOver,      // CSGOperationType.Additive == 0
            SubtractiveCrossOver,   // CSGOperationType.Subtractive == 1
            IntersectingCrossOver   // CSGOperationType.Intersecting == 2
        };

        private static void DeterminePolygonCrossOvers(CSGOperationType operationType, List<CategoryIndex> edgeFlags, List<PointFlags> pointFlags, List<int> crossingToOtherLoop, ref int crossingCount, bool primary)
        {
            Debug.Assert(pointFlags.Count == edgeFlags.Count && crossingToOtherLoop.Count >= edgeFlags.Count);

            var operationTable = EdgeOperations[(int)operationType][primary ? 0 : 1];
            for (int fromIndex = pointFlags.Count - 2, toIndex = pointFlags.Count - 1, p2 = 0; p2 < pointFlags.Count; fromIndex = toIndex, toIndex = p2, p2++)
            {
                // from-edge goes from A to B, to-edge goes from B-C
                // point1CrossingToIndex2 is index B
                if (crossingToOtherLoop[fromIndex] == -1)
                    continue;
                    
                var fromFlags = edgeFlags[fromIndex];
                var toFlags   = edgeFlags[toIndex];

                if (!operationTable[(int)fromFlags, (int)toFlags])
                {
                    crossingToOtherLoop[fromIndex] = -1;
                    continue;
                }
                    
                pointFlags[fromIndex] |= PointFlags.Entering;
                crossingCount++;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Intersects(Loop polygon1, Loop polygon2)
        {
            if (!polygon2.Valid ||
                !polygon1.Valid)
                return false;

            var edges1 = polygon1.edges;
            var edges2 = polygon2.edges;

            for (int i = 0; i < polygon1.edges.Count; i++)
            {
                if (edges1[i].index1 == edges1[i].index2)
                    continue;

                var vertexIndex1 = edges1[i].index1;
                var vertexIndex2 = edges1[i].index2;

                for (int j = 0; j < edges2.Count; j++)
                {
                    var vertexIndex3 = edges2[j].index1;
                    var vertexIndex4 = edges2[j].index2;

                    // Does given line segment intersect this line segment?
                    if (vertexIndex3 == vertexIndex1 || vertexIndex3 == vertexIndex2 ||
                        vertexIndex4 == vertexIndex1 || vertexIndex4 == vertexIndex2)
                        return true;
                }
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool IsInside(VertexSoup soup, Loop polygon1, Loop polygon2)
        {
            if (!polygon2.Valid ||
                !polygon1.Valid)
                return false;

            var vertices = soup.vertices;
            var edges1 = polygon1.edges;

            if (IsPointInPolygon(polygon2.info.right, polygon2.info.forward, polygon2.indices, soup, vertices[edges1[0].index1]))
                return true;

            return false;
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

        static void DetermineOnEdgeFlags(//List<Vector3> vertices, 
                                            List<ushort> indices1, List<ushort> indices2, List<PointFlags> pointFlags)
        {
            pointFlags.Clear();
            if (pointFlags.Capacity < indices2.Count)
                pointFlags.Capacity = indices2.Count;

            for (int i = 0; i < indices2.Count; i++)
                pointFlags.Add(PointFlags.None);

            for (int i = indices2.Count - 1, j = 0; j < indices2.Count; i = j, j++)
            {
                var index1 = indices2[i];
                //var vertex1 = vertices[index1];
                for (int a = indices1.Count - 1, b = 0; b < indices1.Count; a = b, b++)
                {
                    var index2 = indices1[a];
                    if (index1 == index2)// || vertices[index2].Equals(vertex1, kEpsilon))
                        pointFlags[i] = PointFlags.On;
                }
            }
        }

        static void DetermineEdgeFlags(VertexSoup soup, List<CategoryIndex> edgeFlags1, List<PointFlags> pointFlags1, 
                                       List<ushort> indices1, List<int> point1CrossingToIndex2, Vector3 right, Vector3 forward,
                                       List<ushort> indices2, List<int> point2CrossingToIndex1, ref int insideCount)
        {
            bool all_on = true;
            for (int i = 0; i < pointFlags1.Count; i++)
            {
                if (point1CrossingToIndex2[i] == -1)
                {
                    all_on = false;
                    continue;
                }
                Debug.Assert((pointFlags1[i] & PointFlags.On) == PointFlags.On);
            }

            var vertices = soup.vertices;

            edgeFlags1.Clear();
            for (int a1 = pointFlags1.Count - 1, b1 = 0; b1 < pointFlags1.Count; a1 = b1, b1++)
            {
                if ((pointFlags1[a1] & PointFlags.On) == PointFlags.On &&
                    (pointFlags1[b1] & PointFlags.On) == PointFlags.On)
                {
                    var a2 = point1CrossingToIndex2[a1];
                    var b2 = point1CrossingToIndex2[b1];
                    if (a2 >= 0 && b2 >= 0)
                    {

                        if ((b2 - a2) == 1 ||
                            (a2 == point2CrossingToIndex1.Count - 1 && b2 == 0))
                        {
                            edgeFlags1.Add(CategoryIndex.Aligned);
                            continue;
                        } else
                        if ((a2 - b2) == 1 ||
                            (b2 == point2CrossingToIndex1.Count - 1 && a2 == 0))
                        {
                            edgeFlags1.Add(CategoryIndex.ReverseAligned);
                            continue;
                        }

                        //*
                        if (all_on)
                        {
                            var va1 = vertices[indices1[a1]];
                            var vb1 = vertices[indices1[b1]];
                            bool degenerate_polygon = true;
                            for (int i = 0; i < indices2.Count; i++)
                            {
                                if (GeometryMath.SqrDistanceFromPointToLineSegment(vertices[indices2[i]], va1, vb1) > kEpsilonSqr)
                                {
                                    degenerate_polygon = false;
                                    break;
                                }
                            }
                            if (degenerate_polygon)
                            {
                                // TODO: how to determine if it's aligned or reverse aligned?
                                edgeFlags1.Add(CategoryIndex.Aligned);
                                continue;
                            }
                        }
                        //*/
                    }
                }

                if (IsPointInPolygon(right, forward, indices2, soup, (vertices[indices1[a1]] + vertices[indices1[b1]]) * 0.5f))
                {
                    edgeFlags1.Add(CategoryIndex.Inside);
                    insideCount++;
                } else
                {
                    edgeFlags1.Add(CategoryIndex.Outside);
                }
            }
        }


        static void Combine(VertexSoup soup,
                            List<ushort> indices1, List<int> point1CrossingToIndex2, int loopIndex1,
                            List<ushort> indices2, List<int> point2CrossingToIndex1, int loopIndex2,
                            List<Loop> newPolygons, int innerDirection, int outerDirection, CSGOperationType operationType)
        {
            int maxVertices = indices1.Count + indices2.Count;

            var loopPolyIndex = 0;
            while (true)
            {
                if (loopPolyIndex > point1CrossingToIndex2.Count) { Debug.Log($"Infinite loop bug (poly) {loopIndex1} {loopIndex2}"); break; }
                loopPolyIndex++;
                 
                int currIndex1 = -1;
                for (int i = 0; i < point1CrossingToIndex2.Count; i++)
                {
                    if (point1CrossingToIndex2[i] >= 0)
                    {
                        if (point2CrossingToIndex1[point1CrossingToIndex2[i]] == -2)
                        {
                            point1CrossingToIndex2[i] = -2;
                        } else
                        {
                            currIndex1 = i;
                            break;
                        }
                    }
                }

                if (currIndex1 < 0)
                    break;

                var loopVertIndex   = 0;
                var newPolygon      = new Loop();
                newPolygon.indices.Capacity = maxVertices;
                var newIndices  = newPolygon.indices;
                int firstIndex1 = -1; 
                for (; currIndex1 != firstIndex1; currIndex1 = Step(currIndex1, outerDirection, point1CrossingToIndex2.Count))
                {
                    if (firstIndex1 < 0)
                        firstIndex1 = (int)currIndex1;

                    if (loopVertIndex > maxVertices) { Debug.Log($"Infinite loop bug (outer) {loopIndex1} {loopIndex2}"); break; }
                    loopVertIndex++;

                    var currVertexIndex1 = indices1[currIndex1];
                    //var currVert1 = soup.vertices[indices1[currIndex1]];
                    if (newIndices.Count == 0 ||
                        (newIndices[0] != currVertexIndex1 &&
                         newIndices[newIndices.Count - 1] != currVertexIndex1))
                        //(!soup.vertices[newIndices[0]].Equals(currVert1, kEpsilon) &&
                        // !soup.vertices[newIndices[newIndices.Count - 1]].Equals(currVert1, kEpsilon)))
                    {
                        newPolygon.indices.Add(currVertexIndex1);
                    }

                    int crossingIndexTo2 = point1CrossingToIndex2[currIndex1];

                    if (crossingIndexTo2 < 0)
                    {
                        if (crossingIndexTo2 == -2)
                            goto ExitLoop;

                        continue;
                    }

                    int currIndex2 = crossingIndexTo2;
                    if (point2CrossingToIndex1[currIndex2] == -2)
                        continue;
                    
                    var startIndex1 = currIndex1;
                    do
                    {
                        if (loopVertIndex > maxVertices) { Debug.Log($"Infinite loop bug (inner) {loopIndex1} {loopIndex2}"); break; }
                        loopVertIndex++;

                        currIndex2 = Step(currIndex2, innerDirection, point2CrossingToIndex1.Count);
                        if (currIndex2 == crossingIndexTo2)
                        {
                            if (operationType == CSGOperationType.Subtractive)
                                break;
                            
                            point1CrossingToIndex2[startIndex1] = -2;
                            goto ExitLoop;
                        }

                        var currVertexIndex2 = indices2[currIndex2];
                        //var currVert2 = soup.vertices[indices2[currIndex2]];
                        if (newIndices.Count == 0 ||
                            (newIndices[0] != currVertexIndex2 &&
                             newIndices[newIndices.Count - 1] != currVertexIndex2))
                            //(!soup.vertices[newIndices[0]].Equals(currVert2, kEpsilon) &&
                            // !soup.vertices[newIndices[newIndices.Count - 1]].Equals(currVert2, kEpsilon)))
                        {
                            newPolygon.indices.Add(currVertexIndex2);
                        }

                        var crossingIndexTo1 = point2CrossingToIndex1[currIndex2];

                        if (crossingIndexTo1 >= 0)
                        {
                            currIndex1 = crossingIndexTo1;
                            break;
                        }
                    } while (true);

                    point1CrossingToIndex2[startIndex1] = -2;
                }
                
            ExitLoop:
                ;
                
                if (IsDegenerate(soup, newPolygon.indices))
                    continue;

                newPolygons.Add(newPolygon);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int Step(int index, int offset, int count)
        {
            return (index + count + offset) % count;
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
            /*
            for (int i = 0; i < categorized_loop.indices.Count; i++)
            {
                if (i > 0)
                    builder.Append(",");
                var index = categorized_loop.indices[i];
                var vertex = soup.vertices[index];

                builder.Append($"({(Decimal)vertex.x}, {(Decimal)vertex.y}, {(Decimal)vertex.z})");
            }
            builder.AppendLine("             ");
            */
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
            result = CSGManagerPerformCSG.PerformBooleanOperation(brushVertices,
                                                                    intersectionLoop, surfaceLoop,
                                                                    s_OverlappingArea,    // the output of cutting operations are both holes for the original polygon (categorized_loop)
                                                                                        // and new polygons on the surface of the brush that need to be categorized
                                                                    CSGOperationType.Intersecting);
            Debug.Assert(s_OverlappingArea.Count <= 1);

            // FIXME: when brush_intersection and categorized_loop are grazing each other, 
            //          technically we cut it but we shouldn't be creating it as a separate polygon + hole (bug7)

            switch (result)
            {
                case CSGManagerPerformCSG.OperationResult.Fail:     return;
                case CSGManagerPerformCSG.OperationResult.Outside: return;

                case CSGManagerPerformCSG.OperationResult.Polygon2InsidePolygon1:
                    {
                        // This new piece overrides the current loop
                        if (surfaceLoop.Valid)
                            loopsOnBrushSurface.Add(new Loop(surfaceLoop) { interiorCategory = newHoleCategory });
                        surfaceLoop.ClearAllIndices();
                        return;
                    }

                case CSGManagerPerformCSG.OperationResult.Overlapping:
                case CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2:
                    {
                        s_OverlappingArea.Add(new Loop(intersectionLoop) { interiorCategory = newHoleCategory });
                        break;
                    }

                case CSGManagerPerformCSG.OperationResult.Cut:
                    {
                        break;
                    }
            }

            // the output of cutting operations are both holes for the original polygon (categorized_loop)
            // and new polygons on the surface of the brush that need to be categorized
            if (surfaceLoop.holes.Capacity < surfaceLoop.holes.Count + s_OverlappingArea.Count)
                surfaceLoop.holes.Capacity = surfaceLoop.holes.Count + s_OverlappingArea.Count;

            for (int o = 0; o < s_OverlappingArea.Count; o++)
            {
                var overlappingLoop = s_OverlappingArea[o];
                if (!overlappingLoop.Valid)
                    continue;

                overlappingLoop.interiorCategory = newHoleCategory;
                surfaceLoop.holes.Add(new Loop(overlappingLoop));  // but it is also a hole for our polygon
                loopsOnBrushSurface.Add(overlappingLoop);               // this loop is a polygon on its own
            }
        }
        #endregion

        #region CleanUp

        static readonly List<Loop> resultLoops2 = new List<Loop>();
        static readonly List<Loop> resultLoops1 = new List<Loop>();


        // Clean up, after performing CSG
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void CleanUp(VertexSoup soup, List<Loop>[] surfaceLoops)
        {
            for (int surfaceIndex = 0; surfaceIndex < surfaceLoops.Length; surfaceIndex++)
                CleanUp(soup, surfaceLoops[surfaceIndex]);
        }

        internal static void CleanUp(VertexSoup soup, List<Loop> loops)
        {
            //*
            var loopDependencies = new List<LoopDependency>(loops.Count);

            if (resultLoops1.Capacity < loops.Count * 2)
                resultLoops1.Capacity = loops.Count * 2;
            if (resultLoops2.Capacity < loops.Count * 2)
                resultLoops2.Capacity = loops.Count * 2;

            for (int l = loops.Count - 1; l >= 0; l--)
            {
                var categorized_loop = loops[l];
                var interiorCategory = categorized_loop.interiorCategory - 1;

                // Don't bother processing holes when this polygon will not be visible
                if ((interiorCategory != (CategoryGroupIndex)CategoryIndex.Aligned && 
                     interiorCategory != (CategoryGroupIndex)CategoryIndex.ReverseAligned) ||
                    !categorized_loop.Valid)
                {
                    // Just remove it since it won't be visible anyway
                    loops.RemoveAt(l);
                    continue;
                }

                var holes = categorized_loop.holes;
                if (holes.Count == 0)
                    continue;

                categorized_loop.convex = false;

                // If we have more than one hole, we need to merge those that touch, to avoid issues during triangulation
                if (holes.Count > 1)
                {
                    FindHoleDependencies(soup, holes, loopDependencies);

                    {
                        int a = 0;
                        while (true)
                        {
#if DEBUG
                            if (loopDependencies.Count > holes.Count * holes.Count * 2) { Debug.Log("Infinite loop bug"); break; }
#endif
                            if (a >= loopDependencies.Count ||
                                a >= holes.Count)
                                break;

                            var loopDependency = loopDependencies[a];
                            var dependencies = loopDependency.dependencies;

                            for (; loopDependency.startIndex < dependencies.Length; loopDependency.startIndex++)
                            {
                                var b = dependencies[loopDependency.startIndex];
                                if (!holes[a].Valid ||
                                    !holes[b].Valid)
                                    continue;

                                resultLoops1.Clear();
                                var resultType = CSGManagerPerformCSG.PerformBooleanOperation(soup, holes[a], holes[b], resultLoops1, CSGOperationType.Additive);
                                switch (resultType)
                                {
                                    case CSGManagerPerformCSG.OperationResult.Outside:
                                    case CSGManagerPerformCSG.OperationResult.Fail:
                                        continue;

                                    case CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2:
                                    {
                                        holes[a].ClearAllIndices();
                                        continue;
                                    }
                                    case CSGManagerPerformCSG.OperationResult.Overlapping:
                                    case CSGManagerPerformCSG.OperationResult.Polygon2InsidePolygon1:
                                        //holes[a].vertices.Clear();
                                        continue;
                                }

                                // We clear them since we can't easily remove them yet, since it would change the order of the indices
                                // TODO: find a better way to do this
                                holes[a].ClearAllIndices();
                                holes[b].ClearAllIndices();
                                loopDependency.startIndex++;

                                if (resultLoops1 != null &&
                                    resultLoops1.Count > 0)
                                {
                                    for (int n = 0; n < resultLoops1.Count; n++)
                                    {
                                        loopDependencies.Add(loopDependency);
                                        resultLoops1[n].CopyDetails(holes[a]);
                                    }

                                    for (int i = 0; i < resultLoops1.Count; i++)
                                    {
                                        if (!resultLoops1[i].Valid)
                                            continue;
                                        holes.Add(resultLoops1[i]);
                                    }
                                }
                                break;
                            }
                            a++;
                        }
                    }
                }

                if (!categorized_loop.Valid)
                {
                    loops.RemoveAt(l);
                    continue;
                }

                for (int h = holes.Count - 1; h >= 0; h--)
                {
                    if (!holes[h].Valid)
                    {
                        holes.RemoveAt(h);
                        continue;
                    }
                }

                if (holes.Count == 0)
                    continue;


                // Subtract holes that touch the outside of the polygon to avoid issues during triangulation
                // Since we already merged all touching holes, we don't need to worry about dependencies between holes here anymore
                {
                    var testLoops = new List<Loop>
                    {
                        new Loop(categorized_loop)
                    };
                    loops.RemoveAt(l);
                    for (int h = holes.Count - 1; h >= 0; h--)
                    {
                        for (int t = testLoops.Count - 1; t >= 0; t--)
                        {
                            if (!testLoops[t].Valid)
                            {
                                testLoops.RemoveAt(t);
                                continue;
                            }


                            var testLoop = testLoops[t];
                            var testHole = holes[h];

                            resultLoops2.Clear();
                            var resultType = CSGManagerPerformCSG.PerformBooleanOperation(soup, testLoop, testHole, resultLoops2, CSGOperationType.Subtractive);

                            switch (resultType)
                            {
                                case CSGManagerPerformCSG.OperationResult.Outside:
                                case CSGManagerPerformCSG.OperationResult.Fail:
                                    continue;

                                case CSGManagerPerformCSG.OperationResult.Overlapping:
                                {
                                    // our hole is completely overlapping us, we've just been eliminated.
                                    testLoops.RemoveAt(t);
                                    continue;
                                }
                                case CSGManagerPerformCSG.OperationResult.Polygon1InsidePolygon2:
                                    //holes.RemoveAt(h);
                                    continue;
                                case CSGManagerPerformCSG.OperationResult.Polygon2InsidePolygon1:
                                    // keep hole as hole
                                    testLoop.holes.Add(new Loop(testHole));
                                    continue;
                                default:
                                {
                                    testLoop.ClearAllIndices();
                                    foreach (var resultLoop in resultLoops2)
                                    {
                                        var temp = new Loop(resultLoop);
                                        foreach (var otherHole in testLoop.holes)
                                            temp.holes.Add(new Loop(otherHole));
                                        testLoops.Add(temp);
                                    }
                                    continue;
                                }
                            }
                        }
                    }

                    for (int t = 0; t < testLoops.Count; t++)
                    {
                        if (testLoops[t].Valid)
                            loops.Add(testLoops[t]);
                    }
                }
            }

            for (int l = 0; l < loops.Count; l++)
            {
                var loop = loops[l];
                if (!loop.Valid)
                {
                    loop.edges.Clear();
                    continue;
                }

                //loop.edges.Clear();
                //loop.AddEdges(loop.indices);

                for (int h = 0; h < loop.holes.Count; h++)
                {
                    var hole = loop.holes[h];
                    loop.AddEdges(hole.edges);
                }
            }
            /*/

            // TODO: figure out why code can't handle situation where polygon is split into multiple parts

            for (int l = 0; l < loops.Count; l++)
            {
                var loop = loops[l];
                if (!loop.Valid)
                {
                    loop.edges.Clear();
                    continue;
                }

                if (loop.holes.Count == 0)
                    continue;

                for (int h = loop.holes.Count - 1; h >= 0; h--)
                {
                    var hole = loop.holes[h];
                    if (!hole.Valid)
                        continue;

                    CSGManagerPerformCSG.Subtract(soup, loop, hole);
                }

                for (int h = loop.holes.Count - 1; h >= 0; h--)
                    loop.AddEdges(loop.holes[h].edges);

                loop.holes.Clear();
            }
            //*/

            for (int l = 0; l < loops.Count; l++)
            {
                var loop = loops[l];
                if (!loop.Valid)
                    continue;

                var interiorCategory = loop.interiorCategory - 1;
                loop.interiorCategory = interiorCategory;
            }
        }

        static void Subtract(VertexSoup soup, Loop loop1, Loop loop2)
        {
            if (loop1.edges.Count == 0 ||
                loop2.edges.Count == 0)
                return;

            var categories1 = new CategoryIndex[loop1.edges.Count];
            var categories2 = new CategoryIndex[loop2.edges.Count];


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
                if (categories1[e] == CategoryIndex.Outside)
                    continue;

                loop1.edges.RemoveAt(e);
            }

            for (int e = loop2.edges.Count - 1; e >= 0; e--)
            {
                if (categories2[e] == CategoryIndex.Inside)
                    continue;

                loop2.edges.RemoveAt(e);
            }
        }
        #endregion
    }
#endif
}
