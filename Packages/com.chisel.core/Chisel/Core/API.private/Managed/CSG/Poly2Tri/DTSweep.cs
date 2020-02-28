/* Poly2Tri
 * Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Sweep-line, Constrained Delauney Triangulation (CDT) See: Domiter, V. and
 * Zalik, B.(2008)'Sweep-line algorithm for constrained Delaunay triangulation',
 * International Journal of Geographical Information Science
 * 
 * "FlipScan" Constrained Edge Algorithm invented by author of this code.
 * 
 * Author: Thomas Åhlén, thahlen@gmail.com 
 */

/// Changes from the Java version
///   Turned DTSweep into a static class
///   Lots of deindentation via early bailout
/// Future possibilities
///   Comments!

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;

namespace Chisel.Core
{
    public struct Edge
    {
        public ushort index1;
        public ushort index2;
        public override string ToString() => $"({index1}, {index2})";
        
        internal void Flip()
        {
            //if (index1 < index2)
            //    return;
            var t = index1; index1 = index2; index2 = t;
        }
    }
}

namespace Poly2Tri
{
    public unsafe sealed class DTSweep
    {
        const double PI_div2     = (double)(Math.PI / 2);
        const double PI_3div4    = (double)(3 * Math.PI / 4);

        unsafe struct DelaunayTriangle
        {
            public enum EdgeFlags : byte
            {
                None = 0,
                Constrained = 1,
                Delaunay = 2
            }

            public fixed ushort indices[3];
            public fixed ushort neighbors[3];
            fixed byte          edgeFlags[3];

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public DelaunayTriangle(ushort p1, ushort p2, ushort p3)
            {
                indices[0] = p1;
                indices[1] = p2;
                indices[2] = p3;

                neighbors[0] = ushort.MaxValue;
                neighbors[1] = ushort.MaxValue;
                neighbors[2] = ushort.MaxValue;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ClearDelauney()
            {
                edgeFlags[0] &= (byte)~EdgeFlags.Delaunay;
                edgeFlags[1] &= (byte)~EdgeFlags.Delaunay;
                edgeFlags[2] &= (byte)~EdgeFlags.Delaunay;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetDelauneyEdge(int index, bool value)
            {
                if (value)
                    edgeFlags[index] |= (byte)EdgeFlags.Delaunay;
                else
                    edgeFlags[index] &= (byte)~EdgeFlags.Delaunay;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetDelaunayEdge(int idx) { return (edgeFlags[idx] & (byte)EdgeFlags.Delaunay) != 0; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetConstrainedEdge(int index, bool value)
            {
                if (value)
                    edgeFlags[index] |= (byte)EdgeFlags.Constrained;
                else
                    edgeFlags[index] &= (byte)~EdgeFlags.Constrained;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetConstrainedEdge(int idx) { return (edgeFlags[idx] & (byte)EdgeFlags.Constrained) != 0; }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ushort IndexOf(ushort p)
            {
                if (indices[0] == p) return (ushort)0; else if (indices[1] == p) return (ushort)1; else if (indices[2] == p) return (ushort)2;
                return ushort.MaxValue;
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Contains(ushort p)
            {
                if (indices[0] == p || indices[1] == p || indices[2] == p) 
                    return true;
                return false;
            }


            /// <summary>
            /// Update neighbor pointers
            /// </summary>
            /// <param name="p1">Point 1 of the shared edge</param>
            /// <param name="p2">Point 2 of the shared edge</param>
            /// <param name="t">This triangle's new neighbor</param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void MarkNeighbor(ushort p1, ushort p2, ushort triangleIndex)
            {
                ushort i = EdgeIndex(p1, p2);
                if (i == ushort.MaxValue)
                {
                    throw new Exception("Error marking neighbors -- t doesn't contain edge p1-p2!");
                }
                neighbors[i] = triangleIndex;
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ushort NeighborCWFrom(ushort point)
            {
                return neighbors[(IndexOf(point) + 1) % 3];
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ushort NeighborCCWFrom(ushort point)
            {
                return neighbors[(IndexOf(point) + 2) % 3];
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ushort NeighborAcrossFrom(ushort point)
            {
                return neighbors[IndexOf(point)];
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ushort PointCCWFrom(ushort point)
            {
                return indices[(IndexOf(point) + 1) % 3];
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ushort PointCWFrom(ushort point)
            {
                return indices[(IndexOf(point) + 2) % 3];
            }

            /// <summary>
            /// Legalize triangle by rotating clockwise around oPoint
            /// </summary>
            /// <param name="oPoint">The origin point to rotate around</param>
            /// <param name="nPoint">???</param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Legalize(ushort oPoint, ushort nPoint)
            {
                //RotateCW();
                {
                    var index0 = indices[0];
                    var index1 = indices[1];
                    var index2 = indices[2];

                    indices[2] = index1;
                    indices[1] = index0;
                    indices[0] = index2;
                }

                indices[(IndexOf(oPoint) + 1) % 3] = nPoint;
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void MarkConstrainedEdge(int index) { SetConstrainedEdge(index, true); }



            /// <summary>
            /// Mark edge as constrained
            /// </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void MarkConstrainedEdge(ushort p, ushort q)
            {
                ushort i = EdgeIndex(p, q);
                if (i != ushort.MaxValue)
                    SetConstrainedEdge(i, true);
            }



            /// <summary>
            /// Get the index of the neighbor that shares this edge (or ushort.MaxValue if it isn't shared)
            /// </summary>
            /// <returns>index of the shared edge or ushort.MaxValue if edge isn't shared</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ushort EdgeIndex(ushort p1, ushort p2)
            {
                ushort i1 = IndexOf(p1);
                ushort i2 = IndexOf(p2);

                // Points of this triangle in the edge p1-p2
                bool a = (i1 == 0 || i2 == 0);
                bool b = (i1 == 1 || i2 == 1);
                bool c = (i1 == 2 || i2 == 2);

                if (b && c)
                {
                    return 0;
                }
                if (a && c)
                {
                    return 1;
                }
                if (a && b)
                {
                    return 2;
                }

                return ushort.MaxValue;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetConstrainedEdgeCCW(ushort p) { return GetConstrainedEdge((IndexOf(p) + 2) % 3); }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetConstrainedEdgeCW(ushort p) { return GetConstrainedEdge((IndexOf(p) + 1) % 3); }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetConstrainedEdgeCCW(ushort p, bool ce)
            {
                int idx = (IndexOf(p) + 2) % 3;
                SetConstrainedEdge(idx, ce);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetConstrainedEdgeCW(ushort p, bool ce)
            {
                int idx = (IndexOf(p) + 1) % 3;
                SetConstrainedEdge(idx, ce);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetConstrainedEdgeAcross(ushort p, bool ce)
            {
                int idx = IndexOf(p);
                SetConstrainedEdge(idx, ce);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetDelaunayEdgeCCW(ushort p) { return GetDelaunayEdge((IndexOf(p) + 2) % 3); }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetDelaunayEdgeCW(ushort p) { return GetDelaunayEdge((IndexOf(p) + 1) % 3); }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetDelaunayEdgeCCW(ushort p, bool ce) { SetDelauneyEdge((IndexOf(p) + 2) % 3, ce); }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetDelaunayEdgeCW(ushort p, bool ce) { SetDelauneyEdge((IndexOf(p) + 1) % 3, ce); }
        
        }

        enum Orientation
        {
            CW,
            CCW,
            Collinear
        }

        struct DTSweepConstraint
        {
            public ushort P;
            public ushort Q;
        }

        struct AdvancingFrontNode
        {
            public ushort prevNodeIndex;
            public ushort nextNodeIndex;
            public ushort triangleIndex;
            public ushort pointIndex;
            public float2 nodePoint;
        }

        struct DirectedEdge
        {
            public ushort index2;
            public ushort next;
        }

        //
        // SweepContext
        //

        List<float3>                        vertices;
        float2[]                            points;
        ushort[]                            edges;
        List<DirectedEdge>                  allEdges                = new List<DirectedEdge>();
        readonly List<int>                  finalTriangleIndices    = new List<int>();
        readonly List<int>                  triangleIndices         = new List<int>();
        readonly List<DelaunayTriangle>     triangles               = new List<DelaunayTriangle>();
        readonly List<bool>                 triangleInterior        = new List<bool>();
        readonly List<ushort>               sortedPoints            = new List<ushort>();
        readonly List<AdvancingFrontNode>   advancingFrontNodes     = new List<AdvancingFrontNode>();


        // Inital triangle factor, seed triangle will extend 30% of 
        // PointSet width to both left and right.
        readonly float ALPHA = 0.3f;

        ushort headNodeIndex;
        ushort tailNodeIndex;
        ushort searchNodeIndex;
        ushort headPointIndex;
        ushort tailPointIndex;

        //Basin
        ushort leftNodeIndex;
        ushort bottomNodeIndex;
        ushort rightNodeIndex;
        float basinWidth;
        bool basinLeftHighest;

        DTSweepConstraint edgeEventConstrainedEdge;
        bool edgeEventRight;

        readonly Dictionary<ushort, List<Chisel.Core.Edge>> edgeLookups = new Dictionary<ushort, List<Chisel.Core.Edge>>();
        readonly List<List<Chisel.Core.Edge>> foundLoops = new List<List<Chisel.Core.Edge>>();
        

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe static bool IsPointInPolygon(float3 right, float3 forward, List<Chisel.Core.Edge> indices1, List<Chisel.Core.Edge> indices2, List<float3> vertices)
        {
            int index = 0;
            while (index < indices2.Count &&
                indices1.Contains(indices2[index]))
                index++;

            if (index >= indices2.Count)
                return false;

            var point = vertices[indices2[index].index1];

            var px = math.dot(right, point);
            var py = math.dot(forward, point);

            float ix, iy, jx, jy;

            var vert = vertices[indices1[indices1.Count - 1].index1];
            ix = math.dot(right,   vert);
            iy = math.dot(forward, vert);

            bool result = false;
            for (int i = 0; i < indices1.Count; i++)
            {
                jx = ix;
                jy = iy;

                vert = vertices[indices1[i].index1];
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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int[] TriangulateLoops(Chisel.Core.Loop loop, List<float3> vertices, List<Chisel.Core.Edge> inputEdges, quaternion rotation)
        {
            if (inputEdges.Count < 4)
            {
                return Triangulate(vertices, inputEdges, rotation).ToArray();
            }

            // This is a hack around bugs in the triangulation code

            finalTriangleIndices.Clear();
            edgeLookups.Clear();
            foundLoops.Clear();

            //if (inputEdges.Count == 5)
            //    Debug.Log($"{loop.info.brush.brushNodeID} {loop.info.worldPlane} {inputEdges.Count}");
            
            for (int i = 0; i < inputEdges.Count; i++)
            {
                if (!edgeLookups.TryGetValue(inputEdges[i].index1, out List<Chisel.Core.Edge> edges))
                {
                    edges = new List<Chisel.Core.Edge>();
                    edgeLookups[inputEdges[i].index1] = edges;
                }
                edges.Add(inputEdges[i]);
            }


            while (inputEdges.Count > 0)
            {
                var lastIndex = inputEdges.Count - 1;
                var edge = inputEdges[lastIndex];
                var newLoops = new List<Chisel.Core.Edge> { edge };

                var edgesStartingAtVertex = edgeLookups[edge.index1];
                if (edgesStartingAtVertex.Count > 1)
                    edgesStartingAtVertex.Remove(edge);
                else
                    edgeLookups.Remove(edge.index1);

                inputEdges.RemoveAt(lastIndex);
                foundLoops.Add(newLoops);

                var firstIndex = edge.index1;
                while (edgeLookups.ContainsKey(edge.index2))
                { 
                    var nextEdges = edgeLookups[edge.index2];
                    Chisel.Core.Edge nextEdge = nextEdges[0];
                    if (nextEdges.Count > 1)
                    {
                        var vertex1 = vertices[edge.index1];
                        var vertex2 = vertices[edge.index2];
                        var vertex3 = vertices[nextEdge.index2];
                        var prevAngle = math.dot((vertex2 - vertex1), (vertex1 - vertex3));
                        for (int i = 1; i < nextEdges.Count; i++)
                        {
                            vertex3 = vertices[nextEdge.index2];
                            var currAngle = math.dot((vertex2 - vertex1), (vertex3 - vertex1));
                            if (currAngle > prevAngle)
                            {
                                nextEdge = nextEdges[i];
                            }
                        }
                        nextEdges.Remove(nextEdge);
                    } else
                        edgeLookups.Remove(edge.index2);
                    newLoops.Add(nextEdge);
                    inputEdges.Remove(nextEdge);
                    edge = nextEdge;
                    if (edge.index2 == firstIndex)
                        break;
                }
            }

            if (foundLoops.Count <= 1)
            {
                return Triangulate(vertices, foundLoops[0], rotation).ToArray();
            }

            var children = new List<int>[foundLoops.Count];
            for (int i = 0; i < foundLoops.Count; i++)
                children[i] = new List<int>();


            var right = loop.info.right;
            var forward = loop.info.forward;
            for (int l1 = foundLoops.Count - 1; l1 >= 0; l1--)
            {
                if (foundLoops[l1].Count == 0)
                    continue;
                for (int l2 = l1 - 1; l2 >= 0; l2--)
                {
                    if (foundLoops[l2].Count == 0)
                        continue;
                    if (IsPointInPolygon(right, forward, foundLoops[l1], foundLoops[l2], vertices))
                    {
                        //children[l1].AddRange(children[l2]);
                        children[l1].Add(l2);
                        //foundLoops[l1].AddRange(foundLoops[l2]);
                        //children[l2].Clear();
                    } else
                    if (IsPointInPolygon(right, forward, foundLoops[l2], foundLoops[l1], vertices))
                    {
                        //children[l2].AddRange(children[l1]);
                        children[l2].Add(l1);
                        //foundLoops[l2].AddRange(foundLoops[l1]);
                        //children[l1].Clear();
                        break;
                    }
                }
            }

            for (int l1 = children.Length - 1; l1 >= 0; l1--)
            {
                if (children[l1].Count > 0) children[l1].Remove(l1); // just in case
                if (children[l1].Count > 0)
                {
                    int startOffset = 0;
                    while (startOffset < children[l1].Count)
                    {
                        var nextOffset = children[l1].Count;
                        for (int l2 = nextOffset - 1; l2 >= startOffset; l2--)
                        {
                            var index = children[l1][l2];
                            if (children[index].Count > 0)
                            {
                                children[l1].AddRange(children[index]);
                                children[l1].Remove(l1); // just in case
                                children[index].Clear();
                            }
                        }
                        startOffset = nextOffset;
                    }

                    for (int l2 = 0; l2 < children[l1].Count; l2++)
                    {
                        var index = children[l1][l2];
                        foundLoops[l1].AddRange(foundLoops[index]);
                        foundLoops[index].Clear();
                    }
                }
            }


            for (int l1 = foundLoops.Count - 1; l1 >= 0; l1--)
            {
                if (foundLoops[l1].Count == 0)
                    continue;
                finalTriangleIndices.AddRange(Triangulate(vertices, foundLoops[l1], rotation));
            }
            
            return finalTriangleIndices.ToArray();
        }

        /// <summary>
        /// Triangulate simple polygon with holes
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public List<int> Triangulate(List<float3> vertices, List<Chisel.Core.Edge> inputEdges, quaternion rotation)
        {
            this.vertices = vertices;
            this.rotation = rotation;

            triangleIndices.Clear();
            int prevEdgeCount = inputEdges.Count;
            AddMoreTriangles:
            Clear(vertices.Count);
            PrepareTriangulation(inputEdges);
            CreateAdvancingFront(0);
            Sweep();
            FixupConstrainedEdges();
            FinalizationPolygon();

            //TODO: Optimize
            for (int i = 0; i < triangleIndices.Count; i += 3)
            {
                var index0 = triangleIndices[i + 0];
                var index1 = triangleIndices[i + 1];
                var index2 = triangleIndices[i + 2];
                for (int e = inputEdges.Count - 1; e >= 0; e--)
                {
                    var edge = inputEdges[e];
                    if (index0 == edge.index1)
                    {
                        if (index1 == edge.index2 ||
                            index2 == edge.index2)
                        {
                            inputEdges.RemoveAt(e);
                        }
                    } else
                    if (index1 == edge.index1)
                    {
                        if (index0 == edge.index2 ||
                            index2 == edge.index2)
                        {
                            inputEdges.RemoveAt(e);
                        }
                    } else
                    if (index2 == edge.index1)
                    {
                        if (index1 == edge.index2 ||
                            index0 == edge.index2)
                        {
                            inputEdges.RemoveAt(e);
                        }
                    }
                }
            }
            if (inputEdges.Count > 3 && prevEdgeCount != inputEdges.Count)
            {
                prevEdgeCount = inputEdges.Count;
                goto AddMoreTriangles;
            }
            return triangleIndices;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Clear(int pointCount)
        {
            if (edges == null || 
                edges.Length < pointCount + 2)
                edges = new ushort[pointCount + 2];

            for (int i = 0; i < pointCount + 2; i++)
                edges[i] = ushort.MaxValue;

            allEdges.Clear();
            if (allEdges.Capacity < pointCount + 2)
                allEdges.Capacity = pointCount + 2;
            triangles.Clear();
            triangleInterior.Clear();
            advancingFrontNodes.Clear();
            sortedPoints.Clear();
            if (sortedPoints.Capacity < pointCount + 2)
                sortedPoints.Capacity = pointCount + 2;
            if (points == null ||
                points.Length < pointCount + 2)
                points = new float2[pointCount + 2];
            if (triangles.Capacity < pointCount * 3)
                triangles.Capacity = pointCount * 3;
            if (triangleInterior.Capacity < triangles.Capacity)
                triangleInterior.Capacity = triangles.Capacity;
            if (triangleIndices.Capacity < triangles.Capacity * 3)
                triangleIndices.Capacity = triangles.Capacity * 3;
        }

        quaternion rotation;



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddTriangle(in DelaunayTriangle triangle)
        {
            triangles.Add(triangle);
            triangleInterior.Add(false);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool HasNext(ushort nodeIndex) { if (nodeIndex == ushort.MaxValue) return false; return advancingFrontNodes[nodeIndex].nextNodeIndex != ushort.MaxValue; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool HasPrev(ushort nodeIndex) { if (nodeIndex == ushort.MaxValue) return false; return advancingFrontNodes[nodeIndex].prevNodeIndex != ushort.MaxValue; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ushort LocateNode(float x)
        {
            var nodeIndex = searchNodeIndex;
            if (nodeIndex >= advancingFrontNodes.Count)
                return ushort.MaxValue;
            if (x < advancingFrontNodes[nodeIndex].nodePoint.x)
            {
                nodeIndex = advancingFrontNodes[nodeIndex].prevNodeIndex;
                while (nodeIndex != ushort.MaxValue)
                {
                    if (x >= advancingFrontNodes[nodeIndex].nodePoint.x)
                    {
                        searchNodeIndex = nodeIndex;
                        return nodeIndex;
                    }
                    nodeIndex = advancingFrontNodes[nodeIndex].prevNodeIndex;
                }
            } else
            {
                nodeIndex = advancingFrontNodes[nodeIndex].nextNodeIndex;
                while (nodeIndex != ushort.MaxValue)
                {
                    if (x < advancingFrontNodes[nodeIndex].nodePoint.x)
                    {
                        searchNodeIndex = advancingFrontNodes[nodeIndex].prevNodeIndex;
                        return advancingFrontNodes[nodeIndex].prevNodeIndex;
                    }
                    nodeIndex = advancingFrontNodes[nodeIndex].nextNodeIndex;
                }
            }

            return ushort.MaxValue;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal ushort CreateAdvancingFrontNode(float2 point, ushort pointIndex, ushort prevIndex, ushort nextIndex)
        {
            var newIndex = (ushort)advancingFrontNodes.Count;
            advancingFrontNodes.Add(new AdvancingFrontNode() { nodePoint = point, pointIndex = pointIndex, nextNodeIndex = nextIndex, prevNodeIndex = prevIndex, triangleIndex = ushort.MaxValue });
            return newIndex;
        }


        /// <summary>
        /// This implementation will use simple node traversal algorithm to find a point on the front
        /// </summary>
        ushort LocatePoint(ushort index)
        {
            var px          = points[index].x;
            var nodeIndex   = searchNodeIndex;
            var nx          = advancingFrontNodes[nodeIndex].nodePoint.x;

            if (px == nx)
            {
                if (index != advancingFrontNodes[nodeIndex].pointIndex)
                {
                    UnityEngine.Debug.Assert(advancingFrontNodes[nodeIndex].prevNodeIndex != ushort.MaxValue, "advancingFrontNodes[nodeIndex].prevNodeIndex != ushort.MaxValue");
                    // We might have two nodes with same x value for a short time
                    if (advancingFrontNodes[nodeIndex].prevNodeIndex != ushort.MaxValue &&
                        index == advancingFrontNodes[advancingFrontNodes[nodeIndex].prevNodeIndex].pointIndex)
                    {
                        nodeIndex = advancingFrontNodes[nodeIndex].prevNodeIndex;
                    }
                    else
                    {
                        UnityEngine.Debug.Assert(advancingFrontNodes[nodeIndex].nextNodeIndex != ushort.MaxValue, "advancingFrontNodes[nodeIndex].nextNodeIndex != ushort.MaxValue");
                        if (advancingFrontNodes[nodeIndex].nextNodeIndex != ushort.MaxValue &&
                        index == advancingFrontNodes[advancingFrontNodes[nodeIndex].nextNodeIndex].pointIndex)
                        {
                            nodeIndex = advancingFrontNodes[nodeIndex].nextNodeIndex;
                        }
                        else
                        {
                            throw new Exception("Failed to find Node for given afront point");
                        }
                    }
                }
            } else 
            if (px < nx)
            {
                nodeIndex = advancingFrontNodes[nodeIndex].prevNodeIndex;
                while (nodeIndex != ushort.MaxValue)
                {
                    if (index == advancingFrontNodes[nodeIndex].pointIndex)
                        break;
                    nodeIndex = advancingFrontNodes[nodeIndex].prevNodeIndex;
                }
            } else
            {
                nodeIndex = advancingFrontNodes[nodeIndex].nextNodeIndex;
                while (nodeIndex != ushort.MaxValue)
                {
                    if (index == advancingFrontNodes[nodeIndex].pointIndex)
                        break;
                    nodeIndex = advancingFrontNodes[nodeIndex].nextNodeIndex;
                }
            }
            searchNodeIndex = nodeIndex;

            return nodeIndex;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void MeshClean(ushort triangleIndex)
        {
            if (triangleIndex == ushort.MaxValue || triangleInterior[triangleIndex])
                return;
            
            triangleInterior[triangleIndex] = true;

            var triangle = triangles[triangleIndex];

            var index0 = triangle.indices[0];
            var index1 = triangle.indices[1];
            var index2 = triangle.indices[2];
            if (index0 < vertices.Count &&
                index1 < vertices.Count &&
                index2 < vertices.Count)
            {
                triangleIndices.Add(index0);
                triangleIndices.Add(index1);
                triangleIndices.Add(index2);
            }/*else
            { 
                UnityEngine.Debug.LogWarning($"invalid triangle {index0} {index1} {index2} / {vertices.Count}");
            }*/

            if (!triangle.GetConstrainedEdge(0)) MeshClean(triangle.neighbors[0]);
            if (!triangle.GetConstrainedEdge(1)) MeshClean(triangle.neighbors[1]);
            if (!triangle.GetConstrainedEdge(2)) MeshClean(triangle.neighbors[2]);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void CreateAdvancingFront(int index)
        {
            // Initial triangle
            var triangleIndex = (ushort)triangles.Count;
            AddTriangle(new DelaunayTriangle(sortedPoints[index], tailPointIndex, headPointIndex));

            headNodeIndex = (ushort)advancingFrontNodes.Count;
            var middleNodeIndex = (ushort)(headNodeIndex + 1);
            tailNodeIndex = (ushort)(middleNodeIndex + 1);

            var triangle = triangles[triangleIndex];
            var index0 = triangle.indices[0];
            var index1 = triangle.indices[1];
            var index2 = triangle.indices[2];

            advancingFrontNodes.Add(new AdvancingFrontNode() { nodePoint = points[index1], pointIndex = index1, triangleIndex = triangleIndex, prevNodeIndex = ushort.MaxValue, nextNodeIndex = middleNodeIndex });
            advancingFrontNodes.Add(new AdvancingFrontNode() { nodePoint = points[index0], pointIndex = index0, triangleIndex = triangleIndex, prevNodeIndex = headNodeIndex, nextNodeIndex = tailNodeIndex });
            advancingFrontNodes.Add(new AdvancingFrontNode() { nodePoint = points[index2], pointIndex = index2, triangleIndex = ushort.MaxValue, prevNodeIndex = middleNodeIndex, nextNodeIndex = ushort.MaxValue });

            searchNodeIndex = headNodeIndex;
        }


        /// <summary>
        /// Try to map a node to all sides of this triangle that don't have 
        /// a neighbor.
        /// </summary>
        void MapTriangleToNodes(ushort triangleIndex)
        {
            var triangle = triangles[triangleIndex];
            if (triangle.neighbors[0] == ushort.MaxValue)
            {
                var index = triangle.indices[0];
                var nodeIndex = LocatePoint(triangle.PointCWFrom(index));
                if (nodeIndex != ushort.MaxValue)
                {
                    var node = advancingFrontNodes[nodeIndex];
                    node.triangleIndex = triangleIndex;
                    advancingFrontNodes[nodeIndex] = node;
                }
            }

            if (triangle.neighbors[1] == ushort.MaxValue)
            {
                var index = triangle.indices[1];
                var nodeIndex = LocatePoint(triangle.PointCWFrom(index));
                if (nodeIndex != ushort.MaxValue)
                {
                    var node = advancingFrontNodes[nodeIndex];
                    node.triangleIndex = triangleIndex;
                    advancingFrontNodes[nodeIndex] = node;
                }
            }

            if (triangle.neighbors[2] == ushort.MaxValue)
            {
                var index = triangle.indices[2];
                var nodeIndex = LocatePoint(triangle.PointCWFrom(index));
                if (nodeIndex != ushort.MaxValue)
                {
                    var node = advancingFrontNodes[nodeIndex];
                    node.triangleIndex = triangleIndex;
                    advancingFrontNodes[nodeIndex] = node;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int PointComparerMethod(ushort i1, ushort i2)
        {
            var pt1 = points[i1];
            var pt2 = points[i2];
            if (pt1.y < pt2.y) return -1; if (pt1.y > pt2.y) return 1;
            if (pt1.x < pt2.x) return -1; if (pt1.x > pt2.x) return 1;
            return 0;
        }


        static bool[] s_KnownVertices;
        void PrepareTriangulation(List<Chisel.Core.Edge> inputEdges)
        {
            var min = new float2(float.PositiveInfinity, float.PositiveInfinity);
            var max = new float2(float.NegativeInfinity, float.NegativeInfinity);

            if (s_KnownVertices == null ||
                s_KnownVertices.Length < vertices.Count)
                s_KnownVertices = new bool[vertices.Count];
            else
                Array.Clear(s_KnownVertices, 0, vertices.Count);
            for (int e = 0; e < inputEdges.Count; e++)
            {
                var edge = inputEdges[e];
                var index1 = edge.index1;
                var index2 = edge.index2;

                if (index1 == index2)
                    continue;

                if (!s_KnownVertices[index1])
                {
                    s_KnownVertices[index1] = true;
                    sortedPoints.Add(index1);
                    var pt = math.mul(rotation, vertices[index1]).xy;
                    points[index1] = pt;

                    // Calculate bounds
                    max = math.max(max, pt);
                    min = math.min(min, pt);
                }
                if (!s_KnownVertices[index2])
                {
                    s_KnownVertices[index2] = true;
                    sortedPoints.Add(index2);
                    var pt = math.mul(rotation, vertices[index2]).xy;
                    points[index2] = pt;

                    // Calculate bounds
                    max = math.max(max, pt);
                    min = math.min(min, pt);
                }

                // Add constraints
                var p1 = points[index1];
                var p2 = points[index2];

                ushort P, Q;
                if (p1.y > p2.y || (p1.y == p2.y && p1.x > p2.x)) { Q = index1; P = index2; }
                else { P = index1; Q = index2; }

                allEdges.Add(new DirectedEdge() { index2 = P, next = this.edges[Q] });
                this.edges[Q] = (ushort)(allEdges.Count - 1);
            }


            // Sort the points along y-axis
            sortedPoints.Sort(PointComparerMethod);

            headPointIndex = (ushort)(vertices.Count);
            tailPointIndex = (ushort)(vertices.Count + 1);

            var delta = ALPHA * (max - min);
            points[headPointIndex] = new float2(max.x + delta.x, min.y - delta.y);
            points[tailPointIndex] = new float2(min.x - delta.x, min.y - delta.y);
        }




        //
        // Sweep
        //

        
        /// <summary>
        /// Start sweeping the Y-sorted point set from bottom to top
        /// </summary>
        void Sweep()
        {
            var sortedPoints = this.sortedPoints;
            for (int i = 1; i < sortedPoints.Count; i++)
            {
                var pointIndex      = sortedPoints[i];
                var point           = points[pointIndex];
                var frontNodeIndex  = LocateNode(point.x);

                if (frontNodeIndex == ushort.MaxValue)
                    continue;

                var triangleIndex       = (ushort)triangles.Count;
                var frontNodeNextIndex  = advancingFrontNodes[frontNodeIndex].nextNodeIndex;
                AddTriangle(new DelaunayTriangle(pointIndex, advancingFrontNodes[frontNodeIndex].pointIndex, advancingFrontNodes[frontNodeNextIndex].pointIndex));

                MarkNeighbor(advancingFrontNodes[frontNodeIndex].triangleIndex, triangleIndex);

                var nodeIndex       = CreateAdvancingFrontNode(point, pointIndex, frontNodeIndex, frontNodeNextIndex);

                {
                    var frontNodeNext = advancingFrontNodes[frontNodeNextIndex];
                    if (nodeIndex == frontNodeIndex)
                    {
                        UnityEngine.Debug.Assert(nodeIndex != frontNodeIndex);
                    } else
                    {
                        frontNodeNext.prevNodeIndex = nodeIndex;
                        advancingFrontNodes[frontNodeNextIndex] = frontNodeNext;
                    }
                }
                {
                    var frontNode = advancingFrontNodes[frontNodeIndex];
                    if (nodeIndex == frontNodeIndex)
                    {
                        UnityEngine.Debug.Assert(nodeIndex != frontNodeIndex);
                    } else
                    {
                        frontNode.nextNodeIndex = nodeIndex;
                        advancingFrontNodes[frontNodeIndex] = frontNode;
                    }
                }

                if (!Legalize(triangleIndex))
                {
                    MapTriangleToNodes(triangleIndex); 
                }

                // Only need to check +epsilon since point never have smaller 
                // x value than node due to how we fetch nodes from the front
                if ((point.x - advancingFrontNodes[frontNodeIndex].nodePoint.x) <= kEpsilon)
                {
                    Fill(frontNodeIndex);
                }

                {
                    // Fill right holes
                    { 
                        var iterator = advancingFrontNodes[nodeIndex].nextNodeIndex;
                        while (HasNext(iterator))
                        {
                            var angle = HoleAngle(iterator);
                            if (angle > PI_div2 || angle < -PI_div2)
                            {
                                break;
                            }
                            Fill(iterator);
                            iterator = advancingFrontNodes[iterator].nextNodeIndex;
                        }
                    }

                    // Fill left holes
                    {
                        var iterator = advancingFrontNodes[nodeIndex].prevNodeIndex;
                        while (HasPrev(iterator))
                        {
                            var angle = HoleAngle(iterator);
                            if (angle > PI_div2 || angle < -PI_div2)
                            {
                                break;
                            }
                            Fill(iterator);
                            iterator = advancingFrontNodes[iterator].prevNodeIndex;
                        }
                    }

                    // Fill right basins
                    if (HasNext(nodeIndex) && HasNext(advancingFrontNodes[nodeIndex].nextNodeIndex))
                    {
                        var angle = BasinAngle(nodeIndex);
                        if (angle < PI_3div4)
                        {
                            FillBasin(nodeIndex);
                        }
                    }
                }

                var edgeIndex = this.edges[pointIndex];
                while (edgeIndex != ushort.MaxValue)
                {
                    var pIndex  = allEdges[edgeIndex].index2;
                    edgeIndex = allEdges[edgeIndex].next;
                    var qIndex  = pointIndex;
                    var edge    = new DTSweepConstraint() { P = pIndex, Q = qIndex };
                    
                    edgeEventConstrainedEdge = edge;

                    var P       = points[pIndex];
                    var Q       = points[qIndex];
                    edgeEventRight = P.x > Q.x;

                    if (IsEdgeSideOfTriangle(advancingFrontNodes[nodeIndex].triangleIndex, pIndex, qIndex))
                        continue;

                    // For now we will do all needed filling
                    // TODO: integrate with flip process might give some better performance 
                    //       but for now this avoid the issue with cases that needs both flips and fills
                    if (edgeEventRight)
                        FillRightAboveEdgeEvent(edge, nodeIndex);
                    else
                        FillLeftAboveEdgeEvent(edge, nodeIndex);


                    PerformEdgeEvent(pIndex, qIndex, advancingFrontNodes[nodeIndex].triangleIndex, qIndex);
                }
            }
        }


        void FixupConstrainedEdges()
        {
            for (int t = 0; t < triangles.Count; t++)
            {
                var triangle = triangles[t];
                var index0  = triangle.indices[0];
                var index1  = triangle.indices[1];
                var index2  = triangle.indices[2];
                if (!triangle.GetConstrainedEdgeCCW(index0) && HasEdgeCCW(t, index0))
                {
                    triangle.MarkConstrainedEdge(2);
                    triangles[t] = triangle;
                }

                if (!triangle.GetConstrainedEdgeCCW(index1) && HasEdgeCCW(t, index1))
                {
                    triangle.MarkConstrainedEdge(0);
                    triangles[t] = triangle;
                }

                if (!triangle.GetConstrainedEdgeCCW(index2) && HasEdgeCCW(t, index2))
                {
                    triangle.MarkConstrainedEdge(1);
                    triangles[t] = triangle;
                }
            }
        }


        void FinalizationPolygon()
        {
            // Get an Internal triangle to start with
            var headNextNode    = advancingFrontNodes[advancingFrontNodes[headNodeIndex].nextNodeIndex];
            var pointIndex      = headNextNode.pointIndex;
            var triangleIndex   = headNextNode.triangleIndex;

            while (!triangles[triangleIndex].GetConstrainedEdgeCW(pointIndex))
            {
                var ccwNeighborIndex = triangles[triangleIndex].NeighborCCWFrom(pointIndex);
                if (ccwNeighborIndex == ushort.MaxValue)
                    break;
                triangleIndex = ccwNeighborIndex;
            }

            // Collect interior triangles constrained by edges
            MeshClean(triangleIndex);
        }


        
        void FillRightConcaveEdgeEvent(DTSweepConstraint edge, ushort nodeIndex)
        {
            var nodeNextIndex = advancingFrontNodes[nodeIndex].nextNodeIndex;
            Fill(nodeNextIndex); 
            nodeNextIndex = advancingFrontNodes[nodeIndex].nextNodeIndex;
            var nodeNext = advancingFrontNodes[nodeNextIndex];
            if (nodeNext.pointIndex != edge.P)
            {
                // Next above or below edge?
                if (Orient2d(points[edge.Q], nodeNext.nodePoint, points[edge.P]) == Orientation.CCW)
                {
                    var node         = advancingFrontNodes[nodeIndex];
                    var nodeNextNext = advancingFrontNodes[nodeNext.nextNodeIndex];
                    // Below
                    if (Orient2d(node.nodePoint, nodeNext.nodePoint, nodeNextNext.nodePoint) == Orientation.CCW)
                    {
                        // Next is concave
                        FillRightConcaveEdgeEvent(edge, nodeIndex);
                    }
                    else
                    {
                        // Next is convex
                    }
                }
            }
        }


        void FillRightConvexEdgeEvent(DTSweepConstraint edge, ushort nodeIndex)
        {
            var node                = advancingFrontNodes[nodeIndex];
            var nodeNext            = advancingFrontNodes[node.nextNodeIndex];
            var nodeNextNext        = advancingFrontNodes[nodeNext.nextNodeIndex];
            var nodeNextNextNext    = advancingFrontNodes[nodeNextNext.nextNodeIndex];
            // Next concave or convex?
            if (Orient2d(nodeNext.nodePoint, nodeNextNext.nodePoint, nodeNextNextNext.nodePoint) == Orientation.CCW)
            {
                // Concave
                FillRightConcaveEdgeEvent(edge, node.nextNodeIndex);
            } else
            {
                // Convex
                // Next above or below edge?
                if (Orient2d(points[edge.Q], nodeNextNext.nodePoint, points[edge.P]) == Orientation.CCW)
                {
                    // Below
                    FillRightConvexEdgeEvent(edge, node.nextNodeIndex);
                } else
                {
                    // Above
                }
            }
        }

        void FillRightBelowEdgeEvent(DTSweepConstraint edge, ushort nodeIndex)
        {
            var node = advancingFrontNodes[nodeIndex];
            if (node.nodePoint.x < points[edge.P].x)
            {
                var nodeNext     = advancingFrontNodes[node.nextNodeIndex];
                var nodeNextNext = advancingFrontNodes[nodeNext.nextNodeIndex];
                // needed?
                if (Orient2d(node.nodePoint, nodeNext.nodePoint, nodeNextNext.nodePoint) == Orientation.CCW)
                {
                    // Concave 
                    FillRightConcaveEdgeEvent(edge, nodeIndex);
                } else
                {
                    // Convex
                    FillRightConvexEdgeEvent(edge, nodeIndex);
                    // Retry this one
                    FillRightBelowEdgeEvent(edge, nodeIndex);
                }
            }
        }


        void FillRightAboveEdgeEvent(DTSweepConstraint edge, ushort nodeIndex)
        {
            var node = advancingFrontNodes[nodeIndex];
            var edgeP = points[edge.P];
            while (advancingFrontNodes[node.nextNodeIndex].nodePoint.x < edgeP.x)
            {
                // Check if next node is below the edge
                var o1 = Orient2d(points[edge.Q], advancingFrontNodes[node.nextNodeIndex].nodePoint, edgeP);
                if (o1 == Orientation.CCW)
                {
                    FillRightBelowEdgeEvent(edge, nodeIndex);
                } else
                    nodeIndex = node.nextNodeIndex;
                node = advancingFrontNodes[nodeIndex];
                edgeP = points[edge.P];
            }
        }

        
        void FillLeftConvexEdgeEvent(DTSweepConstraint edge, ushort nodeIndex)
        {
            var node             = advancingFrontNodes[nodeIndex];
            var nodePrev         = advancingFrontNodes[node.prevNodeIndex];
            var nodePrevPrev     = advancingFrontNodes[nodePrev.prevNodeIndex];
            var nodePrevPrevPrev = advancingFrontNodes[nodePrevPrev.prevNodeIndex];
            // Next concave or convex?
            if (Orient2d(nodePrev.nodePoint, nodePrevPrev.nodePoint, nodePrevPrevPrev.nodePoint) == Orientation.CW)
            {
                // Concave
                FillLeftConcaveEdgeEvent(edge, node.prevNodeIndex);
            } else
            {
                // Convex
                // Next above or below edge?
                if (Orient2d(points[edge.Q], nodePrevPrev.nodePoint, points[edge.P]) == Orientation.CW)
                {
                    // Below
                    FillLeftConvexEdgeEvent(edge, node.prevNodeIndex);
                }
                else
                {
                    // Above
                }
            }
        }

        
        void FillLeftConcaveEdgeEvent(DTSweepConstraint edge, ushort nodeIndex)
        {
            var node = advancingFrontNodes[nodeIndex];
            Fill(node.prevNodeIndex); 
            node = advancingFrontNodes[nodeIndex];
            var nodePrev = advancingFrontNodes[node.prevNodeIndex];
            if (nodePrev.pointIndex != edge.P)
            {
                // Next above or below edge?
                if (Orient2d(points[edge.Q], nodePrev.nodePoint, points[edge.P]) == Orientation.CW)
                {
                    var nodePrevPrev = advancingFrontNodes[nodePrev.prevNodeIndex];
                    // Below
                    if (Orient2d(node.nodePoint, nodePrev.nodePoint, nodePrevPrev.nodePoint) == Orientation.CW)
                    {
                        // Next is concave
                        FillLeftConcaveEdgeEvent(edge, nodeIndex);
                    }
                    else
                    {
                        // Next is convex
                    }
                }
            }
        }

        
        void FillLeftBelowEdgeEvent(DTSweepConstraint edge, ushort nodeIndex)
        {
            var node = advancingFrontNodes[nodeIndex];
            if (node.nodePoint.x > points[edge.P].x)
            {
                var nodePrev     = advancingFrontNodes[node.prevNodeIndex];
                var nodePrevPrev = advancingFrontNodes[nodePrev.prevNodeIndex];
                if (Orient2d(node.nodePoint, nodePrev.nodePoint, nodePrevPrev.nodePoint) == Orientation.CW)
                {
                    // Concave 
                    FillLeftConcaveEdgeEvent(edge, nodeIndex);
                }
                else
                {
                    // Convex
                    FillLeftConvexEdgeEvent(edge, nodeIndex);
                    // Retry this one
                    FillLeftBelowEdgeEvent(edge, nodeIndex);
                }

            }
        }

        
        void FillLeftAboveEdgeEvent(DTSweepConstraint edge, ushort nodeIndex)
        {
            var node = advancingFrontNodes[nodeIndex];
            var edgeP = points[edge.P];
            while (advancingFrontNodes[node.prevNodeIndex].nodePoint.x > edgeP.x)
            {
                // Check if next node is below the edge
                var o1 = Orient2d(points[edge.Q], advancingFrontNodes[node.prevNodeIndex].nodePoint, edgeP);
                if (o1 == Orientation.CW)
                    FillLeftBelowEdgeEvent(edge, nodeIndex);
                else
                    nodeIndex = node.prevNodeIndex;
                node = advancingFrontNodes[nodeIndex];
                edgeP = points[edge.P];
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool IsEdgeSideOfTriangle(ushort triangleIndex, ushort epIndex, ushort eqIndex)
        {
            if (triangleIndex == ushort.MaxValue)
                return false;

            ushort index = triangles[triangleIndex].EdgeIndex(epIndex, eqIndex);
            if (index == ushort.MaxValue)
                return false;

            var triangle = triangles[triangleIndex];
            triangle.MarkConstrainedEdge(index);
            triangles[triangleIndex] = triangle;

            triangleIndex = triangle.neighbors[index];
            if (triangleIndex != ushort.MaxValue)
            {
                triangle = triangles[triangleIndex];
                triangle.MarkConstrainedEdge(epIndex, eqIndex);
                triangles[triangleIndex] = triangle;
            }
            return true;
        }

        
        void PerformEdgeEvent(ushort epIndex, ushort eqIndex, ushort triangleIndex, ushort pointIndex)
        {
            UnityEngine.Debug.Assert(triangleIndex != ushort.MaxValue, "triangleIndex != ushort.MaxValue");
            if (triangleIndex == ushort.MaxValue)
                return;

            if (IsEdgeSideOfTriangle(triangleIndex, epIndex, eqIndex))
                return;

            var eqPoint = points[eqIndex];
            var epPoint = points[epIndex];

            var triangle = triangles[triangleIndex];
            var p1Index = triangle.PointCCWFrom(pointIndex);
            var o1 = Orient2d(eqPoint, points[p1Index], epPoint);
            if (o1 == Orientation.Collinear)
            {
                if (triangle.Contains(eqIndex) && triangle.Contains(p1Index))
                {
                    triangle.MarkConstrainedEdge(eqIndex, p1Index);
                    // We are modifying the constraint maybe it would be better to
                    // not change the given constraint and just keep a variable for the new constraint
                    edgeEventConstrainedEdge.Q = p1Index;
                    triangles[triangleIndex] = triangle;
                    triangleIndex = triangle.NeighborAcrossFrom(pointIndex);
                    PerformEdgeEvent(epIndex, p1Index, triangleIndex, p1Index);
                } else
                    throw new Exception($"PerformEdgeEvent - Point on constrained edge not supported yet {epIndex} {eqIndex} {p1Index}");
                return;
            }

            var p2Index = triangle.PointCWFrom(pointIndex);
            var o2 = Orient2d(eqPoint, points[p2Index], epPoint);
            if (o2 == Orientation.Collinear)
            {
                if (triangle.Contains(eqIndex) && triangle.Contains(p2Index))
                {
                    triangle.MarkConstrainedEdge(eqIndex, p2Index);
                    // We are modifying the constraint maybe it would be better to
                    // not change the given constraint and just keep a variable for the new constraint
                    edgeEventConstrainedEdge.Q = p2Index;
                    triangles[triangleIndex] = triangle;
                    triangleIndex = triangle.NeighborAcrossFrom(pointIndex);
                    if (triangleIndex != ushort.MaxValue)
                        PerformEdgeEvent(epIndex, p2Index, triangleIndex, p2Index);
                } else
                    throw new Exception($"PerformEdgeEvent - Point on constrained edge not supported yet {epIndex} {eqIndex} {p2Index}");
                
                return;
            }

            if (o1 == o2)
            {
                // Need to decide if we are rotating CW or CCW to get to a triangle
                // that will cross edge
                if (o1 == Orientation.CW)
                    triangleIndex = triangle.NeighborCCWFrom(pointIndex);
                else
                    triangleIndex = triangle.NeighborCWFrom(pointIndex);
                PerformEdgeEvent(epIndex, eqIndex, triangleIndex, pointIndex);
            } else
            {
                // This triangle crosses constraint so lets flippin start!
                FlipEdgeEvent(epIndex, eqIndex, triangleIndex, pointIndex);
            }
        }


        /// <param name="triangleIndex2">Opposite triangle</param>
        /// <param name="p">The point in t that isn't shared between the triangles</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ushort OppositePoint(ushort triangleIndex1, ushort triangleIndex2, ushort p)
        {
            return triangles[triangleIndex1].PointCWFrom(triangles[triangleIndex2].PointCWFrom(p));
        }


        void FlipEdgeEvent(ushort epIndex, ushort eqIndex, ushort triangleIndex, ushort pIndex)
        {
            var otIndex = triangles[triangleIndex].NeighborAcrossFrom(pIndex);            
            if (otIndex == ushort.MaxValue)
            {
                // If we want to integrate the fillEdgeEvent do it here
                // With current implementation we should never get here
                throw new InvalidOperationException("[BUG:FIXME] FLIP failed due to missing triangle");
            }

            UnityEngine.Debug.Assert(triangleIndex != otIndex, "self-pointer error");
            var opIndex = OppositePoint(otIndex, triangleIndex, pIndex);

            var opPoint = points[opIndex];
            bool inScanArea = InScanArea(points[pIndex],
                                         points[triangles[triangleIndex].PointCCWFrom(pIndex)],
                                         points[triangles[triangleIndex].PointCWFrom(pIndex)],
                                         opPoint);
            if (inScanArea)
            {
                // Lets rotate shared edge one vertex CW
                RotateTrianglePair(triangleIndex, pIndex, otIndex, opIndex);
                MapTriangleToNodes(triangleIndex); 
                MapTriangleToNodes(otIndex); 

                if (pIndex == eqIndex && opIndex == epIndex)
                {
                    if (eqIndex == edgeEventConstrainedEdge.Q &&
                        epIndex == edgeEventConstrainedEdge.P)
                    {
                        var triangle      = triangles[triangleIndex];
                        var otherTriangle = triangles[otIndex];
                        triangle     .MarkConstrainedEdge(epIndex, eqIndex);
                        otherTriangle.MarkConstrainedEdge(epIndex, eqIndex);
                        triangles[triangleIndex] = triangle;
                        triangles[otIndex] = otherTriangle;
                        Legalize(triangleIndex);
                        Legalize(otIndex);
                    }
                    else
                    {
                        // XXX: I think one of the triangles should be legalized here?
                    }
                }
                else
                {
                    var o = Orient2d(points[eqIndex], opPoint, points[epIndex]);
                    triangleIndex = NextFlipTriangle(o, triangleIndex, otIndex, pIndex, opIndex);
                    FlipEdgeEvent(epIndex, eqIndex, triangleIndex, pIndex);
                }
            }
            else
            {
                if (NextFlipPoint(epIndex, eqIndex, otIndex, opIndex, out ushort newP))
                {
                    FlipScanEdgeEvent(epIndex, eqIndex, triangleIndex, otIndex, newP);
                    PerformEdgeEvent(epIndex, eqIndex, triangleIndex, pIndex);
                }
            }
        }


        /// <summary>
        /// When we need to traverse from one triangle to the next we need 
        /// the point in current triangle that is the opposite point to the next
        /// triangle. 
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool NextFlipPoint(ushort epIndex, ushort eqIndex, ushort otherTriangleIndex, ushort opIndex, out ushort newP)
        {
            newP = ushort.MaxValue;
            var o2d = Orient2d(points[eqIndex], points[opIndex], points[epIndex]);
            switch (o2d)
            {
                case Orientation.CW:
                    newP = triangles[otherTriangleIndex].PointCCWFrom(opIndex);
                    return true;
                case Orientation.CCW:
                    newP = triangles[otherTriangleIndex].PointCWFrom(opIndex);
                    return true;
                case Orientation.Collinear:
                    // TODO: implement support for point on constraint edge
                    throw new NotImplementedException($"Point on constrained edge not supported yet {eqIndex} {opIndex} {epIndex}");
                default:
                    throw new NotImplementedException("Orientation not handled");
            }
        }


        /// <summary>
        /// After a flip we have two triangles and know that only one will still be
        /// intersecting the edge. So decide which to contiune with and legalize the other
        /// </summary>
        /// <param name="o">should be the result of an orient2d( eq, op, ep )</param>
        /// <param name="triangleIndex">triangle 1</param>
        /// <param name="otherTriangleIndex">triangle 2</param>
        /// <param name="p">a point shared by both triangles</param>
        /// <param name="op">another point shared by both triangles</param>
        /// <returns>returns the triangle still intersecting the edge</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ushort NextFlipTriangle(Orientation o, ushort triangleIndex, ushort otherTriangleIndex, ushort pIndex, ushort opIndex)
        {
            ushort edgeIndex;
            if (o == Orientation.CCW)
            {
                // ot is not crossing edge after flip
                var otherTriangle = triangles[otherTriangleIndex];
                edgeIndex = otherTriangle.EdgeIndex(pIndex, opIndex);
                otherTriangle.SetDelauneyEdge(edgeIndex, true);
                triangles[otherTriangleIndex] = otherTriangle;
                Legalize(otherTriangleIndex);
                otherTriangle = triangles[otherTriangleIndex];
                otherTriangle.ClearDelauney();
                triangles[otherTriangleIndex] = otherTriangle;
                return triangleIndex;
            }
            // t is not crossing edge after flip
            var triangle = triangles[triangleIndex];
            edgeIndex = triangle.EdgeIndex(pIndex, opIndex);
            triangle.SetDelauneyEdge(edgeIndex, true);
            triangles[triangleIndex] = triangle;
            Legalize(triangleIndex);
            triangle = triangles[triangleIndex];
            triangle.ClearDelauney();
            triangles[triangleIndex] = triangle;
            return otherTriangleIndex;
        }


        /// <summary>
        /// Scan part of the FlipScan algorithm<br>
        /// When a triangle pair isn't flippable we will scan for the next 
        /// point that is inside the flip triangle scan area. When found 
        /// we generate a new flipEdgeEvent
        /// </summary>
        /// <param name="ep">last point on the edge we are traversing</param>
        /// <param name="eq">first point on the edge we are traversing</param>
        /// <param name="flipTriangle">the current triangle sharing the point eq with edge</param>
        /// <param name="triangleIndex"></param>
        /// <param name="p"></param>
        void FlipScanEdgeEvent(ushort epIndex, ushort eqIndex, ushort flipTriangle, ushort triangleIndex, ushort pIndex)
        {
            var otIndex = triangles[triangleIndex].NeighborAcrossFrom(pIndex);
            if (otIndex == ushort.MaxValue)
            {
                // If we want to integrate the fillEdgeEvent do it here
                // With current implementation we should never get here
                throw new Exception("[BUG:FIXME] FLIP failed due to missing triangle");
            }

            UnityEngine.Debug.Assert(triangleIndex != otIndex, "self-pointer error");
            var opIndex = OppositePoint(otIndex, triangleIndex, pIndex);

            var inScanArea = InScanArea(points[eqIndex],
                                        points[triangles[flipTriangle].PointCCWFrom(eqIndex)],
                                        points[triangles[flipTriangle].PointCWFrom(eqIndex)],
                                        points[opIndex]);
            if (inScanArea)
            {
                // flip with new edge op->eq
                FlipEdgeEvent(eqIndex, opIndex, otIndex, opIndex);
                // TODO: Actually I just figured out that it should be possible to 
                //       improve this by getting the next ot and op before the the above 
                //       flip and continue the flipScanEdgeEvent here
                // set new ot and op here and loop back to inScanArea test
                // also need to set a new flipTriangle first
                // Turns out at first glance that this is somewhat complicated
                // so it will have to wait.
            }
            else
            {
                if (NextFlipPoint(epIndex, eqIndex, otIndex, opIndex, out ushort newP))
                {
                    var triangle = triangles[otIndex];
                    var index0 = triangle.indices[0];
                    var index1 = triangle.indices[1];
                    var index2 = triangle.indices[2];
                    if (index0 != index1 && index0 != index2 && index1 != index2)
                    {
                        FlipScanEdgeEvent(epIndex, eqIndex, flipTriangle, otIndex, newP);
                    }
                }
                //newP = NextFlipPoint(ep, eq, ot, op);
            }
        }


        /// <summary>
        /// Fills a basin that has formed on the Advancing Front to the right
        /// of given node.<br>
        /// First we decide a left,bottom and right node that forms the 
        /// boundaries of the basin. Then we do a reqursive fill.
        /// </summary>
        /// <param name="this"></param>
        /// <param name="node">starting node, this or next node will be left node</param>
        void FillBasin(ushort nodeIndex)
        {
            var node         = advancingFrontNodes[nodeIndex];
            var nodeNext     = advancingFrontNodes[node.nextNodeIndex];
            var nodeNextNext = advancingFrontNodes[nodeNext.nextNodeIndex];
            if (Orient2d(node.nodePoint, nodeNext.nodePoint, nodeNextNext.nodePoint) == Orientation.CCW)
            {
                leftNodeIndex = nodeIndex;
            }
            else
            {
                leftNodeIndex = node.nextNodeIndex;
            }

            // Find the bottom and right node
            bottomNodeIndex = leftNodeIndex;
            var bottomNode      = advancingFrontNodes[bottomNodeIndex];
            while (HasNext(bottomNodeIndex) && bottomNode.nodePoint.y >= advancingFrontNodes[bottomNode.nextNodeIndex].nodePoint.y)
            {
                bottomNodeIndex = bottomNode.nextNodeIndex;
                bottomNode = advancingFrontNodes[bottomNodeIndex];
            }

            if (bottomNodeIndex == leftNodeIndex)
            {
                return; // No valid basin
            }

            rightNodeIndex  = bottomNodeIndex;
            var rightNode       = advancingFrontNodes[rightNodeIndex];
            while (HasNext(rightNodeIndex) && rightNode.nodePoint.y < advancingFrontNodes[rightNode.nextNodeIndex].nodePoint.y)
            {
                rightNodeIndex = rightNode.nextNodeIndex;
                rightNode = advancingFrontNodes[rightNodeIndex];
            }

            if (rightNodeIndex == bottomNodeIndex)
            {
                return; // No valid basins
            }

            var leftNode    = advancingFrontNodes[leftNodeIndex];
            basinWidth  = rightNode.nodePoint.x - leftNode.nodePoint.x;
            basinLeftHighest = leftNode.nodePoint.y > rightNode.nodePoint.y;

            FillBasinReq(bottomNodeIndex);
        }


        /// <summary>
        /// Recursive algorithm to fill a Basin with triangles
        /// </summary>
        void FillBasinReq(ushort nodeIndex)
        {
            if (IsShallow(nodeIndex))
            {
                return; // if shallow stop filling
            }

            Fill(nodeIndex); 

            var node = advancingFrontNodes[nodeIndex];
            if (node.prevNodeIndex == leftNodeIndex && node.nextNodeIndex == rightNodeIndex)
            {
                return;
            }
            else if (node.prevNodeIndex == leftNodeIndex)
            {
                var nodeNext     = advancingFrontNodes[node.nextNodeIndex];
                var nodeNextNext = advancingFrontNodes[nodeNext.nextNodeIndex];
                var o = Orient2d(node.nodePoint, nodeNext.nodePoint, nodeNextNext.nodePoint);
                if (o == Orientation.CW)
                {
                    return;
                }
                nodeIndex = node.nextNodeIndex;
            }
            else if (node.nextNodeIndex == rightNodeIndex)
            {
                var nodePrev = advancingFrontNodes[node.prevNodeIndex];
                var nodePrevPrev = advancingFrontNodes[nodePrev.prevNodeIndex];
                var o = Orient2d(node.nodePoint, nodePrev.nodePoint, nodePrevPrev.nodePoint);
                if (o == Orientation.CCW)
                {
                    return;
                }
                nodeIndex = node.prevNodeIndex;
            }
            else
            {
                var nodePrev = advancingFrontNodes[node.prevNodeIndex];
                var nodeNext = advancingFrontNodes[node.nextNodeIndex];
                // Continue with the neighbor node with lowest Y value
                if (nodePrev.nodePoint.y < nodeNext.nodePoint.y)
                {
                    nodeIndex = node.prevNodeIndex;
                }
                else
                {
                    nodeIndex = node.nextNodeIndex;
                }
            }
            FillBasinReq(nodeIndex);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool IsShallow(ushort nodeIndex)
        {
            var node    = advancingFrontNodes[nodeIndex];
            var height  = basinLeftHighest ? (advancingFrontNodes[leftNodeIndex].nodePoint.y  - node.nodePoint.y)
                                           : (advancingFrontNodes[rightNodeIndex].nodePoint.y - node.nodePoint.y);
            return basinWidth > height;
        }


        /// <summary>
        /// ???
        /// </summary>
        /// <param name="node">middle node</param>
        /// <returns>the angle between 3 front nodes</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        double HoleAngle(ushort nodeIndex)
        {
            var node     = advancingFrontNodes[nodeIndex];
            var nodePrev = advancingFrontNodes[node.prevNodeIndex];
            var nodeNext = advancingFrontNodes[node.nextNodeIndex];
            // XXX: do we really need a signed angle for holeAngle?
            //      could possible save some cycles here
            /* Complex plane
             * ab = cosA +i*sinA
             * ab = (ax + ay*i)(bx + by*i) = (ax*bx + ay*by) + i(ax*by-ay*bx)
             * atan2(y,x) computes the principal value of the argument function
             * applied to the complex number x+iy
             * Where x = ax*bx + ay*by
             *       y = ax*by - ay*bx
             */
            var px = node.nodePoint.x;
            var py = node.nodePoint.y;
            var ax = nodeNext.nodePoint.x - px;
            var ay = nodeNext.nodePoint.y - py;
            var bx = nodePrev.nodePoint.x - px;
            var by = nodePrev.nodePoint.y - py;
            return (double)Math.Atan2((ax * by) - (ay * bx), (ax * bx) + (ay * by));
        }


        /// <summary>
        /// The basin angle is decided against the horizontal line [1,0]
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        double BasinAngle(ushort nodeIndex)
        {
            var node = advancingFrontNodes[nodeIndex];
            var nodeNext = advancingFrontNodes[node.nextNodeIndex];
            var nodeNextNext = advancingFrontNodes[nodeNext.nextNodeIndex];
            var ax = node.nodePoint.x - nodeNextNext.nodePoint.x;
            var ay = node.nodePoint.y - nodeNextNext.nodePoint.y;
            return (double)Math.Atan2(ay, ax);
        }


        /// <summary>
        /// Adds a triangle to the advancing front to fill a hole.
        /// </summary>
        /// <param name="node">middle node, that is the bottom of the hole</param>
        void Fill(ushort nodeIndex)
        {
            var node     = advancingFrontNodes[nodeIndex];
            var nodePrevIndex = node.prevNodeIndex;
            var nodeNextIndex = node.nextNodeIndex;
            var triangleIndex = (ushort)triangles.Count;

            if (nodePrevIndex == ushort.MaxValue ||
                nodeNextIndex == ushort.MaxValue ||
                node.pointIndex == ushort.MaxValue)
            {
                Debug.LogError("nodePrevIndex == ushort.MaxValue || nodeNextIndex == ushort.MaxValue || node.pointIndex == ushort.MaxValue");
                return;
            }

            AddTriangle(new DelaunayTriangle(advancingFrontNodes[nodePrevIndex].pointIndex, node.pointIndex, advancingFrontNodes[nodeNextIndex].pointIndex));
            // TODO: should copy the cEdge value from neighbor triangles
            //       for now cEdge values are copied during the legalize 
            MarkNeighbor(advancingFrontNodes[nodePrevIndex].triangleIndex, triangleIndex);
            MarkNeighbor(node.triangleIndex, triangleIndex);

            // Update the advancing front
            {
                var nodePrev = advancingFrontNodes[nodePrevIndex];
                if (nodeNextIndex == nodePrevIndex)
                {
                    UnityEngine.Debug.Assert(nodeNextIndex != nodePrevIndex);
                } else
                {
                    nodePrev.nextNodeIndex = nodeNextIndex;
                    advancingFrontNodes[nodePrevIndex] = nodePrev;
                }
            }
            {
                var nodeNext = advancingFrontNodes[nodeNextIndex];
                if (nodeNextIndex == nodePrevIndex)
                {
                    UnityEngine.Debug.Assert(nodeNextIndex != nodePrevIndex);
                } else
                {
                    nodeNext.prevNodeIndex = nodePrevIndex;
                    advancingFrontNodes[nodeNextIndex] = nodeNext;
                }
            }
            
            // If it was legalized the triangle has already been mapped
            if (!Legalize(triangleIndex))
            {
                MapTriangleToNodes(triangleIndex); 
            }
        }


        /// <summary>
        /// Returns true if triangle was legalized
        /// </summary>
        bool Legalize(ushort triangleIndex)
        {
            var inputTriangle = triangles[triangleIndex];
            
            // To legalize a triangle we start by finding if any of the three edges
            // violate the Delaunay condition
            for (int i = 0; i < 3; i++)
            {
                // TODO: fix so that cEdge is always valid when creating new triangles then we can check it here
                //       instead of below with ot
                if (inputTriangle.GetDelaunayEdge(i))
                {
                    continue;
                }

                var pIndex = inputTriangle.indices[i];
                var otIndex = inputTriangle.neighbors[i];
                if (otIndex == ushort.MaxValue)
                {
                    continue;
                }

                var otTriangle = triangles[otIndex];

                ushort opIndex = OppositePoint(otIndex, triangleIndex, pIndex);
                ushort oi = otTriangle.IndexOf(opIndex);
                // If this is a Constrained Edge or a Delaunay Edge(only during recursive legalization)
                // then we should not try to legalize
                if (otTriangle.GetConstrainedEdge(oi) ||
                    otTriangle.GetDelaunayEdge(oi))
                {
                    inputTriangle.SetConstrainedEdgeAcross(pIndex, otTriangle.GetConstrainedEdge(oi)); // XXX: have no good way of setting this property when creating new triangles so lets set it here
                    triangles[triangleIndex] = inputTriangle;
                    continue;
                }

                var pt0 = points[pIndex];
                var pt1 = points[inputTriangle.PointCCWFrom(pIndex)];
                var pt2 = points[inputTriangle.PointCWFrom(pIndex)];
                var pt3 = points[opIndex];
                if (!SmartIncircle(pt0, pt1, pt2, pt3))
                {
                    continue;
                }

                // Lets mark this shared edge as Delaunay 
                {
                    inputTriangle.SetDelauneyEdge(i, true);
                    triangles[triangleIndex] = inputTriangle;
                }
                {
                    otTriangle.SetDelauneyEdge(oi, true);
                    triangles[otIndex] = otTriangle;
                }

                // Lets rotate shared edge one vertex CW to legalize it
                RotateTrianglePair(triangleIndex, pIndex, otIndex, opIndex);

                // We now got one valid Delaunay Edge shared by two triangles
                // This gives us 4 new edges to check for Delaunay

                // Make sure that triangle to node mapping is done only one time for a specific triangle
                if (!Legalize(triangleIndex))
                {
                    MapTriangleToNodes(triangleIndex);
                }
                if (!Legalize(otIndex))
                {
                    MapTriangleToNodes(otIndex);
                }
                inputTriangle   = triangles[triangleIndex];
                otTriangle      = triangles[otIndex];

                // Reset the Delaunay edges, since they only are valid Delaunay edges
                // until we add a new triangle or point.
                // XXX: need to think about ctx Can these edges be tried after we 
                //      return to previous recursive level?
                {
                    inputTriangle.SetDelauneyEdge(i, false);
                    triangles[triangleIndex] = inputTriangle;
                }
                {
                    otTriangle.SetDelauneyEdge(oi, false);
                    triangles[otIndex] = otTriangle;
                }

                // If triangle have been legalized no need to check the other edges since
                // the recursive legalization will handles those so we can end here.
                return true;
            }
            return false;
        }


        /// <summary>
        /// Rotates a triangle pair one vertex CW
        ///       n2                    n2
        ///  P +-----+             P +-----+
        ///    | t  /|               |\  t |  
        ///    |   / |               | \   |
        ///  n1|  /  |n3           n1|  \  |n3
        ///    | /   |    after CW   |   \ |
        ///    |/ oT |               | oT \|
        ///    +-----+ oP            +-----+
        ///       n4                    n4
        /// </summary>
        void RotateTrianglePair(ushort triangleIndex, ushort pIndex, ushort otherTriangleIndex, ushort opIndex)
        {
            // TODO: optimize
            var otherTriangle   = triangles[otherTriangleIndex];
            var triangle        = triangles[triangleIndex];

            var n1 = triangle.NeighborCCWFrom(pIndex);
            var n2 = triangle.NeighborCWFrom(pIndex);
            var n3 = otherTriangle.NeighborCCWFrom(opIndex);
            var n4 = otherTriangle.NeighborCWFrom(opIndex);

            var ce1 = triangle.GetConstrainedEdgeCCW(pIndex);
            var ce2 = triangle.GetConstrainedEdgeCW(pIndex);
            var ce3 = otherTriangle.GetConstrainedEdgeCCW(opIndex);
            var ce4 = otherTriangle.GetConstrainedEdgeCW(opIndex);

            var de1 = triangle.GetDelaunayEdgeCCW(pIndex);
            var de2 = triangle.GetDelaunayEdgeCW(pIndex);
            var de3 = otherTriangle.GetDelaunayEdgeCCW(opIndex);
            var de4 = otherTriangle.GetDelaunayEdgeCW(opIndex);
            
            triangle.Legalize(pIndex, opIndex);
            otherTriangle.Legalize(opIndex, pIndex);

            // Remap dEdge
            otherTriangle.SetDelaunayEdgeCCW(pIndex, de1);
            triangle.SetDelaunayEdgeCW(pIndex, de2);
            triangle.SetDelaunayEdgeCCW(opIndex, de3);
            otherTriangle.SetDelaunayEdgeCW(opIndex, de4);

            // Remap cEdge
            otherTriangle.SetConstrainedEdgeCCW(pIndex, ce1);
            triangle.SetConstrainedEdgeCW(pIndex, ce2);
            triangle.SetConstrainedEdgeCCW(opIndex, ce3);
            otherTriangle.SetConstrainedEdgeCW(opIndex, ce4);

            // Remap neighbors
            // XXX: might optimize the markNeighbor by keeping track of
            //      what side should be assigned to what neighbor after the 
            //      rotation. Now mark neighbor does lots of testing to find 
            //      the right side.
            triangle.neighbors[0] = ushort.MaxValue;
            triangle.neighbors[1] = ushort.MaxValue;
            triangle.neighbors[2] = ushort.MaxValue;
            otherTriangle.neighbors[0] = ushort.MaxValue;
            otherTriangle.neighbors[1] = ushort.MaxValue;
            otherTriangle.neighbors[2] = ushort.MaxValue;
            triangles[otherTriangleIndex] = otherTriangle;
            triangles[triangleIndex] = triangle;
            if (n1 != ushort.MaxValue) MarkNeighbor(n1, otherTriangleIndex);
            if (n2 != ushort.MaxValue) MarkNeighbor(n2, triangleIndex);
            if (n3 != ushort.MaxValue) MarkNeighbor(n3, triangleIndex);
            if (n4 != ushort.MaxValue) MarkNeighbor(n4, otherTriangleIndex);
            MarkNeighbor(otherTriangleIndex, triangleIndex);
        }



        /// <summary>
        /// Exhaustive search to update neighbor pointers
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void MarkNeighbor(ushort triangleIndex1, ushort triangleIndex2)
        {
            var triangle2 = triangles[triangleIndex2];

            var index0 = triangle2.indices[0];
            var index1 = triangle2.indices[1];
            var index2 = triangle2.indices[2];

            var triangle1 = triangles[triangleIndex1];

            // Points of this triangle also belonging to t
            bool a = triangle1.Contains(index0);
            bool b = triangle1.Contains(index1);
            bool c = triangle1.Contains(index2);

            if (b && c)
            {
                triangle2.neighbors[0] = triangleIndex1;
                triangle1.MarkNeighbor(index1, index2, triangleIndex2);
            }
            else if (a && c)
            {
                triangle2.neighbors[1] = triangleIndex1;
                triangle1.MarkNeighbor(index0, index2, triangleIndex2);
            }
            else if (a && b)
            {
                triangle2.neighbors[2] = triangleIndex1;
                triangle1.MarkNeighbor(index0, index1, triangleIndex2);
            }
            else
            {
                throw new Exception("Failed to mark neighbor, doesn't share an edge!");
            }

            triangles[triangleIndex1] = triangle1;
            triangles[triangleIndex2] = triangle2;
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool HasEdgeByPoints(ushort p1, ushort p2)
        {
            if (p2 == ushort.MaxValue ||
                p2 == p1)
                return false;

            var edgeIndex = this.edges[p1];
            while (edgeIndex != ushort.MaxValue)
            {
                var sc = allEdges[edgeIndex].index2;
                edgeIndex = allEdges[edgeIndex].next;
                if (sc == p2)
                    return true;
            }
            return false;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool HasEdgeCCW(int triangleIndex, ushort p)
        {
            var triangle = triangles[triangleIndex];
            int pointIndex = triangle.IndexOf(p);
            int idx = (pointIndex + 2) % 3;

            if (idx < 0 || idx > 2)
                return false;

            var p1 = triangle.indices[(idx + 1) % 3];
            var p2 = triangle.indices[(idx + 2) % 3];
            return HasEdgeByPoints(p1, p2) || HasEdgeByPoints(p2, p1);
        }


        /**
         * @author Thomas Åhlén, thahlen@gmail.com
         */


        /// <summary>
        ///   Requirements:
        /// 1. a,b and c form a triangle.
        /// 2. a and d is know to be on opposite side of bc
        /// <code>
        ///                a
        ///                +
        ///               / \
        ///              /   \
        ///            b/     \c
        ///            +-------+ 
        ///           /    B    \  
        ///          /           \ 
        /// </code>
        ///    Facts:
        ///  d has to be in area B to have a chance to be inside the circle formed by a,b and c
        ///  d is outside B if orient2d(a,b,d) or orient2d(c,a,d) is CW
        ///  This preknowledge gives us a way to optimize the incircle test
        /// </summary>
        /// <param name="pa">triangle point, opposite d</param>
        /// <param name="pb">triangle point</param>
        /// <param name="pc">triangle point</param>
        /// <param name="pd">point opposite a</param>
        /// <returns>true if d is inside circle, false if on circle edge</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool SmartIncircle(double2 pa, double2 pb, double2 pc, double2 pd)
        {
            var pdx = pd.x;
            var pdy = pd.y;
            var adx = pa.x - pdx;
            var ady = pa.y - pdy;
            var bdx = pb.x - pdx;
            var bdy = pb.y - pdy;

            var adxbdy = adx * bdy;
            var bdxady = bdx * ady;
            var oabd = adxbdy - bdxady;
            if (oabd <= 0)
            {
                return false;
            }

            var cdx = pc.x - pdx;
            var cdy = pc.y - pdy;

            var cdxady = cdx * ady;
            var adxcdy = adx * cdy;
            var ocad = cdxady - adxcdy;
            if (ocad <= 0)
            {
                return false;
            }

            var bdxcdy = bdx * cdy;
            var cdxbdy = cdx * bdy;

            var alift = adx * adx + ady * ady;
            var blift = bdx * bdx + bdy * bdy;
            var clift = cdx * cdx + cdy * cdy;

            var det = alift * (bdxcdy - cdxbdy) + blift * ocad + clift * oabd;

            return det > 0;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool InScanArea(double2 pa, double2 pb, double2 pc, double2 pd)
        {
            var pdx = pd.x;
            var pdy = pd.y;
            var adx = pa.x - pdx;
            var ady = pa.y - pdy;
            var bdx = pb.x - pdx;
            var bdy = pb.y - pdy;

            var adxbdy = adx * bdy;
            var bdxady = bdx * ady;
            var oabd = adxbdy - bdxady;
            if (oabd <= 0)
            {
                return false;
            }

            var cdx = pc.x - pdx;
            var cdy = pc.y - pdy;

            var cdxady = cdx * ady;
            var adxcdy = adx * cdy;
            var ocad = cdxady - adxcdy;
            if (ocad <= 0)
            {
                return false;
            }
            return true;
        }

        const double kEpsilon = 1e-8f;//12f;

        /// Forumla to calculate signed area
        /// Positive if CCW
        /// Negative if CW
        /// 0 if collinear
        /// A[P1,P2,P3]  =  (x1*y2 - y1*x2) + (x2*y3 - y2*x3) + (x3*y1 - y3*x1)
        ///              =  (x1-x3)*(y2-y3) - (y1-y3)*(x2-x3)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static Orientation Orient2d(double2 pa, double2 pb, double2 pc)
        {
            var detleft     = (pa.x - pc.x) * (pb.y - pc.y);
            var detright    = (pa.y - pc.y) * (pb.x - pc.x);
            var val = detleft - detright;
            if (val > -kEpsilon && 
                val <  kEpsilon)
            {
                return Orientation.Collinear;
            }
            else if (val > 0)
            {
                return Orientation.CCW;
            }
            return Orientation.CW;
        }
    }
}
