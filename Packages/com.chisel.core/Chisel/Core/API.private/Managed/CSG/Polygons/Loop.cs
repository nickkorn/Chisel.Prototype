using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    // TODO: rename
    public struct LoopInfo
    {
        [NonSerialized] public Plane            worldPlane;
        [NonSerialized] public Vector3          right;
        [NonSerialized] public Vector3          forward;
        [NonSerialized] public int              basePlaneIndex;
        [NonSerialized] public CSGTreeBrush     brush;
        [NonSerialized] public SurfaceLayers    layers;				    // always on owning brush
    }

    // TODO: rename
    [Serializable] 
    public sealed class Loop
    {
        public static int loopDebugCounter = 0;
        public int loopIndex = loopDebugCounter++;

        public readonly List<ushort>        indices     = new List<ushort>();
        public List<Edge>                   edges       = new List<Edge>();
        public bool[]                       destroyed;

        [NonSerialized] public List<Loop>   holes       = new List<Loop>();
        [NonSerialized] public LoopInfo     info;

        [NonSerialized] public CategoryGroupIndex   _interiorCategory    = CategoryGroupIndex.Invalid; // determine if the loop is inside another brush or aligned with another brush
        public CategoryGroupIndex   interiorCategory
        {
            get
            {
                return _interiorCategory;
            }
            set
            {
                if (loopIndex == 178 ||
                    loopIndex == 182 ||
                    loopIndex == 186 ||
                    loopIndex == 190)
                {
                    Debug.Log($"[{loopIndex}] {_interiorCategory} -> {value} | 'Brush {info.brush}' {info.basePlaneIndex}");
                }
                _interiorCategory = value;
            }
        }
        [NonSerialized] public bool                 convex              = false;

        public bool Valid { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return indices.Count >= 3; } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop() { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop(Loop original)
        {
            this.edges  .AddRange(original.edges);
            this.indices.AddRange(original.indices);
            this.info               = original.info;
            this.interiorCategory   = original.interiorCategory;
            this.convex             = original.convex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearAllIndices()
        {
            indices.Clear();
            edges.Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddIndex(VertexSoup soup, ushort newIndex)
        {
            if (indices.Contains(newIndex))
                return;

            indices.Add(newIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Loop FindOrAddLoop(List<Loop> loops, int basePlaneIndex, CSGTreeBrush brush, bool isConvex = true)
        {
            Loop loop = null;
            for (int l = 0; l < loops.Count; l++)
            {
                if (loops[l].info.basePlaneIndex == basePlaneIndex)
                {
                    loop = loops[l];
                    break;
                }
            }

            if (loop == null)
            {
                loop = new Loop() { info = new LoopInfo() { basePlaneIndex = basePlaneIndex, brush = brush }, convex = isConvex };
                loops.Add(loop);
            }
            return loop;
        }


        static HashSet<Edge> s_UniqueEdges = new HashSet<Edge>();
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddEdges(List<ushort> indices)
        {
            if (indices.Count == 0)
                return;
            s_UniqueEdges.Clear();
            if (edges.Capacity < edges.Count + indices.Count)
                edges.Capacity = edges.Count + indices.Count;
            for (int i = 0; i < indices.Count - 1; i++)
            {
                Debug.Assert(indices[i] != indices[i + 1]);
                s_UniqueEdges.Add(new Edge() { index1 = indices[i], index2 = indices[i + 1] });
            }
            Debug.Assert(indices[indices.Count - 1] != indices[0]);
            s_UniqueEdges.Add(new Edge() { index1 = indices[indices.Count - 1], index2 = indices[0] });

            for (int e = 0; e < edges.Count; e++)
            {
                if (s_UniqueEdges.Contains(edges[e]))
                    s_UniqueEdges.Remove(edges[e]);
            }

            edges.AddRange(s_UniqueEdges);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddEdges(List<Edge> addEdges, bool removeDuplicates = false)
        {
            if (addEdges.Count == 0)
                return;

            s_UniqueEdges.Clear();
            for (int e = 0; e < addEdges.Count; e++)
            {
                if (!s_UniqueEdges.Add(addEdges[e]))
                {
                    Debug.Log("not unique");
                }
            }

            if (removeDuplicates)
            {
                for (int e = edges.Count - 1; e >= 0; e--)
                {
                    if (s_UniqueEdges.Remove(edges[e]))
                    {
                        edges.RemoveAt(e);
                    }
                }
            }

            var removeEdges = new List<int>();
            foreach (var addEdge in s_UniqueEdges)
            {
                var index = IndexOf(addEdge, out bool inverted);
                if (index != -1)
                {
                    Debug.Log($"Duplicate edge {inverted}  {addEdge.index1}/{addEdge.index2} {edges[index].index1}/{edges[index].index2}");
                    continue;
                }
                this.edges.Add(addEdge);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(Edge edge, out bool inverted)
        {
            //var builder = new System.Text.StringBuilder();
            for (int e = 0; e < edges.Count; e++)
            {
                //builder.AppendLine($"{e}/{edges.Count}: {edges[e]} {edge}");
                if (edges[e].index1 == edge.index1 && edges[e].index2 == edge.index2) { inverted = false; return e; }
                if (edges[e].index1 == edge.index2 && edges[e].index2 == edge.index1) { inverted = true;  return e; }
            }
            //Debug.Log(builder.ToString());
            inverted = false;
            return -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public EdgeCategory CategorizeEdge(VertexSoup soup, Edge edge)
        {
            if (IndexOf(edge, out bool inverted) != -1)
                return (inverted) ? EdgeCategory.ReverseAligned : EdgeCategory.Aligned;
            var vertices = soup.vertices;
            var midPoint = (vertices[edge.index1] + vertices[edge.index2]) * 0.5f;

            if (CSGManagerPerformCSG.IsPointInPolygon(info.right, info.forward, indices, soup, midPoint))
                return EdgeCategory.Inside;
            return EdgeCategory.Outside;
        }
    }
#endif
}
