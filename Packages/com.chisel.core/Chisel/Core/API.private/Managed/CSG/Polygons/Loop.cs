//#define DebugInfo
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
    public sealed class LoopGroup
    {
        public Dictionary<int, Loop> loopLookup = new Dictionary<int, Loop>();
        public List<Loop> loops = new List<Loop>();

        public void Clear()
        {
            loopLookup.Clear();
            loops.Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop FindOrAddLoop(int basePlaneIndex, CSGTreeBrush brush, bool isConvex = true)
        {
            if (!loopLookup.TryGetValue(basePlaneIndex, out Loop loop))
            {
                loop = new Loop() { info = new LoopInfo() { basePlaneIndex = basePlaneIndex, brush = brush }, convex = isConvex };
                loopLookup.Add(basePlaneIndex, loop);
                loops.Add(loop);
            }
            return loop;
        }
    }

    // TODO: rename
    public sealed class Loop
    {
#if DebugInfo
        public static int loopDebugCounter = 0;
        public int loopIndex = loopDebugCounter++;
        public static void DebugInit()
        {
            loopDebugCounter = 0;
        }
#else
        public static void DebugInit()
        {
        }
#endif

        public readonly HashSet<ushort>     indexUsed   = new HashSet<ushort>();
        public readonly List<ushort>        indices     = new List<ushort>();
        public List<Edge>                   edges       = new List<Edge>();
        public bool[]                       destroyed;

        [NonSerialized] public List<Loop>   holes       = new List<Loop>();
        [NonSerialized] public LoopInfo     info;

        [NonSerialized] public bool                 convex              = false;
#if DebugInfo
        CategoryGroupIndex   _interiorCategory    = CategoryGroupIndex.Invalid; // determine if the loop is inside another brush or aligned with another brush
        public CategoryGroupIndex   interiorCategory
        {
            get { return _interiorCategory; }
            set
            {
                if (logLoopIndices.Contains(loopIndex))
                    Debug.Log($"{loopIndex}: Set {(CategoryIndex)_interiorCategory} => {(CategoryIndex)value}");
                _interiorCategory = value;
            }
        }
        internal static HashSet<int> logLoopIndices = new HashSet<int>()
        {
            245,118
        };
#else
        [NonSerialized] public CategoryGroupIndex   interiorCategory    = CategoryGroupIndex.Invalid; // determine if the loop is inside another brush or aligned with another brush
#endif

        public bool Valid { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return indices.Count >= 3; } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop()
        {
#if DebugInfo
            if (logLoopIndices.Contains(loopIndex))
                Debug.Log($"new => {loopIndex} {interiorCategory}");
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop(Loop original)
        {
#if DebugInfo
            if (logLoopIndices.Contains(loopIndex))
                Debug.Log($"copy {original.loopIndex} ({original.holes.Count}/{original.indices.Count}/{original.edges.Count}/{(CategoryIndex)original.interiorCategory}) => {loopIndex}");
#endif
            this.edges  .AddRange(original.edges);
            this.indices.AddRange(original.indices);
            this.info               = original.info;
            this.interiorCategory   = original.interiorCategory;
            this.convex             = original.convex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearAllIndices()
        {
            indexUsed.Clear();
            indices.Clear();
            edges.Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddIndex(VertexSoup soup, ushort newIndex)
        {
            if (indexUsed.Contains(newIndex))
                return;
            indexUsed.Add(newIndex);
            indices.Add(newIndex);
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
        public bool AddEdges(List<Edge> addEdges, bool removeDuplicates = false)
        {
            if (addEdges.Count == 0)
                return false;

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

            bool duplicates = false;

            foreach (var addEdge in s_UniqueEdges)
            {
                var index = IndexOf(addEdge, out bool inverted);
                if (index != -1)
                {
                    Debug.Log($"Duplicate edge {inverted}  {addEdge.index1}/{addEdge.index2} {edges[index].index1}/{edges[index].index2}");
                    duplicates = true;
                    continue;
                }
                this.edges.Add(addEdge);
            }

            return duplicates;
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
