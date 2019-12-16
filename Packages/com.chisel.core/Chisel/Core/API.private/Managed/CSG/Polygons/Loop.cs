using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
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
    {/*
        public static int[] loopTestIndices =
        {
            28,92,
            //35
        };
        */
        public static int loopDebugCounter = 0;
        public int loopIndex = loopDebugCounter++;

        public readonly List<ushort>         indices = new List<ushort>();
        public List<Edge>                    edges   = new List<Edge>();

        [NonSerialized] public List<Loop>    holes   = new List<Loop>();
        [NonSerialized] public LoopInfo      info;

        [NonSerialized] public CategoryGroupIndex   interiorCategory    = CategoryGroupIndex.First; // determine if the loop is inside another brush or aligned with another brush
        [NonSerialized] public bool                 convex              = false;


        public bool Valid { get { return indices.Count >= 3; } }

        public Loop() { }

        public Loop(Loop original)
        {
            this.edges  .AddRange(original.edges);
            this.indices.AddRange(original.indices);
            this.info               = original.info;
            this.interiorCategory   = original.interiorCategory;
            this.convex             = original.convex;
        }

        public void CopyDetails(Loop other)
        {
            info                = other.info;
            convex              = other.convex;
            interiorCategory    = other.interiorCategory;
        }

        public void ClearAllIndices()
        {
            indices.Clear();
            edges.Clear();
        }

        public void AddIndex(VertexSoup soup, ushort newIndex)
        {
            indices.Add(newIndex);
        }

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

        public void AddEdges(List<ushort> indices)
        {
            if (indices.Count == 0)
                return;
            s_UniqueEdges.Clear();
            if (edges.Capacity < edges.Count + indices.Count)
                edges.Capacity = edges.Count + indices.Count;
            for (int i = 0; i < indices.Count - 1; i++)
            {
                s_UniqueEdges.Add(new Edge() { index1 = indices[i], index2 = indices[i + 1] });
            }
            s_UniqueEdges.Add(new Edge() { index1 = indices[0], index2 = indices[indices.Count - 1] });

            for (int e = 0; e < edges.Count; e++)
            {
                if (s_UniqueEdges.Contains(edges[e]))
                    s_UniqueEdges.Remove(edges[e]);
            }

            edges.AddRange(s_UniqueEdges);
        }

        public void AddEdges(List<Edge> addEdges)
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

            foreach (var addEdge in s_UniqueEdges)
            {
                var index = IndexOf(addEdge, out bool inverted);
                if (index != -1)
                {
                    Debug.Log($"Double edge {inverted}  {addEdge.index1}/{addEdge.index2} {edges[index].index1}/{edges[index].index2}");
                    continue;
                }
                this.edges.Add(addEdge);
            }
        }

        public int IndexOf(Edge edge)
        {
            for (int e = 0; e < edges.Count; e++)
            {
                if (edges[e].index1 == edge.index1 && edges[e].index2 == edge.index2) 
                    return e; 
            }
            return -1;
        }

        public int IndexOf(Edge edge, out bool inverted)
        {
            inverted = false;
            for (int e = 0; e < edges.Count; e++)
            {
                if (edges[e].index1 == edge.index1 && edges[e].index2 == edge.index2) {                  return e; }
                if (edges[e].index1 == edge.index2 && edges[e].index2 == edge.index1) { inverted = true; return e; }
            }
            return -1;
        }
        
        public CategoryIndex CategorizeEdge(VertexSoup soup, Edge edge)
        {
            if (IndexOf(edge, out bool inverted) != -1)
                return (inverted) ? CategoryIndex.ReverseAligned : CategoryIndex.Aligned;
            
            var vertices = soup.vertices;
            var midPoint = (vertices[edge.index1] + vertices[edge.index2]) * 0.5f;

            if (CSGManagerPerformCSG.IsPointInPolygon(info.right, info.forward, indices, soup, midPoint))
                return CategoryIndex.Inside;
            return CategoryIndex.Outside;
        }
    }
#endif
}
