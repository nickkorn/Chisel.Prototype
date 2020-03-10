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

        public readonly List<ushort>        indices     = new List<ushort>();
        public List<Edge>                   edges       = new List<Edge>();
        public bool[]                       destroyed;

        [NonSerialized] public List<Loop>   holes       = new List<Loop>();
        [NonSerialized] public SurfaceInfo  info;

        public bool Valid { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return indices.Count >= 3 || edges.Count >= 3; } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop()
        {
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop(Loop original)
        {
            this.edges  .AddRange(original.edges);
            this.indices.AddRange(original.indices);
            this.info = original.info;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop(Loop original, CategoryGroupIndex newHoleCategory)
        {
            this.edges.AddRange(original.edges);
            this.indices.AddRange(original.indices);
            this.info = original.info;
            this.info.interiorCategory = newHoleCategory;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe public Loop(in SurfaceInfo surfaceInfo, ushort* srcIndices, int offset, int length)
        {
            indices.Capacity = length;
            for (int j = 0; j < length; j++, offset++)
                indices.Add(srcIndices[offset]);
            AddEdges(indices);
            this.info = surfaceInfo;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearAllEdges()
        {
            indices.Clear();
            edges.Clear();
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void SetIndices(NativeList<ushort> srcIndices)
        {
            indices.Clear();
            if (indices.Capacity < srcIndices.Length)
                indices.Capacity = srcIndices.Length;
            for (int j = 0; j < srcIndices.Length; j++)
                indices.Add(srcIndices[j]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void SetEdges(NativeList<Edge> srcEdges)
        {
            edges.Clear();
            if (edges.Capacity < srcEdges.Length)
                edges.Capacity = srcEdges.Length;
            for (int j = 0; j < srcEdges.Length; j++)
                edges.Add(srcEdges[j]);
        }


        static HashSet<Edge> s_UniqueEdges = new HashSet<Edge>();
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddEdges(List<ushort> indices)
        {
            if (indices.Count < 3)
                return;

            s_UniqueEdges.Clear();
            if (edges.Capacity < edges.Count + indices.Count)
                edges.Capacity = edges.Count + indices.Count;
            for (int i = 0; i < indices.Count - 1; i++)
            {
                Debug.Assert(indices[i] != indices[i + 1], "indices[i] != indices[i + 1]");
                s_UniqueEdges.Add(new Edge() { index1 = indices[i], index2 = indices[i + 1] });
            }
            Debug.Assert(indices[indices.Count - 1] != indices[0], "indices[indices.Count - 1] != indices[0]");
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
    }
#endif
    }
