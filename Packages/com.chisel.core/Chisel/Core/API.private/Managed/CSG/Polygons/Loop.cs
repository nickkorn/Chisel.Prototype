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
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    // TODO: rename
    public sealed class Loop : IDisposable
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

        public NativeList<Edge>             edges;        
        
        public bool Valid { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return edges.Length >= 3; } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop()
        {
            edges = new NativeList<Edge>(Allocator.Persistent);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop(NativeList<Edge> edges)
        {
            this.edges = edges;
        } 

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Loop(Loop original)
        {
            edges = new NativeList<Edge>(Allocator.Persistent);
            this.edges.AddRange(original.edges);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe Loop(BlobAssetReference<BrushIntersectionLoop> brushIntersectionLoop, VertexSoup vertexSoup)
        {            
            ref var vertices = ref brushIntersectionLoop.Value.loopVertices;
            var srcIndices = new ushort[vertices.Length];
            vertexSoup.Reserve(srcIndices.Length);
            for (int j = 0; j < srcIndices.Length; j++)
                srcIndices[j] = vertexSoup.AddNoResize(vertices[j]);

            {
                edges = new NativeList<Edge>(Allocator.Persistent);
                edges.Capacity = srcIndices.Length;
                for (int j = 1; j < srcIndices.Length; j++)
                {
                    edges.Add(new Edge() { index1 = srcIndices[j - 1], index2 = srcIndices[j] });
                }
                edges.Add(new Edge() { index1 = srcIndices[srcIndices.Length - 1], index2 = srcIndices[0] });
            }
        }


        ~Loop()
        {
            Dispose();
        }

        public void Dispose()
        {
            if (edges.IsCreated)
                edges.Dispose();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearAllEdges()
        {
            edges.Clear();
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
        public bool AddEdges(NativeList<Edge> addEdges, bool removeDuplicates = false)
        {
            if (addEdges.Length == 0)
                return false;

            s_UniqueEdges.Clear();
            for (int e = 0; e < addEdges.Length; e++)
            {
                if (!s_UniqueEdges.Add(addEdges[e]))
                {
                    Debug.Log("not unique");
                }
            }

            if (removeDuplicates)
            {
                for (int e = edges.Length - 1; e >= 0; e--)
                {
                    if (s_UniqueEdges.Remove(edges[e]))
                    {
                        edges.RemoveAtSwapBack(e);
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
            for (int e = 0; e < edges.Length; e++)
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
