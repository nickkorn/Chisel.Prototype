using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    internal sealed class BrushLoops : IDisposable
    {
        public CSGTreeBrush brush;
        public VertexSoup   vertexSoup   = new VertexSoup();
        public List<Loop>   basePolygons = new List<Loop>();

        // TODO: Have list of surfaceloops, dictionary holds index into list
        // TODO: Actually, do not need lookups?
        public Dictionary<int, SurfaceLoops>    intersectionSurfaceLoops    = new Dictionary<int, SurfaceLoops>();
        public Dictionary<int, Loop[]>          intersectionLoops           = new Dictionary<int, Loop[]>();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            intersectionSurfaceLoops.Clear();
            intersectionLoops.Clear();
        }
        ~BrushLoops() { Dispose(); }
        public void Dispose()
        {
            vertexSoup.Dispose();
            foreach (var surfaceLoop in intersectionSurfaceLoops.Values)
                surfaceLoop.Dispose();
            intersectionSurfaceLoops.Clear();
            foreach (var loopArray in intersectionLoops.Values)
            {
                if (loopArray != null &&
                    loopArray.Length > 0)
                {
                    foreach (var loop in loopArray)
                    {
                        if (loop != null)
                            loop.Dispose();
                    }
                }
            }
            intersectionLoops.Clear();
            foreach (var loop in basePolygons)
                loop.Dispose();
            basePolygons.Clear();
        }
    }

#endif
}
