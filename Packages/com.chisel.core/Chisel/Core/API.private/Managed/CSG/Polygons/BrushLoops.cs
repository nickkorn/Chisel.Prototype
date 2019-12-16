using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
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
    internal sealed class BrushLoops
    {
        public CSGTreeBrush brush;
        public VertexSoup   vertexSoup   = new VertexSoup();
        public List<Loop>   basePolygons = new List<Loop>();

        // TODO: Have list of surfaceloops, dictionary holds index into list
        public Dictionary<int, SurfaceLoops>    intersectionSurfaceLoops    = new Dictionary<int, SurfaceLoops>();
        public Dictionary<int, Loop[]>          intersectionLoops           = new Dictionary<int, Loop[]>();

        public void Clear()
        {
            intersectionSurfaceLoops.Clear();
            intersectionLoops.Clear();
        }
    }

#endif
}
