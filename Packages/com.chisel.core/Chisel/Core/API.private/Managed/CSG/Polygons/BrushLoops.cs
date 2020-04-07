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
    public sealed class BrushOutputLoops
    {
        public NativeList<SurfaceInfo>  intersectionSurfaceInfos;
        public NativeListArray<Edge>    intersectionEdges;
        public NativeList<SurfaceInfo>  basePolygonSurfaceInfos;
        public NativeListArray<Edge>    basePolygonEdges;
        public VertexSoup               vertexSoup;
    }

#endif
}
