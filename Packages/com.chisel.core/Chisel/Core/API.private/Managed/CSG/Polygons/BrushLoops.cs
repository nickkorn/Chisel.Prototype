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
    public sealed class BrushOutputLoops : IDisposable
    {
        public NativeList<SurfaceInfo>  intersectionSurfaceInfos;
        public NativeListArray<Edge>    intersectionEdges;
        public NativeList<SurfaceInfo>  basePolygonSurfaceInfos;
        public NativeListArray<Edge>    basePolygonEdges;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            if (basePolygonEdges.IsCreated)
                basePolygonEdges.Dispose();
            if (basePolygonSurfaceInfos.IsCreated)
                basePolygonSurfaceInfos.Dispose();

            if (intersectionEdges.IsCreated)
                intersectionEdges.Dispose();
            if (intersectionSurfaceInfos.IsCreated)
                intersectionSurfaceInfos.Dispose();
        }

        public void Dispose()
        {
            Clear();
        }
    }

#endif
}
