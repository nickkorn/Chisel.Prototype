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
    public sealed class SurfaceLoops : IDisposable
    {
        public NativeList<int>          loopIndices;
        public NativeListArray<int>     holeIndices;
        public NativeList<SurfaceInfo>  allInfos;
        public NativeListArray<Edge>    allEdges;
        
        ~SurfaceLoops() { Dispose(); }

        public SurfaceLoops()
        {
            loopIndices = new NativeList<int>(Allocator.TempJob);
            holeIndices = new NativeListArray<int>(Allocator.TempJob);
            allInfos    = new NativeList<SurfaceInfo>(Allocator.TempJob);
            allEdges    = new NativeListArray<Edge>(Allocator.TempJob);
        }

        public void Dispose()
        {
            if (loopIndices.IsCreated)  loopIndices.Dispose();
            if (holeIndices.IsCreated)  holeIndices.Dispose();
            if (allInfos.IsCreated)     allInfos.Dispose();
            if (allEdges.IsCreated)     allEdges.Dispose();
        }
    }

    public sealed class BrushLoops : IDisposable
    {
        public SurfaceLoops[] surfaces;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BrushLoops() {}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BrushLoops(int surfaceCount) 
        {
            surfaces = new SurfaceLoops[surfaceCount];
            for (int i = 0; i < surfaceCount; i++)
                surfaces[i] = new SurfaceLoops();
        }
         
        ~BrushLoops() { Dispose(); }

        public void Dispose()
        {
            if (surfaces != null)
            {
                foreach (var surfaceLoops in surfaces)
                    surfaceLoops.Dispose();
                surfaces = null;
            }
        }
    }
#endif
}
