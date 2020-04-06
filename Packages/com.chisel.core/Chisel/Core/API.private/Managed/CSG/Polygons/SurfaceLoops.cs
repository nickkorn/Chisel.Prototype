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
    public struct SurfaceLoops
    {
        public NativeList<int>          loopIndices;//l
        public NativeListArray<int>     holeIndices;//index
        public NativeList<SurfaceInfo>  allInfos;   //index
        public NativeListArray<Edge>    allEdges;   //index
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
            {
                surfaces[i] = new SurfaceLoops();
                surfaces[i].loopIndices = new NativeList<int>(Allocator.TempJob);
                surfaces[i].holeIndices = new NativeListArray<int>(Allocator.TempJob);
                surfaces[i].allInfos    = new NativeList<SurfaceInfo>(Allocator.TempJob);
                surfaces[i].allEdges    = new NativeListArray<Edge>(Allocator.TempJob);
            }
        }
         
        ~BrushLoops() { Dispose(); }

        public void Dispose()
        {
            if (surfaces != null)
            {
                foreach (var surfaceLoop in surfaces)
                {
                    if (surfaceLoop.loopIndices.IsCreated) surfaceLoop.loopIndices.Dispose();
                    if (surfaceLoop.holeIndices.IsCreated) surfaceLoop.holeIndices.Dispose();
                    if (surfaceLoop.allInfos.IsCreated) surfaceLoop.allInfos.Dispose();
                    if (surfaceLoop.allEdges.IsCreated) surfaceLoop.allEdges.Dispose();
                }
                surfaces = null;
            }
        }
    }
#endif
}
