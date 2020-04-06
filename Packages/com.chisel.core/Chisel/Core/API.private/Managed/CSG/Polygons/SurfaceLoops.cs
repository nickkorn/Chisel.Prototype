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
        public NativeList<int>          loopIndices;
    }

    public sealed class BrushLoops : IDisposable
    {
        public SurfaceLoops[] surfaces;
        public NativeListArray<int>     holeIndices;
        public NativeList<SurfaceInfo>  allInfos;
        public NativeListArray<Edge>    allEdges;

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
#if false
                surfaces[i].holeIndices = new NativeListArray<int>(Allocator.TempJob);
                surfaces[i].allInfos    = new NativeList<SurfaceInfo>(Allocator.TempJob);
                surfaces[i].allEdges    = new NativeListArray<Edge>(Allocator.TempJob);
            }
#else
            }
            holeIndices = new NativeListArray<int>(Allocator.TempJob);
            allInfos    = new NativeList<SurfaceInfo>(Allocator.TempJob);
            allEdges    = new NativeListArray<Edge>(Allocator.TempJob);
#endif
        }
         
        ~BrushLoops() { Dispose(); }

        public void Dispose()
        {
            if (surfaces != null)
            {
                foreach (var surfaceLoop in surfaces)
                {
                    if (surfaceLoop.loopIndices.IsCreated) surfaceLoop.loopIndices.Dispose();
#if false
                    if (surfaceLoop.holeIndices.IsCreated) surfaceLoop.holeIndices.Dispose();
                    if (surfaceLoop.allInfos.IsCreated) surfaceLoop.allInfos.Dispose();
                    if (surfaceLoop.allEdges.IsCreated) surfaceLoop.allEdges.Dispose();
                }
#else
                }
                if (holeIndices.IsCreated) holeIndices.Dispose();
                if (allInfos.IsCreated) allInfos.Dispose();
                if (allEdges.IsCreated) allEdges.Dispose();
#endif
                surfaces = null;
            }
        }
    }
#endif
                }
