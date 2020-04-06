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
        public List<int>    loopIndices   = new List<int>();
        public List<Loop>   allLoops      = new List<Loop>();

        ~SurfaceLoops() { Dispose(); }

        public void Dispose()
        {
            if (allLoops != null)
            {
                foreach (var loop in allLoops)
                {
                    if (loop != null)
                        loop.Dispose();
                }
                allLoops = null;
            }
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
