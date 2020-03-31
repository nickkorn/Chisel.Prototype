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
    public sealed class SurfaceLoop : IDisposable
    {
        public int Key;
        public Loop[] surfaces;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SurfaceLoop() { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SurfaceLoop(int key, int surfaceCount)
        {
            this.Key = key;
            surfaces = new Loop[surfaceCount];
        }

        ~SurfaceLoop() { Dispose(); }

        public void Dispose()
        {
            if (surfaces != null)
            {
                foreach (var loopList in surfaces)
                {
                    if (loopList != null)
                        loopList.Dispose();
                }
                surfaces = null;
            }
        }
    }

    public sealed class SurfaceLoops : IDisposable
    {
        public List<Loop>[] surfaces;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SurfaceLoops() {}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SurfaceLoops(int surfaceCount) 
        {
            surfaces = new List<Loop>[surfaceCount];
            for (int i = 0; i < surfaceCount; i++)
                surfaces[i] = new List<Loop>();
        }
         
        ~SurfaceLoops() { Dispose(); }

        public void Dispose()
        {
            if (surfaces != null)
            {
                foreach (var loopList in surfaces)
                {
                    foreach (var loop in loopList)
                        loop.Dispose();
                    loopList.Clear();
                }
            }
        }
    }
#endif
}
