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
    public sealed class SurfaceLoops
    {
        public List<Loop>[] surfaces;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SurfaceLoops() {}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SurfaceLoops(int surfaceCount)
        {
            if (surfaces == null ||
                surfaces.Length != surfaceCount)
                surfaces = new List<Loop>[surfaceCount];
            for (int i = 0; i < surfaceCount; i++)
                surfaces[i] = new List<Loop>();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            if (surfaces == null)
                return;

            for (int i = 0; i < surfaces.Length; i++)
                surfaces[i].Clear();
        }
    }
#endif
}
