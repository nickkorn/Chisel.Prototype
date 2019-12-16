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
    public sealed class SurfaceLoops
    {
        public List<Loop>[] surfaces;

        public SurfaceLoops() {}

        public SurfaceLoops(int surfaceCount)
        {
            EnsureSize(surfaceCount);
        }

        public void EnsureSize(int length)
        {
            if (surfaces == null ||
                surfaces.Length != length)
                surfaces = new List<Loop>[length];
            for (int i = 0; i < length; i++)
                surfaces[i] = new List<Loop>();
        }

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
