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
    public sealed class BrushLoops : IDisposable
    {
        public List<Loop>   basePolygons = new List<Loop>();

        public Loop[][] intersectionLoops;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
        }

        ~BrushLoops() { Dispose(); }
        public void Dispose()
        {
            if (intersectionLoops != null)
            {
                foreach (var loopArray in intersectionLoops)
                {
                    if (loopArray != null)
                    {
                        foreach (var loop in loopArray)
                        {
                            if (loop != null)
                                loop.Dispose();
                        }
                    }
                }
                intersectionLoops = null;
            }
            foreach (var loop in basePolygons)
                loop.Dispose();
            basePolygons.Clear();
        }
    }

#endif
}
