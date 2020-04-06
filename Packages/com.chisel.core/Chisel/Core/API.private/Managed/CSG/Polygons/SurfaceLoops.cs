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

    public struct BrushLoops : IDisposable
    {
        public NativeListArray<int>     surfaceLoopIndices;
        public NativeListArray<int>     holeIndices;
        public NativeList<SurfaceInfo>  allInfos;
        public NativeListArray<Edge>    allEdges;
        public bool IsCreated => surfaceLoopIndices.IsCreated && holeIndices.IsCreated && allInfos.IsCreated && allEdges.IsCreated;

        public void Allocate() 
        {
            surfaceLoopIndices = new NativeListArray<int>(Allocator.TempJob);
            holeIndices = new NativeListArray<int>(Allocator.TempJob);
            allInfos    = new NativeList<SurfaceInfo>(Allocator.TempJob);
            allEdges    = new NativeListArray<Edge>(Allocator.TempJob);
        }
         
        public void Dispose()
        {
            if (surfaceLoopIndices.IsCreated) surfaceLoopIndices.Dispose();
            if (holeIndices.IsCreated) holeIndices.Dispose();
            if (allInfos.IsCreated) allInfos.Dispose();
            if (allEdges.IsCreated) allEdges.Dispose();
        }
    }
#endif
                }
