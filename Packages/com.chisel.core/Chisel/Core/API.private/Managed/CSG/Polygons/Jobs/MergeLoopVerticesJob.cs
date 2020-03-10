using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    
    [BurstCompile(Debug = false)]
    public struct MergeLoopVerticesJob : IJob
    {
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeArray<float4> verticesSrc;
        //readwrite
        public NativeArray<float4> verticesDst;

        // TODO: optimize
        public unsafe void Execute()
        {
            const float kVertexEqualEpsilonSqr = (float)CSGManagerPerformCSG.kEpsilonSqr;
            var verticesSrcPtr  = (float4*)NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(verticesSrc);
            var verticesDstPtr  = (float4*)NativeArrayUnsafeUtility.GetUnsafePtr(verticesDst);

            var verticesSrcCount = verticesSrc.Length;
            var verticesDstCount = verticesDst.Length;

            for (int v2 = 0; v2 < verticesDstCount; v2++)
            {
                var vertex2 = verticesDstPtr[v2];
                for (int v1 = 0; v1 < verticesSrcCount; v1++)
                {
                    var vertex1 = verticesDstPtr[v1];
                    if (math.lengthsq(vertex1 - vertex2) >= kVertexEqualEpsilonSqr)
                        continue;

                    verticesDstPtr[v2] = vertex1;
                    
                    // We already removed all duplicate vertices within the loops in a previous phase, so this is safe
                    // FIXME: no longer true now that we're sharing vertices between multiple loops, need to find a better way of doing this
                    break; 
                }
            }
        }
    }

#endif
}
