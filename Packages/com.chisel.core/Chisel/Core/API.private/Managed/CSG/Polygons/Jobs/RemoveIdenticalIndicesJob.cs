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
    
    [BurstCompile]
    public unsafe struct RemoveIdenticalIndicesJob : IJob
    {
        public NativeList<ushort> indices;

        public void Execute()
        {
            if (indices.Length < 3)
            {
                indices.Clear();
                return;
            }

            var lastValidIndex = indices.Length;
            while (lastValidIndex > 3 && indices[lastValidIndex - 1] == indices[0])
                lastValidIndex--;
            if (lastValidIndex != indices.Length)
                indices.Resize(lastValidIndex, NativeArrayOptions.UninitializedMemory);

            if (indices.Length < 3)
            {
                indices.Clear();
                return;
            }

            lastValidIndex = 0;
            for (int v0 = 0, v1 = 1; v1 < indices.Length; v0 = v1, v1++)
            {
                if (indices[v0] == indices[v1])
                    break;
                lastValidIndex++;
            }
            if (lastValidIndex == indices.Length)
                return;


            var newIndices = (ushort*)UnsafeUtility.Malloc(indices.Length * sizeof(ushort), 4, Allocator.TempJob);
            {
                newIndices[0] = indices[0];
                int newIndicesLength = 1;
                for (int v0 = 0, v1 = 1; v1 < indices.Length; v0 = v1, v1++)
                {
                    if (indices[v0] == indices[v1])
                        continue;
                    newIndices[newIndicesLength] = indices[v1]; newIndicesLength++;
                }
                if (newIndicesLength != indices.Length)
                {
                    indices.Clear();
                    if (newIndicesLength >= 3)
                        indices.AddRange(newIndices, newIndicesLength);
                }
            }
            UnsafeUtility.Free(newIndices, Allocator.TempJob);
        }
    }

#endif
}
