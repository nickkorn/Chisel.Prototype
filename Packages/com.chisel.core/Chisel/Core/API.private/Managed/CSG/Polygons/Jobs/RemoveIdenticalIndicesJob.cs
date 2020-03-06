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
    
    public struct RemoveIdenticalIndicesJob : IJob
    {
        public NativeList<ushort> indices;

        // TODO: optimize
        public void Execute()
        {
            var newIndices = new List<ushort>(indices.Length);
            for (int i = 0; i < indices.Length; i++)
                newIndices.Add(indices[i]);

            while (newIndices.Count > 3 && newIndices[newIndices.Count - 1] == newIndices[0])
                newIndices.RemoveAt(newIndices.Count - 1);

            // TODO: fix this mess
            for (int v0 = newIndices.Count - 2, v1 = newIndices.Count - 1; newIndices.Count > 3 && v0 >= 0; v1 = v0, v0--)
            {
                if (indices[v0] != indices[v1])
                    continue;

                newIndices.RemoveAt(v1);
            }
            indices.Clear();
            for (int i = 0; i < newIndices.Count; i++)
                indices.Add(newIndices[i]);
        }
    }

#endif
}
