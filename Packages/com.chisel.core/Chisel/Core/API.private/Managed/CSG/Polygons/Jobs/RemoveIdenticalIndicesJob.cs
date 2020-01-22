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
        public List<ushort> indices;
        public List<float3> vertices;

        // TODO: optimize
        public void Execute()
        {
            while (indices.Count > 3 && indices[indices.Count - 1] == indices[0])
                indices.RemoveAt(indices.Count - 1);

            for (int v0 = indices.Count - 2, v1 = indices.Count - 1; indices.Count > 3 && v0 >= 0; v1 = v0, v0--)
            {
                if (indices[v0] != indices[v1])
                    continue;

                indices.RemoveAt(v1);
            }
        }
    }

#endif
}
