using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
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

        public static void RemoveDuplicates(ref NativeList<ushort> indices)
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

        public void Execute()
        {
            RemoveDuplicates(ref indices);
        }
    }

    [BurstCompile]
    public unsafe struct CopyPolygonToIndicesJob : IJob
    {
        [ReadOnly] public BlobAssetReference<BrushMeshBlob> mesh;
        [ReadOnly] public int       polygonIndex;
        [ReadOnly] public float4x4  nodeToTreeSpaceMatrix;

        public VertexSoup           vertexSoup;
        public NativeList<ushort>   indices;

        public float3               min, max;

        public void Execute()
        {
            ref var halfEdges = ref mesh.Value.halfEdges;
            ref var vertices  = ref mesh.Value.vertices;

            ref var polygon   = ref mesh.Value.polygons[polygonIndex];
            var firstEdge = polygon.firstEdge;
            var lastEdge  = firstEdge + polygon.edgeCount;
            var indexCount = lastEdge - firstEdge;

            vertexSoup.Reserve(indexCount); // ensure we have at least this many extra vertices in capacity

            // TODO: put in job so we can burstify this, maybe join with RemoveIdenticalIndicesJob & IsDegenerate?
            for (int e = firstEdge; e < lastEdge; e++)
            {
                var vertexIndex = halfEdges[e].vertexIndex;
                var localVertex = new float4(vertices[vertexIndex], 1);
                var worldVertex = math.mul(nodeToTreeSpaceMatrix, localVertex);
                min.x = math.min(min.x, worldVertex.x); max.x = math.max(max.x, worldVertex.x);
                min.y = math.min(min.y, worldVertex.y); max.y = math.max(max.y, worldVertex.y);
                min.z = math.min(min.z, worldVertex.z); max.z = math.max(max.z, worldVertex.z);

                var newIndex = vertexSoup.AddNoResize(worldVertex.xyz);
                //Debug.Assert(indices.Length == 0 || (indices[0] != newIndex && indices[indices.Length - 1] != newIndex));
                indices.Add(newIndex);
            }

            RemoveIdenticalIndicesJob.RemoveDuplicates(ref indices);

            if (indices.Length > 0 &&
                CSGManagerPerformCSG.IsDegenerate(vertexSoup, indices))
            {
                indices.Clear();
            }
        }
    }
#endif
}
