using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Chisel.Core
{
    public struct BrushWorldPlanes
    {
        public BlobArray<float4> worldPlanes;

        public static BlobAssetReference<BrushWorldPlanes> BuildPlanes(BlobAssetReference<BrushMeshBlob> brushMeshBlob, float4x4 nodeToTreeTransformation)
        {
            if (!brushMeshBlob.IsCreated)
                return BlobAssetReference<BrushWorldPlanes>.Null;

            var nodeToTreeInverseTransposed = math.transpose(math.inverse(nodeToTreeTransformation));
            using (var builder = new BlobBuilder(Allocator.Temp))
            {
                ref var root = ref builder.ConstructRoot<BrushWorldPlanes>();
                var worldPlaneArray = builder.Allocate(ref root.worldPlanes, brushMeshBlob.Value.localPlanes.Length);
                for (int i = 0; i < brushMeshBlob.Value.localPlanes.Length; i++)
                {
                    var localPlane = brushMeshBlob.Value.localPlanes[i];
                    worldPlaneArray[i] = math.mul(nodeToTreeInverseTransposed, localPlane);
                }
                return builder.CreateBlobAssetReference<BrushWorldPlanes>(Allocator.Persistent);
            }
        }
    }
}
