﻿using System;
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
    [BurstCompile]
    struct CreateBlobPolygonsBlobs : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> treeBrushes;
        [ReadOnly] public NativeArray<int> brushMeshInstanceIDs;
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<NodeTransformations>> transformations;
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>> brushMeshBlobs;
        [WriteOnly] public NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>>.ParallelWriter basePolygons;

        public void Execute(int b)
        {
            var brushNodeID     = treeBrushes[b];
            var brushNodeIndex  = brushNodeID - 1;
            var brushMeshID     = brushMeshInstanceIDs[b];
            var transform       = transformations[brushNodeID - 1];

            var mesh    = brushMeshBlobs[brushMeshID - 1];
            var result  = BasePolygonsBlob.Create(brushNodeIndex, mesh, transform);
            basePolygons.TryAdd(brushNodeIndex, result);
        }
    }
}