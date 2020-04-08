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
    [BurstCompile(CompileSynchronously = true)]
    public struct CreateBrushWorldPlanesJob : IJobParallelFor   
    {
        [NoAlias,ReadOnly] public NativeArray<int> treeBrushIDs;
        [NoAlias,ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>         brushMeshLookup;
        [NoAlias,ReadOnly] public NativeHashMap<int, BlobAssetReference<NodeTransformations>>   transformations;

        [NoAlias,WriteOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>.ParallelWriter brushWorldPlanes;

        public void Execute(int index)
        {
            var brushNodeID     = treeBrushIDs[index];
            var brushNodeIndex  = brushNodeID - 1;
            var worldPlanes     = BrushWorldPlanes.Build(brushMeshLookup[brushNodeIndex], 
                                                         transformations[brushNodeIndex].Value.nodeToTree);
            brushWorldPlanes.TryAdd(brushNodeIndex, worldPlanes);
        }
    }
}
