using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;
using System.Runtime.CompilerServices;

namespace Chisel.Core
{
    unsafe struct FindAllIntersectionBrushesJob : IJobParallelFor
    {/*
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kPlaneDistanceEpsilon;

        [ReadOnly] public NativeArray<float4>  intersectingPlanes2;
        [ReadOnly] public NativeArray<float4>  intersectingPlanes1;
        [ReadOnly] public NativeArray<int>     intersectingPlaneIndices1;
        [ReadOnly] public NativeArray<int>     usedVertices1;
        [ReadOnly] public NativeArray<float3>  allVertices1;
        [ReadOnly] public VertexSoup           brushVertices1;

        public float4x4 nodeToTreeSpaceMatrix;
        public float4x4 vertexToLocal1;
        */
        [WriteOnly] public NativeStream.Writer foundBrushIntersections;

        public void Execute(int index)
        {
            foundBrushIntersections.BeginForEachIndex(index);

            foundBrushIntersections.EndForEachIndex();
        }
    }
}
