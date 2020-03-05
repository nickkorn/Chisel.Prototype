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
using Unity.Entities;

namespace Chisel.Core
{

    [BurstCompile]
    unsafe struct FindInsideVerticesJob : IJobParallelFor
    { 
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<float4>  intersectingPlanes2;
        [ReadOnly] public NativeList<int>     usedVertices1;
        [NativeDisableUnsafePtrRestriction] [ReadOnly] public float3* allVertices1;
        [ReadOnly] public float4x4            nodeToTreeSpaceMatrix;
        [ReadOnly] public float4x4            vertexToLocal1;

        [NativeDisableUnsafePtrRestriction]
        [WriteOnly] public NativeStream.Writer vertexWriter;

        public void Execute(int index)
        {
            vertexWriter.BeginForEachIndex(index);
            var vertexIndex1 = usedVertices1[index];
            var brushVertex1 = new float4(allVertices1[vertexIndex1], 1);
            var localVertex1 = math.mul(vertexToLocal1, brushVertex1);
            if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes2, localVertex1))
            { 
                var worldVertex = math.mul(nodeToTreeSpaceMatrix, brushVertex1);
                vertexWriter.Write(localVertex1);
                vertexWriter.Write(worldVertex);
            }
            vertexWriter.EndForEachIndex();
        }
    }
    
    [BurstCompile]
    unsafe struct InsertInsideVerticesJob : IJob
    {
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kPlaneDistanceEpsilon;

        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeStream.Reader   vertexReader;
        [ReadOnly] public NativeList<float4>    intersectingPlanes;
        [ReadOnly] public NativeList<int>       intersectingPlaneIndices;

        public VertexSoup                       brushVertices;

        [WriteOnly] public NativeList<PlaneVertexIndexPair> foundIndices;


        public void Execute() 
        {
            int maxIndex = vertexReader.ForEachCount;

            int index = 0;
            vertexReader.BeginForEachIndex(index++);
            while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                vertexReader.BeginForEachIndex(index++);
            while (vertexReader.RemainingItemCount > 0)
            {
                var localVertex1    = vertexReader.Read<float4>();
                var worldVertex     = vertexReader.Read<float4>();

                var worldVertexIndex = -1;
                // TODO: optimize this, we already know these vertices are ON the planes of this brush, just not which: this can be precalculated
                for (int i = 0; i < intersectingPlaneIndices.Length; i++)
                {
                    // only use a plane when the vertex is ON it
                    var distance = math.dot(intersectingPlanes[i], localVertex1);
                    if (distance >= -kPlaneDistanceEpsilon && distance <= kPlaneDistanceEpsilon) // Note: this is false on NaN/Infinity, so don't invert
                    {
                        var planeIndex = intersectingPlaneIndices[i];
                        if (worldVertexIndex == -1)
                            worldVertexIndex = brushVertices.Add(worldVertex.xyz);
                        foundIndices.Add(new PlaneVertexIndexPair() { planeIndex = (ushort)planeIndex, vertexIndex = (ushort)worldVertexIndex });
                    }
                }

                while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                    vertexReader.BeginForEachIndex(index++);
            }
        }
    }
}
