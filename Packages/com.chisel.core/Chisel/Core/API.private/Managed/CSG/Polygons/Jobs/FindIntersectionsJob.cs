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

    [BurstCompile]
    unsafe struct FindIntersectionsJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon   = CSGManagerPerformCSG.kPlaneDistanceEpsilon;
        const float kNormalEpsilon          = CSGManagerPerformCSG.kNormalEpsilon;

        [ReadOnly] public NativeArray<CSGManagerPerformCSG.PlanePair> usedPlanePairs;
        [ReadOnly] public NativeArray<float4> intersectingPlanes1;
        [ReadOnly] public NativeArray<float4> intersectingPlanes2;
        [ReadOnly] public float4x4 nodeToTreeSpaceMatrix1;

        [WriteOnly] public NativeStream.Writer  foundVertices;

            
        public void Execute(int index)
        {
            foundVertices.BeginForEachIndex(index);
            var plane2 = intersectingPlanes1[index];

            for (int i = 0; i < usedPlanePairs.Length; i++)
            {
                var planePair = usedPlanePairs[i];

                // should never happen
                if (planePair.P0 == planePair.P1)
                    continue;

                var plane0 = planePair.Plane0;
                var plane1 = planePair.Plane1;


                // FIXME: Fix situation when plane intersects edge but is not identical to either planes of the edge ...
                if (!(math.abs(plane0.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) > kNormalEpsilon) &&
                    !(math.abs(plane1.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) > kNormalEpsilon) &&

                    !(math.abs(plane0.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) < -kNormalEpsilon) &&
                    !(math.abs(plane1.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) < -kNormalEpsilon))
                {
                    var localVertex = new float4(PlaneExtensions.Intersection(plane2, plane0, plane1), 1);

                    // TODO: since we're using a pair in the outer loop, we could also determine which 
                    //       2 planes it intersects at both ends and just check those two planes ..

                    // NOTE: for brush2, the intersection will always be only on two planes
                    //       UNLESS it's a corner vertex along that edge (we can compare to the two vertices)
                    //       in which case we could use a pre-calculated list of planes ..
                    //       OR when the intersection is outside of the edge ..

                    if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes2, localVertex) &&
                        !CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes1, localVertex))
                    {
                        var worldVertex = math.mul(nodeToTreeSpaceMatrix1, localVertex).xyz;
                        foundVertices.Write(worldVertex);
                        foundVertices.Write(new int2(planePair.P0, planePair.P1));
                    }
                }
            }

            foundVertices.EndForEachIndex();
        }
    }

    [BurstCompile]
    unsafe struct InsertIntersectionVerticesJob : IJob
    {
        [ReadOnly] public NativeArray<int> intersectingPlaneIndices1;
        [ReadOnly] public NativeStream.Reader vertexReader;

        public VertexSoup brushVertices1;
        public VertexSoup brushVertices2;

        [WriteOnly] public NativeList<int2> foundIndices1;
        [WriteOnly] public NativeList<int2> foundIndices2;

        public void Execute()
        {
            int maxIndex = vertexReader.ForEachCount;

            int index = 0;
            vertexReader.BeginForEachIndex(index++);
            while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                vertexReader.BeginForEachIndex(index++);
            while (vertexReader.RemainingItemCount > 0)
            {
                var worldVertex = vertexReader.Read<float3>();
                var planePair = vertexReader.Read<int2>();

                // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex
                var vertexIndex1 = brushVertices1.Add(worldVertex);
                var vertexIndex2 = brushVertices2.Add(worldVertex);

                foundIndices1.Add(new int2(intersectingPlaneIndices1[index - 1], vertexIndex1));
                foundIndices2.Add(new int2(planePair.x, vertexIndex2));
                foundIndices2.Add(new int2(planePair.y, vertexIndex2));

                while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                    vertexReader.BeginForEachIndex(index++);
            }
        }
    }


    unsafe struct InsertIntersectionIndicesJob : IJob
    {
        [ReadOnly] public NativeList<int2> foundIndices;
        [ReadOnly] public CSGTreeBrush brush;

        [WriteOnly] public LoopGroup holeLoops;

        public void Execute()
        {
            for (int i = 0; i < foundIndices.Length; i++)
            {
                var indices = foundIndices[i];
                holeLoops.FindOrAddLoop(indices.x, brush).AddIndex((ushort)indices.y);
            }
        }
    }
}
