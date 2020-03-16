﻿using System;
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
    public struct BrushIntersectionTestData
    {
        public Bounds bounds;
//      public float4x4 treeToNodeSpaceMatrix;
//      public float4x4 nodeToTreeSpaceMatrix;
        public BlobAssetReference<BrushMeshBlob> brushMesh;
    }

    [BurstCompile(Debug = false)]
    unsafe struct IntersectionTestJob : IJobParallelFor
    {
        const double kEpsilon = CSGManagerPerformCSG.kEpsilon;

        [ReadOnly] public NativeArray<int> treeBrushes;
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<NodeTransformations>> transformations;
        [ReadOnly] public NativeArray<BrushIntersectionTestData> brushData;

        //[WriteOnly] public NativeStream.Writer output;
        [WriteOnly] public NativeMultiHashMap<int, BrushBrushIntersection>.ParallelWriter output;

        static void TransformOtherIntoBrushSpace(ref float4x4 treeToBrushSpaceMatrix, ref float4x4 brushToTreeSpaceMatrix, ref BlobArray<float4> srcPlanes, float4* dstPlanes)
        {
            var brush1ToBrush0LocalLocalSpace = math.transpose(math.mul(treeToBrushSpaceMatrix, brushToTreeSpaceMatrix));
            for (int plane_index = 0; plane_index < srcPlanes.Length; plane_index++)
            {
                ref var srcPlane = ref srcPlanes[plane_index];
                dstPlanes[plane_index] = math.mul(brush1ToBrush0LocalLocalSpace, srcPlane);
            }
        }


        static IntersectionType ConvexPolytopeTouching(BlobAssetReference<BrushMeshBlob> brushMesh0,
                                                       ref float4x4 treeToNode0SpaceMatrix,
                                                       ref float4x4 nodeToTree0SpaceMatrix,
                                                       BlobAssetReference<BrushMeshBlob> brushMesh1,
                                                       ref float4x4 treeToNode1SpaceMatrix,
                                                       ref float4x4 nodeToTree1SpaceMatrix)
        {
            ref var brushPlanes0   = ref brushMesh0.Value.localPlanes;
            ref var brushPlanes1   = ref brushMesh1.Value.localPlanes;

            ref var brushVertices0 = ref brushMesh0.Value.vertices;
            ref var brushVertices1 = ref brushMesh1.Value.vertices;

            var transformedPlanes0 = stackalloc float4[brushPlanes0.Length];
            TransformOtherIntoBrushSpace(ref treeToNode0SpaceMatrix, ref nodeToTree1SpaceMatrix, ref brushPlanes0, transformedPlanes0);
            
            int negativeSides1 = 0;
            for (var i = 0; i < brushPlanes0.Length; i++)
            {
                var plane0 = transformedPlanes0[i];
                int side = WhichSide(ref brushVertices1, plane0, kEpsilon);
                if (side < 0) negativeSides1++;
                if (side > 0) return IntersectionType.NoIntersection;
            }

            //if (intersectingSides1 != transformedPlanes0.Length) return IntersectionType.Intersection;
            //if (intersectingSides > 0) return IntersectionType.Intersection;
            //if (positiveSides1 > 0) return IntersectionType.NoIntersection;
            //if (negativeSides > 0 && positiveSides > 0) return IntersectionType.Intersection;
            if (negativeSides1 == brushPlanes0.Length)
                return IntersectionType.BInsideA;

            //*
            var transformedPlanes1 = stackalloc float4[brushPlanes1.Length];
            TransformOtherIntoBrushSpace(ref treeToNode1SpaceMatrix, ref nodeToTree0SpaceMatrix, ref brushPlanes1, transformedPlanes1);

            int negativeSides2 = 0;
            int intersectingSides2 = 0;
            for (var i = 0; i < brushPlanes1.Length; i++)
            {
                var plane1 = transformedPlanes1[i];
                int side = WhichSide(ref brushVertices0, plane1, kEpsilon);
                if (side < 0) negativeSides2++;
                if (side > 0) return IntersectionType.NoIntersection;
                if (side == 0) intersectingSides2++;
            }

            if (intersectingSides2 > 0) return IntersectionType.Intersection;
            //if (negativeSides > 0 && positiveSides > 0) return IntersectionType.Intersection;
            if (negativeSides2 == brushPlanes1.Length)
                return IntersectionType.AInsideB;
            
            return IntersectionType.Intersection;//*/
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int WhichSide(ref BlobArray<float3> vertices, float4 plane, double epsilon)
        {
            {
                var t = math.dot(plane, new float4(vertices[0], 1));
                if (t >=  epsilon) goto HavePositive;
                if (t <= -epsilon) goto HaveNegative;
                return 0;
            }
        HaveNegative:
            for (var i = 1; i < vertices.Length; i++)
            {
                var t = math.dot(plane, new float4(vertices[i], 1));
                if (t > -epsilon)
                    return 0;
            }
            return -1;
        HavePositive:
            for (var i = 1; i < vertices.Length; i++)
            {
                var t = math.dot(plane, new float4(vertices[i], 1));
                if (t < epsilon)
                    return 0;
            }
            return 1;
        }

        public void Execute(int index)
        {
            //output.BeginForEachIndex(index);

            var arrayIndex = GeometryMath.GetTriangleArrayIndex(index, treeBrushes.Length);
            var brushIndex0 = arrayIndex.x;
            var brushIndex1 = arrayIndex.y;

            /*
            var brushIndex0 = index % brushData.Length;
            var brushIndex1 = index / brushData.Length;

            // Remove this inefficiency
            if (brushIndex0 >= brushIndex1)
                return;
            */
            {
                var brushMesh0 = brushData[brushIndex0].brushMesh;
                var brushMesh1 = brushData[brushIndex1].brushMesh;

                var bounds0 = brushData[brushIndex0].bounds;
                var bounds1 = brushData[brushIndex1].bounds;

                IntersectionType result;
                if (brushMesh0 != BlobAssetReference<BrushMeshBlob>.Null &&
                    brushMesh1 != BlobAssetReference<BrushMeshBlob>.Null &&
                    bounds0.Intersects(bounds1, kEpsilon))
                {
                    var brush0NodeID = treeBrushes[brushIndex0];
                    var brush1NodeID = treeBrushes[brushIndex1];
                    //*
                    var brush0NodeIndex = brush0NodeID - 1;
                    var brush1NodeIndex = brush1NodeID - 1;

                    ref var transformation0 = ref transformations[brush0NodeIndex].Value;
                    ref var transformation1 = ref transformations[brush1NodeIndex].Value;

                    var treeToNode0SpaceMatrix = transformation0.treeToNode;
                    var nodeToTree0SpaceMatrix = transformation0.nodeToTree;
                    var treeToNode1SpaceMatrix = transformation1.treeToNode;
                    var nodeToTree1SpaceMatrix = transformation1.nodeToTree;
                    /*/
                    var treeToNode0SpaceMatrix = brushData[brushIndex0].treeToNodeSpaceMatrix;
                    var nodeToTree0SpaceMatrix = brushData[brushIndex0].nodeToTreeSpaceMatrix;
                    var treeToNode1SpaceMatrix = brushData[brushIndex1].treeToNodeSpaceMatrix;
                    var nodeToTree1SpaceMatrix = brushData[brushIndex1].nodeToTreeSpaceMatrix;
                    //*/
                    result = ConvexPolytopeTouching(brushMesh0,
                                                    ref treeToNode0SpaceMatrix,
                                                    ref nodeToTree0SpaceMatrix,
                                                    brushMesh1,
                                                    ref treeToNode1SpaceMatrix,
                                                    ref nodeToTree1SpaceMatrix);

                    if (result != IntersectionType.NoIntersection)
                    {
                        if (result == IntersectionType.Intersection)
                        {
                            output.Add(brush0NodeID, new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.Intersection });
                            output.Add(brush1NodeID, new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.Intersection });
                        } else
                        if (result == IntersectionType.AInsideB)
                        {
                            output.Add(brush0NodeID, new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.AInsideB });
                            output.Add(brush1NodeID, new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.BInsideA });
                        } else
                        //if (intersectionType == IntersectionType.BInsideA)
                        {
                            output.Add(brush0NodeID, new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.BInsideA });
                            output.Add(brush1NodeID, new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.AInsideB });
                        }
                    }
                }
            }
            //output.EndForEachIndex();
        }
    }
}
