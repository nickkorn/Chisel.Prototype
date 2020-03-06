using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using UnityEngine;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Entities;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    internal class BrushIntersection
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [BurstCompile]
        unsafe static void TransformOtherIntoBrushSpace(CSGTreeBrush brush0, CSGTreeBrush brush1, ref BlobArray<float4> srcPlanes, float4* dstPlanes)
        {
            // inverse of (otherTransform.localToWorldSpace * this->worldToLocalSpace)

            var treeToBrush0SpaceMatrix = brush0.TreeToNodeSpaceMatrix;
            var brushToTree1SpaceMatrix = brush1.NodeToTreeSpaceMatrix;

            var brush1ToBrush0LocalLocalSpace = math.transpose(math.mul(treeToBrush0SpaceMatrix, brushToTree1SpaceMatrix));
            for (int plane_index = 0; plane_index < srcPlanes.Length; plane_index++)
            {
                ref var srcPlane = ref srcPlanes[plane_index];
                dstPlanes[plane_index] = math.mul(brush1ToBrush0LocalLocalSpace, srcPlane);
            }
        }


        [BurstCompile]
        unsafe internal static IntersectionType ConvexPolytopeTouching(CSGTreeBrush brush0, CSGTreeBrush brush1, double epsilon)
        {
            var brushMesh0 = BrushMeshManager.GetBrushMeshBlob(brush0.BrushMesh.brushMeshID);
            var brushMesh1 = BrushMeshManager.GetBrushMeshBlob(brush1.BrushMesh.brushMeshID);
            if (brushMesh0 == BlobAssetReference<BrushMeshBlob>.Null ||
                brushMesh1 == BlobAssetReference<BrushMeshBlob>.Null)
                return IntersectionType.NoIntersection;

            ref var brushPlanes0   = ref brushMesh0.Value.planes;
            ref var brushPlanes1   = ref brushMesh1.Value.planes;

            ref var brushVertices0 = ref brushMesh0.Value.vertices;
            ref var brushVertices1 = ref brushMesh1.Value.vertices;

            var transformedPlanes0 = stackalloc float4[brushPlanes0.Length];
            TransformOtherIntoBrushSpace(brush0, brush1, ref brushPlanes0, transformedPlanes0);
            
            int negativeSides1 = 0;
            int positiveSides1 = 0;
            int intersectingSides1 = 0;
            for (var i = 0; i < brushPlanes0.Length; i++)
            {
                var plane0 = transformedPlanes0[i];
                int side = WhichSide(ref brushVertices1, plane0, epsilon);
                if (side < 0) negativeSides1++;
                if (side > 0) positiveSides1++;
                if (side == 0) intersectingSides1++;
            }

            //Debug.Log($"A positive: {positiveSides1} negative: {negativeSides1} intersecting: {intersectingSides1} / {transformedPlanes0.Length}");

            //if (intersectingSides1 != transformedPlanes0.Length) return IntersectionType.Intersection;
            //if (intersectingSides > 0) return IntersectionType.Intersection;
            if (positiveSides1 > 0) return IntersectionType.NoIntersection;
            //if (negativeSides > 0 && positiveSides > 0) return IntersectionType.Intersection;
            if (negativeSides1 == brushPlanes0.Length)
                return IntersectionType.BInsideA;

            //*
            var transformedPlanes1 = stackalloc float4[brushPlanes1.Length];
            TransformOtherIntoBrushSpace(brush1, brush0, ref brushPlanes1, transformedPlanes1);

            int negativeSides2 = 0;
            int positiveSides2 = 0;
            int intersectingSides2 = 0;
            for (var i = 0; i < brushPlanes1.Length; i++)
            {
                var plane1 = transformedPlanes1[i];
                int side = WhichSide(ref brushVertices0, plane1, epsilon);
                if (side < 0) negativeSides2++;
                if (side > 0) positiveSides2++;
                if (side == 0) intersectingSides2++;
            }

            //Debug.Log($"B positive: {positiveSides2} negative: {negativeSides2} intersecting: {intersectingSides2} / {transformedPlanes1.Length}");

            if (positiveSides2     > 0) return IntersectionType.NoIntersection;
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
    }
#endif
}
