using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using UnityEngine;
using System.Runtime.CompilerServices;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    internal class BrushIntersection
    {
        internal static IntersectionType ConvexPolytopeTouching(CSGTreeBrush brush0, CSGTreeBrush brush1, double epsilon)
        {
            var brushMesh0          = BrushMeshManager.GetBrushMesh(brush0.BrushMesh.brushMeshID);
            var brushMesh1          = BrushMeshManager.GetBrushMesh(brush1.BrushMesh.brushMeshID);

            var brushSurfaces0      = brushMesh0.surfaces;
            var brushSurfaces1      = brushMesh1.surfaces;

            var transformedPlanes0   = TransformOtherIntoBrushSpace(brush0, brush1, brushSurfaces0);

            int negativeSides = 0;
            int positiveSides = 0;
            int intersectingSides = 0;
            for (var i = 0; i < transformedPlanes0.Length; i++)
            {
                var plane0  = transformedPlanes0[i];
                int side    = WhichSide(brushMesh1.vertices, plane0, epsilon);
                if (side < 0) negativeSides++;
                if (side > 0) positiveSides++;
                if (side == 0) intersectingSides++;
            }
            
            //Debug.Log($"A {positiveSides} {negativeSides}/{transformedPlanes0.Length} {intersectingSides}");
            
            if (intersectingSides > 0) return IntersectionType.Intersection;
            if (positiveSides > 0) return IntersectionType.NoIntersection;
            //if (negativeSides > 0 && positiveSides > 0) return IntersectionType.Intersection;
            if (negativeSides == transformedPlanes0.Length)
                return IntersectionType.BInsideA;
            //*
            var transformedPlanes1 = TransformOtherIntoBrushSpace(brush1, brush0, brushSurfaces1);

            negativeSides = 0;
            positiveSides = 0;
            intersectingSides = 0;
            for (var i = 0; i < transformedPlanes1.Length; i++)
            {
                var plane1  = transformedPlanes1[i];
                int side    = WhichSide(brushMesh0.vertices, plane1, epsilon);
                if (side < 0) negativeSides++;
                if (side > 0) positiveSides++;
                if (side == 0) intersectingSides++;
            }

            //Debug.Log($"B {positiveSides} {negativeSides} {intersectingSides}");

            if (intersectingSides > 0) return IntersectionType.Intersection;
            if (positiveSides > 0) return IntersectionType.NoIntersection;
            //if (negativeSides > 0 && positiveSides > 0) return IntersectionType.Intersection;
            if (negativeSides == transformedPlanes1.Length)
                return IntersectionType.AInsideB;

            return IntersectionType.Intersection;//*/
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int WhichSide(Vector3[] vertices, Plane plane, double epsilon)
        {
            {
                float t = plane.GetDistanceToPoint(vertices[0]);
                if (t >= epsilon) goto HavePositive;
                if (t <= -epsilon) goto HaveNegative;
                return 0;
            }
        HaveNegative:
            for (var i = 1; i < vertices.Length; i++)
            {
                float t = plane.GetDistanceToPoint(vertices[i]);
                if (t > -epsilon)
                    return 0;
            }
            return -1;
        HavePositive:
            for (var i = 1; i < vertices.Length; i++)
            {
                float t = plane.GetDistanceToPoint(vertices[i]);
                if (t < epsilon)
                    return 0;
            }
            return 1;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static Plane[] TransformOtherIntoBrushSpace(CSGTreeBrush brush0, CSGTreeBrush brush1, BrushMesh.Surface[] srcPlanes1)
        {
            // inverse of (otherTransform.localToWorldSpace * this->worldToLocalSpace)

            var brush0TreeSpaceMatrix = brush0.NodeToTreeSpaceMatrix;
            var brush1TreeSpaceMatrix = brush1.TreeToNodeSpaceMatrix;

            var brush1ToBrush0LocalLocalSpace = brush1TreeSpaceMatrix * brush0TreeSpaceMatrix;

            var dstPlanes = new Plane[srcPlanes1.Length];
            for (int plane_index = 0; plane_index < srcPlanes1.Length; plane_index++)
                dstPlanes[plane_index] = brush1ToBrush0LocalLocalSpace.TransformPlane(srcPlanes1[plane_index].localPlane);

            return dstPlanes;
        }
    }
#endif
}
