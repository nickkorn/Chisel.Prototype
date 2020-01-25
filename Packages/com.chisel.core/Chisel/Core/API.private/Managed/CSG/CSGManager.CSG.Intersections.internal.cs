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
#if USE_MANAGED_CSG_IMPLEMENTATION
    static partial class CSGManagerPerformCSG
    {
        #region GetIntersectionLoops

        #region GetIntersectingPlanes / GetBrushPlanes

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetIntersectingPlanes(List<int> intersectingPlanes, in BrushMesh.Surface[] otherSurfaces, in Bounds selfBounds, in Vector3[] vertices, in float4x4 treeToNodeSpaceInverseTransposed)
        {
            intersectingPlanes.Clear();
            if (intersectingPlanes.Capacity < otherSurfaces.Length)
                intersectingPlanes.Capacity = otherSurfaces.Length;

            var min = (float3)selfBounds.min;
            var max = (float3)selfBounds.max;

            for (int i = 0; i < otherSurfaces.Length; i++)
            {
                // bring plane into local space of mesh, the same space as the bounds of the mesh
                
                var localPlane  = (float4)otherSurfaces[i].localPlane;

                // note: a transpose is part of this transformation
                var transformedPlane    = math.mul(treeToNodeSpaceInverseTransposed, localPlane);
                var normal              = transformedPlane.xyz; // only need the signs, so don't care about normalization
                //transformedPlane /= math.length(normal);      // we don't have to normalize the plane
                
                var corner = new float4((normal.x < 0) ? max.x : min.x,
                                        (normal.y < 0) ? max.y : min.y,
                                        (normal.z < 0) ? max.z : min.z,
                                        1.0f);
                float forward = math.dot(transformedPlane, corner);
                if (forward > kDistanceEpsilon) // closest point is outside
                {
                    intersectingPlanes.Clear();
                    return;
                }

                // do a bounds check
                corner = new float4((normal.x >= 0) ? max.x : min.x,
                                        (normal.y >= 0) ? max.y : min.y,
                                        (normal.z >= 0) ? max.z : min.z,
                                        1.0f);
                float backward = math.dot(transformedPlane, corner);
                if (backward < -kDistanceEpsilon) // closest point is inside
                    continue;

                float minDistance = float.PositiveInfinity;
                float maxDistance = float.NegativeInfinity;
                int onCount = 0;
                for (int v = 0; v < vertices.Length; v++)
                {
                    float distance = math.dot(transformedPlane, new float4(vertices[v], 1));
                    minDistance = math.min(distance, minDistance);
                    maxDistance = math.max(distance, maxDistance);
                    onCount += (distance >= -kDistanceEpsilon && distance <= kDistanceEpsilon) ? 1 : 0;
                }

                // if all vertices are 'inside' this plane, then we're not truly intersecting with it
                if ((minDistance >= -kDistanceEpsilon ||
                    maxDistance < -kDistanceEpsilon)
                    && onCount < 3) // If we have a polygon intersecting with a plane, we use it
                    continue;

                intersectingPlanes.Add(i);
            }
        }

        // TODO: Optimize
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [BurstCompile]
        static unsafe void TransformByMatrix(float4* outputPlanes, float4* surfaces, int length, float4x4 nodeToTreeSpace)
        {
            var planeTransform = math.transpose(math.inverse(nodeToTreeSpace));
            for (int p = 0; p < length; p++)
            {
                var planeVector = math.mul(planeTransform, surfaces[p]);
                outputPlanes[p] = planeVector / math.length(planeVector.xyz);
            }
        }

        // TODO: Optimize
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [BurstCompile]
        static unsafe void TransformByTransposedInversedMatrix(float4* outputPlanes, float4* surfaces, int length, float4x4 nodeToTreeSpaceInversed)
        {
            for (int p = 0; p < length; p++)
            {
                var planeVector = math.mul(nodeToTreeSpaceInversed, surfaces[p]);
                outputPlanes[p] = planeVector / math.length(planeVector.xyz);
            }
        }

        #endregion

        #region SortIndices

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float3 FindPolygonCentroid(List<ushort> indices, List<float3> vertices)
        {
            var centroid = float3.zero;
            for (int i = 0; i < indices.Count; i++)
                centroid += vertices[indices[i]];
            return centroid / indices.Count;
        }

        /*
        static void SortIndices(List<ushort> indices, List<float3> vertices, float3 tangentX, float3 tangentY, float2 center, int l, int r)
        {
            var left    = l;
            var right   = r;
            var va      = (float3)vertices[indices[(left + right) / 2]]; 
            while (true)
            {
                var a_angle = math.atan2(math.dot(tangentX, va) - center.x, math.dot(tangentY, va) - center.y);

                {
                    var vb      = (float3)vertices[indices[left]];
                    var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);

                    while (b_angle > a_angle)
                    {
                        left++; 
                        vb = (float3)vertices[indices[left]];
                        b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                    }
                }

                { 
                    var vb      = (float3)vertices[indices[right]];
                    var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                    while (a_angle > b_angle)
                    {
                        right--;
                        vb = (float3)vertices[indices[right]];
                        b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                    }
                }

                if (left <= right)
                {
                    if (left != right)
                    {
                        var t = indices[left];
                        indices[left] = indices[right];
                        indices[right] = t;
                    }

                    left++;
                    right--;
                }
                if (left > right)
                    break;
            }
            if (l < right)
                SortIndices(indices, vertices, tangentX, tangentY, center, l, right);
            if (left < r)
                SortIndices(indices, vertices, tangentX, tangentY, center, left, r);
        }
        */
        static List<int2> s_SortStack = new List<int2>();
        // TODO: sort by using plane information instead of unreliable floating point math ..
        // TODO: make this work on non-convex polygons
        static void SortIndices(List<ushort> indices, List<float3> vertices, float3 normal)
        {
            // There's no point in trying to sort a point or a line 
            if (indices.Count < 3)
                return;

            float3 tangentX, tangentY;
            if (normal.x > normal.y)
            {
                if (normal.x > normal.z)
                {
                    tangentX = math.cross(normal, new float3(0, 1, 0));
                    tangentY = math.cross(normal, tangentX);
                } else
                {
                    tangentX = math.cross(normal, new float3(0, 0, 1));
                    tangentY = math.cross(normal, tangentX);
                }
            } else
            {
                if (normal.y > normal.z)
                {
                    tangentX = math.cross(normal, new float3(1, 0, 0));
                    tangentY = math.cross(normal, tangentX);
                } else
                {
                    tangentX = math.cross(normal, new float3(0, 1, 0));
                    tangentY = math.cross(normal, tangentX);
                }
            }

            var centroid = FindPolygonCentroid(indices, vertices);
            var center = new float2(math.dot(tangentX, centroid), // distance in direction of tangentX
                                    math.dot(tangentY, centroid)); // distance in direction of tangentY
#if true
            s_SortStack.Clear();
            s_SortStack.Add(new int2(0, indices.Count - 1));
            while (s_SortStack.Count > 0)
            {
                var top = s_SortStack[s_SortStack.Count - 1];
                s_SortStack.RemoveAt(s_SortStack.Count - 1);
                var l = top.x;
                var r = top.y;
                var left = l;
                var right = r;
                var va = (float3)vertices[indices[(left + right) / 2]];
                while (true)
                {
                    var a_angle = math.atan2(math.dot(tangentX, va) - center.x, math.dot(tangentY, va) - center.y);

                    {
                        var vb = (float3)vertices[indices[left]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (b_angle > a_angle)
                        {
                            left++;
                            vb = (float3)vertices[indices[left]];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    {
                        var vb = (float3)vertices[indices[right]];
                        var b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        while (a_angle > b_angle)
                        {
                            right--;
                            vb = (float3)vertices[indices[right]];
                            b_angle = math.atan2(math.dot(tangentX, vb) - center.x, math.dot(tangentY, vb) - center.y);
                        }
                    }

                    if (left <= right)
                    {
                        if (left != right)
                        {
                            var t = indices[left];
                            indices[left] = indices[right];
                            indices[right] = t;
                        }

                        left++;
                        right--;
                    }
                    if (left > right)
                        break;
                }
                if (l < right)
                {
                    s_SortStack.Add(new int2(l, right));
                    //SortIndices(indices, vertices, tangentX, tangentY, center, l, right);
                }
                if (left < r)
                {
                    s_SortStack.Add(new int2(left, r));
                    //SortIndices(indices, vertices, tangentX, tangentY, center, left, r);
                }
            }
            //SortIndices(indices, vertices, tangentX, tangentY, center, 0, indices.Count - 1);
#else
            // sort vertices according to their angle relative to the centroid on plane defined by tangentX/tangentY
            vertices.Sort(delegate (Vector3 a, Vector3 b) 
            {
                var ax = math.dot(tangentX, a) - center.x;  // distance in direction of tangentX
                var ay = math.dot(tangentY, a) - center.y;  // distance in direction of tangentY
                var bx = math.dot(tangentX, b) - center.x;  // distance in direction of tangentX
                var by = math.dot(tangentY, b) - center.y;  // distance in direction of tangentY

                var a1 = math.atan2(ax, ay); // angle between ax/ay and cx/cy
                var a2 = math.atan2(bx, by); // angle between bx/by and cx/cy
                return (int)math.sign(a2 - a1);
            });
#endif
        }

        #endregion

        #region Phases

        struct PlanePair
        {
            public float4 Plane0;
            public float4 Plane1;
            //public double4 N0;
            //public double4 N1;
            public int P0;
            public int P1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe bool IsOutsidePlanes(float4[] planes, int planeCount, float4 localVertex)
        {
            for (int n = 0; n < planeCount; n++)
            {
                var distance = math.dot(planes[n], localVertex);
                if (distance > CSGManagerPerformCSG.kDistanceEpsilon) return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool IsOutsidePlanes(List<int> planeIndices, float4[] planes, float4 localVertex)
        {
            for (int n = 0; n < planeIndices.Count; n++)
            {
                var distance = math.dot(planes[planeIndices[n]], localVertex);
                if (distance > CSGManagerPerformCSG.kDistanceEpsilon) return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe bool IsOutsidePlanes(List<int> planeIndices, float4* planes, float4 localVertex)
        {
            for (int n = 0; n < planeIndices.Count; n++)
            {
                var distance = math.dot(planes[planeIndices[n]], localVertex);
                if (distance > CSGManagerPerformCSG.kDistanceEpsilon) return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe bool IsOutsidePlanes(float4* planes, int length, float4 localVertex)
        {
            const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;
            for (int n = 0; n < length; n++)
            {
                var distance = math.dot(planes[n], localVertex);
                
                // will be 'false' when distance is NaN or Infinity
                if (!(distance <= kEpsilon))
                    return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool IsOutsidePlanes(float4[] planes, float4 localVertex)
        {
            for (int n = 0; n < planes.Length; n++)
            {
                var distance = math.dot(planes[n], localVertex);
                //Debug.Log($"[{n}/{planes.Length}] {planes[n]} {distance} {localVertex}");
                if (distance > CSGManagerPerformCSG.kDistanceEpsilon) 
                    return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool IsInsidePlanes(float4[] planes, float4 localVertex)
        {
            for (int n = 0; n < planes.Length; n++)
            {
                var distance = math.dot(planes[n], localVertex);
                //Debug.Log($"[{n}/{planes.Length}] {planes[n]} {distance} {localVertex}");
                if (distance < -CSGManagerPerformCSG.kDistanceEpsilon)
                    return true;
            }
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FindPlanePairs(HashSet<int>       usedVertices1,
                                   List<PlanePair>    usedPlanePairs1,
                                   in List<int>       intersectingPlanes1,
                                   in float4[]        localSpacePlanes1,
                                   in BrushMesh       mesh1)
        {
            // TODO: this can be partially stored in brushmesh 
            // TODO: optimize
            usedVertices1.Clear();
            usedPlanePairs1.Clear();
            if (usedPlanePairs1.Capacity < mesh1.halfEdges.Length)
                usedPlanePairs1.Capacity = mesh1.halfEdges.Length;

            for (int e = 0; e < mesh1.halfEdges.Length; e++)
            {
                var twinIndex = mesh1.halfEdges[e].twinIndex;
                if (twinIndex < e)
                    continue;

                var pI0 = mesh1.halfEdgePolygonIndices[e];
                var pI1 = mesh1.halfEdgePolygonIndices[twinIndex];

                var sI0 = mesh1.polygons[pI0].surfaceID;
                var sI1 = mesh1.polygons[pI1].surfaceID;
                if (!intersectingPlanes1.Contains(sI0) ||
                    !intersectingPlanes1.Contains(sI1))
                    continue;

                var vI0 = mesh1.halfEdges[e].vertexIndex;
                var vI1 = mesh1.halfEdges[twinIndex].vertexIndex;

                //PlaneExtensions.IntersectionFirst(localSpacePlanes1[pI0], localSpacePlanes1[pI1], out double4 N0, out double4 N1);

                usedVertices1.Add(vI0);
                usedVertices1.Add(vI1);
                usedPlanePairs1.Add(new PlanePair()
                {
                    Plane0 = localSpacePlanes1[pI0],
                    Plane1 = localSpacePlanes1[pI1],
                    P0 = sI0,
                    P1 = sI1,
                    //N0 = N0,
                    //N1 = N1,
                });
            }
        }


        [BurstCompile]
        unsafe struct FindIntersectionsJob : IJobParallelFor
        {
            public float4x4             nodeToTreeSpaceMatrix1;
            public float4               plane0;
            public float4               plane1;
            [NativeDisableUnsafePtrRestriction] [ReadOnly] public float4*   intersectingPlanes1;
            [NativeDisableUnsafePtrRestriction] [ReadOnly] public float4*   intersectingPlanes2;
            public int                  intersectingPlanes1Length;
            public int                  intersectingPlanes2Length;

            [WriteOnly] public NativeStream.Writer  foundVertices;
            
            public void Execute(int index)
            {
                foundVertices.BeginForEachIndex(index);
                var plane2 = intersectingPlanes1[index];

                if (!(math.abs(plane0.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) > kNormalEpsilon) &&
                    !(math.abs(plane1.w - plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) > kNormalEpsilon) &&

                    !(math.abs(plane0.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane0.xyz, plane2.xyz) < -kNormalEpsilon) &&
                    !(math.abs(plane1.w + plane2.w) < kPlaneDistanceEpsilon && math.dot(plane1.xyz, plane2.xyz) < -kNormalEpsilon))
                {
                    var localVertex = new float4(PlaneExtensions.Intersection(plane2, plane0, plane1), 1);

                    // FIXME: sometimes we have two planes of a brush2 intersecting one plane on brush1, 
                    //		  and even though it's outside of brush1, it's still *just* within kDistanceEpsilon
                    //		  and can cause issues .. we need to find a better way of doing this

                    // TODO: since we're using a pair in the outer loop, we could also determine which 
                    //       2 planes it intersects at both ends and just check those two planes ..

                    // NOTE: for brush2, the intersection will always be only on two planes
                    //       UNLESS it's a corner vertex along that edge (we can compare to the two vertices)
                    //       in which case we could use a pre-calculated list of planes ..
                    //       OR when the intersection is outside of the edge ..

                    if (!IsOutsidePlanes(intersectingPlanes2, intersectingPlanes2Length, localVertex) &&
                        !IsOutsidePlanes(intersectingPlanes1, intersectingPlanes1Length, localVertex))
                    {
                        var worldVertex = math.mul(nodeToTreeSpaceMatrix1, localVertex).xyz;
                        foundVertices.Write(worldVertex);
                    }
                }

                foundVertices.EndForEachIndex();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void FindPlanePairIntersections(float4x4 nodeToTreeSpaceMatrix1,
                                                      List<int> intersectingPlaneIndices1, float4* intersectingPlanes1, int intersectingPlanes1Length, VertexSoup brushVertices1, LoopGroup holeLoops1, CSGTreeBrush brush1,
                                                                                           float4* intersectingPlanes2, int intersectingPlanes2Length, VertexSoup brushVertices2, LoopGroup holeLoops2, CSGTreeBrush brush2, 
                                                      List<PlanePair> usedPlanePairs2)
        {


            // find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
            for (int i = 0; i < usedPlanePairs2.Count; i++)
            {
                var planePair2  = usedPlanePairs2[i];

                // should never happen
                if (planePair2.P0 == planePair2.P1)
                    continue;

                var plane0      = planePair2.Plane0;
                var plane1      = planePair2.Plane1;

                //*
                var foundVertices = new NativeStream(intersectingPlanes1Length, Allocator.TempJob);
                try
                { 
                    var job = new FindIntersectionsJob
                    {
                        nodeToTreeSpaceMatrix1      = nodeToTreeSpaceMatrix1,
                        plane0                      = plane0,
                        plane1                      = plane1,
                        intersectingPlanes1         = intersectingPlanes1,
                        intersectingPlanes2         = intersectingPlanes2,
                        intersectingPlanes1Length   = intersectingPlanes1Length,
                        intersectingPlanes2Length   = intersectingPlanes2Length,

                        foundVertices               = foundVertices.AsWriter()
                    };
                    job.Run(intersectingPlanes1Length);
                    {
                        var vertexReader = foundVertices.AsReader();
                        int maxIndex = vertexReader.ForEachCount;

                        int index = 0;
                        vertexReader.BeginForEachIndex(index++);
                        while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                            vertexReader.BeginForEachIndex(index++);
                        while (vertexReader.RemainingItemCount > 0)
                        {
                            var worldVertex = vertexReader.Read<float3>();

                            // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex
                            var vertexIndex1 = brushVertices1.Add(worldVertex);
                            holeLoops1.FindOrAddLoop(intersectingPlaneIndices1[index - 1], brush2).AddIndex(brushVertices1, vertexIndex1);

                            var vertexIndex2 = brushVertices2.Add(worldVertex);
                            holeLoops2.FindOrAddLoop(planePair2.P0, brush1).AddIndex(brushVertices2, vertexIndex2);
                            holeLoops2.FindOrAddLoop(planePair2.P1, brush1).AddIndex(brushVertices2, vertexIndex2);

                            while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                                vertexReader.BeginForEachIndex(index++);
                        }
                    }
                }
                finally
                { 
                    foundVertices.Dispose();
                }
                /*/
                for (int a = 0; a < intersectingPlanes1Length; a++)
                {
                    var plane2 = intersectingPlanes1[a];

                    if ((math.abs(plane0.w - plane2.w) >= kPlaneDistanceEpsilon || math.dot(plane0.xyz, plane2.xyz) <= kNormalEpsilon) &&
                        (math.abs(plane1.w - plane2.w) >= kPlaneDistanceEpsilon || math.dot(plane1.xyz, plane2.xyz) <= kNormalEpsilon))
                    {
                        var localVertex = new float4(PlaneExtensions.Intersection(plane2, plane0, plane1), 1);

                        // FIXME: sometimes we have two planes of a brush2 intersecting one plane on brush1, 
                        //		  and even though it's outside of brush1, it's still *just* within kDistanceEpsilon
                        //		  and can cause issues .. we need to find a better way of doing this

                        // TODO: since we're using a pair in the outer loop, we could also determine which 
                        //       2 planes it intersects at both ends and just check those two planes ..

                        if (!IsOutsidePlanes(intersectingPlanes2, intersectingPlanes2Length, localVertex) &&
                            !IsOutsidePlanes(intersectingPlanes1, intersectingPlanes1Length, localVertex))
                        {
                            var worldVertex = math.mul(nodeToTreeSpaceMatrix1, localVertex).xyz;

                            // NOTE: for brush2, the intersection will always be only on two planes
                            //       UNLESS it's a corner vertex along that edge (we can compare to the two vertices)
                            //       in which case we could use a pre-calculated list of planes ..
                            //       OR when the intersection is outside of the edge ..

                            // TODO: should be having a Loop for each plane that intersects this vertex, and add that vertex
                            var vertexIndex1 = brushVertices1.Add(worldVertex);
                            holeLoops1.FindOrAddLoop(intersectingPlaneIndices1[a], brush2).AddIndex(brushVertices1, vertexIndex1);

                            var vertexIndex2 = brushVertices2.Add(worldVertex);
                            holeLoops2.FindOrAddLoop(planePair2.P0, brush1).AddIndex(brushVertices2, vertexIndex2);
                            holeLoops2.FindOrAddLoop(planePair2.P1, brush1).AddIndex(brushVertices2, vertexIndex2);
                        }
                    }
                }
                //*/
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void FindSharedVertices(CSGTreeBrush brush2,
                                                                                   float4* intersectingPlanes2, int intersectingPlanes2Length,
                                              List<int> intersectingPlaneIndices1, float4* intersectingPlanes1, int intersectingPlanes1Length,
                                              LoopGroup holeLoops1, 
                                              HashSet<int> usedVertices1, VertexSoup brushVertices1,
                                              BrushMesh inputMesh1,

                                              float4x4 nodeToTreeSpaceMatrix,
                                              float4x4 vertexToLocal1)
        {
            // TODO: when all vertices of a polygon are inside the other brush, don't bother intersecting it.
            //       same when two planes overlap each other ...

            // Find all vertices of brush1 that are inside brush2, and put their intersections into the appropriate loops
            foreach (var vertexIndex1 in usedVertices1)
            {
                var brushVertex1 = new float4(inputMesh1.vertices[vertexIndex1], 1);
                var localVertex1 = math.mul(vertexToLocal1, brushVertex1);
                if (CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes2, intersectingPlanes2Length, localVertex1))
                    continue;

                var worldVertex = math.mul(nodeToTreeSpaceMatrix, brushVertex1).xyz;
                var worldVertexIndex = -1;
                // TODO: optimize this, we already know these vertices are ON the planes of this brush, just not which: this can be precalculated
                for (int i = 0; i < intersectingPlanes1Length; i++)
                {
                    var planeIndex = intersectingPlaneIndices1[i];
                    var distance = math.dot(intersectingPlanes1[i], localVertex1);

                    // skip any plane this vertex is NOT on
                    if (distance >= -kDistanceEpsilon && distance <= kDistanceEpsilon) // is false on NaN/Infinity
                    {
                        if (worldVertexIndex == -1)
                            worldVertexIndex = brushVertices1.Add(worldVertex);

                        var planeLoop = holeLoops1.FindOrAddLoop(planeIndex, brush2);
                        planeLoop.AddIndex(brushVertices1, (ushort)worldVertexIndex);
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void AllCreateLoopsFromIntersections(SurfaceLoops   outputLoops, 
                                                           LoopGroup      holeLoops1,     int[] alignedPlaneLookup1,
                                                           VertexSoup brushVertices1, 
                                                           BrushMesh      inputMesh1,

                                                           float4x4 treeToNodeSpaceMatrix)
        {
            outputLoops.EnsureSize(inputMesh1.surfaces.Length);
            var inverseNodeToTreeSpaceMatrix1 = math.transpose(treeToNodeSpaceMatrix);

            var loops = holeLoops1.loops;
            for (int l = loops.Count - 1; l >= 0; l--)
            {
                var hole = loops[l];
                if (!hole.Valid)
                    continue;

                var basePlaneIndex = hole.info.basePlaneIndex;

                var alignedPlaneIndex = alignedPlaneLookup1[basePlaneIndex];
                hole.interiorCategory = (CategoryGroupIndex)CategoryIndex.Inside;
                if (alignedPlaneIndex != 0)
                    hole.interiorCategory = (CategoryGroupIndex)(alignedPlaneIndex < 0 ? CategoryIndex.ReverseAligned : CategoryIndex.Aligned);

                var localPlaneVector = inputMesh1.surfaces[basePlaneIndex].localPlane;
                var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix1, localPlaneVector);
                
                MathExtensions.CalculateTangents(worldPlaneVector.xyz, out Vector3 right, out Vector3 forward);

                hole.info.worldPlane    = new Plane(worldPlaneVector.xyz, worldPlaneVector.w);
                hole.info.right         = right;
                hole.info.forward       = forward;
                hole.info.layers        = inputMesh1.polygons[basePlaneIndex].surface.brushMaterial.LayerDefinition;

                // TODO: make sorting of vertices work in local space
                // TODO: or rewrite, use plane information for sorting
                SortIndices(hole.indices, brushVertices1.vertices, hole.info.worldPlane.normal);
                
                //if (CSGManagerPerformCSG.IsDegenerate(brushVertices1, hole.indices))
                //    continue;

                outputLoops.surfaces[basePlaneIndex].Add(hole);
            }
        }
        #endregion
        
        static readonly List<int>       s_IntersectingPlaneIndices1 = new List<int>();
        static readonly List<int>       s_IntersectingPlaneIndices2 = new List<int>();
        
        static float4[]                 s_IntersectingPlanes1       = new float4[0];
        static float4[]                 s_IntersectingPlanes2       = new float4[0];

        static readonly LoopGroup       s_HoleLoops1        = new LoopGroup();
        static readonly LoopGroup       s_HoleLoops2        = new LoopGroup();
        static readonly List<PlanePair> s_UsedPlanePairs2   = new List<PlanePair>();
        static readonly List<PlanePair> s_UsedPlanePairs1   = new List<PlanePair>();
        static readonly HashSet<int>    s_UsedVertices1     = new HashSet<int>();
        static readonly HashSet<int>    s_UsedVertices2     = new HashSet<int>();

        // TODO: create loops for both brushes TOGETHER, taking advantage of overlapping edges/vertices (necessary to avoid gaps!)
        //       right now loops are generated separately for each brush
        internal unsafe static void GetIntersectionLoops(CSGTreeBrush brush1, CSGTreeBrush brush2, ref SurfaceLoops loops12, ref SurfaceLoops loops21)
        {
            loops12.Clear();
            loops21.Clear();
            if (!brush1.Valid ||
                !brush2.Valid)
            {
                Debug.LogError($"!leaf1.Valid {brush1.brushNodeID} || !leaf2.Valid {brush2.brushNodeID}");
                CSGManager.AssertNodeIDValid(brush1.brushNodeID);
                CSGManager.AssertNodeIDValid(brush2.brushNodeID);
                return;
            }

            Debug.Assert(brush1.NodeID != brush2.NodeID);

            var reverseOrder    = brush1.NodeID > brush2.NodeID; // ensures we do calculations exactly the same for each brush pair
            var meshID1         = brush1.BrushMesh.BrushMeshID;
            var meshID2         = brush2.BrushMesh.BrushMeshID;

            if (!BrushMeshManager.IsBrushMeshIDValid(meshID1) ||
                !BrushMeshManager.IsBrushMeshIDValid(meshID2))
            {
                Debug.Log("!BrushMeshManager.IsBrushMeshIDValid(meshID1) || !BrushMeshManager.IsBrushMeshIDValid(meshID2)");
                return;
            }

            var mesh1 = BrushMeshManager.GetBrushMesh(meshID1);
            var mesh2 = BrushMeshManager.GetBrushMesh(meshID2);

            if (mesh1 == null ||
                mesh2 == null)
            {
                Debug.Log("mesh1 == null || mesh2 == null");
                return;
            }

            if (mesh1.IsEmpty() ||
                mesh2.IsEmpty())
            {
                return;
            }

            var nodeToTreeSpaceMatrix1  = (float4x4)brush1.NodeToTreeSpaceMatrix;
            var treeToNodeSpaceMatrix2  = (float4x4)brush2.TreeToNodeSpaceMatrix;
            var node1ToNode2            = math.mul(treeToNodeSpaceMatrix2, nodeToTreeSpaceMatrix1);
            var inversedNode2ToNode1    = math.transpose(node1ToNode2); //math.inverse(node1ToNode2);
            GetIntersectingPlanes(s_IntersectingPlaneIndices2, mesh2.surfaces, mesh1.localBounds, mesh1.vertices, inversedNode2ToNode1);

            var treeToNodeSpaceMatrix1  = (float4x4)brush1.TreeToNodeSpaceMatrix;
            var nodeToTreeSpaceMatrix2  = (float4x4)brush2.NodeToTreeSpaceMatrix;
            var node2ToNode1            = math.mul(treeToNodeSpaceMatrix1, nodeToTreeSpaceMatrix2);
            var inversedNode1ToNode2    = math.transpose(node2ToNode1); //math.inverse(node2ToNode1);
            GetIntersectingPlanes(s_IntersectingPlaneIndices1, mesh1.surfaces, mesh2.localBounds, mesh2.vertices, inversedNode1ToNode2);
            if (s_IntersectingPlaneIndices1.Count == 0 ||
                s_IntersectingPlaneIndices2.Count == 0)
            {
                Debug.Assert(s_IntersectingPlaneIndices2.Count == 0 && s_IntersectingPlaneIndices1.Count == 0, $"Expected intersection, but no intersection found between {brush1.NodeID} & {brush2.NodeID}");
                return;
            }


            // TODO: we don't actually use ALL of these planes .. Optimize this
            var localSpacePlanes1 = new float4[mesh1.surfaces.Length];
            for (int p = 0; p < localSpacePlanes1.Length; p++)
                localSpacePlanes1[p] = mesh1.surfaces[p].localPlane;

            // TODO: we don't actually use ALL of these planes .. Optimize this
            var localSpacePlanes2 = new float4[mesh2.surfaces.Length];
            for (int p = 0; p < localSpacePlanes2.Length; p++)
            {
                var transformedPlane = math.mul(inversedNode2ToNode1, mesh2.surfaces[p].localPlane);
                localSpacePlanes2[p] = transformedPlane / math.length(transformedPlane.xyz);
            }

            if (s_IntersectingPlanes1.Length < s_IntersectingPlaneIndices1.Count)
                s_IntersectingPlanes1 = new float4[s_IntersectingPlaneIndices1.Count];
            
            if (s_IntersectingPlanes2.Length < s_IntersectingPlaneIndices2.Count)
                s_IntersectingPlanes2 = new float4[s_IntersectingPlaneIndices2.Count];

            for (int i = 0; i < s_IntersectingPlaneIndices1.Count; i++)
                s_IntersectingPlanes1[i] = localSpacePlanes1[s_IntersectingPlaneIndices1[i]];
            for (int i = 0; i < s_IntersectingPlaneIndices2.Count; i++)
                s_IntersectingPlanes2[i] = localSpacePlanes2[s_IntersectingPlaneIndices2[i]];
            
            s_HoleLoops1.Clear();
            s_HoleLoops2.Clear();

            FindPlanePairs(s_UsedVertices2, s_UsedPlanePairs2, s_IntersectingPlaneIndices2, localSpacePlanes2, mesh2);
            FindPlanePairs(s_UsedVertices1, s_UsedPlanePairs1, s_IntersectingPlaneIndices1, localSpacePlanes1, mesh1);

            // decide which planes of brush1 align with brush2
            // TODO: optimize
            // TODO: should do this as a separate pass
            var alignedPlaneLookup1 = new int[mesh1.surfaces.Length];
            var alignedPlaneLookup2 = new int[mesh2.surfaces.Length];
            if (reverseOrder)
            {
                for (int i2 = 0; i2 < s_IntersectingPlaneIndices2.Count; i2++)
                {
                    var p2          = s_IntersectingPlaneIndices2[i2];
                    var localPlane2 = localSpacePlanes2[p2];
                    for (int i1 = 0; i1 < s_IntersectingPlaneIndices1.Count; i1++)
                    {
                        var p1          = s_IntersectingPlaneIndices1[i1];
                        var localPlane1 = localSpacePlanes1[p1];
                        if (math.abs(localPlane1.w - localPlane2.w) >= kPlaneDistanceEpsilon ||
                            math.dot(localPlane1.xyz, localPlane2.xyz) <= kNormalEpsilon)
                        {
                            localPlane1 = -localPlane1;
                            if (math.abs(localPlane1.w - localPlane2.w) >= kPlaneDistanceEpsilon ||
                                math.dot(localPlane1.xyz, localPlane2.xyz) <= kNormalEpsilon)
                                continue;

                            alignedPlaneLookup1[p1] = -(p2 + 1);
                            alignedPlaneLookup2[p2] = -(p1 + 1);
                        } else
                        {
                            alignedPlaneLookup1[p1] = p2 + 1;
                            alignedPlaneLookup2[p2] = p1 + 1;
                        }
                    }
                }
            } else
            {
                for (int i1 = 0; i1 < s_IntersectingPlaneIndices1.Count; i1++)
                {
                    var p1          = s_IntersectingPlaneIndices1[i1];
                    var localPlane1 = localSpacePlanes1[p1];
                    for (int i2 = 0; i2 < s_IntersectingPlaneIndices2.Count; i2++)
                    {
                        var p2          = s_IntersectingPlaneIndices2[i2];
                        var localPlane2 = localSpacePlanes2[p2];
                        if (math.abs(localPlane1.w - localPlane2.w) >= kPlaneDistanceEpsilon ||
                            math.dot(localPlane1.xyz, localPlane2.xyz) < kNormalEpsilon)
                        {
                            localPlane2 = -localPlane2;
                            if (math.abs(localPlane1.w - localPlane2.w) >= kPlaneDistanceEpsilon ||
                                math.dot(localPlane1.xyz, localPlane2.xyz) < kNormalEpsilon)
                                continue;

                            alignedPlaneLookup1[p1] = -(p2 + 1);
                            alignedPlaneLookup2[p2] = -(p1 + 1);
                        } else
                        {
                            alignedPlaneLookup1[p1] = p2 + 1;
                            alignedPlaneLookup2[p2] = p1 + 1;
                        }
                    }
                }
            }

            var brushVertices1 = CSGManager.GetBrushInfo(brush1.brushNodeID).brushOutputLoops.vertexSoup;
            var brushVertices2 = CSGManager.GetBrushInfo(brush2.brushNodeID).brushOutputLoops.vertexSoup;

            fixed (float4* intersectingPlanes1Ptr = &s_IntersectingPlanes1[0])
            {
                fixed (float4* intersectingPlanes2Ptr = &s_IntersectingPlanes2[0])
                {
                    FindSharedVertices(brush2,
                                                                      intersectingPlanes2Ptr, s_IntersectingPlaneIndices2.Count,                         
                                       s_IntersectingPlaneIndices1,   intersectingPlanes1Ptr, s_IntersectingPlaneIndices1.Count, 
                                       s_HoleLoops1, s_UsedVertices1, brushVertices1, mesh1, nodeToTreeSpaceMatrix1, float4x4.identity);

                    FindSharedVertices(brush1,
                                                                      intersectingPlanes1Ptr, s_IntersectingPlaneIndices1.Count,
                                       s_IntersectingPlaneIndices2,   intersectingPlanes2Ptr, s_IntersectingPlaneIndices2.Count, 
                                       s_HoleLoops2, s_UsedVertices2, brushVertices2, mesh2, nodeToTreeSpaceMatrix2, node2ToNode1);

                    FindPlanePairIntersections(nodeToTreeSpaceMatrix1,
                        s_IntersectingPlaneIndices1, intersectingPlanes1Ptr, s_IntersectingPlaneIndices1.Count, brushVertices1, s_HoleLoops1, brush1,
                                                     intersectingPlanes2Ptr, s_IntersectingPlaneIndices2.Count, brushVertices2, s_HoleLoops2, brush2,
                        s_UsedPlanePairs2);

                    FindPlanePairIntersections(nodeToTreeSpaceMatrix1,
                        s_IntersectingPlaneIndices2, intersectingPlanes2Ptr, s_IntersectingPlaneIndices2.Count, brushVertices2, s_HoleLoops2, brush2,
                                                     intersectingPlanes1Ptr, s_IntersectingPlaneIndices1.Count, brushVertices1, s_HoleLoops1, brush1,
                        s_UsedPlanePairs1);

                    AllCreateLoopsFromIntersections(loops12, s_HoleLoops1, alignedPlaneLookup1, brushVertices1, mesh1, treeToNodeSpaceMatrix1);             
                    AllCreateLoopsFromIntersections(loops21, s_HoleLoops2, alignedPlaneLookup2, brushVertices2, mesh2, treeToNodeSpaceMatrix2);
                }
            }
        }
        #endregion


        #region FindLoopOverlapIntersections

        struct IntersectionLoop
        {
            public NativeArray<float4>  selfPlanes;
            public List<ushort>         indices;
            public List<Edge>           edges;
            public Loop                 loop;
        }

        // TODO: do something more intelligent to avoid allocations
        static readonly List<IntersectionLoop>                  s_AllIntersectionLoops  = new List<IntersectionLoop>();
        static readonly Dictionary<int, NativeArray<float4>>    s_BrushPlanes           = new Dictionary<int, NativeArray<float4>>();
        static List<IntersectionLoop>[]                         s_IntersectionSurfaces  = new List<IntersectionLoop>[0];
        static FindLoopIntersectionVerticesJob                  s_IntersectionJob       = new FindLoopIntersectionVerticesJob();

        static unsafe void FindLoopOverlapIntersections(BrushLoops outputLoops)
        {
            if (outputLoops.intersectionSurfaceLoops.Count == 0)
                return;

            var mesh1 = BrushMeshManager.GetBrushMesh(outputLoops.brush.BrushMesh.BrushMeshID);
            Debug.Assert(mesh1 != null);

            if (mesh1.surfaces.Length == 0)
                return;

            using (var worldSpacePlanesS = new NativeArray<float4>(mesh1.surfaces.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory))
            {
                var treeToNodeSpaceTransposed = math.transpose(outputLoops.brush.TreeToNodeSpaceMatrix);
                fixed (BrushMesh.Surface* mesh1Surfaces = &mesh1.surfaces[0])
                {
                    float4* mesh1Planes = (float4*)mesh1Surfaces;
                    var worldSpacePlanesPtr = (float4*)worldSpacePlanesS.GetUnsafePtr();
                    CSGManagerPerformCSG.TransformByTransposedInversedMatrix(worldSpacePlanesPtr, mesh1Planes, mesh1.surfaces.Length, treeToNodeSpaceTransposed);
                }

                s_BrushPlanes.Clear();
                s_AllIntersectionLoops.Clear();
                if (s_IntersectionSurfaces.Length < mesh1.surfaces.Length)
                {
                    s_IntersectionSurfaces = new List<IntersectionLoop>[mesh1.surfaces.Length];
                    for (int i = 0; i < mesh1.surfaces.Length; i++)
                        s_IntersectionSurfaces[i] = new List<IntersectionLoop>(16);
                } else
                {
                    for (int i = 0; i < mesh1.surfaces.Length; i++)
                        s_IntersectionSurfaces[i].Clear();
                }

                foreach (var pair in outputLoops.intersectionSurfaceLoops)
                {
                    var intersectingBrush = new CSGTreeBrush() { brushNodeID = pair.Key };
                    var mesh2 = BrushMeshManager.GetBrushMesh(intersectingBrush.BrushMesh.BrushMeshID);
                    Debug.Assert(mesh2 != null);

                    if (mesh2.surfaces.Length == 0 || pair.Value == null)
                        continue;

                    var worldSpacePlanes = new NativeArray<float4>(mesh2.surfaces.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                    var worldSpacePlanesPtr = (float4*)worldSpacePlanes.GetUnsafePtr();
                    treeToNodeSpaceTransposed = math.transpose(intersectingBrush.TreeToNodeSpaceMatrix);
                    fixed (BrushMesh.Surface* mesh2Surfaces = &mesh2.surfaces[0])
                    {
                        float4* mesh2Planes = (float4*)mesh2Surfaces;
                        CSGManagerPerformCSG.TransformByTransposedInversedMatrix(worldSpacePlanesPtr, mesh2Planes, mesh2.surfaces.Length, treeToNodeSpaceTransposed);
                    }
                    s_BrushPlanes.Add(intersectingBrush.brushNodeID, worldSpacePlanes);

                    var surfaces = pair.Value.surfaces;
                    if (surfaces == null)
                        continue;
                    for (int s = 0; s < surfaces.Length; s++)
                    {
                        var intersectionSurface = s_IntersectionSurfaces[s];
                        var loops = surfaces[s];
                        if (loops == null || loops.Count == 0)
                            continue;

                        // At this point it should not be possible to have more loop from the same intersecting brush since 
                        // we only support convex brushes. 
                        // Later on, however, when we start intersecting loops with each other, we can end up with multiple fragments.
                        Debug.Assert(loops.Count == 1);

                        var loop = loops[0];
                        var intersectionLoop = new IntersectionLoop()
                        {
                            selfPlanes  = worldSpacePlanes,
                            indices     = loop.indices,
                            edges       = loop.edges,
                            loop        = loop
                        };

                        // We add the intersection loop of this particular brush with our own brush
                        intersectionSurface.Add(intersectionLoop);
                        s_AllIntersectionLoops.Add(intersectionLoop);
                    }
                }



                s_IntersectionJob.verticesSrc = new NativeArray<float3>(FindLoopIntersectionVerticesJob.kMaxVertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                s_IntersectionJob.verticesDst = new NativeArray<float3>(FindLoopIntersectionVerticesJob.kMaxVertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                try
                {
                    var srcVertices = outputLoops.vertexSoup.vertices;
                    UnityEngine.Profiling.Profiler.BeginSample("Loop2");
                    try
                    {
                        // TODO: only use planes that intersect with bounding box?
                        //          -> reuse this info from GetIntersectionLoops
                        //          -> should this be part of GetIntersectionLoops instead?
                        for (int s = 0; s < mesh1.surfaces.Length; s++)
                        {
                            var intersectionSurface = s_IntersectionSurfaces[s];
                            for (int l0 = 0; l0 < intersectionSurface.Count; l0++)
                            {
                                var planes0 = intersectionSurface[l0].selfPlanes;
                                var indices = intersectionSurface[l0].indices;

                                s_IntersectionJob.vertexCount = indices.Count;
                                for (int v = 0; v < indices.Count; v++)
                                    s_IntersectionJob.verticesDst[v] = srcVertices[indices[v]];

                                //var first = true;
                                //JobHandle lastHandle = new JobHandle();
                                //for (int l1 = l0 + 1; l1 < intersectionSurface.Count; l1++)
                                for (int l1 = 0; l1 < intersectionSurface.Count; l1++)
                                {
                                    if (l0 == l1)
                                        continue;

                                    var t = s_IntersectionJob.verticesSrc;
                                    s_IntersectionJob.verticesSrc = s_IntersectionJob.verticesDst;
                                    s_IntersectionJob.verticesDst = t;

                                    var planes1 = intersectionSurface[l1].selfPlanes;

                                    // TODO: merge these so that intersections will be identical on both loops (without using math, use logic)
                                    // TODO: make sure that intersections between loops will be identical on OTHER brushes (without using math, use logic)
                                    {
                                        s_IntersectionJob.otherPlanesNative = planes1;
                                        s_IntersectionJob.selfPlanesNative  = planes0;

                                        {
                                            s_IntersectionJob.Execute();
                                            //s_IntersectionJob.Schedule().Complete();
                                            // TODO: eventually actually use jobs
                                            //if (first)
                                            //    lastHandle = s_IntersectionJob.Schedule();
                                            //else
                                            //    lastHandle = s_IntersectionJob.Schedule(lastHandle);
                                            //first = false;
                                        }
                                    }
                                }
                                //lastHandle.Complete();

                                if (s_IntersectionJob.vertexCount > indices.Count)
                                {
                                    indices.Clear();
                                    if (indices.Capacity < s_IntersectionJob.vertexCount)
                                        indices.Capacity = s_IntersectionJob.vertexCount;
                                    for (int n = 0; n < s_IntersectionJob.vertexCount; n++)
                                    {
                                        var worldVertex = s_IntersectionJob.verticesDst[n];
                                        var vertexIndex = outputLoops.vertexSoup.Add(worldVertex);

                                        if (indices.Contains(vertexIndex))
                                            continue;

                                        indices.Add(vertexIndex);
                                    }
                                }
                            }
                        }
                    }
                    finally { UnityEngine.Profiling.Profiler.EndSample(); }


                    UnityEngine.Profiling.Profiler.BeginSample("Loop3");
                    try
                    {
                        // TODO: should only intersect with all brushes that each particular basepolygon intersects with
                        //       but also need adjency information between basePolygons to ensure that intersections exist on 
                        //       both sides of each edge on a brush. 
                        for (int b = 0; b < outputLoops.basePolygons.Count; b++)
                        {
                            var basePolygon = outputLoops.basePolygons[b];
                            var indices = basePolygon.indices;
                            {
                                s_IntersectionJob.vertexCount = indices.Count;
                                for (int v = 0; v < indices.Count; v++)
                                    s_IntersectionJob.verticesDst[v] = srcVertices[indices[v]];

                                //JobHandle lastHandle = new JobHandle();
                                //bool first = true;
                                foreach (var pair in s_BrushPlanes)
                                {
                                    var t = s_IntersectionJob.verticesSrc;
                                    s_IntersectionJob.verticesSrc = s_IntersectionJob.verticesDst;
                                    s_IntersectionJob.verticesDst = t;


                                    s_IntersectionJob.otherPlanesNative = pair.Value;
                                    s_IntersectionJob.selfPlanesNative = worldSpacePlanesS;

                                    s_IntersectionJob.Execute();
                                    //s_IntersectionJob.Schedule().Complete();
                                    // TODO: eventually actually use jobs
                                    //if (first)
                                    //    lastHandle = s_IntersectionJob.Schedule();
                                    //else
                                    //    lastHandle = s_IntersectionJob.Schedule(lastHandle);
                                    //first = false;
                                }
                                //lastHandle.Complete();

                                if (s_IntersectionJob.vertexCount > indices.Count)
                                {
                                    indices.Clear();
                                    if (indices.Capacity < s_IntersectionJob.vertexCount)
                                        indices.Capacity = s_IntersectionJob.vertexCount;
                                    for (int n = 0; n < s_IntersectionJob.vertexCount; n++)
                                    {
                                        var worldVertex = s_IntersectionJob.verticesDst[n];
                                        var vertexIndex = outputLoops.vertexSoup.Add(worldVertex);
                                    
                                        if (indices.Contains(vertexIndex))
                                            continue;

                                        indices.Add(vertexIndex);
                                    }
                                }
                            }
                        }
                    }
                    finally { UnityEngine.Profiling.Profiler.EndSample(); }
                }
                finally
                {
                    s_IntersectionJob.verticesSrc.Dispose();
                    s_IntersectionJob.verticesDst.Dispose();
                }

                var removeIdenticalIndicesJob = new RemoveIdenticalIndicesJob
                {
                    vertices = outputLoops.vertexSoup.vertices
                };

                // TODO: handle this more carefully to avoid gaps
                for (int i = s_AllIntersectionLoops.Count - 1; i >= 0; i--)
                {
                    var indices = s_AllIntersectionLoops[i].indices;

                    removeIdenticalIndicesJob.indices = indices;
                    // TODO: eventually actually use jobs
                    removeIdenticalIndicesJob.Execute();

                    if (indices.Count < 3)
                        s_AllIntersectionLoops.RemoveAt(i);
                }
 
                for (int i = outputLoops.basePolygons.Count - 1; i >= 0; i--)
                {
                    var indices = outputLoops.basePolygons[i].indices;

                    removeIdenticalIndicesJob.indices = indices;
                    // TODO: eventually actually use jobs
                    removeIdenticalIndicesJob.Execute();
                }

                // TODO: eventually merge indices across multiple loops when vertices are identical

                foreach (var brushPlane in s_BrushPlanes)
                    brushPlane.Value.Dispose();
                s_BrushPlanes.Clear();
            }

            outputLoops.vertexSoup.vertexArray = new NativeArray<float3>(outputLoops.vertexSoup.vertices.Count, Allocator.Persistent, options: NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < outputLoops.vertexSoup.vertices.Count; i++)
                outputLoops.vertexSoup.vertexArray[i] = outputLoops.vertexSoup.vertices[i];

            for (int i = 0; i < outputLoops.basePolygons.Count; i++)
            {
                var basePolygon = outputLoops.basePolygons[i];
                basePolygon.edges.Clear();
                basePolygon.AddEdges(basePolygon.indices);
            }

            foreach (var pair in outputLoops.intersectionSurfaceLoops)
            {
                var surfaceLoops = pair.Value.surfaces;
                if (surfaceLoops == null)
                    continue;
                for (int i = 0; i < surfaceLoops.Length; i++)
                {
                    var loops = surfaceLoops[i];
                    for (int s = 0; s < loops.Count; s++)
                    {
                        var intersectionLoop = loops[s];
                        intersectionLoop.edges.Clear();
                        intersectionLoop.AddEdges(intersectionLoop.indices);
                    }
                }
            }
        }


        static readonly HashSet<ulong> s_IntersectingBrushes = new HashSet<ulong>();

        internal static void FindAllIntersectionLoops(List<int> treeBrushes)
        {
            // Find all intersections between brushes
            s_IntersectingBrushes.Clear();
            for (int b0 = 0; b0 < treeBrushes.Count; b0++)
            {
                var brush0NodeID    = treeBrushes[b0];
                var output          = CSGManager.GetBrushInfo(brush0NodeID);
                var brushTouches    = output.brushTouch;
                if (brushTouches.Count == 0)
                    continue;

                foreach (var touch in brushTouches)
                {
                    // TODO: store touching brushes separately in an array
                    if (touch.Value != IntersectionType.Intersection)
                        continue;

                    var brush1NodeID = touch.Key.brushNodeID;

                    if (brush0NodeID < brush1NodeID) 
                        s_IntersectingBrushes.Add((ulong)brush0NodeID + ((ulong)brush1NodeID << 32));
                    else
                        s_IntersectingBrushes.Add((ulong)brush1NodeID + ((ulong)brush0NodeID << 32)); 
                }
            }

            // Create unique loops between brush intersections
            UnityEngine.Profiling.Profiler.BeginSample("GetIntersectionLoops");
            try 
            { 
                CSGTreeBrush treeBrush0;
                CSGTreeBrush treeBrush1;
                foreach (var pair in s_IntersectingBrushes)
                {
                    var brush0Index = (int)(pair & (~(uint)0));
                    var brush1Index = (int)(pair >> 32);

                    SurfaceLoops loops01;
                    SurfaceLoops loops10;
                    {
                        var output = CSGManager.GetBrushInfo(brush0Index);
                        var outputLoops = output.brushOutputLoops;
                        if (!outputLoops.intersectionSurfaceLoops.TryGetValue(brush1Index, out loops01))
                        {
                            loops01 = new SurfaceLoops();
                            outputLoops.intersectionSurfaceLoops[brush1Index] = loops01;
                        }
                    }

                    {
                        var output = CSGManager.GetBrushInfo(brush1Index);
                        var outputLoops = output.brushOutputLoops;
                        if (!outputLoops.intersectionSurfaceLoops.TryGetValue(brush0Index, out loops10))
                        {
                            loops10 = new SurfaceLoops();
                            outputLoops.intersectionSurfaceLoops[brush0Index] = loops10;
                        }
                    }

                    treeBrush0.brushNodeID = brush0Index;
                    treeBrush1.brushNodeID = brush1Index;

                    CSGManagerPerformCSG.GetIntersectionLoops(
                                treeBrush1,
                                treeBrush0,
                                ref loops10,
                                ref loops01);
                }
            } finally { UnityEngine.Profiling.Profiler.EndSample(); }

            UnityEngine.Profiling.Profiler.BeginSample("FindLoopOverlapIntersections");
            try
            {
                for (int b0 = 0; b0 < treeBrushes.Count; b0++)
                {
                    var brush0NodeID = treeBrushes[b0];
                    var output       = CSGManager.GetBrushInfo(brush0NodeID);
                    var outputLoops  = output.brushOutputLoops;
                    CSGManagerPerformCSG.FindLoopOverlapIntersections(outputLoops);

                    foreach (var pair in outputLoops.intersectionSurfaceLoops)
                    {
                        var surfaceLoops = pair.Value.surfaces;
                        if (surfaceLoops == null)
                        {
                            outputLoops.intersectionLoops[pair.Key] = null;
                        } else
                        {
                            var loops = new Loop[surfaceLoops.Length];
                            for (int i = 0; i < surfaceLoops.Length; i++)
                            {
                                var surfaceLoop = surfaceLoops[i];
                                if (surfaceLoop == null ||
                                    surfaceLoop.Count == 0 ||
                                    surfaceLoop[0] == null ||
                                    !surfaceLoop[0].Valid)
                                {
                                    loops[i] = null;
                                    continue;
                                }

                                Debug.Assert(surfaceLoop[0].Valid && (int)surfaceLoop[0].interiorCategory < CategoryRoutingRow.Length);

                                loops[i] = surfaceLoop[0];
                            }
                            outputLoops.intersectionLoops[pair.Key] = loops;
                        }
                    }
                }
            }
            finally { UnityEngine.Profiling.Profiler.EndSample(); }
        }
        
        #endregion
    }
#endif
}
