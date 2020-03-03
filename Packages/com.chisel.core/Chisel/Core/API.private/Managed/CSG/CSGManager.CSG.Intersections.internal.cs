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
        static void GetIntersectingPlanes(NativeList<int> intersectingPlanes, in BrushMesh.Surface[] otherSurfaces, in Bounds selfBounds, in float3[] vertices, in float4x4 treeToNodeSpaceInverseTransposed)
        {
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
                //if ((
                    //minDistance >= -kDistanceEpsilon        // TODO: sometimes this fails, find out why
                    //||                                      
                    //maxDistance < -kDistanceEpsilon       // TODO: sometimes this fails, find out why
                    //) 
                    //&& 
                    //onCount < 3) // If we have a polygon intersecting with a plane, we use it
                    //)
                    //continue;

                intersectingPlanes.Add(i);
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
        static float3 FindPolygonCentroid(List<ushort> indices, NativeList<float3> vertices)
        {
            var centroid = float3.zero;
            for (int i = 0; i < indices.Count; i++)
                centroid += vertices[indices[i]];
            return centroid / indices.Count;
        }

        static List<int2> s_SortStack = new List<int2>();
        // TODO: sort by using plane information instead of unreliable floating point math ..
        // TODO: make this work on non-convex polygons
        static void SortIndices(List<ushort> indices, NativeList<float3> vertices, float3 normal)
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

        public struct PlanePair
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
        public static unsafe bool IsOutsidePlanes(float4* planes, int length, float4 localVertex)
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
        public static unsafe bool IsOutsidePlanes(NativeArray<float4> planes, float4 localVertex)
        {
            const float kEpsilon = CSGManagerPerformCSG.kDistanceEpsilon;
            for (int n = 0; n < planes.Length; n++)
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
                if (distance < -CSGManagerPerformCSG.kDistanceEpsilon)
                    return true;
            }
            return false;
        }

        static HashSet<int> s_UsedVertices1 = new HashSet<int>();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FindPlanePairs(NativeList<PlanePair>    usedPlanePairs1,
                                   NativeList<int>          intersectingPlanes1,
                                   float4[]                 localSpacePlanes1,
                                   BrushMesh                mesh1,
                                   Allocator                allocator,
                                   out NativeArray<int>     usedVertices1)
        {
            // TODO: this can be partially stored in brushmesh 
            // TODO: optimize
            s_UsedVertices1.Clear();
            usedPlanePairs1.Clear();

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

                s_UsedVertices1.Add(vI0);
                s_UsedVertices1.Add(vI1);
                usedPlanePairs1.Add(new PlanePair()
                {
                    Plane0 = localSpacePlanes1[pI0],
                    Plane1 = localSpacePlanes1[pI1],
                    P0 = sI0,
                    P1 = sI1
                });
            }
            usedVertices1 = new NativeArray<int>(s_UsedVertices1.ToArray(), allocator);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FindPlanePairIntersections(NativeStream foundVertices,

                                               float4x4               nodeToTreeSpaceMatrix1, 
                                               NativeList<int>        intersectingPlaneIndices1,  
                                               NativeArray<float4>    intersectingPlanes1, in VertexSoup brushVertices1,
                                               NativeArray<float4>    intersectingPlanes2, in VertexSoup brushVertices2,
                                               NativeArray<PlanePair> usedPlanePairs,

                                               NativeList<PlaneVertexIndexPair> foundIndices1,
                                               NativeList<PlaneVertexIndexPair> foundIndices2)
        {
            { 
                var findIntersectionsJob = new FindIntersectionsJob
                {
                    nodeToTreeSpaceMatrix1      = nodeToTreeSpaceMatrix1,
                    intersectingPlanes1         = intersectingPlanes1,
                    intersectingPlanes2         = intersectingPlanes2,

                    usedPlanePairs              = usedPlanePairs,

                    foundVertices               = foundVertices.AsWriter()
                };
                findIntersectionsJob.Run(intersectingPlanes1.Length);

                var insertIntersectionVerticesJob = new InsertIntersectionVerticesJob
                {
                    vertexReader                = foundVertices.AsReader(),
                    foundIndices1               = foundIndices1,
                    foundIndices2               = foundIndices2,

                    brushVertices1              = brushVertices1,
                    brushVertices2              = brushVertices2,

                    intersectingPlaneIndices1   = intersectingPlaneIndices1
                };
                insertIntersectionVerticesJob.Run();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static JobHandle FindInsideVertices(JobHandle            handleDep,

                                            NativeArray<float4>  intersectingPlanes2,
                                            NativeArray<int>     usedVertices1,
                                            NativeArray<float3>  allVertices1,

                                            float4x4             nodeToTreeSpaceMatrix,
                                            float4x4             vertexToLocal1,

                                            NativeStream         foundVertices)
        {
            var findInsideVerticesJob = new FindInsideVerticesJob
            {
                intersectingPlanes2         = intersectingPlanes2,
                usedVertices1               = usedVertices1,
                allVertices1                = allVertices1,

                nodeToTreeSpaceMatrix       = nodeToTreeSpaceMatrix,
                vertexToLocal1              = vertexToLocal1,

                foundVertices               = foundVertices.AsWriter()
            };
            findInsideVerticesJob.Run(usedVertices1.Length);
            return handleDep;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static JobHandle InsertInsideVertices(JobHandle           handleDep,

                                              NativeStream        foundVertices,
                                              NativeArray<float4> intersectingPlanes,
                                              NativeList<int>     intersectingPlaneIndices,

                                              in VertexSoup                     brushVertices,
                                              NativeList<PlaneVertexIndexPair>  foundIndices)
        {
            var insertInsideVerticesJob = new InsertInsideVerticesJob
            {
                vertexReader                = foundVertices.AsReader(),
                intersectingPlanes          = intersectingPlanes,
                intersectingPlaneIndices    = intersectingPlaneIndices,

                brushVertices               = brushVertices,
                foundIndices                = foundIndices
            };
            insertInsideVerticesJob.Run();
            return handleDep;
        }
        #endregion

        public struct SurfaceInfo
        {
            public CategoryGroupIndex   category;
            public Plane                worldPlane;
            public float3               right;
            public float3               forward;
            public SurfaceLayers        layers;
        }

        sealed class SharedPlaneData : IDisposable
        {
            public CSGTreeBrush             treeBrush0;
            public CSGTreeBrush             treeBrush1;
            public IntersectionType         intersectionType;

            public float4x4                 nodeToTreeSpaceMatrix0;
            public float4x4                 treeToNodeSpaceMatrix0;
            public float4x4                 nodeToTreeSpaceMatrix1;
            public float4x4                 node1ToNode0;


            public NativeList<int>          intersectingPlaneIndices0;
            public NativeList<int>          intersectingPlaneIndices1;

            public NativeArray<float4>      intersectingPlanes0;
            public NativeArray<float4>      intersectingPlanes1;

            public NativeList<PlanePair>    usedPlanePairs0;
            public NativeList<PlanePair>    usedPlanePairs1;

            public NativeArray<int>         usedVertices0;
            public NativeArray<int>         usedVertices1;

            public NativeArray<SurfaceInfo>  surfaceCategory0;
            public NativeArray<SurfaceInfo>  surfaceCategory1;

            public NativeArray<float3>      allVertices0;
            public NativeArray<float3>      allVertices1;

            public VertexSoup       vertexSoup0;
            public VertexSoup       vertexSoup1;

            public SurfaceLoops     loops01;
            public SurfaceLoops     loops10;


            public void Dispose()
            {
                if (intersectingPlaneIndices0.IsCreated) intersectingPlaneIndices0.Dispose();
                if (intersectingPlaneIndices1.IsCreated) intersectingPlaneIndices1.Dispose();

                if (intersectingPlanes0.IsCreated) intersectingPlanes0.Dispose();
                if (intersectingPlanes1.IsCreated) intersectingPlanes1.Dispose();

                if (usedPlanePairs0.IsCreated) usedPlanePairs0.Dispose();
                if (usedPlanePairs1.IsCreated) usedPlanePairs1.Dispose();

                if (usedVertices0.IsCreated) usedVertices0.Dispose();
                if (usedVertices1.IsCreated) usedVertices1.Dispose();

                if (surfaceCategory0.IsCreated) surfaceCategory0.Dispose();
                if (surfaceCategory1.IsCreated) surfaceCategory1.Dispose();

                if (allVertices0.IsCreated) allVertices0.Dispose();
                if (allVertices1.IsCreated) allVertices1.Dispose();
            }
        }

        unsafe static SharedPlaneData CalculateSharedPlaneData(CSGTreeBrush brush0, BrushMesh mesh0, CSGTreeBrush brush1, BrushMesh mesh1, IntersectionType intersectionType, Allocator allocator)
        {
            var brushInfo0 = CSGManager.GetBrushInfo(brush0.brushNodeID);
            var brushInfo1 = CSGManager.GetBrushInfo(brush1.brushNodeID);

            var sharedPlaneData = new SharedPlaneData();
            sharedPlaneData.treeBrush0      = brush0;
            sharedPlaneData.treeBrush1      = brush1;
            sharedPlaneData.vertexSoup0     = brushInfo0.brushOutputLoops.vertexSoup;
            sharedPlaneData.vertexSoup1     = brushInfo1.brushOutputLoops.vertexSoup;
            sharedPlaneData.loops01         = new SurfaceLoops();
            sharedPlaneData.loops10         = new SurfaceLoops();

            sharedPlaneData.nodeToTreeSpaceMatrix0  = (float4x4)brush0.NodeToTreeSpaceMatrix;
            sharedPlaneData.treeToNodeSpaceMatrix0  = (float4x4)brush0.TreeToNodeSpaceMatrix;
            sharedPlaneData.nodeToTreeSpaceMatrix1  = (float4x4)brush1.NodeToTreeSpaceMatrix;
            sharedPlaneData.node1ToNode0            = math.mul(sharedPlaneData.treeToNodeSpaceMatrix0, sharedPlaneData.nodeToTreeSpaceMatrix1);

            var treeToNodeSpaceMatrix1          = (float4x4)brush1.TreeToNodeSpaceMatrix;
            var node0ToNode1                    = math.mul(treeToNodeSpaceMatrix1, sharedPlaneData.nodeToTreeSpaceMatrix0);
            var inversedNode1ToNode0            = math.transpose(node0ToNode1);

            var inversedNode0ToNode1            = math.transpose(sharedPlaneData.node1ToNode0);
            var inverseNodeToTreeSpaceMatrix0   = math.transpose(sharedPlaneData.treeToNodeSpaceMatrix0);
            var inverseNodeToTreeSpaceMatrix1   = math.transpose(treeToNodeSpaceMatrix1);

            sharedPlaneData.loops01.EnsureSize(mesh0.surfaces.Length);
            sharedPlaneData.loops10.EnsureSize(mesh1.surfaces.Length);
            brushInfo0.brushOutputLoops.intersectionSurfaceLoops[brush1.brushNodeID] = sharedPlaneData.loops01;
            brushInfo1.brushOutputLoops.intersectionSurfaceLoops[brush0.brushNodeID] = sharedPlaneData.loops10;



            // TODO: we don't actually use ALL of these planes .. Optimize this
            var localSpacePlanes0 = new float4[mesh0.surfaces.Length];
            for (int p = 0; p < localSpacePlanes0.Length; p++)
                localSpacePlanes0[p] = mesh0.surfaces[p].localPlane;

            // TODO: we don't actually use ALL of these planes .. Optimize this
            var localSpacePlanes1 = new float4[mesh1.surfaces.Length];
            for (int p = 0; p < localSpacePlanes1.Length; p++)
            {
                var transformedPlane = math.mul(inversedNode1ToNode0, mesh1.surfaces[p].localPlane);
                localSpacePlanes1[p] = transformedPlane / math.length(transformedPlane.xyz);
            }

            sharedPlaneData.intersectionType = intersectionType;

            sharedPlaneData.intersectingPlaneIndices0 = new NativeList<int>(mesh0.surfaces.Length, allocator);
            sharedPlaneData.intersectingPlaneIndices1 = new NativeList<int>(mesh1.surfaces.Length, allocator);

            sharedPlaneData.usedPlanePairs0 = new NativeList<PlanePair>(mesh0.halfEdges.Length, allocator);
            sharedPlaneData.usedPlanePairs1 = new NativeList<PlanePair>(mesh1.halfEdges.Length, allocator);

            sharedPlaneData.surfaceCategory0 = new NativeArray<SurfaceInfo>(mesh0.surfaces.Length, allocator, NativeArrayOptions.ClearMemory); // all set to Inside (0)
            sharedPlaneData.surfaceCategory1 = new NativeArray<SurfaceInfo>(mesh1.surfaces.Length, allocator, NativeArrayOptions.ClearMemory); // all set to Inside (0)

            sharedPlaneData.allVertices0    = new NativeArray<float3>(mesh0.vertices.ToArray(), allocator);
            sharedPlaneData.allVertices1    = new NativeArray<float3>(mesh1.vertices.ToArray(), allocator);

            for (int i = 0; i < sharedPlaneData.surfaceCategory0.Length; i++)
            {
                var localPlaneVector = mesh0.surfaces[i].localPlane;
                var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix0, localPlaneVector);

                MathExtensions.CalculateTangents(worldPlaneVector.xyz, out Vector3 right, out Vector3 forward);

                sharedPlaneData.surfaceCategory0[i] = new SurfaceInfo()
                {
                    category    = (CategoryGroupIndex)CategoryIndex.Inside,
                    worldPlane  = new Plane(worldPlaneVector.xyz, worldPlaneVector.w),
                    right       = right,
                    forward     = forward,
                    layers      = mesh0.polygons[i].surface.brushMaterial.LayerDefinition
                };
            }
            for (int i = 0; i < sharedPlaneData.surfaceCategory1.Length; i++)
            {
                var localPlaneVector = mesh1.surfaces[i].localPlane;
                var worldPlaneVector = math.mul(inverseNodeToTreeSpaceMatrix1, localPlaneVector);

                MathExtensions.CalculateTangents(worldPlaneVector.xyz, out Vector3 right, out Vector3 forward);

                sharedPlaneData.surfaceCategory1[i] = new SurfaceInfo()
                {
                    category    = (CategoryGroupIndex)CategoryIndex.Inside,
                    worldPlane  = new Plane(worldPlaneVector.xyz, worldPlaneVector.w),
                    right       = right,
                    forward     = forward,
                    layers      = mesh1.polygons[i].surface.brushMaterial.LayerDefinition
                };
            }

            if (sharedPlaneData.intersectionType == IntersectionType.Intersection)
            {
                GetIntersectingPlanes(sharedPlaneData.intersectingPlaneIndices1, mesh1.surfaces, mesh0.localBounds, mesh0.vertices, inversedNode1ToNode0);
                GetIntersectingPlanes(sharedPlaneData.intersectingPlaneIndices0, mesh0.surfaces, mesh1.localBounds, mesh1.vertices, inversedNode0ToNode1);
            }

            if (sharedPlaneData.intersectingPlaneIndices0.Length == 0 ||
                sharedPlaneData.intersectingPlaneIndices1.Length == 0)
            {
                if (sharedPlaneData.intersectionType == IntersectionType.Intersection)
                {
#if UNITY_EDITOR
                    Debug.Assert(sharedPlaneData.intersectingPlaneIndices1.Length == 0 && sharedPlaneData.intersectingPlaneIndices0.Length == 0, $"Expected intersection, but no intersection found between {brush0.NodeID} & {brush1.NodeID}");
                    Debug.LogError($"{brush0.NodeID}", UnityEditor.EditorUtility.InstanceIDToObject(brush0.UserID));
                    Debug.LogError($"{brush1.NodeID}", UnityEditor.EditorUtility.InstanceIDToObject(brush1.UserID));
#endif
                    sharedPlaneData.intersectionType = IntersectionType.NoIntersection;
                }

                // One brush could be inside the other
                sharedPlaneData.intersectingPlanes0 = new NativeArray<float4>(localSpacePlanes0, allocator);
                sharedPlaneData.intersectingPlanes1 = new NativeArray<float4>(localSpacePlanes1, allocator);

                for (int i = 0; i < mesh0.surfaces.Length; i++) sharedPlaneData.intersectingPlaneIndices0.Add(i);
                for (int i = 0; i < mesh1.surfaces.Length; i++) sharedPlaneData.intersectingPlaneIndices1.Add(i);

                sharedPlaneData.usedVertices0 = new NativeArray<int>(mesh0.vertices.Length, allocator, NativeArrayOptions.UninitializedMemory);
                sharedPlaneData.usedVertices1 = new NativeArray<int>(mesh1.vertices.Length, allocator, NativeArrayOptions.UninitializedMemory);

                for (int i = 0; i < mesh0.vertices.Length; i++) sharedPlaneData.usedVertices0[i] = i;
                for (int i = 0; i < mesh1.vertices.Length; i++) sharedPlaneData.usedVertices1[i] = i;


                return sharedPlaneData;
            }

            sharedPlaneData.intersectingPlanes0 = new NativeArray<float4>(sharedPlaneData.intersectingPlaneIndices0.Length, allocator, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < sharedPlaneData.intersectingPlaneIndices0.Length; i++)
                sharedPlaneData.intersectingPlanes0[i] = localSpacePlanes0[sharedPlaneData.intersectingPlaneIndices0[i]];

            sharedPlaneData.intersectingPlanes1 = new NativeArray<float4>(sharedPlaneData.intersectingPlaneIndices1.Length, allocator, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < sharedPlaneData.intersectingPlaneIndices1.Length; i++)
                sharedPlaneData.intersectingPlanes1[i] = localSpacePlanes1[sharedPlaneData.intersectingPlaneIndices1[i]];

            FindPlanePairs(sharedPlaneData.usedPlanePairs1, sharedPlaneData.intersectingPlaneIndices1, localSpacePlanes1, mesh1, allocator, out sharedPlaneData.usedVertices1);
            FindPlanePairs(sharedPlaneData.usedPlanePairs0, sharedPlaneData.intersectingPlaneIndices0, localSpacePlanes0, mesh0, allocator, out sharedPlaneData.usedVertices0);

            // decide which planes of brush1 align with brush2
            // TODO: optimize
            // TODO: should do this as a separate pass
            for (int i1 = 0; i1 < sharedPlaneData.intersectingPlaneIndices0.Length; i1++)
            {
                var p1          = sharedPlaneData.intersectingPlaneIndices0[i1];
                var localPlane1 = localSpacePlanes0[p1];
                for (int i2 = 0; i2 < sharedPlaneData.intersectingPlaneIndices1.Length; i2++)
                {
                    var p2          = sharedPlaneData.intersectingPlaneIndices1[i2];
                    var localPlane2 = localSpacePlanes1[p2];
                    if (math.abs(localPlane1.w - localPlane2.w) >= kPlaneDistanceEpsilon ||
                        math.dot(localPlane1.xyz, localPlane2.xyz) < kNormalEpsilon)
                    {
                        localPlane2 = -localPlane2;
                        if (math.abs(localPlane1.w - localPlane2.w) >= kPlaneDistanceEpsilon ||
                            math.dot(localPlane1.xyz, localPlane2.xyz) < kNormalEpsilon)
                            continue;

                        var surfaceInfo0 = sharedPlaneData.surfaceCategory0[p1];
                        surfaceInfo0.category = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
                        sharedPlaneData.surfaceCategory0[p1] = surfaceInfo0;
                        var surfaceInfo1 = sharedPlaneData.surfaceCategory1[p2];
                        surfaceInfo1.category = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
                        sharedPlaneData.surfaceCategory1[p2] = surfaceInfo1;
                    } else
                    {
                        var surfaceInfo0 = sharedPlaneData.surfaceCategory0[p1];
                        surfaceInfo0.category = (CategoryGroupIndex)CategoryIndex.Aligned;
                        sharedPlaneData.surfaceCategory0[p1] = surfaceInfo0;
                        var surfaceInfo1 = sharedPlaneData.surfaceCategory1[p2];
                        surfaceInfo1.category = (CategoryGroupIndex)CategoryIndex.Aligned;
                        sharedPlaneData.surfaceCategory1[p2] = surfaceInfo1;
                    }
                }
            }

            return sharedPlaneData;
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
        static FindLoopIntersectionVerticesJob2                 s_IntersectionJob2      = new FindLoopIntersectionVerticesJob2();

        static unsafe void FindLoopOverlapIntersections(BrushLoops outputLoops)
        {
            if (outputLoops.intersectionSurfaceLoops.Count == 0)
                return;

            var mesh = BrushMeshManager.GetBrushMesh(outputLoops.brush.BrushMesh.BrushMeshID);
            Debug.Assert(mesh != null);

            if (mesh.surfaces.Length == 0)
                return;

            using (var worldSpacePlanesS = new NativeArray<float4>(mesh.surfaces.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory))
            {
                var treeToNodeSpaceTransposed = math.transpose(outputLoops.brush.TreeToNodeSpaceMatrix);
                fixed (BrushMesh.Surface* meshSurfaces = &mesh.surfaces[0])
                {
                    float4* meshPlanes = (float4*)meshSurfaces;
                    var worldSpacePlanesPtr = (float4*)worldSpacePlanesS.GetUnsafePtr();
                    CSGManagerPerformCSG.TransformByTransposedInversedMatrix(worldSpacePlanesPtr, meshPlanes, mesh.surfaces.Length, treeToNodeSpaceTransposed);
                }

                s_BrushPlanes.Clear();
                s_AllIntersectionLoops.Clear();
                var s_IntersectionSurfaceCount = mesh.surfaces.Length;
                if (s_IntersectionSurfaces.Length < s_IntersectionSurfaceCount)
                {
                    s_IntersectionSurfaces = new List<IntersectionLoop>[s_IntersectionSurfaceCount];
                    for (int i = 0; i < s_IntersectionSurfaceCount; i++)
                        s_IntersectionSurfaces[i] = new List<IntersectionLoop>(16);
                } else
                {
                    for (int i = 0; i < s_IntersectionSurfaceCount; i++)
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



                s_IntersectionJob.verticesInput     = new NativeArray<float3>(FindLoopIntersectionVerticesJob.kMaxVertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                s_IntersectionJob.verticesOutput    = new NativeArray<float3>(FindLoopIntersectionVerticesJob.kMaxVertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                
                s_IntersectionJob2.verticesInput    = new NativeArray<float3>(FindLoopIntersectionVerticesJob2.kMaxVertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                s_IntersectionJob2.otherVertices    = new NativeArray<float3>(FindLoopIntersectionVerticesJob2.kMaxVertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                s_IntersectionJob2.verticesOutput   = new NativeArray<float3>(FindLoopIntersectionVerticesJob2.kMaxVertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                try
                {
                    var srcVertices = outputLoops.vertexSoup.vertices;
                    UnityEngine.Profiling.Profiler.BeginSample("Loop2");
                    try
                    {
                        // TODO: only use planes that intersect with bounding box?
                        //          -> reuse this info from GetIntersectionLoops
                        //          -> should this be part of GetIntersectionLoops instead?
                        for (int s = 0; s < s_IntersectionSurfaceCount; s++)
                        {
                            var intersectionSurface = s_IntersectionSurfaces[s];
                            for (int l0 = 0; l0 < intersectionSurface.Count; l0++)
                            {
                                var planes0 = intersectionSurface[l0].selfPlanes;
                                var indices = intersectionSurface[l0].indices;

                                s_IntersectionJob.vertexCount = indices.Count;
                                for (int v = 0; v < indices.Count; v++)
                                    s_IntersectionJob.verticesOutput[v] = srcVertices[indices[v]];

                                for (int l1 = 0; l1 < intersectionSurface.Count; l1++)
                                {
                                    if (l0 == l1)
                                        continue;

                                    var t = s_IntersectionJob.verticesInput;
                                    s_IntersectionJob.verticesInput = s_IntersectionJob.verticesOutput;
                                    s_IntersectionJob.verticesOutput = t;

                                    var planes1 = intersectionSurface[l1].selfPlanes;

                                    // TODO: merge these so that intersections will be identical on both loops (without using math, use logic)
                                    // TODO: make sure that intersections between loops will be identical on OTHER brushes (without using math, use logic)
                                    {
                                        s_IntersectionJob.otherPlanesNative = planes1;
                                        s_IntersectionJob.selfPlanesNative  = planes0;

                                        s_IntersectionJob.Execute();
                                    }
                                }

                                if (s_IntersectionJob.vertexCount > indices.Count)
                                {
                                    indices.Clear();
                                    if (indices.Capacity < s_IntersectionJob.vertexCount)
                                        indices.Capacity = s_IntersectionJob.vertexCount;
                                    for (int n = 0; n < s_IntersectionJob.vertexCount; n++)
                                    {
                                        var worldVertex = s_IntersectionJob.verticesOutput[n];
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
                                    s_IntersectionJob.verticesOutput[v] = srcVertices[indices[v]];

                                foreach (var pair in s_BrushPlanes)
                                {
                                    var t = s_IntersectionJob.verticesInput;
                                    s_IntersectionJob.verticesInput = s_IntersectionJob.verticesOutput;
                                    s_IntersectionJob.verticesOutput = t;

                                    s_IntersectionJob.otherPlanesNative = pair.Value;
                                    s_IntersectionJob.selfPlanesNative = worldSpacePlanesS;

                                    s_IntersectionJob.Execute();
                                }

                                if (s_IntersectionJob.vertexCount > indices.Count)
                                {
                                    indices.Clear();
                                    if (indices.Capacity < s_IntersectionJob.vertexCount)
                                        indices.Capacity = s_IntersectionJob.vertexCount;
                                    for (int n = 0; n < s_IntersectionJob.vertexCount; n++)
                                    {
                                        var worldVertex = s_IntersectionJob.verticesOutput[n];
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
                    UnityEngine.Profiling.Profiler.BeginSample("Find intersection-loop/base-loop intersections");
                    try
                    {
                        for (int s = 0; s < s_IntersectionSurfaceCount; s++)
                        {
                            var intersectionSurface = s_IntersectionSurfaces[s];
                            if (intersectionSurface.Count > 0)
                            {
                                var basePolygon = outputLoops.basePolygons[s];
                                for (int l0 = 0; l0 < intersectionSurface.Count; l0++)
                                {
                                    s_IntersectionJob2.FindIntersections(outputLoops.vertexSoup, intersectionSurface[l0].indices, basePolygon.indices, intersectionSurface[l0].selfPlanes);
                                    s_IntersectionJob2.GetOutput(outputLoops.vertexSoup, intersectionSurface[l0].indices);
                                }
                            }
                        }
                    }
                    finally { UnityEngine.Profiling.Profiler.EndSample(); }
                }
                finally
                {
                    s_IntersectionJob.verticesInput.Dispose();
                    s_IntersectionJob.verticesOutput.Dispose();

                    s_IntersectionJob2.verticesInput.Dispose();
                    s_IntersectionJob2.otherVertices.Dispose();
                    s_IntersectionJob2.verticesOutput.Dispose();
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

            //outputLoops.vertexSoup.vertexArray.Clear();
            //for (int i = 0; i < outputLoops.vertexSoup.vertexArray.Count; i++)
            //    outputLoops.vertexSoup.vertexArray.Add(outputLoops.vertexSoup.vertices[i]);

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


        static readonly HashSet<BrushBrushIntersection> s_IntersectingBrushes = new HashSet<BrushBrushIntersection>();
        

        internal static void FindAllIntersectionLoops(List<int> treeBrushes)
        {
            // Find all intersections between brushes
            s_IntersectingBrushes.Clear();
            for (int b0 = 0; b0 < treeBrushes.Count; b0++)
            {
                var intersections = CSGManager.GetBrushInfo(treeBrushes[b0]).brushBrushIntersections;
                if (intersections.Count == 0)
                    continue;

                foreach (var intersection in intersections)
                {
                    s_IntersectingBrushes.Add(intersection); // uses hashset to ensure this is unique
                }
            }

            // Create unique loops between brush intersections
            UnityEngine.Profiling.Profiler.BeginSample("CreateIntersectionLoops");
            try
            {
                foreach (var intersection in s_IntersectingBrushes)
                {
                    var reverseOrder = intersection.brushNodeID0 > intersection.brushNodeID1; // ensures we do calculations exactly the same for each brush pair
                    var type = intersection.type;

                    CSGTreeBrush brush0, brush1;
                    if (reverseOrder)
                    {
                        if      (type == IntersectionType.AInsideB) type = IntersectionType.BInsideA;
                        else if (type == IntersectionType.BInsideA) type = IntersectionType.AInsideB;

                        brush0.brushNodeID = intersection.brushNodeID1;
                        brush1.brushNodeID = intersection.brushNodeID0;
                    } else
                    {
                        brush0.brushNodeID = intersection.brushNodeID0;
                        brush1.brushNodeID = intersection.brushNodeID1;
                    }

                    if (!brush0.Valid ||
                        !brush1.Valid)
                    {
                        Debug.LogError($"!leaf1.Valid {brush0.brushNodeID} || !leaf2.Valid {brush1.brushNodeID}");
                        CSGManager.AssertNodeIDValid(brush0.brushNodeID);
                        CSGManager.AssertNodeIDValid(brush1.brushNodeID);
                        continue;
                    }

                    Debug.Assert(brush0.NodeID != brush1.NodeID);
                    var meshID0 = brush0.BrushMesh.BrushMeshID;
                    var meshID1 = brush1.BrushMesh.BrushMeshID;

                    if (!BrushMeshManager.IsBrushMeshIDValid(meshID0) ||
                        !BrushMeshManager.IsBrushMeshIDValid(meshID1))
                    {
                        Debug.Log("!BrushMeshManager.IsBrushMeshIDValid(meshID1) || !BrushMeshManager.IsBrushMeshIDValid(meshID2)");
                        continue;
                    }

                    var mesh0 = BrushMeshManager.GetBrushMesh(meshID0);
                    var mesh1 = BrushMeshManager.GetBrushMesh(meshID1);

                    if (mesh0 == null ||
                        mesh1 == null)
                    {
                        Debug.Log("mesh1 == null || mesh2 == null");
                        continue;
                    }

                    if (mesh0.IsEmpty() ||
                        mesh1.IsEmpty())
                    {
                        continue;
                    }

                    using (var sharedPlaneData = CalculateSharedPlaneData(brush0, mesh0, brush1, mesh1, type, Allocator.TempJob))
                    {
                        if (sharedPlaneData.intersectingPlanes0.Length == 0 && sharedPlaneData.intersectingPlanes1.Length == 0)
                            continue;

                        using (var foundIndices0 = new NativeList<PlaneVertexIndexPair>(sharedPlaneData.intersectingPlanes0.Length, Allocator.TempJob))
                        using (var foundIndices1 = new NativeList<PlaneVertexIndexPair>(sharedPlaneData.intersectingPlanes1.Length, Allocator.TempJob))
                        {
                            var handleDep = new JobHandle();

                            // First find vertices from other brush that are inside the other brush, so that any vertex we 
                            // find during the intersection part will be snapped to those vertices and not the other way around

                            using (var foundVertices0 = new NativeStream(math.max(1,sharedPlaneData.usedVertices0.Length), Allocator.TempJob))
                            using (var foundVertices1 = new NativeStream(math.max(1, sharedPlaneData.usedVertices1.Length), Allocator.TempJob))
                            {
                                // TODO: when all vertices of a polygon are inside the other brush, don't bother intersecting it.
                                //       same when two planes overlap each other ...

                                // Find all vertices of brush1 that are inside brush2, and put their intersections into the appropriate loops
                                if (sharedPlaneData.usedVertices0.Length > 0)
                                {
                                    handleDep = FindInsideVertices(handleDep,
                                                                   sharedPlaneData.intersectingPlanes1, sharedPlaneData.usedVertices0, sharedPlaneData.allVertices0,
                                                                   sharedPlaneData.nodeToTreeSpaceMatrix0, float4x4.identity, foundVertices0);
                                }
                                if (sharedPlaneData.usedVertices1.Length > 0)
                                {
                                    handleDep = FindInsideVertices(handleDep,
                                                                   sharedPlaneData.intersectingPlanes0, sharedPlaneData.usedVertices1, sharedPlaneData.allVertices1,
                                                                   sharedPlaneData.nodeToTreeSpaceMatrix1, sharedPlaneData.node1ToNode0, foundVertices1);
                                }

                                if (sharedPlaneData.usedVertices0.Length > 0)
                                {
                                    handleDep = InsertInsideVertices(handleDep,
                                                                     foundVertices0, sharedPlaneData.intersectingPlanes0, sharedPlaneData.intersectingPlaneIndices0,
                                                                     sharedPlaneData.vertexSoup0, foundIndices0);
                                }
                                if (sharedPlaneData.usedVertices1.Length > 0)
                                {
                                    handleDep = InsertInsideVertices(handleDep,
                                                                     foundVertices1, sharedPlaneData.intersectingPlanes1, sharedPlaneData.intersectingPlaneIndices1,                                                                     
                                                                     sharedPlaneData.vertexSoup1, foundIndices1);
                                } 
                            }

                            // Now find all the intersection vertices
                            if (sharedPlaneData.intersectionType == IntersectionType.Intersection)
                            {
                                if (sharedPlaneData.usedPlanePairs1.Length > 0)
                                {
                                    using (var foundVertices = new NativeStream(sharedPlaneData.intersectingPlaneIndices0.Length, Allocator.TempJob))
                                    {
                                        // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                                        FindPlanePairIntersections(foundVertices,
                                                                   sharedPlaneData.nodeToTreeSpaceMatrix0, sharedPlaneData.intersectingPlaneIndices0,
                                                                   sharedPlaneData.intersectingPlanes0, sharedPlaneData.vertexSoup0,
                                                                   sharedPlaneData.intersectingPlanes1, sharedPlaneData.vertexSoup1,
                                                                   sharedPlaneData.usedPlanePairs1,

                                                                   foundIndices0, foundIndices1);
                                    }
                                }

                                if (sharedPlaneData.usedPlanePairs0.Length > 0)
                                {
                                    using (var foundVertices = new NativeStream(sharedPlaneData.intersectingPlaneIndices1.Length, Allocator.TempJob))
                                    {
                                        // Find all edges of brush2 that intersect brush1, and put their intersections into the appropriate loops
                                        FindPlanePairIntersections(foundVertices,
                                                                   sharedPlaneData.nodeToTreeSpaceMatrix0, sharedPlaneData.intersectingPlaneIndices1,
                                                                   sharedPlaneData.intersectingPlanes1, sharedPlaneData.vertexSoup1,
                                                                   sharedPlaneData.intersectingPlanes0, sharedPlaneData.vertexSoup0,
                                                                   sharedPlaneData.usedPlanePairs0,

                                                                   foundIndices1, foundIndices0);
                                    }
                                }
                            }


                            if (foundIndices0.Length > 0)
                            {
                                NativeSortExtension.SortJob(foundIndices0.AsArray()).Complete();
                                using (var uniqueIndices = new NativeList<PlaneVertexIndexPair>(foundIndices0.Length, Allocator.TempJob))
                                using (var loopLengths   = new NativeList<int>(sharedPlaneData.surfaceCategory0.Length, Allocator.TempJob))
                                using (var sortedStack   = new NativeList<int2>(32, Allocator.TempJob))
                                {
                                    var sortLoopsJob0 = new SortLoopsJob
                                    {
                                        foundIndices    = foundIndices0,
                                        uniqueIndices   = uniqueIndices,
                                        surfaceCategory = sharedPlaneData.surfaceCategory0,
                                        sortedStack     = sortedStack,
                                        loopLengths     = loopLengths,
                                        vertexSoup      = sharedPlaneData.vertexSoup0
                                    };
                                    sortLoopsJob0.Execute();

                                    // TODO: make this burstable
                                    var createLoopsJob0 = new CreateLoopsJob
                                    {
                                        foundIndices    = uniqueIndices,
                                        loopLengths     = loopLengths,
                                        surfaceCategory = sharedPlaneData.surfaceCategory0,
                                        outputLoops     = sharedPlaneData.loops01,
                                        brush           = sharedPlaneData.treeBrush1 
                                    };
                                    createLoopsJob0.Execute();
                                }
                            }

                            if (foundIndices1.Length > 0)
                            {
                                NativeSortExtension.SortJob(foundIndices1.AsArray()).Complete();
                                using (var uniqueIndices = new NativeList<PlaneVertexIndexPair>(foundIndices1.Length, Allocator.TempJob))
                                using (var loopLengths   = new NativeList<int>(sharedPlaneData.surfaceCategory1.Length, Allocator.TempJob))
                                using (var sortedStack   = new NativeList<int2>(32, Allocator.TempJob))
                                {
                                    var sortLoopsJob1 = new SortLoopsJob
                                    {
                                        foundIndices    = foundIndices1,
                                        uniqueIndices   = uniqueIndices,
                                        surfaceCategory = sharedPlaneData.surfaceCategory1,
                                        sortedStack     = sortedStack,
                                        loopLengths     = loopLengths,
                                        vertexSoup      = sharedPlaneData.vertexSoup1
                                    };
                                    sortLoopsJob1.Execute();

                                    // TODO: make this burstable
                                    var createLoopsJob1 = new CreateLoopsJob
                                    {
                                        foundIndices    = uniqueIndices,
                                        loopLengths     = loopLengths,
                                        surfaceCategory = sharedPlaneData.surfaceCategory1,
                                        outputLoops     = sharedPlaneData.loops10,
                                        brush           = sharedPlaneData.treeBrush0
                                    };
                                    createLoopsJob1.Execute();
                                }
                            }
                        }
                    }
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
