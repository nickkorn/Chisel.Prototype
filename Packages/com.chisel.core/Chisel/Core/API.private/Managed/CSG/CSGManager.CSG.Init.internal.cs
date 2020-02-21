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
        // TODO: unify all epsilons
        public const float  kDistanceEpsilon	    = 0.0001f;//0.01f;
        public const float  kSqrDistanceEpsilon	    = kDistanceEpsilon * kDistanceEpsilon;
        public const float  kMergeEpsilon	        = 0.00006f;
        public const float  kSqrMergeEpsilon	    = kMergeEpsilon * kMergeEpsilon;
        const float         kNormalEpsilon			= 0.9999f;
        const float         kPlaneDistanceEpsilon	= 0.0006f;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool IsDegenerate(VertexSoup soup, List<ushort> indices)
        {
            if (indices.Count < 3)
                return true;

            var vertices = soup.vertices;
            for (int i = 0; i < indices.Count; i++)
            {
                var vertexIndex1 = indices[i];
                var vertex1      = vertices[vertexIndex1];
                for (int j = 1; j < indices.Count - 1; j++)
                {
                    int a = (i + j) % indices.Count;
                    int b = (a + 1) % indices.Count;

                    var vertexIndexA = indices[a];
                    var vertexIndexB = indices[b];

                    // Loop loops back on same vertex
                    if (vertexIndex1 == vertexIndexA || 
                        vertexIndex1 == vertexIndexB ||
                        vertexIndexA == vertexIndexB)
                        continue;

                    var vertexA = vertices[vertexIndexA];
                    var vertexB = vertices[vertexIndexB];

                    var distance = GeometryMath.SqrDistanceFromPointToLineSegment(vertex1, vertexA, vertexB);
                    if (distance <= CSGManagerPerformCSG.kSqrDistanceEpsilon)
                        return true;
                }
            }
            return false;
        }
        

        #region GenerateBasePolygons

        static readonly List<ushort> s_Indices = new List<ushort>(32);
        public static Bounds GenerateBasePolygons(BrushLoops outputLoops)
        {
            if (!BrushMeshManager.IsBrushMeshIDValid(outputLoops.brush.BrushMesh.BrushMeshID))
                return new Bounds();

            var mesh                    = BrushMeshManager.GetBrushMesh(outputLoops.brush.BrushMesh.BrushMeshID);
            if (mesh == null)
            {
                Debug.Log("mesh == null");
                return new Bounds();
            }

            var halfEdges               = mesh.halfEdges;
            var vertices                = mesh.vertices;
            var surfaces                = mesh.surfaces;
            var polygons                = mesh.polygons;
            //var surfacesAroundVertex    = mesh.surfacesAroundVertex;
            var nodeToTreeSpaceMatrix   = outputLoops.brush.NodeToTreeSpaceMatrix;
            outputLoops.basePolygons.Clear();
            if (outputLoops.basePolygons.Capacity < polygons.Length)
                outputLoops.basePolygons.Capacity = polygons.Length;

            outputLoops.vertexSoup.Clear(vertices.Length);


            var removeIdenticalIndicesJob = new RemoveIdenticalIndicesJob
            {
                vertices = outputLoops.vertexSoup.vertices
            };

            var min = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            var max = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);

            for (int p = 0; p < polygons.Length; p++)
            {
                var polygon      = polygons[p];
                var surfaceIndex = polygon.surfaceID;
                var firstEdge    = polygon.firstEdge;
                var lastEdge     = firstEdge + polygon.edgeCount;

                if (polygon.edgeCount < 3 ||
                    surfaceIndex >= surfaces.Length)
                    continue;

                var localPlane          = (Plane)surfaces[surfaceIndex];
                var worldPlane          = outputLoops.brush.NodeToTreeSpaceMatrix.TransformPlane(localPlane);

                MathExtensions.CalculateTangents(worldPlane.normal, out Vector3 right, out Vector3 forward);

                s_Indices.Clear();
                if (s_Indices.Capacity < lastEdge - firstEdge)
                    s_Indices.Capacity = lastEdge - firstEdge;

                //if (loop != null && loop.worldPlane.normal == Vector3.zero) Debug.LogError("!");
                for (int e = firstEdge; e < lastEdge; e++)
                {
                    var vertexIndex = halfEdges[e].vertexIndex;
                    var vertex      = nodeToTreeSpaceMatrix.MultiplyPoint(vertices[vertexIndex]);
                    min.x = Mathf.Min(min.x, vertex.x); max.x = Mathf.Max(max.x, vertex.x);
                    min.y = Mathf.Min(min.y, vertex.y); max.y = Mathf.Max(max.y, vertex.y);
                    min.z = Mathf.Min(min.z, vertex.z); max.z = Mathf.Max(max.z, vertex.z);

                    var newIndex = outputLoops.vertexSoup.Add(vertex);
                    Debug.Assert(s_Indices.Count == 0 ||
                                    (s_Indices[0] != newIndex &&
                                    s_Indices[s_Indices.Count - 1] != newIndex));
                    s_Indices.Add(newIndex);
                }


                // THEORY: can end up with duplicate vertices when close enough vertices are snapped together
                removeIdenticalIndicesJob.indices = s_Indices;
                // TODO: eventually actually use jobs
                removeIdenticalIndicesJob.Execute();

                if (CSGManagerPerformCSG.IsDegenerate(outputLoops.vertexSoup, s_Indices))
                    continue;

                var surfacePolygon = new Loop()
                {
                    info = new LoopInfo()
                    {
                        basePlaneIndex  = surfaceIndex,
                        brush           = outputLoops.brush,
                        layers          = polygon.surface.brushMaterial.LayerDefinition,
                        worldPlane      = worldPlane,
                        right           = right,
                        forward         = forward
                    },
                    interiorCategory = (CategoryGroupIndex)(int)CategoryIndex.ValidAligned,
                    convex           = true,
                    holes            = new List<Loop>()
                };

                surfacePolygon.indices.AddRange(s_Indices);
                surfacePolygon.AddEdges(s_Indices);
                outputLoops.basePolygons.Add(surfacePolygon);

                #if false
                var builder = new System.Text.StringBuilder();
                builder.AppendLine($"{p}: {s_Indices.Count} {surfacePolygon.info.worldPlane}");
                CSGManagerPerformCSG.Dump(builder, surfacePolygon, outputLoops.vertexSoup, Quaternion.FromToRotation(surfacePolygon.info.worldPlane.normal, Vector3.forward));
                Debug.Log(builder.ToString());
                #endif
            }
            var bounds = new Bounds();
            if (!float.IsInfinity(min.x)) 
                bounds.SetMinMax(min, max);
            return bounds;
        }
        #endregion
    }
#endif
}
