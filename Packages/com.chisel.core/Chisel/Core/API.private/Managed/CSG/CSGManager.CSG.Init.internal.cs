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
#if USE_MANAGED_CSG_IMPLEMENTATION
    static partial class CSGManagerPerformCSG
    {
        // TODO: unify all epsilons
        public const float  kDistanceEpsilon	    = 0.0001f;//0.01f;
        public const float  kSqrDistanceEpsilon	    = kDistanceEpsilon * kDistanceEpsilon;
        public const float  kMergeEpsilon	        = 0.0005f;
        public const float  kSqrMergeEpsilon	    = kMergeEpsilon * kMergeEpsilon;
        public const float  kNormalEpsilon			= 0.9999f;
        public const float  kPlaneDistanceEpsilon	= 0.0006f;

        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool IsDegenerate(in VertexSoup soup, List<ushort> indices)
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
        

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool IsDegenerate(in VertexSoup soup, NativeList<ushort> indices)
        {
            if (indices.Length < 3)
                return true;

            var vertices = soup.vertices;
            for (int i = 0; i < indices.Length; i++)
            {
                var vertexIndex1 = indices[i];
                var vertex1      = vertices[vertexIndex1];
                for (int j = 1; j < indices.Length - 1; j++)
                {
                    int a = (i + j) % indices.Length;
                    int b = (a + 1) % indices.Length;

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

        //static readonly List<ushort> s_Indices = new List<ushort>(32);
        public static Bounds GenerateBasePolygons(BrushLoops outputLoops)
        {
            if (!BrushMeshManager.IsBrushMeshIDValid(outputLoops.brush.BrushMesh.BrushMeshID))
                return new Bounds();

            var mesh = BrushMeshManager.GetBrushMeshBlob(outputLoops.brush.BrushMesh.BrushMeshID);
            if (mesh == BlobAssetReference<BrushMeshBlob>.Null)
            {
                Debug.Log("mesh == null");
                return new Bounds();
            }

            ref var vertices   = ref mesh.Value.vertices;
            ref var planes     = ref mesh.Value.planes;
            ref var polygons   = ref mesh.Value.polygons;
            var nodeToTreeSpaceMatrix   = outputLoops.brush.NodeToTreeSpaceMatrix;
            outputLoops.basePolygons.Clear(); 
            if (outputLoops.basePolygons.Capacity < polygons.Length)
                outputLoops.basePolygons.Capacity = polygons.Length;

            outputLoops.vertexSoup.Initialize(vertices.Length);

            var min = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            var max = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);

            for (int p = 0; p < polygons.Length; p++)
            {
                var polygon      = polygons[p];

                if (polygon.edgeCount < 3 ||
                    p >= planes.Length)
                    continue;

                // TODO: simplify
                var localPlane          = new Plane(planes[p].xyz, planes[p].w);
                var worldPlane          = outputLoops.brush.NodeToTreeSpaceMatrix.TransformPlane(localPlane);
                var worldPlaneVector    = new float4(worldPlane.normal, worldPlane.distance);

                var firstEdge    = polygon.firstEdge;
                var lastEdge     = firstEdge + polygon.edgeCount;
                var indexCount   = lastEdge - firstEdge;

                using (var indices = new NativeList<ushort>(indexCount, Allocator.TempJob))
                {
                    // THEORY: can end up with duplicate vertices when close enough vertices are snapped together
                    var removeIdenticalIndicesJob = new CopyPolygonToIndicesJob
                    {
                        mesh = mesh,
                        polygonIndex = p,
                        nodeToTreeSpaceMatrix = nodeToTreeSpaceMatrix,
                        vertexSoup = outputLoops.vertexSoup,
                        indices = indices,

                        min = min,
                        max = max
                    };

                    removeIdenticalIndicesJob.Run();

                    min = removeIdenticalIndicesJob.min;
                    max = removeIdenticalIndicesJob.max;

                    if (indices.Length == 0)
                        continue;

                    var surfacePolygon = new Loop()
                    {
                        info = new SurfaceInfo()
                        {
                            worldPlane          = worldPlaneVector,
                            layers              = polygon.layerDefinition,
                            basePlaneIndex      = p,
                            brush               = outputLoops.brush,
                            interiorCategory    = (CategoryGroupIndex)(int)CategoryIndex.ValidAligned,
                        },
                        holes = new List<Loop>()
                    };

                    for (int i = 0; i < indices.Length; i++)
                        surfacePolygon.indices.Add(indices[i]);
                    surfacePolygon.AddEdges(surfacePolygon.indices);
                    outputLoops.basePolygons.Add(surfacePolygon);

                    #if false
                    var builder = new System.Text.StringBuilder();
                    builder.AppendLine($"{p}: {s_Indices.Count} {surfacePolygon.info.worldPlane}");
                    CSGManagerPerformCSG.Dump(builder, surfacePolygon, outputLoops.vertexSoup, Quaternion.FromToRotation(surfacePolygon.info.worldPlane.normal, Vector3.forward));
                    Debug.Log(builder.ToString());
                    #endif
                }
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
