using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Chisel.Core
{
    public struct BrushMeshBlob
    {
        public struct Polygon
        {
            public Int32 firstEdge;
            public Int32 edgeCount;
            public SurfaceLayers layerDefinition;
        }

        public struct EdgeVertexPlanePair
        {
            public float4   plane0;
            public float4   plane1;
            public int      planeIndex0;
            public int      planeIndex1;
            public int      vertexIndex0;
            public int      vertexIndex1;
        }

        public struct VertexPlanesIndex
        {
            public int      start;
            public int      count;
        }

        // TODO: turn this into AABB
        public Bounds		                    localBounds;
        public BlobArray<float3>	            vertices;
        public BlobArray<BrushMesh.HalfEdge>	halfEdges;
        public BlobArray<int>                   halfEdgePolygonIndices;
        public BlobArray<Polygon>	            polygons;
        public BlobArray<float4>                localPlanes;
//      public BlobArray<EdgeVertexPlanePair>   edgeVertexPlanePair;
//      public BlobArray<float4>                vertexPlanes;
//      public BlobArray<VertexPlanesIndex>     vertexPlaneIndices;
        

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsEmpty()
        {
            return (localPlanes.Length == 0 || polygons.Length == 0 || vertices.Length == 0 || halfEdges.Length == 0);
        }

        public static BlobAssetReference<BrushMeshBlob> Build(BrushMesh brushMesh)
        {
            if (brushMesh == null ||
                brushMesh.vertices.Length < 4 ||
                brushMesh.polygons.Length < 4 ||
                brushMesh.halfEdges.Length < 12)
                return BlobAssetReference<BrushMeshBlob>.Null;
            UnityEngine.Profiling.Profiler.BeginSample("BrushMeshBlob.Build");
            try
            {
                using (var builder = new BlobBuilder(Allocator.Temp))
                {
                    ref var root = ref builder.ConstructRoot<BrushMeshBlob>();
                    root.localBounds = brushMesh.localBounds;
                    builder.Construct(ref root.vertices, brushMesh.vertices);
                    builder.Construct(ref root.halfEdges, brushMesh.halfEdges);
                    builder.Construct(ref root.halfEdgePolygonIndices, brushMesh.halfEdgePolygonIndices);
                    var polygonArray = builder.Allocate(ref root.polygons, brushMesh.polygons.Length);
                    for (int p = 0; p < brushMesh.polygons.Length; p++)
                    {
                        var polygon = brushMesh.polygons[p];
                        polygonArray[p] = new Polygon()
                        {
                            firstEdge = polygon.firstEdge,
                            edgeCount = polygon.edgeCount,
                            layerDefinition = polygon.surface.brushMaterial.LayerDefinition
                        };
                    }
                    /*
                    var vertexHalfEdge = new int[brushMesh.vertices.Length];
                    for (int e = 0; e < brushMesh.halfEdges.Length; e++)
                        vertexHalfEdge[brushMesh.halfEdges[e].vertexIndex] = e + 1;

                    var counter = 0;
                    var vertexPlanes = new List<float4>();
                    var vertexPlaneIndices = new List<VertexPlanesIndex>();
                    for (int v = 0; v < vertexHalfEdge.Length; v++)
                    {
                        var startPlaneIndex = vertexPlanes.Count;
                        var firstHalfEdge = vertexHalfEdge[v] - 1;
                        if (firstHalfEdge < 0)
                        {
                            vertexPlaneIndices.Add(new VertexPlanesIndex() { start = startPlaneIndex, count = 0 });
                        }

                        var iterator = firstHalfEdge;
                        while (iterator != firstHalfEdge)
                        {
                            counter++;
                            if (counter > brushMesh.planes.Length)
                            {
                                Debug.LogError("Malformed mesh found");
                                return BlobAssetReference<BrushMeshBlob>.Null;
                            }

                            var polygonIndex = brushMesh.halfEdgePolygonIndices[iterator];
                            var firstEdge    = brushMesh.polygons[polygonIndex].firstEdge;
                            var edgeCount    = brushMesh.polygons[polygonIndex].edgeCount;

                            vertexPlanes.Add(brushMesh.planes[polygonIndex]);

                            // Next
                            iterator = ((iterator - firstEdge + 1) % edgeCount) + firstEdge;
                            // Twin
                            iterator = brushMesh.halfEdges[iterator].twinIndex;
                        }

                        var lastPlaneIndex = vertexPlanes.Count;
                        vertexPlaneIndices.Add(new VertexPlanesIndex() { start = startPlaneIndex, count = lastPlaneIndex - startPlaneIndex });
                    }

                    builder.Construct(ref root.vertexPlanes,        vertexPlanes.ToArray());
                    builder.Construct(ref root.vertexPlaneIndices,  vertexPlaneIndices.ToArray());

                    var edgeVertexPlanePairArray = builder.Allocate(ref root.edgeVertexPlanePair, brushMesh.halfEdges.Length);
                    for (int e = 0; e < brushMesh.halfEdges.Length; e++)
                    {
                        var halfEdge    = brushMesh.halfEdges[e];
                        var twinEdge    = brushMesh.halfEdges[halfEdge.twinIndex];
                        var planeIndex0 = brushMesh.halfEdgePolygonIndices[e];
                        var planeIndex1 = brushMesh.halfEdgePolygonIndices[halfEdge.twinIndex];
                        var plane0      = brushMesh.planes[planeIndex0];
                        var plane1      = brushMesh.planes[planeIndex1];
                        edgeVertexPlanePairArray[e] = new EdgeVertexPlanePair()
                        {
                            plane0          = plane0,
                            plane1          = plane1,
                            planeIndex0     = planeIndex0,
                            planeIndex1     = planeIndex1,
                            vertexIndex0    = halfEdge.vertexIndex,
                            vertexIndex1    = twinEdge.vertexIndex
                        };
                    }*/

                    builder.Construct(ref root.localPlanes, brushMesh.planes);
                    return builder.CreateBlobAssetReference<BrushMeshBlob>(Allocator.Persistent);
                }
            }
            finally
            {
                UnityEngine.Profiling.Profiler.EndSample();
            }
        }
    }
}
