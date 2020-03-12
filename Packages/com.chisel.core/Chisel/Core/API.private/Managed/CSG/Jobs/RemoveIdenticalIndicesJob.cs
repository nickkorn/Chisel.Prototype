using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    
    [BurstCompile(Debug = false)]
    public unsafe struct RemoveIdenticalIndicesEdgesJob : IJob
    {
        public NativeList<Edge> edges;

        public static void RemoveDuplicates(ref NativeList<Edge> edges)
        {
            if (edges.Length < 3)
            {
                edges.Clear();
                return;
            }

            for (int e = edges.Length - 1; e >= 0; e--)
            {
                if (edges[e].index1 != edges[e].index2)
                    continue;
                edges.RemoveAtSwapBack(e);
            }
        }

        public void Execute()
        {
            RemoveDuplicates(ref edges);
        }
    }


    // TODO: probably makes sense to break this up into multiple pieces/multiple jobs that can run parallel,
    //      but requires that change some storage formats first
    [BurstCompile(Debug = false)]
    public unsafe struct CopyPolygonToIndicesJob : IJob
    {
        [ReadOnly] public BlobAssetReference<BrushMeshBlob> mesh;
        [ReadOnly] public int       polygonIndex;
        [ReadOnly] public float4x4  nodeToTreeSpaceMatrix;
        [ReadOnly] public float4x4  nodeToTreeSpaceInvertedTransposedMatrix;

        public VertexSoup           vertexSoup;
        public NativeList<Edge>     edges;

        // TODO: do this in separate loop so we don't need to rely on pointers to make this work
        [NativeDisableUnsafePtrRestriction] public AABB* aabb;
        [WriteOnly] [NativeDisableUnsafePtrRestriction] public float4* worldPlane;

        public void Execute()
        {
            ref var halfEdges   = ref mesh.Value.halfEdges;
            ref var vertices    = ref mesh.Value.vertices;
            ref var planes      = ref mesh.Value.localPlanes;
            ref var polygon     = ref mesh.Value.polygons[polygonIndex];

            var localPlane  = planes[polygonIndex];
            var firstEdge   = polygon.firstEdge;
            var lastEdge    = firstEdge + polygon.edgeCount;
            var indexCount  = lastEdge - firstEdge;

            vertexSoup.Reserve(indexCount); // ensure we have at least this many extra vertices in capacity

            var min = aabb->min;
            var max = aabb->max;

            // TODO: put in job so we can burstify this, maybe join with RemoveIdenticalIndicesJob & IsDegenerate?
            for (int e = firstEdge; e < lastEdge; e++)
            {
                var vertexIndex = halfEdges[e].vertexIndex;
                var localVertex = new float4(vertices[vertexIndex], 1);
                var worldVertex = math.mul(nodeToTreeSpaceMatrix, localVertex);

                // TODO: could do this in separate loop on vertexSoup vertices
                min.x = math.min(min.x, worldVertex.x); max.x = math.max(max.x, worldVertex.x);
                min.y = math.min(min.y, worldVertex.y); max.y = math.max(max.y, worldVertex.y);
                min.z = math.min(min.z, worldVertex.z); max.z = math.max(max.z, worldVertex.z);

                var newIndex = vertexSoup.AddNoResize(worldVertex.xyz);
                if (e > firstEdge)
                {
                    var edge = edges[edges.Length - 1];
                    edge.index2 = newIndex;
                    edges[edges.Length - 1] = edge;
                }
                edges.Add(new Edge() { index1 = newIndex });
            }
            {
                var edge = edges[edges.Length - 1];
                edge.index2 = edges[0].index1;
                edges[edges.Length - 1] = edge;
            }

            RemoveIdenticalIndicesEdgesJob.RemoveDuplicates(ref edges);

            if (edges.Length > 0 &&
                CSGManagerPerformCSG.IsDegenerate(vertexSoup, edges))
            {
                edges.Clear();
            }

            if (edges.Length > 0)
            {
                aabb->min = min;
                aabb->max = max;

                localPlane = math.mul(nodeToTreeSpaceInvertedTransposedMatrix, localPlane);
                var length = math.length(localPlane.xyz);
                if (length > 0)
                    localPlane /= length;
                *worldPlane = localPlane;
            }
        }
    }
#endif
}
