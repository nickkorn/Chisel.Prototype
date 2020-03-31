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
    struct IntersectionLoop
    {
        public int2                 segment;
        public int                  surfaceIndex;
        public int                  brushNodeID;
        public NativeList<Edge>     edges;
    }

    struct LocalWorldPair
    {
        public float4 localVertex1;
        public float4 worldVertex;
    }

    [BurstCompile(Debug = false)]
    unsafe struct FindInsideVerticesJob : IJob// IJobParallelFor
    { 
        [ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [ReadOnly] public int                   intersectionPlaneIndex1;
        [ReadOnly] public int                   usedVerticesIndex0;

        [WriteOnly] public NativeList<LocalWorldPair> vertexWriter;

        public void Execute(int index)
        {
            ref var intersectingPlanes1 = ref intersection.Value.brushes[intersectionPlaneIndex1].localSpacePlanes0;
            ref var usedVertices0       = ref intersection.Value.brushes[usedVerticesIndex0].usedVertices;
            ref var allVertices0        = ref intersection.Value.brushes[usedVerticesIndex0].blobMesh.Value.vertices;
            var nodeToTreeSpaceMatrix   = intersection.Value.brushes[usedVerticesIndex0].transformation.nodeToTree;
            var vertexToLocal0          = usedVerticesIndex0 == 0 ? float4x4.identity : intersection.Value.brushes[usedVerticesIndex0].toOtherBrushSpace;

            var vertexIndex1 = usedVertices0[index];
            var brushVertex1 = new float4(allVertices0[vertexIndex1], 1);
            var localVertex1 = math.mul(vertexToLocal0, brushVertex1);
            if (!CSGManagerPerformCSG.IsOutsidePlanes(ref intersectingPlanes1, localVertex1))
            { 
                var worldVertex = math.mul(nodeToTreeSpaceMatrix, brushVertex1);
                vertexWriter.AddNoResize(new LocalWorldPair() { localVertex1 = localVertex1, worldVertex = worldVertex });
            }
        }
        
        public void Execute()
        {
            ref var usedVertices0 = ref intersection.Value.brushes[usedVerticesIndex0].usedVertices;
            for (int index = 0; index < usedVertices0.Length; index++)
            {
                Execute(index);
            }
        }
    }
    
    [BurstCompile(Debug = false)]
    unsafe struct InsertInsideVerticesJob : IJob
    {
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kPlaneDistanceEpsilon;

        [ReadOnly] public NativeArray<LocalWorldPair>               vertexReader;
        [ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [ReadOnly] public int                   intersectionPlaneIndex;

        [WriteOnly] public VertexSoup                        brushVertices;
        [WriteOnly] public NativeList<PlaneVertexIndexPair>  outputIndices;


        public void Execute() 
        {
            ref var intersectingPlaneIndices    = ref intersection.Value.brushes[intersectionPlaneIndex].localSpacePlaneIndices0;
            ref var intersectingPlanes          = ref intersection.Value.brushes[intersectionPlaneIndex].localSpacePlanes0;

            for (int index=0;index< vertexReader.Length;index++)
            { 
                //for (int index = 0; index < vertexReader.Length; index++)
                {
                    var localVertex1    = vertexReader[index].localVertex1;
                    var worldVertex     = vertexReader[index].worldVertex;

                    var worldVertexIndex = -1;
                    // TODO: optimize this, we already know these vertices are ON the planes of this brush, just not which: this can be precalculated
                    for (int i = 0; i < intersectingPlaneIndices.Length; i++)
                    {
                        // only use a plane when the vertex is ON it
                        var distance = math.dot(intersectingPlanes[i], localVertex1);
                        if (distance >= -kPlaneDistanceEpsilon && distance <= kPlaneDistanceEpsilon) // Note: this is false on NaN/Infinity, so don't invert
                        {
                            var planeIndex = intersectingPlaneIndices[i];
                            if (worldVertexIndex == -1)
                                worldVertexIndex = brushVertices.AddNoResize(worldVertex.xyz);
                            outputIndices.AddNoResize(new PlaneVertexIndexPair() { planeIndex = (ushort)planeIndex, vertexIndex = (ushort)worldVertexIndex });
                        }
                    }
                }
            }
        }
    }
}
