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

    unsafe struct OverlapIntersectionData : IDisposable
    {
        public int                                  brushNodeID0;
        public BlobAssetReference<BrushMeshBlob>    meshBlob0;
        public float4x4                             treeToNodeSpaceMatrix0;
        
        public NativeList<float4>                   allWorldSpacePlanes;
        public NativeList<int2>                     brushPlaneSegments;
        public int2                                 worldSpacePlanes0Segment;

        public List<Loop>                           basePolygons;
        public Dictionary<int, SurfaceLoops>        intersectionSurfaceLoops;

        public List<int>[]                          intersectionSurfaces;
        public List<IntersectionLoop>               allIntersectionLoops;
        public IntersectionLoop[]                   basePolygonLoops;

        public OverlapIntersectionData(int brushNodeID0, BlobAssetReference<NodeTransformations> transform0, BlobAssetReference<BrushMeshBlob> meshBlob0, Dictionary<int, SurfaceLoops> intersectionSurfaceLoops, List<Loop> basePolygons)
        {
            this.brushNodeID0 = brushNodeID0;
            this.meshBlob0 = meshBlob0;
            this.treeToNodeSpaceMatrix0 = transform0.Value.treeToNode;
            this.allWorldSpacePlanes = new NativeList<float4>(meshBlob0.Value.localPlanes.Length, Allocator.Persistent);
            this.brushPlaneSegments = new NativeList<int2>(intersectionSurfaceLoops.Count, Allocator.Persistent);
            this.worldSpacePlanes0Segment = new int2();

            this.basePolygons = basePolygons;
            this.intersectionSurfaceLoops = intersectionSurfaceLoops;
            this.basePolygonLoops = new IntersectionLoop[basePolygons.Count];
            this.allIntersectionLoops = new List<IntersectionLoop>();
            this.intersectionSurfaces = new List<int>[meshBlob0.Value.localPlanes.Length];
            for (int i = 0; i < this.intersectionSurfaces.Length; i++)
                this.intersectionSurfaces[i] = new List<int>(16);
        }

        public void Dispose()
        {
            if (allWorldSpacePlanes.IsCreated)
                allWorldSpacePlanes.Dispose();

            if (brushPlaneSegments.IsCreated)
                brushPlaneSegments.Dispose();
        }

        public void StoreOutput(Dictionary<int, SurfaceLoops> intersectionSurfaceLoops, Dictionary<int, Loop[]> intersectionLoops, List<Loop> basePolygons)
        {
            /*
            foreach (var intersectionLoop in allIntersectionLoops)
            {
                var surfaceLoops = intersectionSurfaceLoops[intersectionLoop.brushNodeID];
                if (surfaceLoops.surfaces == null)
                    continue;
                if (intersectionLoop.edges.Length >= 3)
                {
                    surfaceLoops.surfaces[intersectionLoop.surfaceIndex][0].SetEdges(intersectionLoop.edges);
                } else
                    surfaceLoops.surfaces[intersectionLoop.surfaceIndex][0].ClearAllEdges();
            }

            foreach (var intersectionLoop in basePolygonLoops)
            {
                basePolygons[intersectionLoop.surfaceIndex].SetEdges(intersectionLoop.edges);
            }
            */

            // Note: those above are only the INTERSECTING surfaces, below ALL surfaces
            foreach (var pair in intersectionSurfaceLoops)
            {
                var surfaceLoops = pair.Value.surfaces;
                if (surfaceLoops == null)
                {
                    intersectionLoops[pair.Key] = null;
                } else
                {
                    var loops = new Loop[surfaceLoops.Length];
                    for (int i = 0; i < surfaceLoops.Length; i++)
                    {
                        if (loops[i] != null)
                            loops[i].Dispose();

                        var surfaceLoop = surfaceLoops[i];
                        if (surfaceLoop == null ||
                            surfaceLoop.Count == 0 ||
                            surfaceLoop[0] == null ||
                            !surfaceLoop[0].Valid)
                        {
                            loops[i] = null;
                            continue;
                        }

                        Debug.Assert(surfaceLoop[0].Valid && (int)surfaceLoop[0].info.interiorCategory < CategoryRoutingRow.Length);

                        var intersectionLoop = surfaceLoop[0];
                        loops[i] = intersectionLoop;
                    }
                    intersectionLoops[pair.Key] = loops;
                }
            }            
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void TransformByTransposedInversedMatrix(float4* outputPlanes, float4* surfaces, int length, float4x4 nodeToTreeSpaceInversed)
        {
            for (int p = 0; p < length; p++)
            {
                var planeVector = math.mul(nodeToTreeSpaceInversed, surfaces[p]);
                outputPlanes[p] = planeVector / math.length(planeVector.xyz);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void TransformByTransposedInversedMatrix(float4* planes, int length, float4x4 nodeToTreeSpaceInversed)
        {
            for (int p = 0; p < length; p++)
            {
                var planeVector = math.mul(nodeToTreeSpaceInversed, planes[p]);
                planes[p] = planeVector / math.length(planeVector.xyz);
            }
        }

        public NativeArray<float4> GetPlanes(int2 segment)
        {
            return allWorldSpacePlanes.AsArray().GetSubArray(segment.x, segment.y);
        }

        public void Execute()
        {
            var treeToNodeSpaceTransposed = math.transpose(treeToNodeSpaceMatrix0);            
            {
                var meshPlanes = (float4*)meshBlob0.Value.localPlanes.GetUnsafePtr();
                var startIndex = allWorldSpacePlanes.Length;
                allWorldSpacePlanes.AddRange(meshPlanes, meshBlob0.Value.localPlanes.Length);
                worldSpacePlanes0Segment = new int2(startIndex, allWorldSpacePlanes.Length - startIndex);
                var worldSpacePlanesPtr = ((float4*)allWorldSpacePlanes.GetUnsafePtr()) + worldSpacePlanes0Segment.x;
                TransformByTransposedInversedMatrix(worldSpacePlanesPtr, worldSpacePlanes0Segment.y, treeToNodeSpaceTransposed);
            }

            for (int s = 0; s < basePolygons.Count; s++)
            {
                var loop = basePolygons[s];
                var intersectionLoop = new IntersectionLoop()
                {
                    segment         = worldSpacePlanes0Segment,
                    edges           = loop.edges,
                    surfaceIndex    = s,
                    brushNodeID     = brushNodeID0
                };

                basePolygonLoops[s] = intersectionLoop;                
            }

            foreach (var pair in intersectionSurfaceLoops)
            {
                var intersectingBrush       = new CSGTreeBrush() { brushNodeID = pair.Key };
                var mesh2                   = BrushMeshManager.GetBrushMeshBlob(intersectingBrush.BrushMesh.BrushMeshID);
                Debug.Assert(!mesh2.Value.IsEmpty());

                treeToNodeSpaceTransposed   = math.transpose(intersectingBrush.TreeToNodeSpaceMatrix);

                var mesh2Planes = (float4*)mesh2.Value.localPlanes.GetUnsafePtr();
                var startIndex  = allWorldSpacePlanes.Length;
                allWorldSpacePlanes.AddRange(mesh2Planes, mesh2.Value.localPlanes.Length);
                var segment = new int2(startIndex, allWorldSpacePlanes.Length - startIndex);
                brushPlaneSegments.Add(segment);
                var worldSpacePlanesPtr = ((float4*)allWorldSpacePlanes.GetUnsafePtr()) + segment.x;
                TransformByTransposedInversedMatrix(worldSpacePlanesPtr, segment.y, treeToNodeSpaceTransposed);
                
                var surfaces = pair.Value.surfaces;
                if (surfaces == null)
                    continue;
                for (int s = 0; s < surfaces.Length; s++)
                {
                    var intersectionSurface = intersectionSurfaces[s];
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
                        segment         = segment,
                        edges           = loop.edges,
                        surfaceIndex    = s,
                        brushNodeID     = pair.Key
                    }; 

                    // We add the intersection loop of this particular brush with our own brush
                    intersectionSurface.Add(allIntersectionLoops.Count);
                    allIntersectionLoops.Add(intersectionLoop);
                }
            }
        }
    }

    struct LocalWorldPair
    {
        public float4 localVertex1;
        public float4 worldVertex;
    }

    [BurstCompile(Debug = false)]
    unsafe struct FindInsideVerticesJob : IJobParallelFor
    { 
        [ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [ReadOnly] public int                   intersectionPlaneIndex1;
        [ReadOnly] public int                   usedVerticesIndex0;

        [WriteOnly] public NativeList<LocalWorldPair>.ParallelWriter vertexWriter;

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
    }
    
    [BurstCompile(Debug = false)]
    unsafe struct InsertInsideVerticesJob : IJobParallelFor
    {
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kPlaneDistanceEpsilon;

        [ReadOnly] public NativeArray<LocalWorldPair>               vertexReader;
        [ReadOnly] public BlobAssetReference<BrushPairIntersection> intersection;
        [ReadOnly] public int                   intersectionPlaneIndex;

        public VertexSoup                       brushVertices;

        [WriteOnly] public NativeList<PlaneVertexIndexPair>.ParallelWriter outputIndices;


        public void Execute(int index) 
        {
            ref var intersectingPlaneIndices    = ref intersection.Value.brushes[intersectionPlaneIndex].localSpacePlaneIndices0;
            ref var intersectingPlanes          = ref intersection.Value.brushes[intersectionPlaneIndex].localSpacePlanes0;

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
