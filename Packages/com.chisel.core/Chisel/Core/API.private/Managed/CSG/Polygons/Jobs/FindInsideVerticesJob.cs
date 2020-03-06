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
        public NativeArray<float4>  selfPlanes;
        public List<Edge>           edges;
        public Loop                 loop;
        public List<ushort>         indices;
    }

    unsafe struct OverlapIntersectionData : IDisposable
    {
        public CSGTreeBrush                         brush0;
        public BlobAssetReference<BrushMeshBlob>    meshBlob0;
        public float4x4                             treeToNodeSpaceMatrix0;

        public NativeArray<float4>                  allWorldSpacePlanes;

        public List<Loop>                           basePolygons;
        public Dictionary<int, SurfaceLoops>        intersectionSurfaceLoops;

        public List<IntersectionLoop>[]             intersectionSurfaces;
        public List<IntersectionLoop>               allIntersectionLoops;

        public Dictionary<int, NativeArray<float4>>     brushPlanes;
        
        public OverlapIntersectionData(CSGTreeBrush brush0, BlobAssetReference<BrushMeshBlob> meshBlob0, Dictionary<int, SurfaceLoops> intersectionSurfaceLoops, List<Loop> basePolygons)
        {
            this.brush0                 = brush0;
            this.meshBlob0              = meshBlob0;
            this.treeToNodeSpaceMatrix0 = brush0.TreeToNodeSpaceMatrix;
            this.allWorldSpacePlanes    = new NativeArray<float4>(meshBlob0.Value.planes.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            
            this.basePolygons               = basePolygons;
            this.intersectionSurfaceLoops   = intersectionSurfaceLoops;
            this.allIntersectionLoops       = new List<IntersectionLoop>();
            this.intersectionSurfaces       = new List<IntersectionLoop>[0];
            this.brushPlanes                = new Dictionary<int, NativeArray<float4>>();
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

        public void Execute()
        {
            var treeToNodeSpaceTransposed = math.transpose(brush0.TreeToNodeSpaceMatrix);
            fixed (float4* meshSurfaces = &meshBlob0.Value.planes[0])
            {
                float4* meshPlanes = (float4*)meshSurfaces;
                var worldSpacePlanesPtr = (float4*)allWorldSpacePlanes.GetUnsafePtr();
                TransformByTransposedInversedMatrix(worldSpacePlanesPtr, meshPlanes, meshBlob0.Value.planes.Length, treeToNodeSpaceTransposed);
            }

            brushPlanes.Clear();
            allIntersectionLoops.Clear();
            var intersectionSurfaceLength = meshBlob0.Value.planes.Length;
            intersectionSurfaces = new List<IntersectionLoop>[intersectionSurfaceLength];
            for (int i = 0; i < intersectionSurfaceLength; i++)
                intersectionSurfaces[i] = new List<IntersectionLoop>(16);

            foreach (var pair in intersectionSurfaceLoops)
            {
                var intersectingBrush = new CSGTreeBrush() { brushNodeID = pair.Key };
                var mesh2 = BrushMeshManager.GetBrushMesh(intersectingBrush.BrushMesh.BrushMeshID);
                Debug.Assert(mesh2 != null);

                if (mesh2.planes.Length == 0 || pair.Value == null)
                    continue;

                var worldSpacePlanes = new NativeArray<float4>(mesh2.planes.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                var worldSpacePlanesPtr = (float4*)worldSpacePlanes.GetUnsafePtr();
                treeToNodeSpaceTransposed = math.transpose(intersectingBrush.TreeToNodeSpaceMatrix);
                fixed (float4* mesh2Surfaces = &mesh2.planes[0])
                {
                    float4* mesh2Planes = (float4*)mesh2Surfaces;
                    TransformByTransposedInversedMatrix(worldSpacePlanesPtr, mesh2Planes, mesh2.planes.Length, treeToNodeSpaceTransposed);
                }
                brushPlanes.Add(intersectingBrush.brushNodeID, worldSpacePlanes);

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
                        selfPlanes  = worldSpacePlanes,
                        indices     = loop.indices,
                        edges       = loop.edges,
                        loop        = loop
                    };

                    // We add the intersection loop of this particular brush with our own brush
                    intersectionSurface.Add(intersectionLoop);
                    allIntersectionLoops.Add(intersectionLoop);
                }
            }
        }

        public void StoreOutput()
        {
            foreach (var brushPlane in brushPlanes)
                brushPlane.Value.Dispose();
            brushPlanes.Clear();
        }

        public void Dispose()
        {
            if (allWorldSpacePlanes.IsCreated)
                allWorldSpacePlanes.Dispose();
        }
    }

    [BurstCompile]
    unsafe struct FindInsideVerticesJob : IJobParallelFor
    { 
        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeList<float4>  intersectingPlanes1;
        [ReadOnly] public NativeList<int>     usedVertices0;
        [NativeDisableUnsafePtrRestriction] [ReadOnly] public float3* allVertices0;
        [ReadOnly] public float4x4            nodeToTreeSpaceMatrix;
        [ReadOnly] public float4x4            vertexToLocal0;

        [NativeDisableUnsafePtrRestriction]
        [WriteOnly] public NativeStream.Writer vertexWriter;

        public void Execute(int index)
        {
            vertexWriter.BeginForEachIndex(index);
            var vertexIndex1 = usedVertices0[index];
            var brushVertex1 = new float4(allVertices0[vertexIndex1], 1);
            var localVertex1 = math.mul(vertexToLocal0, brushVertex1);
            if (!CSGManagerPerformCSG.IsOutsidePlanes(intersectingPlanes1, localVertex1))
            { 
                var worldVertex = math.mul(nodeToTreeSpaceMatrix, brushVertex1);
                vertexWriter.Write(localVertex1);
                vertexWriter.Write(worldVertex);
            }
            vertexWriter.EndForEachIndex();
        }
    }
    
    [BurstCompile]
    unsafe struct InsertInsideVerticesJob : IJob
    {
        const float kPlaneDistanceEpsilon = CSGManagerPerformCSG.kPlaneDistanceEpsilon;

        // Add [NativeDisableContainerSafetyRestriction] when done, for performance
        [ReadOnly] public NativeStream.Reader   vertexReader;
        [ReadOnly] public NativeList<float4>    intersectingPlanes;
        [ReadOnly] public NativeList<int>       intersectingPlaneIndices;

        public VertexSoup                       brushVertices;

        [WriteOnly] public NativeList<PlaneVertexIndexPair> outputIndices;


        public void Execute() 
        {
            int maxIndex = vertexReader.ForEachCount;

            int index = 0;
            vertexReader.BeginForEachIndex(index++);
            while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                vertexReader.BeginForEachIndex(index++);
            while (vertexReader.RemainingItemCount > 0)
            {
                var localVertex1    = vertexReader.Read<float4>();
                var worldVertex     = vertexReader.Read<float4>();

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
                            worldVertexIndex = brushVertices.Add(worldVertex.xyz);
                        outputIndices.Add(new PlaneVertexIndexPair() { planeIndex = (ushort)planeIndex, vertexIndex = (ushort)worldVertexIndex });
                    }
                }

                while (vertexReader.RemainingItemCount == 0 && index < maxIndex)
                    vertexReader.BeginForEachIndex(index++);
            }
        }
    }
}
