﻿using System;
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
    public struct Edge : IEquatable<Edge>
    {
        public ushort index1;
        public ushort index2;

        public bool Equals(Edge other)
        {
            return index1 == other.index1 && index2 == other.index2;
        }
        public override int GetHashCode()
        {
            return (int)math.hash(new int2(index1, index2));
        }

        public override string ToString() => $"({index1}, {index2})";

        internal void Flip()
        {
            //if (index1 < index2)
            //    return;
            var t = index1; index1 = index2; index2 = t;
        }
    }

    [BurstCompile(CompileSynchronously = true)] // Fails for some reason
    unsafe struct GenerateSurfaceTrianglesJob : IJobParallelFor
    {
        [NoAlias, ReadOnly] public NativeArray<int>                                         treeBrushNodeIndices;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>> basePolygons;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>> brushWorldPlanes;

        [NoAlias, WriteOnly] public NativeHashMap<int, BlobAssetReference<ChiselBrushRenderBuffer>>.ParallelWriter brushRenderBuffers;

        [NoAlias, ReadOnly] public NativeStream.Reader input;
        
        [BurstDiscard]
        public static void InvalidFinalCategory(CategoryIndex _interiorCategory)
        {
            Debug.Assert(false, $"Invalid final category {_interiorCategory}");
        }

        const Allocator allocator = Allocator.Temp;

        public void Execute(int index)
        {
            //var brushNodeIndex = treeBrushNodeIndices[index];
            var count = input.BeginForEachIndex(index);
            if (count == 0)
                return;

            VertexSoup brushVertices;
            NativeListArray<int> surfaceLoopIndices;
            NativeList<SurfaceInfo> surfaceLoopAllInfos;
            NativeListArray<Edge> surfaceLoopAllEdges;


            var brushNodeIndex = input.Read<int>();
            var vertexCount = input.Read<int>();
            brushVertices = new VertexSoup(vertexCount, allocator);
            for (int v = 0; v < vertexCount; v++)
            {
                var vertex = input.Read<float3>();
                brushVertices.AddNoResize(vertex);
            }


            var surfaceOuterCount = input.Read<int>();
            surfaceLoopIndices = new NativeListArray<int>(surfaceOuterCount, allocator);
            surfaceLoopIndices.ResizeExact(surfaceOuterCount);
            for (int o = 0; o < surfaceOuterCount; o++)
            {
                var surfaceInnerCount = input.Read<int>();
                var inner = surfaceLoopIndices[o];
                inner.ResizeUninitialized(surfaceInnerCount);
                for (int i = 0; i < surfaceInnerCount; i++)
                {
                    inner[i] = input.Read<int>();
                }
            }

            var surfaceLoopCount = input.Read<int>();
            surfaceLoopAllInfos = new NativeList<SurfaceInfo>(surfaceLoopCount, allocator);
            surfaceLoopAllEdges = new NativeListArray<Edge>(surfaceLoopCount, allocator);

            surfaceLoopAllInfos.ResizeUninitialized(surfaceLoopCount);
            surfaceLoopAllEdges.ResizeExact(surfaceLoopCount);
            for (int l = 0; l < surfaceLoopCount; l++)
            {
                surfaceLoopAllInfos[l] = input.Read<SurfaceInfo>();
                var edgeCount   = input.Read<int>();
                var edgesInner  = surfaceLoopAllEdges[l];
                edgesInner.ResizeUninitialized(edgeCount);
                for (int e = 0; e < edgeCount; e++)
                {
                    edgesInner[e] = input.Read<Edge>();
                }
            }
            input.EndForEachIndex();





            var maxLoops = 0;
            var maxIndices = 0;
            for (int s = 0; s < surfaceLoopIndices.Length; s++)
            {
                var length = surfaceLoopIndices[s].Length;
                maxIndices += length;
                maxLoops = math.max(maxLoops, length);
            }


            var basePolygonsBlob = basePolygons[brushNodeIndex];
            var brushWorldPlanesBlob = brushWorldPlanes[brushNodeIndex];

            var pointCount                  = brushVertices.Length + 2;
            var context_points              = new NativeArray<float2>(pointCount, allocator);
            var context_edges               = new NativeArray<int>(pointCount, allocator);
            var context_allEdges            = new NativeList<Poly2Tri.DTSweep.DirectedEdge>(pointCount, allocator);
            var context_sortedPoints        = new NativeList<int>(pointCount, allocator);
            var context_triangles           = new NativeList<Poly2Tri.DTSweep.DelaunayTriangle>(pointCount * 3, allocator);
            var context_triangleInterior    = new NativeList<bool>(pointCount * 3, allocator);
            var context_advancingFrontNodes = new NativeList<Poly2Tri.DTSweep.AdvancingFrontNode>(pointCount, allocator);
            var context_edgeLookupEdges     = new NativeListArray<Chisel.Core.Edge>(pointCount, allocator);
            var context_edgeLookups         = new NativeHashMap<int, int>(pointCount, allocator);
            var context_foundLoops          = new NativeListArray<Chisel.Core.Edge>(pointCount, allocator);

            var context_children            = new NativeListArray<int>(64, allocator);
            var context_inputEdgesCopy      = new NativeList<Edge>(64, allocator);


            var builder = new BlobBuilder(allocator);
            ref var root = ref builder.ConstructRoot<ChiselBrushRenderBuffer>();
            var surfaceRenderBuffers = builder.Allocate(ref root.surfaces, surfaceLoopIndices.Length);

            var loops               = new NativeList<int>(maxLoops, allocator);
            var surfaceIndexList    = new NativeList<int>(maxIndices, allocator);
            for (int s = 0; s < surfaceLoopIndices.Length; s++)
            {
                ref var surfaceRenderBuffer = ref surfaceRenderBuffers[s];
                loops.Clear();

                var loopIndices = surfaceLoopIndices[s];
                for (int l = 0; l < loopIndices.Length; l++)
                {
                    var surfaceLoopIndex = loopIndices[l];
                    var surfaceLoopInfo = surfaceLoopAllInfos[surfaceLoopIndex];
                    var _interiorCategory = (CategoryIndex)surfaceLoopInfo.interiorCategory;
                    //if (_interiorCategory > CategoryIndex.LastCategory)
                    //    InvalidFinalCategory(_interiorCategory);

                    // TODO: put this check in previous job
                    if (_interiorCategory != CategoryIndex.ValidAligned &&
                        _interiorCategory != CategoryIndex.ValidReverseAligned)
                        continue;

                    var surfaceLoopEdges = surfaceLoopAllEdges[surfaceLoopIndex];
                    // TODO: put this check in previous job
                    if (surfaceLoopEdges.Length < 3)
                        continue;

                    loops.Add(surfaceLoopIndex);
                }

                // TODO: why are we doing this in tree-space? better to do this in brush-space, then we can more easily cache this
                var surfaceIndex = s;
                var surfaceLayers = basePolygonsBlob.Value.surfaces[surfaceIndex].layers;
                var surfaceWorldPlane = brushWorldPlanesBlob.Value.worldPlanes[surfaceIndex];
                var UV0 = basePolygonsBlob.Value.surfaces[surfaceIndex].UV0;
                var localSpaceToPlaneSpace = MathExtensions.GenerateLocalToPlaneSpaceMatrix(surfaceWorldPlane);
                var uv0Matrix = math.mul(UV0.ToFloat4x4(), localSpaceToPlaneSpace);

                // Ensure we have the rotation properly calculated, and have a valid normal
                float3 normal = surfaceWorldPlane.xyz;
                quaternion rotation;
                if (((Vector3)normal) == Vector3.forward)
                    rotation = quaternion.identity;
                else
                    rotation = (quaternion)Quaternion.FromToRotation(normal, Vector3.forward);

                surfaceIndexList.Clear();

                CategoryIndex interiorCategory = CategoryIndex.ValidAligned;

                for (int l = 0; l < loops.Length; l++)
                {
                    var loopIndex = loops[l];
                    var loopEdges = surfaceLoopAllEdges[loopIndex];
                    var loopInfo = surfaceLoopAllInfos[loopIndex];
                    interiorCategory = (CategoryIndex)loopInfo.interiorCategory;

                    Debug.Assert(surfaceIndex == loopInfo.basePlaneIndex);




                    var surfaceIndicesArray = new NativeList<int>(allocator);
                    
                    var context = new Poly2Tri.DTSweep
                    {
                        vertices            = brushVertices,
                        points              = context_points,
                        edges               = context_edges,
                        allEdges            = context_allEdges,
                        triangles           = context_triangles,
                        triangleInterior    = context_triangleInterior,
                        sortedPoints        = context_sortedPoints,
                        advancingFrontNodes = context_advancingFrontNodes,
                        edgeLookupEdges     = context_edgeLookupEdges,
                        edgeLookups         = context_edgeLookups,
                        foundLoops          = context_foundLoops,
                        children            = context_children,
                        inputEdgesCopy      = context_inputEdgesCopy,
                        rotation            = rotation,
                        normal              = normal,
                        inputEdges          = loopEdges,
                        surfaceIndicesArray = surfaceIndicesArray
                    };
                    context.Execute();



                    if (surfaceIndicesArray.Length >= 3)
                    {
                        if (interiorCategory == CategoryIndex.ValidReverseAligned ||
                            interiorCategory == CategoryIndex.ReverseAligned)
                        {
                            var maxCount = surfaceIndicesArray.Length - 1;
                            for (int n = (maxCount / 2); n >= 0; n--)
                            {
                                var t = surfaceIndicesArray[n];
                                surfaceIndicesArray[n] = surfaceIndicesArray[maxCount - n];
                                surfaceIndicesArray[maxCount - n] = t;
                            }
                        }

                        for (int n = 0; n < surfaceIndicesArray.Length; n++)
                            surfaceIndexList.Add(surfaceIndicesArray[n]);
                    }
                    surfaceIndicesArray.Dispose();
                }

                if (surfaceIndexList.Length == 0)
                    continue;

                var surfaceIndicesCount = surfaceIndexList.Length;
                var surfaceIndices = (int*)surfaceIndexList.GetUnsafePtr();

                // Only use the vertices that we've found in the indices
                var surfaceVerticesCount = 0;
                var surfaceVertices = stackalloc float3[brushVertices.Length];
                var indexRemap = stackalloc int[brushVertices.Length];
                for (int i = 0; i < surfaceIndicesCount; i++)
                {
                    var vertexIndexSrc = surfaceIndices[i];
                    var vertexIndexDst = indexRemap[vertexIndexSrc];
                    if (vertexIndexDst == 0)
                    {
                        vertexIndexDst = surfaceVerticesCount;
                        surfaceVertices[surfaceVerticesCount] = brushVertices[vertexIndexSrc];
                        surfaceVerticesCount++;
                        indexRemap[vertexIndexSrc] = vertexIndexDst + 1;
                    }
                    else
                        vertexIndexDst--;
                    surfaceIndices[i] = vertexIndexDst;
                }

                var vertexHash = math.hash(surfaceVertices, surfaceVerticesCount);
                var indicesHash = math.hash(surfaceIndices, surfaceIndicesCount);
                var geometryHash = math.hash(new uint2(vertexHash, indicesHash));

                var surfaceNormals = stackalloc float3[surfaceVerticesCount];
                {
                    if (interiorCategory == CategoryIndex.ValidReverseAligned || interiorCategory == CategoryIndex.ReverseAligned)
                        normal = -normal;
                    for (int i = 0; i < surfaceVerticesCount; i++)
                        surfaceNormals[i] = normal;
                }
                var normalHash = math.hash(surfaceNormals, surfaceVerticesCount);
                var surfaceUV0 = stackalloc float2[surfaceVerticesCount];
                {
                    for (int v = 0; v < surfaceVerticesCount; v++)
                        surfaceUV0[v] = math.mul(uv0Matrix, new float4(surfaceVertices[v], 1)).xy;
                }
                var uv0Hash = math.hash(surfaceUV0, surfaceVerticesCount);

                builder.Construct(ref surfaceRenderBuffer.indices, surfaceIndices, surfaceIndicesCount);
                builder.Construct(ref surfaceRenderBuffer.vertices, surfaceVertices, surfaceVerticesCount);
                builder.Construct(ref surfaceRenderBuffer.normals, surfaceNormals, surfaceVerticesCount);
                builder.Construct(ref surfaceRenderBuffer.uv0, surfaceUV0, surfaceVerticesCount);

                surfaceRenderBuffer.surfaceHash = math.hash(new uint2(normalHash, uv0Hash));
                surfaceRenderBuffer.geometryHash = geometryHash;
                surfaceRenderBuffer.surfaceLayers = surfaceLayers;
                surfaceRenderBuffer.surfaceIndex = surfaceIndex;
            }

            var brushRenderBuffer = builder.CreateBlobAssetReference<ChiselBrushRenderBuffer>(Allocator.Persistent);

            brushRenderBuffers.TryAdd(brushNodeIndex, brushRenderBuffer);

            // Allocated using Temp, so do not need to dispose
            /*
            builder.Dispose();
            loops.Dispose();
            surfaceIndexList.Dispose();


            context_children.Dispose();
            context_inputEdgesCopy.Dispose();

            context_points.Dispose();
            context_edges.Dispose();

            context_allEdges.Dispose();
            context_sortedPoints.Dispose();
            context_triangles.Dispose();
            context_triangleInterior.Dispose();
            context_advancingFrontNodes.Dispose();
            context_edgeLookupEdges.Dispose();
            context_edgeLookups.Dispose();
            context_foundLoops.Dispose();
            */
        }
    }
}
