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
    //[BurstCompile(CompileSynchronously = true)] // Fails for some reason
    unsafe struct GenerateSurfaceTrianglesJob : IJob
    {
        [NoAlias, ReadOnly] public int                      brushNodeIndex;
        [NoAlias, ReadOnly] public VertexSoup               brushVertices;
        [NoAlias, ReadOnly] public NativeListArray<int>     surfaceLoopIndices;
        [NoAlias, ReadOnly] public NativeList<SurfaceInfo>  surfaceLoopAllInfos;
        [NoAlias, ReadOnly] public NativeListArray<Edge>    surfaceLoopAllEdges;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>> basePolygons;
        [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>> brushWorldPlanes;

        [NoAlias, WriteOnly] public NativeList<BlobAssetReference<ChiselSurfaceRenderBuffer>>   surfaceRenderBuffers;


        [BurstDiscard]
        public static void InvalidFinalCategory(CategoryIndex _interiorCategory)
        {
            Debug.Assert(false, $"Invalid final category {_interiorCategory}");
        }

        public void Execute()
        {
            var brushNodeID = brushNodeIndex + 1;

            var maxLoops = 0;
            var maxIndices = 0;
            for (int s = 0; s < surfaceLoopIndices.Length; s++)
            {
                var length = surfaceLoopIndices[s].Length;
                maxIndices += length;
                maxLoops = math.max(maxLoops, length);
            }


            var basePolygonsBlob        = basePolygons[brushNodeIndex];
            var brushWorldPlanesBlob    = brushWorldPlanes[brushNodeIndex];

            surfaceRenderBuffers.Capacity = surfaceLoopIndices.Length;

            var pointCount = brushVertices.Length + 2;
            var context_points              = new NativeArray<float2>(pointCount, Allocator.Temp);
            var context_edges               = new NativeArray<ushort>(pointCount, Allocator.Temp);
            var context_allEdges            = new NativeList<Poly2Tri.DTSweep.DirectedEdge>(pointCount, Allocator.Temp);
            var context_sortedPoints        = new NativeList<ushort>(pointCount, Allocator.Temp);
            var context_triangles           = new NativeList<Poly2Tri.DTSweep.DelaunayTriangle>(pointCount * 3, Allocator.Temp);
            var context_triangleInterior    = new NativeList<bool>(pointCount * 3, Allocator.Temp);
            var context_advancingFrontNodes = new NativeList<Poly2Tri.DTSweep.AdvancingFrontNode>(pointCount, Allocator.Temp);
            var context_edgeLookupEdges     = new NativeListArray<Chisel.Core.Edge>(pointCount, Allocator.Temp);
            var context_edgeLookups         = new NativeHashMap<ushort, int>(pointCount, Allocator.Temp);
            var context_foundLoops          = new NativeListArray<Chisel.Core.Edge>(pointCount, Allocator.Temp);

            var context_children            = new NativeListArray<int>(64, Allocator.Temp);
            var context_inputEdgesCopy      = new NativeList<Edge>(64, Allocator.Temp);

            var context = new Poly2Tri.DTSweep
            {
                vertices                = brushVertices,
                points                  = context_points,
                edges                   = context_edges,
                allEdges                = context_allEdges,
                sortedPoints            = context_sortedPoints,
                triangles               = context_triangles,
                triangleInterior        = context_triangleInterior,
                advancingFrontNodes     = context_advancingFrontNodes,
                edgeLookupEdges         = context_edgeLookupEdges,
                edgeLookups             = context_edgeLookups,
                foundLoops              = context_foundLoops,
                children                = context_children,
                inputEdgesCopy          = context_inputEdgesCopy,
            };

            var loops               = new NativeList<int>(maxLoops, Allocator.Temp);
            var surfaceIndexList    = new NativeList<int>(maxIndices, Allocator.Temp);
            for (int s = 0; s < surfaceLoopIndices.Length; s++)
            {
                loops.Clear();

                var loopIndices = surfaceLoopIndices[s];
                for (int l = 0; l < loopIndices.Length; l++)
                {
                    var surfaceLoopIndex = loopIndices[l];
                    var surfaceLoopInfo  = surfaceLoopAllInfos[surfaceLoopIndex];
                    var _interiorCategory = (CategoryIndex)surfaceLoopInfo.interiorCategory;
                    if (_interiorCategory > CategoryIndex.LastCategory)
                        InvalidFinalCategory(_interiorCategory);

                    if (_interiorCategory != CategoryIndex.ValidAligned && 
                        _interiorCategory != CategoryIndex.ValidReverseAligned)
                        continue;

                    var surfaceLoopEdges   = surfaceLoopAllEdges[surfaceLoopIndex];
                    if (surfaceLoopEdges.Length < 3)
                        continue;

                    loops.Add(surfaceLoopIndex);
                }

                // TODO: why are we doing this in tree-space? better to do this in brush-space, then we can more easily cache this
                var surfaceIndex            = s;
                var surfaceLayers           = basePolygonsBlob.Value.surfaces[surfaceIndex].surfaceInfo.layers;
                var surfaceWorldPlane       = brushWorldPlanesBlob.Value.worldPlanes[surfaceIndex];
                var UV0		                = basePolygonsBlob.Value.surfaces[surfaceIndex].UV0;
                var localSpaceToPlaneSpace	= MathExtensions.GenerateLocalToPlaneSpaceMatrix(surfaceWorldPlane);
                var uv0Matrix				= math.mul(UV0.ToFloat4x4(), localSpaceToPlaneSpace);

                // Ensure we have the rotation properly calculated, and have a valid normal
                quaternion rotation;
                if (((Vector3)surfaceWorldPlane.xyz) == Vector3.forward)
                    rotation = quaternion.identity;
                else
                    rotation = (quaternion)Quaternion.FromToRotation(surfaceWorldPlane.xyz, Vector3.forward);


                surfaceIndexList.Clear();

                CategoryIndex   interiorCategory    = CategoryIndex.ValidAligned;

                for (int l = 0; l < loops.Length; l++)
                {
                    var loopIndex   = loops[l];
                    var loopEdges   = surfaceLoopAllEdges[loopIndex];
                    var loopInfo    = surfaceLoopAllInfos[loopIndex];
                    interiorCategory = (CategoryIndex)loopInfo.interiorCategory;

                    Debug.Assert(surfaceIndex == loopInfo.basePlaneIndex);



                    var surfaceIndicesArray = new NativeList<int>(Allocator.Temp);
                    context.TriangulateLoops(loopInfo, rotation, loopEdges, surfaceIndicesArray);



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
                var surfaceIndices      = (int*)surfaceIndexList.GetUnsafePtr();

                // Only use the vertices that we've found in the indices
                var surfaceVerticesCount    = 0;
                var surfaceVertices         = stackalloc float3[brushVertices.Length];
                var indexRemap              = stackalloc int[brushVertices.Length];
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
                    } else
                        vertexIndexDst--;
                    surfaceIndices[i] = vertexIndexDst;
                }

                var vertexHash	    = math.hash(surfaceVertices, surfaceVerticesCount);
                var indicesHash	    = math.hash(surfaceIndices, surfaceIndicesCount);
                var geometryHash    = math.hash(new uint2(vertexHash, indicesHash));

                var surfaceNormals = stackalloc float3[surfaceVerticesCount];
                {
                    var normal = (interiorCategory == CategoryIndex.ValidReverseAligned || interiorCategory == CategoryIndex.ReverseAligned) ? -surfaceWorldPlane.xyz : surfaceWorldPlane.xyz;
                    for (int i = 0; i < surfaceVerticesCount; i++)
                        surfaceNormals[i] = normal;
                }
                var normalHash = math.hash(surfaceNormals, surfaceVerticesCount);
                var surfaceUV0  = stackalloc float2[surfaceVerticesCount];
                {
                    for (int v = 0; v < surfaceVerticesCount; v++)
                        surfaceUV0[v] = math.mul(uv0Matrix, new float4(surfaceVertices[v], 1)).xy;
                }
                var uv0Hash = math.hash(surfaceUV0, surfaceVerticesCount);

                var builder = new BlobBuilder(Allocator.Temp);
                ref var root = ref builder.ConstructRoot<ChiselSurfaceRenderBuffer>();
                builder.Construct(ref root.indices,     surfaceIndices,     surfaceIndicesCount);
                builder.Construct(ref root.vertices,    surfaceVertices,    surfaceVerticesCount);
                builder.Construct(ref root.normals,     surfaceNormals,     surfaceVerticesCount);
                builder.Construct(ref root.uv0,         surfaceUV0,         surfaceVerticesCount);

                root.surfaceHash        = math.hash(new uint2(normalHash, uv0Hash));
                root.geometryHash       = geometryHash;
                root.surfaceLayers      = surfaceLayers;
                root.surfaceIndex       = surfaceIndex;
                var surfaceRenderBuffer = builder.CreateBlobAssetReference<ChiselSurfaceRenderBuffer>(Allocator.Persistent);
                builder.Dispose();

                surfaceRenderBuffers.Add(surfaceRenderBuffer);
            }
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
        }
    }
}
