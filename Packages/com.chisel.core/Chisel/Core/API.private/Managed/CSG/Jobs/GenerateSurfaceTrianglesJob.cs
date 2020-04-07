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
    public struct SurfaceTrianglesBlob
    {
        public BlobArray<float3>    vertices;
        public BlobArray<int>	    indices;        
    }
    /*
    [BurstCompile(CompileSynchronously = true)]
    unsafe struct GenerateSurfaceTrianglesJob : IJob
    {
        [NoAlias, ReadOnly] public NativeListArray<Edge>.NativeList loopEdges;
        [NoAlias, ReadOnly] public SurfaceInfo                      loopInfo;
        [NoAlias, ReadOnly] public SurfaceDescription               surfaceDescription;
        [NoAlias, ReadOnly] public Matrix4x4                        worldToLocal;
        [NoAlias, ReadOnly] public float4[]                         meshPlanes;
        [NoAlias, ReadOnly] public VertexSoup                       brushVertices;

        public void Execute()
        {
            var interiorCategory    = (CategoryIndex)loopInfo.interiorCategory;
            var surfaceIndex        = loopInfo.basePlaneIndex;

            // TODO: why are we doing this in tree-space? better to do this in brush-space, then we can more easily cache this
            var localSpaceToPlaneSpace  = MathExtensions.GenerateLocalToPlaneSpaceMatrix(meshPlanes[surfaceIndex]);
            var uv0Matrix               = surfaceDescription.UV0.ToMatrix() * (localSpaceToPlaneSpace * worldToLocal);

            // Ensure we have the rotation properly calculated, and have a valid normal
            quaternion rotation;
            if (((Vector3)loopInfo.worldPlane.xyz) == Vector3.forward)
                rotation = quaternion.identity;
            else
                rotation = (quaternion)Quaternion.FromToRotation(loopInfo.worldPlane.xyz, Vector3.forward);

            // TODO: all separate loops on same surface should be put in same OutputSurfaceMesh!                    

            var context         = new Poly2Tri.DTSweep();
            var surfaceIndices  = context.TriangulateLoops(loopInfo, brushVertices, loopEdges.ToArray().ToList(), rotation);

            if (surfaceIndices == null ||
                surfaceIndices.Length < 3)
                return;

            if (interiorCategory == CategoryIndex.ValidReverseAligned ||
                interiorCategory == CategoryIndex.ReverseAligned)
            {
                var maxCount = surfaceIndices.Length - 1;
                for (int n = (maxCount / 2); n >= 0; n--)
                {
                    var t = surfaceIndices[n];
                    surfaceIndices[n] = surfaceIndices[maxCount - n];
                    surfaceIndices[maxCount - n] = t;
                }
            }

            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<SurfaceTrianglesBlob>();

            // TODO: only use the vertices that we found in the indices (we're using too many vertices!)
            var surfaceVertices = builder.Allocate(ref root.vertices, brushVertices.Length);
            var vertices = brushVertices.GetUnsafeReadOnlyPtr();
            for (int v = 0; v < brushVertices.Length; v++)
                surfaceVertices[v] = vertices[v];
            builder.Construct(ref root.indices, surfaceIndices);

            var surfaceTrianglesBlob = builder.CreateBlobAssetReference<SurfaceTrianglesBlob>(Allocator.Persistent);
            builder.Dispose();
        }
    }*/
}
