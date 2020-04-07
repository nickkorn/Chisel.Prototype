using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    internal struct ChiselSurfaceRenderBuffer
    {
        public BlobArray<Int32>		indices;
        public BlobArray<float3>	vertices;
        public BlobArray<float3>	normals;
        public BlobArray<float2>    uv0;

        public UInt64		    geometryHash;
        public UInt64		    surfaceHash;

        //public MeshQuery	    meshQuery;
        public SurfaceLayers    surfaceLayers;
        //public Int32		    surfaceParameter;
        public Int32		    surfaceIndex;
    };

    internal sealed class Outline
    {
        public Int32[] visibleOuterLines;
        public Int32[] visibleInnerLines;
        public Int32[] visibleTriangles;
        public Int32[] invisibleOuterLines;
        public Int32[] invisibleInnerLines;
        public Int32[] invalidLines;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Reset()
        {
            visibleOuterLines   = new Int32[0];
            visibleInnerLines   = new Int32[0];
            visibleTriangles    = new Int32[0];
            invisibleOuterLines = new Int32[0];
            invisibleInnerLines = new Int32[0];
            invalidLines        = new Int32[0];
        }
    };

    internal sealed class BrushOutline
    {
        public Outline      brushOutline   = new Outline();
        public Outline[]    surfaceOutlines;
        public float3[]     vertices;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Reset()
        {
            brushOutline.Reset();
            surfaceOutlines = new Outline[0];
            vertices = new float3[0];
        }
    };
#endif

    static partial class CSGManager
    {
#if USE_MANAGED_CSG_IMPLEMENTATION
        const int kMaxVertexCount = VertexSoup.kMaxVertexCount;
        internal sealed class ChiselBrushRenderBuffer
        {
            public readonly List<BlobAssetReference<ChiselSurfaceRenderBuffer>> surfaceRenderBuffers = new List<BlobAssetReference<ChiselSurfaceRenderBuffer>>();
        };

        internal sealed class SubMeshCounts
        {
            public MeshQuery meshQuery;
            public int		surfaceIdentifier;

            public int		meshIndex;
            public int		subMeshIndex;
            
            public ulong	surfaceHash;   // used to detect changes in color, normal, tangent or uv (doesn't effect lighting)
            public ulong	geometryHash;  // used to detect changes in vertex positions / indices

            public int		indexCount;
            public int		vertexCount;

            public readonly List<BlobAssetReference<ChiselSurfaceRenderBuffer>> surfaces = new List<BlobAssetReference<ChiselSurfaceRenderBuffer>>();
        };

        private struct MeshID
        {
            public MeshQuery	meshQuery;
            public Int32		surfaceParameter;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool operator ==(MeshID left, MeshID right) { return (left.meshQuery == right.meshQuery && left.surfaceParameter == right.surfaceParameter); }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool operator !=(MeshID left, MeshID right) { return (left.meshQuery != right.meshQuery || left.surfaceParameter != right.surfaceParameter); }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public override bool Equals(object obj) { if (ReferenceEquals(this, obj)) return true; if (!(obj is MeshID)) return false; var other = (MeshID)obj; return other == this; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public override int GetHashCode() { return surfaceParameter.GetHashCode() ^ meshQuery.GetHashCode(); }
        };

        internal sealed class BrushInfo : IDisposable
        {
            public int					    brushMeshInstanceID;
            public UInt64                   brushOutlineGeneration;
            public bool                     brushOutlineDirty = true;
            
            public BrushLoops               brushSurfaceLoops;
            public BrushOutputLoops		    brushOutputLoops	= new BrushOutputLoops();
            public ChiselBrushRenderBuffer  renderBuffers       = new ChiselBrushRenderBuffer();

            public BrushOutline             brushOutline        = new BrushOutline();

            ~BrushInfo() { Dispose(); }

            public void Dispose()
            {
                if (brushSurfaceLoops.IsCreated) brushSurfaceLoops.Dispose();
                brushOutputLoops.Dispose();
                foreach (var buffer in renderBuffers.surfaceRenderBuffers)
                    buffer.Dispose();
                renderBuffers.surfaceRenderBuffers.Clear();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Reset() 
            {
                Dispose();
                brushOutlineDirty = true;
                brushOutlineGeneration  = 0;
                brushOutline.Reset();
                brushOutputLoops.Clear();
            }
        }




        internal static GeneratedMeshContents GetGeneratedMesh(int treeNodeID, GeneratedMeshDescription meshDescription, GeneratedMeshContents previousGeneratedMeshContents)
        {
            if (!AssertNodeIDValid(treeNodeID) || !AssertNodeType(treeNodeID, CSGNodeType.Tree)) return null;
            if (meshDescription.vertexCount <= 0 ||
                meshDescription.indexCount <= 0)
            {
                Debug.LogWarning(string.Format("{0} called with a {1} that isn't valid", typeof(CSGTree).Name, typeof(GeneratedMeshDescription).Name));
                return null;
            }

            var meshIndex		= meshDescription.meshQueryIndex;
            var subMeshIndex	= meshDescription.subMeshQueryIndex;
            if (meshIndex    < 0) { Debug.LogError("GetGeneratedMesh: MeshIndex cannot be negative"); return null; }
            if (subMeshIndex < 0) { Debug.LogError("GetGeneratedMesh: SubMeshIndex cannot be negative"); return null; }

            if (nodeHierarchies[treeNodeID - 1].treeInfo == null) { Debug.LogWarning("Tree has not been initialized properly"); return null;			}

            TreeInfo tree = nodeHierarchies[treeNodeID - 1].treeInfo;
            if (tree == null) { Debug.LogError("GetGeneratedMesh: Invalid node index used"); return null; }
            if (tree.subMeshCounts == null) { Debug.LogWarning("Tree has not been initialized properly"); return null; }
            



            int subMeshCountSize = (int)tree.subMeshCounts.Count;
            if (subMeshIndex >= (int)subMeshCountSize) { Debug.LogError("GetGeneratedMesh: SubMeshIndex is higher than the number of generated meshes"); return null; }
            if (meshIndex    >= (int)subMeshCountSize) { Debug.LogError("GetGeneratedMesh: MeshIndex is higher than the number of generated meshes"); return null; }

            int foundIndex = -1;
            for (int i = 0; i < subMeshCountSize; i++)
            {
                if (meshIndex    == tree.subMeshCounts[i].meshIndex &&
                    subMeshIndex == tree.subMeshCounts[i].subMeshIndex)
                {
                    foundIndex = i;
                    break;
                }
            }
            if (foundIndex < 0 || foundIndex >= subMeshCountSize) { Debug.LogError("GetGeneratedMesh: Could not find mesh associated with MeshIndex/SubMeshIndex pair"); return null; }
            
            var subMeshCount = tree.subMeshCounts[foundIndex];
            if (subMeshCount.indexCount > meshDescription.indexCount) { Debug.LogError("GetGeneratedMesh: The destination indices array (" + meshDescription.indexCount + ") is smaller than the size of the source data (" + (int)subMeshCount.indexCount + ")"); return null; }
            if (subMeshCount.vertexCount > meshDescription.vertexCount) { Debug.LogError("GetGeneratedMesh: The destination vertices array (" + meshDescription.vertexCount + ") is smaller than the size of the source data (" + (int)subMeshCount.vertexCount + ")"); return null; }
            if (subMeshCount.indexCount == 0 || subMeshCount.vertexCount == 0) { Debug.LogWarning("GetGeneratedMesh: Mesh is empty"); return null; }



            var generatedMesh			= (previousGeneratedMeshContents != null) ? previousGeneratedMeshContents : new GeneratedMeshContents();
            var usedVertexChannels		= meshDescription.meshQuery.UsedVertexChannels;
            var vertexCount				= meshDescription.vertexCount;
            var indexCount				= meshDescription.indexCount;
            generatedMesh.description	= meshDescription;
            
            // create our arrays on the managed side with the correct size
            generatedMesh.tangents		= ((usedVertexChannels & VertexChannelFlags.Tangent) == 0) ? null : (generatedMesh.tangents != null && generatedMesh.tangents.Length == vertexCount) ? generatedMesh.tangents : new Vector4[vertexCount];
            generatedMesh.normals		= ((usedVertexChannels & VertexChannelFlags.Normal ) == 0) ? null : (generatedMesh.normals  != null && generatedMesh.normals .Length == vertexCount) ? generatedMesh.normals  : new Vector3[vertexCount];
            generatedMesh.uv0			= ((usedVertexChannels & VertexChannelFlags.UV0    ) == 0) ? null : (generatedMesh.uv0      != null && generatedMesh.uv0     .Length == vertexCount) ? generatedMesh.uv0      : new Vector2[vertexCount];
            generatedMesh.positions		= (generatedMesh.positions != null && generatedMesh.positions .Length == vertexCount) ? generatedMesh.positions : new Vector3[vertexCount];
            generatedMesh.indices		= (generatedMesh.indices   != null && generatedMesh.indices   .Length == indexCount ) ? generatedMesh.indices   : new int    [indexCount ];

            generatedMesh.bounds = new Bounds();		

        
            bool result = CSGManager.GenerateVertexBuffers(subMeshCount, generatedMesh);

            if (!result)
                return null;

            var min = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            var max = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
            for (int i = 0, count = subMeshCount.indexCount; i < count; i++)
            {
                var position = generatedMesh.positions[generatedMesh.indices[i]];
                min.x = Mathf.Min(min.x, position.x);
                min.y = Mathf.Min(min.y, position.y);
                min.z = Mathf.Min(min.z, position.z);

                max.x = Mathf.Max(max.x, position.x);
                max.y = Mathf.Max(max.y, position.y);
                max.z = Mathf.Max(max.z, position.z);
            }

            var boundsCenter = (max + min) * 0.5f;
            var boundsSize	 = (max - min);

            if (float.IsInfinity(boundsSize.x) || float.IsInfinity(boundsSize.y) || float.IsInfinity(boundsSize.z) ||
                float.IsNaN(boundsSize.x) || float.IsNaN(boundsSize.y) || float.IsNaN(boundsSize.z))
                return null;

            generatedMesh.bounds = new Bounds(boundsCenter, boundsSize);
            return generatedMesh;
        }


        unsafe static bool GenerateVertexBuffers(SubMeshCounts subMeshCount, GeneratedMeshContents generatedMesh)
        {
            var submeshVertexCount	= subMeshCount.vertexCount;
            var submeshIndexCount	= subMeshCount.indexCount;
            var subMeshSurfaces		= subMeshCount.surfaces;

            if (subMeshSurfaces == null ||
                submeshVertexCount != generatedMesh.positions.Length ||
                submeshIndexCount  != generatedMesh.indices.Length ||
                generatedMesh.indices == null ||
                generatedMesh.positions == null)
                return false;

            bool needTangents		= generatedMesh.tangents != null && (((Int32)subMeshCount.meshQuery.UsedVertexChannels & (Int32)VertexChannelFlags.Tangent) != 0);
            bool needNormals		= generatedMesh.normals  != null && (((Int32)subMeshCount.meshQuery.UsedVertexChannels & (Int32)VertexChannelFlags.Normal) != 0);
            bool needUV0s			= generatedMesh.uv0      != null && (((Int32)subMeshCount.meshQuery.UsedVertexChannels & (Int32)VertexChannelFlags.UV0) != 0);
            bool needTempNormals	= needTangents && !needNormals;
            bool needTempUV0		= needTangents && !needUV0s;

            var normals	= !needTempNormals ? generatedMesh.normals : new Vector3[submeshVertexCount];
            var uv0s	= !needTempUV0     ? generatedMesh.uv0     : new Vector2[submeshVertexCount];

            // double snap_size = 1.0 / ants.SnapDistance();

            fixed (Vector3* dstVertices = &generatedMesh.positions[0])
            { 
                // copy all the vertices & indices to the sub-meshes for each material
                for (int surfaceIndex = 0, indexOffset = 0, vertexOffset = 0, surfaceCount = (int)subMeshSurfaces.Count;
                     surfaceIndex < surfaceCount;
                     ++surfaceIndex)
                {
                    var sourceBufferRef = subMeshSurfaces[surfaceIndex];
                    ref var sourceBuffer = ref sourceBufferRef.Value;
                    if (sourceBuffer.indices.Length == 0 ||
                        sourceBuffer.vertices.Length == 0)
                        continue;
                    for (int i = 0, sourceIndexCount = sourceBuffer.indices.Length; i < sourceIndexCount; i++)
                    {
                        generatedMesh.indices[indexOffset] = (int)(sourceBuffer.indices[i] + vertexOffset);
                        indexOffset++;
                    }

                    var sourceVertexCount = sourceBuffer.vertices.Length;

                    fixed (float3* srcVertices = &sourceBuffer.vertices[0])
                    {
                        UnsafeUtility.MemCpy(dstVertices + vertexOffset, srcVertices, sourceVertexCount * UnsafeUtility.SizeOf<float3>());
                        //Array.Copy(sourceBuffer.vertices, 0, generatedMesh.positions, vertexOffset, sourceVertexCount);
                    }

                    if (needUV0s || needTangents)
                    {
                        fixed (Vector2* dstUV0 = &generatedMesh.uv0[0])
                        fixed (float2* srcUV0 = &sourceBuffer.uv0[0])
                        {
                            UnsafeUtility.MemCpy(dstUV0 + vertexOffset, srcUV0, sourceVertexCount * UnsafeUtility.SizeOf<float2>());
                        }
                    }
                    if (needNormals || needTangents)
                    {
                        fixed (Vector3* dstNormals = &generatedMesh.normals[0])
                        fixed (float3* srcNormals = &sourceBuffer.normals[0])
                        {
                            UnsafeUtility.MemCpy(dstNormals + vertexOffset, srcNormals, sourceVertexCount * UnsafeUtility.SizeOf<float3>());
                        }
                    }
                    vertexOffset += sourceVertexCount;
                }
            }

            if (needTangents)
            {
                ComputeTangents(generatedMesh.indices,
                                generatedMesh.positions,
                                uv0s,
                                normals,
                                generatedMesh.tangents);
            }
            return true;
        }
        

        static void ComputeTangents(int[]		meshIndices,
                                    Vector3[]	positions,
                                    Vector2[]	uvs,
                                    Vector3[]	normals,
                                    Vector4[]	tangents) 
        {
            if (meshIndices == null ||
                positions == null ||
                uvs == null ||
                tangents == null ||
                meshIndices.Length == 0 ||
                positions.Length == 0)
                return;

            var tangentU = new Vector3[positions.Length];
            var tangentV = new Vector3[positions.Length];

            for (int i = 0; i < meshIndices.Length; i+=3) 
            {
                int i0 = meshIndices[i + 0];
                int i1 = meshIndices[i + 1];
                int i2 = meshIndices[i + 2];

                var v1 = positions[i0];
                var v2 = positions[i1];
                var v3 = positions[i2];
        
                var w1 = uvs[i0];
                var w2 = uvs[i1];
                var w3 = uvs[i2];

                var edge1 = v2 - v1;
                var edge2 = v3 - v1;

                var uv1 = w2 - w1;
                var uv2 = w3 - w1;
        
                var r = 1.0f / (uv1.x * uv2.y - uv1.y * uv2.x);
                if (float.IsNaN(r) || float.IsInfinity(r))
                    r = 0.0f;

                var udir = new Vector3(
                    ((edge1.x * uv2.y) - (edge2.x * uv1.y)) * r,
                    ((edge1.y * uv2.y) - (edge2.y * uv1.y)) * r,
                    ((edge1.z * uv2.y) - (edge2.z * uv1.y)) * r
                );

                var vdir = new Vector3(
                    ((edge1.x * uv2.x) - (edge2.x * uv1.x)) * r,
                    ((edge1.y * uv2.x) - (edge2.y * uv1.x)) * r,
                    ((edge1.z * uv2.x) - (edge2.z * uv1.x)) * r
                );

                tangentU[i0] += udir;
                tangentU[i1] += udir;
                tangentU[i2] += udir;

                tangentV[i0] += vdir;
                tangentV[i1] += vdir;
                tangentV[i2] += vdir;
            }

            for (int i = 0; i < positions.Length; i++) 
            {
                var n	= normals[i];
                var t0	= tangentU[i];
                var t1	= tangentV[i];

                var t = t0 - (n * Vector3.Dot(n, t0));
                t.Normalize();

                var c = Vector3.Cross(n, t0);
                float w = (Vector3.Dot(c, t1) < 0) ? 1.0f : -1.0f;
                tangents[i] = new Vector4(t.x, t.y, t.z, w);
                normals[i] = n;
            }
        }




        internal static GeneratedMeshDescription[] GetMeshDescriptions(Int32                treeNodeID,
                                                                       MeshQuery[]          meshQueries,
                                                                       VertexChannelFlags   vertexChannelMask)
        {
            if (!AssertNodeIDValid(treeNodeID) || !AssertNodeType(treeNodeID, CSGNodeType.Tree)) return null;
            if (meshQueries == null)
                throw new ArgumentNullException("meshTypes");

            if (meshQueries.Length == 0)
            {
                Debug.Log("meshQueries.Length == 0");
                return null;
            }

            if (!IsValidNodeID(treeNodeID))
            {
                Debug.LogError("GenerateMeshDescriptions: Invalid node index used");
                return null;
            }

            var treeNodeIndex = treeNodeID - 1;
            var treeInfo = nodeHierarchies[treeNodeIndex].treeInfo;
            if (treeInfo == null)
            {
                Debug.LogError("GenerateMeshDescriptions: Invalid node index used");
                return null;
            }

            treeInfo.subMeshCounts.Clear();
            treeInfo.meshDescriptions.Clear();

            if (nodeFlags[treeNodeIndex].IsNodeFlagSet(NodeStatusFlags.TreeNeedsUpdate))
            {
                UnityEngine.Profiling.Profiler.BeginSample("UpdateTreeMesh");
                try
                {
                    UpdateTreeMesh(treeNodeID, out JobHandle handle);
                    handle.Complete();
                } finally { UnityEngine.Profiling.Profiler.EndSample(); }
            }

            UnityEngine.Profiling.Profiler.BeginSample("CombineSubMeshes");
            try
            {
                CombineSubMeshes(treeInfo, meshQueries, vertexChannelMask);
            } finally { UnityEngine.Profiling.Profiler.EndSample(); }

            UnityEngine.Profiling.Profiler.BeginSample("CleanTree");
            try
            {
                CleanTree(treeNodeID);
            } finally { UnityEngine.Profiling.Profiler.EndSample(); }


            {
                var flags = nodeFlags[treeNodeIndex];
                flags.UnSetNodeFlag(NodeStatusFlags.TreeMeshNeedsUpdate);
                nodeFlags[treeNodeIndex] = flags;
            }

            if (treeInfo.subMeshCounts.Count <= 0)
                return null;

            for (int i = (int)treeInfo.subMeshCounts.Count - 1; i >= 0; i--)
            {
                var subMesh = treeInfo.subMeshCounts[i];
                var description = new GeneratedMeshDescription
                {
                    meshQuery           = subMesh.meshQuery,
                    surfaceParameter    = subMesh.surfaceIdentifier,
                    meshQueryIndex      = subMesh.meshIndex,
                    subMeshQueryIndex   = subMesh.subMeshIndex,

                    geometryHashValue   = subMesh.geometryHash,
                    surfaceHashValue    = subMesh.surfaceHash,

                    vertexCount         = subMesh.vertexCount,
                    indexCount          = subMesh.indexCount
                };

                treeInfo.meshDescriptions.Add(description);

            }

            if (treeInfo.meshDescriptions == null ||
                treeInfo.meshDescriptions.Count == 0 ||
                treeInfo.meshDescriptions[0].vertexCount <= 0 ||
                treeInfo.meshDescriptions[0].indexCount <= 0)
            {
                return null;
            }

            return treeInfo.meshDescriptions.ToArray();
        }

        static Dictionary<MeshID, int> uniqueMeshDescriptions = new Dictionary<MeshID, int>();
        internal static void CombineSubMeshes(TreeInfo treeInfo,
                                              MeshQuery[] meshQueries,
                                              VertexChannelFlags vertexChannelMask)
        {
            Debug.Log("CombineSubMeshes");
            var subMeshCounts = treeInfo.subMeshCounts;
            subMeshCounts.Clear();

            var treeBrushNodeIDs = treeInfo.treeBrushes;
            var treeBrushNodeCount = (Int32)(treeBrushNodeIDs.Count);
            if (treeBrushNodeCount <= 0)
                return;

            uniqueMeshDescriptions.Clear();
            for (int b = 0, count_b = treeBrushNodeCount; b < count_b; b++)
            {
                var brushNodeID = treeBrushNodeIDs[b];
                
                var nodeIndex = brushNodeID - 1;
                var nodeType = nodeFlags[nodeIndex].nodeType;
                if (nodeType != CSGNodeType.Brush)
                    continue;

                var brushInfo = nodeHierarchies[nodeIndex].brushInfo;
                //var operation_type_bits = GetNodeOperationByIndex(nodeIndex);
                if (brushInfo == null //||
                                        //brush.triangleMesh == null //||
                                        //((int)operation_type_bits & InfiniteBrushBits) == InfiniteBrushBits 
                    )
                    continue;

                GenerateSurfaceRenderBuffers(brushNodeID, //brushOutput.brushSurfaceLoops, 
                                             meshQueries, vertexChannelMask);

                var renderBuffers = brushInfo.renderBuffers;
                if (renderBuffers.surfaceRenderBuffers.Count == 0)
                    continue;

                var surfaceRenderBuffers = renderBuffers.surfaceRenderBuffers;
                for (int j = 0, count_j = (int)renderBuffers.surfaceRenderBuffers.Count; j < count_j; j++)
                {
                    var brushSurfaceBufferRef   = surfaceRenderBuffers[j];
                    ref var brushSurfaceBuffer  = ref brushSurfaceBufferRef.Value;
                    var surfaceVertexCount      = brushSurfaceBuffer.vertices.Length;
                    var surfaceIndexCount       = brushSurfaceBuffer.indices.Length;

                    if (surfaceVertexCount <= 0 || surfaceIndexCount <= 0)
                        continue;


                    ref var surfaceLayers = ref brushSurfaceBuffer.surfaceLayers;

                    for (int t=0;t< meshQueries.Length;t++)
                    { 
                        var meshQuery = meshQueries[t];

                        var core_surface_flags = surfaceLayers.layerUsage;
                        if ((core_surface_flags & meshQuery.LayerQueryMask) != meshQuery.LayerQuery)
                        {
                            continue;
                        }

                        int surfaceParameter = 0;
                        if (meshQuery.LayerParameterIndex >= LayerParameterIndex.LayerParameter1 &&
                            meshQuery.LayerParameterIndex <= LayerParameterIndex.MaxLayerParameterIndex)
                        {
                            // TODO: turn this into array lookup
                            switch (meshQuery.LayerParameterIndex)
                            {
                                case LayerParameterIndex.LayerParameter1: surfaceParameter = surfaceLayers.layerParameter1; break;
                                case LayerParameterIndex.LayerParameter2: surfaceParameter = surfaceLayers.layerParameter2; break;
                                case LayerParameterIndex.LayerParameter3: surfaceParameter = surfaceLayers.layerParameter3; break;
                            }
                        }

                        var meshID = new MeshID() { meshQuery = meshQuery, surfaceParameter = surfaceParameter };
                     
                        if (!uniqueMeshDescriptions.TryGetValue(meshID, out int generatedMeshIndex))
                            generatedMeshIndex = -1;

                        if (generatedMeshIndex == -1 ||
                            (subMeshCounts[generatedMeshIndex].vertexCount + surfaceVertexCount) >= kMaxVertexCount)
                        {
                            int meshIndex, subMeshIndex;
                            if (generatedMeshIndex != -1)
                            {
                                var prevMeshCountIndex = generatedMeshIndex;
                                generatedMeshIndex = (int)subMeshCounts.Count;
                                subMeshIndex = subMeshCounts[prevMeshCountIndex].subMeshIndex + 1;
                                meshIndex = subMeshCounts[prevMeshCountIndex].meshIndex;
                            }
                            else
                            {
                                generatedMeshIndex = (int)subMeshCounts.Count;
                                meshIndex = generatedMeshIndex;
                                subMeshIndex = 0;
                            }

                            uniqueMeshDescriptions[meshID] = generatedMeshIndex;
                            var newSubMesh = new SubMeshCounts
                            {
                                meshIndex           = meshIndex,
                                subMeshIndex        = subMeshIndex,
                                meshQuery           = meshID.meshQuery,
                                surfaceIdentifier   = surfaceParameter,
                                indexCount          = surfaceIndexCount,
                                vertexCount         = surfaceVertexCount,
                                surfaceHash         = brushSurfaceBuffer.surfaceHash,
                                geometryHash        = brushSurfaceBuffer.geometryHash
                            };
                            newSubMesh.surfaces.Add(brushSurfaceBufferRef);
                            subMeshCounts.Add(newSubMesh);
                            continue;
                        }

                        var currentSubMesh = subMeshCounts[generatedMeshIndex];
                        currentSubMesh.indexCount   += surfaceIndexCount;
                        currentSubMesh.vertexCount  += surfaceVertexCount;
                        currentSubMesh.surfaceHash  = Hashing.XXH64_mergeRound(currentSubMesh.surfaceHash, brushSurfaceBuffer.surfaceHash);
                        currentSubMesh.geometryHash = Hashing.XXH64_mergeRound(currentSubMesh.geometryHash, brushSurfaceBuffer.geometryHash);
                        currentSubMesh.surfaces.Add(brushSurfaceBufferRef);
                    }
                }
            }
        }
        

        static readonly Poly2Tri.DTSweep context = new Poly2Tri.DTSweep();
        internal static unsafe void GenerateSurfaceRenderBuffers(int                brushNodeID, 
                                                                 MeshQuery[]        meshQueries,
                                                                 VertexChannelFlags vertexChannelMask)
        {
            var brushNodeIndex      = brushNodeID - 1;
            var output			    = CSGManager.GetBrushInfo(brushNodeID);
            var brushLoops          = output.brushSurfaceLoops;
            if (!brushLoops.IsCreated)
                return;

            var surfaceLoopIndices  = brushLoops.surfaceLoopIndices;
            
            var treeNodeID          = nodeHierarchies[brushNodeIndex].treeNodeID;
            var chiselLookupValues  = ChiselTreeLookup.Value[treeNodeID - 1];

            var brushVertices       = chiselLookupValues.vertexSoups[brushNodeIndex];
            var brushWorldPlanes    = chiselLookupValues.brushWorldPlanes[brushNodeIndex];
            var basePolygonsBlob    = chiselLookupValues.basePolygons[brushNodeIndex];

            var maxLoops = 0;
            var maxIndices = 0;
            for (int s = 0; s < surfaceLoopIndices.Length; s++)
            {
                var length = surfaceLoopIndices[s].Length;
                maxIndices += length;
                maxLoops = math.max(maxLoops, length);
            }

            var outputSurfaces      = new List<BlobAssetReference<ChiselSurfaceRenderBuffer>>(surfaceLoopIndices.Length); // TODO: should be same size as brush.surfaces.Length
            var loops               = new List<NativeListArray<Edge>.NativeList>(maxLoops);
            var loopInfos           = new List<SurfaceInfo>(maxLoops);
            var surfaceIndexList    = new List<int>(maxIndices);
            for (int s = 0; s < surfaceLoopIndices.Length; s++)
            {
                loops.Clear();
                loopInfos.Clear();

                var loopIndices = surfaceLoopIndices[s];
                for (int l = 0; l < loopIndices.Length; l++)
                {
                    var surfaceLoopIndex = loopIndices[l];
                    var surfaceLoopInfo  = brushLoops.allInfos[surfaceLoopIndex];
                    var _interiorCategory = (CategoryIndex)surfaceLoopInfo.interiorCategory;
                    if (_interiorCategory > CategoryIndex.LastCategory)
                        Debug.Assert(false, $"Invalid final category {_interiorCategory}");

                    if (_interiorCategory != CategoryIndex.ValidAligned && 
                        _interiorCategory != CategoryIndex.ValidReverseAligned)
                        continue;

                    var surfaceLoopEdges   = brushLoops.allEdges[surfaceLoopIndex];
                    if (surfaceLoopEdges.Length < 3)
                        continue;

                    loops.Add(surfaceLoopEdges);
                    loopInfos.Add(surfaceLoopInfo);
                }

                // TODO: why are we doing this in tree-space? better to do this in brush-space, then we can more easily cache this
                var surfaceIndex            = s;
                var surfaceLayers           = basePolygonsBlob.Value.surfaces[surfaceIndex].surfaceInfo.layers;
                var surfaceWorldPlane       = brushWorldPlanes.Value.worldPlanes[surfaceIndex];
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

                UnityEngine.Profiling.Profiler.BeginSample("Triangulate");
                for (int l = 0; l < loops.Count; l++)
                {
                    var loopEdges        = loops[l];
                    var loopInfo         = loopInfos[l];
                    interiorCategory = (CategoryIndex)loopInfo.interiorCategory;

                    Debug.Assert(surfaceIndex == loopInfo.basePlaneIndex);
                
                    Int32[] surfaceIndicesArray = null;
                    try
                    {
                        surfaceIndicesArray = context.TriangulateLoops(loopInfo, brushVertices, loopEdges.ToArray().ToList(), rotation);
                    }
                    catch (System.Exception e)
                    {
                        Debug.LogException(e);
                    }

                    if (surfaceIndicesArray == null ||
                        surfaceIndicesArray.Length < 3)
                        continue;

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

                    surfaceIndexList.AddRange(surfaceIndicesArray);
                }
                UnityEngine.Profiling.Profiler.EndSample();

                if (surfaceIndexList.Count == 0)
                    continue;

                var surfaceIndices = surfaceIndexList.ToArray();

                // Only use the vertices that we've found in the indices
                var surfaceVerticesList = new List<float3>(brushVertices.Length);
                var indexRemap      = new int[brushVertices.Length];
                for (int i = 0; i < surfaceIndices.Length; i++)
                {
                    var vertexIndexSrc = surfaceIndices[i];
                    var vertexIndexDst = indexRemap[vertexIndexSrc];
                    if (vertexIndexDst == 0)
                    {
                        vertexIndexDst = surfaceVerticesList.Count;
                        surfaceVerticesList.Add(brushVertices[vertexIndexSrc]);
                        indexRemap[vertexIndexSrc] = vertexIndexDst + 1;
                    } else
                        vertexIndexDst--;
                    surfaceIndices[i] = vertexIndexDst;
                }

                float3[] surfaceVertices = surfaceVerticesList.ToArray();
                
                var anySurfaceTargetHasNormals  = true; // TODO: actually determine this
                var anySurfaceTargetHasUVs      = true; // TODO: actually determine this

                var vertexHash	    = (ulong)Hashing.ComputeHashKey(surfaceVertices);
                var indicesHash	    = (ulong)Hashing.ComputeHashKey(surfaceIndices);
                var geometryHash    = Hashing.XXH64_mergeRound(vertexHash, indicesHash);

                var normalHash  = (ulong)0;
                float3[] surfaceNormals = null;
                if (anySurfaceTargetHasNormals)
                {
                    var normal = (interiorCategory == CategoryIndex.ValidReverseAligned || interiorCategory == CategoryIndex.ReverseAligned) ? -surfaceWorldPlane.xyz : surfaceWorldPlane.xyz;

                    surfaceNormals = surfaceVertices == null ? null : new float3[surfaceVertices.Length];
                    if (surfaceVertices != null)
                    {
                        for (int i = 0; i < surfaceVertices.Length; i++)
                            surfaceNormals[i] = normal;
                    }
                    normalHash = Hashing.ComputeHashKey(surfaceNormals);
                }
                var uv0Hash = (ulong)0;
                float2[] surfaceUV0 = null;
                if (anySurfaceTargetHasUVs)
                {
                    surfaceUV0 = surfaceVertices == null ? null : new float2[surfaceVertices.Length];
                    if (surfaceVertices != null)
                    {
                        for (int v = 0; v < surfaceVertices.Length; v++)
                            surfaceUV0[v] = math.mul(uv0Matrix, new float4(surfaceVertices[v], 1)).xy;
                    }
                    uv0Hash = Hashing.ComputeHashKey(surfaceUV0);
                }

                /*
                for (int t = 0, tSize = meshQueries.Length; t < tSize; t++)
                {
                    var meshQuery = meshQueries[t];

                    var core_surface_flags = surfaceLayers.layerUsage;
                    if ((core_surface_flags & meshQuery.LayerQueryMask) != meshQuery.LayerQuery)
                    {
                        continue;
                    }
                    
                    int surfaceParameter = 0;
                    if (meshQuery.LayerParameterIndex >= LayerParameterIndex.LayerParameter1 &&
                        meshQuery.LayerParameterIndex <= LayerParameterIndex.MaxLayerParameterIndex)
                    {
                        // TODO: turn this into array lookup
                        switch (meshQuery.LayerParameterIndex)
                        {
                            case LayerParameterIndex.LayerParameter1: surfaceParameter = surfaceLayers.layerParameter1; break;
                            case LayerParameterIndex.LayerParameter2: surfaceParameter = surfaceLayers.layerParameter2; break;
                            case LayerParameterIndex.LayerParameter3: surfaceParameter = surfaceLayers.layerParameter3; break;
                        }
                    }
                    */
                    var haveUV0     = false; // TODO: actually determine this
                    var haveNormal  = false; // TODO: actually determine this

                    var builder = new BlobBuilder(Allocator.TempJob);
                    ref var root = ref builder.ConstructRoot<ChiselSurfaceRenderBuffer>();
                    builder.Construct(ref root.indices,     surfaceIndices);
                    builder.Construct(ref root.vertices,    surfaceVertices);
                    builder.Construct(ref root.normals,     surfaceNormals);
                    builder.Construct(ref root.uv0,         surfaceUV0);

                    root.surfaceHash        = Hashing.XXH64_mergeRound(!haveNormal ? 0 : normalHash, !haveUV0 ? 0 : uv0Hash);
                    root.geometryHash       = geometryHash;
                    //root.meshQuery	        = meshQuery;
                    root.surfaceLayers      = surfaceLayers;
                    //root.surfaceParameter   = surfaceParameter;
                    root.surfaceIndex       = surfaceIndex;
                    var surfaceRenderBuffer = builder.CreateBlobAssetReference<ChiselSurfaceRenderBuffer>(Allocator.Persistent);
                    builder.Dispose();

                    outputSurfaces.Add(surfaceRenderBuffer);
                //}
            }
            output.renderBuffers.surfaceRenderBuffers.Clear();
            output.renderBuffers.surfaceRenderBuffers.AddRange(outputSurfaces);
        }
        

        private static void UpdateDelayedHierarchyModifications()
        {
            for (var i = 0; i < branches.Count; i++)
            {
                var branchNodeID = branches[i];
                var branchNodeIndex = branchNodeID - 1;

                {
                    var flags = nodeFlags[branchNodeIndex];
                    flags.UnSetNodeFlag(NodeStatusFlags.OperationNeedsUpdate);
                    nodeFlags[branchNodeIndex] = flags;
                }
                if (!nodeFlags[branchNodeIndex].IsNodeFlagSet(NodeStatusFlags.NeedPreviousSiblingsUpdate))
                    continue;

                // TODO: implement
                //operation.RebuildPreviousSiblings();
                {
                    var flags = nodeFlags[branchNodeIndex];
                    flags.UnSetNodeFlag(NodeStatusFlags.NeedPreviousSiblingsUpdate);
                    nodeFlags[branchNodeIndex] = flags;
                }
            }

            // TODO: implement
            /*
            var foundOperations = new List<int>();
            for (var i = 0; i < branches.Count; i++)
            {
                var branchNodeID = branches[i];
                var branchNodeIndex = branchNodeID - 1;
                if (!nodeFlags[branchNodeIndex].IsNodeFlagSet(NodeStatusFlags.NeedAllTouchingUpdated))
                    continue;

                foundOperations.Add(branchNodeIndex);
            }

            for (int i = 0; i < foundOperations.Count; i++)
            {
                //UpdateChildOperationTouching(foundOperations[i]);
            }
            */

            for (var i = 0; i < branches.Count; i++)
            {
                var branchNodeID = branches[i];
                var branchNodeIndex = branchNodeID - 1;
                if (!nodeFlags[branchNodeIndex].IsNodeFlagSet(NodeStatusFlags.NeedAllTouchingUpdated))
                    continue;

                // TODO: implement
                //UpdateChildBrushTouching(branchNodeID);
                {
                    var flags = nodeFlags[branchNodeIndex];
                    flags.UnSetNodeFlag(NodeStatusFlags.NeedAllTouchingUpdated);
                    nodeFlags[branchNodeIndex] = flags;
                }
            }
        }
#endif
    }
}