using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    internal struct ChiselSurfaceRenderBuffer
    {
        public Int32[]		indices;
        public float3[]	    vertices;
        public Vector3[]	normals;
        public Vector2[]	uv0;

        public UInt64		geometryHash;
        public UInt64		surfaceHash;

        public MeshQuery	meshQuery;
        public Int32		surfaceParameter;
        public Int32		surfaceIndex;
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
    public enum IntersectionType
    {
        NoIntersection,
        Intersection,
        AInsideB,
        BInsideA,

        InvalidValue
    };

    public struct BrushBrushIntersection : IComparable<BrushBrushIntersection>, IComparer<BrushBrushIntersection>, IEqualityComparer<BrushBrushIntersection>, IEquatable<BrushBrushIntersection>
    {
        public int brushNodeID0;
        public int brushNodeID1;
        public IntersectionType type;

        #region Equals
        public override bool Equals(object obj)
        {
            if (obj == null || !(obj is BrushBrushIntersection))
                return false;

            var other = (BrushBrushIntersection)obj;
            return  ((brushNodeID0 == other.brushNodeID0) && (brushNodeID1 == other.brushNodeID1)) ||
                    ((brushNodeID0 == other.brushNodeID1) && (brushNodeID1 == other.brushNodeID0));
        }

        public bool Equals(BrushBrushIntersection x, BrushBrushIntersection y)
        {
            return ((x.brushNodeID0 == y.brushNodeID0) && (x.brushNodeID1 == y.brushNodeID1)) ||
                    ((x.brushNodeID0 == y.brushNodeID1) && (x.brushNodeID1 == y.brushNodeID0));
        }

        public bool Equals(BrushBrushIntersection other)
        {
            return ((brushNodeID0 == other.brushNodeID0) && (brushNodeID1 == other.brushNodeID1)) ||
                   ((brushNodeID0 == other.brushNodeID1) && (brushNodeID1 == other.brushNodeID0));
        }
        #endregion

        #region Compare
        public int Compare(BrushBrushIntersection x, BrushBrushIntersection y)
        {
            if (x.brushNodeID0 < y.brushNodeID0)
                return -1;
            if (x.brushNodeID0 > y.brushNodeID0)
                return 1;
            if (x.brushNodeID1 < y.brushNodeID1)
                return -1;
            if (x.brushNodeID1 > y.brushNodeID1)
                return 1;
            if (x.type < y.type)
                return -1;
            if (x.type > y.type)
                return 1;
            return 0;
        }
        public int CompareTo(BrushBrushIntersection other)
        {
            if (brushNodeID0 < other.brushNodeID0)
                return -1;
            if (brushNodeID0 > other.brushNodeID0)
                return 1;
            if (brushNodeID1 < other.brushNodeID1)
                return -1;
            if (brushNodeID1 > other.brushNodeID1)
                return 1;
            if (type < other.type)
                return -1;
            if (type > other.type)
                return 1;
            return 0;
        }
        #endregion

        #region GetHashCode
        public override int GetHashCode()
        {
            return GetHashCode(this);
        }

        public int GetHashCode(BrushBrushIntersection obj)
        {
            if (obj.brushNodeID0 < obj.brushNodeID1)
                return ((ulong)obj.brushNodeID0 + ((ulong)obj.brushNodeID1 << 32)).GetHashCode();
            else
                return ((ulong)obj.brushNodeID1 + ((ulong)obj.brushNodeID0 << 32)).GetHashCode();
        }
        #endregion
    }

    static partial class CSGManager
    {
#if USE_MANAGED_CSG_IMPLEMENTATION
        const int kMaxVertexCount = 65000;
        internal sealed class ChiselBrushRenderBuffer
        {
            public readonly List<ChiselSurfaceRenderBuffer> surfaceRenderBuffers = new List<ChiselSurfaceRenderBuffer>();
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

            public readonly List<ChiselSurfaceRenderBuffer> surfaces = new List<ChiselSurfaceRenderBuffer>();
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
            
            public SurfaceLoops             brushSurfaceLoops;
            public BrushLoops			    brushOutputLoops	= new BrushLoops();
            public ChiselBrushRenderBuffer  renderBuffers       = new ChiselBrushRenderBuffer();

            public BrushOutline             brushOutline        = new BrushOutline();


            public readonly List<BrushBrushIntersection> brushBrushIntersections = new List<BrushBrushIntersection>();

            public readonly RoutingTable    routingTable        = new RoutingTable();

            public void Dispose()
            {
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Reset()
            {
                brushOutlineGeneration  = 0;
                brushOutline.Reset();
                if (brushSurfaceLoops != null)
                    brushSurfaceLoops.Clear();
                brushOutputLoops.Clear();
                renderBuffers.surfaceRenderBuffers.Clear();
                brushBrushIntersections.Clear();
                routingTable.Clear();
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

            fixed(Vector3* dstVertices = &generatedMesh.positions[0])
            { 
                // copy all the vertices & indices to the sub-meshes for each material
                for (int surfaceIndex = 0, indexOffset = 0, vertexOffset = 0, surfaceCount = (int)subMeshSurfaces.Count;
                     surfaceIndex < surfaceCount;
                     ++surfaceIndex)
                {
                    var sourceBuffer = subMeshSurfaces[surfaceIndex];
                    if (sourceBuffer.indices == null ||
                        sourceBuffer.vertices == null ||
                        sourceBuffer.indices.Length == 0 ||
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

                    if (needUV0s    || needTangents) Array.Copy(sourceBuffer.uv0,     0, uv0s,    vertexOffset, sourceVertexCount);
                    if (needNormals || needTangents) Array.Copy(sourceBuffer.normals, 0, normals, vertexOffset, sourceVertexCount);
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




        internal static GeneratedMeshDescription[] GetMeshDescriptions(Int32 treeNodeID,
                                                                       MeshQuery[] meshQueries,
                                                                       VertexChannelFlags vertexChannelMask)
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
                    UpdateTreeMesh(treeNodeID);
                }
                finally
                {
                    UnityEngine.Profiling.Profiler.EndSample();
                }
            }

            UnityEngine.Profiling.Profiler.BeginSample("CombineSubMeshes");
            try
            {
                CombineSubMeshes(treeInfo, meshQueries, vertexChannelMask);
            }
            finally
            {
                UnityEngine.Profiling.Profiler.EndSample();
            }

            UnityEngine.Profiling.Profiler.BeginSample("Clean");
            try
            {
                CleanTree(treeNodeID);
            }
            finally
            {
                UnityEngine.Profiling.Profiler.EndSample();
            }


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

                var brushOutput = nodeHierarchies[nodeIndex].brushInfo;
                //var operation_type_bits = GetNodeOperationByIndex(nodeIndex);
                if (brushOutput == null //||
                                        //brush.triangleMesh == null //||
                                        //((int)operation_type_bits & InfiniteBrushBits) == InfiniteBrushBits 
                    )
                    continue;

                GenerateSurfaceRenderBuffers(brushNodeID, //brushOutput.brushSurfaceLoops, 
                                             meshQueries, vertexChannelMask);

                var renderBuffers = brushOutput.renderBuffers;
                if (renderBuffers.surfaceRenderBuffers.Count == 0)
                    continue;

                var surfaceRenderBuffers = renderBuffers.surfaceRenderBuffers;
                for (int j = 0, count_j = (int)renderBuffers.surfaceRenderBuffers.Count; j < count_j; j++)
                {
                    var brushSurfaceBuffer  = surfaceRenderBuffers[j];
                    var surfaceVertexCount  = (brushSurfaceBuffer.vertices == null) ? 0 : brushSurfaceBuffer.vertices.Length;
                    var surfaceIndexCount   = (brushSurfaceBuffer.indices  == null) ? 0 : brushSurfaceBuffer.indices.Length;

                    if (surfaceVertexCount <= 0 || surfaceIndexCount <= 0)
                        continue;

                    var surfaceParameter = (brushSurfaceBuffer.meshQuery.LayerParameterIndex == LayerParameterIndex.None) ? 0 : brushSurfaceBuffer.surfaceParameter;
                    var meshID = new MeshID() { meshQuery = brushSurfaceBuffer.meshQuery, surfaceParameter = surfaceParameter };
                     
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
                            meshIndex = meshIndex,
                            subMeshIndex = subMeshIndex,
                            meshQuery = meshID.meshQuery,
                            surfaceIdentifier = surfaceParameter,
                            indexCount = surfaceIndexCount,
                            vertexCount = surfaceVertexCount,
                            surfaceHash = brushSurfaceBuffer.surfaceHash,
                            geometryHash = brushSurfaceBuffer.geometryHash
                        };
                        newSubMesh.surfaces.Add(brushSurfaceBuffer);
                        subMeshCounts.Add(newSubMesh);
                        continue;
                    }

                    var currentSubMesh = subMeshCounts[generatedMeshIndex];
                    currentSubMesh.indexCount += surfaceIndexCount;
                    currentSubMesh.vertexCount += surfaceVertexCount;
                    currentSubMesh.surfaceHash = Hashing.XXH64_mergeRound(currentSubMesh.surfaceHash, brushSurfaceBuffer.surfaceHash);
                    currentSubMesh.geometryHash = Hashing.XXH64_mergeRound(currentSubMesh.geometryHash, brushSurfaceBuffer.geometryHash);
                    currentSubMesh.surfaces.Add(brushSurfaceBuffer);
                }
            }
        }
        

        static readonly Poly2Tri.DTSweep context = new Poly2Tri.DTSweep();
        internal static void GenerateSurfaceRenderBuffers(int                   brushNodeID, 
                                                          //SurfaceLoops        loopList, 
                                                          MeshQuery[]           meshQueries,
                                                          VertexChannelFlags    vertexChannelMask)
        {
            var output			= CSGManager.GetBrushInfo(brushNodeID);
            var brushInstance	= CSGManager.GetBrushMeshID(brushNodeID);
            if (!BrushMeshManager.IsBrushMeshIDValid(brushInstance))
            {
                output.renderBuffers.surfaceRenderBuffers.Clear();
                return;
            }

            var mesh			= BrushMeshManager.GetBrushMesh(brushInstance);
            if (mesh == null)
            {
                output.renderBuffers.surfaceRenderBuffers.Clear();
                return;
            }

            var meshPolygons	= mesh.polygons;
            var meshPlanes	    = mesh.planes;
            var brushVertices   = output.brushOutputLoops.vertexSoup;

            CSGManager.GetTreeToNodeSpaceMatrix(brushNodeID, out Matrix4x4 worldToLocal);

            var surfaceLoops = output.brushSurfaceLoops.surfaces;
            var maxLoops = 0;
            for (int s = 0; s < surfaceLoops.Length; s++)
                maxLoops += surfaceLoops[s].Count;

            var loops = new List<Loop>(maxLoops);
            for (int s = 0; s < surfaceLoops.Length; s++)
            {
                var surfaceLoopList = surfaceLoops[s];
                for (int l = 0; l < surfaceLoopList.Count; l++)
                {
                    var interiorCategory = (CategoryIndex)surfaceLoopList[l].info.interiorCategory;
                    if (interiorCategory > CategoryIndex.LastCategory)
                        Debug.Assert(false, $"Invalid final category {interiorCategory}");

                    //*
                    if (interiorCategory != CategoryIndex.ValidAligned && 
                        interiorCategory != CategoryIndex.ValidReverseAligned)
                        continue;
                    /*/

                    if (brushNodeID != 1)// || s!=5)
                        continue;

                    
                    #if true
                    if (interiorCategory == CategoryIndex.ValidReverseAligned)
                        surfaceLoopList[l].interiorCategory = (CategoryGroupIndex)CategoryIndex.ValidAligned;
                    if (interiorCategory == CategoryIndex.ReverseAligned)
                        surfaceLoopList[l].interiorCategory = (CategoryGroupIndex)CategoryIndex.Aligned;
                    #endif
                    
                    #if false
                    var builder = new System.Text.StringBuilder();
                    //builder.AppendLine($"{surfaceLoopList[l].loopIndex}: {s}/{l}/{surfaceLoopList[l].info.worldPlane}");
                    builder.AppendLine($"{s}/{l}/{interiorCategory}/{surfaceLoopList[l].indices.Count}/{surfaceLoopList[l].edges.Count}");
                    CSGManagerPerformCSG.Dump(builder, surfaceLoopList[l], brushVertices, Quaternion.FromToRotation(surfaceLoopList[l].info.worldPlane.normal, Vector3.forward));
                    Debug.Log(builder.ToString());
                    #endif
                    //*/

                    //if (brushNodeID != 3)// || s!=5)
                    //    continue;

                    var loop = surfaceLoopList[l];
                    if (loop.edges.Count < 3)
                        continue;

                    loops.Add(loop);
                }
            }


            var outputSurfaces  = new List<ChiselSurfaceRenderBuffer>(loops.Count * meshQueries.Length); // TODO: should be same size as brush.surfaces.Length
            for (int l = 0; l < loops.Count; l++)
            {
                var loop            = loops[l];
                var interiorCategory = (CategoryIndex)loop.info.interiorCategory;


                var info            = loop.info;
                var polygonIndex	= info.basePlaneIndex;      // TODO: fix this
                var meshPolygon		= meshPolygons[polygonIndex];
                var surfaceIndex	= meshPolygon.surfaceID;    // TODO: fix this

                // TODO: why are we doing this in tree-space? better to do this in brush-space, then we can more easily cache this
                var localSpaceToPlaneSpace	= MathExtensions.GenerateLocalToPlaneSpaceMatrix(meshPlanes[surfaceIndex]);
                var uv0Matrix				= meshPolygon.surface.surfaceDescription.UV0.ToMatrix() * (localSpaceToPlaneSpace * worldToLocal);

                var anySurfaceTargetHasNormals  = true; // TODO: actually determine this
                var anySurfaceTargetHasUVs      = true; // TODO: actually determine this

                
                Int32[] surfaceIndices = null;

                UnityEngine.Profiling.Profiler.BeginSample("Triangulate");
                try
                {
                    // Ensure we have the rotation properly calculated, and have a valid normal
                    quaternion rotation;
                    if (((Vector3)info.worldPlane.xyz) == Vector3.forward)
                        rotation = quaternion.identity;
                    else
                        rotation = (quaternion)Quaternion.FromToRotation(info.worldPlane.xyz, Vector3.forward);

                    // TODO: all separate loops on same surface should be put in same OutputSurfaceMesh!                    

                    surfaceIndices = context.TriangulateLoops(loop, brushVertices.vertices, loop.edges, rotation);

                    
                    #if false
                    var builder = new System.Text.StringBuilder();
                    //builder.AppendLine($"{surfaceLoopList[l].loopIndex}: {s}/{l}/{surfaceLoopList[l].info.worldPlane}");
                    builder.AppendLine($"{loop.info.basePlaneIndex}/{l}/{interiorCategory}/{loop.indices.Count}/{loop.edges.Count}");
                    CSGManagerPerformCSG.Dump(builder, loop, brushVertices, Quaternion.FromToRotation(loop.info.worldPlane.normal, Vector3.forward));
                    for (int i = 0; i < surfaceIndices.Length; i++)
                    {
                        builder.Append(i);
                        builder.Append(", ");
                    }
                    builder.AppendLine();
                    Debug.Log(builder.ToString());
                    #endif
                }
                catch (System.Exception e)
                {
                    Debug.LogException(e);
                    //Debug.Log($"BrushNodeID: {loop.info.brush.brushNodeID} / BasePlaneIndex: {loop.info.basePlaneIndex} / WorldPlane: {loop.info.worldPlane}");// / LoopIndex: {loop.loopIndex}");
                }
                finally
                {
                    UnityEngine.Profiling.Profiler.EndSample();
                }

                if (surfaceIndices == null ||
                    surfaceIndices.Length < 3)
                    continue;

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

                // TODO: only use the vertices that we found in the indices (we're using too many vertices!)
                float3[] surfaceVertices = brushVertices.vertices.ToArray();

                var vertexHash	    = (ulong)Hashing.ComputeHashKey(surfaceVertices);
                var indicesHash	    = (ulong)Hashing.ComputeHashKey(surfaceIndices);
                var geometryHash    = Hashing.XXH64_mergeRound(vertexHash, indicesHash);

                var normalHash  = (ulong)0;
                Vector3[] surfaceNormals = null;
                if (anySurfaceTargetHasNormals)
                {
                    var normal = (interiorCategory == CategoryIndex.ValidReverseAligned || interiorCategory == CategoryIndex.ReverseAligned) ? -info.worldPlane.xyz : info.worldPlane.xyz;

                    surfaceNormals = surfaceVertices == null ? null : new Vector3[surfaceVertices.Length];
                    if (surfaceVertices != null)
                    {
                        for (int i = 0; i < surfaceVertices.Length; i++)
                            surfaceNormals[i] = normal;
                    }
                    normalHash = Hashing.ComputeHashKey(surfaceNormals);
                }
                var uv0Hash = (ulong)0;
                Vector2[] surfaceUV0 = null;
                if (anySurfaceTargetHasUVs)
                {
                    surfaceUV0 = surfaceVertices == null ? null : new Vector2[surfaceVertices.Length];
                    if (surfaceVertices != null)
                    {
                        for (int v = 0; v < surfaceVertices.Length; v++)
                            surfaceUV0[v] = uv0Matrix.MultiplyPoint3x4(surfaceVertices[v]);
                    }
                    uv0Hash = Hashing.ComputeHashKey(surfaceUV0);
                }


                for (int t = 0, tSize = meshQueries.Length; t < tSize; t++)
                {
                    var meshQuery = meshQueries[t];

                    var core_surface_flags = info.layers.layerUsage;
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
                            case LayerParameterIndex.LayerParameter1: surfaceParameter = info.layers.layerParameter1; break;
                            case LayerParameterIndex.LayerParameter2: surfaceParameter = info.layers.layerParameter2; break;
                            case LayerParameterIndex.LayerParameter3: surfaceParameter = info.layers.layerParameter3; break;
                        }
                    }

                    var surfaceRenderBuffer = new ChiselSurfaceRenderBuffer
                    {
                        indices     = surfaceIndices,
                        vertices    = surfaceVertices,
                        normals     = surfaceNormals,
                        uv0         = surfaceUV0
                    };

                    var haveUV0     = false; // TODO: actually determine this
                    var haveNormal  = false; // TODO: actually determine this

                    surfaceRenderBuffer.surfaceHash         = Hashing.XXH64_mergeRound(!haveNormal ? 0 : normalHash, !haveUV0 ? 0 : uv0Hash);
                    surfaceRenderBuffer.geometryHash        = geometryHash;
                    surfaceRenderBuffer.meshQuery	        = meshQuery;
                    surfaceRenderBuffer.surfaceParameter    = surfaceParameter;
                    surfaceRenderBuffer.surfaceIndex        = surfaceIndex;
                    outputSurfaces.Add(surfaceRenderBuffer);
                }
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