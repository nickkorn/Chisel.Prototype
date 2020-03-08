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
    public struct BrushWorldPlanes
    {
        public BlobArray<float4> worldPlanes;

        public static BlobAssetReference<BrushWorldPlanes> BuildPlanes(BlobAssetReference<BrushMeshBlob> brushMeshBlob, float4x4 nodeToTreeTransformation)
        {
            if (!brushMeshBlob.IsCreated)
                return BlobAssetReference<BrushWorldPlanes>.Null;

            var nodeToTreeInverseTransposed = math.transpose(math.inverse(nodeToTreeTransformation));
            using (var builder = new BlobBuilder(Allocator.Temp))
            {
                ref var root = ref builder.ConstructRoot<BrushWorldPlanes>();
                var worldPlaneArray = builder.Allocate(ref root.worldPlanes, brushMeshBlob.Value.localPlanes.Length);
                for (int i = 0; i < brushMeshBlob.Value.localPlanes.Length; i++)
                {
                    var localPlane = brushMeshBlob.Value.localPlanes[i];
                    worldPlaneArray[i] = math.mul(nodeToTreeInverseTransposed, localPlane);
                }
                return builder.CreateBlobAssetReference<BrushWorldPlanes>(Allocator.Persistent);
            }
        }
    }

    public struct BrushMeshBlob
    {
        public struct Polygon
        {
            public Int32 firstEdge;
            public Int32 edgeCount;
            public SurfaceLayers layerDefinition;
        }

        public Bounds		                    localBounds;
        public BlobArray<float3>	            vertices;
        public BlobArray<BrushMesh.HalfEdge>	halfEdges;
        public BlobArray<int>                   halfEdgePolygonIndices;
        public BlobArray<Polygon>	            polygons;
        public BlobArray<float4>                localPlanes;
        

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsEmpty()
        {
            return (localPlanes.Length == 0 || polygons.Length == 0 || vertices.Length == 0 || halfEdges.Length == 0);
        }

        public static BlobAssetReference<BrushMeshBlob> Build(BrushMesh brushMesh)
        {
            if (brushMesh == null)
                return BlobAssetReference<BrushMeshBlob>.Null;
            using (var builder = new BlobBuilder(Allocator.Temp))
            {
                ref var root = ref builder.ConstructRoot<BrushMeshBlob>();
                root.localBounds = brushMesh.localBounds;
                builder.Construct(ref root.vertices, brushMesh.vertices);
                builder.Construct(ref root.halfEdges, brushMesh.halfEdges);
                builder.Construct(ref root.halfEdgePolygonIndices, brushMesh.halfEdgePolygonIndices);
                var polygonArray = builder.Allocate(ref root.polygons, brushMesh.polygons.Length);
                for (int i = 0; i < brushMesh.polygons.Length; i++)
                {
                    var polygon = brushMesh.polygons[i];
                    polygonArray[i] = new Polygon()
                    {
                        firstEdge = polygon.firstEdge,
                        edgeCount = polygon.edgeCount,
                        layerDefinition = polygon.surface.brushMaterial.LayerDefinition
                    };
                }
                builder.Construct(ref root.localPlanes, brushMesh.planes);
                return builder.CreateBlobAssetReference<BrushMeshBlob>(Allocator.Persistent);
            }
        }
    }

    internal partial class BrushMeshManager
    {
#if USE_MANAGED_CSG_IMPLEMENTATION
        static List<BrushMesh>	brushMeshes		= new List<BrushMesh>();
        static List<int>		userIDs			= new List<int>();
        static List<int>		unusedIDs		= new List<int>();
        static List<BlobAssetReference<BrushMeshBlob>> brushMeshesBlobs = new List<BlobAssetReference<BrushMeshBlob>>();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool		IsBrushMeshIDValid		(Int32 brushMeshInstanceID)	{ return brushMeshInstanceID > 0 && brushMeshInstanceID <= brushMeshes.Count; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool			AssertBrushMeshIDValid	(Int32 brushMeshInstanceID)
        {
            if (!IsBrushMeshIDValid(brushMeshInstanceID))
            {
                var nodeIndex = brushMeshInstanceID - 1;
                if (nodeIndex >= 0 && nodeIndex < brushMeshes.Count)
                    Debug.LogError($"Invalid ID {brushMeshInstanceID}");
                else
                    Debug.LogError($"Invalid ID {brushMeshInstanceID}, outside of bounds (min 1, max {brushMeshes.Count})");
                return false;
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static int			GetBrushMeshCount		()					{ return brushMeshes.Count - unusedIDs.Count; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Int32			GetBrushMeshUserID		(Int32 brushMeshInstanceID)
        {
            if (!AssertBrushMeshIDValid(brushMeshInstanceID))
                return CSGManager.kDefaultUserID;
            return userIDs[brushMeshInstanceID - 1];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BrushMesh		GetBrushMesh			(Int32 brushMeshInstanceID)
        {
            if (!AssertBrushMeshIDValid(brushMeshInstanceID))
                return null;
            var brushMesh = brushMeshes[brushMeshInstanceID - 1];
            if (brushMesh == null)
                return null;
            return brushMesh;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [BurstDiscard]
        internal static BlobAssetReference<BrushMeshBlob> GetBrushMeshBlob(Int32 brushMeshInstanceID)
        {
            if (!IsBrushMeshIDValid(brushMeshInstanceID))
                return BlobAssetReference<BrushMeshBlob>.Null;
            return brushMeshesBlobs[brushMeshInstanceID - 1];
        }

        public static Int32 CreateBrushMesh(Int32				 userID,
                                            float3[]			 vertices,
                                            BrushMesh.HalfEdge[] halfEdges,
                                            BrushMesh.Polygon[]	 polygons)
        {
            int			brushMeshID		= CreateBrushMeshID(userID);
            BrushMesh	brushMesh		= GetBrushMesh(brushMeshID);

            if (brushMesh == null)
            {
                Debug.LogWarning("brushMesh == nullptr");
                DestroyBrushMesh(brushMeshID);
                return BrushMeshInstance.InvalidInstanceID;
            }

            if (!brushMesh.Set(vertices, halfEdges, polygons))
            {
                Debug.LogWarning("GenerateMesh failed");
                DestroyBrushMesh(brushMeshID);
                return BrushMeshInstance.InvalidInstanceID;
            }

            var brushMeshIndex = brushMeshID - 1;
            brushMeshesBlobs[brushMeshIndex] = BrushMeshBlob.Build(brushMesh);
            return brushMeshID;
        }


        public static bool UpdateBrushMesh(Int32				brushMeshInstanceID,
                                           float3[]			    vertices,
                                           BrushMesh.HalfEdge[] halfEdges,
                                           BrushMesh.Polygon[]	polygons)
        {
            if (vertices == null || halfEdges == null || polygons == null) return false;
            
            if (!AssertBrushMeshIDValid(brushMeshInstanceID))
                return false;

            BrushMesh brushMesh = GetBrushMesh(brushMeshInstanceID);
            if (brushMesh == null)
            {
                Debug.LogWarning("Brush has no BrushMeshInstance set");
                return false;
            }

            if (!brushMesh.Set(vertices, halfEdges, polygons))
            {
                Debug.LogWarning("GenerateMesh failed");
                return false;
            }

            var brushMeshIndex = brushMeshInstanceID - 1;
            if (brushMeshesBlobs[brushMeshIndex].IsCreated)
                brushMeshesBlobs[brushMeshIndex].Dispose();
            brushMeshesBlobs[brushMeshIndex] = BrushMeshBlob.Build(brushMesh);
            CSGManager.NotifyBrushMeshModified(brushMeshInstanceID);
            return true;
        }

        private static int CreateBrushMeshID(Int32 userID)
        {
            if (unusedIDs.Count == 0)
            {
                int index = brushMeshes.Count;
                brushMeshes.Add(new BrushMesh());
                brushMeshesBlobs.Add(BlobAssetReference<BrushMeshBlob>.Null);
                userIDs.Add(userID);
                return index + 1;
            }

            unusedIDs.Sort(); // sorry!
            var brushMeshID		= unusedIDs[0];
            var brushMeshIndex	= brushMeshID - 1;
            unusedIDs.RemoveAt(0); // sorry again
            brushMeshes[brushMeshIndex].Reset();
            if (brushMeshesBlobs[brushMeshIndex].IsCreated)
                brushMeshesBlobs[brushMeshIndex].Dispose();
            brushMeshesBlobs[brushMeshIndex] = BlobAssetReference<BrushMeshBlob>.Null;
            userIDs[brushMeshIndex] = userID;
            return brushMeshID;
        }

        public static bool DestroyBrushMesh(Int32 brushMeshInstanceID)
        {
            if (!AssertBrushMeshIDValid(brushMeshInstanceID))
                return false;

            CSGManager.NotifyBrushMeshRemoved(brushMeshInstanceID);

            var brushMeshIndex = brushMeshInstanceID - 1;
            brushMeshes[brushMeshIndex].Reset();
            if (brushMeshesBlobs[brushMeshIndex].IsCreated)
                brushMeshesBlobs[brushMeshIndex].Dispose();
            brushMeshesBlobs[brushMeshIndex] = BlobAssetReference<BrushMeshBlob>.Null;
            userIDs[brushMeshIndex] = CSGManager.kDefaultUserID;
            unusedIDs.Add(brushMeshInstanceID);

            // TODO: remove elements when last values are invalid

            return true;
        }
        
        internal static BrushMeshInstance[] GetAllBrushMeshInstances()
        {
            var instanceCount = GetBrushMeshCount();
            var allInstances = new BrushMeshInstance[instanceCount];
            if (instanceCount == 0)
                return allInstances;
            
            int index = 0;
            for (int i = 0; i < brushMeshes.Count; i++)
            {
                if (IsBrushMeshIDValid(i))
                    continue;
                
                allInstances[index] = new BrushMeshInstance() { brushMeshID = i };
                index++;
            }
            return allInstances;
        }
#endif
    }
}
