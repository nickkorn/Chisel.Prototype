using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;

namespace Chisel.Core
{
    partial struct BrushMeshInstance
    {
#if USE_MANAGED_CSG_IMPLEMENTATION
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Int32 CreateBrushMesh(Int32					userID,
                                             float3[]				vertices,
                                             BrushMesh.HalfEdge[]	halfEdges,
                                             BrushMesh.Polygon[]	polygons)
        {
            return BrushMeshManager.CreateBrushMesh(userID, vertices, halfEdges, polygons);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Int32 GetBrushMeshUserID(Int32 brushMeshIndex)
        {
            return BrushMeshManager.GetBrushMeshUserID(brushMeshIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool UpdateBrushMesh(Int32				 brushMeshID,
                                            float3[]             vertices,
                                            BrushMesh.HalfEdge[] halfEdges,
                                            BrushMesh.Polygon[]  polygons)
        {
            return BrushMeshManager.UpdateBrushMesh(brushMeshID, vertices, halfEdges, polygons);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool DestroyBrushMesh(Int32 brushMeshID)
        {
            return BrushMeshManager.DestroyBrushMesh(brushMeshID);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool IsBrushMeshIDValid(Int32 brushMeshID)
        {
            return BrushMeshManager.IsBrushMeshIDValid(brushMeshID);
        }
#endif
    }
}