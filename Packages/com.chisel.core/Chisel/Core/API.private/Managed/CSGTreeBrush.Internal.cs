using System;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace Chisel.Core
{
    partial struct CSGTreeBrush
    {
#if USE_MANAGED_CSG_IMPLEMENTATION
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool					GenerateBrush(Int32 userID, out Int32 generatedNodeID)		{ return CSGManager.GenerateBrush(userID, out generatedNodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static CSGTreeBrushFlags	GetBrushFlags(Int32 brushNodeID)							{ return CSGManager.GetBrushFlags(brushNodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool					SetBrushFlags(Int32 brushNodeID, CSGTreeBrushFlags flags)	{ return CSGManager.SetBrushFlags(brushNodeID, flags); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Int32				GetBrushMeshID(Int32 brushNodeID)							{ return CSGManager.GetBrushMeshID(brushNodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool					SetBrushMeshID(Int32 brushNodeID, Int32 brushMeshID)		{ return CSGManager.SetBrushMeshID(brushNodeID, brushMeshID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        private static bool					GetBrushBounds(Int32 brushNodeID, ref Bounds bounds)		{ return CSGManager.GetBrushBounds(brushNodeID, ref bounds); }
#endif
    }
}