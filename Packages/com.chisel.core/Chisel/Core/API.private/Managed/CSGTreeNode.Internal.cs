using System;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace Chisel.Core
{
    partial struct CSGTreeNode
    {
#if USE_MANAGED_CSG_IMPLEMENTATION
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	IsNodeDirty(Int32 nodeID)		{ return CSGManager.IsNodeDirty(nodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	SetDirty(Int32 nodeID)			{ return CSGManager.SetDirty(nodeID); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static CSGNodeType GetTypeOfNode(Int32 nodeID) { return CSGManager.GetTypeOfNode(nodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	IsNodeIDValid(Int32 nodeID)		{ return CSGManager.IsValidNodeID(nodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static Int32	GetUserIDOfNode(Int32 nodeID)	{ return CSGManager.GetUserIDOfNode(nodeID); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static Int32	GetParentOfNode(Int32 nodeID)	{ return CSGManager.GetParentOfNode(nodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static Int32	GetTreeOfNode(Int32 nodeID)		{ return CSGManager.GetTreeOfNode(nodeID); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static Int32	GetChildNodeCount(Int32 nodeID) { return CSGManager.GetChildNodeCount(nodeID); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	RemoveChildNode(Int32 nodeID, Int32 childNodeID)				{ return CSGManager.RemoveChildNode(nodeID, childNodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	AddChildNode(Int32 nodeID, Int32 childNodeID)					{ return CSGManager.AddChildNode(nodeID, childNodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	ClearChildNodes(Int32 nodeID)									{ return CSGManager.ClearChildNodes(nodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static Int32	GetChildNodeAtIndex(Int32 nodeID, Int32 index)					{ return CSGManager.GetChildNodeAtIndex(nodeID, index); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	RemoveChildNodeAt(Int32 nodeID, Int32 index)					{ return CSGManager.RemoveChildNodeAt(nodeID, index); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	InsertChildNode(Int32 nodeID, Int32 index, Int32 childNodeID)	{ return CSGManager.InsertChildNode(nodeID, index, childNodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static Int32	IndexOfChildNode(Int32 nodeID, Int32 childNodeID)				{ return CSGManager.IndexOfChildNode(nodeID, childNodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	RemoveChildNodeRange(Int32 nodeID, Int32 index, Int32 count)	{ return CSGManager.RemoveChildNodeRange(nodeID, index, count); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	DestroyNode(Int32 nodeID)										{ return CSGManager.DestroyNode(nodeID); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	GetNodeLocalTransformation(Int32 nodeID, out Matrix4x4 localTransformation)		{ return CSGManager.GetNodeLocalTransformation(nodeID, out localTransformation); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	SetNodeLocalTransformation(Int32 nodeID, ref Matrix4x4 localTransformation)		{ return CSGManager.SetNodeLocalTransformation(nodeID, ref localTransformation); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	GetTreeToNodeSpaceMatrix(Int32 nodeID, out Matrix4x4 treeToNodeMatrix)			{ return CSGManager.GetTreeToNodeSpaceMatrix(nodeID, out treeToNodeMatrix); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	GetNodeToTreeSpaceMatrix(Int32 nodeID, out Matrix4x4 nodeToTreeMatrix)			{ return CSGManager.GetNodeToTreeSpaceMatrix(nodeID, out nodeToTreeMatrix); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static Int32	GetNodeOperationType(Int32 nodeID)												{ return (int)CSGManager.GetNodeOperationType(nodeID); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool	SetNodeOperationType(Int32 nodeID, CSGOperationType operation)					{ return CSGManager.SetNodeOperationType(nodeID, operation); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool SetChildNodes(Int32 nodeID, CSGTreeNode[] children)
        {
            return CSGManager.SetChildNodes(nodeID, children);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static CSGTreeNode[] GetChildNodes(Int32 nodeID)
        {
            return CSGManager.GetChildNodes(nodeID);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static int CopyToUnsafe(Int32 nodeID, int childCount, CSGTreeNode[] children, int arrayIndex)
        {
            return CSGManager.CopyToUnsafe(nodeID, childCount, children, arrayIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool InsertChildNodeRange(Int32 nodeID, Int32 index, CSGTreeNode[] children)
        {
            return CSGManager.InsertChildNodeRange(nodeID, index, children);
        }
#endif
    }
}