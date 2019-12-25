﻿using System;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace Chisel.Core
{
    partial struct CSGTreeNode
    {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static Matrix4x4 GetNodeLocalTransformation(Int32 nodeID)
        {
            Matrix4x4 result = Matrix4x4.identity;
            if (GetNodeLocalTransformation(nodeID, out result))
                return result;
            return Matrix4x4.identity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static int CopyTo(Int32 nodeID, CSGTreeNode[] children, int arrayIndex)
        {
            if (children == null)
                throw new ArgumentNullException("children");

            var childCount = GetChildNodeCount(nodeID);
            if (childCount <= 0)
                return 0;

            if (children.Length + arrayIndex < childCount)
                throw new ArgumentException(string.Format("The array does not have enough elements, its length is {0} and needs at least {1}", children.Length, childCount), "children");

            return CopyToUnsafe(nodeID, childCount, children, arrayIndex);
        }
    }
}