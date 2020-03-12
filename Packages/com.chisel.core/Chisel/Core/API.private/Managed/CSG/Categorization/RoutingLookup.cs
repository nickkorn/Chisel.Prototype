﻿using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Entities;
using UnityEngine;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    public struct RoutingLookup
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RoutingLookup(int startIndex, int endIndex)
        {
            this.startIndex = startIndex;
            this.endIndex = endIndex;
        }

        public readonly int startIndex;
        public readonly int endIndex;

        //public const int kRoutingOffset = 1 + (int)CategoryIndex.LastCategory;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetRoute(BlobAssetReference<RoutingTable> table, CategoryGroupIndex inputIndex, out CategoryRoutingRow routingRow)
        {
            var tableIndex = startIndex + (int)inputIndex;// (inputIndex == CategoryGroupIndex.First) ? (int)CategoryGroupIndex.First : ((int)inputIndex - kRoutingOffset);

            if (tableIndex < startIndex || tableIndex >= endIndex)
            {
                routingRow = new CategoryRoutingRow(inputIndex);
                return false;
            }

            //Debug.LogWarning($"{tableIndex} {inputIndex}");
            Debug.Assert(inputIndex == table.Value.inputs[tableIndex]);
            routingRow = table.Value.routingRows[tableIndex];
            return true;
        }
    }
#endif
}
