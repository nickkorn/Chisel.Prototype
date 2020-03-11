using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION    
    class RoutingTable : IDisposable
    {
        public CategoryGroupIndex[] inputs;
        public CategoryRoutingRow[] routingRows;
        public RoutingLookup[]      routingLookups;
        public Loop[][]             intersectionLoops;
        
        static readonly CategoryGroupIndex[]    kEmptyInputs            = new CategoryGroupIndex[0];
        static readonly CategoryRoutingRow[]    kEmptyRoutingRows       = new CategoryRoutingRow[0];
        static readonly RoutingLookup[]         kEmptyRoutingLookups    = new RoutingLookup[0];
        static readonly Loop[][]                kEmptyIntersectionLoops = new Loop[0][];

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            inputs              = kEmptyInputs;
            routingRows         = kEmptyRoutingRows;
            routingLookups      = kEmptyRoutingLookups;
            intersectionLoops   = kEmptyIntersectionLoops;
        }

        ~RoutingTable() { Dispose(); }
        public void Dispose()
        {
            if (intersectionLoops != null)
            {
                foreach (var loopArray in intersectionLoops)
                {
                    if (loopArray != null)
                    {
                        foreach (var loop in loopArray)
                        {
                            if (loop != null)
                                loop.Dispose();
                        }
                    }
                }
                intersectionLoops = null;
            }
            Clear();
        }
    }
    
    struct RoutingLookup
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
        public bool TryGetRoute(RoutingTable table, CategoryGroupIndex inputIndex, out CategoryRoutingRow routingRow)
        {
            var tableIndex = startIndex + (int)inputIndex;// (inputIndex == CategoryGroupIndex.First) ? (int)CategoryGroupIndex.First : ((int)inputIndex - kRoutingOffset);

            if (tableIndex < startIndex || tableIndex >= endIndex)
            {
                routingRow = new CategoryRoutingRow(inputIndex);
                return false;
            }

            //Debug.LogWarning($"{tableIndex} {inputIndex}");
            Debug.Assert(inputIndex == table.inputs[tableIndex]);
            routingRow = table.routingRows[tableIndex];
            return true;
        }
    }
#endif
}
