using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using UnityEngine;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION    
    public struct RoutingTable
    {
        public BlobArray<CategoryGroupIndex>	inputs;
        public BlobArray<CategoryRoutingRow>	routingRows;
        public BlobArray<RoutingLookup>         routingLookups;
        public BlobArray<int>	                nodes;
        
        public unsafe static BlobAssetReference<RoutingTable> Build(NativeList<CategoryGroupIndex> inputs,
                                                                    NativeList<CategoryRoutingRow> routingRows,
                                                                    NativeList<RoutingLookup>      routingLookups,
                                                                    NativeList<int>                nodes)
        {
            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<RoutingTable>();
            builder.Construct(ref root.inputs,          inputs);
            builder.Construct(ref root.routingRows,     routingRows);
            builder.Construct(ref root.routingLookups,  routingLookups);
            builder.Construct(ref root.nodes,           nodes);
            var routingTable = builder.CreateBlobAssetReference<RoutingTable>(Allocator.Persistent);
            builder.Dispose();
            return routingTable;
        }
    }
#endif
}
