﻿using System;
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
    }
#endif
}
