using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Entities;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Profiler = UnityEngine.Profiling.Profiler;
using System.Runtime.CompilerServices;
using Unity.Burst;

namespace Chisel.Core
{
    public struct AABB
    {
        public AABB(Bounds bounds) { min = bounds.min; max = bounds.max; }
        public float3 min, max;
    }

    // TODO: create blob for basepolygons
    public struct BasePolygonsBlob
    {
        public AABB                     aabb;
        public BlobArray<Edge>          polygonEdges;
        public BlobArray<int>           polygonIndices;
        public BlobArray<SurfaceInfo>   polygonSurfaceInfos;
    }

    struct CompactTopDownNode
    {
        // TODO: combine bits
        public CSGNodeType      Type;
        public CSGOperationType Operation;
        public int              nodeIndex;
        public int              childCount;
        public int              childOffset;

        public override string ToString() { return $"({nameof(Type)}: {Type}, {nameof(childCount)}: {childCount}, {nameof(childOffset)}: {childOffset}, {nameof(Operation)}: {Operation}, {nameof(nodeIndex)}: {nodeIndex})"; }
    }

    struct BottomUpNodeIndex
    {
        public int nodeIndex; // TODO: might not be needed
        public int bottomUpStart;
        public int bottomUpEnd;

        public override string ToString() { return $"({nameof(nodeIndex)}: {nodeIndex}, {nameof(bottomUpStart)}: {bottomUpStart}, {nameof(bottomUpEnd)}: {bottomUpEnd})"; }
    }

    struct CompactTree
    {
        public BlobArray<CompactTopDownNode>        topDownNodes;
        public BlobArray<BottomUpNodeIndex>         bottomUpNodeIndices;
        public BlobArray<int>                       bottomUpNodes;

        public int                                  indexOffset;
        public BlobArray<int>                       brushIndexToBottomUpIndex;

        struct CompactTopDownBuilderNode
        {
            public CSGTreeNode node;
            public int index;
        }

        internal static BlobAssetReference<CompactTree> Create(List<CSGManager.NodeHierarchy> nodeHierarchies, int treeNodeIndex)
        {
            if (ChiselLookup.Value.compactTrees.TryGetValue(treeNodeIndex, out BlobAssetReference<CompactTree> oldCompactTree))
            {
                ChiselLookup.Value.compactTrees.Remove(treeNodeIndex); // Remove first in case there's an error
                if (oldCompactTree.IsCreated)
                    oldCompactTree.Dispose();
            }

            var treeInfo                    = nodeHierarchies[treeNodeIndex].treeInfo;
            var treeBrushes                 = treeInfo.treeBrushes;

            if (treeBrushes.Count == 0)
                return BlobAssetReference<CompactTree>.Null;

            var bottomUpNodeIndices         = new List<BottomUpNodeIndex>();
            var bottomUpNodes               = new List<int>();
            
            var minBrushIndex               = nodeHierarchies.Count;
            var maxBrushIndex               = 0;
            for (int b = 0; b < treeBrushes.Count; b++)
            {
                var brush = new CSGTreeNode() { nodeID = treeBrushes[b] };
                if (!brush.Valid)
                    continue;

                minBrushIndex = math.min(brush.NodeID - 1, minBrushIndex);
                maxBrushIndex = math.max(brush.NodeID - 1, maxBrushIndex);
            }
            var brushIndexToBottomUpIndex = new int[(maxBrushIndex + 1) - minBrushIndex];

            // Bottom-up -> per brush list of all ancestors to root
            for (int b = 0; b < treeBrushes.Count; b++)
            {
                var brush = new CSGTreeNode() { nodeID = treeBrushes[b] };
                if (!brush.Valid)
                    continue;

                var parentStart = bottomUpNodes.Count;

                var parent = brush.Parent;
                var treeNodeID = treeNodeIndex + 1;
                while (parent.Valid && parent.NodeID != treeNodeID)
                {
                    var parentIndex = parent.NodeID - 1;
                    bottomUpNodes.Add(parentIndex);
                    parent = parent.Parent;
                }

                brushIndexToBottomUpIndex[brush.NodeID - 1 - minBrushIndex] = bottomUpNodeIndices.Count;
                bottomUpNodeIndices.Add(new BottomUpNodeIndex()
                {
                    nodeIndex       = (brush.NodeID - 1),
                    bottomUpEnd     = bottomUpNodes.Count,
                    bottomUpStart   = parentStart
                });
            }

            // Top-down
            var nodeQueue       = new Queue<CompactTopDownBuilderNode>();
            var topDownNodes    = new List<CompactTopDownNode>(); // TODO: set capacity to number of nodes in tree

            nodeQueue.Enqueue(new CompactTopDownBuilderNode() { node = new CSGTreeNode() { nodeID =  treeNodeIndex + 1 }, index = 0 });
            topDownNodes.Add(new CompactTopDownNode()
            {
                Type        = CSGNodeType.Tree,
                Operation   = CSGOperationType.Additive,
                nodeIndex   = treeNodeIndex,
            });

            while (nodeQueue.Count > 0)
            {
                var parent      = nodeQueue.Dequeue();
                var nodeCount   = parent.node.Count;
                if (nodeCount == 0)
                {
                    var item = topDownNodes[parent.index];
                    item.childOffset = -1;
                    item.childCount = 0;
                    topDownNodes[parent.index] = item;
                    continue;
                }

                int firstIndex = 0;
                // Skip all nodes that are not additive at the start of the branch since they will never produce any geometry
                for (; firstIndex < nodeCount && parent.node[firstIndex].Valid && 
                                    (parent.node[firstIndex].Operation != CSGOperationType.Additive &&
                                     parent.node[firstIndex].Operation != CSGOperationType.Copy); firstIndex++)
                    firstIndex++;

                var firstChildIndex = topDownNodes.Count;
                for (int i = firstIndex; i < nodeCount; i++)
                {
                    var child = parent.node[i];
                    // skip invalid nodes (they don't contribute to the mesh)
                    if (!child.Valid)
                        continue;

                    var childType = child.Type;
                    if (childType != CSGNodeType.Brush)
                        nodeQueue.Enqueue(new CompactTopDownBuilderNode()
                        {
                            node = child,
                            index = topDownNodes.Count
                        });
                    topDownNodes.Add(new CompactTopDownNode()
                    {
                        Type        = childType,
                        Operation   = child.Operation,
                        nodeIndex   = child.NodeID - 1
                    });
                }

                {
                    var item = topDownNodes[parent.index];
                    item.childOffset = firstChildIndex;
                    item.childCount = topDownNodes.Count - firstChildIndex;
                    topDownNodes[parent.index] = item;
                }
            }

            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<CompactTree>();
            builder.Construct(ref root.topDownNodes, topDownNodes);
            builder.Construct(ref root.bottomUpNodeIndices, bottomUpNodeIndices);
            builder.Construct(ref root.bottomUpNodes, bottomUpNodes);
            root.indexOffset = minBrushIndex;
            builder.Construct(ref root.brushIndexToBottomUpIndex, brushIndexToBottomUpIndex);
            ChiselLookup.Value.compactTrees[treeNodeIndex] = builder.CreateBlobAssetReference<CompactTree>(Allocator.Persistent);
            builder.Dispose();

            return ChiselLookup.Value.compactTrees[treeNodeIndex];
        }
        
    }

    struct BrushIntersectionIndex
    {
        public int nodeIndex;
        public int bottomUpStart;
        public int bottomUpEnd;
        public int intersectionStart;
        public int intersectionEnd;

        public override string ToString() { return $"({nameof(nodeIndex)}: {nodeIndex}, {nameof(bottomUpStart)}: {bottomUpStart}, {nameof(bottomUpEnd)}: {bottomUpEnd}, {nameof(intersectionStart)}: {intersectionStart}, {nameof(intersectionEnd)}: {intersectionEnd})"; }
    }

    struct BrushIntersection
    {
        public int              nodeIndex;
        public IntersectionType type;
        public int              bottomUpStart;
        public int              bottomUpEnd;

        public override string ToString() { return $"({nameof(nodeIndex)}: {nodeIndex}, {nameof(type)}: {type}, {nameof(bottomUpStart)}: {bottomUpStart}, {nameof(bottomUpEnd)}: {bottomUpEnd})"; }
    }

    struct BrushesTouchedByBrush
    {
        public BlobArray<BrushIntersection> brushIntersections;
        public BlobArray<uint>              intersectionBits;
        public int Length;
        public int Offset;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public IntersectionType Get(int index)
        {
            index -= Offset;
            if (index < 0 || index >= Length)
                return IntersectionType.InvalidValue;

            index <<= 1;
            var int32Index = index >> 5;	// divide by 32
            var bitIndex = index & 31;	// remainder
            var twoBit = ((UInt32)3) << bitIndex;

            return (IntersectionType)((intersectionBits[int32Index] & twoBit) >> bitIndex);
        }
    }
        
    public struct NodeTransformations
    {
        public float4x4 nodeToTree;
        public float4x4 treeToNode;

        public static BlobAssetReference<NodeTransformations> Build(float4x4 nodeToTree, float4x4 treeToNode)
        {
            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<NodeTransformations>();
            root.nodeToTree = nodeToTree;
            root.treeToNode = treeToNode;
            var result = builder.CreateBlobAssetReference<NodeTransformations>(Allocator.Persistent);
            builder.Dispose();
            return result;
        }
    };

    internal sealed unsafe class ChiselLookup : ScriptableObject
    {
        public unsafe struct ChiselLookupValues
        {
            public NativeHashMap<int, BlobAssetReference<RoutingTable>>             routingTableLookup;
            public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>         brushWorldPlanes;
            public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>    brushesTouchedByBrushes;
            public NativeHashMap<int, BlobAssetReference<NodeTransformations>>      transformations;

            public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>            brushMeshBlobs;
            public NativeHashMap<int, BlobAssetReference<CompactTree>>              compactTrees;

            internal void RemoveByBrushID(NativeArray<int> treeBrushes)
            {
                for (int b = 0; b < treeBrushes.Length; b++)
                {
                    var brushNodeID = treeBrushes[b];
                    if (routingTableLookup.TryGetValue(brushNodeID - 1, out BlobAssetReference<RoutingTable> routingTable))
                    {
                        routingTableLookup.Remove(brushNodeID - 1);
                        if (routingTable.IsCreated)
                            routingTable.Dispose();
                    }
                    if (brushWorldPlanes.TryGetValue(brushNodeID - 1, out BlobAssetReference<BrushWorldPlanes> worldPlanes))
                    {
                        brushWorldPlanes.Remove(brushNodeID - 1);
                        if (worldPlanes.IsCreated)
                            worldPlanes.Dispose();
                    }
                    if (brushesTouchedByBrushes.TryGetValue(brushNodeID - 1, out BlobAssetReference<BrushesTouchedByBrush> brushesTouchedByBrush))
                    {
                        brushesTouchedByBrushes.Remove(brushNodeID - 1);
                        if (brushesTouchedByBrush.IsCreated)
                            brushesTouchedByBrush.Dispose();
                    }
                    if (transformations.TryGetValue(brushNodeID - 1, out BlobAssetReference<NodeTransformations> transformation))
                    {
                        transformations.Remove(brushNodeID - 1);
                        if (transformation.IsCreated)
                            transformation.Dispose();
                    }
                }
            }


            internal void Initialize()
            {
                // brushIndex
                routingTableLookup      = new NativeHashMap<int, BlobAssetReference<RoutingTable>>(1000, Allocator.Persistent);
                brushWorldPlanes        = new NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>(1000, Allocator.Persistent);
                brushesTouchedByBrushes = new NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>(1000, Allocator.Persistent);
                transformations         = new NativeHashMap<int, BlobAssetReference<NodeTransformations>>(1000, Allocator.Persistent);

                // brushMeshIndex
                brushMeshBlobs          = new NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>(1000, Allocator.Persistent);

                // treeIndex
                compactTrees            = new NativeHashMap<int, BlobAssetReference<CompactTree>>(100, Allocator.Persistent);
            }

            internal void Dispose()
            {
                if (routingTableLookup.IsCreated)
                {
                    using (var items = routingTableLookup.GetValueArray(Allocator.Temp))
                    {
                        foreach (var item in items)
                        {
                            if (item.IsCreated)
                                item.Dispose();
                        }
                    }
                    routingTableLookup.Clear();
                    routingTableLookup.Dispose();
                }
                if (brushWorldPlanes.IsCreated)
                {
                    using (var items = brushWorldPlanes.GetValueArray(Allocator.Temp))
                    {
                        brushWorldPlanes.Clear();
                        brushWorldPlanes.Dispose();
                        foreach (var item in items)
                        {
                            if (item.IsCreated)
                                item.Dispose();
                        }
                    }
                }
                if (brushesTouchedByBrushes.IsCreated)
                {
                    using (var items = brushesTouchedByBrushes.GetValueArray(Allocator.Temp))
                    {
                        brushesTouchedByBrushes.Clear();
                        brushesTouchedByBrushes.Dispose();
                        foreach (var item in items)
                        {
                            if (item.IsCreated)
                                item.Dispose();
                        }
                    }
                }
                if (transformations.IsCreated)
                {
                    using (var items = transformations.GetValueArray(Allocator.Temp))
                    {
                        transformations.Clear();
                        transformations.Dispose();
                        foreach (var item in items)
                        {
                            if (item.IsCreated)
                                item.Dispose();
                        }
                    }
                }
                if (brushMeshBlobs.IsCreated)
                {
                    using (var items = brushMeshBlobs.GetValueArray(Allocator.Temp))
                    {
                        brushMeshBlobs.Clear();
                        brushMeshBlobs.Dispose();
                        foreach (var item in items)
                        {
                            if (item.IsCreated)
                                item.Dispose();
                        }
                    }
                }
                if (compactTrees.IsCreated)
                {
                    using (var items = compactTrees.GetValueArray(Allocator.Temp))
                    {
                        compactTrees.Clear();
                        compactTrees.Dispose();
                        foreach (var item in items)
                        {
                            if (item.IsCreated)
                                item.Dispose();
                        }
                    }
                }
            }
        }

        static void* _instance;
        static ChiselLookup _singleton;

        [BurstDiscard]
        static void UpdateValue()
        {
            if (_singleton == null)
            {
                _singleton = ScriptableObject.CreateInstance<ChiselLookup>();
                _singleton.hideFlags = HideFlags.HideAndDontSave;
            }
            _instance = UnsafeUtility.AddressOf<ChiselLookupValues>(ref _singleton.chiselLookup);
        }

        public static ref ChiselLookupValues Value
        {
            get
            {
                if (_instance == null)
                    UpdateValue();
                return ref UnsafeUtilityEx.AsRef<ChiselLookupValues>(_instance);
            }
        }

        ChiselLookupValues chiselLookup = new ChiselLookupValues();

        internal void OnEnable() { chiselLookup.Initialize(); }
        internal void OnDisable() { chiselLookup.Dispose(); _singleton = null; }
    }
}
