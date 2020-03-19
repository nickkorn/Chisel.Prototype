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


    struct BasePolygon
    {
        public SurfaceInfo  surfaceInfo;
        public int          startEdgeIndex;
        public int          endEdgeIndex;
    }

    struct BasePolygonsBlob
    {
        public BlobArray<BasePolygon>   surfaces;
        public BlobArray<Edge>          edges;
        public BlobArray<float3>        vertices;
        public AABB                     bounds;

        public static unsafe BlobAssetReference<BasePolygonsBlob> Create(int brushNodeIndex, BlobAssetReference<BrushMeshBlob> mesh, BlobAssetReference<NodeTransformations> transform)
        {
            ref var vertices   = ref mesh.Value.vertices;
            ref var planes     = ref mesh.Value.localPlanes;
            ref var polygons   = ref mesh.Value.polygons;
            var nodeToTreeSpaceMatrix   = transform.Value.nodeToTree;
            var nodeToTreeSpaceInvertedTransposedMatrix = math.transpose(math.inverse(nodeToTreeSpaceMatrix));

            var min = new float3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            var max = new float3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);

            var aabb = new AABB();

            var edges           = new NativeList<Edge>(Allocator.Temp);
            var surfaces        = new NativeList<BasePolygon>(Allocator.Temp);
            var vertexSoup      = new VertexSoup(vertices.Length, Allocator.Temp);
            var polygonEdges    = new NativeList<Edge>(Allocator.Temp);
            for (int p = 0; p < polygons.Length; p++)
            {
                var polygon      = polygons[p];

                if (polygon.edgeCount < 3 ||
                    p >= planes.Length)
                    continue;

                var firstEdge    = polygon.firstEdge;
                var lastEdge     = firstEdge + polygon.edgeCount;
                var indexCount   = lastEdge - firstEdge;

                polygonEdges.Clear();
                if (polygonEdges.Capacity < indexCount)
                    polygonEdges.Capacity = indexCount;
                
                float4 worldPlane = float4.zero;
                // THEORY: can end up with duplicate vertices when close enough vertices are snapped together
                var copyPolygonToIndicesJob = new CopyPolygonToIndicesJob
                {
                    mesh                                    = mesh,
                    polygonIndex                            = p,
                    nodeToTreeSpaceMatrix                   = nodeToTreeSpaceMatrix,
                    nodeToTreeSpaceInvertedTransposedMatrix = nodeToTreeSpaceInvertedTransposedMatrix,
                    vertexSoup                              = vertexSoup,
                    edges                                   = polygonEdges,

                    aabb                                    = &aabb,

                    worldPlane                              = &worldPlane
                };

                // TODO: inline this into this job
                copyPolygonToIndicesJob.Execute(); 

                if (polygonEdges.Length == 0)
                    continue;

                min = aabb.min;
                max = aabb.max;

                int startEdgeIndex = edges.Length;
                for (int i = 0; i < polygonEdges.Length; i++)
                    edges.Add(polygonEdges[i]);
                var endEdgeIndex = edges.Length;

                surfaces.Add(new BasePolygon()
                {
                    surfaceInfo = new SurfaceInfo()
                    {
                        worldPlane          = worldPlane,
                        layers              = polygon.layerDefinition,
                        basePlaneIndex      = p,
                        brushNodeIndex      = brushNodeIndex,
                        interiorCategory    = (CategoryGroupIndex)(int)CategoryIndex.ValidAligned,
                    },
                    startEdgeIndex  = startEdgeIndex,
                    endEdgeIndex    = endEdgeIndex
                });
            }

            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<BasePolygonsBlob>();
            builder.Construct(ref root.surfaces, surfaces);
            builder.Construct(ref root.edges, edges);
            builder.Construct(ref root.vertices, vertexSoup);
            root.bounds = new AABB() { min = min, max = max };
            var result = builder.CreateBlobAssetReference<BasePolygonsBlob>(Allocator.Persistent);
            builder.Dispose();
            vertexSoup.Dispose();
            edges.Dispose();
            surfaces.Dispose();
            polygonEdges.Dispose();

            return result;
        }
    }
    
    public enum IntersectionType
    {
        NoIntersection,
        Intersection,
        AInsideB,
        BInsideA,

        InvalidValue
    };

    public struct BrushIntersectionInfo
    {
        public int                               brushNodeIndex;
        public BlobAssetReference<BrushMeshBlob> blobMesh;
        public NodeTransformations               transformation;
        public float4x4                          toOtherBrushSpace;

        public BlobArray<int>                    localSpacePlaneIndices0;   // planes in local space of >brush0<
        public BlobArray<float4>                 localSpacePlanes0;         // planes in local space of >brush0<

        public BlobArray<PlanePair>              usedPlanePairs;
        public BlobArray<int>                    usedVertices;
        public BlobArray<SurfaceInfo>            surfaceInfos;
    }
    
    public struct BrushIntersectionLoop
    {
        public SurfaceInfo          surfaceInfo;
        public BlobArray<float3>    loopVertices;
    }

    public struct BrushPairIntersection
    {
        public IntersectionType type;
        // Note: that the localSpacePlanes0/localSpacePlaneIndices0 parameters for both brush0 and brush1 are in localspace of >brush0<
        public BlobArray<BrushIntersectionInfo> brushes;
    }


    internal sealed unsafe class ChiselLookup : ScriptableObject
    {
        public unsafe struct Data
        {
            public NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>>         basePolygons;
            public NativeHashMap<int, BlobAssetReference<RoutingTable>>             routingTableLookup;
            public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>         brushWorldPlanes;
            public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>    brushesTouchedByBrushes;
            public NativeHashMap<int, BlobAssetReference<NodeTransformations>>      transformations;
            public Dictionary<int, VertexSoup>                                      vertexSoups;

            public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>            brushMeshBlobs;
            public NativeHashMap<int, BlobAssetReference<CompactTree>>              compactTrees;

            internal void RemoveByBrushID(NativeArray<int> treeBrushes)
            {
                for (int b = 0; b < treeBrushes.Length; b++)
                {
                    var brushNodeID = treeBrushes[b];
                    if (basePolygons.TryGetValue(brushNodeID - 1, out BlobAssetReference<BasePolygonsBlob> basePolygonsBlob))
                    {
                        basePolygons.Remove(brushNodeID - 1);
                        if (basePolygonsBlob.IsCreated)
                            basePolygonsBlob.Dispose();
                    }
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
                    if (vertexSoups.TryGetValue(brushNodeID - 1, out VertexSoup vertexSoup))
                    {
                        vertexSoups.Remove(brushNodeID - 1);
                        vertexSoup.Dispose();
                    }
                }
            }


            internal void Initialize()
            {
                // brushIndex
                basePolygons            = new NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>>(1000, Allocator.Persistent);
                routingTableLookup      = new NativeHashMap<int, BlobAssetReference<RoutingTable>>(1000, Allocator.Persistent);
                brushWorldPlanes        = new NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>(1000, Allocator.Persistent);
                brushesTouchedByBrushes = new NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>(1000, Allocator.Persistent);
                transformations         = new NativeHashMap<int, BlobAssetReference<NodeTransformations>>(1000, Allocator.Persistent);
                vertexSoups             = new Dictionary<int, VertexSoup>();

                // brushMeshIndex
                brushMeshBlobs          = new NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>(1000, Allocator.Persistent);

                // treeIndex
                compactTrees            = new NativeHashMap<int, BlobAssetReference<CompactTree>>(100, Allocator.Persistent);
            }

            internal void Dispose()
            {
                if (basePolygons.IsCreated)
                {
                    using (var items = basePolygons.GetValueArray(Allocator.Temp))
                    {
                        basePolygons.Clear();
                        basePolygons.Dispose();
                        foreach (var item in items)
                        {
                            if (item.IsCreated)
                                item.Dispose();
                        }
                    }
                }
                if (routingTableLookup.IsCreated)
                {
                    using (var items = routingTableLookup.GetValueArray(Allocator.Temp))
                    {
                        routingTableLookup.Clear();
                        routingTableLookup.Dispose();
                        foreach (var item in items)
                        {
                            if (item.IsCreated)
                                item.Dispose();
                        }
                    }
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
                if (vertexSoups != null)
                {
                    foreach (var item in vertexSoups)
                        item.Value.Dispose();
                    vertexSoups.Clear();
                    vertexSoups = null;
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
            _instance = UnsafeUtility.AddressOf<Data>(ref _singleton.chiselLookup);
        }

        public static ref Data Value
        {
            get
            {
                if (_instance == null)
                    UpdateValue();
                return ref UnsafeUtilityEx.AsRef<Data>(_instance);
            }
        }

        Data chiselLookup = new Data();

        internal void OnEnable() { chiselLookup.Initialize(); }
        internal void OnDisable() { chiselLookup.Dispose(); _singleton = null; }
    }
}
