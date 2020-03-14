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
#if USE_MANAGED_CSG_IMPLEMENTATION
    static partial class CSGManager
    {

        // TODO: review flags, might not make sense any more
        enum NodeStatusFlags : UInt16
        {
            None						= 0,
//			NeedChildUpdate				= 1,
            NeedPreviousSiblingsUpdate	= 2,

            TreeIsDisabled				= 1024,// TODO: remove, or make more useful
            OperationNeedsUpdate		= 4,
            TreeNeedsUpdate				= 8,
            TreeMeshNeedsUpdate			= 16,
            
            NeedBaseMeshUpdate			= 32,	// -> leads to NeedsMeshReset
            NeedMeshReset				= 64,	
            NeedOutlineUpdate			= 128,
            NeedAllTouchingUpdated		= 256,	// all brushes that touch this brush need to be updated,
            NeedFullUpdate				= NeedBaseMeshUpdate | NeedMeshReset | NeedOutlineUpdate | NeedAllTouchingUpdated,
            NeedCSGUpdate				= NeedBaseMeshUpdate | NeedMeshReset | NeedAllTouchingUpdated,
            NeedUpdate					= NeedMeshReset | NeedOutlineUpdate | NeedAllTouchingUpdated,
            NeedUpdateDirectOnly		= NeedMeshReset | NeedOutlineUpdate,
        };
        

        internal sealed class TreeInfo : IDisposable
        {
            public readonly List<int>						treeBrushes			= new List<int>();
            public readonly List<GeneratedMeshDescription>	meshDescriptions	= new List<GeneratedMeshDescription>();
            public readonly List<SubMeshCounts>				subMeshCounts		= new List<SubMeshCounts>();


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Reset()
            {
                subMeshCounts.Clear();
            }

            public void Dispose()
            {

            }
        }

        struct CompactTopDownBuilderNode
        {
            public CSGTreeNode node;
            public int index;
        }


        static BlobAssetReference<CompactTree> GenerateCompactTree(CSGTree tree)
        {
            if (ChiselLookup.Value.compactTrees.TryGetValue(tree.NodeID, out BlobAssetReference<CompactTree> oldCompactTree))
            {
                ChiselLookup.Value.compactTrees.Remove(tree.NodeID); // Remove first in case there's an error
                if (oldCompactTree.IsCreated)
                    oldCompactTree.Dispose();
            }

            if (!tree.Valid)
                return BlobAssetReference<CompactTree>.Null;

            var treeNodeIndex               = tree.treeNodeID - 1;
            var treeInfo                    = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;
            var treeBrushes                 = treeInfo.treeBrushes;
            var bottomUpNodeIndices         = new List<BottomUpNodeIndex>();
            var bottomUpNodes               = new List<int>();
            
            var minBrushIndex               = CSGManager.nodeHierarchies.Count;
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
                while (parent.Valid && parent.NodeID != tree.NodeID)
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

            nodeQueue.Enqueue(new CompactTopDownBuilderNode() { node = tree, index = 0 });
            topDownNodes.Add(new CompactTopDownNode()
            {
                Type        = CSGNodeType.Tree,
                Operation   = CSGOperationType.Additive,
                nodeIndex   = tree.NodeID - 1,
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
            ChiselLookup.Value.compactTrees[tree.NodeID] = builder.CreateBlobAssetReference<CompactTree>(Allocator.Persistent);
            builder.Dispose();

            return ChiselLookup.Value.compactTrees[tree.NodeID];
        }
        

        internal static void SetUsedNodesBits(BlobAssetReference<CompactTree> compactTree, List<BrushIntersection> brushIntersections, int brushNodeID, int rootNodeID, BrushIntersectionLookup bitset)
        {
            var brushNodeIndex  = brushNodeID - 1;
            var rootNodeIndex   = rootNodeID - 1;

            bitset.Clear();
            bitset.Set(brushNodeIndex, IntersectionType.Intersection);
            bitset.Set(rootNodeIndex, IntersectionType.Intersection);

            var indexOffset = compactTree.Value.indexOffset;
            ref var bottomUpNodes               = ref compactTree.Value.bottomUpNodes;
            ref var bottomUpNodeIndices         = ref compactTree.Value.bottomUpNodeIndices;
            ref var brushIndexToBottomUpIndex   = ref compactTree.Value.brushIndexToBottomUpIndex;

            var intersectionIndex   = brushIndexToBottomUpIndex[brushNodeIndex - indexOffset];
            var intersectionInfo    = bottomUpNodeIndices[intersectionIndex];
            for (int b = intersectionInfo.bottomUpStart; b < intersectionInfo.bottomUpEnd; b++)
                bitset.Set(bottomUpNodes[b], IntersectionType.Intersection);

            for (int i = 0; i < brushIntersections.Count; i++)
            {
                var otherIntersectionInfo = brushIntersections[i];
                bitset.Set(otherIntersectionInfo.nodeIndex, otherIntersectionInfo.type);
                for (int b = otherIntersectionInfo.bottomUpStart; b < otherIntersectionInfo.bottomUpEnd; b++)
                    bitset.Set(bottomUpNodes[b], IntersectionType.Intersection);
            }
        }
        
        static BlobAssetReference<BrushesTouchedByBrush> GenerateBrushesTouchedByBrush(BlobAssetReference<CompactTree> compactTree, CSGTreeBrush brush, List<BrushBrushIntersection> touchingBrushes)
        {
            if (ChiselLookup.Value.brushesTouchedByBrushes.TryGetValue(brush.NodeID - 1, out BlobAssetReference<BrushesTouchedByBrush> oldBrushTouchedByBrushes))
            {
                ChiselLookup.Value.brushesTouchedByBrushes.Remove(brush.NodeID - 1); // Remove first in case there's an error
                if (oldBrushTouchedByBrushes.IsCreated)
                    oldBrushTouchedByBrushes.Dispose();
            }

            if (!compactTree.IsCreated || !brush.Valid)
                return BlobAssetReference<BrushesTouchedByBrush>.Null;

            var indexOffset = compactTree.Value.indexOffset;
            ref var bottomUpNodeIndices         = ref compactTree.Value.bottomUpNodeIndices;
            ref var brushIndexToBottomUpIndex   = ref compactTree.Value.brushIndexToBottomUpIndex;

            // Intersections
            var brushIntersectionIndices    = new List<BrushIntersectionIndex>();
            var brushIntersections          = new List<BrushIntersection>();
            var brushNodeID                 = brush.brushNodeID;
            var brushNodeIndex              = brushNodeID - 1;
            var rootNodeID                  = brush.Tree.treeNodeID;
            var intersectionStart           = brushIntersections.Count;
            foreach (var touchingBrush in touchingBrushes)
            {
                int otherBrushID    = touchingBrush.brushNodeID1;
                var otherBrush      = new CSGTreeNode() { nodeID = otherBrushID };
                var otherBrushIndex = otherBrushID - 1;
                if (!otherBrush.Valid || (otherBrushIndex < indexOffset || (otherBrushIndex-indexOffset) >= brushIndexToBottomUpIndex.Length))
                    continue;

                var otherBottomUpIndex = brushIndexToBottomUpIndex[otherBrushIndex - indexOffset];
                brushIntersections.Add(new BrushIntersection()
                {
                    nodeIndex       = otherBrushIndex,
                    type            = touchingBrush.type,
                    bottomUpStart   = bottomUpNodeIndices[otherBottomUpIndex].bottomUpStart,
                    bottomUpEnd     = bottomUpNodeIndices[otherBottomUpIndex].bottomUpEnd
                });
            }
            var bottomUpIndex = brushIndexToBottomUpIndex[brushNodeIndex - indexOffset];
            brushIntersectionIndices.Add(new BrushIntersectionIndex()
            {
                nodeIndex           = brushNodeIndex,
                bottomUpStart       = bottomUpNodeIndices[bottomUpIndex].bottomUpStart,
                bottomUpEnd         = bottomUpNodeIndices[bottomUpIndex].bottomUpEnd,    
                intersectionStart   = intersectionStart,
                intersectionEnd     = brushIntersections.Count
            });

            var bitset = new BrushIntersectionLookup(indexOffset, bottomUpNodeIndices.Length, Allocator.Temp);
            SetUsedNodesBits(compactTree, brushIntersections, brushNodeID, rootNodeID, bitset);
            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<BrushesTouchedByBrush>();
            builder.Construct(ref root.brushIntersections, brushIntersections);
            builder.Construct(ref root.intersectionBits, bitset.twoBits);
            root.Length = bitset.Length;
            root.Offset = bitset.Offset;
            var result = builder.CreateBlobAssetReference<BrushesTouchedByBrush>(Allocator.Persistent);
            builder.Dispose();            
            bitset.Dispose();
            return result;
        }

        internal static bool UpdateTreeMesh(int treeNodeID)
        {
            if (!IsValidNodeID(treeNodeID) || !AssertNodeType(treeNodeID, CSGNodeType.Tree))
                return false;

            var treeNodeIndex = treeNodeID - 1;
            var treeInfo = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;
            if (treeInfo == null)
                return false;

            var compactTree = GenerateCompactTree(new CSGTree() { treeNodeID = treeNodeID });

            Reset(treeInfo);

            OperationTables.EnsureInitialized(); // <-- store this in ChiselLookup & make blob

            var treeBrushes = treeInfo.treeBrushes;
            using (var treeBrushesArray = treeBrushes.ToNativeArray(Allocator.TempJob))
            {
                ChiselLookup.Value.RemoveByBrushID(treeBrushesArray);

                // TODO: store this in blob
                Profiler.BeginSample("UpdateBrushTransformations"); try {       UpdateBrushTransformations(treeBrushes);                            } finally { Profiler.EndSample(); }

                // TODO: should only do this at creation time + when moved
                Profiler.BeginSample("UpdateBrushWorldPlanes"); try {           UpdateBrushWorldPlanes(treeBrushesArray);                           } finally { Profiler.EndSample(); }
                Profiler.BeginSample("FindIntersectingBrushes"); try {          FindIntersectingBrushes(compactTree, treeBrushesArray);             } finally { Profiler.EndSample(); }

                // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
                Profiler.BeginSample("UpdateBrushCategorizationTables"); try {  UpdateBrushCategorizationTables(compactTree, treeBrushesArray);                  } finally { Profiler.EndSample(); }
                
                // TODO: should only do this once at creation time, part of brushMeshBlob?
                Profiler.BeginSample("GenerateBasePolygonLoops"); try {         GenerateBasePolygonLoops(treeBrushesArray);                         } finally { Profiler.EndSample(); }

                // TODO: split in two:
                //          1. find intersection loops, create blob per intersection (only update when necessary)
                //          2. instance intersection loop blob and basepolygon loop blobs & find intersections between them
                Profiler.BeginSample("FindAllIntersectionLoops"); try {         CSGManagerPerformCSG.FindAllIntersectionLoops(treeBrushesArray);    } finally { Profiler.EndSample(); }
                // TODO: use the above data to perform CSG
                Profiler.BeginSample("PerformAllCSG"); try {                    PerformAllCSG(treeBrushesArray);                                    } finally { Profiler.EndSample(); }
            }

            OperationTables.EnsureCleanedUp(); // <-- store this in ChiselLookup & make blob

            {
                var flags = nodeFlags[treeNodeIndex];
                flags.UnSetNodeFlag(NodeStatusFlags.TreeNeedsUpdate);
                flags.SetNodeFlag(NodeStatusFlags.TreeMeshNeedsUpdate);
                nodeFlags[treeNodeIndex] = flags;
            } 
            return true;
        }

        internal static bool CleanTree(int treeNodeID)
        {
            if (!IsValidNodeID(treeNodeID) || !AssertNodeType(treeNodeID, CSGNodeType.Tree))
                return false;

            var treeNodeIndex = treeNodeID - 1;
            var treeInfo = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;
            if (treeInfo == null)
                return false;

            var treeBrushes = treeInfo.treeBrushes;
            for (int b0 = 0; b0 < treeBrushes.Count; b0++)
            {
                var brush0NodeID = treeBrushes[b0];
                var output = CSGManager.GetBrushInfo(brush0NodeID);
                output.brushOutputLoops.Dispose();
                if (output.brushSurfaceLoops != null) output.brushSurfaceLoops.Dispose();
                output.brushSurfaceLoops = null;
            }
            return true;
        }

        #region Rebuild / Update
        static void UpdateBrushTransformation(int brushNodeID)
        {
            var brushNodeIndex				 = brushNodeID - 1;
            var parentNodeID				 = nodeHierarchies[brushNodeIndex].parentNodeID;
            var parentNodeIndex				 = parentNodeID - 1;
            var parentLocalTransformation	 = (parentNodeIndex < 0) ? Matrix4x4.identity : nodeLocalTransforms[parentNodeIndex].invLocalTransformation;
            var parentLocalInvTransformation = (parentNodeIndex < 0) ? Matrix4x4.identity : nodeLocalTransforms[parentNodeIndex].localTransformation;

            // TODO: should be transformations the way up to the tree, not just tree vs brush
            var brushLocalTransformation    = nodeLocalTransforms[brushNodeIndex].localTransformation;
            var brushLocalInvTransformation = nodeLocalTransforms[brushNodeIndex].invLocalTransformation;

            var nodeTransform				 = nodeTransforms[brushNodeIndex];
            nodeTransform.nodeToTree         = brushLocalTransformation * parentLocalInvTransformation;
            nodeTransform.treeToNode         = parentLocalTransformation * brushLocalInvTransformation;
            nodeTransforms[brushNodeIndex]	 = nodeTransform;

            if (ChiselLookup.Value.transformations.TryGetValue(brushNodeIndex, out BlobAssetReference<NodeTransformations> oldTransformations))
            {
                if (oldTransformations.IsCreated)
                    oldTransformations.Dispose();
            }
            ChiselLookup.Value.transformations[brushNodeIndex] = NodeTransformations.Build(nodeTransform.nodeToTree, nodeTransform.treeToNode);
        }

        static void UpdateBrushTransformations(List<int> treeBrushes)
        {
            // TODO: optimize, only do this when necessary
            for (int i = 0; i < treeBrushes.Count; i++)
            {
                UpdateBrushTransformation(treeBrushes[i]);
            }
        }

        static void FindBrushMeshInstanceIDs(NativeArray<int> treeBrushes, NativeArray<int> brushMeshInstanceIDs)
        {
            for (int i = 0; i < treeBrushes.Length; i++)
            {
                var brushNodeID = treeBrushes[i];
                var brushNodeIndex = brushNodeID - 1;
                brushMeshInstanceIDs[i] = nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID;
            }
        }

        static void UpdateBrushWorldPlanes(NativeArray<int> treeBrushes)
        {
            // TODO: optimize, only do this when necessary
            using (var brushMeshInstanceIDs = new NativeArray<int>(treeBrushes.Length, Allocator.TempJob))
            {
                FindBrushMeshInstanceIDs(treeBrushes, brushMeshInstanceIDs);
                var createBrushWorldPlanesJob = new CreateBrushWorldPlanesJob()
                {
                    treeBrushes             = treeBrushes,
                    brushMeshInstanceIDs    = brushMeshInstanceIDs,
                    transformations         = ChiselLookup.Value.transformations,
                    brushWorldPlanes        = ChiselLookup.Value.brushWorldPlanes.AsParallelWriter()
                };
                createBrushWorldPlanesJob.Run(treeBrushes.Length);
            }
        }

        static void GenerateBasePolygonLoops(NativeArray<int> treeBrushes)
        {
            // TODO: cache this

            // Generate base polygon loops
            for (int b = 0; b < treeBrushes.Length; b++)
            {
                var brushNodeID     = treeBrushes[b];
                var brush           = new CSGTreeBrush() { brushNodeID = brushNodeID };
                var output          = CSGManager.GetBrushInfo(brushNodeID);
                var outputLoops     = output.brushOutputLoops;
                outputLoops.Dispose();
                outputLoops.brush   = new CSGTreeBrush() { brushNodeID = brushNodeID };
                var bounds = CSGManagerPerformCSG.GenerateBasePolygons(brush, outputLoops);
                nodeBounds[brushNodeID - 1] = bounds;
            }
        }

        unsafe static void FillBrushIntersectionTestData(NativeArray<int> treeBrushes, NativeArray<BrushIntersectionTestData> brushData)
        {
            for (int b0 = 0; b0 < treeBrushes.Length; b0++)
            {
                var brush0NodeID    = treeBrushes[b0];
                var brush0          = new CSGTreeBrush { brushNodeID = brush0NodeID };
                var bounds0         = nodeBounds[brush0NodeID - 1];

                var treeToNode0SpaceMatrix = (float4x4)brush0.TreeToNodeSpaceMatrix;
                var nodeToTree0SpaceMatrix = (float4x4)brush0.NodeToTreeSpaceMatrix;

                var brushMesh0 = BrushMeshManager.GetBrushMeshBlob(brush0.BrushMesh.brushMeshID);
                brushData[b0] = new BrushIntersectionTestData()
                {
                    bounds                  = bounds0,
                    treeToNodeSpaceMatrix   = treeToNode0SpaceMatrix,
                    nodeToTreeSpaceMatrix   = nodeToTree0SpaceMatrix,
                    brushMesh               = brushMesh0
                };
            }
        }

        unsafe static void FindIntersectingBrushes(BlobAssetReference<CompactTree> compactTree, NativeArray<int> treeBrushes)
        {
            // Find intersecting brushes
            // TODO: do this at insertion/removal time (grouped, in update loop)
            // TODO: optimize, use hashed grid

            using (var brushData                = new NativeArray<BrushIntersectionTestData>(treeBrushes.Length, Allocator.TempJob))
            using (var intersectionResults      = new NativeStream(treeBrushes.Length * treeBrushes.Length, Allocator.TempJob))
            //using (var brushBrushIntersections  = new NativeMultiHashMap<int, BrushBrushIntersection>(treeBrushes.Length * treeBrushes.Length, Allocator.TempJob))
            {
                FillBrushIntersectionTestData(treeBrushes, brushData);

                int maxIndex = brushData.Length * brushData.Length;

                var intersectionTestJob = new IntersectionTestJob()
                {
                    brushData = brushData,
                    output = intersectionResults.AsWriter()
                    //output = brushBrushIntersections.AsParallelWriter()
                };
                intersectionTestJob.Run(maxIndex);
                
                var reader = intersectionResults.AsReader();

                int index = 0;
                reader.BeginForEachIndex(index++);
                while (reader.RemainingItemCount == 0 && index < maxIndex)
                    reader.BeginForEachIndex(index++);
                while (reader.RemainingItemCount > 0)
                {
                    var result = reader.Read<BrushBrushIntersection>();
                    var brush0NodeID = treeBrushes[result.brushNodeID0];
                    var brush1NodeID = treeBrushes[result.brushNodeID1];
                    var output0 = CSGManager.GetBrushInfo(brush0NodeID);
                    var output1 = CSGManager.GetBrushInfo(brush1NodeID);
                    if (result.type == IntersectionType.Intersection)
                    {
                        output0.brushBrushIntersections.Add(new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.Intersection });
                        output1.brushBrushIntersections.Add(new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.Intersection });
                    } else
                    if (result.type == IntersectionType.AInsideB)
                    {
                        output0.brushBrushIntersections.Add(new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.AInsideB });
                        output1.brushBrushIntersections.Add(new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.BInsideA });
                    } else
                    //if (intersectionType == IntersectionType.BInsideA)
                    {
                        output0.brushBrushIntersections.Add(new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.BInsideA });
                        output1.brushBrushIntersections.Add(new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.AInsideB });
                    }

                    while (reader.RemainingItemCount == 0 && index < maxIndex)
                        reader.BeginForEachIndex(index++);
                }

                for (int b = 0; b < treeBrushes.Length; b++)
                {
                    var brushNodeID = treeBrushes[b];
                    var brushBrushIntersections = GetBrushInfo(brushNodeID).brushBrushIntersections;
                    //using (var brushIntersections = brushBrushIntersections.GetValuesForKey(brushNodeID - 1))
                    {
                        var result = GenerateBrushesTouchedByBrush(compactTree, new CSGTreeBrush() { brushNodeID = brushNodeID }, brushBrushIntersections);
                        if (result.IsCreated)
                            ChiselLookup.Value.brushesTouchedByBrushes[brushNodeID - 1] = result;
                    }
                }
            }
        }

        // TODO: only do this when necessary (when brushes have been modified)
        static void UpdateBrushCategorizationTables(BlobAssetReference<CompactTree> compactTree, NativeArray<int> treeBrushes)
        {                
            // Build categorization trees for brushes
            var createRoutingTableJob = new CreateRoutingTableJob()
            {
                treeBrushes             = treeBrushes,
                brushesTouchedByBrushes = ChiselLookup.Value.brushesTouchedByBrushes,
                compactTree             = compactTree,
                operationTables         = OperationTables.Rows,
                routingTableLookup      = ChiselLookup.Value.routingTableLookup.AsParallelWriter()
            };
            createRoutingTableJob.Run(treeBrushes.Length);
        }

        static bool PerformAllCSG(NativeArray<int> treeBrushes)
        {
            // Perform CSG
            // TODO: only do this when necessary (when brushes have been modified)
            // TODO: determine when a brush is completely inside another brush
            //		 (might not have any intersection loops)
            // TODO: Cache the output surface meshes, only update when necessary

            for (int b = 0; b < treeBrushes.Length; b++)
            {
                var brushNodeID = treeBrushes[b];
                var output = CSGManager.GetBrushInfo(brushNodeID);

                if (output.brushOutputLoops.basePolygons.Count > 0)
                {
                    // TODO: get rid of this somehow
                    if (output.brushSurfaceLoops != null)
                        output.brushSurfaceLoops.Dispose();
                    output.brushSurfaceLoops = new SurfaceLoops(output.brushOutputLoops.basePolygons.Count);
                    PerformCSG(output.brushSurfaceLoops, new CSGTreeBrush { brushNodeID = brushNodeID });
                }

                // TODO: put somewhere else
                ChiselWireframe.UpdateOutline(brushNodeID);
            }
            return true;
        }

        static bool PerformCSG(SurfaceLoops categorizedLoopList, CSGTreeBrush brush)
        {
            Debug.Assert(brush.Valid);

            var brushNodeID         = brush.brushNodeID;
            var brushInstance       = CSGManager.GetBrushMeshID(brushNodeID);

            if (!BrushMeshManager.IsBrushMeshIDValid(brushInstance))
            {
                if (brushInstance != 0)
                    Debug.LogError($"!BrushMeshManager.IsBrushMeshIDValid({brushInstance})");
                return true;
            }

            var output      = CSGManager.GetBrushInfo(brushNodeID);
            var outputLoops = output.brushOutputLoops;

            if (outputLoops.basePolygons.Count == 0)
            {
                Debug.LogError("outputLoops.basePolygons.Count == 0");
                return false;
            }

            if (!ChiselLookup.Value.routingTableLookup.TryGetValue(brushNodeID - 1, out BlobAssetReference<RoutingTable> routingTable))
            {
                Debug.LogError("No routing table found");
                return false;
            }
            /*
            var routingTable        = output.routingTable;
            if (!routingTable.IsCreated)
            {
                Debug.LogError("No routing table found");
                return false;
            }*/

            ref var nodes           = ref routingTable.Value.nodes;
            ref var routingLookups  = ref routingTable.Value.routingLookups;
            var intersectionLoops   = output.brushOutputLoops.intersectionLoops;


            var allBrushSurfaces = categorizedLoopList.surfaces;

            Debug.Assert(routingLookups.Length != 0);

            for (int p = 0; p < outputLoops.basePolygons.Count; p++)
            {
                // Don't want to change the original loops so we copy them
                var newLoop = new Loop(outputLoops.basePolygons[p], CategoryGroupIndex.First);
                allBrushSurfaces[p].Add(newLoop);
            }

            var brushVertices = outputLoops.vertexSoup;
            for (int i = 0; i < routingLookups.Length; i++)
            {
                var routingLookup                   = routingLookups[i];
                var cuttingNodeIntersectionLoops    = intersectionLoops[i];
                for (int surfaceIndex = 0; surfaceIndex < allBrushSurfaces.Length; surfaceIndex++)
                {
                    var loopsOnBrushSurface     = allBrushSurfaces[surfaceIndex];
                    var intersectionLoop        = cuttingNodeIntersectionLoops?[surfaceIndex];
                    for (int l = loopsOnBrushSurface.Count - 1; l >= 0; l--)
                    {
                        var surfaceLoop = loopsOnBrushSurface[l];
                        if (!surfaceLoop.Valid)
                            continue;

                        if (!routingLookup.TryGetRoute(routingTable, surfaceLoop.info.interiorCategory, out CategoryRoutingRow routingRow))
                        {
                            Debug.Assert(false, "Could not find route");
                            continue;
                        }

                        bool overlap = intersectionLoop != null && BooleanEdgesUtility.AreLoopsOverlapping(surfaceLoop, intersectionLoop);

                        // Lookup categorization lookup between original surface & other surface ...
                        if (overlap)
                        {
                            // If we overlap don't bother with creating a new polygon + hole and reuse existing one
                            surfaceLoop.info.interiorCategory = routingRow[intersectionLoop.info.interiorCategory];
                            continue;
                        } else
                        {
                            surfaceLoop.info.interiorCategory = routingRow[CategoryIndex.Outside];
                        }

                        // Add all holes that share the same plane to the polygon
                        if (intersectionLoop != null)
                        {
                            // Categorize between original surface & intersection
                            var intersectionCategory = routingRow[intersectionLoop.info.interiorCategory];

                            // If the intersection polygon would get the same category, we don't need to do a pointless intersection
                            if (intersectionCategory == surfaceLoop.info.interiorCategory)
                                continue;

                            CSGManagerPerformCSG.Intersect(brushVertices, loopsOnBrushSurface, surfaceLoop, intersectionLoop, intersectionCategory);
                        }                        
                    }

                    // TODO: remove the need for this (check on insertion)
                    CSGManagerPerformCSG.RemoveEmptyLoops(loopsOnBrushSurface);
                }
            }

            CSGManagerPerformCSG.CleanUp(brushVertices, allBrushSurfaces);
            return true;
        }

        #endregion

        #region Reset/Rebuild
        static void Reset()
        {
            for (int t = 0; t < trees.Count; t++)
            {
                var treeNodeID = trees[t];
                if (!IsValidNodeID(treeNodeID) || !AssertNodeType(treeNodeID, CSGNodeType.Tree))
                    continue;

                var treeNodeIndex   = treeNodeID - 1;
                var treeInfo        = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;
                Reset(treeInfo);
            }
        }

        static bool Reset(TreeInfo treeInfo)
        {
            if (treeInfo == null)
                return false;
            
            var treeBrushes = treeInfo.treeBrushes;
            for (int t = 0; t < treeBrushes.Count; t++)
            {
                var brushNodeID     = treeBrushes[t];
                var brushNodeIndex  = brushNodeID - 1;
                var brushOutput     = CSGManager.nodeHierarchies[brushNodeIndex].brushInfo;
                if (brushOutput == null)
                    continue;
                brushOutput.Reset();
            }

            treeInfo.Reset();
            return true;
        }

        // TODO: rename
        internal static bool UpdateAllTreeMeshes()
        {
            bool needUpdate = false;
            // Check if we have a tree that needs updates
            for (int t = 0; t < trees.Count; t++)
            {
                var treeNodeID = trees[t];
                var treeNodeIndex = treeNodeID - 1;
                if (nodeFlags[treeNodeIndex].IsNodeFlagSet(NodeStatusFlags.TreeNeedsUpdate))
                {
                    needUpdate = true;
                    break;
                }
            }

            if (!needUpdate)
                return false;

            UpdateDelayedHierarchyModifications();

            for (int t = 0; t < trees.Count; t++)
                UpdateTreeMesh(trees[t]);

            // Check if we have a tree that actually has an updated mesh
            for (int t = 0; t < trees.Count; t++)
            {
                var treeNodeID = trees[t];
                var treeNodeIndex = treeNodeID - 1;

                if (nodeFlags[treeNodeIndex].IsNodeFlagSet(NodeStatusFlags.TreeMeshNeedsUpdate))
                    return true;
            }

            return false;
        }

        internal static void RebuildAll()
        {
            Reset();
            UpdateAllTreeMeshes();
        }
        #endregion
    }
#endif
}
