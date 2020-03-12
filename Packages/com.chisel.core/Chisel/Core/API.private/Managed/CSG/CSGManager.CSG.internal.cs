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
    internal sealed unsafe class ChiselLookup : ScriptableObject
    {
        public unsafe struct ChiselLookupValues
        {
            public NativeHashMap<int, BlobAssetReference<RoutingTable>> routingTableLookup;

            internal void DisposeOldRoutingTables(NativeArray<int> treeBrushes)
            {
                for (int b = 0; b < treeBrushes.Length; b++)
                {
                    var brushNodeID = treeBrushes[b];
                    if (routingTableLookup.TryGetValue(brushNodeID - 1, out BlobAssetReference<RoutingTable> item))
                    {
                        if (item.IsCreated)
                            item.Dispose();
                        routingTableLookup.Remove(brushNodeID - 1);
                    }
                }
            }


            internal void Initialize()
            {
                routingTableLookup = new NativeHashMap<int, BlobAssetReference<RoutingTable>>(8000, Allocator.Persistent);
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


        internal static bool UpdateTreeMesh(int treeNodeID)
        {
            if (!IsValidNodeID(treeNodeID) || !AssertNodeType(treeNodeID, CSGNodeType.Tree))
                return false;

            var treeNodeIndex = treeNodeID - 1;
            var treeInfo = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;
            if (treeInfo == null)
                return false;

            Reset(treeInfo);

            OperationTables.EnsureInitialized();

            var treeBrushes = treeInfo.treeBrushes;
            using (var treeBrushesArray = treeBrushes.ToNativeArray(Allocator.TempJob))
            {
                ChiselLookup.Value.DisposeOldRoutingTables(treeBrushesArray);

                Profiler.BeginSample("UpdateBrushTransformations"); try {       UpdateBrushTransformations(treeBrushes);                            } finally { Profiler.EndSample(); }

                // TODO: should only do this at creation time + when moved
                Profiler.BeginSample("UpdateBrushWorldPlanes"); try {           UpdateBrushWorldPlanes(treeBrushesArray);                           } finally { Profiler.EndSample(); }
                Profiler.BeginSample("FindIntersectingBrushes"); try {          FindIntersectingBrushes(treeBrushesArray);                          } finally { Profiler.EndSample(); }

                // TODO: should only do this once at creation time
                Profiler.BeginSample("GenerateBasePolygonLoops"); try {         GenerateBasePolygonLoops(treeBrushesArray);                         } finally { Profiler.EndSample(); }

                Profiler.BeginSample("FindAllIntersectionLoops"); try {         CSGManagerPerformCSG.FindAllIntersectionLoops(treeBrushesArray);    } finally { Profiler.EndSample(); }
                Profiler.BeginSample("UpdateBrushCategorizationTables"); try {  UpdateBrushCategorizationTables(treeBrushesArray);                  } finally { Profiler.EndSample(); }
                Profiler.BeginSample("PerformAllCSG"); try {                    PerformAllCSG(treeBrushesArray);                                    } finally { Profiler.EndSample(); }
            }

            OperationTables.EnsureCleanedUp(); 

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
        }

        static void UpdateBrushTransformations(List<int> treeBrushes)
        {
            // TODO: optimize, only do this when necessary
            for (int i = 0; i < treeBrushes.Count; i++)
            {
                UpdateBrushTransformation(treeBrushes[i]);
            }
        }

        static void UpdateBrushWorldPlanes(NativeArray<int> treeBrushes)
        {
            // TODO: optimize, only do this when necessary
            for (int i = 0; i < treeBrushes.Length; i++)
            {
                var brushNodeID = treeBrushes[i];
                var brushNodeIndex = brushNodeID - 1;
                var brushInfo = nodeHierarchies[brushNodeIndex].brushInfo;
                if (brushInfo.brushWorldPlanes.IsCreated)
                    brushInfo.brushWorldPlanes.Dispose();
                brushInfo.brushWorldPlanes = BrushWorldPlanes.BuildPlanes(BrushMeshManager.GetBrushMeshBlob(brushInfo.brushMeshInstanceID), nodeTransforms[brushNodeIndex].nodeToTree);
            }
        }

        static void GenerateBasePolygonLoops(NativeArray<int> treeBrushes)
        {
            // TODO: cache this

            // Generate base polygon loops
            for (int b = 0; b < treeBrushes.Length; b++)
            {
                var brushNodeID     = treeBrushes[b];
                var output          = CSGManager.GetBrushInfo(brushNodeID);
                var outputLoops     = output.brushOutputLoops;
                outputLoops.Dispose();
                outputLoops.brush   = new CSGTreeBrush() { brushNodeID = brushNodeID };
                nodeBounds[brushNodeID - 1] = CSGManagerPerformCSG.GenerateBasePolygons(outputLoops);
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

        unsafe static void FindIntersectingBrushes(NativeArray<int> treeBrushes)
        {
            // Find intersecting brushes
            // TODO: do this at insertion/removal time (grouped, in update loop)
            // TODO: optimize, use hashed grid

            using (var brushData            = new NativeArray<BrushIntersectionTestData>(treeBrushes.Length, Allocator.TempJob))
            using (var intersectionResults  = new NativeStream(treeBrushes.Length * treeBrushes.Length, Allocator.TempJob))
            {
                FillBrushIntersectionTestData(treeBrushes, brushData);

                int maxIndex = brushData.Length * brushData.Length;

                var intersectionTestJob = new IntersectionTestJob()
                {
                    brushData = brushData,
                    output = intersectionResults.AsWriter()
                };
                intersectionTestJob.Run(maxIndex); // FIXME: When I use "Run", it fails?

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
            }
        }

        static List<Loop[]> sIntersectionLoops = new List<Loop[]>();
        static void UpdateBrushCategorizationTables(NativeArray<int> treeBrushes)
        {
            var routingTableLookup = ChiselLookup.Value.routingTableLookup;
            // Build categorization trees for brushes
            // TODO: only do this when necessary (when brushes have been modified)
            for (int b = 0; b < treeBrushes.Length; b++)
            {
                using (var intersectionTypeLookup = new BrushIntersectionLookup(CSGManager.GetMaxNodeIndex(), Allocator.TempJob))
                {
                    var processedNode = new CSGTreeBrush() { brushNodeID = treeBrushes[b] };
                    var rootNode = processedNode.Tree;
                    CategoryStackNode.SetUsedNodesBits(processedNode, rootNode, in intersectionTypeLookup);
                    using (var routingTable = new NativeList<CategoryStackNode>(Allocator.TempJob))
                    {
                        CategoryStackNode.GetStack(in OperationTables.Rows, in intersectionTypeLookup, processedNode, rootNode, routingTable);
                    
                        var createRoutingTableJob = new CategoryStackNode.CreateRoutingTableJob()
                        {
                            routingTable = routingTable,
                            //treeBrushes = treeBrushes,
                            //index = b,
                            processedNode = processedNode,
                            //rootNode = rootNode,
                            //intersectionTypeLookup = intersectionTypeLookup,
                            //operationTables = OperationTables.Rows,
                            routingTableLookup = routingTableLookup.AsParallelWriter()
                        };

                        createRoutingTableJob.Run();
                    }
                }
            }

            for (int b = 0; b < treeBrushes.Length; b++)
            {
                var brushNodeID = treeBrushes[b];
                var brushInfo = GetBrushInfo(brushNodeID);
                if (!routingTableLookup.TryGetValue(brushNodeID - 1, out brushInfo.routingTable))
                {
                    brushInfo.routingTable = BlobAssetReference<RoutingTable>.Null;
                    continue;
                }
                brushInfo.routingTable = routingTableLookup[brushNodeID - 1];

                ref var nodes = ref brushInfo.routingTable.Value.nodes;
                sIntersectionLoops.Clear();
                for (int i = 0; i < nodes.Length; i++)
                {
                    var cutting_node_id = nodes[i];
                    // Get the intersection loops between the two brushes on every surface of the brush we're performing CSG on
                    if (!brushInfo.brushOutputLoops.intersectionLoopLookup.TryGetValue(cutting_node_id, out Loop[] cuttingNodeIntersectionLoops))
                        cuttingNodeIntersectionLoops = null;
                    sIntersectionLoops.Add(cuttingNodeIntersectionLoops);
                }
                brushInfo.brushOutputLoops.intersectionLoops = sIntersectionLoops.ToArray();
            }
        }

        static bool PerformAllCSG(NativeArray<int> treeBrushes)
        {
            // Perform CSG
            // TODO: only do this when necessary (when brushes have been modified)
            // TODO: use BrushInfo.brushTouch to determine when a brush is completely inside another brush
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
            
            var routingTable        = output.routingTable;
            if (!routingTable.IsCreated)
            {
                Debug.LogError("No routing table found");
                return false;
            }

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

                        // Determining if a surface overlaps with the baseLoop can be pre-generated
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
