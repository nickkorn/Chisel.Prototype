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

        internal static bool UpdateTreeMesh(int treeNodeID)
        {
            if (!IsValidNodeID(treeNodeID) || !AssertNodeType(treeNodeID, CSGNodeType.Tree))
                return false;

            var treeNodeIndex = treeNodeID - 1;
            var treeInfo = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;
            if (treeInfo == null)
                return false;

            Reset(treeInfo);

            OperationTables.EnsureInitialized(); // <-- store this in ChiselLookup & make blob

            var treeBrushes = treeInfo.treeBrushes;
            if (treeInfo.treeBrushes.Count > 0)
            {
                using (var treeBrushesArray = treeBrushes.ToNativeArray(Allocator.TempJob))
                {
                    // Clean up values we're rebuilding below
                    ChiselLookup.Value.RemoveByBrushID(treeBrushesArray);

                    using (var brushMeshInstanceIDs = new NativeArray<int>(treeBrushesArray.Length, Allocator.TempJob))
                    using (var brushMeshLookup = new NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>(treeBrushesArray.Length, Allocator.TempJob))
                    {
                        // TODO: somehow just keep this up to date instead of rebuilding it from scratch every update
                        FindBrushMeshInstanceIDs(treeBrushesArray, brushMeshInstanceIDs);
                        FindBrushMeshBobs(treeBrushesArray, brushMeshLookup); // <-- assumes all brushes in tree

                        // TODO: only rebuild this when the hierarchy changes
                        var compactTree = CompactTree.Create(nodeHierarchies, treeNodeID - 1); // Note: stored/destroyed in ChiselLookup


                        // TODO: store this in blob
                        Profiler.BeginSample("UpdateBrushTransformations"); try {       UpdateBrushTransformations(treeBrushes);                            } finally { Profiler.EndSample(); }

                        // TODO: should only do this once at creation time, part of brushMeshBlob?
/*+*/                   Profiler.BeginSample("GenerateBasePolygonLoops"); try {         GenerateBasePolygonLoops(treeBrushesArray, brushMeshInstanceIDs);   } finally { Profiler.EndSample(); }

                        // TODO: should only do this at creation time + when moved
/*+*/                   Profiler.BeginSample("UpdateBrushWorldPlanes"); try {           UpdateBrushWorldPlanes(treeBrushesArray, brushMeshLookup);     } finally { Profiler.EndSample(); }
/*+*/                   Profiler.BeginSample("FindIntersectingBrushes"); try {          FindIntersectingBrushes(treeNodeIndex, compactTree, treeBrushesArray, brushMeshInstanceIDs); } finally { Profiler.EndSample(); }

                        // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
/*+*/                   Profiler.BeginSample("UpdateBrushCategorizationTables"); try {  UpdateBrushCategorizationTables(compactTree, treeBrushesArray);     } finally { Profiler.EndSample(); }

                        Profiler.BeginSample("FindAllIntersectionLoops"); try {         CSGManagerPerformCSG.FindAllIntersectionLoops(treeBrushesArray, brushMeshLookup); } finally { Profiler.EndSample(); }
                        Profiler.BeginSample("FindLoopOverlapIntersections"); try {     CSGManagerPerformCSG.FindLoopOverlapIntersections(treeBrushesArray, brushMeshInstanceIDs); } finally { Profiler.EndSample(); }

                        // TODO: use the above data to perform CSG
                        Profiler.BeginSample("PerformAllCSG"); try {                    PerformAllCSG(treeBrushesArray, brushMeshInstanceIDs);             } finally { Profiler.EndSample(); }
                    }
                }
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

        static void FindBrushMeshBobs(NativeArray<int> treeBrushes, NativeHashMap<int, BlobAssetReference<BrushMeshBlob>> brushMeshLookup)
        {
            for (int i = 0; i < treeBrushes.Length; i++)
            {
                var brushNodeID = treeBrushes[i];
                var brushNodeIndex = brushNodeID - 1;
                brushMeshLookup[brushNodeIndex] = ChiselLookup.Value.brushMeshBlobs[nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID - 1];
            }
        }

        static void UpdateBrushWorldPlanes(NativeArray<int> treeBrushes, NativeHashMap<int, BlobAssetReference<BrushMeshBlob>> brushMeshLookup)
        {
            // TODO: optimize, only do this when necessary
            var createBrushWorldPlanesJob = new CreateBrushWorldPlanesJob()
            {
                treeBrushes         = treeBrushes,
                brushMeshLookup     = brushMeshLookup,
                transformations     = ChiselLookup.Value.transformations,
                brushWorldPlanes    = ChiselLookup.Value.brushWorldPlanes.AsParallelWriter()
            };
            createBrushWorldPlanesJob.Run(treeBrushes.Length);
        }

        // Generate base polygon loops
        static void GenerateBasePolygonLoops(NativeArray<int> treeBrushes, NativeArray<int> brushMeshInstanceIDs)
        {
            var createBlobPolygonsBlobs = new CreateBlobPolygonsBlobs()
            {
                treeBrushes             = treeBrushes,
                brushMeshInstanceIDs    = brushMeshInstanceIDs,
                brushMeshBlobs          = ChiselLookup.Value.brushMeshBlobs,
                transformations         = ChiselLookup.Value.transformations,
                basePolygons            = ChiselLookup.Value.basePolygons.AsParallelWriter()
            };
            createBlobPolygonsBlobs.Run(treeBrushes.Length);
        }

        // Find intersecting brushes
        unsafe static void FindIntersectingBrushes(int treeNodeIndex, BlobAssetReference<CompactTree> compactTree, NativeArray<int> treeBrushes, NativeArray<int> brushMeshInstanceIDs)
        {
            // TODO: only do this for brushes that have moved/changed shape
            // TODO: optimize, use hashed grid

            var triangleArraySize = GeometryMath.GetTriangleArraySize(treeBrushes.Length);

            using (var brushBrushIntersections  = new NativeMultiHashMap<int, BrushPair>(triangleArraySize * 2, Allocator.TempJob))
            {
                var findAllIntersectionsJob = new FindAllIntersectionsJob()
                {
                    brushNodeIDs            = treeBrushes,
                    transformations         = ChiselLookup.Value.transformations,
                    brushMeshBlobs          = ChiselLookup.Value.brushMeshBlobs,
                    basePolygons            = ChiselLookup.Value.basePolygons,
                    brushMeshIDs            = brushMeshInstanceIDs,
                    //bounds                = bounds,
                    output                  = brushBrushIntersections.AsParallelWriter()
                };
                findAllIntersectionsJob.Run(triangleArraySize);

                var storeBrushIntersectionsJob = new StoreBrushIntersectionsJob()
                {
                    treeNodeIndex           = treeNodeIndex,
                    treeBrushes             = treeBrushes,
                    compactTree             = compactTree,
                    brushBrushIntersections = brushBrushIntersections,
                    brushesTouchedByBrushes = ChiselLookup.Value.brushesTouchedByBrushes.AsParallelWriter()
                };

                storeBrushIntersectionsJob.Run(treeBrushes.Length);
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

        static bool PerformAllCSG(NativeArray<int> treeBrushes, NativeArray<int> brushMeshInstanceIDs)
        {
            // Perform CSG
            // TODO: only do this when necessary (when brushes have been modified)
            // TODO: determine when a brush is completely inside another brush
            //		 (might not have any intersection loops)
            // TODO: Cache the output surface meshes, only update when necessary

            for (int b = 0; b < treeBrushes.Length; b++)
            {
                var brushNodeID = treeBrushes[b];
                var brushMeshID = brushMeshInstanceIDs[b];

                var output = CSGManager.GetBrushInfo(brushNodeID);

                if (output.brushOutputLoops.basePolygons.Count > 0)
                {
                    // TODO: get rid of this somehow
                    if (output.brushSurfaceLoops != null)
                        output.brushSurfaceLoops.Dispose();
                    output.brushSurfaceLoops = new SurfaceLoops(output.brushOutputLoops.basePolygons.Count);
                    if (brushMeshID > 0)
                        PerformCSG(output.brushSurfaceLoops, brushNodeID);
                }

                // TODO: put somewhere else
                ChiselWireframe.UpdateOutline(brushNodeID);
            }
            return true;
        }

        static bool PerformCSG(SurfaceLoops categorizedLoopList, int brushNodeID)
        {
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

            var brushVertices = ChiselLookup.Value.vertexSoups[brushNodeID - 1];
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
