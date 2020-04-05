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

            // NOTE: THIS IS RUN **PER MODEL** AND WE HAVE MULTIPLE MODELS IN THE TEST SCENE
            // TODO: Make sure we complete for each model, or ensure that everything in model is stored separately

            OperationTables.EnsureInitialized(); // <-- store this in ChiselLookup & make blob

            var treeBrushes = treeInfo.treeBrushes;
            if (treeInfo.treeBrushes.Count > 0)
            {
                using (var treeBrushesArray = treeBrushes.ToNativeArray(Allocator.TempJob))
                {
                    ref var chiselLookupValues = ref ChiselLookup.Value;
                    // Clean up values we're rebuilding below
                    chiselLookupValues.RemoveByBrushID(treeBrushesArray);

                    using (var brushMeshInstanceIDs = new NativeArray<int>(treeBrushesArray.Length, Allocator.TempJob))
                    using (var brushMeshLookup = new NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>(treeBrushesArray.Length, Allocator.TempJob))
                    {
                        // TODO: somehow just keep this up to date instead of rebuilding it from scratch every update
                        FindBrushMeshInstanceIDs(treeBrushesArray, brushMeshInstanceIDs);
                        FindBrushMeshBobs(ref chiselLookupValues, treeBrushesArray, brushMeshLookup); // <-- assumes all brushes in tree


                        // TODO: only rebuild this when the hierarchy changes
                        var compactTree = CompactTree.Create(nodeHierarchies, treeNodeID - 1); // Note: stored/destroyed in ChiselLookup


                        // TODO: store this in blob / store with brush component itself & only update when transformations change
                        Profiler.BeginSample("UpdateBrushTransformations");
                        try
                        {
                            UpdateBrushTransformations(ref chiselLookupValues, treeBrushes);
                        } finally { Profiler.EndSample(); }


                        // TODO: should only do this once at creation time, part of brushMeshBlob? store with brush component itself
                        JobHandle generateBasePolygonLoopsJob;
/*+*/                   Profiler.BeginSample("GenerateBasePolygonLoops");
                        try
                        {
                            var createBlobPolygonsBlobs = new CreateBlobPolygonsBlobs()
                            {
                                // Read
                                treeBrushes             = treeBrushesArray,
                                brushMeshInstanceIDs    = brushMeshInstanceIDs,
                                brushMeshBlobs          = chiselLookupValues.brushMeshBlobs,
                                transformations         = chiselLookupValues.transformations,

                                // Write
                                basePolygons            = chiselLookupValues.basePolygons.AsParallelWriter()
                            };
                            generateBasePolygonLoopsJob = createBlobPolygonsBlobs.Schedule(treeBrushesArray.Length, 64);
                        } finally { Profiler.EndSample(); }

                        // TODO: should only do this at creation time + when moved / store with brush component itself
                        JobHandle updateBrushWorldPlanesJob;
/*+*/                   Profiler.BeginSample("UpdateBrushWorldPlanes");
                        try
                        {
                            // TODO: optimize, only do this when necessary
                            var createBrushWorldPlanesJob = new CreateBrushWorldPlanesJob()
                            {
                                // Read
                                treeBrushes         = treeBrushesArray,
                                brushMeshLookup     = brushMeshLookup,
                                transformations     = chiselLookupValues.transformations,

                                // Write
                                brushWorldPlanes    = chiselLookupValues.brushWorldPlanes.AsParallelWriter()
                            };
                            updateBrushWorldPlanesJob = createBrushWorldPlanesJob.Schedule(treeBrushesArray.Length, 64);
                        } finally { Profiler.EndSample(); }


                        // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
                        JobHandle findIntersectingBrushesJob;
/*+*/                   Profiler.BeginSample("FindIntersectingBrushes");
                        try
                        {
                            // TODO: optimize, use hashed grid

                            var triangleArraySize       = GeometryMath.GetTriangleArraySize(treeBrushesArray.Length);
                            var brushBrushIntersections = new NativeMultiHashMap<int, BrushPair>(triangleArraySize * 2, Allocator.TempJob);
                            var findAllIntersectionsJob = new FindAllIntersectionsJob()
                            {
                                // Read
                                brushNodeIDs            = treeBrushesArray,
                                transformations         = chiselLookupValues.transformations,
                                brushMeshBlobs          = chiselLookupValues.brushMeshBlobs,
                                basePolygons            = chiselLookupValues.basePolygons,
                                brushMeshIDs            = brushMeshInstanceIDs,

                                // Write
                                output                  = brushBrushIntersections.AsParallelWriter()
                            };
                            var findAllIntersectionsJobHandle = findAllIntersectionsJob.Schedule(triangleArraySize, 64, generateBasePolygonLoopsJob);

                            var storeBrushIntersectionsJob = new StoreBrushIntersectionsJob()
                            {
                                // Read
                                treeNodeIndex           = treeNodeIndex,
                                treeBrushes             = treeBrushesArray,
                                compactTree             = compactTree,
                                brushBrushIntersections = brushBrushIntersections,

                                // Write
                                brushesTouchedByBrushes = chiselLookupValues.brushesTouchedByBrushes.AsParallelWriter()
                            };
                            findIntersectingBrushesJob = storeBrushIntersectionsJob.Schedule(treeBrushesArray.Length, 64, findAllIntersectionsJobHandle);
                            brushBrushIntersections.Dispose(findIntersectingBrushesJob);
                        } finally { Profiler.EndSample(); }

                        // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
                        JobHandle updateBrushCategorizationTablesJob;
/*+*/                   Profiler.BeginSample("UpdateBrushCategorizationTables");
                        try
                        {
                            // Build categorization trees for brushes
                            var createRoutingTableJob = new CreateRoutingTableJob()
                            {
                                // Read
                                treeBrushes             = treeBrushesArray,
                                brushesTouchedByBrushes = chiselLookupValues.brushesTouchedByBrushes,
                                compactTree             = compactTree,
                                operationTables         = OperationTables.Rows,

                                // Write
                                routingTableLookup      = chiselLookupValues.routingTableLookup.AsParallelWriter()
                            };
                            updateBrushCategorizationTablesJob = createRoutingTableJob.Schedule(treeBrushesArray.Length, 64, findIntersectingBrushesJob);
                        } finally { Profiler.EndSample(); }

                        JobHandle findAllIntersectionLoopsJobHandle = default(JobHandle);
                        JobHandle findLoopOverlapIntersectionsJobHandle = default(JobHandle);
                        {
                            NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>> intersectionLoopBlobs;
                            
                            // Create unique loops between brush intersections
                            Profiler.BeginSample("FindAllIntersectionLoops");
                            try
                            {
                                intersectionLoopBlobs = new NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>(65500, Allocator.TempJob);
                                int maxPairs = treeBrushesArray.Length * treeBrushesArray.Length; // TODO: figure out a way to get a more exact number ...

                                // TODO: - calculate all data per brush here (currently unused)
                                //         and use this later on, hopefully making it possible to remove all intermediates
                                //       - also get rid of all SurfaceLoops etc. in the middle, only store at the end
                                //       - finally instead of storing stuff in SurfaceLoops at the end, store as blob

                                var uniqueBrushPairs = new NativeList<BrushPair>(maxPairs, Allocator.TempJob);
                                var findBrushPairsJob = new FindBrushPairsJob
                                {
                                    maxPairs                    = maxPairs,
                                    treeBrushes                 = treeBrushesArray,
                                    brushesTouchedByBrushes     = chiselLookupValues.brushesTouchedByBrushes,
                                    uniqueBrushPairs            = uniqueBrushPairs,
                                };
                                var findBrushPairsJobHandle = findBrushPairsJob.Schedule(findIntersectingBrushesJob);

                                var intersectingBrushes = new NativeList<BlobAssetReference<BrushPairIntersection>>(GeometryMath.GetTriangleArraySize(treeBrushesArray.Length), Allocator.TempJob);
                                var prepareBrushPairIntersectionsJob = new PrepareBrushPairIntersectionsJob
                                {
                                    uniqueBrushPairs        = uniqueBrushPairs.AsDeferredJobArray(),
                                    brushMeshBlobLookup     = brushMeshLookup,
                                    transformations         = chiselLookupValues.transformations,
                                    intersectingBrushes     = intersectingBrushes.AsParallelWriter()
                                };
                                var prepareBrushPairIntersectionsJobHandle = prepareBrushPairIntersectionsJob.Schedule(maxPairs, 64, findBrushPairsJobHandle);
                                var createBrushPairsJobHandle = JobHandle.CombineDependencies(prepareBrushPairIntersectionsJobHandle, findBrushPairsJobHandle);
                                uniqueBrushPairs.Dispose(createBrushPairsJobHandle);

                                var findAllIntersectionLoopsJob = new CSGManagerPerformCSG.FindAllIntersectionLoopsJob
                                {
                                    brushWorldPlanes    = chiselLookupValues.brushWorldPlanes,
                                    intersectingBrushes = intersectingBrushes.AsDeferredJobArray(),
                                    outputSurfaces      = intersectionLoopBlobs.AsParallelWriter()
                                };
                                findAllIntersectionLoopsJobHandle = findAllIntersectionLoopsJob.Schedule(maxPairs, 64, JobHandle.CombineDependencies(updateBrushWorldPlanesJob, createBrushPairsJobHandle));
                                findAllIntersectionLoopsJobHandle = JobHandle.CombineDependencies(findAllIntersectionLoopsJobHandle, prepareBrushPairIntersectionsJobHandle);

                                intersectingBrushes.Dispose(findAllIntersectionLoopsJobHandle); // TODO: can use handle on dispose
                            } finally { Profiler.EndSample(); }

                            Profiler.BeginSample("FindLoopOverlapIntersections");
                            try
                            {
                                var dependencies = JobHandle.CombineDependencies(findIntersectingBrushesJob, findAllIntersectionLoopsJobHandle, updateBrushWorldPlanesJob);

                                ref var brushWorldPlaneBlobs        = ref chiselLookupValues.brushWorldPlanes;
                                ref var basePolygonBlobs            = ref chiselLookupValues.basePolygons;
                                ref var vertexSoups                 = ref chiselLookupValues.vertexSoups;

                                for (int index = 0; index < treeBrushesArray.Length; index++)
                                {
                                    var intersectionSurfaceInfos    = new NativeList<SurfaceInfo>(0, Allocator.TempJob);
                                    var intersectionEdges           = new NativeListArray<Edge>(16, Allocator.TempJob);
                                    var basePolygonSurfaceInfos     = new NativeList<SurfaceInfo>(0, Allocator.TempJob);
                                    var basePolygonEdges            = new NativeListArray<Edge>(16, Allocator.TempJob);
                                    var vertexSoup                  = new VertexSoup(64, Allocator.TempJob);

                                    var findLoopOverlapIntersectionsJob = new FindLoopOverlapIntersectionsJob
                                    {
                                        index                       = index,

                                        treeBrushesArray            = treeBrushesArray,

                                        intersectionLoopBlobs       = intersectionLoopBlobs,
                        
                                        brushWorldPlanes            = brushWorldPlaneBlobs,
                                        basePolygonBlobs            = basePolygonBlobs,
                        
                                        vertexSoup                  = vertexSoup,
                                        basePolygonEdges            = basePolygonEdges,
                                        basePolygonSurfaceInfos     = basePolygonSurfaceInfos,
                                        intersectionEdges           = intersectionEdges,
                                        intersectionSurfaceInfos    = intersectionSurfaceInfos
                                    };
                                    findLoopOverlapIntersectionsJobHandle = JobHandle.CombineDependencies(findLoopOverlapIntersectionsJob.Schedule(dependencies), findLoopOverlapIntersectionsJobHandle);


                                    //***GET RID OF THIS***//
                                    var brushNodeID                 = treeBrushesArray[index];
                                    var brushNodeIndex              = brushNodeID - 1;

                                    vertexSoups[brushNodeIndex] = vertexSoup;

                                    var outputLoops = CSGManager.GetBrushInfo(brushNodeID).brushOutputLoops;/*!!*/
                                    outputLoops.Dispose();/*!!*/

                                    outputLoops.basePolygonSurfaceInfos     = basePolygonSurfaceInfos;
                                    outputLoops.basePolygonEdges            = basePolygonEdges;
                                    outputLoops.intersectionSurfaceInfos    = intersectionSurfaceInfos;
                                    outputLoops.intersectionEdges           = intersectionEdges;
                                    //***GET RID OF THIS***//
                                }
                                intersectionLoopBlobs.Dispose(JobHandle.CombineDependencies(findLoopOverlapIntersectionsJobHandle, findAllIntersectionLoopsJobHandle));
                            }
                            finally { Profiler.EndSample(); }
                        }



                        // TODO: use the above data to perform CSG
                        Profiler.BeginSample("PerformAllCSG");
                        try
                        {

                            //findLoopOverlapIntersectionsJobHandle.Complete();
                            //findAllIntersectionLoopsJobHandle.Complete();
                            //updateBrushCategorizationTablesJob.Complete();
                            //updateBrushWorldPlanesJob.Complete();
                            var dependencies = JobHandle.CombineDependencies(findAllIntersectionLoopsJobHandle, findLoopOverlapIntersectionsJobHandle, updateBrushCategorizationTablesJob);
                            dependencies.Complete();

                            PerformAllCSG(ref chiselLookupValues, treeBrushesArray, brushMeshInstanceIDs);
                        } finally { Profiler.EndSample(); }

                        Profiler.BeginSample("UpdateAllOutlines");
                        try
                        {
                            UpdateAllOutlines(treeBrushesArray);
                        }
                        finally { Profiler.EndSample(); }

                        Profiler.BeginSample("Complete");
                        try
                        {
                            generateBasePolygonLoopsJob.Complete();
                            findIntersectingBrushesJob.Complete();
                            findIntersectingBrushesJob.Complete();
                        } finally { Profiler.EndSample(); }
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
        static void UpdateBrushTransformation(ref NativeHashMap<int, BlobAssetReference<NodeTransformations>> transformations, int brushNodeID)
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

            if (transformations.TryGetValue(brushNodeIndex, out BlobAssetReference<NodeTransformations> oldTransformations))
            {
                if (oldTransformations.IsCreated)
                    oldTransformations.Dispose();
            }
            transformations[brushNodeIndex] = NodeTransformations.Build(nodeTransform.nodeToTree, nodeTransform.treeToNode);
        }

        static void UpdateBrushTransformations(ref ChiselLookup.Data chiselLookupValues, List<int> treeBrushes)
        {
            ref var transformations = ref chiselLookupValues.transformations;

            // TODO: optimize, only do this when necessary

            for (int i = 0; i < treeBrushes.Count; i++)
            {
                UpdateBrushTransformation(ref transformations, treeBrushes[i]);
            }
        }

        static void FindBrushMeshInstanceIDs(NativeArray<int> treeBrushesArray, NativeArray<int> brushMeshInstanceIDs)
        {
            for (int i = 0; i < treeBrushesArray.Length; i++)
            {
                var brushNodeID = treeBrushesArray[i];
                var brushNodeIndex = brushNodeID - 1;
                brushMeshInstanceIDs[i] = nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID;
            }
        }

        static void FindBrushMeshBobs(ref ChiselLookup.Data chiselLookupValues, NativeArray<int> treeBrushesArray, NativeHashMap<int, BlobAssetReference<BrushMeshBlob>> brushMeshLookup)
        {
            ref var brushMeshBlobs = ref chiselLookupValues.brushMeshBlobs;
            for (int i = 0; i < treeBrushesArray.Length; i++)
            {
                var brushNodeID = treeBrushesArray[i];
                var brushNodeIndex = brushNodeID - 1;
                brushMeshLookup[brushNodeIndex] = brushMeshBlobs[nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID - 1];
            }
        }


        static void PerformAllCSG(ref ChiselLookup.Data chiselLookupValues, NativeArray<int> treeBrushesArray, NativeArray<int> brushMeshInstanceIDs)
        {
            ref var routingTableLookup = ref chiselLookupValues.routingTableLookup;

            // Perform CSG
            // TODO: only do this when necessary (when brushes have been modified)
            // TODO: determine when a brush is completely inside another brush
            //		 (might not have any intersection loops)
            // TODO: Cache the output surface meshes, only update when necessary

            for (int b = 0; b < treeBrushesArray.Length; b++)
            {
                var brushNodeID     = treeBrushesArray[b];
                var brushNodeIndex  = brushNodeID - 1;
                var brushMeshID     = brushMeshInstanceIDs[b];

                var output      = CSGManager.GetBrushInfo(brushNodeID);
                var outputLoops = output.brushOutputLoops;

                // TODO: get rid of this somehow
                if (output.brushSurfaceLoops != null)
                    output.brushSurfaceLoops.Dispose();

                if (outputLoops.basePolygonSurfaceInfos.Length == 0)
                    continue;

                if (!routingTableLookup.TryGetValue(brushNodeIndex, out BlobAssetReference<RoutingTable> routingTable))
                    continue;

                var basePolygonEdges        = outputLoops.basePolygonEdges;
                var basePolygonSurfaceInfos = outputLoops.basePolygonSurfaceInfos;

                Loop[][] intersectionLoops;
                { 
                    ref var routingTableNodes = ref routingTable.Value.nodes;

                    var nodeIDtoIndex = new NativeHashMap<int, int>(routingTableNodes.Length, Allocator.TempJob);
                    for (int i = 0; i < routingTableNodes.Length; i++)
                        nodeIDtoIndex[routingTableNodes[i]] = i;

                    // TODO: Sort the brushSurfaceInfos/intersectionEdges based on nodeIDtoIndex[surfaceInfo.brushNodeID], 
                    //       have a sequential list of all data. 
                    //       Have segment list to determine which part of array belong to which brushNodeID
                    //       Don't need bottom part, can determine this in Job
                    
                    var surfaceCount                = outputLoops.basePolygonEdges.Length;
                    var intersectionEdges           = outputLoops.intersectionEdges;
                    var intersectionSurfaceInfos    = outputLoops.intersectionSurfaceInfos;
                    intersectionLoops           = new Loop[routingTableNodes.Length][];
                    for (int i = 0; i < intersectionSurfaceInfos.Length; i++)
                    {
                        var surfaceInfo     = intersectionSurfaceInfos[i];
                        var brushNodeIndex1 = surfaceInfo.brushNodeIndex;
                        var brushNodeID1    = brushNodeIndex1 + 1;

                        if (!nodeIDtoIndex.TryGetValue(brushNodeID1, out int brushIndex))
                            continue;

                        if (intersectionLoops[brushIndex] == null)
                            intersectionLoops[brushIndex] = new Loop[surfaceCount];
                        intersectionLoops[brushIndex][surfaceInfo.basePlaneIndex] = new Loop(intersectionEdges[i], surfaceInfo);
                    }

                    nodeIDtoIndex.Dispose();
                }

                {
                    output.brushSurfaceLoops = new SurfaceLoops(basePolygonSurfaceInfos.Length);
                    if (brushMeshID > 0)
                        PerformCSG(ref chiselLookupValues, output.brushSurfaceLoops, brushNodeID, intersectionLoops, basePolygonSurfaceInfos, basePolygonEdges);
                }

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
            }
        }

        static void UpdateAllOutlines(NativeArray<int> treeBrushesArray)
        {
            for (int b = 0; b < treeBrushesArray.Length; b++)
            {
                var brushNodeID = treeBrushesArray[b];
                // TODO: put somewhere else
                Profiler.BeginSample("ChiselWireframe.UpdateOutline");
                ChiselWireframe.UpdateOutline(brushNodeID);
                Profiler.EndSample();
            }
        }

        static bool PerformCSG(ref ChiselLookup.Data chiselLookupValues, SurfaceLoops categorizedLoopList, int brushNodeID, Loop[][] intersectionLoops,
                                NativeList<SurfaceInfo> basePolygonSurfaceInfos,
                                NativeListArray<Edge>   basePolygonEdges)
        {
            if (basePolygonSurfaceInfos.Length == 0)
            {
                Debug.LogError("basePolygonSurfaceInfos.Length == 0");
                return false;
            }

            if (!chiselLookupValues.routingTableLookup.TryGetValue(brushNodeID - 1, out BlobAssetReference<RoutingTable> routingTable))
            {
                Debug.LogError("No routing table found");
                return false;
            }

            ref var nodes           = ref routingTable.Value.nodes;
            ref var routingLookups  = ref routingTable.Value.routingLookups;
            

            var allBrushSurfaces = categorizedLoopList.surfaces;

            Debug.Assert(routingLookups.Length != 0);

            for (int p = 0; p < basePolygonSurfaceInfos.Length; p++)
            {
                // Don't want to change the original loops so we copy them
                var newLoop = new Loop()
                {
                    info    = basePolygonSurfaceInfos[p],
                    holes   = new List<Loop>()
                };
                newLoop.info.interiorCategory = CategoryGroupIndex.First;

                var edges = basePolygonEdges[p];
                for (int e = 0; e < edges.Length; e++)
                    newLoop.edges.Add(edges[e]);
                allBrushSurfaces[p].Add(newLoop);
            }

            var brushVertices = chiselLookupValues.vertexSoups[brushNodeID - 1];
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

                            Profiler.BeginSample("Intersect");
                            CSGManagerPerformCSG.Intersect(brushVertices, loopsOnBrushSurface, surfaceLoop, intersectionLoop, intersectionCategory);
                            Profiler.EndSample();
                        }                        
                    }

                    // TODO: remove the need for this (check on insertion)
                    Profiler.BeginSample("CSGManagerPerformCSG.RemoveEmptyLoops");
                    CSGManagerPerformCSG.RemoveEmptyLoops(loopsOnBrushSurface);
                    Profiler.EndSample();
                }
            }

            Profiler.BeginSample("CleanUp");
            CSGManagerPerformCSG.CleanUp(brushVertices, allBrushSurfaces);
            Profiler.EndSample();
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
            {
                var startTime = Time.realtimeSinceStartup;
                UnityEngine.Profiling.Profiler.BeginSample("UpdateTreeMesh");
                try
                {
                    UpdateTreeMesh(trees[t]);
                }
                finally { UnityEngine.Profiling.Profiler.EndSample(); }
                var endTime = Time.realtimeSinceStartup;
                Debug.Log($"    UpdateTreeMesh done in {((endTime - startTime) * 1000)} ms. ");
            }

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
