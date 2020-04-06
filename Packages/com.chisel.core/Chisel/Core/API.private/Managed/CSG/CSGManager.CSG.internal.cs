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
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    static partial class CSGManager
    {

        // TODO: review flags, might not make sense any more
        enum NodeStatusFlags : UInt16
        {
            None = 0,
            //			NeedChildUpdate				= 1,
            NeedPreviousSiblingsUpdate = 2,

            TreeIsDisabled = 1024,// TODO: remove, or make more useful
            OperationNeedsUpdate = 4,
            TreeNeedsUpdate = 8,
            TreeMeshNeedsUpdate = 16,

            NeedBaseMeshUpdate = 32,	// -> leads to NeedsMeshReset
            NeedMeshReset = 64,
            NeedOutlineUpdate = 128,
            NeedAllTouchingUpdated = 256,	// all brushes that touch this brush need to be updated,
            NeedFullUpdate = NeedBaseMeshUpdate | NeedMeshReset | NeedOutlineUpdate | NeedAllTouchingUpdated,
            NeedCSGUpdate = NeedBaseMeshUpdate | NeedMeshReset | NeedAllTouchingUpdated,
            NeedUpdate = NeedMeshReset | NeedOutlineUpdate | NeedAllTouchingUpdated,
            NeedUpdateDirectOnly = NeedMeshReset | NeedOutlineUpdate,
        };


        internal sealed class TreeInfo
        {
            public readonly List<int> treeBrushes = new List<int>();
            public readonly List<GeneratedMeshDescription> meshDescriptions = new List<GeneratedMeshDescription>();
            public readonly List<SubMeshCounts> subMeshCounts = new List<SubMeshCounts>();


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Reset()
            {
                subMeshCounts.Clear();
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
                        /*+*/
                        Profiler.BeginSample("GenerateBasePolygonLoops");
                        try
                        {
                            var createBlobPolygonsBlobs = new CreateBlobPolygonsBlobs()
                            {
                                // Read
                                treeBrushes = treeBrushesArray,
                                brushMeshInstanceIDs = brushMeshInstanceIDs,
                                brushMeshBlobs = chiselLookupValues.brushMeshBlobs,
                                transformations = chiselLookupValues.transformations,

                                // Write
                                basePolygons = chiselLookupValues.basePolygons.AsParallelWriter()
                            };
                            generateBasePolygonLoopsJob = createBlobPolygonsBlobs.Schedule(treeBrushesArray.Length, 64);
                        } finally { Profiler.EndSample(); }

                        // TODO: should only do this at creation time + when moved / store with brush component itself
                        JobHandle updateBrushWorldPlanesJob;
                        /*+*/
                        Profiler.BeginSample("UpdateBrushWorldPlanes");
                        try
                        {
                            // TODO: optimize, only do this when necessary
                            var createBrushWorldPlanesJob = new CreateBrushWorldPlanesJob()
                            {
                                // Read
                                treeBrushes = treeBrushesArray,
                                brushMeshLookup = brushMeshLookup,
                                transformations = chiselLookupValues.transformations,

                                // Write
                                brushWorldPlanes = chiselLookupValues.brushWorldPlanes.AsParallelWriter()
                            };
                            updateBrushWorldPlanesJob = createBrushWorldPlanesJob.Schedule(treeBrushesArray.Length, 64);
                        } finally { Profiler.EndSample(); }


                        // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
                        JobHandle findIntersectingBrushesJob;
                        /*+*/
                        Profiler.BeginSample("FindIntersectingBrushes");
                        try
                        {
                            // TODO: optimize, use hashed grid

                            var triangleArraySize = GeometryMath.GetTriangleArraySize(treeBrushesArray.Length);
                            var brushBrushIntersections = new NativeMultiHashMap<int, BrushPair>(triangleArraySize * 2, Allocator.TempJob);
                            var findAllIntersectionsJob = new FindAllIntersectionsJob()
                            {
                                // Read
                                brushNodeIDs = treeBrushesArray,
                                transformations = chiselLookupValues.transformations,
                                brushMeshBlobs = chiselLookupValues.brushMeshBlobs,
                                basePolygons = chiselLookupValues.basePolygons,
                                brushMeshIDs = brushMeshInstanceIDs,

                                // Write
                                output = brushBrushIntersections.AsParallelWriter()
                            };
                            var findAllIntersectionsJobHandle = findAllIntersectionsJob.Schedule(triangleArraySize, 64, generateBasePolygonLoopsJob);

                            var storeBrushIntersectionsJob = new StoreBrushIntersectionsJob()
                            {
                                // Read
                                treeNodeIndex = treeNodeIndex,
                                treeBrushes = treeBrushesArray,
                                compactTree = compactTree,
                                brushBrushIntersections = brushBrushIntersections,

                                // Write
                                brushesTouchedByBrushes = chiselLookupValues.brushesTouchedByBrushes.AsParallelWriter()
                            };
                            findIntersectingBrushesJob = storeBrushIntersectionsJob.Schedule(treeBrushesArray.Length, 64, findAllIntersectionsJobHandle);
                            brushBrushIntersections.Dispose(findIntersectingBrushesJob);
                        } finally { Profiler.EndSample(); }

                        // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
                        JobHandle updateBrushCategorizationTablesJob;
                        /*+*/
                        Profiler.BeginSample("UpdateBrushCategorizationTables");
                        try
                        {
                            // Build categorization trees for brushes
                            var createRoutingTableJob = new CreateRoutingTableJob()
                            {
                                // Read
                                treeBrushes = treeBrushesArray,
                                brushesTouchedByBrushes = chiselLookupValues.brushesTouchedByBrushes,
                                compactTree = compactTree,
                                operationTables = OperationTables.Rows,

                                // Write
                                routingTableLookup = chiselLookupValues.routingTableLookup.AsParallelWriter()
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
                                int maxPairs = GeometryMath.GetTriangleArraySize(treeBrushesArray.Length); // TODO: figure out a way to get a more exact number ...

                                var uniqueBrushPairs = new NativeList<BrushPair>(maxPairs, Allocator.TempJob);
                                var findBrushPairsJob = new FindBrushPairsJob
                                {
                                    maxPairs = maxPairs,
                                    treeBrushes = treeBrushesArray,
                                    brushesTouchedByBrushes = chiselLookupValues.brushesTouchedByBrushes,
                                    uniqueBrushPairs = uniqueBrushPairs,
                                };
                                var findBrushPairsJobHandle = findBrushPairsJob.Schedule(findIntersectingBrushesJob);

                                var intersectingBrushes = new NativeList<BlobAssetReference<BrushPairIntersection>>(GeometryMath.GetTriangleArraySize(treeBrushesArray.Length), Allocator.TempJob);
                                var prepareBrushPairIntersectionsJob = new PrepareBrushPairIntersectionsJob
                                {
                                    uniqueBrushPairs = uniqueBrushPairs.AsDeferredJobArray(),
                                    brushMeshBlobLookup = brushMeshLookup,
                                    transformations = chiselLookupValues.transformations,
                                    intersectingBrushes = intersectingBrushes.AsParallelWriter()
                                };
                                var prepareBrushPairIntersectionsJobHandle = prepareBrushPairIntersectionsJob.Schedule(maxPairs, 64, findBrushPairsJobHandle);
                                var createBrushPairsJobHandle = JobHandle.CombineDependencies(prepareBrushPairIntersectionsJobHandle, findBrushPairsJobHandle);
                                uniqueBrushPairs.Dispose(createBrushPairsJobHandle);

                                var findAllIntersectionLoopsJob = new CSGManagerPerformCSG.FindAllIntersectionLoopsJob
                                {
                                    brushWorldPlanes = chiselLookupValues.brushWorldPlanes,
                                    intersectingBrushes = intersectingBrushes.AsDeferredJobArray(),
                                    outputSurfaces = intersectionLoopBlobs.AsParallelWriter()
                                };
                                findAllIntersectionLoopsJobHandle = findAllIntersectionLoopsJob.Schedule(maxPairs, 64, JobHandle.CombineDependencies(updateBrushWorldPlanesJob, createBrushPairsJobHandle));
                                findAllIntersectionLoopsJobHandle = JobHandle.CombineDependencies(findAllIntersectionLoopsJobHandle, prepareBrushPairIntersectionsJobHandle);

                                intersectingBrushes.Dispose(findAllIntersectionLoopsJobHandle); // TODO: can use handle on dispose
                            } finally { Profiler.EndSample(); }

                            Profiler.BeginSample("FindLoopOverlapIntersections");
                            try
                            {
                                var dependencies = JobHandle.CombineDependencies(findIntersectingBrushesJob, findAllIntersectionLoopsJobHandle, updateBrushWorldPlanesJob);

                                ref var brushWorldPlaneBlobs = ref chiselLookupValues.brushWorldPlanes;
                                ref var basePolygonBlobs = ref chiselLookupValues.basePolygons;
                                ref var vertexSoups = ref chiselLookupValues.vertexSoups;

                                for (int index = 0; index < treeBrushesArray.Length; index++)
                                {
                                    var intersectionSurfaceInfos = new NativeList<SurfaceInfo>(0, Allocator.TempJob);
                                    var intersectionEdges = new NativeListArray<Edge>(16, Allocator.TempJob);
                                    var basePolygonSurfaceInfos = new NativeList<SurfaceInfo>(0, Allocator.TempJob);
                                    var basePolygonEdges = new NativeListArray<Edge>(16, Allocator.TempJob);
                                    var vertexSoup = new VertexSoup(64, Allocator.TempJob);

                                    var findLoopOverlapIntersectionsJob = new FindLoopOverlapIntersectionsJob
                                    {
                                        index = index,

                                        treeBrushesArray = treeBrushesArray,

                                        intersectionLoopBlobs = intersectionLoopBlobs,

                                        brushWorldPlanes = brushWorldPlaneBlobs,
                                        basePolygonBlobs = basePolygonBlobs,

                                        vertexSoup = vertexSoup,
                                        basePolygonEdges = basePolygonEdges,
                                        basePolygonSurfaceInfos = basePolygonSurfaceInfos,
                                        intersectionEdges = intersectionEdges,
                                        intersectionSurfaceInfos = intersectionSurfaceInfos
                                    };
                                    findLoopOverlapIntersectionsJobHandle = JobHandle.CombineDependencies(findLoopOverlapIntersectionsJob.Schedule(dependencies), findLoopOverlapIntersectionsJobHandle);


                                    //***GET RID OF THIS***//
                                    var brushNodeID = treeBrushesArray[index];
                                    var brushNodeIndex = brushNodeID - 1;

                                    vertexSoups[brushNodeIndex] = vertexSoup;

                                    var outputLoops = CSGManager.GetBrushInfo(brushNodeID).brushOutputLoops;/*!!*/
                                    outputLoops.Dispose();/*!!*/

                                    outputLoops.basePolygonSurfaceInfos = basePolygonSurfaceInfos;
                                    outputLoops.basePolygonEdges = basePolygonEdges;
                                    outputLoops.intersectionSurfaceInfos = intersectionSurfaceInfos;
                                    outputLoops.intersectionEdges = intersectionEdges;
                                    //***GET RID OF THIS***//
                                }
                                intersectionLoopBlobs.Dispose(JobHandle.CombineDependencies(findLoopOverlapIntersectionsJobHandle, findAllIntersectionLoopsJobHandle));
                            }
                            finally { Profiler.EndSample(); }
                        }

                        Profiler.BeginSample("PerformAllCSG");
                        try
                        {
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
            var brushNodeIndex = brushNodeID - 1;
            var parentNodeID = nodeHierarchies[brushNodeIndex].parentNodeID;
            var parentNodeIndex = parentNodeID - 1;
            var parentLocalTransformation = (parentNodeIndex < 0) ? Matrix4x4.identity : nodeLocalTransforms[parentNodeIndex].invLocalTransformation;
            var parentLocalInvTransformation = (parentNodeIndex < 0) ? Matrix4x4.identity : nodeLocalTransforms[parentNodeIndex].localTransformation;

            // TODO: should be transformations the way up to the tree, not just tree vs brush
            var brushLocalTransformation = nodeLocalTransforms[brushNodeIndex].localTransformation;
            var brushLocalInvTransformation = nodeLocalTransforms[brushNodeIndex].invLocalTransformation;

            var nodeTransform = nodeTransforms[brushNodeIndex];
            nodeTransform.nodeToTree = brushLocalTransformation * parentLocalInvTransformation;
            nodeTransform.treeToNode = parentLocalTransformation * brushLocalInvTransformation;
            nodeTransforms[brushNodeIndex] = nodeTransform;

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
                var brushNodeID = treeBrushesArray[b];
                var brushNodeIndex = brushNodeID - 1;
                var brushMeshID = brushMeshInstanceIDs[b];

                var output = CSGManager.GetBrushInfo(brushNodeID);
                var outputLoops = output.brushOutputLoops;

                // TODO: get rid of this somehow
                if (output.brushSurfaceLoops != null)
                    output.brushSurfaceLoops.Dispose();

                if (brushMeshID == 0 ||
                    outputLoops.basePolygonSurfaceInfos.Length == 0)
                    continue;

                if (!routingTableLookup.TryGetValue(brushNodeIndex, out BlobAssetReference<RoutingTable> routingTable))
                {
                    Debug.LogError("No routing table found");
                    continue;
                }

                var basePolygonEdges = outputLoops.basePolygonEdges;
                var basePolygonSurfaceInfos = outputLoops.basePolygonSurfaceInfos;

                NativeListArray<Edge> intersectionLoops;
                NativeArray<SurfaceInfo> intersectionSurfaceInfo;
                {
                    ref var routingTableNodes = ref routingTable.Value.nodes;

                    var nodeIDtoIndex = new NativeHashMap<int, int>(routingTableNodes.Length, Allocator.TempJob);
                    for (int i = 0; i < routingTableNodes.Length; i++)
                        nodeIDtoIndex[routingTableNodes[i]] = i;

                    // TODO: Sort the brushSurfaceInfos/intersectionEdges based on nodeIDtoIndex[surfaceInfo.brushNodeID], 
                    //       have a sequential list of all data. 
                    //       Have segment list to determine which part of array belong to which brushNodeID
                    //       Don't need bottom part, can determine this in Job

                    var surfaceCount = outputLoops.basePolygonEdges.Length;
                    var intersectionEdges = outputLoops.intersectionEdges;
                    var intersectionSurfaceInfos = outputLoops.intersectionSurfaceInfos;
                    intersectionLoops = new NativeListArray<Edge>(16, Allocator.TempJob);
                    intersectionSurfaceInfo = new NativeArray<SurfaceInfo>(routingTableNodes.Length * surfaceCount, Allocator.TempJob);
                    intersectionLoops.ResizeExact(routingTableNodes.Length * surfaceCount);
                    for (int i = 0; i < intersectionSurfaceInfos.Length; i++)
                    {
                        var surfaceInfo = intersectionSurfaceInfos[i];
                        var brushNodeIndex1 = surfaceInfo.brushNodeIndex;
                        var brushNodeID1 = brushNodeIndex1 + 1;

                        if (!nodeIDtoIndex.TryGetValue(brushNodeID1, out int brushIndex))
                            continue;

                        int offset = (brushIndex * surfaceCount) + surfaceInfo.basePlaneIndex;
                        intersectionLoops[offset].AddRange(intersectionEdges[i]);
                        intersectionSurfaceInfo[offset] = surfaceInfo;
                    }

                    nodeIDtoIndex.Dispose();
                }

                {
                    // TODO: separate storage of holes, from their polygons, so we can make this native
                    //              have list of loops per surface + list of indices for loops and indices of holes
                    // TODO: turn this into a Native equiv.
                    output.brushSurfaceLoops = new BrushLoops(basePolygonSurfaceInfos.Length);/*!!*/
                    var brushVertices = chiselLookupValues.vertexSoups[brushNodeIndex];/*!!*/
                    PerformCSG(output.brushSurfaceLoops, routingTable, brushVertices, intersectionLoops, intersectionSurfaceInfo, basePolygonSurfaceInfos, basePolygonEdges);
                }

                intersectionLoops.Dispose();
                intersectionSurfaceInfo.Dispose();


                // TODO: generate surface triangles (blobAssets), use brushSurfaceLoops, 
                //       remove from BrushInfo

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

        static bool PerformCSG(BrushLoops                       brushLoops,  /*!!*/
                               BlobAssetReference<RoutingTable> routingTable,
                               VertexSoup                       brushVertices,
                               NativeListArray<Edge>            intersectionLoops,
                               NativeArray<SurfaceInfo>         intersectionSurfaceInfo,
                               NativeList<SurfaceInfo>          basePolygonSurfaceInfos,
                               NativeListArray<Edge>            basePolygonEdges)
        {
            ref var nodes = ref routingTable.Value.nodes;
            ref var routingLookups = ref routingTable.Value.routingLookups;

            ref var chiselLookupValues = ref ChiselLookup.Value;
            var brushWorldPlanes = chiselLookupValues.brushWorldPlanes;
            var brushesTouchedByBrushes = chiselLookupValues.brushesTouchedByBrushes;


            var surfaceCount = basePolygonSurfaceInfos.Length;

            Debug.Assert(routingLookups.Length != 0);

            for (int s = 0; s < surfaceCount; s++)
            {
                var info = basePolygonSurfaceInfos[s];
                info.interiorCategory = CategoryGroupIndex.First;

                var surfaceLoops = brushLoops.surfaces[s];/*!!*/
                surfaceLoops.loopIndices.Add(0);
                surfaceLoops.holeIndices.Add();
                surfaceLoops.allInfos.Add(info);
                surfaceLoops.allEdges.Add(basePolygonEdges[s]);
            }

            for (int i = 0, offset = 0; i < routingLookups.Length; i++)
            {
                ref var routingLookup = ref routingLookups[i];
                for (int surfaceIndex = 0; surfaceIndex < surfaceCount; surfaceIndex++, offset++)
                {
                    var surfaceLoops = brushLoops.surfaces[surfaceIndex];/*!!*/
                    var intersectionLoop = intersectionLoops[offset];
                    var intersectionInfo = intersectionSurfaceInfo[offset];
                    for (int l = surfaceLoops.loopIndices.Length - 1; l >= 0; l--)
                    {
                        var surfaceLoopIndex = surfaceLoops.loopIndices[l];
                        var surfaceLoopEdges = surfaceLoops.allEdges[surfaceLoopIndex];
                        if (surfaceLoopEdges.Length < 3)
                            continue;

                        var surfaceLoopInfo = surfaceLoops.allInfos[surfaceLoopIndex];
                        var currentInteriorCategory = surfaceLoopInfo.interiorCategory;

                        if (!routingLookup.TryGetRoute(routingTable, currentInteriorCategory, out CategoryRoutingRow routingRow))
                        {
                            Debug.Assert(false, "Could not find route");
                            continue;
                        }

                        bool overlap = intersectionLoop.Length != 0 &&
                                        BooleanEdgesUtility.AreLoopsOverlapping(surfaceLoopEdges, intersectionLoop);

                        // Lookup categorization lookup between original surface & other surface ...
                        if (overlap)
                        {
                            // If we overlap don't bother with creating a new polygon + hole and reuse existing one
                            surfaceLoopInfo.interiorCategory = routingRow[intersectionInfo.interiorCategory];
                            surfaceLoops.allInfos[surfaceLoopIndex] = surfaceLoopInfo;
                            continue;
                        } else
                        {
                            surfaceLoopInfo.interiorCategory = routingRow[CategoryIndex.Outside];
                            surfaceLoops.allInfos[surfaceLoopIndex] = surfaceLoopInfo;
                        }

                        // Add all holes that share the same plane to the polygon
                        if (intersectionLoop.Length != 0)
                        {
                            // Categorize between original surface & intersection
                            var intersectionCategory = routingRow[intersectionInfo.interiorCategory];

                            // If the intersection polygon would get the same category, we don't need to do a pointless intersection
                            if (intersectionCategory == currentInteriorCategory)
                                continue;

                            Profiler.BeginSample("Intersect");
                            var intersectLoopsJob = new IntersectLoopsJob
                            {
                                brushVertices = brushVertices,
                                brushWorldPlanes = brushWorldPlanes,
                                brushesTouchedByBrushes = brushesTouchedByBrushes,
                                l = l,
                                surfaceLoopIndex = surfaceLoopIndex,
                                intersectionLoop = intersectionLoop,
                                newHoleCategory = intersectionCategory,

                                intersectionInfo = intersectionInfo,
                                surfaceLoops = surfaceLoops
                            };
                            intersectLoopsJob.Run();
                            Profiler.EndSample();
                        }
                    }

                    // TODO: remove the need for this (check on insertion)
                    Profiler.BeginSample("CSGManagerPerformCSG.RemoveEmptyLoops");
                    var removeEmptyLoopsJob = new RemoveEmptyLoopsJob
                    {
                        surfaceLoops = surfaceLoops
                    };
                    removeEmptyLoopsJob.Run();
                    Profiler.EndSample();
                }
            }

            Profiler.BeginSample("CleanUp");
            {
                for (int surfaceIndex = 0; surfaceIndex < brushLoops.surfaces.Length; surfaceIndex++)
                {
                    var cleanUpJob = new CleanUpJob
                    {
                        brushVertices       = brushVertices,
                        brushWorldPlanes    = brushWorldPlanes,
                        surfaceLoops        = brushLoops.surfaces[surfaceIndex]/*!!*/
                    };
                    cleanUpJob.Run();
                }
            }
            Profiler.EndSample();
            return true;
        }


        [BurstCompile(CompileSynchronously = true)]
        unsafe struct IntersectLoopsJob : IJob
        {
            [NoAlias, ReadOnly] public VertexSoup brushVertices;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>> brushWorldPlanes;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>> brushesTouchedByBrushes;
            [NoAlias, ReadOnly] public int l;
            [NoAlias, ReadOnly] public int surfaceLoopIndex;
            [NoAlias, ReadOnly] public NativeListArray<Edge>.NativeList intersectionLoop;
            [NoAlias, ReadOnly] public CategoryGroupIndex newHoleCategory;

            [NoAlias] public SurfaceInfo intersectionInfo;
            [NoAlias] public SurfaceLoops surfaceLoops;

            public void Execute()
            {
                var currentLoopEdges = surfaceLoops.allEdges[surfaceLoopIndex];
                var currentInfo = surfaceLoops.allInfos[surfaceLoopIndex];
                var currentHoleIndices = surfaceLoops.holeIndices[l];

                // It might look like we could just set the interiorCategory of brush_intersection here, and let all other cut loops copy from it below,
                // but the same brush_intersection might be used by another categorized_loop and then we'd try to reroute it again, which wouldn't work
                //brush_intersection.interiorCategory = newHoleCategory;

                var outEdges = new NativeList<Edge>(math.max(intersectionLoop.Length, currentLoopEdges.Length), Allocator.Temp);

                var result = OperationResult.Fail;
                var intersectEdgesJob = new IntersectEdgesJob
                {
                    vertices = brushVertices,
                    edges1 = intersectionLoop,
                    edges2 = currentLoopEdges,
                    worldPlanes1 = brushWorldPlanes[intersectionInfo.brushNodeIndex],
                    worldPlanes2 = brushWorldPlanes[currentInfo.brushNodeIndex],

                    result = &result,
                    outEdges = outEdges
                };
                intersectEdgesJob.Execute();

                // *somehow* put whats below in a job

                if (result == OperationResult.Outside ||
                    result == OperationResult.Fail)
                {
                    outEdges.Dispose();
                } else
                if (result == OperationResult.Polygon2InsidePolygon1)
                {
                    // This new piece overrides the current loop
                    currentLoopEdges.Clear();
                    currentLoopEdges.AddRange(outEdges);
                    currentInfo.interiorCategory = newHoleCategory;
                    surfaceLoops.allInfos[surfaceLoopIndex] = currentInfo;
                    outEdges.Dispose();
                } else
                if (result == OperationResult.Overlapping)
                {
                    outEdges.Dispose();
                    currentInfo.interiorCategory = newHoleCategory;
                    surfaceLoops.allInfos[surfaceLoopIndex] = currentInfo;
                } else
                {
                    // FIXME: when brush_intersection and categorized_loop are grazing each other, 
                    //          technically we cut it but we shouldn't be creating it as a separate polygon + hole (bug7)

                    // the output of cutting operations are both holes for the original polygon (categorized_loop)
                    // and new polygons on the surface of the brush that need to be categorized
                    intersectionInfo.interiorCategory = newHoleCategory;
                    var intersectedHoleIndices = new NativeList<int>(Allocator.Temp);

                    // the output of cutting operations are both holes for the original polygon (categorized_loop)
                    // and new polygons on the surface of the brush that need to be categorized
                    if (currentHoleIndices.Capacity < currentHoleIndices.Length + 1)
                        currentHoleIndices.Capacity = currentHoleIndices.Length + 1;

                    if (currentHoleIndices.Length > 0)
                    {
                        ref var brushIntersections = ref brushesTouchedByBrushes[currentInfo.brushNodeIndex].Value.brushIntersections;
                        for (int h = 0; h < currentHoleIndices.Length; h++)
                        {
                            // Need to make a copy so we can edit it without causing side effects
                            var holeIndex = currentHoleIndices[h];
                            var holeEdges = surfaceLoops.allEdges[holeIndex];
                            if (holeEdges.Length < 3)
                                continue;

                            var holeInfo = surfaceLoops.allInfos[holeIndex];
                            var holeBrushNodeID = holeInfo.brushNodeIndex + 1;

                            // TODO: Optimize and make this a BlobAsset that's created in a pass,
                            //       this BlobAsset must make it easy to quickly determine if two
                            //       brushes intersect. Use this for both routing table generation 
                            //       and loop merging.
                            bool touches = false;
                            for (int t = 0; t < brushIntersections.Length; t++)
                            {
                                if (brushIntersections[t].nodeIndex == holeBrushNodeID)
                                {
                                    touches = true;
                                    break;
                                }
                            }

                            // But only if they touch
                            if (touches)
                            {
                                intersectedHoleIndices.Add(surfaceLoops.allEdges.Length);
                                surfaceLoops.allInfos.Add(holeInfo);

                                {
                                    surfaceLoops.allEdges.Add(holeEdges);
                                }
                            }
                        }
                    }

                    // TODO: Separate loop "shapes" from category/loop-hole hierarchy, 
                    //       so we can simply assign the same shape to a hole and loop without 
                    //       needing to copy data we can create a new shape when we modify it.

                    // this loop is a hole 
                    currentHoleIndices.Add(surfaceLoops.allEdges.Length);
                    surfaceLoops.allInfos.Add(intersectionInfo);
                    {
                        surfaceLoops.allEdges.Add(outEdges);
                    }

                    // but also a polygon on its own
                    surfaceLoops.loopIndices.Add(surfaceLoops.allEdges.Length);
                    surfaceLoops.holeIndices.Add(intersectedHoleIndices);
                    surfaceLoops.allInfos.Add(intersectionInfo);
                    {
                        surfaceLoops.allEdges.Add(outEdges);
                    }

                    intersectedHoleIndices.Dispose();
                    outEdges.Dispose();
                }
            }

        }

        [BurstCompile(CompileSynchronously = true)]
        unsafe struct RemoveEmptyLoopsJob : IJob
        {
            [NoAlias] public SurfaceLoops surfaceLoops;

            public void Execute()
            {
                for (int l = surfaceLoops.loopIndices.Length - 1; l >= 0; l--)
                {
                    var loopIndex = surfaceLoops.loopIndices[l];
                    var loopEdges = surfaceLoops.allEdges[loopIndex];
                    if (loopEdges.Length < 3)
                    {
                        surfaceLoops.loopIndices.RemoveAtSwapBack(l);
                        break;
                    }

                    var holeIndices = surfaceLoops.holeIndices[l];
                    for (int h = holeIndices.Length - 1; h >= 0; h--)
                    {
                        var holeIndex = holeIndices[h];
                        var holeEdges = surfaceLoops.allEdges[holeIndex];
                        if (holeEdges.Length < 3)
                        {
                            holeIndices.RemoveAtSwapBack(h);
                            break;
                        }
                    }
                }
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        unsafe struct CleanUpJob : IJob
        {
            [NoAlias, ReadOnly] public VertexSoup      brushVertices;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>> brushWorldPlanes;
            [NoAlias] public SurfaceLoops surfaceLoops;

            struct Empty {}

            [BurstDiscard]
            private static void NotUniqueEdgeException()
            {
                throw new Exception("Edge is not unique");
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool AddEdges(NativeListArray<Edge>.NativeList edges, NativeListArray<Edge>.NativeList addEdges, bool removeDuplicates = false)
            {
                if (addEdges.Length == 0)
                    return false;

                var uniqueEdges = new NativeHashMap<Edge, Empty>(addEdges.Length, Allocator.Temp);
                uniqueEdges.Clear();
                for (int e = 0; e < addEdges.Length; e++)
                {
                    if (!uniqueEdges.TryAdd(addEdges[e], new Empty()))
                        NotUniqueEdgeException();
                }

                if (removeDuplicates)
                {
                    for (int e = edges.Length - 1; e >= 0; e--)
                    {
                        if (uniqueEdges.Remove(edges[e]))
                        {
                            edges.RemoveAtSwapBack(e);
                        }
                    }
                }

                bool duplicates = false;

                var values = uniqueEdges.GetKeyArray(Allocator.Temp);
                uniqueEdges.Dispose();
                for (int v= 0;v<values.Length;v++)
                {
                    var index = IndexOf(edges, values[v], out bool _);
                    if (index != -1)
                    {
                        //Debug.Log($"Duplicate edge {inverted}  {values[v].index1}/{values[v].index2} {edges[index].index1}/{edges[index].index2}");
                        duplicates = true;
                        continue;
                    }
                    edges.Add(values[v]);
                }
                values.Dispose();

                return duplicates;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static int IndexOf(NativeListArray<Edge>.NativeList edges, Edge edge, out bool inverted)
            {
                //var builder = new System.Text.StringBuilder();
                for (int e = 0; e < edges.Length; e++)
                {
                    //builder.AppendLine($"{e}/{edges.Count}: {edges[e]} {edge}");
                    if (edges[e].index1 == edge.index1 && edges[e].index2 == edge.index2) { inverted = false; return e; }
                    if (edges[e].index1 == edge.index2 && edges[e].index2 == edge.index1) { inverted = true;  return e; }
                }
                //Debug.Log(builder.ToString());
                inverted = false;
                return -1;
            }

            public void Execute()
            {
                for (int l = surfaceLoops.loopIndices.Length - 1; l >= 0; l--)
                {
                    var baseloopIndex   = surfaceLoops.loopIndices[l];
                    var baseLoopEdges   = surfaceLoops.allEdges[baseloopIndex];
                    if (baseLoopEdges.Length < 3)
                    {
                        baseLoopEdges.Clear();
                        continue;
                    }

                    var baseLoopNormal = CSGManagerPerformCSG.CalculatePlaneEdges(baseLoopEdges, brushVertices);
                    if (math.all(baseLoopNormal == float3.zero))
                    {
                        baseLoopEdges.Clear();
                        continue;
                    }

                    var holeIndices = surfaceLoops.holeIndices[l];
                    if (holeIndices.Length == 0)
                        continue;

                    var baseLoopInfo = surfaceLoops.allInfos[baseloopIndex];
                
                    for (int h = holeIndices.Length - 1; h >= 0; h--)
                    {
                        var holeIndex   = holeIndices[h];
                        var holeEdges   = surfaceLoops.allEdges[holeIndex];
                        if (holeEdges.Length < 3)
                        {
                            holeIndices.RemoveAtSwapBack(h);
                            continue;
                        }
                        var holeNormal = CSGManagerPerformCSG.CalculatePlaneEdges(holeEdges, brushVertices);
                        if (math.all(holeNormal == float3.zero))
                        {
                            holeIndices.RemoveAtSwapBack(h);
                            continue;
                        }
                    }
                    if (holeIndices.Length == 0)
                        continue;

                    var allWorldPlanes   = new NativeList<float4>(Allocator.Temp);
                    var allSegments      = new NativeList<LoopSegment>(Allocator.Temp);
                    var allEdges         = new NativeList<Edge>(Allocator.Temp);
                    {
                        int edgeOffset = 0;
                        int planeOffset = 0;
                        for (int h = 0; h < holeIndices.Length; h++)
                        {
                            var holeIndex   = holeIndices[h];
                            var holeInfo    = surfaceLoops.allInfos[holeIndex];
                            var holeEdges   = surfaceLoops.allEdges[holeIndex];

                            // TODO: figure out why sometimes polygons are flipped around, and try to fix this at the source
                            var holeNormal  = CSGManagerPerformCSG.CalculatePlaneEdges(holeEdges, brushVertices);
                            if (math.dot(holeNormal, baseLoopNormal) > 0)
                            {
                                for (int n = 0; n < holeEdges.Length; n++)
                                {
                                    var holeEdge = holeEdges[n];
                                    var i1 = holeEdge.index1;
                                    var i2 = holeEdge.index2;
                                    holeEdge.index1 = i2;
                                    holeEdge.index2 = i1;
                                    holeEdges[n] = holeEdge;
                                }
                            }

                            ref var worldPlanes = ref brushWorldPlanes[holeInfo.brushNodeIndex].Value.worldPlanes;
                        
                            var edgesLength     = holeEdges.Length;
                            var planesLength    = worldPlanes.Length;

                            allSegments.Add(new LoopSegment()
                            {
                                edgeOffset      = edgeOffset,
                                edgeLength      = edgesLength,
                                planesOffset    = planeOffset,
                                planesLength    = planesLength
                            });

                            allEdges.AddRange(holeEdges);

                            // TODO: ideally we'd only use the planes that intersect our edges
                            allWorldPlanes.AddRange(worldPlanes.GetUnsafePtr(), planesLength);

                            edgeOffset += edgesLength;
                            planeOffset += planesLength;
                        }
                        if (baseLoopEdges.Length > 0)
                        {
                            ref var worldPlanes = ref brushWorldPlanes[baseLoopInfo.brushNodeIndex].Value.worldPlanes;

                            var planesLength    = worldPlanes.Length;
                            var edgesLength     = baseLoopEdges.Length;

                            allSegments.Add(new LoopSegment()
                            {
                                edgeOffset      = edgeOffset,
                                edgeLength      = edgesLength,
                                planesOffset    = planeOffset,
                                planesLength    = planesLength
                            });

                            allEdges.AddRange(baseLoopEdges);

                            // TODO: ideally we'd only use the planes that intersect our edges
                            allWorldPlanes.AddRange(worldPlanes.GetUnsafePtr(), planesLength);

                            edgeOffset += edgesLength;
                            planeOffset += planesLength;
                        }

                        var destroyedEdges = new NativeArray<byte>(edgeOffset, Allocator.Temp);
                        {
                            {
                                var subtractEdgesJob = new SubtractEdgesJob()
                                {
                                    vertices            = brushVertices,
                                    segmentIndex        = holeIndices.Length,
                                    destroyedEdges      = destroyedEdges,
                                    allWorldPlanes      = allWorldPlanes,
                                    allSegments         = allSegments,
                                    allEdges            = allEdges
                                };
                                for (int j=0;j< holeIndices.Length;j++)
                                    subtractEdgesJob.Execute(j);
                            }

                            // TODO: optimize, keep track which holes (potentially) intersect
                            // TODO: create our own bounds data structure that doesn't use stupid slow properties for everything
                            {
                                var mergeEdgesJob = new MergeEdgesJob()
                                {
                                    vertices            = brushVertices,
                                    segmentCount        = holeIndices.Length,
                                    destroyedEdges      = destroyedEdges,
                                    allWorldPlanes      = allWorldPlanes,
                                    allSegments         = allSegments,
                                    allEdges            = allEdges
                                };
                                for (int j = 0, length = GeometryMath.GetTriangleArraySize(holeIndices.Length); j < length; j++)
                                    mergeEdgesJob.Execute(j);
                            }

                            {
                                var segment = allSegments[holeIndices.Length];
                                for (int e = baseLoopEdges.Length - 1; e >= 0; e--)
                                {
                                    if (destroyedEdges[segment.edgeOffset + e] == 0)
                                        continue;
                                    baseLoopEdges.RemoveAtSwapBack(e);
                                }
                            }

                            for (int h1 = holeIndices.Length - 1; h1 >= 0; h1--)
                            {
                                var holeIndex1  = holeIndices[h1];
                                var holeEdges1  = surfaceLoops.allEdges[holeIndex1];
                                var segment     = allSegments[h1];
                                for (int e = holeEdges1.Length - 1; e >= 0; e--)
                                {
                                    if (destroyedEdges[segment.edgeOffset + e] == 0)
                                        continue;
                                    holeEdges1.RemoveAtSwapBack(e);
                                }
                            }
                        }
                        destroyedEdges.Dispose();
                    }
                    allWorldPlanes.Dispose();
                    allSegments   .Dispose();
                    allEdges      .Dispose();

                    for (int h = holeIndices.Length - 1; h >= 0; h--)
                    {
                        var holeIndex   = holeIndices[h];
                        var holeEdges   = surfaceLoops.allEdges[holeIndex];
                        // Note: can have duplicate edges when multiple holes share an edge
                        //          (only edges between holes and base-loop are guaranteed to not be duplciate)
                        AddEdges(baseLoopEdges, holeEdges, removeDuplicates: true);
                    }

                    holeIndices.Clear();
                }

                // TODO: remove the need for this
                for (int l = surfaceLoops.loopIndices.Length - 1; l >= 0; l--)
                {
                    var baseloopIndex   = surfaceLoops.loopIndices[l];
                    var baseLoopEdges   = surfaceLoops.allEdges[baseloopIndex];
                    if (baseLoopEdges.Length < 3)
                    {
                        surfaceLoops.loopIndices.RemoveAtSwapBack(l);
                        continue;
                    }
                }
            }
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
