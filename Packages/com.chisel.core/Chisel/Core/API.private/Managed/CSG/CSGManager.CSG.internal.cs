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

        internal struct TreeUpdate
        {
            public int                              treeNodeIndex;
            public int                              triangleArraySize;
            public NativeArray<int>                 allTreeBrushIndices;
            public NativeArray<int>                 modifiedTreeBrushIndices;

            public BlobAssetReference<CompactTree>  compactTree;

            public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>            brushMeshLookup;
            public NativeHashMap<int, BlobAssetReference<NodeTransformations>>      transformations;
            public NativeHashMap<int, BlobAssetReference<BasePolygonsBlob>>         basePolygons;
            public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>         brushWorldPlanes;
            public NativeHashMap<int, BlobAssetReference<RoutingTable>>             routingTableLookup;
            public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>    brushesTouchedByBrushes;

            public Dictionary<int, NativeList<BlobAssetReference<ChiselSurfaceRenderBuffer>>> surfaceRenderBuffers;
            public NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>> intersectionLoopBlobs;
            public NativeMultiHashMap<int, BrushPair>                       brushBrushIntersections;
            public NativeList<BrushPair>                                    uniqueBrushPairs;
            public NativeList<BlobAssetReference<BrushPairIntersection>>    intersectingBrushes;

            public BrushOutputLoops[] brushOutputLoops;
            public BrushOutputLoops2[] brushOutputLoops2;

            public JobHandle generateBasePolygonLoopsJobHandle;

            public JobHandle findAllIntersectionsJobHandle;
            public JobHandle findIntersectingBrushesJobHandle;

            public JobHandle updateBrushWorldPlanesJobHandle;

            public JobHandle updateBrushCategorizationTablesJobHandle;

            public JobHandle findBrushPairsJobHandle;
            public JobHandle prepareBrushPairIntersectionsJobHandle;
            public JobHandle findAllIntersectionLoopsJobHandle;
            public JobHandle allFindAllIntersectionLoopsJobHandle;

            public JobHandle allFindLoopOverlapIntersectionsJobHandle;

            public JobHandle allGenerateSurfaceTrianglesJobHandle;
            public JobHandle performCSGJobDependenciesJobHandle;
        }

        public struct BrushOutputLoops2
        {
            public NativeListArray<int>     surfaceLoopIndices;
            public NativeList<SurfaceInfo>  allInfos;
            public NativeListArray<Edge>    allEdges;

            public JobHandle performAllCSGJobHandle;
            public JobHandle generateSurfaceTrianglesJobHandle;
        }

        internal static JobHandle UpdateTreeMeshes(int[] treeNodeIDs)
        {
            var finalJobHandle = default(JobHandle);
            
            var treeUpdates = new TreeUpdate[treeNodeIDs.Length];
            var treeUpdateLength = 0;
            Profiler.BeginSample("Setup");
            for (int t = 0; t < treeNodeIDs.Length; t++)
            {
                var treeNodeIndex       = treeNodeIDs[t] - 1;
                var treeInfo            = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;

                Profiler.BeginSample("Reset");
                Reset(treeInfo);
                Profiler.EndSample();
                var treeBrushes = treeInfo.treeBrushes;
                if (treeInfo.treeBrushes.Count == 0)
                    continue;
                
                var chiselLookupValues  = ChiselTreeLookup.Value[treeNodeIndex];
                var chiselMeshValues    = ChiselMeshLookup.Value;
                
                ref var brushMeshBlobs  = ref chiselMeshValues.brushMeshBlobs;
                ref var transformations = ref chiselLookupValues.transformations;

                // Removes all brushes that have MeshID == 0 from treeBrushesArray
                var allBrushNodeIndices = new List<int>();
                var modifiedBrushNodeIndices = new List<int>();
                for (int b = 0; b < treeBrushes.Count; b++)
                {
                    // 
                    var brushNodeID     = treeBrushes[b];
                    var brushNodeIndex  = brushNodeID - 1;
                    var brushMeshID     = CSGManager.nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID;
                    if (brushMeshID == 0)
                        continue;

                    allBrushNodeIndices.Add(brushNodeIndex);
                    if ((CSGManager.nodeFlags[brushNodeIndex].status & NodeStatusFlags.NeedMeshReset) == NodeStatusFlags.None)
                        continue;

                    var nodeFlags = CSGManager.nodeFlags[brushNodeIndex];
                    nodeFlags.status &= ~NodeStatusFlags.NeedMeshReset;
                    CSGManager.nodeFlags[brushNodeIndex] = nodeFlags;
                    modifiedBrushNodeIndices.Add(brushNodeIndex);
                }

                if (modifiedBrushNodeIndices.Count == 0)
                    continue;

                // Clean up values we're rebuilding below, including the ones with brushMeshID == 0
                chiselLookupValues.RemoveByBrushID(treeBrushes);

                Profiler.BeginSample("Allocations");
                var allTreeBrushIndices         = allBrushNodeIndices.ToNativeArray(Allocator.TempJob);
                var modifiedTreeBrushIndices    = modifiedBrushNodeIndices.ToNativeArray(Allocator.TempJob);
                var brushMeshLookup             = new NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>(allTreeBrushIndices.Length, Allocator.TempJob);
                Profiler.EndSample();

                // NOTE: needs to contain ALL brushes in tree, EVEN IF THEY ARE NOT UPDATED!
                Profiler.BeginSample("BuildBrushMeshLookup");
                {
                    for (int i = 0; i < allTreeBrushIndices.Length; i++)
                    {
                        var brushNodeIndex = allTreeBrushIndices[i];
                        var brushMeshIndex = CSGManager.nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID - 1;
                        brushMeshLookup[brushNodeIndex] = brushMeshBlobs[brushMeshIndex];
                    }
                }
                Profiler.EndSample();

                // TODO: optimize, only do this when necessary
                Profiler.BeginSample("UpdateBrushTransformations");
                {
                    for (int i = 0; i < allTreeBrushIndices.Length; i++)
                    {
                        UpdateNodeTransformation(ref transformations, allTreeBrushIndices[i]);
                    }
                }
                Profiler.EndSample();

                Profiler.BeginSample("DirtyAllOutlines");
                {
                    for (int b = 0; b < allTreeBrushIndices.Length; b++)
                    {
                        var brushNodeIndex = allTreeBrushIndices[b];
                        var brushInfo = CSGManager.nodeHierarchies[brushNodeIndex].brushInfo;
                        brushInfo.brushOutlineGeneration++;
                        brushInfo.brushOutlineDirty = true;
                    }
                }
                Profiler.EndSample();

                // TODO: only rebuild this when the hierarchy changes
                // TODO: jobify?
                Profiler.BeginSample("CompactTree.Create");
                var compactTree = CompactTree.Create(CSGManager.nodeHierarchies, treeNodeIndex); // Note: stored/destroyed in ChiselLookup
                Profiler.EndSample();


                Profiler.BeginSample("Allocations");
                Profiler.BeginSample("BrushOutputLoops");
                var brushOutputLoops    = new BrushOutputLoops[allTreeBrushIndices.Length];
                var brushOutputLoops2   = new BrushOutputLoops2[allTreeBrushIndices.Length];
                for (int index = 0; index < allTreeBrushIndices.Length; index++)
                {
                    var brushNodeIndex  = allTreeBrushIndices[index];

                    var basePolygonSurfaceInfos     = new NativeList<SurfaceInfo>(0, Allocator.TempJob);
                    var basePolygonEdges            = new NativeListArray<Edge>(16, Allocator.TempJob);
                    var intersectionSurfaceInfos    = new NativeList<SurfaceInfo>(0, Allocator.TempJob);
                    var intersectionEdges           = new NativeListArray<Edge>(16, Allocator.TempJob);
                    var vertexSoup                  = new VertexSoup(2048, Allocator.TempJob);

                    brushOutputLoops[index] = new BrushOutputLoops
                    { 
                        basePolygonSurfaceInfos     = basePolygonSurfaceInfos,
                        basePolygonEdges            = basePolygonEdges,
                        intersectionSurfaceInfos    = intersectionSurfaceInfos,
                        intersectionEdges           = intersectionEdges,
                        vertexSoup                  = vertexSoup
                    };
                    
                    var surfaceLoopIndices  = new NativeListArray<int>(Allocator.TempJob);
                    var allInfos            = new NativeList<SurfaceInfo>(Allocator.TempJob);
                    var allEdges            = new NativeListArray<Edge>(Allocator.TempJob);
                    brushOutputLoops2[index] = new BrushOutputLoops2
                    {
                        surfaceLoopIndices  = surfaceLoopIndices,
                        allInfos            = allInfos,
                        allEdges            = allEdges
                    };

                    var surfaceRenderBuffer = new NativeList<BlobAssetReference<ChiselSurfaceRenderBuffer>>(0, Allocator.Persistent);
                    chiselLookupValues.surfaceRenderBuffers[brushNodeIndex] = surfaceRenderBuffer;
                }
                Profiler.EndSample();

                var triangleArraySize       = GeometryMath.GetTriangleArraySize(allTreeBrushIndices.Length);
                var intersectionLoopBlobs   = new NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>(65500, Allocator.TempJob);
                var brushBrushIntersections = new NativeMultiHashMap<int, BrushPair>(triangleArraySize * 2, Allocator.TempJob);
                var uniqueBrushPairs        = new NativeList<BrushPair>(triangleArraySize, Allocator.TempJob);
                var intersectingBrushes     = new NativeList<BlobAssetReference<BrushPairIntersection>>(triangleArraySize, Allocator.TempJob);
                Profiler.EndSample();


                treeUpdates[treeUpdateLength] = new TreeUpdate
                {
                    treeNodeIndex               = treeNodeIndex,
                    triangleArraySize           = triangleArraySize,
                    allTreeBrushIndices         = allTreeBrushIndices,
                    modifiedTreeBrushIndices    = modifiedTreeBrushIndices,
                    brushMeshLookup             = brushMeshLookup,
                    transformations             = chiselLookupValues.transformations,
                    basePolygons                = chiselLookupValues.basePolygons,
                    brushWorldPlanes            = chiselLookupValues.brushWorldPlanes,
                    routingTableLookup          = chiselLookupValues.routingTableLookup,
                    brushesTouchedByBrushes     = chiselLookupValues.brushesTouchedByBrushes,
                    surfaceRenderBuffers        = chiselLookupValues.surfaceRenderBuffers,
                    intersectionLoopBlobs       = intersectionLoopBlobs,
                    brushBrushIntersections     = brushBrushIntersections,
                    uniqueBrushPairs            = uniqueBrushPairs,
                    intersectingBrushes         = intersectingBrushes,

                    compactTree                 = compactTree,
                    brushOutputLoops            = brushOutputLoops,
                    brushOutputLoops2           = brushOutputLoops2
                };
                treeUpdateLength++;
            }
            Profiler.EndSample();

            // TODO: should only do this once at creation time, part of brushMeshBlob? store with brush component itself
            Profiler.BeginSample("GenerateBasePolygonLoops");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate  = ref treeUpdates[t];
                    var createBlobPolygonsBlobs = new CreateBlobPolygonsBlobs
                    {
                        // Read
                        treeBrushIndices        = treeUpdate.allTreeBrushIndices,
                        brushMeshLookup         = treeUpdate.brushMeshLookup,
                        transformations         = treeUpdate.transformations,

                        // Write
                        basePolygons            = treeUpdate.basePolygons.AsParallelWriter()
                    };
                    treeUpdate.generateBasePolygonLoopsJobHandle = createBlobPolygonsBlobs.Schedule(treeUpdate.allTreeBrushIndices.Length, 64);
                }
            }
            finally { Profiler.EndSample(); }

            // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
            Profiler.BeginSample("FindIntersectingBrushes");
            try
            {
                // TODO: optimize, use hashed grid

                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var findAllIntersectionsJob = new FindAllIntersectionsJob
                    {
                        // Read
                        treeBrushIndices    = treeUpdate.allTreeBrushIndices,
                        transformations     = treeUpdate.transformations,
                        brushMeshLookup     = treeUpdate.brushMeshLookup,
                        basePolygons        = treeUpdate.basePolygons,

                        // Write
                        brushBrushIntersections = treeUpdate.brushBrushIntersections.AsParallelWriter()
                    };
                    treeUpdate.findAllIntersectionsJobHandle = findAllIntersectionsJob.Schedule(treeUpdate.generateBasePolygonLoopsJobHandle);
                }
                

                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var storeBrushIntersectionsJob = new StoreBrushIntersectionsJob
                    {
                        // Read
                        treeNodeIndex           = treeNodeIndex,
                        treeBrushIndices        = treeUpdate.allTreeBrushIndices,
                        compactTree             = treeUpdate.compactTree,
                        brushBrushIntersections = treeUpdate.brushBrushIntersections,

                        // Write
                        brushesTouchedByBrushes = treeUpdate.brushesTouchedByBrushes.AsParallelWriter()
                    };
                    treeUpdate.findIntersectingBrushesJobHandle = storeBrushIntersectionsJob.Schedule(treeUpdate.allTreeBrushIndices.Length, 64, treeUpdate.findAllIntersectionsJobHandle);
                    treeUpdate.brushBrushIntersections.Dispose(treeUpdate.findIntersectingBrushesJobHandle);
                }
            } finally { Profiler.EndSample(); }


            // TODO: should only do this at creation time + when moved / store with brush component itself
            Profiler.BeginSample("UpdateBrushWorldPlanes");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    // TODO: optimize, only do this when necessary
                    var createBrushWorldPlanesJob = new CreateBrushWorldPlanesJob
                    {
                        // Read
                        treeBrushIndices        = treeUpdate.allTreeBrushIndices,
                        brushMeshLookup         = treeUpdate.brushMeshLookup,
                        transformations         = treeUpdate.transformations,

                        // Write
                        brushWorldPlanes        = treeUpdate.brushWorldPlanes.AsParallelWriter()
                    };
                    treeUpdate.updateBrushWorldPlanesJobHandle   = createBrushWorldPlanesJob.Schedule(treeUpdate.allTreeBrushIndices.Length, 64);
                }
            } finally { Profiler.EndSample(); }

            // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
            Profiler.BeginSample("UpdateBrushCategorizationTables");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    // Build categorization trees for brushes
                    var createRoutingTableJob = new CreateRoutingTableJob
                    {
                        // Read
                        treeBrushIndices        = treeUpdate.allTreeBrushIndices,
                        brushesTouchedByBrushes = treeUpdate.brushesTouchedByBrushes,
                        compactTree             = treeUpdate.compactTree,

                        // Write
                        routingTableLookup      = treeUpdate.routingTableLookup.AsParallelWriter()
                    };
                    treeUpdate.updateBrushCategorizationTablesJobHandle = createRoutingTableJob.Schedule(treeUpdate.allTreeBrushIndices.Length, 64, treeUpdate.findIntersectingBrushesJobHandle);
                }
            } finally { Profiler.EndSample(); }
                                
            // Create unique loops between brush intersections
            Profiler.BeginSample("FindAllIntersectionLoops");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var findBrushPairsJob = new FindBrushPairsJob
                    {
                        // Read
                        maxPairs                = treeUpdate.triangleArraySize,
                        treeBrushIndices        = treeUpdate.allTreeBrushIndices,
                        brushesTouchedByBrushes = treeUpdate.brushesTouchedByBrushes,
                                    
                        // Write
                        uniqueBrushPairs        = treeUpdate.uniqueBrushPairs,
                    };
                    treeUpdate.findBrushPairsJobHandle = findBrushPairsJob.Schedule(treeUpdate.findIntersectingBrushesJobHandle);
                }

                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var prepareBrushPairIntersectionsJob = new PrepareBrushPairIntersectionsJob
                    {
                        // Read
                        uniqueBrushPairs        = treeUpdate.uniqueBrushPairs.AsDeferredJobArray(),
                        brushMeshBlobLookup     = treeUpdate.brushMeshLookup,
                        transformations         = treeUpdate.transformations,

                        // Write
                        intersectingBrushes     = treeUpdate.intersectingBrushes.AsParallelWriter()
                    };
                    treeUpdate.prepareBrushPairIntersectionsJobHandle = prepareBrushPairIntersectionsJob.Schedule(treeUpdate.triangleArraySize, 64, treeUpdate.findBrushPairsJobHandle);
                    treeUpdate.uniqueBrushPairs.Dispose(treeUpdate.prepareBrushPairIntersectionsJobHandle);
                }

                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var findAllIntersectionLoopsJob = new CSGManagerPerformCSG.FindAllIntersectionLoopsJob
                    {
                        // Read
                        brushWorldPlanes        = treeUpdate.brushWorldPlanes,
                        intersectingBrushes     = treeUpdate.intersectingBrushes.AsDeferredJobArray(),

                        // Write
                        outputSurfaces          = treeUpdate.intersectionLoopBlobs.AsParallelWriter()
                    };
                    treeUpdate.findAllIntersectionLoopsJobHandle = findAllIntersectionLoopsJob.Schedule(treeUpdate.triangleArraySize, 64, JobHandle.CombineDependencies(treeUpdate.updateBrushWorldPlanesJobHandle, treeUpdate.prepareBrushPairIntersectionsJobHandle));
                    treeUpdate.allFindAllIntersectionLoopsJobHandle = JobHandle.CombineDependencies(treeUpdate.findAllIntersectionLoopsJobHandle, treeUpdate.prepareBrushPairIntersectionsJobHandle, treeUpdate.allFindAllIntersectionLoopsJobHandle);

                    treeUpdate.intersectingBrushes.Dispose(treeUpdate.findAllIntersectionLoopsJobHandle);
                }
            } finally { Profiler.EndSample(); }
                
            Profiler.BeginSample("FindLoopOverlapIntersections");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var dependencies = JobHandle.CombineDependencies(treeUpdate.findIntersectingBrushesJobHandle, treeUpdate.allFindAllIntersectionLoopsJobHandle, treeUpdate.updateBrushWorldPlanesJobHandle);

                    for (int index = 0; index < treeUpdate.allTreeBrushIndices.Length; index++)
                    {
                        var outputLoops = treeUpdate.brushOutputLoops[index];
                        var findLoopOverlapIntersectionsJob = new FindLoopOverlapIntersectionsJob
                        {
                            // Read
                            index                       = index,
                            treeBrushIndices            = treeUpdate.allTreeBrushIndices,
                            intersectionLoopBlobs       = treeUpdate.intersectionLoopBlobs,
                            brushWorldPlanes            = treeUpdate.brushWorldPlanes,
                            basePolygonBlobs            = treeUpdate.basePolygons,

                            // Read/Write
                            vertexSoup                  = outputLoops.vertexSoup,
                            basePolygonEdges            = outputLoops.basePolygonEdges,
                            basePolygonSurfaceInfos     = outputLoops.basePolygonSurfaceInfos,
                            intersectionEdges           = outputLoops.intersectionEdges,
                            intersectionSurfaceInfos    = outputLoops.intersectionSurfaceInfos
                        };
                        var findLoopOverlapIntersectionsJobHandle = findLoopOverlapIntersectionsJob.Schedule(dependencies);
                        treeUpdate.allFindLoopOverlapIntersectionsJobHandle = JobHandle.CombineDependencies(findLoopOverlapIntersectionsJobHandle, treeUpdate.allFindLoopOverlapIntersectionsJobHandle);
                    }
                    treeUpdate.intersectionLoopBlobs.Dispose(treeUpdate.allFindLoopOverlapIntersectionsJobHandle);
                    treeUpdate.performCSGJobDependenciesJobHandle = JobHandle.CombineDependencies(treeUpdate.allFindLoopOverlapIntersectionsJobHandle, treeUpdate.updateBrushCategorizationTablesJobHandle);
                }
            }
            finally { Profiler.EndSample(); }

            for (int t = 0; t < treeUpdateLength; t++)
            {
                ref var treeUpdate = ref treeUpdates[t];

                // Perform CSG
                // TODO: only do this when necessary (when brushes have been modified)
                // TODO: determine when a brush is completely inside another brush
                //		 (might not have any intersection loops)

                Profiler.BeginSample("PerformCSGJob");
                for (int b = 0; b < treeUpdate.allTreeBrushIndices.Length; b++)
                {
                    var brushNodeIndex  = treeUpdate.allTreeBrushIndices[b];

                    ref var outputLoops     = ref treeUpdate.brushOutputLoops[b];
                    ref var outputLoops2    = ref treeUpdate.brushOutputLoops2[b];
                        
                    var performCSGJob = new PerformCSGJob
                    {
                        // Read
                        brushNodeIndex              = brushNodeIndex,
                        routingTableLookup          = treeUpdate.routingTableLookup,
                        brushWorldPlanes            = treeUpdate.brushWorldPlanes,
                        brushesTouchedByBrushes     = treeUpdate.brushesTouchedByBrushes,
                        brushVertices               = outputLoops.vertexSoup,
                        intersectionEdges           = outputLoops.intersectionEdges,
                        intersectionSurfaceInfos    = outputLoops.intersectionSurfaceInfos,

                        // Read / Write
                        basePolygonSurfaceInfos     = outputLoops.basePolygonSurfaceInfos,
                        basePolygonEdges            = outputLoops.basePolygonEdges,
                                    
                        surfaceLoopIndices          = outputLoops2.surfaceLoopIndices,
                        allInfos                    = outputLoops2.allInfos,
                        allEdges                    = outputLoops2.allEdges
                    };
                    outputLoops2.performAllCSGJobHandle = performCSGJob.Schedule(treeUpdate.performCSGJobDependenciesJobHandle);
                }
                Profiler.EndSample();

                // TODO: Cache the output surface meshes, only update when necessary
                Profiler.BeginSample("GenerateSurfaceTrianglesJob");
                for (int b = 0; b < treeUpdate.allTreeBrushIndices.Length; b++)
                {
                    var brushNodeIndex  = treeUpdate.allTreeBrushIndices[b];

                    ref var outputLoops         = ref treeUpdate.brushOutputLoops[b];
                    ref var outputLoops2        = ref treeUpdate.brushOutputLoops2[b];
                    var surfaceRenderBuffers    = treeUpdate.surfaceRenderBuffers[brushNodeIndex];

                    var generateSurfaceRenderBuffers = new GenerateSurfaceTrianglesJob
                    {
                        // Read
                        brushNodeIndex          = brushNodeIndex,
                        brushVertices           = outputLoops.vertexSoup,
                        surfaceLoopIndices      = outputLoops2.surfaceLoopIndices,
                        surfaceLoopAllInfos     = outputLoops2.allInfos,
                        surfaceLoopAllEdges     = outputLoops2.allEdges,
                        brushWorldPlanes        = treeUpdate.brushWorldPlanes,
                        basePolygons            = treeUpdate.basePolygons,

                        // Write
                        surfaceRenderBuffers    = surfaceRenderBuffers,
                    };

                    outputLoops2.generateSurfaceTrianglesJobHandle = generateSurfaceRenderBuffers.Schedule(outputLoops2.performAllCSGJobHandle);
                    treeUpdate.allGenerateSurfaceTrianglesJobHandle = JobHandle.CombineDependencies(outputLoops2.generateSurfaceTrianglesJobHandle, treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                }
                Profiler.EndSample();

                Profiler.BeginSample("IntermediateDispose");
                for (int b = 0; b < treeUpdate.brushOutputLoops2.Length; b++)
                {
                    ref var outputLoops2 = ref treeUpdate.brushOutputLoops2[b];

                    outputLoops2.surfaceLoopIndices  .Dispose(outputLoops2.generateSurfaceTrianglesJobHandle);
                    outputLoops2.allInfos            .Dispose(outputLoops2.generateSurfaceTrianglesJobHandle);
                    outputLoops2.allEdges            .Dispose(outputLoops2.generateSurfaceTrianglesJobHandle);
                }
                Profiler.EndSample();
            }

            Profiler.BeginSample("BrushOutputLoopsDispose");
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    for (int b = 0; b < treeUpdate.brushOutputLoops.Length; b++)
                    {
                        var outputLoops = treeUpdate.brushOutputLoops[b];
                        if (!outputLoops.intersectionSurfaceInfos.IsCreated)
                            continue;
                        outputLoops.intersectionSurfaceInfos.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                        outputLoops.intersectionEdges.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                        outputLoops.basePolygonSurfaceInfos.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                        outputLoops.basePolygonEdges.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                        outputLoops.vertexSoup.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                    }
                    treeUpdate.brushOutputLoops = null;

                    treeUpdate.brushMeshLookup.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                     
                    treeUpdate.allTreeBrushIndices.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                    treeUpdate.modifiedTreeBrushIndices.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                    finalJobHandle = JobHandle.CombineDependencies(treeUpdate.allGenerateSurfaceTrianglesJobHandle, finalJobHandle);

                    {
                        var flags = nodeFlags[treeNodeIndex];
                        flags.UnSetNodeFlag(NodeStatusFlags.TreeNeedsUpdate);
                        flags.SetNodeFlag(NodeStatusFlags.TreeMeshNeedsUpdate);
                        nodeFlags[treeNodeIndex] = flags;
                    }
                }
            }
            Profiler.EndSample();
            return finalJobHandle;
        }



        #region Rebuild / Update
        static void UpdateNodeTransformation(ref NativeHashMap<int, BlobAssetReference<NodeTransformations>> transformations, int nodeIndex)
        {
            // TODO: clean this up and make this sensible

            // Note: Currently "localTransformation" is actually nodeToTree, but only for all the brushes. 
            //       Branches do not have a transformation set at the moment.
            
            // TODO: should be transformations the way up to the tree, not just tree vs brush
            var brushLocalTransformation     = CSGManager.nodeLocalTransforms[nodeIndex].localTransformation;
            var brushLocalInvTransformation  = CSGManager.nodeLocalTransforms[nodeIndex].invLocalTransformation;

            var nodeTransform                = CSGManager.nodeTransforms[nodeIndex];
            nodeTransform.nodeToTree = brushLocalTransformation;
            nodeTransform.treeToNode = brushLocalInvTransformation;
            CSGManager.nodeTransforms[nodeIndex] = nodeTransform;

            if (transformations.TryGetValue(nodeIndex, out BlobAssetReference<NodeTransformations> oldTransformations))
            {
                if (oldTransformations.IsCreated)
                    oldTransformations.Dispose();
            }
            transformations[nodeIndex] = NodeTransformations.Build(nodeTransform.nodeToTree, nodeTransform.treeToNode);
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

        internal static bool UpdateAllTreeMeshes(out JobHandle allTrees)
        {
            allTrees = default(JobHandle);
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

            allTrees = UpdateTreeMeshes(trees.ToArray());
            return true;
        }

        internal static bool RebuildAll()
        {
            Reset();
            if (!UpdateAllTreeMeshes(out JobHandle handle))
                return false;
            handle.Complete();
            return true;
        }
        #endregion
    }
#endif
}
