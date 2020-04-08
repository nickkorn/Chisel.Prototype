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
            public NativeArray<int>                 treeBrushIndices;
            public NativeArray<int>                 brushMeshInstanceIDs;

            public BlobAssetReference<CompactTree>  compactTree;

            // TODO: why are these the same?!
            public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>            brushMeshLookup;
            public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>            brushMeshBlobs;

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
                
                // Clean up values we're rebuilding below
                chiselLookupValues.RemoveByBrushID(treeBrushes);

                //** REWRITE THIS **
                // Remove all brushes that have MeshID == 0 from treeBrushesArray (BUT **AFTER** WE REMOVE ALL DATA IN PREV STEP)
                var temp = new List<int>();
                for (int b = 0; b < treeBrushes.Count; b++)
                {
                    var brushNodeID     = treeBrushes[b];
                    var brushNodeIndex  = brushNodeID - 1;
                    var brushMeshID     = CSGManager.nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID;
                    if (brushMeshID == 0)
                        continue;
                    temp.Add(brushNodeIndex);
                }
                Profiler.BeginSample("Allocations");
                var treeBrushIndices = temp.ToNativeArray(Allocator.TempJob);
                Profiler.EndSample();
                //** REWRITE THIS **


                Profiler.BeginSample("Allocations");
                var brushMeshInstanceIDs = new NativeArray<int>(treeBrushIndices.Length, Allocator.TempJob);
                var brushMeshLookup      = new NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>(treeBrushIndices.Length, Allocator.TempJob);
                Profiler.EndSample();

                // TODO: somehow just keep this up to date instead of rebuilding it from scratch every update
                Profiler.BeginSample("FindBrushMeshInstanceIDs");
                {
                    FindBrushMeshInstanceIDs(treeBrushIndices, brushMeshInstanceIDs);
                }
                Profiler.EndSample();
                Profiler.BeginSample("FindBrushMeshBobs");
                {
                    FindBrushMeshBobs(ref chiselMeshValues, treeBrushIndices, brushMeshLookup); // <-- assumes all brushes in tree
                }
                Profiler.EndSample();

                // TODO: store this in blob / store with brush component itself & only update when transformations change
                Profiler.BeginSample("UpdateBrushTransformations");
                {
                    UpdateBrushTransformations(ref chiselLookupValues, treeBrushIndices);
                }
                Profiler.EndSample();

                Profiler.BeginSample("DirtyAllOutlines");
                {
                    UpdateAllOutlines(treeBrushIndices);
                }
                Profiler.EndSample();

                // TODO: only rebuild this when the hierarchy changes
                Profiler.BeginSample("CompactTree.Create");
                var compactTree = CompactTree.Create(CSGManager.nodeHierarchies, treeNodeIndex); // Note: stored/destroyed in ChiselLookup
                Profiler.EndSample();


                Profiler.BeginSample("Allocations");
                Profiler.BeginSample("BrushOutputLoops");
                var brushOutputLoops    = new BrushOutputLoops[treeBrushIndices.Length];
                var brushOutputLoops2   = new BrushOutputLoops2[treeBrushIndices.Length];
                for (int index = 0; index < treeBrushIndices.Length; index++)
                {
                    var brushNodeIndex  = treeBrushIndices[index];

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

                var triangleArraySize       = GeometryMath.GetTriangleArraySize(treeBrushIndices.Length);
                var intersectionLoopBlobs   = new NativeHashMap<BrushSurfacePair, BlobAssetReference<BrushIntersectionLoop>>(65500, Allocator.TempJob);
                var brushBrushIntersections = new NativeMultiHashMap<int, BrushPair>(triangleArraySize * 2, Allocator.TempJob);
                var uniqueBrushPairs        = new NativeList<BrushPair>(triangleArraySize, Allocator.TempJob);
                var intersectingBrushes     = new NativeList<BlobAssetReference<BrushPairIntersection>>(triangleArraySize, Allocator.TempJob);
                Profiler.EndSample();


                treeUpdates[treeUpdateLength] = new TreeUpdate
                {
                    treeNodeIndex           = treeNodeIndex,
                    triangleArraySize       = triangleArraySize,
                    treeBrushIndices        = treeBrushIndices,
                    brushMeshInstanceIDs    = brushMeshInstanceIDs,
                    brushMeshLookup         = brushMeshLookup,
                    brushMeshBlobs          = chiselMeshValues.brushMeshBlobs,
                    transformations         = chiselLookupValues.transformations,
                    basePolygons            = chiselLookupValues.basePolygons,
                    brushWorldPlanes        = chiselLookupValues.brushWorldPlanes,
                    routingTableLookup      = chiselLookupValues.routingTableLookup,
                    brushesTouchedByBrushes = chiselLookupValues.brushesTouchedByBrushes,
                    surfaceRenderBuffers    = chiselLookupValues.surfaceRenderBuffers,
                    intersectionLoopBlobs   = intersectionLoopBlobs,
                    brushBrushIntersections = brushBrushIntersections,
                    uniqueBrushPairs        = uniqueBrushPairs,
                    intersectingBrushes     = intersectingBrushes,

                    compactTree             = compactTree,
                    brushOutputLoops        = brushOutputLoops,
                    brushOutputLoops2       = brushOutputLoops2
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
                        treeBrushIndices        = treeUpdate.treeBrushIndices,
                        brushMeshInstanceIDs    = treeUpdate.brushMeshInstanceIDs,
                        brushMeshBlobs          = treeUpdate.brushMeshBlobs,
                        transformations         = treeUpdate.transformations,

                        // Write
                        basePolygons            = treeUpdate.basePolygons.AsParallelWriter()
                    };
                    treeUpdate.generateBasePolygonLoopsJobHandle = createBlobPolygonsBlobs.Schedule(treeUpdate.treeBrushIndices.Length, 64);
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
                        treeBrushIndices        = treeUpdate.treeBrushIndices,
                        transformations         = treeUpdate.transformations,
                        brushMeshBlobs          = treeUpdate.brushMeshBlobs,
                        basePolygons            = treeUpdate.basePolygons,
                        brushMeshIDs            = treeUpdate.brushMeshInstanceIDs,

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
                        treeBrushIndices        = treeUpdate.treeBrushIndices,
                        compactTree             = treeUpdate.compactTree,
                        brushBrushIntersections = treeUpdate.brushBrushIntersections,

                        // Write
                        brushesTouchedByBrushes = treeUpdate.brushesTouchedByBrushes.AsParallelWriter()
                    };
                    treeUpdate.findIntersectingBrushesJobHandle = storeBrushIntersectionsJob.Schedule(treeUpdate.treeBrushIndices.Length, 64, treeUpdate.findAllIntersectionsJobHandle);
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
                        treeBrushIndices        = treeUpdate.treeBrushIndices,
                        brushMeshLookup         = treeUpdate.brushMeshLookup,
                        transformations         = treeUpdate.transformations,

                        // Write
                        brushWorldPlanes        = treeUpdate.brushWorldPlanes.AsParallelWriter()
                    };
                    treeUpdate.updateBrushWorldPlanesJobHandle   = createBrushWorldPlanesJob.Schedule(treeUpdate.treeBrushIndices.Length, 64);
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
                        treeBrushIndices        = treeUpdate.treeBrushIndices,
                        brushesTouchedByBrushes = treeUpdate.brushesTouchedByBrushes,
                        compactTree             = treeUpdate.compactTree,

                        // Write
                        routingTableLookup      = treeUpdate.routingTableLookup.AsParallelWriter()
                    };
                    treeUpdate.updateBrushCategorizationTablesJobHandle = createRoutingTableJob.Schedule(treeUpdate.treeBrushIndices.Length, 64, treeUpdate.findIntersectingBrushesJobHandle);
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
                        treeBrushIndices        = treeUpdate.treeBrushIndices,
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

                    for (int index = 0; index < treeUpdate.treeBrushIndices.Length; index++)
                    {
                        var outputLoops = treeUpdate.brushOutputLoops[index];
                        var findLoopOverlapIntersectionsJob = new FindLoopOverlapIntersectionsJob
                        {
                            // Read
                            index                       = index,
                            treeBrushIndices            = treeUpdate.treeBrushIndices,
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
                // TODO: Cache the output surface meshes, only update when necessary

                Profiler.BeginSample("PerformCSGJob");
                for (int b = 0; b < treeUpdate.treeBrushIndices.Length; b++)
                {
                    var brushNodeIndex  = treeUpdate.treeBrushIndices[b];

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

                Profiler.BeginSample("GenerateSurfaceTrianglesJob");
                for (int b = 0; b < treeUpdate.treeBrushIndices.Length; b++)
                {
                    var brushNodeIndex  = treeUpdate.treeBrushIndices[b];

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

                    treeUpdate.brushMeshInstanceIDs.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                    treeUpdate.brushMeshLookup.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                     
                    treeUpdate.treeBrushIndices.Dispose(treeUpdate.allGenerateSurfaceTrianglesJobHandle);
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
        static void UpdateBrushTransformation(ref NativeHashMap<int, BlobAssetReference<NodeTransformations>> transformations, int brushNodeIndex)
        {
            var parentNodeID                 = CSGManager.nodeHierarchies[brushNodeIndex].parentNodeID;
            var parentNodeIndex              = parentNodeID - 1;
            var parentLocalTransformation    = (parentNodeIndex < 0) ? Matrix4x4.identity : nodeLocalTransforms[parentNodeIndex].invLocalTransformation;
            var parentLocalInvTransformation = (parentNodeIndex < 0) ? Matrix4x4.identity : nodeLocalTransforms[parentNodeIndex].localTransformation;

            // TODO: should be transformations the way up to the tree, not just tree vs brush
            var brushLocalTransformation     = CSGManager.nodeLocalTransforms[brushNodeIndex].localTransformation;
            var brushLocalInvTransformation  = CSGManager.nodeLocalTransforms[brushNodeIndex].invLocalTransformation;

            var nodeTransform                = CSGManager.nodeTransforms[brushNodeIndex];
            nodeTransform.nodeToTree = brushLocalTransformation * parentLocalInvTransformation;
            nodeTransform.treeToNode = parentLocalTransformation * brushLocalInvTransformation;
            CSGManager.nodeTransforms[brushNodeIndex] = nodeTransform;

            if (transformations.TryGetValue(brushNodeIndex, out BlobAssetReference<NodeTransformations> oldTransformations))
            {
                if (oldTransformations.IsCreated)
                    oldTransformations.Dispose();
            }
            transformations[brushNodeIndex] = NodeTransformations.Build(nodeTransform.nodeToTree, nodeTransform.treeToNode);
        }

        static void UpdateBrushTransformations(ref ChiselTreeLookup.Data chiselLookupValues, NativeArray<int> treeBrushIndices)
        {
            ref var transformations = ref chiselLookupValues.transformations;

            // TODO: optimize, only do this when necessary

            for (int i = 0; i < treeBrushIndices.Length; i++)
            {
                UpdateBrushTransformation(ref transformations, treeBrushIndices[i]);
            }
        }

        static void FindBrushMeshInstanceIDs(NativeArray<int> treeBrushIndices, NativeArray<int> brushMeshInstanceIDs)
        {
            for (int i = 0; i < treeBrushIndices.Length; i++)
            {
                var brushNodeIndex = treeBrushIndices[i];
                brushMeshInstanceIDs[i] = CSGManager.nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID;
            }
        }

        static void FindBrushMeshBobs(ref ChiselMeshLookup.Data chiselMeshValues, NativeArray<int> treeBrushIndices, NativeHashMap<int, BlobAssetReference<BrushMeshBlob>> brushMeshLookup)
        {
            ref var brushMeshBlobs = ref chiselMeshValues.brushMeshBlobs;
            for (int i = 0; i < treeBrushIndices.Length; i++)
            {
                var brushNodeIndex = treeBrushIndices[i];
                brushMeshLookup[brushNodeIndex] = brushMeshBlobs[CSGManager.nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID - 1];
            }
        }


        static void UpdateAllOutlines(NativeArray<int> treeBrushIndices)
        {
            for (int b = 0; b < treeBrushIndices.Length; b++)
            {
                var brushNodeIndex  = treeBrushIndices[b];
                var brushInfo       = CSGManager.nodeHierarchies[brushNodeIndex].brushInfo;
                brushInfo.brushOutlineGeneration++;
                brushInfo.brushOutlineDirty = true;
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        unsafe struct PerformCSGJob : IJob
        {
            [NoAlias, ReadOnly] public int                                                              brushNodeIndex;
            [NoAlias, ReadOnly] public VertexSoup                                                       brushVertices;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<RoutingTable>>             routingTableLookup;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>         brushWorldPlanes;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>    brushesTouchedByBrushes;

            [NoAlias, ReadOnly] public NativeList<SurfaceInfo>  intersectionSurfaceInfos;
            [NoAlias, ReadOnly] public NativeListArray<Edge>    intersectionEdges;

            [NoAlias] public NativeList<SurfaceInfo>            basePolygonSurfaceInfos;    // <-- should be readonly?
            [NoAlias] public NativeListArray<Edge>              basePolygonEdges;           // <-- should be readonly?

            [NoAlias] public NativeListArray<int>               surfaceLoopIndices;
            [NoAlias] public NativeList<SurfaceInfo>            allInfos;
            [NoAlias] public NativeListArray<Edge>              allEdges;

            public void Execute()
            {
                if (basePolygonSurfaceInfos.Length == 0)
                    return;


                if (!routingTableLookup.TryGetValue(brushNodeIndex, out BlobAssetReference<RoutingTable> routingTable))
                {
                    //Debug.LogError("No routing table found");
                    return;
                }

                ref var nodes                   = ref routingTable.Value.nodes;
                ref var routingLookups          = ref routingTable.Value.routingLookups;
                
                var surfaceCount                = basePolygonEdges.Length;

                NativeListArray<Edge>       intersectionLoops;
                NativeArray<SurfaceInfo>    intersectionSurfaceInfo;
                {
                    ref var routingTableNodes = ref routingTable.Value.nodes;

                    var nodeIDtoIndex = new NativeHashMap<int, int>(routingTableNodes.Length, Allocator.Temp);
                    for (int i = 0; i < routingTableNodes.Length; i++)
                        nodeIDtoIndex[routingTableNodes[i]] = i;

                    // TODO: Sort the brushSurfaceInfos/intersectionEdges based on nodeIDtoIndex[surfaceInfo.brushNodeID], 
                    //       have a sequential list of all data. 
                    //       Have segment list to determine which part of array belong to which brushNodeID
                    //       Don't need bottom part, can determine this in Job

                    intersectionLoops           = new NativeListArray<Edge>(16, Allocator.Temp);
                    intersectionSurfaceInfo     = new NativeArray<SurfaceInfo>(routingTableNodes.Length * surfaceCount, Allocator.Temp);
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


                var holeIndices = new NativeListArray<int>(surfaceCount, Allocator.Temp);
                surfaceLoopIndices.ResizeExact(surfaceCount);
                for (int s = 0; s < surfaceCount; s++)
                {
                    var info = basePolygonSurfaceInfos[s];
                    info.interiorCategory = CategoryGroupIndex.First; // TODO: make sure that it's always set to "First" so we don't need to do this

                    var loopIndices = surfaceLoopIndices[s];
                    loopIndices.Add(allEdges.Length);
                    holeIndices.Add();
                    allInfos.Add(info);
                    allEdges.Add(basePolygonEdges[s]);
                }

                for (int i = 0, offset = 0; i < routingLookups.Length; i++)
                {
                    ref var routingLookup = ref routingLookups[i];
                    for (int surfaceIndex = 0; surfaceIndex < surfaceCount; surfaceIndex++, offset++)
                    {
                        var loopIndices         = surfaceLoopIndices[surfaceIndex];
                        var intersectionLoop    = intersectionLoops[offset];
                        var intersectionInfo    = intersectionSurfaceInfo[offset];
                        for (int l = loopIndices.Length - 1; l >= 0; l--)
                        {
                            var surfaceLoopIndex = loopIndices[l];
                            var surfaceLoopEdges = allEdges[surfaceLoopIndex];
                            if (surfaceLoopEdges.Length < 3)
                                continue;

                            var surfaceLoopInfo = allInfos[surfaceLoopIndex];

                            if (!routingLookup.TryGetRoute(routingTable, surfaceLoopInfo.interiorCategory, out CategoryRoutingRow routingRow))
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
                                allInfos[surfaceLoopIndex] = surfaceLoopInfo;
                                continue;
                            } else
                            {
                                surfaceLoopInfo.interiorCategory = routingRow[CategoryIndex.Outside];
                                allInfos[surfaceLoopIndex] = surfaceLoopInfo;
                            }

                            // Add all holes that share the same plane to the polygon
                            if (intersectionLoop.Length != 0)
                            {
                                // Categorize between original surface & intersection
                                var intersectionCategory = routingRow[intersectionInfo.interiorCategory];

                                // If the intersection polygon would get the same category, we don't need to do a pointless intersection
                                if (intersectionCategory == surfaceLoopInfo.interiorCategory)
                                    continue;

                                //Profiler.BeginSample("Intersect");
                                var intersectLoopsJob = new IntersectLoopsJob
                                {
                                    brushVertices           = brushVertices,
                                    brushWorldPlanes        = brushWorldPlanes,
                                    brushesTouchedByBrushes = brushesTouchedByBrushes,
                                    surfaceLoopIndex        = surfaceLoopIndex,
                                    intersectionLoop        = intersectionLoop,
                                    newHoleCategory         = intersectionCategory,

                                    intersectionInfo        = intersectionInfo,
                                    loopIndices             = loopIndices,

                                    holeIndices             = holeIndices,
                                    allInfos                = allInfos,
                                    allEdges                = allEdges,
                                };
                                intersectLoopsJob.Execute();
                                //Profiler.EndSample();
                            }
                        }

                        // TODO: remove the need for this (check on insertion)
                        //Profiler.BeginSample("CSGManagerPerformCSG.RemoveEmptyLoops");
                        var removeEmptyLoopsJob = new RemoveEmptyLoopsJob
                        {
                            loopIndices     = loopIndices,
                                
                            holeIndices     = holeIndices,
                            allInfos        = allInfos,
                            allEdges        = allEdges,
                        };
                        removeEmptyLoopsJob.Execute();
                        //Profiler.EndSample();
                    }
                }

                intersectionLoops.Dispose();
                intersectionSurfaceInfo.Dispose();

                //Profiler.BeginSample("CleanUp");
                {
                    for (int surfaceIndex = 0; surfaceIndex < surfaceLoopIndices.Length; surfaceIndex++)
                    {
                        var loopIndices = surfaceLoopIndices[surfaceIndex];
                        var cleanUpJob = new CleanUpJob
                        {
                            brushVertices       = brushVertices,
                            brushWorldPlanes    = brushWorldPlanes,
                            loopIndices         = loopIndices,
                                
                            holeIndices         = holeIndices,
                            allInfos            = allInfos,
                            allEdges            = allEdges,
                        };
                        cleanUpJob.Execute();
                    }
                }
                //Profiler.EndSample();

                holeIndices.Dispose();
            }
        }


        [BurstCompile(CompileSynchronously = true)]
        unsafe struct IntersectLoopsJob : IJob
        {
            [NoAlias, ReadOnly] public VertexSoup brushVertices;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>> brushWorldPlanes;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>> brushesTouchedByBrushes;
            [NoAlias, ReadOnly] public int surfaceLoopIndex;
            [NoAlias, ReadOnly] public NativeListArray<Edge>.NativeList intersectionLoop;
            [NoAlias, ReadOnly] public CategoryGroupIndex newHoleCategory;

            [NoAlias] public SurfaceInfo intersectionInfo;
            
            [NoAlias] public NativeListArray<int>.NativeList    loopIndices;
            [NoAlias] public NativeListArray<int>               holeIndices;
            [NoAlias] public NativeList<SurfaceInfo>            allInfos;
            [NoAlias] public NativeListArray<Edge>              allEdges;

            public void Execute()
            {
                var currentLoopEdges    = allEdges[surfaceLoopIndex];
                var currentInfo         = allInfos[surfaceLoopIndex];
                var currentHoleIndices  = holeIndices[surfaceLoopIndex];

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
                    allInfos[surfaceLoopIndex] = currentInfo;
                    outEdges.Dispose();
                } else
                if (result == OperationResult.Overlapping)
                {
                    outEdges.Dispose();
                    currentInfo.interiorCategory = newHoleCategory;
                    allInfos[surfaceLoopIndex] = currentInfo;
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
                            var holeEdges = allEdges[holeIndex];
                            if (holeEdges.Length < 3)
                                continue;

                            var holeInfo        = allInfos[holeIndex];
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
                                intersectedHoleIndices.Add(allEdges.Length);
                                holeIndices.Add();
                                allInfos.Add(holeInfo);
                                allEdges.Add(holeEdges);
                            }
                        }
                    }

                    // TODO: Separate loop "shapes" from category/loop-hole hierarchy, 
                    //       so we can simply assign the same shape to a hole and loop without 
                    //       needing to copy data we can create a new shape when we modify it.

                    // this loop is a hole 
                    currentHoleIndices.Add(allEdges.Length);
                    holeIndices.Add();
                    allInfos.Add(intersectionInfo);
                    allEdges.Add(outEdges);

                    // but also a polygon on its own
                    loopIndices.Add(allEdges.Length);
                    holeIndices.Add(intersectedHoleIndices);
                    allInfos.Add(intersectionInfo);
                    allEdges.Add(outEdges);

                    intersectedHoleIndices.Dispose();
                    outEdges.Dispose();
                }
            }

        }

        [BurstCompile(CompileSynchronously = true)]
        unsafe struct RemoveEmptyLoopsJob : IJob
        {
            [NoAlias] public NativeListArray<int>.NativeList    loopIndices;
            [NoAlias] public NativeListArray<int>               holeIndices;
            [NoAlias] public NativeList<SurfaceInfo>            allInfos;
            [NoAlias] public NativeListArray<Edge>              allEdges;

            public void Execute()
            {
                for (int l = loopIndices.Length - 1; l >= 0; l--)
                {
                    var loopIndex = loopIndices[l];
                    var loopEdges = allEdges[loopIndex];
                    if (loopEdges.Length < 3)
                    {
                        loopIndices.RemoveAtSwapBack(l);
                        break;
                    }

                    var holeIndicesList = holeIndices[loopIndex];
                    for (int h = holeIndicesList.Length - 1; h >= 0; h--)
                    {
                        var holeIndex = holeIndicesList[h];
                        var holeEdges = allEdges[holeIndex];
                        if (holeEdges.Length < 3)
                        {
                            holeIndicesList.RemoveAtSwapBack(h);
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
            
            [NoAlias] public NativeListArray<int>.NativeList    loopIndices;
            [NoAlias] public NativeListArray<int>               holeIndices;
            [NoAlias] public NativeList<SurfaceInfo>            allInfos;
            [NoAlias] public NativeListArray<Edge>              allEdges;

            struct Empty {}

            [BurstDiscard]
            private static void NotUniqueEdgeException()
            {
                throw new Exception("Edge is not unique");
            }

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
                for (int l = loopIndices.Length - 1; l >= 0; l--)
                {
                    var baseloopIndex   = loopIndices[l];
                    var baseLoopEdges   = allEdges[baseloopIndex];
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

                    var holeIndicesList = holeIndices[baseloopIndex];
                    if (holeIndicesList.Length == 0)
                        continue;

                    var baseLoopInfo    = allInfos[baseloopIndex];
                
                    for (int h = holeIndicesList.Length - 1; h >= 0; h--)
                    {
                        var holeIndex   = holeIndicesList[h];
                        var holeEdges   = allEdges[holeIndex];
                        if (holeEdges.Length < 3)
                        {
                            holeIndicesList.RemoveAtSwapBack(h);
                            continue;
                        }
                        var holeNormal = CSGManagerPerformCSG.CalculatePlaneEdges(holeEdges, brushVertices);
                        if (math.all(holeNormal == float3.zero))
                        {
                            holeIndicesList.RemoveAtSwapBack(h);
                            continue;
                        }
                    }
                    if (holeIndicesList.Length == 0)
                        continue;

                    var allWorldPlanes   = new NativeList<float4>(Allocator.Temp);
                    var allSegments      = new NativeList<LoopSegment>(Allocator.Temp);
                    var allCombinedEdges = new NativeList<Edge>(Allocator.Temp);
                    {
                        int edgeOffset = 0;
                        int planeOffset = 0;
                        for (int h = 0; h < holeIndicesList.Length; h++)
                        {
                            var holeIndex   = holeIndicesList[h];
                            var holeInfo    = allInfos[holeIndex];
                            var holeEdges   = allEdges[holeIndex];

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

                            allCombinedEdges.AddRange(holeEdges);

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

                            allCombinedEdges.AddRange(baseLoopEdges);

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
                                    segmentIndex        = holeIndicesList.Length,
                                    destroyedEdges      = destroyedEdges,
                                    allWorldPlanes      = allWorldPlanes,
                                    allSegments         = allSegments,
                                    allEdges            = allCombinedEdges
                                };
                                for (int j=0;j< holeIndicesList.Length;j++)
                                    subtractEdgesJob.Execute(j);
                            }

                            // TODO: optimize, keep track which holes (potentially) intersect
                            // TODO: create our own bounds data structure that doesn't use stupid slow properties for everything
                            {
                                var mergeEdgesJob = new MergeEdgesJob()
                                {
                                    vertices            = brushVertices,
                                    segmentCount        = holeIndicesList.Length,
                                    destroyedEdges      = destroyedEdges,
                                    allWorldPlanes      = allWorldPlanes,
                                    allSegments         = allSegments,
                                    allEdges            = allCombinedEdges
                                };
                                for (int j = 0, length = GeometryMath.GetTriangleArraySize(holeIndicesList.Length); j < length; j++)
                                    mergeEdgesJob.Execute(j);
                            }

                            {
                                var segment = allSegments[holeIndicesList.Length];
                                for (int e = baseLoopEdges.Length - 1; e >= 0; e--)
                                {
                                    if (destroyedEdges[segment.edgeOffset + e] == 0)
                                        continue;
                                    baseLoopEdges.RemoveAtSwapBack(e);
                                }
                            }

                            for (int h1 = holeIndicesList.Length - 1; h1 >= 0; h1--)
                            {
                                var holeIndex1  = holeIndicesList[h1];
                                var holeEdges1  = allEdges[holeIndex1];
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
                    allWorldPlanes  .Dispose();
                    allSegments     .Dispose();
                    allCombinedEdges.Dispose();

                    for (int h = holeIndicesList.Length - 1; h >= 0; h--)
                    {
                        var holeIndex   = holeIndicesList[h];
                        var holeEdges   = allEdges[holeIndex];
                        // Note: can have duplicate edges when multiple holes share an edge
                        //          (only edges between holes and base-loop are guaranteed to not be duplciate)
                        AddEdges(baseLoopEdges, holeEdges, removeDuplicates: true);
                    }

                    holeIndicesList.Clear();
                }

                // TODO: remove the need for this
                for (int l = loopIndices.Length - 1; l >= 0; l--)
                {
                    var baseloopIndex   = loopIndices[l];
                    var baseLoopEdges   = allEdges[baseloopIndex];
                    if (baseLoopEdges.Length < 3)
                    {
                        loopIndices.RemoveAtSwapBack(l);
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
