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
            None                        = 0,
            //NeedChildUpdate		    = 1,
            NeedPreviousSiblingsUpdate  = 2,

            TreeIsDisabled              = 1024,// TODO: remove, or make more useful
            OperationNeedsUpdate        = 4,
            TreeNeedsUpdate             = 8,
            TreeMeshNeedsUpdate         = 16,

            ShapeModified               = 32,
            TransformationModified      = 64,
            HierarchyModified           = 128,
            OutlineModified             = 256,
            NeedAllTouchingUpdated      = 512,	// all brushes that touch this brush need to be updated,
            NeedFullUpdate              = ShapeModified | TransformationModified | OutlineModified | HierarchyModified,
            NeedCSGUpdate               = ShapeModified | TransformationModified | HierarchyModified,
            NeedUpdate                  = TransformationModified | OutlineModified | HierarchyModified,
            NeedUpdateDirectOnly        = TransformationModified | OutlineModified,
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
            public NativeList<int>                  modifiedTreeBrushIndices;
            public NativeArray<int>                 baseMeshUpdateIndices;

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

            public BrushOutputLoops[]   brushOutputLoops;
            public BrushOutputLoops2[]  brushOutputLoops2;
            public BrushHandles[]       brushHandles;

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

        public struct BrushOutputLoops
        {
            [NoAlias] public NativeList<SurfaceInfo>  intersectionSurfaceInfos;
            [NoAlias] public NativeListArray<Edge>    intersectionEdges;
            [NoAlias] public NativeList<SurfaceInfo>  basePolygonSurfaceInfos;
            [NoAlias] public NativeListArray<Edge>    basePolygonEdges;
            [NoAlias] public VertexSoup               vertexSoup;
        }

        public struct BrushOutputLoops2
        {
            [NoAlias] public NativeListArray<int>     surfaceLoopIndices;
            [NoAlias] public NativeList<SurfaceInfo>  allInfos;
            [NoAlias] public NativeListArray<Edge>    allEdges;
        }

        public struct BrushHandles
        {
            public JobHandle performAllCSGJobHandle;
            public JobHandle generateSurfaceTrianglesJobHandle;
        }

        struct TreeSorter : IComparer<TreeUpdate>
        {
            public int Compare(TreeUpdate x, TreeUpdate y)
            {
                var xBrushBrushIntersectionsCount = x.brushBrushIntersections.Count();
                var yBrushBrushIntersectionsCount = y.brushBrushIntersections.Count();
                if (xBrushBrushIntersectionsCount < yBrushBrushIntersectionsCount)
                    return 1;
                if (xBrushBrushIntersectionsCount > yBrushBrushIntersectionsCount)
                    return -1;

                if (x.modifiedTreeBrushIndices.Length < y.modifiedTreeBrushIndices.Length)
                    return 1;
                if (x.modifiedTreeBrushIndices.Length > y.modifiedTreeBrushIndices.Length)
                    return -1;

                return x.treeNodeIndex - y.treeNodeIndex;
            }
        }

        internal static JobHandle UpdateTreeMeshes(int[] treeNodeIDs)
        {
            var finalJobHandle = default(JobHandle);
            
            var treeUpdates = new TreeUpdate[treeNodeIDs.Length];
            var treeUpdateLength = 0;
            Profiler.BeginSample("Tag_Setup");//time=12.17ms
            for (int t = 0; t < treeNodeIDs.Length; t++)
            {
                var treeNodeIndex       = treeNodeIDs[t] - 1;
                var treeInfo            = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;

                Profiler.BeginSample("Tag_Reset");//time=0.617ms
                Reset(treeInfo);
                Profiler.EndSample();

                var treeBrushes = treeInfo.treeBrushes;
                if (treeInfo.treeBrushes.Count == 0)
                    continue;
                
                var chiselLookupValues  = ChiselTreeLookup.Value[treeNodeIndex];
                var chiselMeshValues    = ChiselMeshLookup.Value;
                
                ref var brushMeshBlobs  = ref chiselMeshValues.brushMeshBlobs;
                ref var transformations = ref chiselLookupValues.transformations;
                ref var basePolygons    = ref chiselLookupValues.basePolygons;

                // Removes all brushes that have MeshID == 0 from treeBrushesArray
                var allBrushNodeIndices         = new List<int>();
                var modifiedBrushNodeIndices    = new List<int>();
                for (int b = 0; b < treeBrushes.Count; b++)
                {
                    // 
                    var brushNodeID     = treeBrushes[b];
                    var brushNodeIndex  = brushNodeID - 1;
                    var brushMeshID     = CSGManager.nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID;
                    if (brushMeshID == 0)
                        continue;

                    allBrushNodeIndices.Add(brushNodeIndex);
                    var nodeFlags = CSGManager.nodeFlags[brushNodeIndex];
                    if (nodeFlags.status == NodeStatusFlags.None)
                        continue;

                    modifiedBrushNodeIndices.Add(brushNodeIndex);
                }

                if (modifiedBrushNodeIndices.Count == 0)
                    continue;

                var baseMeshUpdateIndicesList = new List<int>();
                for (int b = 0; b < modifiedBrushNodeIndices.Count; b++)
                {
                    var brushNodeIndex = modifiedBrushNodeIndices[b];
                    
                    var nodeFlags = CSGManager.nodeFlags[brushNodeIndex];
                    if ((nodeFlags.status & NodeStatusFlags.ShapeModified) != NodeStatusFlags.None)
                    {
                        // Need to update the basePolygons for this node
                        baseMeshUpdateIndicesList.Add(brushNodeIndex);
                        if (basePolygons.TryGetValue(brushNodeIndex, out var basePolygonsBlob))
                        {
                            basePolygons.Remove(brushNodeIndex);
                            if (basePolygonsBlob.IsCreated)
                                basePolygonsBlob.Dispose();
                        }
                        nodeFlags.status &= ~NodeStatusFlags.ShapeModified;
                    }
                    CSGManager.nodeFlags[brushNodeIndex] = nodeFlags;
                }

                // Clean up values we're rebuilding below, including the ones with brushMeshID == 0
                chiselLookupValues.RemoveByBrushID(treeBrushes);

                Profiler.BeginSample("Tag_Allocations");//time=2.45ms
                var allTreeBrushIndices         = allBrushNodeIndices.ToNativeArray(Allocator.TempJob);
                var modifiedTreeBrushIndices    = modifiedBrushNodeIndices.ToNativeList(Allocator.TempJob);
                var baseMeshUpdateIndices       = baseMeshUpdateIndicesList.ToNativeArray(Allocator.TempJob);
                var brushMeshLookup             = new NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>(allTreeBrushIndices.Length, Allocator.TempJob);
                Profiler.EndSample();

                // NOTE: needs to contain ALL brushes in tree, EVEN IF THEY ARE NOT UPDATED!
                Profiler.BeginSample("Tag_BuildBrushMeshLookup");//time=2.69ms
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
                Profiler.BeginSample("Tag_UpdateBrushTransformations");//time=2.69ms
                {
                    for (int i = 0; i < allTreeBrushIndices.Length; i++)
                    {
                        UpdateNodeTransformation(ref transformations, allTreeBrushIndices[i]);
                    }
                }
                Profiler.EndSample();

                Profiler.BeginSample("Tag_DirtyAllOutlines");
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
                Profiler.BeginSample("Tag_CompactTree.Create");//time=0.254ms
                var compactTree = CompactTree.Create(CSGManager.nodeHierarchies, treeNodeIndex); // Note: stored/destroyed in ChiselLookup
                Profiler.EndSample();


                Profiler.BeginSample("Tag_Allocations");//time=6.12ms
                Profiler.BeginSample("Tag_BrushOutputLoops");//time=5.44ms
                var brushOutputLoops    = new BrushOutputLoops[allTreeBrushIndices.Length];
                var brushOutputLoops2   = new BrushOutputLoops2[allTreeBrushIndices.Length];
                var brushHandles        = new BrushHandles[allTreeBrushIndices.Length];
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
                    baseMeshUpdateIndices       = baseMeshUpdateIndices,
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
                    brushOutputLoops2           = brushOutputLoops2,
                    brushHandles                = brushHandles
                };
                treeUpdateLength++;
            }
            Profiler.EndSample();


            // Sort trees from largest to smallest
            var treeSorter = new TreeSorter();
            Array.Sort(treeUpdates, treeSorter);

            Profiler.BeginSample("Tag_Jobified");


            // TODO: should only do this once at creation time, part of brushMeshBlob? store with brush component itself
            Profiler.BeginSample("Tag_GenerateBasePolygonLoops");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate  = ref treeUpdates[t];
                    if (treeUpdate.baseMeshUpdateIndices.Length > 0)
                    {
                        var createBlobPolygonsBlobs = new CreateBlobPolygonsBlobs //time=210.11ms
                        {
                            // Read
                            treeBrushIndices    = treeUpdate.baseMeshUpdateIndices,
                            brushMeshLookup     = treeUpdate.brushMeshLookup,
                            transformations     = treeUpdate.transformations,

                            // Write
                            basePolygons        = treeUpdate.basePolygons.AsParallelWriter()
                        };
                        treeUpdate.generateBasePolygonLoopsJobHandle = createBlobPolygonsBlobs.Schedule(treeUpdate.baseMeshUpdateIndices.Length, 16);
                    }
                }
            }
            finally { Profiler.EndSample(); }

            // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
            Profiler.BeginSample("Tag_FindIntersectingBrushes");
            try
            {
                // TODO: optimize, use hashed grid

                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var findAllIntersectionsJob = new FindAllBrushIntersectionsJob//time=5.16ms
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
                    var storeBrushIntersectionsJob = new StoreBrushIntersectionsJob//time=62.62ms
                    {
                        // Read
                        treeNodeIndex           = treeNodeIndex,
                        treeBrushIndices        = treeUpdate.allTreeBrushIndices,
                        compactTree             = treeUpdate.compactTree,
                        brushBrushIntersections = treeUpdate.brushBrushIntersections,

                        // Write
                        brushesTouchedByBrushes = treeUpdate.brushesTouchedByBrushes.AsParallelWriter()
                    };
                    treeUpdate.findIntersectingBrushesJobHandle = storeBrushIntersectionsJob.Schedule(treeUpdate.allTreeBrushIndices.Length, 16, treeUpdate.findAllIntersectionsJobHandle);
                }
            } finally { Profiler.EndSample(); }


            // TODO: should only do this at creation time + when moved / store with brush component itself
            Profiler.BeginSample("Tag_UpdateBrushWorldPlanes");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    // TODO: optimize, only do this when necessary
                    var createBrushWorldPlanesJob = new CreateBrushWorldPlanesJob//time=47.15ms
                    {
                        // Read
                        treeBrushIndices        = treeUpdate.allTreeBrushIndices,
                        brushMeshLookup         = treeUpdate.brushMeshLookup,
                        transformations         = treeUpdate.transformations,

                        // Write
                        brushWorldPlanes        = treeUpdate.brushWorldPlanes.AsParallelWriter()
                    };
                    treeUpdate.updateBrushWorldPlanesJobHandle   = createBrushWorldPlanesJob.Schedule(treeUpdate.allTreeBrushIndices.Length, 16);
                }
            } finally { Profiler.EndSample(); }

            // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
            Profiler.BeginSample("Tag_UpdateBrushCategorizationTables");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    // Build categorization trees for brushes
                    var createRoutingTableJob = new CreateRoutingTableJob//time=236.35ms
                    {
                        // Read
                        treeBrushIndices        = treeUpdate.allTreeBrushIndices,
                        brushesTouchedByBrushes = treeUpdate.brushesTouchedByBrushes,
                        compactTree             = treeUpdate.compactTree,

                        // Write
                        routingTableLookup      = treeUpdate.routingTableLookup.AsParallelWriter()
                    };
                    treeUpdate.updateBrushCategorizationTablesJobHandle = createRoutingTableJob.Schedule(treeUpdate.allTreeBrushIndices.Length, 16, treeUpdate.findIntersectingBrushesJobHandle);
                }
            } finally { Profiler.EndSample(); }
                                
            // Create unique loops between brush intersections
            Profiler.BeginSample("Tag_FindAllIntersectionLoops");
            try
            {
                // TODO: merge this with another job, there's not enough work 
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var findBrushPairsJob = new FindBrushPairsJob//0.685ms
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
                    var prepareBrushPairIntersectionsJob = new PrepareBrushPairIntersectionsJob//time=232.83ms
                    {
                        // Read
                        uniqueBrushPairs        = treeUpdate.uniqueBrushPairs.AsDeferredJobArray(),
                        brushMeshBlobLookup     = treeUpdate.brushMeshLookup,
                        transformations         = treeUpdate.transformations,

                        // Write
                        intersectingBrushes     = treeUpdate.intersectingBrushes.AsParallelWriter()
                    };
                    treeUpdate.prepareBrushPairIntersectionsJobHandle = prepareBrushPairIntersectionsJob.Schedule(treeUpdate.triangleArraySize, 16, treeUpdate.findBrushPairsJobHandle);
                }

                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var findAllIntersectionLoopsJob = new CreateIntersectionLoopsJob//time=812.09ms
                    {
                        // Read
                        brushWorldPlanes        = treeUpdate.brushWorldPlanes,
                        intersectingBrushes     = treeUpdate.intersectingBrushes.AsDeferredJobArray(),

                        // Write
                        outputSurfaces          = treeUpdate.intersectionLoopBlobs.AsParallelWriter()
                    };
                    treeUpdate.findAllIntersectionLoopsJobHandle = findAllIntersectionLoopsJob.Schedule(treeUpdate.triangleArraySize, 64, JobHandle.CombineDependencies(treeUpdate.updateBrushWorldPlanesJobHandle, treeUpdate.prepareBrushPairIntersectionsJobHandle));
                    treeUpdate.allFindAllIntersectionLoopsJobHandle = JobHandle.CombineDependencies(treeUpdate.findAllIntersectionLoopsJobHandle, treeUpdate.prepareBrushPairIntersectionsJobHandle, treeUpdate.allFindAllIntersectionLoopsJobHandle);
                }
            } finally { Profiler.EndSample(); }
                
            Profiler.BeginSample("Tag_FindLoopOverlapIntersections");
            try
            {
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    var treeNodeIndex = treeUpdate.treeNodeIndex;
                    var dependencies = treeUpdate.allFindAllIntersectionLoopsJobHandle;

                    for (int index = 0; index < treeUpdate.allTreeBrushIndices.Length; index++)
                    {
                        var outputLoops = treeUpdate.brushOutputLoops[index];
                        var findLoopOverlapIntersectionsJob = new FindLoopOverlapIntersectionsJob//time=206.81ms
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
                    treeUpdate.performCSGJobDependenciesJobHandle = JobHandle.CombineDependencies(treeUpdate.allFindLoopOverlapIntersectionsJobHandle, treeUpdate.updateBrushCategorizationTablesJobHandle);
                }
            }
            finally { Profiler.EndSample(); }

            Profiler.BeginSample("Tag_PerformCSGJob");
            for (int t = 0; t < treeUpdateLength; t++)
            {
                ref var treeUpdate = ref treeUpdates[t];

                // Perform CSG
                // TODO: only do this when necessary (when brushes have been modified)
                // TODO: determine when a brush is completely inside another brush
                //		 (might not have any intersection loops)

                for (int b = 0; b < treeUpdate.allTreeBrushIndices.Length; b++)
                {
                    ref var outputLoops     = ref treeUpdate.brushOutputLoops[b];
                    ref var outputLoops2    = ref treeUpdate.brushOutputLoops2[b];
                    ref var brushHandles    = ref treeUpdate.brushHandles[b];

                    var performCSGJob = new PerformCSGJob//time=1171.83ms
                    {
                        // Read
                        index                       = b,
                        treeBrushNodeIndices        = treeUpdate.allTreeBrushIndices,
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
                    brushHandles.performAllCSGJobHandle = performCSGJob.Schedule(treeUpdate.performCSGJobDependenciesJobHandle);
                }
            }
            Profiler.EndSample();

            Profiler.BeginSample("Tag_GenerateSurfaceTrianglesJob");
            for (int t = 0; t < treeUpdateLength; t++)
            {
                ref var treeUpdate = ref treeUpdates[t];


                // TODO: Cache the output surface meshes, only update when necessary
                for (int b = 0; b < treeUpdate.allTreeBrushIndices.Length; b++)
                {
                    var brushNodeIndex  = treeUpdate.allTreeBrushIndices[b];

                    ref var outputLoops         = ref treeUpdate.brushOutputLoops[b];
                    ref var outputLoops2        = ref treeUpdate.brushOutputLoops2[b];
                    ref var brushHandles        = ref treeUpdate.brushHandles[b];
                    var surfaceRenderBuffers    = treeUpdate.surfaceRenderBuffers[brushNodeIndex];  

                    var generateSurfaceRenderBuffers = new GenerateSurfaceTrianglesJob//time=786.62ms
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

                    brushHandles.generateSurfaceTrianglesJobHandle = generateSurfaceRenderBuffers.Schedule(brushHandles.performAllCSGJobHandle);
                    treeUpdate.allGenerateSurfaceTrianglesJobHandle = JobHandle.CombineDependencies(brushHandles.generateSurfaceTrianglesJobHandle, treeUpdate.allGenerateSurfaceTrianglesJobHandle);
                }
            }
            Profiler.EndSample();

            // Reset the flags before the dispose of these containers are scheduled
            for (int t = 0; t < treeUpdateLength; t++)
            {
                ref var treeUpdate = ref treeUpdates[t];
                for (int b = 0; b < treeUpdate.allTreeBrushIndices.Length; b++)
                { 
                    var brushNodeIndex = treeUpdate.allTreeBrushIndices[b];
                    var nodeFlags = CSGManager.nodeFlags[brushNodeIndex];
                    nodeFlags.status = NodeStatusFlags.None;
                    CSGManager.nodeFlags[brushNodeIndex] = nodeFlags;
                }
            }


            for (int t = 0; t < treeUpdateLength; t++)
            {
                ref var treeUpdate = ref treeUpdates[t];
                var treeNodeIndex = treeUpdate.treeNodeIndex;
                finalJobHandle = JobHandle.CombineDependencies(treeUpdate.allGenerateSurfaceTrianglesJobHandle, finalJobHandle);

                {
                    var flags = nodeFlags[treeNodeIndex];
                    flags.UnSetNodeFlag(NodeStatusFlags.TreeNeedsUpdate);
                    flags.SetNodeFlag(NodeStatusFlags.TreeMeshNeedsUpdate);
                    nodeFlags[treeNodeIndex] = flags;
                }
            }

            Profiler.EndSample();

            Profiler.BeginSample("Tag_Complete");
            finalJobHandle.Complete();
            Profiler.EndSample();

            // Note: Seems that scheduling a Dispose will cause previous jobs to be completed?
            //       Actually faster to just call them on main thread?
            Profiler.BeginSample("Tag_BrushOutputLoopsDispose");//time=14.72ms
            {
                var disposeJobHandle = finalJobHandle;
                for (int t = 0; t < treeUpdateLength; t++)
                {
                    ref var treeUpdate = ref treeUpdates[t];
                    for (int b = 0; b < treeUpdate.brushOutputLoops.Length; b++)
                    {
                        var outputLoops = treeUpdate.brushOutputLoops[b];
                        if (!outputLoops.intersectionSurfaceInfos.IsCreated)
                            continue;
                        outputLoops.intersectionSurfaceInfos.Dispose();//disposeJobHandle);
                        outputLoops.intersectionEdges       .Dispose();//disposeJobHandle);
                        outputLoops.basePolygonSurfaceInfos .Dispose();//disposeJobHandle);
                        outputLoops.basePolygonEdges        .Dispose();//disposeJobHandle);
                        outputLoops.vertexSoup              .Dispose();//disposeJobHandle);
                    } 
                    treeUpdate.brushOutputLoops = null;
                    
                    for (int b = 0; b < treeUpdate.brushOutputLoops2.Length; b++)
                    {
                        ref var outputLoops2 = ref treeUpdate.brushOutputLoops2[b];

                        outputLoops2.surfaceLoopIndices  .Dispose();//disposeJobHandle);
                        outputLoops2.allInfos            .Dispose();//disposeJobHandle);
                        outputLoops2.allEdges            .Dispose();//disposeJobHandle);
                    }
                    treeUpdate.brushOutputLoops2 = null;

                    treeUpdate.brushMeshLookup          .Dispose();//disposeJobHandle);                     
                    treeUpdate.allTreeBrushIndices      .Dispose();//disposeJobHandle);
                    treeUpdate.modifiedTreeBrushIndices .Dispose();//disposeJobHandle);                    
                    treeUpdate.baseMeshUpdateIndices    .Dispose();//disposeJobHandle);
                    treeUpdate.brushBrushIntersections  .Dispose();//disposeJobHandle);
                    treeUpdate.uniqueBrushPairs         .Dispose();//disposeJobHandle);
                    treeUpdate.intersectionLoopBlobs    .Dispose();//disposeJobHandle);
                    treeUpdate.intersectingBrushes      .Dispose();//disposeJobHandle);
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
