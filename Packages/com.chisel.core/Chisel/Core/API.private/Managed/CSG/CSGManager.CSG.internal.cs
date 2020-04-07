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

        internal static bool UpdateTreeMesh(int treeNodeID, out JobHandle finalJobHandle)
        {
            Debug.Log("UpdateTreeMesh");
            finalJobHandle = default(JobHandle);
            if (!IsValidNodeID(treeNodeID) || !AssertNodeType(treeNodeID, CSGNodeType.Tree))
                return false;

            var treeNodeIndex = treeNodeID - 1;
            var treeInfo = CSGManager.nodeHierarchies[treeNodeIndex].treeInfo;
            if (treeInfo == null)
                return false;

            Reset(treeInfo);

            // NOTE: THIS IS RUN **PER MODEL** AND WE HAVE MULTIPLE MODELS IN THE TEST SCENE
            // TODO: Make sure we complete for each model, or ensure that everything in model is stored completely separately

            var treeBrushes = treeInfo.treeBrushes;
            if (treeInfo.treeBrushes.Count > 0)
            {
                JobHandle performAllCSGJobHandle = default(JobHandle);
                var treeBrushesArray = treeBrushes.ToNativeArray(Allocator.TempJob);
                {
                    var chiselLookupValues  = ChiselTreeLookup.Value[treeNodeIndex];
                    var chiselMeshValues    = ChiselMeshLookup.Value;

                    // Clean up values we're rebuilding below
                    chiselLookupValues.RemoveByBrushID(treeBrushesArray);

                    var brushMeshInstanceIDs = new NativeArray<int>(treeBrushesArray.Length, Allocator.TempJob);
                    var brushMeshLookup      = new NativeHashMap<int, BlobAssetReference<BrushMeshBlob>>(treeBrushesArray.Length, Allocator.TempJob);
                    {
                        // TODO: somehow just keep this up to date instead of rebuilding it from scratch every update
                        FindBrushMeshInstanceIDs(treeBrushesArray, brushMeshInstanceIDs);
                        FindBrushMeshBobs(ref chiselMeshValues, treeBrushesArray, brushMeshLookup); // <-- assumes all brushes in tree


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
                        Profiler.BeginSample("GenerateBasePolygonLoops");
                        try
                        {
                            var createBlobPolygonsBlobs = new CreateBlobPolygonsBlobs()
                            {
                                // Read
                                treeBrushes             = treeBrushesArray,
                                brushMeshInstanceIDs    = brushMeshInstanceIDs,
                                brushMeshBlobs          = chiselMeshValues.brushMeshBlobs,
                                transformations         = chiselLookupValues.transformations,

                                // Write
                                basePolygons            = chiselLookupValues.basePolygons.AsParallelWriter()
                            };
                            generateBasePolygonLoopsJob = createBlobPolygonsBlobs.Schedule(treeBrushesArray.Length, 64);
                        } finally { Profiler.EndSample(); }


                        // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
                        JobHandle findIntersectingBrushesJob;
                        Profiler.BeginSample("FindIntersectingBrushes");
                        try
                        {
                            // TODO: optimize, use hashed grid

                            var triangleArraySize       = GeometryMath.GetTriangleArraySize(treeBrushesArray.Length);
                            var brushBrushIntersections = new NativeMultiHashMap<int, BrushPair>(triangleArraySize * 2, Allocator.TempJob);
                            var findAllIntersectionsJob = new FindAllIntersectionsJob
                            {
                                // Read
                                brushNodeIDs            = treeBrushesArray,
                                transformations         = chiselLookupValues.transformations,
                                brushMeshBlobs          = chiselMeshValues.brushMeshBlobs,
                                basePolygons            = chiselLookupValues.basePolygons,
                                brushMeshIDs            = brushMeshInstanceIDs,

                                // Write
                                output                  = brushBrushIntersections.AsParallelWriter()
                            };
                            var findAllIntersectionsJobHandle = findAllIntersectionsJob.Schedule(//triangleArraySize, 128, 
                                                                                                 generateBasePolygonLoopsJob);

                            var storeBrushIntersectionsJob = new StoreBrushIntersectionsJob
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

                        // TODO: should only do this at creation time + when moved / store with brush component itself
                        JobHandle updateBrushWorldPlanesJob;
                        Profiler.BeginSample("UpdateBrushWorldPlanes");
                        try
                        {
                            // TODO: optimize, only do this when necessary
                            var createBrushWorldPlanesJob = new CreateBrushWorldPlanesJob()
                            {
                                // Read
                                treeBrushes             = treeBrushesArray,
                                brushMeshLookup         = brushMeshLookup,
                                transformations         = chiselLookupValues.transformations,

                                // Write
                                brushWorldPlanes        = chiselLookupValues.brushWorldPlanes.AsParallelWriter()
                            };
                            updateBrushWorldPlanesJob   = createBrushWorldPlanesJob.Schedule(treeBrushesArray.Length, 64);
                        } finally { Profiler.EndSample(); }

                        // TODO: only change when brush or any touching brush has been added/removed or changes operation/order
                        JobHandle updateBrushCategorizationTablesJob;
                        Profiler.BeginSample("UpdateBrushCategorizationTables");
                        try
                        {
                            // Build categorization trees for brushes
                            var createRoutingTableJob = new CreateRoutingTableJob
                            {
                                // Read
                                treeBrushes             = treeBrushesArray,
                                brushesTouchedByBrushes = chiselLookupValues.brushesTouchedByBrushes,
                                compactTree             = compactTree,

                                // Write
                                routingTableLookup      = chiselLookupValues.routingTableLookup.AsParallelWriter()
                            };
                            updateBrushCategorizationTablesJob = createRoutingTableJob.Schedule(treeBrushesArray.Length, 64, findIntersectingBrushesJob);
                        } finally { Profiler.EndSample(); }

                        JobHandle findAllIntersectionLoopsJobHandle;
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
                                    // Read
                                    maxPairs                    = maxPairs,
                                    treeBrushes                 = treeBrushesArray,
                                    brushesTouchedByBrushes     = chiselLookupValues.brushesTouchedByBrushes,
                                    
                                    // Write
                                    uniqueBrushPairs            = uniqueBrushPairs,
                                };
                                var findBrushPairsJobHandle = findBrushPairsJob.Schedule(findIntersectingBrushesJob);

                                var intersectingBrushes = new NativeList<BlobAssetReference<BrushPairIntersection>>(GeometryMath.GetTriangleArraySize(treeBrushesArray.Length), Allocator.TempJob);
                                var prepareBrushPairIntersectionsJob = new PrepareBrushPairIntersectionsJob
                                {
                                    // Read
                                    uniqueBrushPairs        = uniqueBrushPairs.AsDeferredJobArray(),
                                    brushMeshBlobLookup     = brushMeshLookup,
                                    transformations         = chiselLookupValues.transformations,

                                    // Write
                                    intersectingBrushes     = intersectingBrushes.AsParallelWriter()
                                };
                                var prepareBrushPairIntersectionsJobHandle = prepareBrushPairIntersectionsJob.Schedule(maxPairs, 64, findBrushPairsJobHandle);
                                var createBrushPairsJobHandle = JobHandle.CombineDependencies(prepareBrushPairIntersectionsJobHandle, findBrushPairsJobHandle);
                                uniqueBrushPairs.Dispose(createBrushPairsJobHandle);

                                var findAllIntersectionLoopsJob = new CSGManagerPerformCSG.FindAllIntersectionLoopsJob
                                {
                                    // Read
                                    brushWorldPlanes        = chiselLookupValues.brushWorldPlanes,
                                    intersectingBrushes     = intersectingBrushes.AsDeferredJobArray(),

                                    // Write
                                    outputSurfaces          = intersectionLoopBlobs.AsParallelWriter()
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
                                    var basePolygonSurfaceInfos     = new NativeList<SurfaceInfo>(0, Allocator.TempJob);
                                    var basePolygonEdges            = new NativeListArray<Edge>(16, Allocator.TempJob);
                                    var intersectionSurfaceInfos    = new NativeList<SurfaceInfo>(0, Allocator.TempJob);
                                    var intersectionEdges           = new NativeListArray<Edge>(16, Allocator.TempJob);
                                    var vertexSoup                  = new VertexSoup(2048, Allocator.TempJob);

                                    
                                    //***GET RID OF THIS***//
                                    var brushNodeID = treeBrushesArray[index];
                                    var brushNodeIndex = brushNodeID - 1;

                                    vertexSoups[brushNodeIndex] = vertexSoup;

                                    var outputLoops = CSGManager.GetBrushInfo(brushNodeID).brushOutputLoops;/*!!*/
                                    outputLoops.Dispose();/*!!*/

                                    outputLoops.basePolygonSurfaceInfos     = basePolygonSurfaceInfos;
                                    outputLoops.basePolygonEdges            = basePolygonEdges;
                                    outputLoops.intersectionSurfaceInfos    = intersectionSurfaceInfos;
                                    outputLoops.intersectionEdges           = intersectionEdges;
                                    outputLoops.vertexSoup                  = vertexSoup;
                                    //***GET RID OF THIS***//


                                    var findLoopOverlapIntersectionsJob = new FindLoopOverlapIntersectionsJob
                                    {
                                        // Read
                                        index                       = index,
                                        treeBrushesArray            = treeBrushesArray,
                                        intersectionLoopBlobs       = intersectionLoopBlobs,
                                        brushWorldPlanes            = brushWorldPlaneBlobs,
                                        basePolygonBlobs            = basePolygonBlobs,

                                        // Read/Write
                                        vertexSoup                  = vertexSoup,
                                        basePolygonEdges            = basePolygonEdges,
                                        basePolygonSurfaceInfos     = basePolygonSurfaceInfos,
                                        intersectionEdges           = intersectionEdges,
                                        intersectionSurfaceInfos    = intersectionSurfaceInfos
                                    };
                                    findLoopOverlapIntersectionsJobHandle = JobHandle.CombineDependencies(findLoopOverlapIntersectionsJob.Schedule(dependencies), findLoopOverlapIntersectionsJobHandle);

                                }
                                intersectionLoopBlobs.Dispose(JobHandle.CombineDependencies(findLoopOverlapIntersectionsJobHandle, findAllIntersectionLoopsJobHandle));
                            }
                            finally { Profiler.EndSample(); }
                        }

                        Profiler.BeginSample("PerformAllCSG");
                        try
                        {
                            var dependencies = JobHandle.CombineDependencies(findLoopOverlapIntersectionsJobHandle, updateBrushCategorizationTablesJob);
                            
                            ref var routingTableLookup      = ref chiselLookupValues.routingTableLookup;
                            ref var brushWorldPlanes        = ref chiselLookupValues.brushWorldPlanes;
                            ref var brushesTouchedByBrushes = ref chiselLookupValues.brushesTouchedByBrushes;


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
                                if (output.brushSurfaceLoops.IsCreated)
                                    output.brushSurfaceLoops.Dispose();

                                if (brushMeshID == 0)
                                    continue;

                                output.brushSurfaceLoops = new BrushLoops();
                                output.brushSurfaceLoops.Allocate();

                                var performCSGJob = new PerformCSGJob
                                {
                                    // Read
                                    brushNodeIndex              = brushNodeIndex,
                                    routingTableLookup          = routingTableLookup,
                                    brushWorldPlanes            = brushWorldPlanes,
                                    brushesTouchedByBrushes     = brushesTouchedByBrushes,
                                    brushVertices               = outputLoops.vertexSoup,
                                    intersectionEdges           = outputLoops.intersectionEdges,
                                    intersectionSurfaceInfos    = outputLoops.intersectionSurfaceInfos,

                                    // Read / Write
                                    basePolygonSurfaceInfos     = outputLoops.basePolygonSurfaceInfos,
                                    basePolygonEdges            = outputLoops.basePolygonEdges,

                                    brushLoops                  = output.brushSurfaceLoops
                                };
                                performAllCSGJobHandle = JobHandle.CombineDependencies(performCSGJob.Schedule(dependencies), performAllCSGJobHandle);


                                performAllCSGJobHandle.Complete();
                                UnityEngine.Profiling.Profiler.BeginSample("Triangulate");

                                var brushLoops = output.brushSurfaceLoops;
                                chiselLookupValues.surfaceRenderBuffers[brushNodeIndex] = new NativeList<BlobAssetReference<ChiselSurfaceRenderBuffer>>(brushLoops.surfaceLoopIndices.Length, Allocator.Persistent); 

                                var generateSurfaceRenderBuffers = new GenerateSurfaceTrianglesJob
                                {
                                    brushNodeID             = brushNodeID,

                                    brushVertices           = chiselLookupValues.vertexSoups[brushNodeIndex],
                                    brushWorldPlanes        = chiselLookupValues.brushWorldPlanes[brushNodeIndex],
                                    basePolygonsBlob        = chiselLookupValues.basePolygons[brushNodeIndex],
                                    surfaceRenderBuffers    = chiselLookupValues.surfaceRenderBuffers[brushNodeIndex],

                                    surfaceLoopIndices      = brushLoops.surfaceLoopIndices,
                                    surfaceLoopAllInfos     = brushLoops.allInfos,
                                    surfaceLoopAllEdges     = brushLoops.allEdges
                                };
                                generateSurfaceRenderBuffers.Run();

                                UnityEngine.Profiling.Profiler.EndSample();
                            }
                        }
                        finally { Profiler.EndSample(); }

                        Profiler.BeginSample("DirtyAllOutlines");
                        try
                        {
                            UpdateAllOutlines(treeBrushesArray);
                        }
                        finally { Profiler.EndSample(); }

                    }
                    brushMeshInstanceIDs.Dispose(performAllCSGJobHandle);
                    brushMeshLookup.Dispose(performAllCSGJobHandle);
                }
                treeBrushesArray.Dispose(performAllCSGJobHandle);
                finalJobHandle = performAllCSGJobHandle;
            }

            {
                var flags = nodeFlags[treeNodeIndex];
                flags.UnSetNodeFlag(NodeStatusFlags.TreeNeedsUpdate);
                flags.SetNodeFlag(NodeStatusFlags.TreeMeshNeedsUpdate);
                nodeFlags[treeNodeIndex] = flags;
            }
            return true;
        }

        //[BurstCompile(CompileSynchronously = true)]
        unsafe struct GenerateSurfaceTrianglesJob : IJob
        {
            public int brushNodeID;
            [NoAlias, ReadOnly] public BlobAssetReference<BasePolygonsBlob>     basePolygonsBlob;
            [NoAlias, ReadOnly] public BlobAssetReference<BrushWorldPlanes>     brushWorldPlanes;
            [NoAlias, ReadOnly] public VertexSoup                               brushVertices;
            [NoAlias, ReadOnly] public NativeListArray<int>                     surfaceLoopIndices;
            [NoAlias, ReadOnly] public NativeList<SurfaceInfo>                  surfaceLoopAllInfos;
            [NoAlias, ReadOnly] public NativeListArray<Edge>                    surfaceLoopAllEdges;

            [NoAlias, WriteOnly] public NativeList<BlobAssetReference<ChiselSurfaceRenderBuffer>>   surfaceRenderBuffers;
            public void Execute()
            {
                var brushNodeIndex = brushNodeID - 1;

                var maxLoops = 0;
                var maxIndices = 0;
                for (int s = 0; s < surfaceLoopIndices.Length; s++)
                {
                    var length = surfaceLoopIndices[s].Length;
                    maxIndices += length;
                    maxLoops = math.max(maxLoops, length);
                }

                var pointCount = brushVertices.Length + 2;
                var context_points              = new NativeArray<float2>(pointCount, Allocator.Temp);
                var context_edges               = new NativeArray<ushort>(pointCount, Allocator.Temp);
                var context_allEdges            = new NativeList<Poly2Tri.DTSweep.DirectedEdge>(pointCount, Allocator.Temp);
                var context_sortedPoints        = new NativeList<ushort>(pointCount, Allocator.Temp);
                var context_triangles           = new NativeList<Poly2Tri.DTSweep.DelaunayTriangle>(pointCount * 3, Allocator.Temp);
                var context_triangleInterior    = new NativeList<bool>(pointCount * 3, Allocator.Temp);
                var context_advancingFrontNodes = new NativeList<Poly2Tri.DTSweep.AdvancingFrontNode>(pointCount, Allocator.Temp);
                var context_edgeLookupEdges     = new NativeListArray<Chisel.Core.Edge>(pointCount, Allocator.Temp);
                var context_edgeLookups         = new NativeHashMap<ushort, int>(pointCount, Allocator.Temp);
                var context_foundLoops          = new NativeListArray<Chisel.Core.Edge>(pointCount, Allocator.Temp);

                var context_children            = new NativeListArray<int>(64, Allocator.Temp);
                var context_inputEdgesCopy      = new NativeList<Edge>(64, Allocator.Temp);

                var context = new Poly2Tri.DTSweep
                {
                    vertices                = brushVertices,
                    points                  = context_points,
                    edges                   = context_edges,
                    allEdges                = context_allEdges,
                    sortedPoints            = context_sortedPoints,
                    triangles               = context_triangles,
                    triangleInterior        = context_triangleInterior,
                    advancingFrontNodes     = context_advancingFrontNodes,
                    edgeLookupEdges         = context_edgeLookupEdges,
                    edgeLookups             = context_edgeLookups,
                    foundLoops              = context_foundLoops,
                    children                = context_children,
                    inputEdgesCopy          = context_inputEdgesCopy,
                };

                var loops               = new NativeList<int>(maxLoops, Allocator.Temp);
                var surfaceIndexList    = new NativeList<int>(maxIndices, Allocator.Temp);
                for (int s = 0; s < surfaceLoopIndices.Length; s++)
                {
                    loops.Clear();

                    var loopIndices = surfaceLoopIndices[s];
                    for (int l = 0; l < loopIndices.Length; l++)
                    {
                        var surfaceLoopIndex = loopIndices[l];
                        var surfaceLoopInfo  = surfaceLoopAllInfos[surfaceLoopIndex];
                        var _interiorCategory = (CategoryIndex)surfaceLoopInfo.interiorCategory;
                        if (_interiorCategory > CategoryIndex.LastCategory)
                            Debug.Assert(false, $"Invalid final category {_interiorCategory}");

                        if (_interiorCategory != CategoryIndex.ValidAligned && 
                            _interiorCategory != CategoryIndex.ValidReverseAligned)
                            continue;

                        var surfaceLoopEdges   = surfaceLoopAllEdges[surfaceLoopIndex];
                        if (surfaceLoopEdges.Length < 3)
                            continue;

                        loops.Add(surfaceLoopIndex);
                    }

                    // TODO: why are we doing this in tree-space? better to do this in brush-space, then we can more easily cache this
                    var surfaceIndex            = s;
                    var surfaceLayers           = basePolygonsBlob.Value.surfaces[surfaceIndex].surfaceInfo.layers;
                    var surfaceWorldPlane       = brushWorldPlanes.Value.worldPlanes[surfaceIndex];
                    var UV0		                = basePolygonsBlob.Value.surfaces[surfaceIndex].UV0;
                    var localSpaceToPlaneSpace	= MathExtensions.GenerateLocalToPlaneSpaceMatrix(surfaceWorldPlane);
                    var uv0Matrix				= math.mul(UV0.ToFloat4x4(), localSpaceToPlaneSpace);

                    // Ensure we have the rotation properly calculated, and have a valid normal
                    quaternion rotation;
                    if (((Vector3)surfaceWorldPlane.xyz) == Vector3.forward)
                        rotation = quaternion.identity;
                    else
                        rotation = (quaternion)Quaternion.FromToRotation(surfaceWorldPlane.xyz, Vector3.forward);


                    surfaceIndexList.Clear();

                    CategoryIndex   interiorCategory    = CategoryIndex.ValidAligned;

                    for (int l = 0; l < loops.Length; l++)
                    {
                        var loopIndex   = loops[l];
                        var loopEdges   = surfaceLoopAllEdges[loopIndex];
                        var loopInfo    = surfaceLoopAllInfos[loopIndex];
                        interiorCategory = (CategoryIndex)loopInfo.interiorCategory;

                        Debug.Assert(surfaceIndex == loopInfo.basePlaneIndex);

                        var surfaceIndicesArray = new NativeList<int>(Allocator.Temp);
                        try
                        {
                            context.TriangulateLoops(loopInfo, rotation, loopEdges, surfaceIndicesArray);
                        }
                        catch (System.Exception e)
                        {
                            Debug.LogException(e);
                            surfaceIndicesArray.Clear();
                        }

                        if (surfaceIndicesArray.Length >= 3)
                        {
                            if (interiorCategory == CategoryIndex.ValidReverseAligned ||
                                interiorCategory == CategoryIndex.ReverseAligned)
                            {
                                var maxCount = surfaceIndicesArray.Length - 1;
                                for (int n = (maxCount / 2); n >= 0; n--)
                                {
                                    var t = surfaceIndicesArray[n];
                                    surfaceIndicesArray[n] = surfaceIndicesArray[maxCount - n];
                                    surfaceIndicesArray[maxCount - n] = t;
                                }
                            }

                            for (int n = 0; n < surfaceIndicesArray.Length; n++)
                                surfaceIndexList.Add(surfaceIndicesArray[n]);
                        }
                        surfaceIndicesArray.Dispose();
                    }

                    if (surfaceIndexList.Length == 0)
                        continue;

                    var surfaceIndicesCount = surfaceIndexList.Length;
                    var surfaceIndices      = (int*)surfaceIndexList.GetUnsafePtr();

                    // Only use the vertices that we've found in the indices
                    var surfaceVerticesCount    = 0;
                    var surfaceVertices         = stackalloc float3[brushVertices.Length];
                    var indexRemap              = stackalloc int[brushVertices.Length];
                    for (int i = 0; i < surfaceIndicesCount; i++)
                    {
                        var vertexIndexSrc = surfaceIndices[i];
                        var vertexIndexDst = indexRemap[vertexIndexSrc];
                        if (vertexIndexDst == 0)
                        {
                            vertexIndexDst = surfaceVerticesCount;
                            surfaceVertices[surfaceVerticesCount] = brushVertices[vertexIndexSrc];
                            surfaceVerticesCount++;
                            indexRemap[vertexIndexSrc] = vertexIndexDst + 1;
                        } else
                            vertexIndexDst--;
                        surfaceIndices[i] = vertexIndexDst;
                    }

                    var vertexHash	    = math.hash(surfaceVertices, surfaceVerticesCount);
                    var indicesHash	    = math.hash(surfaceIndices, surfaceIndicesCount);
                    var geometryHash    = math.hash(new uint2(vertexHash, indicesHash));

                    var surfaceNormals = stackalloc float3[surfaceVerticesCount];
                    {
                        var normal = (interiorCategory == CategoryIndex.ValidReverseAligned || interiorCategory == CategoryIndex.ReverseAligned) ? -surfaceWorldPlane.xyz : surfaceWorldPlane.xyz;
                        for (int i = 0; i < surfaceVerticesCount; i++)
                            surfaceNormals[i] = normal;
                    }
                    var normalHash = math.hash(surfaceNormals, surfaceVerticesCount);
                    var surfaceUV0  = stackalloc float2[surfaceVerticesCount];
                    {
                        for (int v = 0; v < surfaceVerticesCount; v++)
                            surfaceUV0[v] = math.mul(uv0Matrix, new float4(surfaceVertices[v], 1)).xy;
                    }
                    var uv0Hash = math.hash(surfaceUV0, surfaceVerticesCount);

                    var builder = new BlobBuilder(Allocator.Temp);
                    ref var root = ref builder.ConstructRoot<ChiselSurfaceRenderBuffer>();
                    builder.Construct(ref root.indices,     surfaceIndices,     surfaceIndicesCount);
                    builder.Construct(ref root.vertices,    surfaceVertices,    surfaceVerticesCount);
                    builder.Construct(ref root.normals,     surfaceNormals,     surfaceVerticesCount);
                    builder.Construct(ref root.uv0,         surfaceUV0,         surfaceVerticesCount);

                    root.surfaceHash        = math.hash(new uint2(normalHash, uv0Hash));
                    root.geometryHash       = geometryHash;
                    root.surfaceLayers      = surfaceLayers;
                    root.surfaceIndex       = surfaceIndex;
                    var surfaceRenderBuffer = builder.CreateBlobAssetReference<ChiselSurfaceRenderBuffer>(Allocator.Persistent);
                    builder.Dispose();

                    surfaceRenderBuffers.Add(surfaceRenderBuffer);
                }
                loops.Dispose();
                surfaceIndexList.Dispose();


                context_children.Dispose();
                context_inputEdgesCopy.Dispose();

                context_points.Dispose();
                context_edges.Dispose();

                context_allEdges.Dispose();
                context_sortedPoints.Dispose();
                context_triangles.Dispose();
                context_triangleInterior.Dispose();
                context_advancingFrontNodes.Dispose();
                context_edgeLookupEdges.Dispose();
                context_edgeLookups.Dispose();
                context_foundLoops.Dispose();
            }
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
                if (output.brushSurfaceLoops.IsCreated) output.brushSurfaceLoops.Dispose();
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

        static void UpdateBrushTransformations(ref ChiselTreeLookup.Data chiselLookupValues, List<int> treeBrushes)
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

        static void FindBrushMeshBobs(ref ChiselMeshLookup.Data chiselMeshValues, NativeArray<int> treeBrushesArray, NativeHashMap<int, BlobAssetReference<BrushMeshBlob>> brushMeshLookup)
        {
            ref var brushMeshBlobs = ref chiselMeshValues.brushMeshBlobs;
            for (int i = 0; i < treeBrushesArray.Length; i++)
            {
                var brushNodeID = treeBrushesArray[i];
                var brushNodeIndex = brushNodeID - 1;
                brushMeshLookup[brushNodeIndex] = brushMeshBlobs[nodeHierarchies[brushNodeIndex].brushInfo.brushMeshInstanceID - 1];
            }
        }


        static void UpdateAllOutlines(NativeArray<int> treeBrushesArray)
        {
            for (int b = 0; b < treeBrushesArray.Length; b++)
            {
                var brushNodeID = treeBrushesArray[b];
                var brushInfo   = CSGManager.GetBrushInfo(brushNodeID);
                brushInfo.brushOutlineGeneration++;
                brushInfo.brushOutlineDirty = true;
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        unsafe struct PerformCSGJob : IJob
        {
            [NoAlias, ReadOnly] public int                                                              brushNodeIndex;
            //[NoAlias, ReadOnly] public BlobAssetReference<RoutingTable>                                 routingTable;
            [NoAlias, ReadOnly] public VertexSoup                                                       brushVertices;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<RoutingTable>>             routingTableLookup;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushWorldPlanes>>         brushWorldPlanes;
            [NoAlias, ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>    brushesTouchedByBrushes;

            [NoAlias, ReadOnly] public NativeList<SurfaceInfo>  intersectionSurfaceInfos;
            [NoAlias, ReadOnly] public NativeListArray<Edge>    intersectionEdges;

            [NoAlias] public NativeList<SurfaceInfo>            basePolygonSurfaceInfos;    // <-- should be readonly?
            [NoAlias] public NativeListArray<Edge>              basePolygonEdges;           // <-- should be readonly?

            [NoAlias] public BrushLoops                         brushLoops;

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

                brushLoops.surfaceLoopIndices.ResizeExact(surfaceCount);
                for (int s = 0; s < surfaceCount; s++)
                {
                    var info = basePolygonSurfaceInfos[s];
                    info.interiorCategory = CategoryGroupIndex.First; // TODO: make sure that it's always set to "First" so we don't need to do this

                    var loopIndices = brushLoops.surfaceLoopIndices[s];
                    loopIndices.Add(brushLoops.allEdges.Length);
                    brushLoops.holeIndices.Add();
                    brushLoops.allInfos.Add(info);
                    brushLoops.allEdges.Add(basePolygonEdges[s]);
                }

                for (int i = 0, offset = 0; i < routingLookups.Length; i++)
                {
                    ref var routingLookup = ref routingLookups[i];
                    for (int surfaceIndex = 0; surfaceIndex < surfaceCount; surfaceIndex++, offset++)
                    {
                        var loopIndices         = brushLoops.surfaceLoopIndices[surfaceIndex];
                        var intersectionLoop    = intersectionLoops[offset];
                        var intersectionInfo    = intersectionSurfaceInfo[offset];
                        for (int l = loopIndices.Length - 1; l >= 0; l--)
                        {
                            var surfaceLoopIndex = loopIndices[l];
                            var surfaceLoopEdges = brushLoops.allEdges[surfaceLoopIndex];
                            if (surfaceLoopEdges.Length < 3)
                                continue;

                            var surfaceLoopInfo = brushLoops.allInfos[surfaceLoopIndex];

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
                                brushLoops.allInfos[surfaceLoopIndex] = surfaceLoopInfo;
                                continue;
                            } else
                            {
                                surfaceLoopInfo.interiorCategory = routingRow[CategoryIndex.Outside];
                                brushLoops.allInfos[surfaceLoopIndex] = surfaceLoopInfo;
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

                                    holeIndices             = brushLoops.holeIndices,
                                    allInfos                = brushLoops.allInfos,
                                    allEdges                = brushLoops.allEdges,
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
                                
                            holeIndices     = brushLoops.holeIndices,
                            allInfos        = brushLoops.allInfos,
                            allEdges        = brushLoops.allEdges,
                        };
                        removeEmptyLoopsJob.Execute();
                        //Profiler.EndSample();
                    }
                }

                intersectionLoops.Dispose();
                intersectionSurfaceInfo.Dispose();

                //Profiler.BeginSample("CleanUp");
                {
                    for (int surfaceIndex = 0; surfaceIndex < brushLoops.surfaceLoopIndices.Length; surfaceIndex++)
                    {
                        var loopIndices = brushLoops.surfaceLoopIndices[surfaceIndex];
                        var cleanUpJob = new CleanUpJob
                        {
                            brushVertices       = brushVertices,
                            brushWorldPlanes    = brushWorldPlanes,
                            loopIndices         = loopIndices,
                                
                            holeIndices         = brushLoops.holeIndices,
                            allInfos            = brushLoops.allInfos,
                            allEdges            = brushLoops.allEdges,
                        };
                        cleanUpJob.Execute();
                    }
                }
                //Profiler.EndSample();
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

            for (int t = 0; t < trees.Count; t++)
            {
                UpdateTreeMesh(trees[t], out JobHandle handle);
                allTrees = JobHandle.CombineDependencies(handle, allTrees);
            }
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
