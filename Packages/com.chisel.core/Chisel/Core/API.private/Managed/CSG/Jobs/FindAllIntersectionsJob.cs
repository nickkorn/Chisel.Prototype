using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using ReadOnlyAttribute = Unity.Collections.ReadOnlyAttribute;
using System.Runtime.CompilerServices;
using Unity.Entities;

namespace Chisel.Core
{
    [BurstCompile(Debug = false)]
    unsafe struct FindAllIntersectionsJob : IJobParallelFor
    {
        const double kEpsilon = CSGManagerPerformCSG.kEpsilon;

        [ReadOnly] public NativeArray<int>  treeBrushes;
        // TODO: store blobs in brushMeshInstanceIDs to avoid indirection?
        [ReadOnly] public NativeArray<int>  brushMeshInstanceIDs;
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushMeshBlob>> brushMeshBlobs;
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<NodeTransformations>> transformations;
        // TODO: combine bounds with something else?
        [ReadOnly] public NativeArray<AABB> bounds;

        //[WriteOnly] public NativeStream.Writer output;
        [WriteOnly] public NativeMultiHashMap<int, BrushBrushIntersection>.ParallelWriter output;

        static void TransformOtherIntoBrushSpace(ref float4x4 treeToBrushSpaceMatrix, ref float4x4 brushToTreeSpaceMatrix, ref BlobArray<float4> srcPlanes, float4* dstPlanes)
        {
            var brush1ToBrush0LocalLocalSpace = math.transpose(math.mul(treeToBrushSpaceMatrix, brushToTreeSpaceMatrix));
            for (int plane_index = 0; plane_index < srcPlanes.Length; plane_index++)
            {
                ref var srcPlane = ref srcPlanes[plane_index];
                dstPlanes[plane_index] = math.mul(brush1ToBrush0LocalLocalSpace, srcPlane);
            }
        }


        static IntersectionType ConvexPolytopeTouching(BlobAssetReference<BrushMeshBlob> brushMesh0,
                                                       ref float4x4 treeToNode0SpaceMatrix,
                                                       ref float4x4 nodeToTree0SpaceMatrix,
                                                       BlobAssetReference<BrushMeshBlob> brushMesh1,
                                                       ref float4x4 treeToNode1SpaceMatrix,
                                                       ref float4x4 nodeToTree1SpaceMatrix)
        {
            ref var brushPlanes0   = ref brushMesh0.Value.localPlanes;
            ref var brushPlanes1   = ref brushMesh1.Value.localPlanes;

            ref var brushVertices0 = ref brushMesh0.Value.vertices;
            ref var brushVertices1 = ref brushMesh1.Value.vertices;

            var transformedPlanes0 = stackalloc float4[brushPlanes0.Length];
            TransformOtherIntoBrushSpace(ref treeToNode0SpaceMatrix, ref nodeToTree1SpaceMatrix, ref brushPlanes0, transformedPlanes0);
            
            int negativeSides1 = 0;
            for (var i = 0; i < brushPlanes0.Length; i++)
            {
                var plane0 = transformedPlanes0[i];
                int side = WhichSide(ref brushVertices1, plane0, kEpsilon);
                if (side < 0) negativeSides1++;
                if (side > 0) return IntersectionType.NoIntersection;
            }

            //if (intersectingSides1 != transformedPlanes0.Length) return IntersectionType.Intersection;
            //if (intersectingSides > 0) return IntersectionType.Intersection;
            //if (positiveSides1 > 0) return IntersectionType.NoIntersection;
            //if (negativeSides > 0 && positiveSides > 0) return IntersectionType.Intersection;
            if (negativeSides1 == brushPlanes0.Length)
                return IntersectionType.BInsideA;

            //*
            var transformedPlanes1 = stackalloc float4[brushPlanes1.Length];
            TransformOtherIntoBrushSpace(ref treeToNode1SpaceMatrix, ref nodeToTree0SpaceMatrix, ref brushPlanes1, transformedPlanes1);

            int negativeSides2 = 0;
            int intersectingSides2 = 0;
            for (var i = 0; i < brushPlanes1.Length; i++)
            {
                var plane1 = transformedPlanes1[i];
                int side = WhichSide(ref brushVertices0, plane1, kEpsilon);
                if (side < 0) negativeSides2++;
                if (side > 0) return IntersectionType.NoIntersection;
                if (side == 0) intersectingSides2++;
            }

            if (intersectingSides2 > 0) return IntersectionType.Intersection;
            //if (negativeSides > 0 && positiveSides > 0) return IntersectionType.Intersection;
            if (negativeSides2 == brushPlanes1.Length)
                return IntersectionType.AInsideB;
            
            return IntersectionType.Intersection;//*/
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int WhichSide(ref BlobArray<float3> vertices, float4 plane, double epsilon)
        {
            {
                var t = math.dot(plane, new float4(vertices[0], 1));
                if (t >=  epsilon) goto HavePositive;
                if (t <= -epsilon) goto HaveNegative;
                return 0;
            }
        HaveNegative:
            for (var i = 1; i < vertices.Length; i++)
            {
                var t = math.dot(plane, new float4(vertices[i], 1));
                if (t > -epsilon)
                    return 0;
            }
            return -1;
        HavePositive:
            for (var i = 1; i < vertices.Length; i++)
            {
                var t = math.dot(plane, new float4(vertices[i], 1));
                if (t < epsilon)
                    return 0;
            }
            return 1;
        }

        public void Execute(int index)
        {
            //output.BeginForEachIndex(index);

            var arrayIndex = GeometryMath.GetTriangleArrayIndex(index, treeBrushes.Length);
            var index0 = arrayIndex.x;
            var index1 = arrayIndex.y;

            /*
            var brushIndex0 = index % brushData.Length;
            var brushIndex1 = index / brushData.Length;

            // Remove this inefficiency
            if (brushIndex0 >= brushIndex1)
                return;
            */
            {
                var brushMesh0 = brushMeshBlobs[brushMeshInstanceIDs[index0] - 1];
                var brushMesh1 = brushMeshBlobs[brushMeshInstanceIDs[index1] - 1];

                var bounds0 = bounds[index0];
                var bounds1 = bounds[index1];

                IntersectionType result;
                if (brushMesh0 != BlobAssetReference<BrushMeshBlob>.Null &&
                    brushMesh1 != BlobAssetReference<BrushMeshBlob>.Null &&
                    bounds0.Intersects(bounds1, kEpsilon))
                {
                    var brush0NodeID = treeBrushes[index0];
                    var brush1NodeID = treeBrushes[index1];
                    //*
                    var brush0NodeIndex = brush0NodeID - 1;
                    var brush1NodeIndex = brush1NodeID - 1;

                    ref var transformation0 = ref transformations[brush0NodeIndex].Value;
                    ref var transformation1 = ref transformations[brush1NodeIndex].Value;

                    var treeToNode0SpaceMatrix = transformation0.treeToNode;
                    var nodeToTree0SpaceMatrix = transformation0.nodeToTree;
                    var treeToNode1SpaceMatrix = transformation1.treeToNode;
                    var nodeToTree1SpaceMatrix = transformation1.nodeToTree;
                    /*/
                    var treeToNode0SpaceMatrix = brushData[brushIndex0].treeToNodeSpaceMatrix;
                    var nodeToTree0SpaceMatrix = brushData[brushIndex0].nodeToTreeSpaceMatrix;
                    var treeToNode1SpaceMatrix = brushData[brushIndex1].treeToNodeSpaceMatrix;
                    var nodeToTree1SpaceMatrix = brushData[brushIndex1].nodeToTreeSpaceMatrix;
                    //*/
                    result = ConvexPolytopeTouching(brushMesh0,
                                                    ref treeToNode0SpaceMatrix,
                                                    ref nodeToTree0SpaceMatrix,
                                                    brushMesh1,
                                                    ref treeToNode1SpaceMatrix,
                                                    ref nodeToTree1SpaceMatrix);

                    if (result != IntersectionType.NoIntersection)
                    {
                        if (result == IntersectionType.Intersection)
                        {
                            output.Add(brush0NodeID, new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.Intersection });
                            output.Add(brush1NodeID, new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.Intersection });
                        } else
                        if (result == IntersectionType.AInsideB)
                        {
                            output.Add(brush0NodeID, new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.AInsideB });
                            output.Add(brush1NodeID, new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.BInsideA });
                        } else
                        //if (intersectionType == IntersectionType.BInsideA)
                        {
                            output.Add(brush0NodeID, new BrushBrushIntersection() { brushNodeID0 = brush0NodeID, brushNodeID1 = brush1NodeID, type = IntersectionType.BInsideA });
                            output.Add(brush1NodeID, new BrushBrushIntersection() { brushNodeID0 = brush1NodeID, brushNodeID1 = brush0NodeID, type = IntersectionType.AInsideB });
                        }
                    }
                }
            }
            //output.EndForEachIndex();
        }
    }

    [BurstCompile(Debug = false)]
    unsafe struct StoreBrushIntersectionsJob : IJobParallelFor
    {
        [ReadOnly] public int                               treeNodeIndex;
        [ReadOnly] public NativeArray<int>                  treeBrushes;
        [ReadOnly] public BlobAssetReference<CompactTree>   compactTree;
        [ReadOnly] public NativeMultiHashMap<int, BrushBrushIntersection> brushBrushIntersections;
        [WriteOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>>.ParallelWriter brushesTouchedByBrushes;


        static void SetUsedNodesBits(BlobAssetReference<CompactTree> compactTree, NativeList<BrushIntersection> brushIntersections, int brushNodeIndex, int rootNodeIndex, BrushIntersectionLookup bitset)
        {
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

            for (int i = 0; i < brushIntersections.Length; i++)
            {
                var otherIntersectionInfo = brushIntersections[i];
                bitset.Set(otherIntersectionInfo.nodeIndex, otherIntersectionInfo.type);
                for (int b = otherIntersectionInfo.bottomUpStart; b < otherIntersectionInfo.bottomUpEnd; b++)
                    bitset.Set(bottomUpNodes[b], IntersectionType.Intersection);
            }
        }
        
        static BlobAssetReference<BrushesTouchedByBrush> GenerateBrushesTouchedByBrush(BlobAssetReference<CompactTree> compactTree, int brushNodeIndex, int rootNodeIndex, NativeMultiHashMap<int, BrushBrushIntersection>.Enumerator touchingBrushes)
        {
            if (!compactTree.IsCreated)
                return BlobAssetReference<BrushesTouchedByBrush>.Null;

            var indexOffset = compactTree.Value.indexOffset;
            ref var bottomUpNodeIndices         = ref compactTree.Value.bottomUpNodeIndices;
            ref var brushIndexToBottomUpIndex   = ref compactTree.Value.brushIndexToBottomUpIndex;

            // Intersections
            var bitset                      = new BrushIntersectionLookup(indexOffset, bottomUpNodeIndices.Length, Allocator.Temp);
            var brushIntersectionIndices    = new NativeList<BrushIntersectionIndex>(Allocator.Temp);
            var brushIntersections          = new NativeList<BrushIntersection>(Allocator.Temp);
            { 
                var intersectionStart           = brushIntersections.Length;

                while (touchingBrushes.MoveNext())
                {
                    var touchingBrush   = touchingBrushes.Current;
                    int otherBrushID    = touchingBrush.brushNodeID1;
                    var otherBrushIndex = otherBrushID - 1;
                    if ((otherBrushIndex < indexOffset || (otherBrushIndex-indexOffset) >= brushIndexToBottomUpIndex.Length))
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
                    intersectionEnd     = brushIntersections.Length
                });

                SetUsedNodesBits(compactTree, brushIntersections, brushNodeIndex, rootNodeIndex, bitset);
            }
            var builder = new BlobBuilder(Allocator.Temp);
            ref var root = ref builder.ConstructRoot<BrushesTouchedByBrush>();
            builder.Construct(ref root.brushIntersections, brushIntersections);
            builder.Construct(ref root.intersectionBits, bitset.twoBits);
            root.Length = bitset.Length;
            root.Offset = bitset.Offset;
            var result = builder.CreateBlobAssetReference<BrushesTouchedByBrush>(Allocator.Persistent);
            builder.Dispose();
            brushIntersectionIndices.Dispose();
            brushIntersections.Dispose();
            bitset.Dispose();
            return result;
        }

        public void Execute(int index)
        {
            var brushNodeID = treeBrushes[index];
            var brushIntersections = brushBrushIntersections.GetValuesForKey(brushNodeID);
            {
                var result = GenerateBrushesTouchedByBrush(compactTree, brushNodeID - 1, treeNodeIndex, brushIntersections);
                if (result.IsCreated)
                    brushesTouchedByBrushes.TryAdd(brushNodeID - 1, result);
            }
            brushIntersections.Dispose();
        }
    }
}
