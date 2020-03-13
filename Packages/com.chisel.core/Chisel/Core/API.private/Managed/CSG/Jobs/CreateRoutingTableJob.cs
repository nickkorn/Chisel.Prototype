#define USE_OPTIMIZATIONS
//#define SHOW_DEBUG_MESSAGES 
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
#if USE_MANAGED_CSG_IMPLEMENTATION
    [BurstCompile]
    internal unsafe struct CreateRoutingTableJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int>                          treeBrushes;
        [ReadOnly] public NativeHashMap<int, BlobAssetReference<BrushesTouchedByBrush>> brushesTouchedByBrushes;
        [ReadOnly] public BlobAssetReference<CompactTree>           compactTree;
        [ReadOnly] public NativeArray<CategoryRoutingRow>           operationTables;
        [WriteOnly] public NativeHashMap<int, BlobAssetReference<RoutingTable>>.ParallelWriter routingTableLookup;

        public void Execute(int index)
        {
            var processedNodeID = treeBrushes[index];
            var processedNodeIndex = processedNodeID - 1;

            int categoryStackNodeCount, polygonGroupCount;
            if (brushesTouchedByBrushes.TryGetValue(processedNodeIndex, out BlobAssetReference<BrushesTouchedByBrush> brushesTouchedByBrush))
            {
                var routingTable = new NativeList<CategoryStackNode>(Allocator.Temp);
                {
                    GetStackNodes(in operationTables, ref compactTree.Value.topDownNodes, ref brushesTouchedByBrush.Value, processedNodeIndex, routingTable);

#if SHOW_DEBUG_MESSAGES
                    if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                        Dump(processedNode, stack); 
#endif
                    categoryStackNodeCount = (int)routingTable.Length;

                    int maxCounter = (int)CategoryRoutingRow.Length;
                    for (int i = 0; i < categoryStackNodeCount; i++)
                        maxCounter = Math.Max(maxCounter, (int)routingTable[i].input);
                    polygonGroupCount = maxCounter + 1;

                    var inputs          = new NativeList<CategoryGroupIndex>(Allocator.Temp);
                    var routingRows     = new NativeList<CategoryRoutingRow>(Allocator.Temp);
                    var routingLookups  = new NativeList<RoutingLookup>(Allocator.Temp);
                    var nodes           = new NativeList<int>(Allocator.Temp);
                    {
                        // TODO: clean up
                        for (int i = 0; i < routingTable.Length;)
                        {
                            var cutting_node_index = routingTable[i].nodeIndex;

                            int start_index = i;
                            do
                            {
                                inputs.Add(routingTable[i].input);
                                routingRows.Add(routingTable[i].routingRow);
                                i++;
                            } while (i < routingTable.Length && routingTable[i].nodeIndex == cutting_node_index);
                            int end_index = i;


                            nodes.Add(cutting_node_index + 1);
                            routingLookups.Add(new RoutingLookup(start_index, end_index));
                        }

                        var routingTableBlob = RoutingTable.Build(inputs, routingRows, routingLookups, nodes);
                        if (!routingTableLookup.TryAdd(processedNodeIndex, routingTableBlob))
                            FailureMessage();
                    }
                    inputs.Dispose();
                    routingRows.Dispose();
                    routingLookups.Dispose();
                    nodes.Dispose();
                }
                routingTable.Dispose();
            }
        }

        [BurstDiscard]
        static void FailureMessage()
        {
            Debug.LogError("Unity Burst Compiler is broken");
        }

        // Remap indices to new destinations, used when destination rows have been merged
        static void RemapIndices(NativeList<CategoryStackNode> stack, NativeArray<int> remap, int start, int last)
        {
            for (int i = start; i < last; i++)
            {
                var categoryRow = stack[i];
                var routingRow = categoryRow.routingRow;

                for (int r = 0; r < CategoryRoutingRow.Length; r++)
                {
                    var key = (int)routingRow[r];
                    if (key >= remap.Length || remap[key] == 0) { FailureMessage(); return; }
                }

                for (int r = 0; r < CategoryRoutingRow.Length; r++)
                    routingRow[r] = (CategoryGroupIndex)(remap[(int)routingRow[r]] - 1);

                categoryRow.routingRow = routingRow;
                stack[i] = categoryRow;
            }
        }

        // TODO: rewrite in such a way that we don't rely on stack
        public static void GetStackNodes(in NativeArray<CategoryRoutingRow> operationTables, ref BlobArray<CompactTopDownNode> topDownNodes, ref BrushesTouchedByBrush brushesTouchedByBrush, int processedNodeIndex, NativeList<CategoryStackNode> output)
        {
            int haveGonePastSelf = 0;
            output.Clear();
            GetStack(in operationTables, ref topDownNodes, ref brushesTouchedByBrush, processedNodeIndex, ref topDownNodes[0], ref haveGonePastSelf, output);
            if (output.Length == 0)
                output.Add(new CategoryStackNode() { nodeIndex = processedNodeIndex, operation = CSGOperationType.Additive, routingRow = CategoryRoutingRow.outside });
        }

        static void GetStack(in NativeArray<CategoryRoutingRow> operationTables, ref BlobArray<CompactTopDownNode> topDownNodes, ref BrushesTouchedByBrush brushesTouchedByBrush, int processedNodeIndex, ref CompactTopDownNode currentNode, ref int haveGonePastSelf, NativeList<CategoryStackNode> output)
        {
            var intersectionType    = brushesTouchedByBrush.Get(currentNode.nodeIndex);
            if (intersectionType == IntersectionType.NoIntersection)
                return;// sEmptyStack.ToList();


            // TODO: use other intersection types
            if (intersectionType == IntersectionType.AInsideB) { output.Add(new CategoryStackNode() { nodeIndex = currentNode.nodeIndex, operation = currentNode.Operation, routingRow = CategoryRoutingRow.inside }); return; }
            if (intersectionType == IntersectionType.BInsideA) { output.Add(new CategoryStackNode() { nodeIndex = currentNode.nodeIndex, operation = currentNode.Operation, routingRow = CategoryRoutingRow.outside }); return; }

            switch (currentNode.Type)
            {
                case CSGNodeType.Brush:
                {
                    // All surfaces of processedNode are aligned with it's own surfaces, so all categories are Aligned
                    if (processedNodeIndex == currentNode.nodeIndex)
                    {
                        haveGonePastSelf = 1;
                        output.Add(new CategoryStackNode() { nodeIndex = currentNode.nodeIndex, operation = currentNode.Operation, routingRow = CategoryRoutingRow.selfAligned } );
                        return;
                    }

                    if (haveGonePastSelf > 0)
                        haveGonePastSelf = 2;

                    // Otherwise return identity categories (input == output)
                    output.Add(new CategoryStackNode() { nodeIndex = currentNode.nodeIndex, operation = currentNode.Operation, routingRow = CategoryRoutingRow.identity });
                    return;
                }
                default:
                {
                    var nodeCount = currentNode.childCount;
                    if (nodeCount == 0)
                        return;// sEmptyStack.ToList();

                    // Skip all nodes that are not additive at the start of the branch since they will never produce any geometry
                    var firstIndex = currentNode.childOffset;
                    var lastIndex = firstIndex + nodeCount;
                    for (; firstIndex < lastIndex && (topDownNodes[firstIndex].Operation != CSGOperationType.Additive &&
                                                      topDownNodes[firstIndex].Operation != CSGOperationType.Copy); firstIndex++)
                        firstIndex++;

                    if ((lastIndex - firstIndex) <= 0)
                        return;// sEmptyStack.ToList();

                    if ((lastIndex - firstIndex) == 1)
                    {
                        GetStack(in operationTables, ref topDownNodes, ref brushesTouchedByBrush, processedNodeIndex, ref topDownNodes[firstIndex], ref haveGonePastSelf, output);

                        // Node operation is always Additive at this point, and operation would be performed against .. nothing ..
                        // Anything added with nothing is itself, so we don't need to apply an operation here.

                        if (output.Length > 0)
                        {
                            var item = output[output.Length - 1];
                            item.operation = topDownNodes[firstIndex].Operation;
                            output[output.Length - 1] = item;
                        }

#if SHOW_DEBUG_MESSAGES
                        if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)
                            Dump(stack, 0, "stack return ");
#endif
                        return;
                    } else
                    {
                        var leftHaveGonePastSelf = 0;

                        var leftStack   = output;
                        var rightStack = new NativeList<CategoryStackNode>(Allocator.Temp); // TODO: get rid of allocation, store rightStack after leftStack and duplicate it -> then optimize
                        {
                            leftStack.Clear();
                            GetStack(in operationTables, ref topDownNodes, ref brushesTouchedByBrush, processedNodeIndex, ref topDownNodes[firstIndex], ref leftHaveGonePastSelf, leftStack);
                            haveGonePastSelf |= leftHaveGonePastSelf;
                            for (int i = firstIndex + 1; i < lastIndex; i++)
                            {
#if SHOW_DEBUG_MESSAGES
                                if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)                           
                                    Dump(leftStack, 0, $"before '{node[i - 1]}' {node[i].Operation} '{node[i]}'");
#endif
                                var rightHaveGonePastSelf = leftHaveGonePastSelf >= 1 ? 2 : 0;
                                rightStack.Clear();
                                GetStack(in operationTables, ref topDownNodes, ref brushesTouchedByBrush, processedNodeIndex, ref topDownNodes[i], ref rightHaveGonePastSelf, rightStack);
                                haveGonePastSelf |= rightHaveGonePastSelf;

                                Combine(in operationTables,
                                        ref topDownNodes,
                                        ref brushesTouchedByBrush,
                                        processedNodeIndex,
                                        leftStack,
                                        leftHaveGonePastSelf,
                                        rightStack,
                                        rightHaveGonePastSelf,
                                        topDownNodes[i].Operation
                                );
                                leftHaveGonePastSelf = rightHaveGonePastSelf;

                                //if (leftStack.Length > 0 && node.Operation == CSGOperationType.Copy)
                                //    leftStack[leftStack.Length - 1].operation = node.Operation;

#if SHOW_DEBUG_MESSAGES
                                if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)
                                    Dump(leftStack, 0, $"after '{node[i - 1]}' {node[i].Operation} '{node[i]}'");
#endif
                            }
                        }
                        rightStack.Dispose();
                        return;
                    }
                }
            }
        }

        static void Combine(in NativeArray<CategoryRoutingRow> operationTables, ref BlobArray<CompactTopDownNode> topDownNodes, ref BrushesTouchedByBrush brushesTouchedByBrush, int processedNodeIndex, NativeList<CategoryStackNode> leftStack, int leftHaveGonePastSelf, NativeList<CategoryStackNode> rightStack, int rightHaveGonePastSelf, CSGOperationType operation)
        {
            if (operation == CSGOperationType.Invalid)
                operation = CSGOperationType.Additive;

            if (leftStack.Length == 0) // left node has a branch without children or children are not intersecting with processedNode
            {
                if (rightStack.Length == 0) // right node has a branch without children or children are not intersecting with processedNode
                {
                    leftStack.Clear(); return;// sEmptyStack.ToList();
                }
                switch (operation)
                {
                    case CSGOperationType.Additive:
                    case CSGOperationType.Copy: leftStack.Clear(); leftStack.AddRange(rightStack); return; //rightStack;
                    default: leftStack.Clear(); return;// sEmptyStack.ToList();
                }
            } else
            if (rightStack.Length == 0) // right node has a branch without children or children are not intersecting with processedNode
            {
                switch (operation)
                {
                    case CSGOperationType.Additive:
                    case CSGOperationType.Copy:
                    case CSGOperationType.Subtractive: return; //leftStack
                    default: leftStack.Clear(); return;// sEmptyStack.ToList();
                }
            }

            int index       = 0;
            int vIndex      = 0;

            var firstNode   = rightStack[0].nodeIndex;

            var sCombineUsedIndices = new NativeArray<byte>(leftStack.Length + (CategoryRoutingRow.Length * rightStack.Length), Allocator.Temp);
            var sCombineIndexRemap  = new NativeArray<int>(leftStack.Length + (CategoryRoutingRow.Length * rightStack.Length), Allocator.Temp);
            var routingSteps        = new NativeList<int>(rightStack.Length, Allocator.Temp);
            {

                // Count the number of rows for unique node
                var rightNode    = firstNode;
                int counter = 0;
                for (int r = 0; r < rightStack.Length; r++)
                {
                    if (rightNode != rightStack[r].nodeIndex)
                    {
                        routingSteps.Add(counter);
                        counter = 0;
                        rightNode = rightStack[r].nodeIndex;
                    }
                    counter++;
                }
                routingSteps.Add(counter);


                int prevNodeIndex = leftStack.Length - 1;
                while (prevNodeIndex > 0)
                {
                    if (prevNodeIndex <= 0 ||
                        leftStack[prevNodeIndex - 1].nodeIndex != leftStack[prevNodeIndex].nodeIndex)
                        break;
                    prevNodeIndex--;
                }
                int startNodeIndex = leftStack.Length;

                for (int p = prevNodeIndex; p < startNodeIndex; p++)
                {
                    for (int t = 0; t < CategoryRoutingRow.Length; t++)
                        sCombineUsedIndices[(int)leftStack[p].routingRow[t]] = 1;
                }
                if (startNodeIndex == 0)
                {
                    sCombineUsedIndices[0] = 1;
                    sCombineUsedIndices[1] = 1;
                    sCombineUsedIndices[2] = 1;
                    sCombineUsedIndices[3] = 1;
                }

#if HAVE_SELF_CATEGORIES
                var operationTable  = Operation.Tables[(int)operation];
#else
                var operationTableOffset = (leftHaveGonePastSelf >= 1 && rightStack.Length == 1 ?
                                            OperationTables.RemoveOverlappingOffset : 0) +
                                            ((int)operation) * OperationTables.NumberOfRowsPerOperation;
                var operationTable = operationTables.GetSubArray(operationTableOffset, OperationTables.NumberOfRowsPerOperation);
                bool haveRemap = false;
#endif
                int stackIndex = 1;
                rightNode = firstNode;
                for (int r = 0; r < rightStack.Length; r++)
                {
                    if (rightNode != rightStack[r].nodeIndex)
                    {
#if USE_OPTIMIZATIONS
                        if (prevNodeIndex >= 0 && haveRemap && sCombineIndexRemap.Length > 0)
                        {
                            RemapIndices(leftStack, sCombineIndexRemap, prevNodeIndex, startNodeIndex);
#if SHOW_DEBUG_MESSAGES
                            if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)
                                Dump(sCombineChildren.ToArray(), 0);
#endif
                        }
#endif

                        prevNodeIndex = startNodeIndex;
                        startNodeIndex = leftStack.Length;
                        index = 0; vIndex = 0; stackIndex++;
                        rightNode = rightStack[r].nodeIndex;
                        //sCombineIndexRemap = null;
                        //sCombineIndexRemap.Clear();

                        sCombineUsedIndices.ClearValues();
                        for (int p = prevNodeIndex; p < startNodeIndex; p++)
                        {
                            for (int t = 0; t < CategoryRoutingRow.Length; t++)
                                sCombineUsedIndices[(int)leftStack[p].routingRow[t]] = 1;
                        }
                        sCombineIndexRemap.ClearValues();
                    }

                    var leftKeepContents = leftStack[prevNodeIndex].operation == CSGOperationType.Copy;
                    var rightKeepContents = rightStack[r].operation == CSGOperationType.Copy;// || operation == CSGOperationType.Copy;
                
                    CategoryRoutingRow routingRow;
                    int routingOffset = 0;
                    if (stackIndex >= routingSteps.Length) // last node in right stack
                    {
                        int ncount = 0;
                        var startR = r;

                        // Duplicate route multiple times, bake operation into table
                        for (int t = 0; t < CategoryRoutingRow.Length; t++)
                        {
                            for (r = startR; r < rightStack.Length; r++)
                            {
                                var rightInput = rightStack[r].routingRow;
                                if (rightKeepContents)
                                    rightInput[0] = rightInput[(int)CategoryIndex.Outside];
                                var leftIndex = (leftKeepContents && t == 0) ? CategoryIndex.Outside : (CategoryIndex)t;

                                // Fix up output of last node to include operation between
                                // last left and last right.
                                // We don't add a routingOffset here since this might be the last node & 
                                // we don't even know if there is a next node at this point.
                                routingRow = new CategoryRoutingRow(operationTable, leftIndex, rightInput); // applies operation

                                int foundIndex = -1;
#if USE_OPTIMIZATIONS
                                if (vIndex < sCombineUsedIndices.Length &&
                                    sCombineUsedIndices[vIndex] == 1)
#endif
                                {
#if USE_OPTIMIZATIONS
                                    for (int n = startNodeIndex; n < leftStack.Length; n++)
                                    {
                                        Debug.Assert(rightStack[r].nodeIndex == leftStack[n].nodeIndex);
                                        if (leftStack[n].routingRow.Equals(routingRow))
                                        {
                                            foundIndex = (int)leftStack[n].input;
                                            break;
                                        }
                                    }
#endif
                                    if (foundIndex == -1)
                                    {
                                        leftStack.Add(new CategoryStackNode()
                                        {
                                            nodeIndex   = rightStack[r].nodeIndex,
                                            operation   = rightStack[r].operation,
                                            input       = (CategoryGroupIndex)index,
                                            routingRow  = routingRow
                                        });
                                        foundIndex = index;
                                        index++;
                                    }
                                }

                                haveRemap = true;
                                sCombineIndexRemap[vIndex] = foundIndex + 1;
                                vIndex++;
                                ncount++;
                            }
                        }
#if SHOW_DEBUG_MESSAGES
                        /*
                        if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)
                        {
                            Debug.Log($"[{startR}/{rightStack.Length}] {ncount} {vIndex} / {sCombineIndexRemap.Count} {rightStack[startR].node} {rightStack.Length - startR} +");
                            Dump(sCombineChildren.ToArray(), 0);
                        }*/
#endif
                    } else
                    {
                        int ncount = 0;
                        var startR = r;
                        int routingStep = routingSteps[stackIndex];

                        // Duplicate route multiple times
                        for (int t = 0; t < CategoryRoutingRow.Length; t++, routingOffset += routingStep)
                        {
                            //for (r = startR; r < startR + routingSteps[nodeIndex - 1]; r++)
                            {
                                var rightInput = rightStack[r].routingRow;
                                if (rightKeepContents)
                                    rightInput[0] = rightInput[(int)CategoryIndex.Outside];

                                // Fix up routing to include offset b/c duplication
                                routingRow = rightInput + routingOffset;

                                int foundIndex = -1;
#if USE_OPTIMIZATIONS
                                if (vIndex < sCombineUsedIndices.Length &&
                                    sCombineUsedIndices[vIndex] == 1)
#endif
                                {
#if USE_OPTIMIZATIONS
                                    for (int n = startNodeIndex; n < leftStack.Length; n++)
                                    {
                                        Debug.Assert(rightStack[r].nodeIndex == leftStack[n].nodeIndex);
                                        if (leftStack[n].routingRow.Equals(routingRow))
                                        {
                                            foundIndex = (int)leftStack[n].input;
                                            break;
                                        }
                                    }
#endif
                                    if (foundIndex == -1)
                                    {
                                        leftStack.Add(new CategoryStackNode()
                                        {
                                            nodeIndex   = rightStack[r].nodeIndex,
                                            operation   = rightStack[r].operation,
                                            input       = (CategoryGroupIndex)index,
                                            routingRow  = routingRow
                                        });
                                        foundIndex = index;
                                        index++;
                                    }
                                }

                                haveRemap = true;
                                sCombineIndexRemap[vIndex] = foundIndex + 1;
                                vIndex++;
                                ncount++;
                            }
                        }
#if SHOW_DEBUG_MESSAGES
                        /*
                        if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)
                        {
                            Debug.Log($"[{r}/{rightStack.Length}] {ncount} {vIndex} / {sCombineIndexRemap.Count} {rightStack[r].node} -");
                            Dump(sCombineChildren.ToArray(), 0);
                        }*/
#endif
                    }
                }

#if USE_OPTIMIZATIONS
                if (//nodeIndex > 1 && 
                    prevNodeIndex >= 0 && haveRemap && sCombineIndexRemap.Length > 0)
                {
                    RemapIndices(leftStack, sCombineIndexRemap, prevNodeIndex, startNodeIndex);
#if SHOW_DEBUG_MESSAGES 
                    if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)
                        Dump(sCombineChildren.ToArray(), 0);
#endif
                    bool allEqual = true;
                    sCombineIndexRemap.ClearValues();
                    for (int i = startNodeIndex; i < leftStack.Length; i++)
                    {
                        if (!leftStack[i].routingRow.AreAllTheSame())
                        {
                            allEqual = false;
                            break;
                        }
                        sCombineIndexRemap[(int)leftStack[i].input] = ((int)leftStack[i].routingRow[0]) + 1;
                    }
                    if (allEqual)
                    {
                        leftStack.RemoveRange(startNodeIndex, leftStack.Length - startNodeIndex);
                        RemapIndices(leftStack, sCombineIndexRemap, prevNodeIndex, startNodeIndex);

#if SHOW_DEBUG_MESSAGES
                        if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)
                            Dump(sCombineChildren.ToArray(), 0);
#endif
                    }
                }

                // When all the paths for the first node lead to the same destination, just remove it
                int firstRemoveCount = 0;
                while (firstRemoveCount < leftStack.Length - 1 &&
                        leftStack[firstRemoveCount].nodeIndex != leftStack[firstRemoveCount + 1].nodeIndex &&
                        leftStack[firstRemoveCount].routingRow.AreAllValue(0))
                    firstRemoveCount++;
                if (firstRemoveCount > 0)
                    leftStack.RemoveRange(0, firstRemoveCount);
#endif

#if SHOW_DEBUG_MESSAGES
                if (processedNodeIndex + 1 == kDebugNode || kDebugNode == -1)
                    Dump(sCombineChildren.ToArray(), 0);
#endif
            }
            sCombineUsedIndices.Dispose();
            sCombineIndexRemap.Dispose();
        }

#if SHOW_DEBUG_MESSAGES
        static int kDebugNode = -1; 
        static void Dump(CSGTreeNode processedNode, CategoryStackNode[] stack, int depth = 0)
        {
            var space = new String(' ', depth);
            if (stack == null)
            {
                Debug.Log($"{space}processedNode: {processedNode} null");
                return;
            }
            if (stack.Length == 0)
            {
                Debug.Log($"{space}processedNode: {processedNode} stack.Length == 0");
                return;
            }
            var stringBuilder = new System.Text.StringBuilder();
            for (int i = 0; i < stack.Length; i++)
                stringBuilder.AppendLine($"[{i}]: {stack[i]}");
            Debug.LogWarning($"{space}processedNode: {processedNode}\n{stringBuilder.ToString()}");
        }

        static void Dump(CSGTreeNode processedNode, List<CategoryStackNode> stack, int depth = 0)
        {
            var space = new String(' ', depth);
            if (stack == null)
            {
                Debug.Log($"{space}processedNode: {processedNode} null");
                return;
            }
            if (stack.Count == 0)
            {
                Debug.Log($"{space}processedNode: {processedNode} stack.Count == 0");
                return;
            }
            var stringBuilder = new System.Text.StringBuilder();
            for (int i = 0; i < stack.Count; i++)
                stringBuilder.AppendLine($"[{i}]: {stack[i]}");
            Debug.LogWarning($"{space}processedNode: {processedNode}\n{stringBuilder.ToString()}");
        }

        static void Dump(CategoryStackNode[] stack, int depth = 0, string extra = "")
        {
            var space = new String(' ', depth);
            if (stack == null)
            {
                Debug.Log($"{space}---- {extra}null");
                return;
            }
            if (stack.Length == 0)
            {
                Debug.Log($"{space}---- {extra}stack.Length == 0");
                return;
            }
            var stringBuilder = new System.Text.StringBuilder();
            for (int i = 0; i < stack.Length; i++)
                stringBuilder.AppendLine($"[{i}]: {stack[i]}");
            Debug.Log($"{space}---- {extra}\n{stringBuilder.ToString()}");
        }
#endif
    }
#endif
}
