#define USE_OPTIMIZATIONS
//#define SHOW_DEBUG_MESSAGES
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Chisel.Core 
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    internal unsafe struct CategoryStackNode
    { 
        public CSGTreeNode          node;
        public CSGOperationType     operation;
        public CategoryGroupIndex   input;
        public CategoryRoutingRow   routingRow;

        public override string ToString() { return $"'{node}': {(CategoryIndex)input} -> {routingRow}"; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void SetUsedNodesBits(CSGTreeBrush brush, BrushIntersectionLookup bitset)
        {
            var brushNodeIndex = brush.NodeID - 1;

            bitset.Clear();
            bitset.Set(brushNodeIndex, IntersectionType.Intersection);

            var parent = brush.Parent;
            while (parent.Valid)
            {
                var parentIndex = parent.NodeID - 1;
                bitset.Set(parentIndex, IntersectionType.Intersection);
                parent = parent.Parent;
            }

            var touchingBrushes = CSGManager.GetTouchingBrushes(brush);
            foreach (var touch in touchingBrushes)
            {
                var touchingBrush = touch.Key;
                if (!touchingBrush.Valid)
                    continue;

                var touchingType = touch.Value;
                var touchingBrushIndex = touchingBrush.NodeID - 1;
                bitset.Set(touchingBrushIndex, touchingType);

                parent = touchingBrush.Parent;
                while (parent.Valid)
                {
                    var parentIndex = parent.NodeID - 1;
                    bitset.Set(parentIndex, IntersectionType.Intersection);
                    parent = parent.Parent;
                }
            }
        }


        static List<CategoryStackNode>      sRoutingTable       = new List<CategoryStackNode>();
        static List<RoutingLookup>          sRoutingLookups     = new List<RoutingLookup>();
        static List<CategoryGroupIndex>     sInputs             = new List<CategoryGroupIndex>();
        static List<CategoryRoutingRow>     sRoutingRows        = new List<CategoryRoutingRow>();
        static List<Loop[]>                 sIntersectionLoops  = new List<Loop[]>();

        public static void GenerateCategorizationTable(CSGTreeNode  rootNode,
                                                       CSGTreeBrush processedNode,
                                                       BrushLoops   brushOutputLoops,
                                                       RoutingTable routingTable)
        {
            routingTable.Clear();

            var processed_node_index = processedNode.NodeID;
            if (!CSGManager.IsValidNodeID(processed_node_index))
            {
                return;
            }

            sRoutingTable.Clear();

            var intersectionTypeLookup = new BrushIntersectionLookup(CSGManager.GetMaxNodeIndex());
            SetUsedNodesBits(processedNode, intersectionTypeLookup);
            var rootNodeIndex = rootNode.NodeID - 1;
            intersectionTypeLookup.Set(rootNodeIndex, IntersectionType.Intersection);

            int categoryStackNodeCount, polygonGroupCount;

            {
                var stack = GetStack(intersectionTypeLookup, processedNode, rootNode);
                
                // TODO: remove the need for this
                {
                    int lastNodeIndex = stack.Length - 1;
                    while (lastNodeIndex > 0)
                    {
                        if (lastNodeIndex <= 0 ||
                            stack[lastNodeIndex - 1].node != stack[lastNodeIndex].node)
                            break;
                        lastNodeIndex--;
                    }

                    for (int n = lastNodeIndex; n < stack.Length; n++)
                    {
                        var stackNode = stack[n];
                        var routingRow = stackNode.routingRow;
                        for (int r = 0; r < CategoryRoutingRow.Length; r++)
                            routingRow[r]++;
                        stackNode.routingRow = routingRow;
                        stack[n] = stackNode;
                    }
                }

#if SHOW_DEBUG_MESSAGES
                if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                    Dump(processedNode, stack);
#endif
                sRoutingTable.AddRange(stack);
            }

            categoryStackNodeCount = (int)sRoutingTable.Count;

            int maxCounter = (int)CategoryRoutingRow.Length;
            for (int i = 0; i < categoryStackNodeCount; i++)
                maxCounter = Math.Max(maxCounter, (int)sRoutingTable[i].input);
            polygonGroupCount = maxCounter + 1;

            sRoutingLookups.Clear();
            sInputs.Clear();
            sRoutingRows.Clear();
            sIntersectionLoops.Clear();
            // TODO: clean up
            for (int i = 0; i < sRoutingTable.Count;)
            {
                var cutting_node = sRoutingTable[i].node;
                var cutting_node_id = cutting_node.nodeID;
                Debug.Assert(cutting_node.Valid);
                
                int start_index = i;
                do
                {
                    sInputs.Add(sRoutingTable[i].input);
                    sRoutingRows.Add(sRoutingTable[i].routingRow);
                    i++;
                } while (i < sRoutingTable.Count && sRoutingTable[i].node.nodeID == cutting_node_id);
                int end_index = i;


                // Get the intersection loops between the two brushes on every surface of the brush we're performing CSG on
                if (!brushOutputLoops.intersectionLoops.TryGetValue(cutting_node_id, out Loop[] cuttingNodeIntersectionLoops))
                {
                    if (sRoutingTable.Count > 1)
                    {
                        Debug.Assert(false, $"Brush intersecting with brush has no intersecting surface?  {cutting_node.nodeID} / {processedNode.NodeID}");
                        //Dump(processedNode, sRoutingTable.ToArray());
                    }
                    cuttingNodeIntersectionLoops = null;
                }

                sIntersectionLoops.Add(cuttingNodeIntersectionLoops);

                sRoutingLookups.Add(new RoutingLookup(start_index, end_index));
            }

            routingTable.inputs = sInputs.ToArray();
            routingTable.routingRows = sRoutingRows.ToArray();
            routingTable.routingLookups = sRoutingLookups.ToArray();
            routingTable.intersectionLoops = sIntersectionLoops.ToArray();
        }



        #region Operation tables
        static class Operation
        {
#if HAVE_SELF_CATEGORIES
            const CategoryGroupIndex Inside             = (CategoryGroupIndex)CategoryIndex.Inside;
            const CategoryGroupIndex Aligned            = (CategoryGroupIndex)CategoryIndex.Aligned;
            const CategoryGroupIndex SelfAligned        = (CategoryGroupIndex)CategoryIndex.SelfAligned;
            const CategoryGroupIndex SelfReverseAligned = (CategoryGroupIndex)CategoryIndex.SelfReverseAligned;
            const CategoryGroupIndex ReverseAligned     = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
            const CategoryGroupIndex Outside            = (CategoryGroupIndex)CategoryIndex.Outside;

            public static readonly CategoryRoutingRow[][] Tables = new[]
            {
                // Additive set operation on polygons: output = (left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Additive Operation
                {
	                //             	        right node                                                                                                              |
	                //                                                              self                  self                                                      |
	                //                      inside                aligned           aligned               reverse-aligned       reverse-aligned   outside           |     left-node       
	                //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,               Inside,           Inside,               Inside,               Inside,           Inside            ), // inside
	                new CategoryRoutingRow( Inside,               Aligned,          SelfAligned,          Inside,               Inside,           Aligned           ), // other-aligned
	                new CategoryRoutingRow( Inside,               Aligned,          SelfAligned,          Inside,               Inside,           SelfAligned       ), // self-aligned
	                new CategoryRoutingRow( Inside,               Inside,           Inside,               SelfReverseAligned,   ReverseAligned,   SelfReverseAligned), // self-reverse-aligned
	                new CategoryRoutingRow( Inside,               Inside,           Inside,               SelfReverseAligned,   ReverseAligned,   ReverseAligned    ), // other-reverse-aligned
	                new CategoryRoutingRow( Inside,               Aligned,          SelfAligned,          SelfReverseAligned,   ReverseAligned,   Outside           )  // outside
                },

                // Subtractive set operation on polygons: output = !(!left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Subtractive Operation
                {
	                //             	        right node                                                                                                              |
	                //                                                              self                  self                                                      |
	                //                      inside                aligned           aligned               reverse-aligned       reverse-aligned   outside           |     left-node       
	                //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Outside,              ReverseAligned,   SelfReverseAligned,   SelfAligned,          Aligned,          Inside            ), // inside
	                new CategoryRoutingRow( Outside,              Aligned,          Inside,               SelfAligned,          Aligned,          Aligned           ), // other-aligned
	                new CategoryRoutingRow( Outside,              Aligned,          Inside,               SelfAligned,          Aligned,          SelfAligned       ), // self-aligned
	                new CategoryRoutingRow( Outside,              ReverseAligned,   SelfReverseAligned,   Outside,              Outside,          SelfReverseAligned), // self-reverse-aligned
	                new CategoryRoutingRow( Outside,              ReverseAligned,   SelfReverseAligned,   Outside,              Outside,          ReverseAligned    ), // other-reverse-aligned
	                new CategoryRoutingRow( Outside,              Outside,          Outside,              Outside,              Outside,          Outside           )  // outside
                },

                // Common set operation on polygons: output = !(!left-node || !right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Intersection Operation
                {
	                //             	        right node                                                                                                              |
	                //                                                              self                  self                                                      |
	                //                      inside                aligned           aligned               reverse-aligned       reverse-aligned   outside           |     left-node       
	                //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,               Aligned,          SelfAligned,          SelfReverseAligned,   ReverseAligned,   Outside           ), // inside
	                new CategoryRoutingRow( Aligned,              Aligned,          SelfAligned,          Outside,              Outside,          Outside           ), // other-aligned
	                new CategoryRoutingRow( SelfAligned,          Aligned,          SelfAligned,          Outside,              Outside,          Outside           ), // self-aligned
	                new CategoryRoutingRow( SelfReverseAligned,   Outside,          Outside,              SelfReverseAligned,   ReverseAligned,   Outside           ), // self-reverse-aligned
	                new CategoryRoutingRow( ReverseAligned,       Outside,          Outside,              SelfReverseAligned,   ReverseAligned,   Outside           ), // other-reverse-aligned
	                new CategoryRoutingRow( Outside,              Outside,          Outside,              Outside,              Outside,          Outside           )  // outside
                },

                // Additive set operation on polygons: output = (left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // AdditiveKeepInside Operation
                {
	                //             	        right node                                                                                                              |
	                //                                                              self                  self                                                      |
	                //                      inside                aligned           aligned               reverse-aligned       reverse-aligned   outside           |     left-node       
	                //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,               Inside,           Inside,               Inside,               Inside,           Inside            ), // inside
	                new CategoryRoutingRow( Inside,               Aligned,          SelfAligned,          Inside,               Inside,           Aligned           ), // other-aligned
	                new CategoryRoutingRow( Inside,               Aligned,          SelfAligned,          Inside,               Inside,           SelfAligned       ), // self-aligned
	                new CategoryRoutingRow( Inside,               Inside,           Inside,               SelfReverseAligned,   ReverseAligned,   SelfReverseAligned), // self-reverse-aligned
	                new CategoryRoutingRow( Inside,               Inside,           Inside,               SelfReverseAligned,   ReverseAligned,   ReverseAligned    ), // other-reverse-aligned
	                new CategoryRoutingRow( Inside,               Aligned,          SelfAligned,          SelfReverseAligned,   ReverseAligned,   Outside           )  // outside
                }
            };
#else
            const CategoryGroupIndex Inside             = (CategoryGroupIndex)CategoryIndex.Inside;
            const CategoryGroupIndex Aligned            = (CategoryGroupIndex)CategoryIndex.Aligned;
            const CategoryGroupIndex ReverseAligned     = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
            const CategoryGroupIndex Outside            = (CategoryGroupIndex)CategoryIndex.Outside;

            public static readonly CategoryRoutingRow[][] RemoveOverlappingOperationTables = new[]
            {
                // Additive set operation on polygons: output = (left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Additive Operation
                {
	                //             	        right node                                                              |
	                //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	                //----------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,           Inside,           Inside,           Inside            ), // inside
	                new CategoryRoutingRow( Inside,           Outside,          Outside,          Inside            ), // aligned
	                new CategoryRoutingRow( Inside,           Outside,          Outside,          Inside            ), // reverse-aligned
	                new CategoryRoutingRow( Inside,           Inside,           Inside,           Outside           )  // outside
                },

                // Subtractive set operation on polygons: output = !(!left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Subtractive Operation
                {
	                //             	        right node                                                              |
	                //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	                //----------------------------------------------------------------------------------------------------------------------------
                    new CategoryRoutingRow( Outside,          Inside,            Inside,          Inside            ), // inside
	                new CategoryRoutingRow( Outside,          Outside,           Outside,         Inside            ), // aligned
	                new CategoryRoutingRow( Outside,          Outside,           Outside,         Inside            ), // reverse-aligned
	                new CategoryRoutingRow( Outside,          Outside,           Outside,         Outside           )  // outside
                }, 

                // Common set operation on polygons: output = !(!left-node || !right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Intersection Operation
                {
	                //             	        right node                                                              |
	                //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	                //----------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,           Outside,          Outside,          Outside           ), // inside
	                new CategoryRoutingRow( Inside,           Outside,          Outside,          Outside           ), // aligned
	                new CategoryRoutingRow( Inside,           Outside,          Outside,          Outside           ), // reverse-aligned
	                new CategoryRoutingRow( Outside,          Outside,          Outside,          Outside           )  // outside
                },

                // Additive set operation on polygons: output = (left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // AdditiveKeepInside Operation
                {
	                //             	        right node                                                              |
	                //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	                //----------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,           Inside,           Inside,           Inside            ), // inside
	                new CategoryRoutingRow( Inside,           Outside,          Outside,          Inside            ), // aligned
	                new CategoryRoutingRow( Inside,           Outside,          Outside,          Inside            ), // reverse-aligned
	                new CategoryRoutingRow( Inside,           Inside,           Inside,           Outside           )  // outside
                }
            };
            
            public static readonly CategoryRoutingRow[][] RegularOperationTables = new[]
            {
                // Additive set operation on polygons: output = (left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Additive Operation
                {
	                //             	        right node                                                              |
	                //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	                //----------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,           Inside,           Inside,           Inside            ), // inside
	                new CategoryRoutingRow( Inside,           Aligned,          Inside,           Aligned           ), // aligned
	                new CategoryRoutingRow( Inside,           Inside,           ReverseAligned,   ReverseAligned    ), // reverse-aligned
	                new CategoryRoutingRow( Inside,           Aligned,          ReverseAligned,   Outside           )  // outside
                },

                // Subtractive set operation on polygons: output = !(!left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Subtractive Operation
                {
	                //             	        right node                                                              |
	                //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	                //----------------------------------------------------------------------------------------------------------------------------
                    new CategoryRoutingRow( Outside,          ReverseAligned,   Aligned,          Inside            ), // inside
	                new CategoryRoutingRow( Outside,          Inside,           Aligned,          Aligned           ), // aligned
	                new CategoryRoutingRow( Outside,          ReverseAligned,   Inside,           ReverseAligned    ), // reverse-aligned
	                new CategoryRoutingRow( Outside,          Outside,          Outside,          Outside           )  // outside
                },

                // Common set operation on polygons: output = !(!left-node || !right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // Intersection Operation
                {
	                //             	        right node                                                              |
	                //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	                //----------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,           Aligned,          ReverseAligned,   Outside           ), // inside
	                new CategoryRoutingRow( Aligned,          Aligned,          Outside,          Outside           ), // aligned
	                new CategoryRoutingRow( ReverseAligned,   Outside,          ReverseAligned,   Outside           ), // reverse-aligned
	                new CategoryRoutingRow( Outside,          Outside,          Outside,          Outside           )  // outside
                },

                // Additive set operation on polygons: output = (left-node || right-node)
                // Defines final output from combination of categorization of left and right node
                new CategoryRoutingRow[] // AdditiveKeepInside Operation
                {
	                //             	        right node                                                              |
	                //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	                //----------------------------------------------------------------------------------------------------------------------------
	                new CategoryRoutingRow( Inside,           Inside,           Inside,           Inside            ), // inside
	                new CategoryRoutingRow( Inside,           Aligned,          Inside,           Aligned           ), // aligned
	                new CategoryRoutingRow( Inside,           Inside,           ReverseAligned,   ReverseAligned    ), // reverse-aligned
	                new CategoryRoutingRow( Inside,           Aligned,          ReverseAligned,   Outside           )  // outside
                }
            };
#endif
        }
        #endregion


        static readonly CategoryStackNode[] sEmptyStack = new CategoryStackNode[] { };

        // Remap indices to new destinations, used when destination rows have been merged
        static void RemapIndices(List<CategoryStackNode> stack, Dictionary<int, int> remap, int start, int last)
        {
            for (int i = start; i < last; i++)
            {
                var categoryRow = stack[i];
                var routingRow = categoryRow.routingRow;

                for (int r = 0; r < CategoryRoutingRow.Length; r++)
                {
                    if (!remap.ContainsKey((int)routingRow[r])) { Debug.Assert(false); return; }
                }

                for (int r = 0; r < CategoryRoutingRow.Length; r++)
                    routingRow[r] = (CategoryGroupIndex)remap[(int)routingRow[r]];

                categoryRow.routingRow = routingRow;
                stack[i] = categoryRow;
            }
        }

        static readonly List<CategoryStackNode> sCombineChildren    = new List<CategoryStackNode>();
        static readonly HashSet<int>            sCombineUsedIndices = new HashSet<int>();
        static readonly Dictionary<int, int>    sCombineIndexRemap  = new Dictionary<int, int>();

        static CategoryStackNode[] Combine(BrushIntersectionLookup treeIntersectionTypeLookup, CSGTreeBrush processedNode, CategoryStackNode[] leftStack, CategoryStackNode[] rightStack, CSGOperationType operation, int depth, int haveGonePastSelf, bool lastNode)
        {
            if (operation == CSGOperationType.Invalid)
                operation = CSGOperationType.Additive;

            if (leftStack.Length == 0) // left node has a branch without children or children are not intersecting with processedNode
            {
                if (rightStack.Length == 0) // right node has a branch without children or children are not intersecting with processedNode
                    return sEmptyStack;
                switch (operation)
                {
                    case CSGOperationType.Additive:
                    case CSGOperationType.Copy: return rightStack;
                    default: return sEmptyStack;
                }
            } else
            if (rightStack.Length == 0) // right node has a branch without children or children are not intersecting with processedNode
            {
                switch (operation)
                {
                    case CSGOperationType.Additive:
                    case CSGOperationType.Copy:
                    case CSGOperationType.Subtractive: return leftStack;
                    default: return sEmptyStack;
                }
            }

#if HAVE_SELF_CATEGORIES
            var operationTable  = Operation.Tables[(int)operation];
#else
            var operationTable  = haveGonePastSelf != 1 ? Operation.RemoveOverlappingOperationTables[(int)operation] : Operation.RegularOperationTables[(int)operation];
#endif
            int index           = 0;
            int vIndex          = 0;
            
            var firstNode   = rightStack[0].node;


            // Count the number of rows for unique node
            var rightNode    = firstNode;
            var routingSteps = new List<int>();
            int counter = 0;
            for (int r = 0; r < rightStack.Length; r++)
            {
                if (rightNode != rightStack[r].node)
                {
                    routingSteps.Add(counter);
                    counter = 0;
                    rightNode = rightStack[r].node;
                }
                counter++;
            }
            routingSteps.Add(counter);


            sCombineChildren.Clear();
            sCombineChildren.AddRange(leftStack);

            int prevNodeIndex = sCombineChildren.Count - 1;
            while (prevNodeIndex > 0)
            {
                if (prevNodeIndex <= 0 ||
                    sCombineChildren[prevNodeIndex - 1].node != sCombineChildren[prevNodeIndex].node)
                    break;
                prevNodeIndex--;
            }
            int startNodeIndex = sCombineChildren.Count;

            sCombineUsedIndices.Clear();
            for (int p = prevNodeIndex; p < startNodeIndex; p++)
            {
                for (int t = 0; t < CategoryRoutingRow.Length; t++)
                    sCombineUsedIndices.Add((int)sCombineChildren[p].routingRow[t]);
            }
            if (startNodeIndex == 0)
            {
                sCombineUsedIndices.Add(0);
                sCombineUsedIndices.Add(1);
                sCombineUsedIndices.Add(2);
                sCombineUsedIndices.Add(3);
            }

            sCombineIndexRemap.Clear();

            int nodeIndex = 1;
            rightNode = firstNode;
            for (int r = 0; r < rightStack.Length; r++)
            {
                if (rightNode != rightStack[r].node)
                {
#if USE_OPTIMIZATIONS
                    if (prevNodeIndex >= 0 && sCombineIndexRemap.Count > 0)
                    {
                        RemapIndices(sCombineChildren, sCombineIndexRemap, prevNodeIndex, startNodeIndex);
#if SHOW_DEBUG_MESSAGES
                        if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                            Dump(sCombineChildren.ToArray(), depth);
#endif
                    }
#endif

                    prevNodeIndex = startNodeIndex;
                    startNodeIndex = sCombineChildren.Count;
                    index = 0; vIndex = 0; nodeIndex++;
                    rightNode = rightStack[r].node;
                    sCombineIndexRemap.Clear();

                    sCombineUsedIndices.Clear();
                    for (int p = prevNodeIndex; p < startNodeIndex; p++)
                    {
                        for (int t = 0; t < CategoryRoutingRow.Length; t++)
                            sCombineUsedIndices.Add((int)sCombineChildren[p].routingRow[t]);
                    }
                }

                var leftKeepContents = sCombineChildren[prevNodeIndex].operation == CSGOperationType.Copy;
                var rightKeepContents = rightNode.Operation == CSGOperationType.Copy;// || operation == CSGOperationType.Copy;
                //Debug.Log($"{processedNode.NodeID}: {sCombineChildren[prevNodeIndex].node.Type}:{sCombineChildren[prevNodeIndex].node.nodeID}:{sCombineChildren[prevNodeIndex].operation} <-> {rightNode.Type}:{rightNode.nodeID}:{rightNode.Operation} | {leftKeepContents}/{rightKeepContents}");
                
                CategoryRoutingRow routingRow;
                int routingOffset = 0;
                if (nodeIndex >= routingSteps.Count) // last node in right stack
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

#if USE_OPTIMIZATIONS
                            /*
                            if (lastNode)
                            {
                                for (int i = 0; i < CategoryRoutingRow.Length; i++)
                                {
                                    var output = routingRow[i];
                                    if (output != (CategoryGroupIndex)CategoryIndex.ValidAligned &&
                                        output != (CategoryGroupIndex)CategoryIndex.ValidReverseAligned)
                                        routingRow[i] = CategoryGroupIndex.First;
                                }
                            }*/
#endif

                            int foundIndex = -1;
#if USE_OPTIMIZATIONS
                            if (sCombineUsedIndices.Contains(vIndex))
#endif
                            {
#if USE_OPTIMIZATIONS
                                for (int n = startNodeIndex; n < sCombineChildren.Count; n++)
                                {
                                    Debug.Assert(rightStack[r].node == sCombineChildren[n].node);
                                    if (sCombineChildren[n].routingRow.Equals(routingRow))
                                    {
                                        foundIndex = (int)sCombineChildren[n].input;
                                        break;
                                    }
                                }
#endif
                                if (foundIndex == -1)
                                {
                                    sCombineChildren.Add(new CategoryStackNode()
                                    {
                                        node        = rightStack[r].node,
                                        operation   = rightStack[r].node.Operation,
                                        input       = (CategoryGroupIndex)index,
                                        routingRow  = routingRow
                                    });
                                    foundIndex = index;
                                    index++;
                                }
                            }

                            sCombineIndexRemap[vIndex] = foundIndex;
                            vIndex++;
                            ncount++;
                        }
                    }
#if SHOW_DEBUG_MESSAGES
                    /*
                    if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                    {
                        Debug.Log($"[{startR}/{rightStack.Length}] {ncount} {vIndex} / {sCombineIndexRemap.Count} {rightStack[startR].node} {rightStack.Length - startR} +");
                        Dump(sCombineChildren.ToArray(), depth);
                    }*/
#endif
                } else
                {
                    int ncount = 0;
                    var startR = r;
                    int routingStep = routingSteps[nodeIndex];

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
                            if (sCombineUsedIndices.Contains(vIndex))
#endif
                            {
#if USE_OPTIMIZATIONS
                                for (int n = startNodeIndex; n < sCombineChildren.Count; n++)
                                {
                                    Debug.Assert(rightStack[r].node == sCombineChildren[n].node);
                                    if (sCombineChildren[n].routingRow.Equals(routingRow))
                                    {
                                        foundIndex = (int)sCombineChildren[n].input;
                                        break;
                                    }
                                }
#endif
                                if (foundIndex == -1)
                                {
                                    sCombineChildren.Add(new CategoryStackNode()
                                    {
                                        node        = rightStack[r].node,
                                        operation   = rightStack[r].node.Operation,
                                        input       = (CategoryGroupIndex)index,
                                        routingRow  = routingRow
                                    });
                                    foundIndex = index;
                                    index++;
                                }
                            }

                            sCombineIndexRemap[vIndex] = foundIndex;
                            vIndex++;
                            ncount++;
                        }
                    }
#if SHOW_DEBUG_MESSAGES
                    /*
                    if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                    {
                        Debug.Log($"[{r}/{rightStack.Length}] {ncount} {vIndex} / {sCombineIndexRemap.Count} {rightStack[r].node} -");
                        Dump(sCombineChildren.ToArray(), depth);
                    }*/
#endif
                }
            }

#if USE_OPTIMIZATIONS
            if (//nodeIndex > 1 && 
                prevNodeIndex >= 0 && sCombineIndexRemap.Count > 0)
            {
                RemapIndices(sCombineChildren, sCombineIndexRemap, prevNodeIndex, startNodeIndex);
#if SHOW_DEBUG_MESSAGES
                if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                    Dump(sCombineChildren.ToArray(), depth);
#endif
                bool allEqual = true;
                sCombineIndexRemap.Clear();
                for (int i = startNodeIndex; i < sCombineChildren.Count; i++)
                {
                    if (!sCombineChildren[i].routingRow.AreAllTheSame())
                    {
                        allEqual = false;
                        break;
                    }
                    sCombineIndexRemap[(int)sCombineChildren[i].input] = (int)sCombineChildren[i].routingRow[0];
                }
                if (allEqual)
                {
                    sCombineChildren.RemoveRange(startNodeIndex, sCombineChildren.Count - startNodeIndex);
                    //Debug.Log($"[-/{rightStack.Length}] remove last");
                    RemapIndices(sCombineChildren, sCombineIndexRemap, prevNodeIndex, startNodeIndex);

#if SHOW_DEBUG_MESSAGES
                    if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                        Dump(sCombineChildren.ToArray(), depth);
#endif
                }
            }

            // When all the paths for the first node lead to the same destination, just remove it
            while (sCombineChildren.Count > 1 && 
                    sCombineChildren[0].node != sCombineChildren[1].node &&
                    sCombineChildren[0].routingRow.AreAllValue(0))
                sCombineChildren.RemoveAt(0);
#endif

#if SHOW_DEBUG_MESSAGES
            if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                Dump(sCombineChildren.ToArray(), depth);
#endif

            if (sCombineChildren.Count == 0)
                return sEmptyStack;

            return sCombineChildren.ToArray();
        }

        static CategoryStackNode[] GetStack(BrushIntersectionLookup intersectionTypeLookup, CSGTreeBrush processedNode, CSGTreeNode rootNode)
        {
            int haveGonePastSelf = 0;
            var stack = GetStack(intersectionTypeLookup, processedNode, rootNode, rootNode, 0, ref haveGonePastSelf);
            if (stack == null ||
                stack.Length == 0)
                return new CategoryStackNode[] { new CategoryStackNode() { node = rootNode, operation = rootNode.Operation, routingRow = CategoryRoutingRow.outside } };
            return stack;
        }

        static CategoryStackNode[] GetStack(BrushIntersectionLookup intersectionTypeLookup, CSGTreeBrush processedNode, CSGTreeNode node, CSGTreeNode rootNode, int depth, ref int haveGonePastSelf)
        {
            Debug.Assert(processedNode.Valid);
            Debug.Assert(node.Valid);

            var nodeIndex           = node.nodeID - 1;
            var intersectionType    = intersectionTypeLookup.Get(nodeIndex);
            if (intersectionType == IntersectionType.NoIntersection)
                return sEmptyStack;// new CategoryStackNode[] { new CategoryStackNode() { node = node, operation = node.Operation, routingRow = CategoryRoutingRow.outside } };


            //Debug.Log($"{intersectionType} {node} {processedNode}");

            // TODO: use other intersection types
            if (intersectionType == IntersectionType.AInsideB) return new CategoryStackNode[] { new CategoryStackNode() { node = node, operation = node.Operation, routingRow = CategoryRoutingRow.inside } };
            if (intersectionType == IntersectionType.BInsideA) return new CategoryStackNode[] { new CategoryStackNode() { node = node, operation = node.Operation, routingRow = CategoryRoutingRow.outside } };

            switch (node.Type)
            {
                case CSGNodeType.Brush:
                {
                    // All surfaces of processedNode are aligned with it's own surfaces, so all categories are Aligned
                    if (processedNode == node)
                    {
                        haveGonePastSelf = 1;
                        //Debug.Log($"{processedNode.NodeID}: BRUSH {node.Type}:{node.nodeID}:{node.Operation} {haveGonePastSelf}");
                        return new CategoryStackNode[] { new CategoryStackNode() { node = node, operation = node.Operation, routingRow = CategoryRoutingRow.selfAligned } };
                    }

                    if (haveGonePastSelf > 0)
                        haveGonePastSelf = 2;

                    // Otherwise return identity categories (input == output)
                    return new CategoryStackNode[] { new CategoryStackNode() { node = node, operation = node.Operation, routingRow = CategoryRoutingRow.identity } };
                }
                default:
                {
                    var nodeCount = node.Count;
                    if (nodeCount == 0)
                        return sEmptyStack;// new CategoryStackNode[] { new CategoryStackNode() { node = node, operation = node.Operation, routingRow = CategoryRoutingRow.outside } };

                    // Skip all nodes that are not additive at the start of the branch since they will never produce any geometry
                    int firstIndex = 0;
                    for (; firstIndex < nodeCount && node[firstIndex].Valid && (node[firstIndex].Operation != CSGOperationType.Additive && 
                                                                                node[firstIndex].Operation != CSGOperationType.Copy); firstIndex++)
                        firstIndex++;

                    if ((nodeCount - firstIndex) <= 0)
                        return sEmptyStack;// new CategoryStackNode[] { new CategoryStackNode() { node = node, operation = node.Operation, routingRow = CategoryRoutingRow.outside } };

                    if ((nodeCount - firstIndex) == 1)
                    {
                        var stack = GetStack(intersectionTypeLookup, processedNode, node[firstIndex], rootNode, depth + 1, ref haveGonePastSelf);

                        // Node operation is always Additive at this point, and operation would be performed against .. nothing ..
                        // Anything added with nothing is itself, so we don't need to apply an operation here.

                        //Debug.Log($"{processedNode.NodeID}: STACK {node[firstIndex].Type}:{node[firstIndex].nodeID}:{node[firstIndex].Operation} {haveGonePastSelf}");
                        if (stack.Length > 0)
                            stack[stack.Length - 1].operation = node[firstIndex].Operation;

#if SHOW_DEBUG_MESSAGES
                        if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                            Dump(stack, depth, "stack return ");
#endif
                        return stack;
                    } else
                    {
                        //Debug.Log($"{processedNode.NodeID}: LEFT-STACK");
                        var originalHaveGonePastSelf = haveGonePastSelf;
                        var leftHaveGonePastSelf = originalHaveGonePastSelf;
                        var leftStack = GetStack(intersectionTypeLookup, processedNode, node[firstIndex], rootNode, depth + 1, ref leftHaveGonePastSelf);
                        haveGonePastSelf |= leftHaveGonePastSelf;
                        for (int i = firstIndex + 1; i < nodeCount; i++)
                        {
                            if (!node[i].Valid)
                                continue;
#if SHOW_DEBUG_MESSAGES
                            if (processedNode.NodeID == kDebugNode || kDebugNode == -1)                           
                                Dump(leftStack, depth, $"before '{node[i - 1]}' {node[i].Operation} '{node[i]}'");
#endif
                            //Debug.Log($"{processedNode.NodeID}: RIGHT-STACK");
                            var rightHaveGonePastSelf = originalHaveGonePastSelf;
                            var rightStack = GetStack(intersectionTypeLookup, processedNode, node[i], rootNode, depth + 1, ref rightHaveGonePastSelf);
                            haveGonePastSelf |= rightHaveGonePastSelf;

                            leftStack =
                                Combine(
                                        intersectionTypeLookup, processedNode,
                                        leftStack, rightStack,
                                        node[i].Operation,
                                        depth + 1,
                                        haveGonePastSelf,
                                        node == rootNode
                                );

                            //Debug.Log($"{processedNode.NodeID}: OPERATION {node[i].Type}:{node[i].nodeID}:{node[i].Operation} {haveGonePastSelf}");

                            //if (leftStack.Length > 0 && node.Operation == CSGOperationType.Copy)
                            //    leftStack[leftStack.Length - 1].operation = node.Operation;

#if SHOW_DEBUG_MESSAGES
                            if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                                Dump(leftStack, depth, $"after '{node[i - 1]}' {node[i].Operation} '{node[i]}'");
#endif
                        }
                        return leftStack;
                    }
                }
            }
        }

#if SHOW_DEBUG_MESSAGES
        static int kDebugNode = 10; 
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