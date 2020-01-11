#define USE_OPTIMIZATIONS
#define USE_HALF_OPERATION_TABLE
#define SHOW_DEBUG_MESSAGES
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
    // TODO: store the routingRows per node, since inputs are in order & increase by 1, we can remove it and look them up directly
    internal unsafe partial struct CategoryStackNode
    { 
        public CSGTreeNode node;
        public CategoryGroupIndex input;
        public CategoryRoutingRow routingRow;

        public override string ToString() { return $"'{node}': {input} -> {routingRow}"; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void SetInvalid() { routingRow = CategoryRoutingRow.invalid; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void SetUsedNodesBits(CSGTreeBrush brush, BrushIntersectionLookup bitset)
        {
            var brushNodeIndex = brush.NodeID - 1;

            bitset.Clear();
            //Debug.Log("<<");
            bitset.Set(brushNodeIndex, IntersectionType.Intersection);
            //Debug.Log($"{brush.NodeID}");

            var parent = brush.Parent;
            while (parent.Valid)
            {
                var parentIndex = parent.NodeID - 1;
                bitset.Set(parentIndex, IntersectionType.Intersection);
                //Debug.Log($"parentIndex: {parent.NodeID}");
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
                //Debug.Log($"touchingBrushIndex: {touchingBrush.NodeID}");

                parent = touchingBrush.Parent;
                while (parent.Valid)
                {
                    var parentIndex = parent.NodeID - 1;
                    bitset.Set(parentIndex, IntersectionType.Intersection);
                    //Debug.Log($"parentIndex: {parent.NodeID}");
                    parent = parent.Parent;
                }
            }
        }


        static List<CategoryStackNode>      sRoutingTable       = new List<CategoryStackNode>();
        static List<RoutingLookup>          sRoutingLookups     = new List<RoutingLookup>();
        static List<CategoryGroupIndex>     sInputs             = new List<CategoryGroupIndex>();
        static List<CategoryRoutingRow>     sRoutingRows        = new List<CategoryRoutingRow>();
        static List<Loop[]>                 sIntersectionLoops  = new List<Loop[]>();
        static HashSet<CategoryGroupIndex>  sActiveInputs       = new HashSet<CategoryGroupIndex>();

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
            for (int i = 0; i < sRoutingTable.Count;)
            {
                var cutting_node = sRoutingTable[i].node;
                var cutting_node_id = cutting_node.nodeID;
                Debug.Assert(cutting_node.Valid);

                // TODO: store the routingRows per node, since inputs are in order & increase by 1, we can remove this and look them up directly
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
                    cuttingNodeIntersectionLoops = null;

                sIntersectionLoops.Add(cuttingNodeIntersectionLoops);

                sRoutingLookups.Add(new RoutingLookup(start_index, end_index));
            }

            routingTable.inputs = sInputs.ToArray();
            routingTable.routingRows = sRoutingRows.ToArray();
            routingTable.routingLookups = sRoutingLookups.ToArray();
            routingTable.intersectionLoops = sIntersectionLoops.ToArray();
        }



        #region Operation tables
#if HAVE_SELF_CATEGORIES
        static readonly CategoryRoutingRow[][] operationTables = new[]
        {
            // Additive set operation on polygons: output = (left-node || right-node)
            // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Additive Operation
            {
	            //             	  right node                                                                                     |
	            //                                  other       self            self             other                           |
	            //                  inside          aligned     aligned		    reverse-aligned  reverse-aligned  outside        |     left-node       
	            //-----------------------------------------------------------------------------------------------------------------------------------------
	            new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,               CategoryIndex.Inside,               CategoryIndex.Inside,           CategoryIndex.Inside            ), // inside
	            new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.SelfAligned,          CategoryIndex.Inside,               CategoryIndex.Inside,           CategoryIndex.Aligned           ), // other-aligned
	            new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.SelfAligned,          CategoryIndex.Inside,               CategoryIndex.Inside,           CategoryIndex.SelfAligned       ), // self-aligned
	            new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,               CategoryIndex.SelfReverseAligned,   CategoryIndex.ReverseAligned,   CategoryIndex.SelfReverseAligned), // self-reverse-aligned
	            new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,               CategoryIndex.SelfReverseAligned,   CategoryIndex.ReverseAligned,   CategoryIndex.ReverseAligned    ), // other-reverse-aligned
	            new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.SelfAligned,          CategoryIndex.SelfReverseAligned,   CategoryIndex.ReverseAligned,   CategoryIndex.Outside           )  // outside
            },

            // Subtractive set operation on polygons: output = !(!left-node || right-node)
            // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Subtractive Operation
            {
	            //             	  right node                                                                                     |
	            //                                  other       self            self             other                           |
	            //             	    inside          aligned     aligned		    reverse-aligned  reverse-aligned  outside        |     left-node       
	            //-----------------------------------------------------------------------------------------------------------------------------------------
	            new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.SelfReverseAligned,   CategoryIndex.SelfAligned,          CategoryIndex.Aligned,          CategoryIndex.Inside            ), // inside
	            new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,              CategoryIndex.SelfAligned,          CategoryIndex.Aligned,          CategoryIndex.Aligned           ), // other-aligned
	            new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,              CategoryIndex.SelfAligned,          CategoryIndex.Aligned,          CategoryIndex.SelfAligned       ), // self-aligned
	            new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.SelfReverseAligned,   CategoryIndex.Outside,              CategoryIndex.Outside,          CategoryIndex.SelfReverseAligned), // self-reverse-aligned
	            new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.SelfReverseAligned,   CategoryIndex.Outside,              CategoryIndex.Outside,          CategoryIndex.ReverseAligned    ), // other-reverse-aligned
	            new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,              CategoryIndex.Outside,              CategoryIndex.Outside,          CategoryIndex.Outside           )  // outside
            },

            // Common set operation on polygons: output = !(!left-node || !right-node)
            // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Intersection Operation
            {
	            //             	  right node                                                                                     |
	            //             	                    other       self            self             other                           |
	            //             	    inside          aligned     aligned		    reverse-aligned  reverse-aligned  outside        |     left-node       
	            //----------------------------------------------------------------------------------------------------------------------------
	            new CategoryRoutingRow( CategoryIndex.Inside,               CategoryIndex.Aligned,    CategoryIndex.SelfAligned,    CategoryIndex.SelfReverseAligned,   CategoryIndex.ReverseAligned,   CategoryIndex.Outside        ), // inside
	            new CategoryRoutingRow( CategoryIndex.Aligned,              CategoryIndex.Aligned,    CategoryIndex.SelfAligned,    CategoryIndex.Outside,              CategoryIndex.Outside,          CategoryIndex.Outside        ), // other-aligned
	            new CategoryRoutingRow( CategoryIndex.SelfAligned,          CategoryIndex.Aligned,    CategoryIndex.SelfAligned,    CategoryIndex.Outside,              CategoryIndex.Outside,          CategoryIndex.Outside        ), // self-aligned
	            new CategoryRoutingRow( CategoryIndex.SelfReverseAligned,   CategoryIndex.Outside,    CategoryIndex.Outside,        CategoryIndex.SelfReverseAligned,   CategoryIndex.ReverseAligned,   CategoryIndex.Outside        ), // self-reverse-aligned
	            new CategoryRoutingRow( CategoryIndex.ReverseAligned,       CategoryIndex.Outside,    CategoryIndex.Outside,        CategoryIndex.SelfReverseAligned,   CategoryIndex.ReverseAligned,   CategoryIndex.Outside        ), // other-reverse-aligned
	            new CategoryRoutingRow( CategoryIndex.Outside,              CategoryIndex.Outside,    CategoryIndex.Outside,        CategoryIndex.Outside,              CategoryIndex.Outside,          CategoryIndex.Outside        )  // outside
            }
        };
#else
        static readonly CategoryRoutingRow[][] operationTables = new[]
        {
            // Additive set operation on polygons: output = (left-node || right-node)
            // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Additive Operation
            {
			    //                      right node                                                                                                             |
			    //                                                    other                         other                                                      |
			    //                      inside                        aligned                       reverse-aligned               outside                      |     left-node       
			    //                      --------------------------------------------------------------------------------------------------------------------------------------------------
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside         ), // inside
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.Inside,         CategoryIndex.Aligned        ), // other-aligned
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.ReverseAligned, CategoryIndex.ReverseAligned ), // other-reverse-aligned
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.ReverseAligned, CategoryIndex.Outside        )  // outside
            },
            
		    // Subtractive set operation on polygons: output = !(!left-node || right-node)
		    // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Subtractive Operation
            {
			    //                      right node                                                                                                             |
			    //                                                    other                         other                                                      |
			    //                      inside                        aligned                       reverse-aligned               outside                      |     left-node       
			    //                      --------------------------------------------------------------------------------------------------------------------------------------------------
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.Aligned,        CategoryIndex.Inside         ), // inside
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Aligned,        CategoryIndex.Aligned        ), // other-aligned
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.Outside,        CategoryIndex.ReverseAligned ), // other-reverse-aligned
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside        )  // outside
            },
            
		    // Common set operation on polygons: output = !(!left-node || !right-node)
		    // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Intersection Operation
            {
			    //                      right node                                                                                                             |
			    //                                                    other                         other                                                      |
			    //                      inside                        aligned                       reverse-aligned               outside                      |     left-node       
			    //                      --------------------------------------------------------------------------------------------------------------------------------------------------
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.ReverseAligned, CategoryIndex.Outside        ), // inside
			    new CategoryRoutingRow( CategoryIndex.Aligned,        CategoryIndex.Aligned,        CategoryIndex.Outside,        CategoryIndex.Outside        ), // other-aligned
			    new CategoryRoutingRow( CategoryIndex.ReverseAligned, CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.Outside        ), // other-reverse-aligned
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside        )  // outside
            }
        };
        static readonly CategoryRoutingRow[][] simpleOperationTables = new[]
        {
            // Additive set operation on polygons: output = (left-node || right-node)
            // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Additive Operation
            {
			    //                      right node                                                                                                             |
			    //                                                    other                         other                                                      |
			    //                      inside                        aligned                       reverse-aligned               outside                      |     left-node       
			    //                      --------------------------------------------------------------------------------------------------------------------------------------------------
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside         ), // inside
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside         ), // other-aligned
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside         ), // other-reverse-aligned
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Outside        )  // outside
            },
            
		    // Subtractive set operation on polygons: output = !(!left-node || right-node)
		    // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Subtractive Operation
            {
			    //                      right node                                                                                                             |
			    //                                                    other                         other                                                      |
			    //                      inside                        aligned                       reverse-aligned               outside                      |     left-node       
			    //                      --------------------------------------------------------------------------------------------------------------------------------------------------
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside         ), // inside
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Inside,         CategoryIndex.Inside         ), // other-aligned
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Inside,         CategoryIndex.Outside,        CategoryIndex.Inside         ), // other-reverse-aligned
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside        )  // outside
            },
            
		    // Common set operation on polygons: output = !(!left-node || !right-node)
		    // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Intersection Operation
            {
			    //                      right node                                                                                                             |
			    //                                                    other                         other                                                      |
			    //                      inside                        aligned                       reverse-aligned               outside                      |     left-node       
			    //                      --------------------------------------------------------------------------------------------------------------------------------------------------
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Outside        ), // inside
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Outside,        CategoryIndex.Outside        ), // other-aligned
			    new CategoryRoutingRow( CategoryIndex.Inside,         CategoryIndex.Outside,        CategoryIndex.Inside,         CategoryIndex.Outside        ), // other-reverse-aligned
			    new CategoryRoutingRow( CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside        )  // outside
            }
        };
#endif
#endregion


        static readonly CategoryStackNode[] sEmptyStack = new CategoryStackNode[] { };

        static CategoryRoutingRow[] GetOperationTable(CSGOperationType operation, bool useFullOperationTables)
        {
#if !HAVE_SELF_CATEGORIES
#if USE_HALF_OPERATION_TABLE
            if (!useFullOperationTables)
                return simpleOperationTables[(int)operation];
#endif
#endif
            return operationTables[(int)operation];
        }


        static void FixUpIndices(List<CategoryStackNode> stack, Dictionary<int, int> remap, int start, int last)
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

        static CategoryStackNode[] Combine(BrushIntersectionLookup intersectionTypeLookup, CSGTreeBrush processedNode, CategoryStackNode[] leftStack, CategoryStackNode[] rightStack, CSGOperationType operation, ref bool useFullOperationTables, int depth)
        {
            if (operation == CSGOperationType.Invalid)
                operation = CSGOperationType.Additive;

            if (leftStack.Length == 0) // left node has a branch without children or children are not intersecting with processedNode
            {
                if (rightStack.Length == 0) // right node has a branch without children or children are not intersecting with processedNode
                    return sEmptyStack;
                switch (operation)
                {
                    case CSGOperationType.Additive: return rightStack;
                    default: return sEmptyStack;
                }
            } else
            if (rightStack.Length == 0) // right node has a branch without children or children are not intersecting with processedNode
            {
                switch (operation)
                {
                    case CSGOperationType.Additive:
                    case CSGOperationType.Subtractive: return leftStack;
                    default: return sEmptyStack;
                }
            }

            // TODO: use different tables before and after we passed ourselves to avoid duplicates
            var operationTable  = GetOperationTable(operation, useFullOperationTables);
            int index           = 0;
            int vIndex          = 0;
            
            var firstNode   = rightStack[0].node;


            // Count the number of rows for unique node
            var prevNode    = firstNode;
            var routingSteps = new List<int>();
            int counter = 0;
            for (int r = 0; r < rightStack.Length; r++)
            {
                if (prevNode != rightStack[r].node)
                {
                    routingSteps.Add(counter);
                    counter = 0;
                    prevNode = rightStack[r].node;
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
            /*
            var usedIndicesArray = usedIndices.ToArray();
            for (int t = 0; t < usedIndicesArray.Length; t++)
                Debug.Log(usedIndicesArray[t]);
            Debug.Log($"{prevNodeIndex} {startNodeIndex} {usedIndicesArray.Length}");
            */
            sCombineIndexRemap.Clear();

            int nodeIndex = 1;
            prevNode = firstNode;
            for (int r = 0; r < rightStack.Length; r++)
            {
                if (prevNode != rightStack[r].node)
                {
#if USE_OPTIMIZATIONS
                    //Debug.Log($"[{r}/{rightStack.Length}] &");
                    if (//nodeIndex > 1 && 
                        prevNodeIndex >= 0 && sCombineIndexRemap.Count > 0)
                    {
                        FixUpIndices(sCombineChildren, sCombineIndexRemap, prevNodeIndex, startNodeIndex);
#if SHOW_DEBUG_MESSAGES
                        if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                            Dump(sCombineChildren.ToArray(), depth);
#endif
                    }
#endif

                    prevNodeIndex = startNodeIndex;
                    startNodeIndex = sCombineChildren.Count;
                    index = 0; vIndex = 0; nodeIndex++;
                    prevNode = rightStack[r].node;
                    sCombineIndexRemap.Clear();

                    sCombineUsedIndices.Clear();
                    for (int p = prevNodeIndex; p < startNodeIndex; p++)
                    {
                        for (int t = 0; t < CategoryRoutingRow.Length; t++)
                            sCombineUsedIndices.Add((int)sCombineChildren[p].routingRow[t]);
                    }
                }

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
                            // Fix up output of last node to include operation between
                            // last left and last right.
                            // We don't add a routingOffset here since this might be the last node & 
                            // we don't even know if there is a next node at this point.
                            routingRow = new CategoryRoutingRow(operationTable, (CategoryIndex)t, rightStack[r].routingRow); // applies operation

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
                                        node = rightStack[r].node,
                                        input = (CategoryGroupIndex)index,
                                        routingRow = routingRow
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
                    //if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                    //{
                    //    Debug.Log($"[{startR}/{rightStack.Length}] {ncount} {vIndex} / {sCombineIndexRemap.Count} {rightStack[startR].node} {rightStack.Length - startR} +");
                    //    Dump(sCombineChildren.ToArray(), depth);
                    //}
#endif
                } else
                {
                    int ncount = 0;
                    int routingStep = routingSteps[nodeIndex];
                    // Duplicate route multiple times
                    for (int t = 0; t < CategoryRoutingRow.Length; t++, routingOffset += routingStep)
                    {
                        // Fix up routing to include offset b/c duplication
                        routingRow = rightStack[r].routingRow + routingOffset;

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
                                    node = rightStack[r].node,
                                    input = (CategoryGroupIndex)index,
                                    routingRow = routingRow
                                });
                                foundIndex = index;
                                index++;
                            }
                        }

                        sCombineIndexRemap[vIndex] = foundIndex;
                        vIndex++;
                        ncount++;
                    }
#if SHOW_DEBUG_MESSAGES
                    //if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                    //{
                    //    Debug.Log($"[{r}/{rightStack.Length}] {ncount} {vIndex} / {sCombineIndexRemap.Count} {rightStack[r].node} -");
                    //    Dump(sCombineChildren.ToArray(), depth);
                    //}
#endif
                }
            }

#if USE_OPTIMIZATIONS
            if (//nodeIndex > 1 && 
                prevNodeIndex >= 0 && sCombineIndexRemap.Count > 0)
            {
                FixUpIndices(sCombineChildren, sCombineIndexRemap, prevNodeIndex, startNodeIndex);
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
                    FixUpIndices(sCombineChildren, sCombineIndexRemap, prevNodeIndex, startNodeIndex);
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

        static CategoryStackNode[] GetStack(BrushIntersectionLookup intersectionTypeLookup, CSGTreeBrush processedNode, CSGTreeNode node)
        {
            bool useFullOperationTables = false;
            var stack = GetStack(intersectionTypeLookup, processedNode, node, ref useFullOperationTables, 0);
            if (stack == null ||
                stack.Length == 0)
                return new CategoryStackNode[] { new CategoryStackNode() { node = node, routingRow = CategoryRoutingRow.outside } };
            return stack;
        }

        static CategoryStackNode[] GetStack(BrushIntersectionLookup intersectionTypeLookup, CSGTreeBrush processedNode, CSGTreeNode node, ref bool useFullOperationTables, int depth)
        {
            Debug.Assert(processedNode.Valid);
            Debug.Assert(node.Valid);

            var nodeIndex           = node.nodeID - 1;
            var intersectionType    = intersectionTypeLookup.Get(nodeIndex);
            if (intersectionType == IntersectionType.NoIntersection)
                return sEmptyStack;// new CategoryStackNode[] { new CategoryStackNode() { node = node, routingRow = CategoryRoutingRow.outside } };

            // TODO: use other intersection types
            //if (intersectionType == IntersectionType.AInsideB) return all_inside;
            //if (intersectionType == IntersectionType.BInsideA) return all_outside;

            switch (node.Type)
            {
                case CSGNodeType.Brush:
                {
                    // All surfaces of processedNode are aligned with it's own surfaces, so all categories are Aligned
                    if (processedNode == node)
                    {
                        useFullOperationTables = true; // switch to full operation tables after we encountered ourselves in the hierarchy
                        return new CategoryStackNode[] { new CategoryStackNode() { node = node, routingRow = CategoryRoutingRow.selfAligned } };
                    }

                    // Otherwise return identity categories (input == output)
#if !HAVE_SELF_CATEGORIES
                    if (!useFullOperationTables)
                        return new CategoryStackNode[] { new CategoryStackNode() { node = node, routingRow = CategoryRoutingRow.insideOutside } };
                    else
#endif
                        return new CategoryStackNode[] { new CategoryStackNode() { node = node, routingRow = CategoryRoutingRow.identity } };
                }
                default:
                {
                    var nodeCount = node.Count;
                    if (nodeCount == 0)
                        return sEmptyStack;// new CategoryStackNode[] { new CategoryStackNode() { node = node, routingRow = CategoryRoutingRow.outside } };

                    // Skip all nodes that are not additive at the start of the branch since they will never produce any geometry
                    int firstIndex = 0;
                    for (; firstIndex < nodeCount && node[firstIndex].Valid && node[firstIndex].Operation != CSGOperationType.Additive; firstIndex++)
                        firstIndex++;

                    if ((nodeCount - firstIndex) <= 0)
                        return sEmptyStack;// new CategoryStackNode[] { new CategoryStackNode() { node = node, routingRow = CategoryRoutingRow.outside } };

                    if ((nodeCount - firstIndex) == 1)
                    {
                        var stack = GetStack(intersectionTypeLookup, processedNode, node[firstIndex], ref useFullOperationTables, depth + 1);

                        // Node operation is always Additive at this point, and operation would be performed against .. nothing ..
                        // Anything added with nothing is itself, so we don't need to apply an operation here.
                        //stack = ApplyOperation(node[firstIndex].Operation, CategoryIndex.Outside, stack);

#if SHOW_DEBUG_MESSAGES
                        if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                            Dump(stack, depth, "stack return ");
#endif
                        return stack;
                    } else
                    {
//                      if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
//                          Debug.Log($"{new String(' ', depth)}stack start '{node[firstIndex]}' {useFullOperationTables}");
                        var previousStack = GetStack(intersectionTypeLookup, processedNode, node[firstIndex], ref useFullOperationTables, depth + 1);
//                      if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
//                          Debug.Log($"{new String(' ', depth)}stack end '{node[firstIndex]}' {useFullOperationTables}");
                        for (int i = firstIndex + 1; i < nodeCount; i++)
                        {
                            if (!node[i].Valid)
                                continue;
#if SHOW_DEBUG_MESSAGES
                            if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                                Dump(previousStack, depth, $"before '{node[i-1]}' {node[i].Operation} '{node[i]}' {useFullOperationTables}");
#endif
                            var temp = useFullOperationTables;
//                          if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
//                              Debug.Log($"{new String(' ', depth)}stack start '{node[i]}' {useFullOperationTables}");
                            var currentStack = GetStack(intersectionTypeLookup, processedNode, node[i], ref temp, depth + 1);
//                          if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
//                              Debug.Log($"{new String(' ', depth)}stack end '{node[i]}' {useFullOperationTables}");

//                          if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
//                              Debug.Log($"{new String(' ', depth)}combine start");
                            previousStack =
                                Combine(
                                        intersectionTypeLookup, processedNode,
                                        previousStack, currentStack,
                                        node[i].Operation,
                                        ref useFullOperationTables,
                                        depth + 1
                                );
//                          if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
//                              Debug.Log($"{new String(' ', depth)}combine end");
                            useFullOperationTables = temp || useFullOperationTables;
#if SHOW_DEBUG_MESSAGES
                            if (processedNode.NodeID == kDebugNode || kDebugNode == -1)
                                Dump(previousStack, depth, $"after '{node[i - 1]}' {node[i].Operation} '{node[i]}' {useFullOperationTables}");
#endif
                        }
                        return previousStack;
                    }
                }
            }
        }

#if SHOW_DEBUG_MESSAGES
        static int kDebugNode = 2;
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