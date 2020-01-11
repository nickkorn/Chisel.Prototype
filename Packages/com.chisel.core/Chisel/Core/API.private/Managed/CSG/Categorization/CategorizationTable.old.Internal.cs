//#define OLD_ALGORITHM
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
#if OLD_ALGORITHM
        #region Operation tables
        // Additive set operation on polygons: output = (left-node || right-node)
        // Defines final output from combination of categorization of left and right node
        internal static readonly CategoryIndex[,] additiveOperation = new[,]
        {
			//right node                                                                                                             |
			//                              other                         other                                                      |
			//inside                        aligned                       reverse-aligned               outside                      |     left-node       
			//--------------------------------------------------------------------------------------------------------------------------------------------------
			{ CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.Inside         }, // inside
			{ CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.Inside,         CategoryIndex.Aligned        }, // other-aligned
			{ CategoryIndex.Inside,         CategoryIndex.Inside,         CategoryIndex.ReverseAligned, CategoryIndex.ReverseAligned }, // other-reverse-aligned
			{ CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.ReverseAligned, CategoryIndex.Outside        }  // outside
		};

        // Subtractive set operation on polygons: output = !(!left-node || right-node)
        // Defines final output from combination of categorization of left and right node
        internal static readonly CategoryIndex[,] subtractiveOperation = new[,]
        {
			//right node                                                                                                             |
			//                              other                         other                                                      |
			//inside                        aligned                       reverse-aligned               outside                      |     left-node       
			//--------------------------------------------------------------------------------------------------------------------------------------------------
			{ CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.Aligned,        CategoryIndex.Inside         }, // inside
			{ CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Aligned,        CategoryIndex.Aligned        }, // other-aligned
			{ CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.Outside,        CategoryIndex.ReverseAligned }, // other-reverse-aligned
			{ CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside        }  // outside
		};

        // Common set operation on polygons: output = !(!left-node || !right-node)
        // Defines final output from combination of categorization of left and right node
        internal static readonly CategoryIndex[,] commonOperation = new[,]
        {
			//right node                                                                                                             |
			//                              other                         other                                                      |
			//inside                        aligned                       reverse-aligned               outside                      |     left-node       
			//--------------------------------------------------------------------------------------------------------------------------------------------------
			{ CategoryIndex.Inside,         CategoryIndex.Aligned,        CategoryIndex.ReverseAligned, CategoryIndex.Outside        }, // inside
			{ CategoryIndex.Aligned,        CategoryIndex.Aligned,        CategoryIndex.Outside,        CategoryIndex.Outside        }, // other-aligned
			{ CategoryIndex.ReverseAligned, CategoryIndex.Outside,        CategoryIndex.ReverseAligned, CategoryIndex.Outside        }, // other-reverse-aligned
			{ CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside,        CategoryIndex.Outside        }  // outside
		};

        static readonly CategoryIndex[][,] csg_operations = new[]
        {
            CategoryStackNode.additiveOperation,	// CSGOperationType.Additive == 0
			CategoryStackNode.subtractiveOperation,	// CSGOperationType.Subtractive == 1
			CategoryStackNode.commonOperation,		// CSGOperationType.Common == 2

			CategoryStackNode.additiveOperation		// 3 (invalid value)
		};
        #endregion

        static CategoryRoutingRow CategorizeNode(List<CategoryStackNode> routingTable,
                                                 BrushIntersectionLookup intersectionTypeLookup,
                                                 CategoryRoutingRow parentRoutingRow,
                                                 CSGTreeNode childNode,
                                                 CSGTreeNode processedNode,
                                                 CSGNodeType childType,
                                                 CSGOperationType childOperationType,
                                                 ref int polygonGroupCount,
                                                 bool haveGonePastSelf)
        {
            if (parentRoutingRow.AreAllTheSame())
                return parentRoutingRow;

            var child_node_is_brush = (childType & CSGNodeType.Brush) == CSGNodeType.Brush;
            var current_operation = csg_operations[(int)childOperationType];
            if (childNode == processedNode && child_node_is_brush)
            {
                // All categories lead to 'aligned' since this is the processed brush
                parentRoutingRow.RouteFrom(parentRoutingRow,
                                               current_operation[(int)CategoryIndex.Inside, (int)CategoryIndex.Aligned],
                                               current_operation[(int)CategoryIndex.Aligned, (int)CategoryIndex.Aligned],
                                               current_operation[(int)CategoryIndex.ReverseAligned, (int)CategoryIndex.Aligned],
                                               current_operation[(int)CategoryIndex.Outside, (int)CategoryIndex.Aligned]);
                return parentRoutingRow;
            }
            var childNodeIndex = childNode.NodeID - 1;
            var intersectionType = intersectionTypeLookup.Get(childNodeIndex);
            UnityEngine.Debug.Assert(intersectionType != IntersectionType.InvalidValue, $"intersectionType == IntersectionType.InvalidValue");
            if (intersectionType == IntersectionType.NoIntersection ||
                intersectionType == IntersectionType.BInsideA)
            {
                // All categories lead to 'outside' since this brush doesn't touch the processed brush
                parentRoutingRow.RouteFrom(parentRoutingRow,
                                               current_operation[(int)CategoryIndex.Inside, (int)CategoryIndex.Outside],
                                               current_operation[(int)CategoryIndex.Aligned, (int)CategoryIndex.Outside],
                                               current_operation[(int)CategoryIndex.ReverseAligned, (int)CategoryIndex.Outside],
                                               current_operation[(int)CategoryIndex.Outside, (int)CategoryIndex.Outside]);
                return parentRoutingRow;
            } else
            if (intersectionType == IntersectionType.AInsideB)
            {
                // All categories lead to 'outside' since this brush doesn't touch the processed brush
                parentRoutingRow.RouteFrom(parentRoutingRow,
                                               current_operation[(int)CategoryIndex.Inside, (int)CategoryIndex.Inside],
                                               current_operation[(int)CategoryIndex.Aligned, (int)CategoryIndex.Inside],
                                               current_operation[(int)CategoryIndex.ReverseAligned, (int)CategoryIndex.Inside],
                                               current_operation[(int)CategoryIndex.Outside, (int)CategoryIndex.Inside]);
                return parentRoutingRow;
            }

            var output_routing_row = new CategoryRoutingRow();// output polygon paths

            // determine the required outputs
            for (int rightCategoryIndex = 0; rightCategoryIndex < CategoryRoutingRow.Length; rightCategoryIndex++)
            {
                var new_routing_row = new CategoryRoutingRow();
                // determine the correct destination for all input polygons and add them to the stack for processing
                if (child_node_is_brush)
                {
                    // Eat polygons that are aligned with another brush
                    if (haveGonePastSelf)
                    {
                        new_routing_row.RouteFrom(parentRoutingRow,
                                                  current_operation[rightCategoryIndex, (int)CategoryIndex.Inside],
                                                  current_operation[rightCategoryIndex, (int)CategoryIndex.Outside],
                                                  current_operation[rightCategoryIndex, (int)CategoryIndex.Inside],
                                                  current_operation[rightCategoryIndex, (int)CategoryIndex.Outside]);
                    } else
                    {
                        new_routing_row.RouteFrom(parentRoutingRow,
                                                  current_operation[rightCategoryIndex, (int)CategoryIndex.Inside],
                                                  current_operation[rightCategoryIndex, (int)CategoryIndex.Inside],
                                                  current_operation[rightCategoryIndex, (int)CategoryIndex.Inside],
                                                  current_operation[rightCategoryIndex, (int)CategoryIndex.Outside]);
                    }
                } else
                {
                    new_routing_row.RouteFrom(parentRoutingRow,
                                              current_operation[rightCategoryIndex, (int)CategoryIndex.Inside],
                                              current_operation[rightCategoryIndex, (int)CategoryIndex.Aligned],
                                              current_operation[rightCategoryIndex, (int)CategoryIndex.ReverseAligned],
                                              current_operation[rightCategoryIndex, (int)CategoryIndex.Outside]);
                }

                //
                // All paths in route lead to same location, so don't bother storing it
                //
                if (new_routing_row.AreAllTheSame())
                {
                    output_routing_row[rightCategoryIndex] = new_routing_row[CategoryIndex.Outside]; // pick any one of the lists as destination
                    continue;
                }

                //
                // See if we have a duplicate route
                //
                var prevCategory = (int)routingTable.Count - 1;
                while (prevCategory > 0 && routingTable[prevCategory].node == childNode)
                {
                    if (routingTable[prevCategory].routingRow.Equals(new_routing_row))
                    // found a duplicate, use that one instead
                    {
                        output_routing_row[rightCategoryIndex] = routingTable[prevCategory].input;
                        goto SkipRouteCreation;
                    }
                    prevCategory--;
                }
                {
                    //
                    // Add a new route
                    //
                    var input_polygon_group_index = (CategoryGroupIndex)polygonGroupCount; polygonGroupCount++;
                    {
                        routingTable.Add(new CategoryStackNode()
                        {
                            input = input_polygon_group_index,
                            node = childNode,
                            routingRow = new_routing_row
                        });
                    }

                    output_routing_row[rightCategoryIndex] = input_polygon_group_index;
                }
SkipRouteCreation:
                ;
            }

            // store the current output polygon lists, so we can use it as output for the previous node
            return output_routing_row;
        }

        static CategoryRoutingRow CategorizeFirstNode(List<CategoryStackNode> routingTable,
                                                      BrushIntersectionLookup intersectionTypeLookup,
                                                      CategoryRoutingRow parentRoutingRow,
                                                      CategoryGroupIndex inputPolygonGroupIndex,
                                                      CSGTreeNode childNode,
                                                      CSGTreeNode processedNode,
                                                      CSGNodeType childNodeType,
                                                      CSGOperationType childOperationType,
                                                      bool haveGonePastSelf)
        {
            var child_node_is_brush = (childNodeType & CSGNodeType.Brush) == CSGNodeType.Brush;
            var intersectionType = IntersectionType.InvalidValue;

            // All categories lead to 'aligned' since this is the processed brush
            if (childNode == processedNode && child_node_is_brush)
            {
                parentRoutingRow.RerouteAll(CategoryIndex.Aligned);
                goto found;
            }

            // All categories lead to 'outside' since this brush doesn't touch the processed brush
            else
            {
                var childNodeIndex = childNode.NodeID - 1;
                intersectionType = intersectionTypeLookup.Get(childNodeIndex);
                UnityEngine.Debug.Assert(intersectionType != IntersectionType.InvalidValue, $"intersectionType == IntersectionType.InvalidValue");
                if (intersectionType == IntersectionType.NoIntersection) { parentRoutingRow.RerouteAll(CategoryIndex.Outside); goto found; } else if (intersectionType == IntersectionType.AInsideB) { parentRoutingRow.RerouteAll(CategoryIndex.Inside); goto found; }
            }

            if (child_node_is_brush)
            {
                // Remove polygons that are aligned and removed by another brush
                if (haveGonePastSelf)
                {
                    parentRoutingRow.Reroute(CategoryIndex.Inside, CategoryIndex.Outside, CategoryIndex.Inside, CategoryIndex.Outside);
                    goto found;
                } else
                {
                    parentRoutingRow.Reroute(CategoryIndex.Inside, CategoryIndex.Inside, CategoryIndex.Inside, CategoryIndex.Outside);
                    goto found;
                }
            }
// else, no need to reroute, everything is already pointing to the correct destination

found:
            routingTable.Add(new CategoryStackNode()
            {
                input = inputPolygonGroupIndex,
                node = childNode,
                routingRow = parentRoutingRow
            });

            return parentRoutingRow;
        }


        class CSGStackData
        {
            public CSGStackData(int first_sibling_index, int last_sibling_index, int stack_node_counter, int current_stack_node_index, CSGTreeNode categorization_node)
            {
                firstSiblingIndex = first_sibling_index;
                lastSiblingIndex = last_sibling_index;
                currentSiblingIndex = last_sibling_index;
                stackNodeCount = stack_node_counter;
                parentStackNodeIndex = current_stack_node_index;
                parentNode = categorization_node;
            }
            public int firstSiblingIndex;
            public int lastSiblingIndex;
            public int currentSiblingIndex;
            public int stackNodeCount;
            public int parentStackNodeIndex;
            public CSGTreeNode parentNode;
        };


        static void AddToCSGStack(List<CSGStackData> stackIterator, List<CategoryStackNode> categoryOperations, CSGTreeNode currentNode, int currentStackNodeIndex)
        {
            CSGTreeNode categorization_node = currentNode;
            Int32 child_node_count = (categorization_node.NodeID == CSGTreeNode.InvalidNodeID) ? 0 : categorization_node.Count;
            Int32 last_sibling_index = child_node_count - 1;
            Int32 first_sibling_index = 0;

            // 
            // Find first node that is actually additive (we can't subtract or find common area of ... nothing)
            //
            for (first_sibling_index = 0; first_sibling_index < child_node_count; first_sibling_index++)
            {
                var child_node = categorization_node[first_sibling_index];
                var child_operation_type = child_node.Operation;
                if (child_operation_type == CSGOperationType.Additive)
                    break;
            }

            if (first_sibling_index >= child_node_count)
                return;

            var stack_node_counter = 0;
            do
            {
                stack_node_counter++;
            } while (currentStackNodeIndex - stack_node_counter > 0 &&
                     categoryOperations[currentStackNodeIndex - stack_node_counter].node == currentNode);

            stackIterator.Add(new CSGStackData(first_sibling_index, last_sibling_index, stack_node_counter, currentStackNodeIndex, categorization_node));
        }

        static void OptimizeCategorizationTable(int categoryStackNodeCount)
        {
            //
            // Remove redundant nodes
            //

            //
            // Remove nodes that have a polygon destination that is unreachable
            //
            sActiveInputs.Clear();
            sActiveInputs.Add(CategoryGroupIndex.First);
            for (int i = categoryStackNodeCount - 1; i >= 0; i--)
            {
                var row = sRoutingTable[i].routingRow;
                for (int c = 0; c < CategoryRoutingRow.Length; c++)
                    sActiveInputs.Add(row[c]);
            }

            for (int i = categoryStackNodeCount - 1; i >= 0; i--)
            {
                var input = sRoutingTable[i].input;
                if (sActiveInputs.Contains(input))
                    continue;

                var item = sRoutingTable[i];
                item.SetInvalid();
                sRoutingTable[i] = item;
            }


            //
            // Remove nodes where all its destinations go to the same location
            //
            int offset;
            offset = 0;

            // NOTE: causes artifacts?
            /*
            for (int i = 0; i < categoryStackNodeCount; i++)
            {
                var destination = sRoutingTable[i].routingRow[0];
                if (destination == CategoryGroupIndex.Invalid ||
                    !sRoutingTable[i].routingRow.AreAllTheSame())
                    continue;

                var input = sRoutingTable[i].input;
                var source = input;
                if (input == CategoryGroupIndex.First)
                {
                    if (destination <= (CategoryGroupIndex)CategoryRoutingRow.Length)
                        continue;
                    for (int j = i - 1; j >= 0; j--)
                    {
                        if (sRoutingTable[j].input == CategoryGroupIndex.Invalid)
                            continue;
                        if (sRoutingTable[j].input == destination)
                        {
                            var categoryStackNode = sRoutingTable[j];
                            categoryStackNode.input = source;
                            sRoutingTable[j] = categoryStackNode;
                        }

                        var row = sRoutingTable[j].routingRow;
                        for (int c = 0; c < CategoryRoutingRow.Length; c++)
                        {
                            if (row[c] == destination) row[c] = source;
                        }
                    }
                } else
                {
                    for (int j = offset; j < categoryStackNodeCount; j++)
                    {
                        if (sRoutingTable[j].input == source)
                        {
                            var categoryStackNode = sRoutingTable[j];
                            categoryStackNode.input = destination;
                            sRoutingTable[j] = categoryStackNode;
                        }

                        var row = sRoutingTable[j].routingRow;
                        for (int c = 0; c < CategoryRoutingRow.Length; c++)
                        {
                            if (row[c] == source) row[c] = destination;
                        }
                    }
                }

                var item = sRoutingTable[i];
                item.SetInvalid();
                sRoutingTable[i] = item;
            }
            */


            //
            // Remove nodes that have a polygon destination that is unreachable
            //
            for (int n = 0; n < categoryStackNodeCount / 2; n++) // TODO: solve this a better way
            {
                sActiveInputs.Clear();
                sActiveInputs.Add(CategoryGroupIndex.First);
                for (int i = categoryStackNodeCount - 1; i >= 0; i--)
                {
                    var row = sRoutingTable[i].routingRow;
                    for (int c = 0; c < CategoryRoutingRow.Length; c++)
                        sActiveInputs.Add(row[c]);
                }

                for (int i = categoryStackNodeCount - 1; i >= 0; i--)
                {
                    var input = sRoutingTable[i].input;
                    if (sActiveInputs.Contains(input))
                        continue;

                    var item = sRoutingTable[i];
                    item.SetInvalid();
                    sRoutingTable[i] = item;
                }
            }

            //
            // Same as above but in opposite direction to handle some edge cases
            //
            offset = 0;
            for (int i = categoryStackNodeCount - 1; i >= 0; i--)
            {
                var destination = sRoutingTable[i].routingRow[0];
                if (destination >= CategoryGroupIndex.First)
                {
                    if (!sRoutingTable[i].routingRow.AreAllTheSame())
                    {
                        if (offset > 0)
                        {
                            sRoutingTable[i + offset] = sRoutingTable[i];
                        }
                        continue;
                    }

                    var input = sRoutingTable[i].input;
                    var source = input;
                    if (input == CategoryGroupIndex.First)
                    {
                        if (destination <= (CategoryGroupIndex)CategoryRoutingRow.Length)
                            continue;
                        for (int j = i - 1; j >= 0; j--)
                        {
                            if (sRoutingTable[j].input == CategoryGroupIndex.Invalid)
                                continue;
                            if (sRoutingTable[j].input == destination)
                            {
                                var categoryStackNode = sRoutingTable[j];
                                categoryStackNode.input = source;
                                sRoutingTable[j] = categoryStackNode;
                            }

                            var row = sRoutingTable[j].routingRow;
                            for (int c = 0; c < CategoryRoutingRow.Length; c++)
                            {
                                if (row[c] == destination) row[c] = source;
                            }
                        }
                    } else
                    {
                        for (int j = categoryStackNodeCount - 1; j >= offset; j--)
                        {
                            if (sRoutingTable[j].input == source)
                            {
                                var categoryStackNode = sRoutingTable[j];
                                categoryStackNode.input = destination;
                                sRoutingTable[j] = categoryStackNode;
                            }

                            var row = sRoutingTable[j].routingRow;
                            for (int c = 0; c < CategoryRoutingRow.Length; c++)
                            {
                                if (row[c] == source) row[c] = destination;
                            }
                        }
                    }
                }
                offset++;
            }//*
            if (offset > 0)
            {
                categoryStackNodeCount -= offset;
                if (categoryStackNodeCount > 0 && offset > 0)
                {
                    for (int i = 0; i < categoryStackNodeCount; i++)
                        sRoutingTable[i] = sRoutingTable[i + offset];
                }
                if (categoryStackNodeCount < sRoutingTable.Count)
                    sRoutingTable.RemoveRange(categoryStackNodeCount, sRoutingTable.Count - categoryStackNodeCount);
            }
            //*/
        }

        // Make sure all indices are in order so we can simplify lookups
        static void ReorderIndices()
        {
            if (sRoutingTable.Count < 1)
                return;

            var remap = new Dictionary<CategoryGroupIndex, CategoryGroupIndex>();
            var index = RoutingLookup.kRoutingOffset;
            for (int i = 0; i < sRoutingTable.Count; i++)
            {
                remap[sRoutingTable[i].input] = (CategoryGroupIndex)index;
                index++;

                var item = sRoutingTable[i];
                var row = item.routingRow;
                for (int n = 0; n < CategoryRoutingRow.Length; n++)
                {
                    if ((int)row[n] <= (int)CategoryIndex.LastCategory)
                        row[n] = (CategoryGroupIndex)(-(int)row[n]);
                }
                item.routingRow = row;
                sRoutingTable[i] = item;
            }

            remap[CategoryGroupIndex.First] = CategoryGroupIndex.First;
            remap[CategoryGroupIndex.Invalid] = CategoryGroupIndex.Invalid;
            remap[(CategoryGroupIndex)(-(int)(CategoryIndex.Inside))] = (CategoryGroupIndex)(int)CategoryIndex.Inside;
            remap[(CategoryGroupIndex)(-(int)(CategoryIndex.Aligned))] = (CategoryGroupIndex)(int)CategoryIndex.Aligned;
            remap[(CategoryGroupIndex)(-(int)(CategoryIndex.ReverseAligned))] = (CategoryGroupIndex)(int)CategoryIndex.ReverseAligned;
            remap[(CategoryGroupIndex)(-(int)(CategoryIndex.Outside))] = (CategoryGroupIndex)(int)CategoryIndex.Outside;

            for (int i = 0; i < sRoutingTable.Count; i++)
            {
                var item = sRoutingTable[i];
                var row = item.routingRow;
                item.input = remap[item.input];
                for (int n = 0; n < CategoryRoutingRow.Length; n++)
                {
                    if (remap.ContainsKey(row[n]))
                        row[n] = remap[row[n]];
                }
                item.routingRow = row;
                sRoutingTable[i] = item;
            }
        }

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

            var categoryOperations = new List<CategoryStackNode>(CSGManager.GetMaxNodeIndex() * 6)
            {
                new CategoryStackNode()
                {
                    input       = CategoryGroupIndex.First,
                    node        = rootNode,
                    routingRow  = new CategoryRoutingRow((CategoryGroupIndex)(1), (CategoryGroupIndex)(2), (CategoryGroupIndex)(3), (CategoryGroupIndex)(1))
//					routingRow  = new CategoryRoutingRow((CategoryGroupIndex)(1), (CategoryGroupIndex)(2), (CategoryGroupIndex)(3), (CategoryGroupIndex)(4))
				}
            };

            var stackIterator = new List<CSGStackData>();
            AddToCSGStack(stackIterator, categoryOperations, rootNode,
                            0 //= root StackNodeIndex 
                            );

            if (stackIterator.Count == 0)
            {
                var parent_routing_row = categoryOperations[0].routingRow;
                parent_routing_row.RouteAllFrom(parent_routing_row, CategoryIndex.Outside);
                return;
            }

            polygonGroupCount = CategoryRoutingRow.Length;
            bool haveGonePastSelf = false;
            while (stackIterator.Count > 0)
            {
                var currentStack = stackIterator[stackIterator.Count - 1];
                var sibling_index = currentStack.currentSiblingIndex;
                var first_sibling_index = currentStack.firstSiblingIndex;
                var categorization_node = currentStack.parentNode;
                var stack_node_counter = currentStack.stackNodeCount;
                var current_stack_node_index = currentStack.parentStackNodeIndex;

                if (sibling_index < first_sibling_index)
                {
                    stackIterator.RemoveAt(stackIterator.Count - 1);
                    continue;
                }

                if (!haveGonePastSelf && sibling_index == first_sibling_index && first_sibling_index > 0)
                {
                    // Ensure we haven't passed ourselves in one of the brushes we skipped ..
                    for (int i = 0; !haveGonePastSelf && i < first_sibling_index; i++)
                        haveGonePastSelf = (categorization_node[i] == processedNode);
                }

                var child_node = categorization_node[sibling_index];
                var child_operation_type = child_node.Operation;
                var child_node_type = child_node.Type;
                var child_node_is_brush = (child_node_type & CSGNodeType.Brush) == CSGNodeType.Brush;

                var prev_stack_node_count = categoryOperations.Count;
                var categoryNodes = child_node_is_brush ? sRoutingTable : categoryOperations;

                if (sibling_index != first_sibling_index)
                {
                    for (int stack_node_iterator = stack_node_counter - 1; stack_node_iterator >= 0; stack_node_iterator--)
                    {
                        var parent_stack_node_index = current_stack_node_index - stack_node_iterator;
                        var category_stack_node = categoryOperations[parent_stack_node_index];
                        var parent_routing_row = category_stack_node.routingRow;
                        category_stack_node.routingRow = CategorizeNode(categoryNodes, intersectionTypeLookup, parent_routing_row, child_node, processedNode, child_node_type, child_operation_type, ref polygonGroupCount, haveGonePastSelf);
                        categoryOperations[parent_stack_node_index] = category_stack_node;
                    }
                } else
                {
                    //var 	prev_brush_count = (int)routingTable.Count;
                    for (int stack_node_iterator = stack_node_counter - 1; stack_node_iterator >= 0; stack_node_iterator--)
                    {
                        var parent_stack_node_index = current_stack_node_index - stack_node_iterator;
                        var category_stack_node = categoryOperations[parent_stack_node_index];
                        var parent_routing_row = category_stack_node.routingRow;
                        var input_polygon_group_index = category_stack_node.input;
                        category_stack_node.routingRow = CategorizeFirstNode(categoryNodes, intersectionTypeLookup, parent_routing_row, input_polygon_group_index, child_node, processedNode, child_node_type, child_operation_type, haveGonePastSelf);
                        categoryOperations[parent_stack_node_index] = category_stack_node;
                    }
                }

                if (child_node == processedNode)
                    haveGonePastSelf = true;

                currentStack.currentSiblingIndex--;

                if (child_node_is_brush)
                    continue;

                var curr_stack_node_count = categoryOperations.Count;

                if (prev_stack_node_count == curr_stack_node_count)
                    continue;

                var categorizationNodeIndex = categorization_node.NodeID - 1;
                var intersectionType = intersectionTypeLookup.Get(categorizationNodeIndex);
                UnityEngine.Debug.Assert(intersectionType != IntersectionType.InvalidValue, $"intersectionType == IntersectionType.InvalidValue");
                if (intersectionType == IntersectionType.NoIntersection)
                    continue;

                AddToCSGStack(stackIterator, categoryOperations, child_node, (int)curr_stack_node_count - 1);
            }

            categoryStackNodeCount = (int)sRoutingTable.Count;

#if USE_OPTIMIZATIONS
            // FIXME: fails sometimes
            OptimizeCategorizationTable(categoryStackNodeCount);
            categoryStackNodeCount = (int)sRoutingTable.Count;
#endif

            if (categoryStackNodeCount == 0)
            {
                polygonGroupCount = (CategoryRoutingRow.Length + 1);
                return;
            }


            // TODO:	still possible to have first node be itself, all its destination are set to outside
            //			node could be removed


            sRoutingTable.Reverse();

            /*
            Debug.Log($"-- {processedNode}");
            for (int i = 0; i < sRoutingTable.Count; i++)
            {
                Debug.Log($"{i}/{sRoutingTable.Count}: {sRoutingTable[i]}");
            }*/
            ReorderIndices();

            categoryStackNodeCount = (int)sRoutingTable.Count;

            int maxCounter = (int)CategoryRoutingRow.Length;
            for (int i = 0; i < categoryStackNodeCount; i++)
                maxCounter = Math.Max(maxCounter, (int)sRoutingTable[i].input);
            polygonGroupCount = maxCounter + 1;

            sRoutingLookups.Clear();
            sInputs.Clear();
            sRoutingRows.Clear();
            sIntersectionLoops.Clear();
            //          var nodes = new List<int>();
            for (int i = 0; i < sRoutingTable.Count;)
            {
                var cutting_node = sRoutingTable[i].node;
                var cutting_node_id = cutting_node.nodeID;
                Debug.Assert(cutting_node.Valid);

                // TODO: store the routingRows per node, since inputs are in order & increase by 1, we can remove this and look them up directly
                int start_index = i;
                do
                {
                    //                  nodes.Add(cutting_node_id);
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
#endif
    }
#endif
}