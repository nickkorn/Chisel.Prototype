#define USE_OPTIMIZATIONS
//#define SHOW_DEBUG_MESSAGES 
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using Unity.Collections;
using UnityEngine;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    #region Operation tables
    static class OperationTables
    {
#if HAVE_SELF_CATEGORIES
        const CategoryGroupIndex Inside             = (CategoryGroupIndex)CategoryIndex.Inside;
        const CategoryGroupIndex Aligned            = (CategoryGroupIndex)CategoryIndex.Aligned;
        const CategoryGroupIndex SelfAligned        = (CategoryGroupIndex)CategoryIndex.SelfAligned;
        const CategoryGroupIndex SelfReverseAligned = (CategoryGroupIndex)CategoryIndex.SelfReverseAligned;
        const CategoryGroupIndex ReverseAligned     = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
        const CategoryGroupIndex Outside            = (CategoryGroupIndex)CategoryIndex.Outside;

        // TODO: Burst might support reading this directly now?
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
        const CategoryGroupIndex Inside         = (CategoryGroupIndex)CategoryIndex.Inside;
        const CategoryGroupIndex Aligned        = (CategoryGroupIndex)CategoryIndex.Aligned;
        const CategoryGroupIndex ReverseAligned = (CategoryGroupIndex)CategoryIndex.ReverseAligned;
        const CategoryGroupIndex Outside        = (CategoryGroupIndex)CategoryIndex.Outside;

        // TODO: Burst might support reading this directly now?
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
	            new CategoryRoutingRow( Inside,           Outside,          Inside,           Aligned           ), // aligned
	            new CategoryRoutingRow( Inside,           Inside,           Outside,          ReverseAligned    ), // reverse-aligned
	            new CategoryRoutingRow( Inside,           Outside,          Outside,          Outside           )  // outside
            },

            // Subtractive set operation on polygons: output = !(!left-node || right-node)
            // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Subtractive Operation
            {
	            //             	        right node                                                              |
	            //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	            //----------------------------------------------------------------------------------------------------------------------------
                new CategoryRoutingRow( Outside,          Outside,          Outside,          Inside            ), // inside
                new CategoryRoutingRow( Outside,          Outside,          Outside,          Aligned           ), // aligned
                new CategoryRoutingRow( Outside,          Outside,          Outside,          ReverseAligned    ), // reverse-aligned
                new CategoryRoutingRow( Outside,          Outside,          Outside,          Outside           )  // outside
            }, 

            // Common set operation on polygons: output = !(!left-node || !right-node)
            // Defines final output from combination of categorization of left and right node
            new CategoryRoutingRow[] // Intersection Operation
            {
	            //             	        right node                                                              |
	            //             	        inside            aligned           reverse-aligned   outside           |     left-node       
	            //----------------------------------------------------------------------------------------------------------------------------
	            new CategoryRoutingRow( Inside,           Outside,          Outside,          Outside           ), // inside
	            new CategoryRoutingRow( Aligned,          Outside,          Outside,          Outside           ), // aligned
	            new CategoryRoutingRow( ReverseAligned,   Outside,          Outside,          Outside           ), // reverse-aligned
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
        
        // TODO: Burst might support reading this directly now?
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
                new CategoryRoutingRow( Outside,          Outside,          Aligned,          Aligned           ), // aligned
                new CategoryRoutingRow( Outside,          ReverseAligned,   Outside,          ReverseAligned    ), // reverse-aligned
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

        public const int NumberOfRowsPerOperation = 4;
        public const int RemoveOverlappingOffset = 4 * NumberOfRowsPerOperation;
        public static NativeArray<CategoryRoutingRow> Rows;


        public static void EnsureInitialized()
        {
            if (Rows.IsCreated)
                Rows.Dispose();

            // TODO: blobAsset instead?
            // TODO: Burst might support reading static readonly arrays directly now?
            Rows = new NativeArray<CategoryRoutingRow>((RemoveOverlappingOperationTables.Length * NumberOfRowsPerOperation) +
                                                       (RegularOperationTables.Length * NumberOfRowsPerOperation), Allocator.Persistent);

            Debug.Assert(NumberOfRowsPerOperation == RegularOperationTables[0].Length);
            int n = 0;
            for (int i=0;i< RegularOperationTables.Length;i++)
            {
                for (int j = 0; j < NumberOfRowsPerOperation; j++, n++)
                {
                    Rows[n] = RegularOperationTables[i][j];
                }
            }
            Debug.Assert(n == RemoveOverlappingOffset);
            for (int i = 0; i < RemoveOverlappingOperationTables.Length; i++)
            {
                for (int j = 0; j < NumberOfRowsPerOperation; j++, n++)
                {
                    Rows[n] = RemoveOverlappingOperationTables[i][j];
                }
            }
        }

        public static void EnsureCleanedUp()
        {
            if (Rows.IsCreated)
                Rows.Dispose();
        }
#endif
    }
    #endregion
#endif
}