﻿#if DEBUG
#define DEBUG_CATEGORIES // visual studio debugging bug work around
#endif
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
    public enum CategoryGroupIndex : int
    {
        First = 0,
        Invalid = -1
    }

    internal unsafe struct CategoryRoutingRow
    {
        public static readonly CategoryRoutingRow invalid       = new CategoryRoutingRow(CategoryGroupIndex.Invalid, CategoryGroupIndex.Invalid, CategoryGroupIndex.Invalid, CategoryGroupIndex.Invalid, CategoryGroupIndex.Invalid, CategoryGroupIndex.Invalid);
        public static readonly CategoryRoutingRow identity      = new CategoryRoutingRow(CategoryIndex.Inside, CategoryIndex.Aligned, CategoryIndex.SelfAligned, CategoryIndex.SelfReverseAligned, CategoryIndex.ReverseAligned, CategoryIndex.Outside);
        public readonly static CategoryRoutingRow selfAligned   = new CategoryRoutingRow(CategoryIndex.SelfAligned, CategoryIndex.SelfAligned, CategoryIndex.SelfAligned, CategoryIndex.SelfAligned, CategoryIndex.SelfAligned, CategoryIndex.SelfAligned);
        public readonly static CategoryRoutingRow outside       = new CategoryRoutingRow(CategoryIndex.Outside, CategoryIndex.Outside, CategoryIndex.Outside, CategoryIndex.Outside, CategoryIndex.Outside, CategoryIndex.Outside);

        public const int Length = (int)CategoryIndex.LastCategory + 1;

        // Is PolygonGroupIndex instead of int, but C# doesn't like that
#if !DEBUG_CATEGORIES
        fixed int	destination[Length];
#else
        // visual studio debugging bug work around
        struct Int4Array
        {
            int A; int B; int C; int D; int E; int F;
            public unsafe int this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    fixed (int* ptr = &A)
                    {
                        return ptr[index];
                    }
                }
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set
                {
                    fixed (int* ptr = &A)
                    {
                        ptr[index] = value;
                    }
                }
            }

        }
        Int4Array   destination;
#endif

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CategoryRoutingRow(CategoryRoutingRow[] operationTable, CategoryIndex left, in CategoryRoutingRow right)
        {
#if DEBUG_CATEGORIES
            destination = new Int4Array();
#endif
            var operationRow = operationTable[(int)left];
            for (int i = 0; i < Length; i++)
                destination[(int)i] = (int)operationRow[right[i]];
        }

        public static CategoryRoutingRow operator +(CategoryRoutingRow a, int offset)
        {
            var newRow = new CategoryRoutingRow();
#if DEBUG_CATEGORIES
            newRow.destination = new Int4Array();
#endif
            for (int i = 0; i < Length; i++)
                newRow.destination[(int)i] = (int)a[i] + offset;

            return newRow;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CategoryRoutingRow(CategoryIndex inside, CategoryIndex aligned, CategoryIndex selfAligned, CategoryIndex selfReverseAligned, CategoryIndex reverseAligned, CategoryIndex outside)
        {
#if DEBUG_CATEGORIES
            destination = new Int4Array();
#endif
            destination[(int)CategoryIndex.Inside]              = (int)inside;
            destination[(int)CategoryIndex.Aligned]             = (int)aligned;
            destination[(int)CategoryIndex.SelfAligned]         = (int)selfAligned;
            destination[(int)CategoryIndex.SelfReverseAligned]  = (int)selfReverseAligned;
            destination[(int)CategoryIndex.ReverseAligned]      = (int)reverseAligned;
            destination[(int)CategoryIndex.Outside]             = (int)outside;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CategoryRoutingRow(CategoryGroupIndex inside, CategoryGroupIndex aligned, CategoryGroupIndex selfAligned, CategoryGroupIndex selfReverseAligned, CategoryGroupIndex reverseAligned, CategoryGroupIndex outside)
        {
#if DEBUG_CATEGORIES
            destination = new Int4Array();
#endif
            destination[(int)CategoryIndex.Inside]              = (int)inside;
            destination[(int)CategoryIndex.Aligned]             = (int)aligned;
            destination[(int)CategoryIndex.SelfAligned]         = (int)selfAligned;
            destination[(int)CategoryIndex.SelfReverseAligned]  = (int)selfReverseAligned;
            destination[(int)CategoryIndex.ReverseAligned]      = (int)reverseAligned;
            destination[(int)CategoryIndex.Outside]             = (int)outside;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CategoryRoutingRow(CategoryGroupIndex value)
        {
#if DEBUG_CATEGORIES
            destination = new Int4Array();
#endif
            destination[(int)CategoryIndex.Inside]              = (int)value;
            destination[(int)CategoryIndex.Aligned]             = (int)value;
            destination[(int)CategoryIndex.SelfAligned]         = (int)value;
            destination[(int)CategoryIndex.SelfReverseAligned]  = (int)value;
            destination[(int)CategoryIndex.ReverseAligned]      = (int)value;
            destination[(int)CategoryIndex.Outside]             = (int)value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AreAllTheSame()
        {
            for (var i = 1; i < Length; i++) { if (destination[i - 1] != destination[i]) return false; }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AreAllValue(int value)
        {
            for (var i = 0; i < Length; i++) { if (destination[i] != value) return false; }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(CategoryRoutingRow other)
        {
            for (var i = 0; i < Length; i++) { if (other.destination[i] != destination[i]) return false; }
            return true;
        }

        public CategoryGroupIndex this[CategoryIndex index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (CategoryGroupIndex)destination[(int)index]; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { destination[(int)index] = (int)value; }
        }

        public CategoryGroupIndex this[CategoryGroupIndex index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (CategoryGroupIndex)destination[(int)index]; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { destination[(int)index] = (int)value; }
        }

        public CategoryGroupIndex this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (CategoryGroupIndex)destination[index]; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { destination[index] = (int)value; }
        }

        public override string ToString()
        {
            var inside              = (int)destination[(int)CategoryIndex.Inside];
            var aligned             = (int)destination[(int)CategoryIndex.Aligned];
            var selfAligned         = (int)destination[(int)CategoryIndex.SelfAligned];
            var selfReverseAligned  = (int)destination[(int)CategoryIndex.SelfReverseAligned];
            var reverseAligned      = (int)destination[(int)CategoryIndex.ReverseAligned];
            var outside             = (int)destination[(int)CategoryIndex.Outside];

            return $"({inside}, {aligned}, {selfAligned}, {selfReverseAligned}, {reverseAligned}, {outside})";
        }
    }
#endif
        }