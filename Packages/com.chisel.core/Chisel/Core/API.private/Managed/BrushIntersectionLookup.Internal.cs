﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    internal class BrushIntersectionLookup
    {
        const int bits = 2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BrushIntersectionLookup(int _length)
        {
            var size = ((_length * bits) + 31) / 32;
            if (size <= 0)
                this.twoBits = new UInt32[0];
            else
                this.twoBits = new UInt32[size];
        }

        readonly UInt32[]  twoBits;

        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (twoBits.Length * 32) / bits; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            Array.Clear(twoBits, 0, twoBits.Length);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public IntersectionType Get(int index)
        {
            if (index < 0 || index >= Length)
                return IntersectionType.InvalidValue;
                
            index <<= 1;
            var int32Index = index >> 5;	// divide by 32
            var bitIndex   = index & 31;	// remainder
            var twoBit     = ((UInt32)3) << bitIndex;

            return (IntersectionType) ((twoBits[int32Index] & twoBit) >> bitIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Is(int index, IntersectionType value) 
        {
            if (index < 0 || index >= Length)
				return false;

            index <<= 1;
            var int32Index   = index >> 5;	// divide by 32
            var bitIndex     = index & 31;	// remainder
            var twoBit       = ((UInt32)3) << bitIndex;
            var twoBitValue  = ((UInt32)value) << bitIndex;

            var originalInt32 = twoBits[int32Index];
                
            return (originalInt32 & twoBit) == twoBitValue;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Set(int index, IntersectionType value)
        {
            if (index < 0 || index >= Length)
                return;

            index <<= 1;
            var int32Index   = index >> 5;	// divide by 32
            var bitIndex     = index & 31;	// remainder
            var twoBit       = (UInt32)3 << bitIndex;
            var twoBitValue  = ((UInt32)value) << bitIndex;

            var originalInt32 = twoBits[int32Index];

            twoBits[int32Index] = (originalInt32 & ~twoBit) | twoBitValue;
        }
    };
#endif
}
