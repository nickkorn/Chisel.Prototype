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
    public struct SurfaceInfo
    {
        public float4               worldPlane;
        public SurfaceLayers        layers;
        public int                  basePlaneIndex;
        public int                  brushNodeIndex;
        public CategoryGroupIndex   interiorCategory;
    }

    struct PlaneVertexIndexPair : IComparable<PlaneVertexIndexPair>
    {
        public ushort planeIndex;
        public ushort vertexIndex;

        public int CompareTo(PlaneVertexIndexPair other)
        {
            if (planeIndex > other.planeIndex)
                return 1;
            if (planeIndex == other.planeIndex)
            {
                if (vertexIndex == other.vertexIndex)
                    return 0;
                if (vertexIndex < other.vertexIndex)
                    return 1;
            }
            return -1;
        }
    }

    struct PlaneIndexOffsetLength : IComparable<PlaneIndexOffsetLength>
    {
        public ushort length;
        public ushort offset;
        public ushort planeIndex;

        public int CompareTo(PlaneIndexOffsetLength other)
        {
            if (planeIndex < other.planeIndex)
                return -1;
            if (planeIndex > other.planeIndex)
                return 1;
            if (offset < other.offset)
                return -1;
            if (offset > other.offset)
                return 1;
            if (length < other.length)
                return -1;
            if (length > other.length)
                return 1;
            return 0;
        }
    }

    public struct PlanePair
    {
        public float4 plane0;
        public float4 plane1;
        //public double4 N0;
        //public double4 N1;
        public float4 edgeVertex0;
        public float4 edgeVertex1;
        public int planeIndex0;
        public int planeIndex1;
    }
    
    public struct BrushSurfacePair : IEquatable<BrushSurfacePair>
    {
        public int brushNodeIndex0;
        public int brushNodeIndex1;
        public int basePlaneIndex;

        #region Equals
        public bool Equals(BrushSurfacePair other)
        {
            return (brushNodeIndex0 == other.brushNodeIndex0 && brushNodeIndex1 == other.brushNodeIndex1 && basePlaneIndex == other.basePlaneIndex);
        }

        public override bool Equals(object obj)
        {
            if (!(obj is BrushSurfacePair))
                return false;
            return base.Equals((BrushSurfacePair)obj);
        }
        #endregion

        public override int GetHashCode()
        {
            return (int)math.hash(new int3(brushNodeIndex0, brushNodeIndex1, basePlaneIndex));
        }
    }
}