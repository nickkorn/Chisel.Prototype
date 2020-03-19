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
    static partial class CSGManagerPerformCSG
    {
        // TODO: unify all epsilons
        public const float  kDistanceEpsilon	    = 0.0001f;//0.01f;
        public const float  kSqrDistanceEpsilon	    = kDistanceEpsilon * kDistanceEpsilon;
        public const float  kMergeEpsilon	        = 0.0005f;
        public const float  kSqrMergeEpsilon	    = kMergeEpsilon * kMergeEpsilon;
        public const float  kNormalEpsilon			= 0.9999f;
        public const float  kPlaneDistanceEpsilon	= 0.0006f;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool IsDegenerate(in VertexSoup soup, NativeList<Edge> edges)
        {
            if (edges.Length < 3)
                return true;

            var vertices = soup.vertices;
            for (int i = 0; i < edges.Length; i++)
            {
                var vertexIndex1 = edges[i].index1;
                var vertex1      = vertices[vertexIndex1];
                for (int j = 0; j < edges.Length; j++)
                {
                    if (i == j)
                        continue;

                    var vertexIndexA = edges[j].index1;
                    var vertexIndexB = edges[j].index2;

                    // Loop loops back on same vertex
                    if (vertexIndex1 == vertexIndexA || 
                        vertexIndex1 == vertexIndexB ||
                        vertexIndexA == vertexIndexB)
                        continue;

                    var vertexA = vertices[vertexIndexA];
                    var vertexB = vertices[vertexIndexB];

                    var distance = GeometryMath.SqrDistanceFromPointToLineSegment(vertex1, vertexA, vertexB);
                    if (distance <= CSGManagerPerformCSG.kSqrDistanceEpsilon)
                        return true;
                }
            }
            return false;
        }
    }
#endif
}
