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
using System.Globalization;
using System.Runtime.CompilerServices;
using Unity.Entities;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    static partial class CSGManagerPerformCSG
    {
        #region PerformBooleanOperation

        public const double kEpsilon = 0.00001;
        public const double kEpsilon2 = 0.001;
        public const double kEpsilonSqr = kEpsilon * kEpsilon;
        #endregion

        #region PerformCSG


        //*
        internal static unsafe void Dump(System.Text.StringBuilder builder, NativeList<Edge> categorized_loop, in VertexSoup soup, Quaternion rotation)
        {
            //builder.AppendLine($"loop ({categorized_loop.indices.Count}):");
            //builder.AppendLine($"loop {categorized_loop.loopIndex}:");
            /*
            for (int i = 0; i < categorized_loop.indices.Count; i++)
            {
                if (i > 0)
                    builder.Append(",");
                var index = categorized_loop.indices[i];

                builder.Append($"{index}");
            }
            builder.AppendLine();

            for (int i = 0; i < categorized_loop.indices.Count; i++)
            {
                var index = categorized_loop.indices[i];
                var vertex = ((float3)(rotation * vertices[index])).xy;

                builder.Append($"({Convert.ToString((Decimal)vertex.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex.y, CultureInfo.InvariantCulture)})");
                builder.Append(", ");
            }
            if (categorized_loop.indices.Count > 0)
            { 
                var index = categorized_loop.indices[0];
                var vertex = ((float3)(rotation * vertices[index])).xy;
                builder.Append($"({Convert.ToString((Decimal)vertex.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex.y, CultureInfo.InvariantCulture)})");
            }
            builder.AppendLine("             ");
            */

            var vertices = soup.GetUnsafeReadOnlyPtr();
            for (int i = 0; i < categorized_loop.Length; i++)
            {
                var edge = categorized_loop[i];
                var index1 = edge.index1;
                var index2 = edge.index2;
                var vertex1 = ((float3)(rotation * vertices[index1])).xy;
                var vertex2 = ((float3)(rotation * vertices[index2])).xy;

                builder.Append($"({Convert.ToString((Decimal)vertex1.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex1.y, CultureInfo.InvariantCulture)}), ");
                builder.AppendLine($"({Convert.ToString((Decimal)vertex2.x, CultureInfo.InvariantCulture)}, {Convert.ToString((Decimal)vertex2.y, CultureInfo.InvariantCulture)})             ");
            }
        }
        //*/
        #endregion

        #region CleanUp
        internal static unsafe float3 CalculatePlaneEdges(in NativeListArray<Edge>.NativeList edges, in VertexSoup soup)
        {
            // Newell's algorithm to create a plane for concave polygons.
            // NOTE: doesn't work well for self-intersecting polygons
            var normal = Vector3.zero;
            var vertices = soup.GetUnsafeReadOnlyPtr();
            for (int n = 0; n < edges.Length; n++)
            {
                var edge = edges[n];
                var prevVertex = vertices[edge.index1];
                var currVertex = vertices[edge.index2];
                normal.x = normal.x + ((prevVertex.y - currVertex.y) * (prevVertex.z + currVertex.z));
                normal.y = normal.y + ((prevVertex.z - currVertex.z) * (prevVertex.x + currVertex.x));
                normal.z = normal.z + ((prevVertex.x - currVertex.x) * (prevVertex.y + currVertex.y));
            }
            normal = normal.normalized;

            return normal;
        }

#endregion
    }
#endif
}
