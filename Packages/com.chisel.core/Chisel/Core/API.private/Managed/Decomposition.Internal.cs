using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Vector2 = UnityEngine.Vector2;
using Debug = UnityEngine.Debug;
using System.Linq;
using Unity.Mathematics;

namespace Chisel.Core
{
    partial class Decomposition
    {
#if USE_MANAGED_CSG_IMPLEMENTATION

        // TODO: propertly encapusulate this w/ license etc.

        // From https://github.com/craftworkgames/FarseerPhysics.Portable

        // From phed rev 36: http://code.google.com/p/phed/source/browse/trunk/Polygon.cpp

        /// <summary>
        /// Convex decomposition algorithm created by Mark Bayazit (http://mnbayazit.com/)
        /// 
        /// Properties:
        /// - Tries to decompose using polygons instead of triangles.
        /// - Tends to produce optimal results with low processing time.
        /// - Running time is O(nr), n = number of vertices, r = reflex vertices.
        /// - Does not support holes.
        /// 
        /// For more information about this algorithm, see http://mnbayazit.com/406/bayazit
        /// </summary>
        internal static class BayazitDecomposer
        {
            /// <summary>
            /// The maximum number of vertices on a convex polygon.
            /// </summary>
            const int   MaxPolygonVertices  = 8;
            const float Epsilon             = 1.192092896e-07f;

            /// <summary>
            /// Gets the signed area.
            /// If the area is less than 0, it indicates that the polygon is clockwise winded.
            /// </summary>
            /// <returns>The signed area</returns>
            static float GetSignedArea(List<float2> vertices)
            {
                //The simplest polygon which can exist in the Euclidean plane has 3 sides.
                if (vertices.Count < 3)
                    return 0;

                int i;
                float area = 0;

                for (i = 0; i < vertices.Count; i++)
                {
                    int j = (i + 1) % vertices.Count;

                    float2 vi = vertices[i];
                    float2 vj = vertices[j];

                    area += vi.x * vj.y;
                    area -= vi.y * vj.x;
                }
                area /= 2.0f;
                return area;
            }

            /// <summary>
            /// Indicates if the vertices are in counter clockwise order.
            /// Warning: If the area of the polygon is 0, it is unable to determine the winding.
            /// </summary>
            static bool IsCounterClockWise(List<float2> vertices)
            {
                //The simplest polygon which can exist in the Euclidean plane has 3 sides.
                if (vertices.Count < 3)
                    return false;

                return (GetSignedArea(vertices) > 0.0f);
            }

            /// <summary>
            /// Decompose the polygon into several smaller non-concave polygon.
            /// If the polygon is already convex, it will return the original polygon, unless it is over MaxPolygonVertices.
            /// </summary>
            public static float2[][] ConvexPartition(List<float2> vertices)
            {
                Debug.Assert(vertices.Count > 3);

                if (!IsCounterClockWise(vertices))
                    vertices.Reverse();

                return TriangulatePolygon(vertices);
            }

            private static float2[][] TriangulatePolygon(List<float2> vertices)
            {
                List<float2[]> list = new List<float2[]>();
                float2 lowerInt = new float2();
                float2 upperInt = new float2(); // intersection points
                int lowerIndex = 0, upperIndex = 0;
                List<float2> lowerPoly, upperPoly;

                for (int i = 0; i < vertices.Count; ++i)
                {
                    if (Reflex(i, vertices))
                    {
                        float upperDist;
                        float lowerDist = upperDist = float.MaxValue;
                        for (int j = 0; j < vertices.Count; ++j)
                        {
                            // if line intersects with an edge
                            float d;
                            float2 p;
                            if (Left(At(i - 1, vertices), At(i, vertices), At(j, vertices)) && RightOn(At(i - 1, vertices), At(i, vertices), At(j - 1, vertices)))
                            {
                                // find the point of intersection
                                p = LineIntersect(At(i - 1, vertices), At(i, vertices), At(j, vertices), At(j - 1, vertices));

                                if (Right(At(i + 1, vertices), At(i, vertices), p))
                                {
                                    // make sure it's inside the poly
                                    d = SquareDist(At(i, vertices), p);
                                    if (d < lowerDist)
                                    {
                                        // keep only the closest intersection
                                        lowerDist = d;
                                        lowerInt = p;
                                        lowerIndex = j;
                                    }
                                }
                            }

                            if (Left(At(i + 1, vertices), At(i, vertices), At(j + 1, vertices)) && RightOn(At(i + 1, vertices), At(i, vertices), At(j, vertices)))
                            {
                                p = LineIntersect(At(i + 1, vertices), At(i, vertices), At(j, vertices), At(j + 1, vertices));

                                if (Left(At(i - 1, vertices), At(i, vertices), p))
                                {
                                    d = SquareDist(At(i, vertices), p);
                                    if (d < upperDist)
                                    {
                                        upperDist = d;
                                        upperIndex = j;
                                        upperInt = p;
                                    }
                                }
                            }
                        }

                        // if there are no vertices to connect to, choose a point in the middle
                        if (lowerIndex == (upperIndex + 1) % vertices.Count)
                        {
                            float2 p = ((lowerInt + upperInt) / 2);

                            lowerPoly = Copy(i, upperIndex, vertices);
                            lowerPoly.Add(p);
                            upperPoly = Copy(lowerIndex, i, vertices);
                            upperPoly.Add(p);
                        }
                        else
                        {
                            double highestScore = 0, bestIndex = lowerIndex;
                            while (upperIndex < lowerIndex)
                                upperIndex += vertices.Count;

                            for (int j = lowerIndex; j <= upperIndex; ++j)
                            {
                                if (CanSee(i, j, vertices))
                                {
                                    double score = 1 / (SquareDist(At(i, vertices), At(j, vertices)) + 1);
                                    if (Reflex(j, vertices))
                                    {
                                        if (RightOn(At(j - 1, vertices), At(j, vertices), At(i, vertices)) && LeftOn(At(j + 1, vertices), At(j, vertices), At(i, vertices)))
                                            score += 3;
                                        else
                                            score += 2;
                                    }
                                    else
                                    {
                                        score += 1;
                                    }
                                    if (score > highestScore)
                                    {
                                        bestIndex = j;
                                        highestScore = score;
                                    }
                                }
                            }
                            lowerPoly = Copy(i, (int)bestIndex, vertices);
                            upperPoly = Copy((int)bestIndex, i, vertices);
                        }
                        list.AddRange(TriangulatePolygon(lowerPoly.ToList()));
                        list.AddRange(TriangulatePolygon(upperPoly.ToList()));
                        return list.ToArray();
                    }
                }

                // polygon is already convex
                if (vertices.Count > MaxPolygonVertices)
                {
                    lowerPoly = Copy(0, vertices.Count / 2, vertices);
                    upperPoly = Copy(vertices.Count / 2, 0, vertices);
                    list.AddRange(TriangulatePolygon(lowerPoly.ToList()));
                    list.AddRange(TriangulatePolygon(upperPoly.ToList()));
                } else
                {
                    if (vertices.Count > 3)
                        list.Add(vertices.ToArray());
                }

                return list.ToArray();
            }

            private static float2 At(int i, List<float2> vertices)
            {
                int s = vertices.Count;
                return vertices[i < 0 ? s - 1 - ((-i - 1) % s) : i % s];
            }

            private static List<float2> Copy(int i, int j, List<float2> vertices)
            {
                while (j < i)
                    j += vertices.Count;

                List<float2> p = new List<float2>(j);

                for (; i <= j; ++i)
                {
                    p.Add(At(i, vertices));
                }
                return p;
            }

            public static bool FloatEquals(float value1, float value2)
            {
                return Math.Abs(value1 - value2) <= Epsilon;
            }

            //From Mark Bayazit's convex decomposition algorithm
            static float2 LineIntersect(float2 p1, float2 p2, float2 q1, float2 q2)
            {
                float2 i = float2.zero;
                float a1 = p2.y - p1.y;
                float b1 = p1.x - p2.x;
                float c1 = a1 * p1.x + b1 * p1.y;
                float a2 = q2.y - q1.y;
                float b2 = q1.x - q2.x;
                float c2 = a2 * q1.x + b2 * q1.y;
                float det = a1 * b2 - a2 * b1;

                if (!FloatEquals(det, 0))
                {
                    // lines are not parallel
                    i.x = (b2 * c1 - b1 * c2) / det;
                    i.y = (a1 * c2 - a2 * c1) / det;
                }
                return i;
            }

            /// <summary>
            /// This method detects if two line segments (or lines) intersect,
            /// and, if so, the point of intersection. Use the <paramref name="firstIsSegment"/> and
            /// <paramref name="secondIsSegment"/> parameters to set whether the intersection point
            /// must be on the first and second line segments. Setting these
            /// both to true means you are doing a line-segment to line-segment
            /// intersection. Setting one of them to true means you are doing a
            /// line to line-segment intersection test, and so on.
            /// Note: If two line segments are coincident, then 
            /// no intersection is detected (there are actually
            /// infinite intersection points).
            /// Author: Jeremy Bell
            /// </summary>
            /// <param name="point1">The first point of the first line segment.</param>
            /// <param name="point2">The second point of the first line segment.</param>
            /// <param name="point3">The first point of the second line segment.</param>
            /// <param name="point4">The second point of the second line segment.</param>
            /// <param name="point">This is set to the intersection
            /// point if an intersection is detected.</param>
            /// <param name="firstIsSegment">Set this to true to require that the 
            /// intersection point be on the first line segment.</param>
            /// <param name="secondIsSegment">Set this to true to require that the
            /// intersection point be on the second line segment.</param>
            /// <returns>True if an intersection is detected, false otherwise.</returns>
            static bool LineIntersect(ref float2 point1, ref float2 point2, ref float2 point3, ref float2 point4, bool firstIsSegment, bool secondIsSegment, out float2 point)
            {
                point = new float2();

                // these are reused later.
                // each lettered sub-calculation is used twice, except
                // for b and d, which are used 3 times
                float a = point4.y - point3.y;
                float b = point2.x - point1.x;
                float c = point4.x - point3.x;
                float d = point2.y - point1.y;

                // denominator to solution of linear system
                float denom = (a * b) - (c * d);

                // if denominator is 0, then lines are parallel
                if (!(denom >= -Epsilon && denom <= Epsilon))
                {
                    float e = point1.y - point3.y;
                    float f = point1.x - point3.x;
                    float oneOverDenom = 1.0f / denom;

                    // numerator of first equation
                    float ua = (c * e) - (a * f);
                    ua *= oneOverDenom;

                    // check if intersection point of the two lines is on line segment 1
                    if (!firstIsSegment || ua >= 0.0f && ua <= 1.0f)
                    {
                        // numerator of second equation
                        float ub = (b * e) - (d * f);
                        ub *= oneOverDenom;

                        // check if intersection point of the two lines is on line segment 2
                        // means the line segments intersect, since we know it is on
                        // segment 1 as well.
                        if (!secondIsSegment || ub >= 0.0f && ub <= 1.0f)
                        {
                            // check if they are coincident (no collision in this case)
                            if (ua != 0f || ub != 0f)
                            {
                                //There is an intersection
                                point.x = point1.x + ua * b;
                                point.y = point1.y + ua * d;
                                return true;
                            }
                        }
                    }
                }

                return false;
            }

            /// <summary>
            /// This method detects if two line segments intersect,
            /// and, if so, the point of intersection. 
            /// Note: If two line segments are coincident, then 
            /// no intersection is detected (there are actually
            /// infinite intersection points).
            /// </summary>
            /// <param name="point1">The first point of the first line segment.</param>
            /// <param name="point2">The second point of the first line segment.</param>
            /// <param name="point3">The first point of the second line segment.</param>
            /// <param name="point4">The second point of the second line segment.</param>
            /// <param name="intersectionPoint">This is set to the intersection
            /// point if an intersection is detected.</param>
            /// <returns>True if an intersection is detected, false otherwise.</returns>
            static bool LineIntersect(float2 point1, float2 point2, float2 point3, float2 point4, out float2 intersectionPoint)
            {
                return LineIntersect(ref point1, ref point2, ref point3, ref point4, true, true, out intersectionPoint);
            }


            private static bool CanSee(int i, int j, List<float2> vertices)
            {
                if (Reflex(i, vertices))
                {
                    if (LeftOn(At(i, vertices), At(i - 1, vertices), At(j, vertices)) && RightOn(At(i, vertices), At(i + 1, vertices), At(j, vertices)))
                        return false;
                }
                else
                {
                    if (RightOn(At(i, vertices), At(i + 1, vertices), At(j, vertices)) || LeftOn(At(i, vertices), At(i - 1, vertices), At(j, vertices)))
                        return false;
                }
                if (Reflex(j, vertices))
                {
                    if (LeftOn(At(j, vertices), At(j - 1, vertices), At(i, vertices)) && RightOn(At(j, vertices), At(j + 1, vertices), At(i, vertices)))
                        return false;
                }
                else
                {
                    if (RightOn(At(j, vertices), At(j + 1, vertices), At(i, vertices)) || LeftOn(At(j, vertices), At(j - 1, vertices), At(i, vertices)))
                        return false;
                }
                for (int k = 0; k < vertices.Count; ++k)
                {
                    if ((k + 1) % vertices.Count == i || k == i || (k + 1) % vertices.Count == j || k == j)
                        continue; // ignore incident edges

                    float2 intersectionPoint;

                    if (LineIntersect(At(i, vertices), At(j, vertices), At(k, vertices), At(k + 1, vertices), out intersectionPoint))
                        return false;
                }
                return true;
            }

            private static bool Reflex(int i, List<float2> vertices)
            {
                return Right(i, vertices);
            }

            private static bool Right(int i, List<float2> vertices)
            {
                return Right(At(i - 1, vertices), At(i, vertices), At(i + 1, vertices));
            }

            /// <summary>
            /// Returns a positive number if c is to the left of the line going from a to b.
            /// </summary>
            /// <returns>Positive number if point is left, negative if point is right, 
            /// and 0 if points are collinear.</returns>
            public static float Area(float2 a, float2 b, float2 c)
            {
                return Area(ref a, ref b, ref c);
            }

            /// <summary>
            /// Returns a positive number if c is to the left of the line going from a to b.
            /// </summary>
            /// <returns>Positive number if point is left, negative if point is right, 
            /// and 0 if points are collinear.</returns>
            public static float Area(ref float2 a, ref float2 b, ref float2 c)
            {
                return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y);
            }

            private static bool Left(float2 a, float2 b, float2 c)
            {
                return Area(ref a, ref b, ref c) > 0;
            }

            private static bool LeftOn(float2 a, float2 b, float2 c)
            {
                return Area(ref a, ref b, ref c) >= 0;
            }

            private static bool Right(float2 a, float2 b, float2 c)
            {
                return Area(ref a, ref b, ref c) < 0;
            }

            private static bool RightOn(float2 a, float2 b, float2 c)
            {
                return Area(ref a, ref b, ref c) <= 0;
            }

            private static float SquareDist(float2 a, float2 b)
            {
                float dx = b.x - a.x;
                float dy = b.y - a.y;
                return dx * dx + dy * dy;
            }
        }


        static float2[][] ConvexPartitionInternal(float2[] inputVertices2D)
        {
            var output = BayazitDecomposer.ConvexPartition(inputVertices2D.ToList());
            //for (int i = 0; i < output.Length; i++) output[i].Reverse();
            return output;
        }
#endif
    }
}
