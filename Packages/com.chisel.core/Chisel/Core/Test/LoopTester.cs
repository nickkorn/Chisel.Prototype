using Chisel.Core;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

#if USE_MANAGED_CSG_IMPLEMENTATION
// TODO: put in separate package
namespace LoopTests
{
#if false
    [Serializable]
    public sealed class LoopContainer
    {
        public Chisel.Core.Loop loop;
        [NonSerialized] public int brushIndex;
        public bool intersect = true;
    }

    public class LoopTester : MonoBehaviour
    {
        public CSGOperationType operation = CSGOperationType.Subtractive;

        public LoopContainer[] baseLoops;


        public struct LoopDependency
        {
            public int      startIndex;
            public int[]    dependencies;
        }
        
        static void FillDependencyList(List<int> dependencies, int baseIndex, HashSet<int> skipSet, int[][] intersections)
        {
            if (dependencies.Count == intersections.Length)
                return;

            var baseIntersections = intersections[baseIndex];
            if (baseIntersections != null)
            {
                var startIndex = dependencies.Count;
                for (int i = 0; i < baseIntersections.Length; i++)
                {
                    var loopIndex = baseIntersections[i];
                    if (!skipSet.Add(loopIndex))
                        continue;

                    dependencies.Add(loopIndex);
                }

                // TODO: make this non-recursive
                // We add the child dependencies later, to ensure the order of the entire list is correct
                var endIndex = dependencies.Count;
                for (int i = startIndex; i < endIndex; i++)
                    FillDependencyList(dependencies, dependencies[i], skipSet, intersections);
            }
        }

        public static List<Loop> GeneratedFinalLoops(LoopTester visualizer)
        {
            var baseLoops = visualizer.baseLoops;
            if (visualizer.baseLoops == null)
                return null;


            var plane = new Plane(Vector3.up, 0);
            var generatedLoops = new List<Loop>();
            for (int i = 0; i < baseLoops.Length; i++)
            {
                baseLoops[i].loop.info = new LoopInfo
                {
                    brushNodeID = i,
                    basePlaneIndex = i,
                    worldPlane = plane,
                    localPlane = plane
                };

                if (!baseLoops[i].intersect ||
                    baseLoops[i].loop.vertices.Count == 0)
                    continue;

                baseLoops[i].loop.CalcBounds();

                var newLoop     = new Loop(baseLoops[i].loop);
                var realPlane   = baseLoops[i].loop.CalculatePlane();
                if (Vector3.Dot(realPlane.normal, plane.normal) < 0)
                    newLoop.vertices.Reverse();


                //newLoop.vertices.Reverse();

                generatedLoops.Add(newLoop);
            }

            {
                // Finds dependencies between loops
                var loopDependencies = new List<LoopDependency>();
                {
                    // TODO: optimize
                    // Finds all the intersections between all polygons
                    int[][] intersections = new int[generatedLoops.Count][];
                    for (int a = 0; a < generatedLoops.Count; a++)
                    {
                        var foundIntersections = new List<int>();
                        for (int b = 0; b < generatedLoops.Count; b++)
                        {
                            if (a == b)
                                continue;

                            if (visualizer.operation == CSGOperationType.Intersecting)
                            {
                                // When we're intersecting then dependencies are two way
                                if (Loop.Intersects(generatedLoops[a], generatedLoops[b]) ||
                                    Loop.Intersects(generatedLoops[b], generatedLoops[a]))
                                    foundIntersections.Add(b);
                            } else
                            {
                                if (Loop.Intersects(generatedLoops[a], generatedLoops[b]))
                                    foundIntersections.Add(b);
                            }
                        }
                        intersections[a] = foundIntersections.ToArray();
                    }


                    // Creates a list of dependencies, in order
                    var skipSet         = new HashSet<int>();
                    var dependencies    = new List<int>();
                    for (int a = 0; a < generatedLoops.Count; a++)
                    {
                        dependencies.Clear();

                        skipSet.Clear();
                        skipSet.Add(a);

                        FillDependencyList(dependencies, a, skipSet, intersections);

                        loopDependencies.Add(new LoopDependency()
                        {
                            startIndex = 0,
                            dependencies = dependencies.ToArray()
                        });

                        //Debug.Log($"Dependencies[{a}]: {dependencies.Count}");
                    }
                }

                {
                    var resultLoops = new List<Loop>();

                    int a = 0;
                    while (true)
                    {
#if DEBUG
                        if (loopDependencies.Count > generatedLoops.Count * generatedLoops.Count * 2) { Debug.Log("Infinite loop bug"); break; }
#endif

                        if (a >= loopDependencies.Count ||
                            a >= generatedLoops.Count)
                            break;

                        if (generatedLoops[a].info.brushNodeID != 0)
                        {
                            a++;
                            continue;
                        }

                        var loopDependency  = loopDependencies[a];
                        var dependencies    = loopDependency.dependencies;

                        if (visualizer.operation == CSGOperationType.Intersecting &&
                            a == 0 &&
                            loopDependency.startIndex >= dependencies.Length)
                            generatedLoops[a].vertices.Clear();

                        for (; loopDependency.startIndex < dependencies.Length; loopDependency.startIndex++)
                        { 
                            var b = dependencies[loopDependency.startIndex];
                            //if (!Loop.Intersects(generatedLoops[a], generatedLoops[b]))
                            //    continue;

                            if (visualizer.operation == CSGOperationType.Subtractive)
                            {
                                var result = Loop.PerformBooleanOperation(generatedLoops[a], generatedLoops[b], resultLoops, CSGOperationType.Subtractive);
                                //Debug.Log($"Subtract[{a}/{b}]: {result} {resultLoops.Count}  {generatedLoops[a].vertices.Count}  {generatedLoops[b].vertices.Count}");
                                switch (result)
                                {
                                    case Loop.OperationResult.Outside:
                                    case Loop.OperationResult.Fail:                     continue;
                                    case Loop.OperationResult.Polygon2InsidePolygon1:
                                    {
                                        var loop = new Loop(generatedLoops[a]);
                                        loop.holes.Add(new Loop(generatedLoops[b]));
                                        resultLoops.Clear(); resultLoops.Add(loop);
                                        break;
                                    }
                                    case Loop.OperationResult.Overlapping:              
                                    case Loop.OperationResult.Polygon1InsidePolygon2:   { resultLoops.Clear(); break; }
                                }
                            } else
                            if (visualizer.operation == CSGOperationType.Additive)
                            {
                                var result = Loop.PerformBooleanOperation(generatedLoops[a], generatedLoops[b], resultLoops, CSGOperationType.Additive);
                                //Debug.Log($"Add: {result} {resultLoops.Count}  {generatedLoops[a].vertices.Count}  {generatedLoops[b].vertices.Count}");
                                switch (result)
                                {
                                    case Loop.OperationResult.Outside:
                                    case Loop.OperationResult.Fail:                     continue;
                                    case Loop.OperationResult.Polygon2InsidePolygon1:   continue;
                                    case Loop.OperationResult.Overlapping:              
                                    case Loop.OperationResult.Polygon1InsidePolygon2:
                                    {
                                        resultLoops.Clear();
                                        resultLoops.Add(new Loop(generatedLoops[b]));
                                        break;
                                    }
                                }
                            } else 
                            if (visualizer.operation == CSGOperationType.Intersecting)
                            {
                                var result = Loop.PerformBooleanOperation(generatedLoops[a], generatedLoops[b], resultLoops, CSGOperationType.Intersecting);
                                //Debug.Log($"Intersect: {result} {resultLoops.Count}  {generatedLoops[a].vertices.Count}  {generatedLoops[b].vertices.Count}");
                                switch (result)
                                {
                                    case Loop.OperationResult.Outside:                  { resultLoops.Clear(); break; }
                                    case Loop.OperationResult.Fail:                     continue;
                                    case Loop.OperationResult.Polygon2InsidePolygon1:   { resultLoops.Clear(); resultLoops.Add(new Loop(generatedLoops[b])); break; }
                                    case Loop.OperationResult.Overlapping:              
                                    case Loop.OperationResult.Polygon1InsidePolygon2:   { resultLoops.Clear(); resultLoops.Add(new Loop(generatedLoops[a])); break; }
                                }
                            } else 
                                return null;

                            generatedLoops[a].vertices.Clear();
                            loopDependency.startIndex++;

                            if (resultLoops == null ||
                                resultLoops.Count == 0)
                                break;

                            for (int n = 0; n < resultLoops.Count; n++)
                            {
                                resultLoops[n].CalcBounds();
                                loopDependencies.Add(loopDependency);
                                resultLoops[n].CopyDetails(generatedLoops[a]);
                                //break;
                            }

                            // TODO: check winding, if the winding is reversed it's a hole, not a new polygon

                            for (int i = 0; i < resultLoops.Count; i++)
                            {
                                if (resultLoops[i].info.brushNodeID != 0 ||
                                    resultLoops[i].vertices.Count == 0)
                                    continue;
                                generatedLoops.Add(resultLoops[i]);
                            }

                            break;
                        }

                        a++;
                    }
                }
            }

            for (int i = generatedLoops.Count - 1; i >= 0; i--)
            {
                if (generatedLoops[i].info.brushNodeID != 0 ||
                    generatedLoops[i].vertices.Count == 0)
                    generatedLoops.RemoveAt(i);
            }

            return generatedLoops;
        }
    }
#endif
}
#endif