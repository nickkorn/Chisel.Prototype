using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using Chisel;
using UnityEditor.SceneManagement;
using Chisel.Core;
using System;
using System.Collections.Generic;

#if USE_MANAGED_CSG_IMPLEMENTATION
namespace LoopTests
{
#if false
    [TestFixture]
    public partial class AddLoopTests
    {
        const string sourceFilename         = @"Packages\com.chisel.core.tests\Chisel\Core\Tests\LoopTests\Additive\{0}.prefab";

        struct CaseDetails
        {
            public string sceneName;
            public int[] expectedVertexCounts;
        }

        static CaseDetails[] cases = new[]
        {
            new CaseDetails() { sceneName = "case_0",  expectedVertexCounts = new [] { 4 } },
            new CaseDetails() { sceneName = "case_1",  expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_2",  expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_3",  expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_4",  expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_5",  expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_6",  expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_7",  expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_8",  expectedVertexCounts = new [] { 7 } },
            new CaseDetails() { sceneName = "case_9",  expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_10", expectedVertexCounts = new [] { 6 } },
            new CaseDetails() { sceneName = "case_11", expectedVertexCounts = new [] { 7 } },
            new CaseDetails() { sceneName = "case_12", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_13", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_14", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_15", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_16", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_17", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_18", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_19", expectedVertexCounts = new [] { 7 } },
            new CaseDetails() { sceneName = "case_20", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_21", expectedVertexCounts = new [] { 12 } },
            new CaseDetails() { sceneName = "case_22", expectedVertexCounts = new [] { 4 } },
            new CaseDetails() { sceneName = "case_23", expectedVertexCounts = new [] { 12 } },
            new CaseDetails() { sceneName = "case_24", expectedVertexCounts = new [] { 12 } },
            new CaseDetails() { sceneName = "case_25", expectedVertexCounts = new [] { 10, 3 } },
            new CaseDetails() { sceneName = "case_26", expectedVertexCounts = new [] { 10, 3 } },
            new CaseDetails() { sceneName = "case_27", expectedVertexCounts = new [] { 13 } },
            new CaseDetails() { sceneName = "case_28", expectedVertexCounts = new [] { 11 } },
            new CaseDetails() { sceneName = "case_29", expectedVertexCounts = new [] { 4 } },
            new CaseDetails() { sceneName = "case_30", expectedVertexCounts = new [] { 8 } },
            new CaseDetails() { sceneName = "case_31", expectedVertexCounts = new [] { 4 } },
            new CaseDetails() { sceneName = "case_32", expectedVertexCounts = new [] { 4 } }
        };

        private List<Loop> PerformCase(int index)
        {
            var name = cases[index].sceneName;
            //Debug.Log("Case index " + index + " " + name);
            var srcFile = String.Format(sourceFilename, name);
            Assert.IsTrue(System.IO.File.Exists(srcFile));
            var prefabObject    = AssetDatabase.LoadAssetAtPath(srcFile, typeof(UnityEngine.Object));
            Assert.IsTrue(prefabObject);
            var instance        = PrefabUtility.InstantiatePrefab(prefabObject);
            Assert.IsTrue(instance);
            var loopTest        = instance as GameObject;
            Assert.IsTrue(loopTest);
            Assert.IsTrue(loopTest);
            var tester = loopTest.GetComponent<LoopTests.LoopTester>();
            var generatedLoops = LoopTester.GeneratedFinalLoops(tester);
            Assert.AreNotEqual(null, generatedLoops);
            Assert.AreEqual(cases[index].expectedVertexCounts.Length, generatedLoops.Count);
            for (int i = 0; i < cases[index].expectedVertexCounts.Length; i++)
                Assert.AreEqual(cases[index].expectedVertexCounts[i], generatedLoops[i].vertices.Count);
            UnityEngine.Object.DestroyImmediate(loopTest);
            return generatedLoops;
        }


        [Test] public void TestScene_case_0() { PerformCase(0); }
        [Test] public void TestScene_case_1() { PerformCase(1); }
        [Test] public void TestScene_case_2() { PerformCase(2); }
        [Test] public void TestScene_case_3() { PerformCase(3); }
        [Test] public void TestScene_case_4() { PerformCase(4); }
        [Test] public void TestScene_case_5() { PerformCase(5); }
        [Test] public void TestScene_case_6() { PerformCase(6); }
        [Test] public void TestScene_case_7() { PerformCase(7); }
        [Test] public void TestScene_case_8() { PerformCase(8); }
        [Test] public void TestScene_case_9() { PerformCase(9); }
        [Test] public void TestScene_case_10() { PerformCase(10); }
        [Test] public void TestScene_case_11() { PerformCase(11); }
        [Test] public void TestScene_case_12() { PerformCase(12); }
        [Test] public void TestScene_case_13() { PerformCase(13); }
        [Test] public void TestScene_case_14() { PerformCase(14); }
        [Test] public void TestScene_case_15() { PerformCase(15); }
        [Test] public void TestScene_case_16() { PerformCase(16); }
        [Test] public void TestScene_case_17() { PerformCase(17); }
        [Test] public void TestScene_case_18() { PerformCase(18); }
        [Test] public void TestScene_case_19() { PerformCase(19); }
        [Test] public void TestScene_case_20() { PerformCase(20); }
        [Test] public void TestScene_case_21() { PerformCase(21); }
        [Test] public void TestScene_case_22() { PerformCase(22); }
        [Test] public void TestScene_case_23() { PerformCase(23); }
        [Test] public void TestScene_case_24() { PerformCase(24); }
        [Test] public void TestScene_case_25() { PerformCase(25); }
        [Test] public void TestScene_case_26() { PerformCase(26); }
        [Test] public void TestScene_case_27() { PerformCase(27); }
        [Test] public void TestScene_case_28() { PerformCase(28); }
        [Test] public void TestScene_case_29() { PerformCase(29); }
        [Test] public void TestScene_case_30() { PerformCase(30); }
        [Test] public void TestScene_case_31() { PerformCase(31); }
        [Test] public void TestScene_case_32() { var tester = PerformCase(32); tester[0].CalcBounds(); Assert.AreEqual(new Vector3(3, 0, 7), tester[0].bounds.extents); }
    }
#endif
}
#endif