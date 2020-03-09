#define DO_PROFILING
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
    public struct ProfileSample : IDisposable
    {
        public ProfileSample(string name) 
        {
#if DO_PROFILING
            UnityEngine.Profiling.Profiler.BeginSample(name);
#endif
        }
        public void Dispose()
        {
#if DO_PROFILING
            UnityEngine.Profiling.Profiler.EndSample();
#endif
        }
    }
}
