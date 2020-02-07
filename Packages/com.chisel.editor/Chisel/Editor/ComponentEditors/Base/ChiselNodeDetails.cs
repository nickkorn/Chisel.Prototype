using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System;
using System.Linq;
using System.Collections.Generic;
using Chisel;
using System.Reflection;
using Chisel.Core;
using Chisel.Components;

namespace Chisel.Editors
{
    public interface IChiselNodeDetails
    {
        GUIContent GetHierarchyIconForGenericNode(ChiselNode node);
    }

    public abstract class ChiselNodeDetails<T> : IChiselNodeDetails
        where T : ChiselNode
    {
        GUIContent IChiselNodeDetails.GetHierarchyIconForGenericNode(ChiselNode node) { return GetHierarchyIcon((T)node); }

        public abstract GUIContent GetHierarchyIcon(T node);
    }

    public abstract class ChiselGeneratorDetails<T> : ChiselNodeDetails<T>
        where T : ChiselGeneratorComponent
    {
        const string kAdditiveIconName          = "csg_" + nameof(CSGOperationType.Additive);
        const string kSubtractiveIconName       = "csg_" + nameof(CSGOperationType.Subtractive);
        const string kIntersectingIconName      = "csg_" + nameof(CSGOperationType.Intersecting);
        const string kCopyIconName              = "csg_" + nameof(CSGOperationType.Copy);

        public override GUIContent GetHierarchyIcon(T node)
        {
            switch (node.Operation)
            {
                default:
                case CSGOperationType.Additive:         return ChiselEditorResources.GetIconContent(kAdditiveIconName,     $"Additive {node.NodeTypeName}")[0];
                case CSGOperationType.Subtractive:      return ChiselEditorResources.GetIconContent(kSubtractiveIconName,  $"Subtractive {node.NodeTypeName}")[0];
                case CSGOperationType.Intersecting:     return ChiselEditorResources.GetIconContent(kIntersectingIconName, $"Intersecting {node.NodeTypeName}")[0];
                case CSGOperationType.Copy:             return ChiselEditorResources.GetIconContent(kCopyIconName,         $"Copy {node.NodeTypeName}")[0];
            }
        }
    }
}
