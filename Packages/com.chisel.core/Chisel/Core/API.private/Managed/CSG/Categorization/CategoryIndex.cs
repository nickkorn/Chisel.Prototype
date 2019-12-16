using System;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    [Serializable]
    public enum CategoryIndex
    {
        None            = -1,
        LastCategory    = 3,

        Inside          = 0,
        Aligned         = 1,
        ReverseAligned  = 2,
        Outside         = 3

    };
#endif
}
