using System;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    [Serializable]
    public enum CategoryIndex
    {
        None                = -1,

        Inside              = 0,
        Aligned             = 1,
        SelfAligned         = 2,
        SelfReverseAligned  = 3,
        ReverseAligned      = 4,
        Outside             = 5,

        LastCategory        = 5
    };


    public enum EdgeCategory
    {
        None = -1,

        LastCategory        = 3,

        Inside              = 0,
        Aligned             = 1,
        ReverseAligned      = 2,
        Outside             = 3
    };
#endif
}
