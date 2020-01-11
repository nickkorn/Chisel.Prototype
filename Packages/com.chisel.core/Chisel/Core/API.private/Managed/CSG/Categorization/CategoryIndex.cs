using System;

namespace Chisel.Core
{
#if USE_MANAGED_CSG_IMPLEMENTATION
    [Serializable]
    public enum CategoryIndex
    {
        None                = -1,
#if HAVE_SELF_CATEGORIES
        LastCategory        = 5,

        BrushSelfDefault    = SelfAligned,
        ValidCategory1      = SelfAligned,
        ValidCategory2      = SelfReverseAligned,

        Inside              = 0,
        Aligned             = 1,
        SelfAligned         = 2,
        SelfReverseAligned  = 3,
        ReverseAligned      = 4,
        Outside             = 5
#else
        LastCategory        = 3,

        BrushSelfDefault    = Aligned,
        ValidCategory1      = Aligned,
        ValidCategory2      = ReverseAligned,

        Inside              = 0,
        Aligned             = 1,
        ReverseAligned      = 2,
        Outside             = 3
#endif
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
