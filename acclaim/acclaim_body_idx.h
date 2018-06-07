#ifndef _ACCLAIM_BODY_IDX_H_
#define _ACCLAIM_BODY_IDX_H_

#include <map>
#include <string>
#include <iostream>

typedef enum Body: int
{
    root = 0,  // = 0
    lhipjoint, // = 1
    lfemur,    // = 2
    ltibia,    // = 3
    lfoot,     // = 4
    ltoes,     // = 5
    rhipjoint, // = 6
    rfemur,    // = 7
    rtibia,    // = 8
    rfoot,     // = 9
    rtoes,     // = 10
    lowerback, // = 11
    upperback, // = 12
    thorax,    // = 13
    lowerneck, // = 14
    upperneck, // = 15
    head,      // = 16
    lclavicle, // = 17
    lhumerus,  // = 18
    lradius,   // = 19
    lwrist,    // = 20
    lhand,     // = 21
    lfingers,  // = 22
    lthumb,    // = 23
    rclavicle, // = 24
    rhumerus,  // = 25
    rradius,   // = 26
    rwrist,    // = 27
    rhand,     // = 28
    rfingers,  // = 29
    rthumb,    // = 30
    num        // = 31
} BoneIdx_t;

template<typename charT, typename traits>
std::basic_ostream<charT, traits> &operator<<(
        std::basic_ostream<charT, traits> &ostream,
        const BoneIdx_t &bone_idx
        )
{
    static const std::map<BoneIdx_t, std::string> kIdxNameMap =
    {
        {root,      "root"     },
        {lhipjoint, "lhipjoint"},
        {lfemur,    "lfemur"   },
        {ltibia,    "ltibia"   },
        {lfoot,     "lfoot"    },
        {ltoes,     "ltoes"    },
        {rhipjoint, "rhipjoint"},
        {rfemur,    "rfemur"   },
        {rtibia,    "rtibia"   },
        {rfoot,     "rfoot"    },
        {rtoes,     "rtoes"    },
        {lowerback, "lowerback"},
        {upperback, "upperback"},
        {thorax,    "thorax"   },
        {lowerneck, "lowerneck"},
        {upperneck, "upperneck"},
        {head,      "head"     },
        {lclavicle, "lclavicle"},
        {lhumerus,  "lhumerus" },
        {lradius,   "lradius"  },
        {lwrist,    "lwrist"   },
        {lhand,     "lhand"    },
        {lfingers,  "lfingers" },
        {lthumb,    "lthumb"   },
        {rclavicle, "rclavicle"},
        {rhumerus,  "rhumerus" },
        {rradius,   "rradius"  },
        {rwrist,    "rwrist"   },
        {rhand,     "rhand"    },
        {rfingers,  "rfingers" },
        {rthumb,    "rthumb"   },
    };

    ostream << kIdxNameMap.at(bone_idx);
    return ostream;
}

#endif // #ifndef _ACCLAIM_BODY_IDX_H_
