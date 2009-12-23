#ifndef DGPS_CONFIGURATION_TYPES_HPP
#define DGPS_CONFIGURATION_TYPES_HPP

#include "dgpstypes.hh"

namespace dgps {
    struct UserDynamics {
        int hSpeed;
        int hAccel;
        int vSpeed;
        int vAccel;
#ifndef __orogen
        UserDynamics()
            : hSpeed(0), hAccel(0), vSpeed(0), vAccel(0) {}
#endif
    };

    struct ConstellationInfo {
        gps::SolutionQuality quality;
        gps::SatelliteInfo  satellites;
    };
}

#endif
