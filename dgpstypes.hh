#ifndef DGPS_TYPES_H
#define DGPS_TYPES_H

#ifndef __orogen
#include <vector>
#endif

namespace gps {
	struct Info {
		double UTCTime;
		double latitude;
		double longitude;
		int positionType;
		int noOfSatellites;
		double altitude;
		double geoidalSeparation;
		double ageOfDifferentialCorrections;
	};

	struct Errors {
		double UTCTime;
		double deviationLatitude;
		double deviationLongitude;
		double deviationAltitude;
	};

	struct FullInfo {
		Errors errors;
		Info info;
	};

	struct Satellite {
		int PRN;
		int elevation;
		int azimuth;
		double SNR;
	};

	struct SatelliteInfo {
		int noOfSatellites;
		std::vector < gps::Satellite> sat;
	};

}

#endif
