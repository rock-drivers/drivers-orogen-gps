#include "Task.hpp"
#include <rtt/FileDescriptorActivity.hpp>

#include <iostream>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>
#include <ogr_spatialref.h>

using namespace std;

using namespace dgps;
using namespace RTT;

RTT::FileDescriptorActivity* Task::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }


Task::Task(std::string const& name)
    : TaskBase(name)
{

    _utm_zone.set(32);
    _utm_north.set(true);
    _period.set(1);
    _dynamics_model.set(gps::ADAPTIVE);
}

Task::~Task() {}

int Task::openSocket(std::string const& port)
{
    struct addrinfo hints;
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
    hints.ai_protocol = 0;          /* Any protocol */
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;
    hints.ai_next = NULL;

    struct addrinfo *result;
    int s = getaddrinfo(NULL, port.c_str(), &hints, &result);
    if (s != 0) {
        fprintf(stderr, "cannot bind to port %s: %s\n", port.c_str(), gai_strerror(s));
        return -1;
    }

    int sfd = -1;
    struct addrinfo *rp;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        sfd = socket(rp->ai_family, rp->ai_socktype,
                rp->ai_protocol);
        if (sfd == -1)
            continue;

        if (bind(sfd, rp->ai_addr, rp->ai_addrlen) == 0)
            break;                  /* Success */

        close(sfd);
    }

    freeaddrinfo(result);           /* No longer needed */

    if (rp == NULL)
    {
        fprintf(stderr, "Could not bind\n");
        return -1;
    }

    // Set the socket as nonblocking
    long fd_flags = fcntl(sfd, F_GETFL);                                                                                             
    fcntl(sfd, F_SETFL, fd_flags | O_NONBLOCK);                                                                                      

    return sfd;
}

bool Task::configureHook()
{
    // setup conversion from WGS84 to UTM
    OGRSpatialReference oSourceSRS;
    OGRSpatialReference oTargetSRS;
    
    oSourceSRS.SetWellKnownGeogCS( "WGS84" );
    oTargetSRS.SetWellKnownGeogCS( "WGS84" );
    oTargetSRS.SetUTM( _utm_zone, _utm_north );

    coTransform = OGRCreateCoordinateTransformation( &oSourceSRS,
	    &oTargetSRS );

    if( coTransform == NULL )
    {
	RTT::log(Error) << "failed to initialize CoordinateTransform UTM_ZONE:" << _utm_zone << " UTM_NORTH:" << _utm_north << RTT::endlog();
	return false;
    }

    try {
        if (!gps.openRover(_device))
        {
            RTT::log(Error) << "failed to initialize rover mode" << RTT::endlog();
            return false;
        }

        file_guard socket;
        string correction_input_port = _correction_port;

        if (correction_input_port.size() > 1 || correction_input_port.find_first_of("ABC") != 0)
        {
            //we seem to use UDP
            //create a socket to recevie the data
            int fd = openSocket(correction_input_port);
            if (fd < 0)
                return false;
            
            socket.reset(fd);
            
            //and tell the board, we are sending the correction data
            correction_input_port = _port;
        }

        UserDynamics dynamics = _user_dynamics.get();
        if (dynamics.hSpeed)
        {
            gps.setUserDynamics(dynamics.hSpeed, dynamics.hAccel, dynamics.vSpeed, dynamics.vAccel);
            gps.setReceiverDynamics(gps::USER_DEFINED);
        }
        else
            gps.setReceiverDynamics(_dynamics_model);
            
        if(!gps.setRTKInputPort(correction_input_port))
        {
            RTT::log(Error) << "failed to set RTK input port" << RTT::endlog();
            return false;
        }

        if (socket.get() != -1)
        {
            correction_socket = socket.release();
            getFileDescriptorActivity()->watch(correction_socket);
        }

        // start device
        getFileDescriptorActivity()->watch(gps.getFileDescriptor());

    } catch(timeout_error) {
        RTT::log(Error) << "timeout during board configuration" << RTT::endlog();
        return false;
    }
    return true;
}


bool Task::startHook()
{
    last_update = DFKI::Time();
    last_constellation_update = DFKI::Time();
    return gps.setPeriodicData(_port, _period);
}

void Task::updateHook()
{
    char buffer[1024];
    static DFKI::Time last_rate_display; // to display correction byterate
    static int corrections_rx = 0;

    if (last_rate_display.isNull())
        last_rate_display= DFKI::Time::now();
    DFKI::Time now = DFKI::Time::now();
    
    RTT::FileDescriptorActivity* fd_activity =
        dynamic_cast<RTT::FileDescriptorActivity*>(getActivity().get());

    if (fd_activity->hasError())
        return error();

    if (correction_socket != -1 && fd_activity->isUpdated(correction_socket))
    {
        //std::cout << DFKI::Time::now().toMilliseconds() << " data is available on correction input" << std::endl;
        int rd = read(correction_socket, buffer, 1024);
        if (rd > 0)
        {
            //std::cout << DFKI::Time::now().toMilliseconds() << " got " << rd << " bytes of correction data" << std::endl;
            gps.writeCorrectionData(buffer, rd, 1000);
            corrections_rx += rd;
        }

    }
    
    if (fd_activity->isUpdated(gps.getFileDescriptor()))
    {
        try {
            gps.collectPeriodicData();

            if (last_update < gps.position.timestamp && gps.position.timestamp == gps.errors.timestamp)
            {
                last_update = gps.position.timestamp;

                gps::Solution solution;
                solution.timestamp                    = gps.position.timestamp;
                solution.latitude                     = gps.position.latitude;
                solution.longitude                    = gps.position.longitude;
                solution.positionType                 = gps.position.positionType;
                solution.noOfSatellites               = gps.position.noOfSatellites;
                solution.altitude                     = gps.position.altitude;
                solution.geoidalSeparation            = gps.position.geoidalSeparation;
                solution.ageOfDifferentialCorrections = gps.position.ageOfDifferentialCorrections;
                solution.deviationLatitude            = gps.errors.deviationLatitude;
                solution.deviationLongitude           = gps.errors.deviationLongitude;
                solution.deviationAltitude            = gps.errors.deviationAltitude;
                _solution.write(solution);
		
		// if there is a valid reading, then write it to position readings port
		if( gps.position.positionType != gps::NO_SOLUTION )
		{ 
		    double la = solution.latitude;
		    double lo = solution.longitude;
		    double alt = solution.altitude;

		    coTransform->Transform(1, &la, &lo, &alt);
		    DFKI::PositionReading pos;
		    pos.stamp = gps.position.timestamp;
		    pos.position.x() = la - _origin.value().x();
		    pos.position.y() = lo - _origin.value().y();
		    pos.position.z() = alt - _origin.value().z();
		    pos.error.x() = gps.errors.deviationLatitude;
		    pos.error.y() = gps.errors.deviationLongitude;
		    pos.error.z() = gps.errors.deviationAltitude;
		    _position_readings.write(pos);
		}
            }

            if (gps.solutionQuality.timestamp > last_constellation_update &&
                    gps.satellites.timestamp > last_constellation_update)
            {
                ConstellationInfo info;
                info.quality    = gps.solutionQuality;
                info.satellites = gps.satellites;
                _constellation.write(info);
                last_constellation_update = DFKI::Time::now();
            }
        }
        catch(timeout_error) {
            std::cout << DFKI::Time::now().toMilliseconds() << " got timeout on reading GPS data" << std::endl;
        }
        //std::cout << DFKI::Time::now().toMilliseconds() << " finished reading GPS data" << std::endl;
    }

    float duration = (now -last_rate_display).toSeconds();
    if (duration > 1)
    {
        std::cout << DFKI::Time::now().toMilliseconds() << " good: " << gps.getStats().good_rx << ", bad: " << gps.getStats().bad_rx << ", corrections: " << corrections_rx / duration << " bytes/s" << std::endl;
        last_rate_display= now;
        corrections_rx = 0;
        gps.resetStats();
    }
}

// void Task::errorHook()
// {
// }

void Task::stopHook()
{
    gps.stopPeriodicData();
}

void Task::cleanupHook()
{
    if (correction_socket != -1)
        close(correction_socket);
    gps.close();
}

