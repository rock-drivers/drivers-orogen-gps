#include "MB500Task.hpp"
#include <mb500.hh>
#include <rtt/extras/FileDescriptorActivity.hpp>

#include <iostream>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>

using namespace std;
using namespace gps;
using RTT::Error;

MB500Task::MB500Task(std::string const& name)
    : MB500TaskBase(name), driver(new gps::MB500)
{
    _period.set(1);
    _dynamics_model.set(gps::MB500_ADAPTIVE);
    _ntpd_shm_unit.set(-1);
}

MB500Task::~MB500Task() {}

int MB500Task::openSocket(std::string const& port)
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

bool MB500Task::configureHook()
{
    if (!MB500TaskBase::configureHook())
      return false;

    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (!fd_activity)
    {
        RTT::log(Error) << "the MB500Task requires a FD activity" << RTT::endlog();
        return false;
    }

    try {
        if (!driver->openRover(_device))
        {
            RTT::log(Error) << "failed to initialize rover mode" << RTT::endlog();
            return false;
        }

        iodrivers_base::FileGuard socket;
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

        gps::MB500_AMBIGUITY_THRESHOLD threshold = _fix_threshold.get();
        if (!driver->setFixThreshold(threshold))
        {
            RTT::log(Error) << "failed to change RTK_FIX ambiguity threshold" << RTT::endlog();
            return false;
        }

        UserDynamics dynamics = _user_dynamics.get();
        if (dynamics.hSpeed)
        {
            if(!driver->setUserDynamics(dynamics.hSpeed, dynamics.hAccel, dynamics.vSpeed, dynamics.vAccel))
	    {
		RTT::log(Error) << "failed to set user dynamics" << RTT::endlog();
		return false;
	    }
            if(!driver->setReceiverDynamics(gps::MB500_USER_DEFINED))
	    {
		RTT::log(Error) << "failed to set receiver dynamics" << RTT::endlog();
		return false;
	    }
        }
        else
	{
            if(!driver->setReceiverDynamics(_dynamics_model))
	    {
		RTT::log(Error) << "failed to set receiver dynamics" << RTT::endlog();
		return false;
	    }
	}
            
        if(!driver->setRTKInputPort(correction_input_port))
        {
            RTT::log(Error) << "failed to set RTK input port" << RTT::endlog();
            return false;
        }

        correction_socket = socket.release();

	if (_ntpd_shm_unit.get() > -1 && _ntpd_shm_unit.get() < 4)
        {
	    if (!driver->enableNtpdShm(_ntpd_shm_unit.get()))
            {
                RTT::log(Error) << "failed to initialize the NTP SHM" << RTT::endlog();
                return false;
            }
        }
    } catch(iodrivers_base::TimeoutError) {
        RTT::log(Error) << "timeout during board configuration" << RTT::endlog();
        return false;
    }
    return true;
}


bool MB500Task::startHook()
{
    if (!MB500TaskBase::startHook())
        return false;

    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (correction_socket != -1)
        fd_activity->watch(correction_socket);
    fd_activity->watch(driver->getFileDescriptor());

    if(!driver->setPeriodicData(_port, _period))
    {
	fd_activity->clearAllWatches();
	return false;
    }
    return true;
}

void MB500Task::updateHook()
{
    char buffer[1024];
    static base::Time last_rate_display; // to display correction byterate
    static int corrections_rx = 0;

    if (last_rate_display.isNull())
        last_rate_display= base::Time::now();
    base::Time now = base::Time::now();
    
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();

    if (fd_activity->hasTimeout())
        return exception(IO_TIMEOUT);
    else if (fd_activity->hasError())
        return exception(IO_ERROR);

    if (correction_socket != -1 && fd_activity->isUpdated(correction_socket))
    {
        //std::cout << base::Time::now().toMilliseconds() << " data is available on correction input" << std::endl;
        int rd = read(correction_socket, buffer, 1024);
        if (rd > 0)
        {
            //std::cout << base::Time::now().toMilliseconds() << " got " << rd << " bytes of correction data" << std::endl;
            driver->writeCorrectionData(buffer, rd, 1000);
            corrections_rx += rd;
        }

    }
    
    if (fd_activity->isUpdated(driver->getFileDescriptor()))
    {
        try {
            driver->collectPeriodicData();

	    if (!driver->cpu_time.isNull())
	    {
	        gps::Time time;
	        time.cpu_time		=driver->cpu_time; 
	        time.gps_time		=driver->real_time;
	        time.processing_latency	=driver->processing_latency;
	        _time.write(time); 
	    }
	    
            if (last_update < driver->position.time && driver->position.time == driver->errors.time)
            {
                gps::Solution solution;
                solution.time                    = driver->position.time;
                solution.latitude                     = driver->position.latitude;
                solution.longitude                    = driver->position.longitude;
                solution.positionType                 = driver->position.positionType;
                solution.noOfSatellites               = driver->position.noOfSatellites;
                solution.altitude                     = driver->position.altitude;
                solution.geoidalSeparation            = driver->position.geoidalSeparation;
                solution.ageOfDifferentialCorrections = driver->position.ageOfDifferentialCorrections;
                solution.deviationLatitude            = driver->errors.deviationLatitude;
                solution.deviationLongitude           = driver->errors.deviationLongitude;
                solution.deviationAltitude            = driver->errors.deviationAltitude;
	        update(solution);	
             }

            if (driver->solutionQuality.time > last_constellation_update &&
                    driver->satellites.time > last_constellation_update)
            {
                ConstellationInfo info;
                info.quality    = driver->solutionQuality;
                info.satellites = driver->satellites;
                _constellation.write(info);
                last_constellation_update = base::Time::now();
            }
        }
        catch(iodrivers_base::TimeoutError) {
            std::cout << base::Time::now().toMilliseconds() << " got timeout on reading GPS data" << std::endl;
        }
        //std::cout << base::Time::now().toMilliseconds() << " finished reading GPS data" << std::endl;
    }

    float duration = (now -last_rate_display).toSeconds();
    if (duration > 1)
    {
        std::cout << base::Time::now().toMilliseconds() << " good: " << driver->getStats().good_rx << ", bad: " << driver->getStats().bad_rx << ", corrections: " << corrections_rx / duration << " bytes/s" << std::endl;
        last_rate_display= now;
        corrections_rx = 0;
        driver->resetStats();
    }
}

// void MB500Task::errorHook()
// {
// }

void MB500Task::stopHook()
{
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    fd_activity->clearAllWatches();

    driver->stopPeriodicData();
}

void MB500Task::cleanupHook()
{
    if (correction_socket != -1)
        close(correction_socket);
    driver->close();
}

