#include "Task.hpp"
#include <rtt/FileDescriptorActivity.hpp>

#include <iostream>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>

using namespace std;
using namespace dgps;
using namespace RTT;

RTT::FileDescriptorActivity* Task::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }

Task::Task(std::string const& name)
    : TaskBase(name)
{
    _period.set(1);
    _dynamics_model.set(gps::ADAPTIVE);
    _ntpd_shm_unit.set(-1);
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
    if(!BaseTask::configureHook())
      return false;

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

        gps::AMBIGUITY_THRESHOLD threshold = _fix_threshold.get();
        if (!gps.setFixThreshold(threshold))
        {
            RTT::log(Error) << "failed to change RTK_FIX ambiguity threshold" << RTT::endlog();
            return false;
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

	if (_ntpd_shm_unit.get() > -1 && _ntpd_shm_unit.get() < 4)
        {
	    if (!gps.enableNtpdShm(_ntpd_shm_unit.get()))
            {
                RTT::log(Error) << "failed to initialize the NTP SHM" << RTT::endlog();
                return false;
            }
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
    if (!BaseTask::startHook())
        return false;
    return gps.setPeriodicData(_port, _period);
}

void Task::updateHook()
{
    char buffer[1024];
    static base::Time last_rate_display; // to display correction byterate
    static int corrections_rx = 0;

    if (last_rate_display.isNull())
        last_rate_display= base::Time::now();
    base::Time now = base::Time::now();
    
    RTT::FileDescriptorActivity* fd_activity =
        dynamic_cast<RTT::FileDescriptorActivity*>(getActivity().get());

    if (fd_activity->hasError())
        return BaseTask::error();

    if (correction_socket != -1 && fd_activity->isUpdated(correction_socket))
    {
        //std::cout << base::Time::now().toMilliseconds() << " data is available on correction input" << std::endl;
        int rd = read(correction_socket, buffer, 1024);
        if (rd > 0)
        {
            //std::cout << base::Time::now().toMilliseconds() << " got " << rd << " bytes of correction data" << std::endl;
            gps.writeCorrectionData(buffer, rd, 1000);
            corrections_rx += rd;
        }

    }
    
    if (fd_activity->isUpdated(gps.getFileDescriptor()))
    {
        try {
            gps.collectPeriodicData();

	    if (!gps.cpu_time.isNull())
	    {
	        gps::Time time;
	        time.cpu_time		=gps.cpu_time; 
	        time.gps_time		=gps.real_time;
	        time.processing_latency	=gps.processing_latency;
	        _time.write(time); 
	    }
	    
            if (last_update < gps.position.time && gps.position.time == gps.errors.time)
            {
                gps::Solution solution;
                solution.time                    = gps.position.time;
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
	        update(solution);	
             }

            if (gps.solutionQuality.time > last_constellation_update &&
                    gps.satellites.time > last_constellation_update)
            {
                ConstellationInfo info;
                info.quality    = gps.solutionQuality;
                info.satellites = gps.satellites;
                updateConstallation(info);
            }
        }
        catch(timeout_error) {
            std::cout << base::Time::now().toMilliseconds() << " got timeout on reading GPS data" << std::endl;
        }
        //std::cout << base::Time::now().toMilliseconds() << " finished reading GPS data" << std::endl;
    }

    float duration = (now -last_rate_display).toSeconds();
    if (duration > 1)
    {
        std::cout << base::Time::now().toMilliseconds() << " good: " << gps.getStats().good_rx << ", bad: " << gps.getStats().bad_rx << ", corrections: " << corrections_rx / duration << " bytes/s" << std::endl;
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

