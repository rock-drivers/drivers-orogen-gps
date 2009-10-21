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

RTT::FileDescriptorActivity* Task::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }


Task::Task(std::string const& name)
    : TaskBase(name)
{
}

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
    if (!gps.openRover(_device))
        return false;

    
    correction_socket = -1;
    string correction_input_port = _correctionPort;
    if(_correctionPort.value().size() != 1 || _correctionPort.value().find_first_of("ABC") != 0) {
	//we seem to use UDP
	//create a socket to recevie the data
        correction_socket = openSocket(_correctionPort);
	
	if(correction_socket < 0)
	    return false;
	
        guard_socket.reset(new file_guard(correction_socket));
	
	//and tell the board, we are sending the correction data
        correction_input_port = _port;
    }
        
    if(!gps.setRTKInputPort(correction_input_port))
	return false;

    
    // start device
    getFileDescriptorActivity()->watch(gps.getFileDescriptor());
    return true;
}


bool Task::startHook()
{
    // start GPS information looping
    last_update = DFKI::Time();
     
    return gps.setPeriodicData(_port, _period);
}

void Task::updateHook()
{
    char buffer[1024];
    
    if(correction_socket != -1) {
	int rd = recv(correction_socket, buffer, 1024, 0);
	if (rd > 0)
	{
	    gps.writeCorrectionData(buffer, rd, 1000);
	}
	else if (rd < 0 && errno != EAGAIN)
	{
	    cerr << "error reading socket: " << strerror(errno) << endl;
	    error();
	}
    }
    
    gps.collectPeriodicData();

    if (last_update < gps.position.timestamp && gps.position.timestamp == gps.errors.timestamp)
    {
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
    }

//if (_satellite.connected())
//        _satellite.write(gps.satellites);
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
    gps.close();
}

