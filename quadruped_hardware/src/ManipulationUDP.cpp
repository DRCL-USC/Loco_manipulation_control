
#include "ManipulationUDP.h"
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

ManipulationUDP::ManipulationUDP(int port) : 
loop_loco_manipulation("loco_manipulation", 0.001, boost::bind(&ManipulationUDP::socketSendRecv, this))
{

    rotmat = Eigen::Matrix3d::Identity();

    std::thread socketThread(&ManipulationUDP::setupSocket, this, port);
    socketThread.detach();
}

void ManipulationUDP::run(LowlevelState *state)
{
    Eigen::Quaterniond quat(state->imu.quaternion[0], state->imu.quaternion[1], state->imu.quaternion[2], state->imu.quaternion[3]);
    rotmat = quat.toRotationMatrix();

    for (int i = 0; i < 2; i++)
    {
        state->userValue.manipulation_force[i] = Highcmd.manipulation_force(i);
    }
    state->userValue.vx = Highcmd.velocity_cmd[0];
    state->userValue.vy = Highcmd.velocity_cmd[1];
    state->userValue.turn_rate = Highcmd.omega_cmd[2];
}

int ManipulationUDP::setupSocket(int port)
{
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        std::cerr << "Error creating socket" << std::endl;
        return -1;
    }

    // Bind the socket to an IP address and port
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port); // Change to your desired port
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
    {
        std::cerr << "Error binding socket" << std::endl;
        return -1;
    }

    // Listen for incoming connections
    if (listen(serverSocket, 5) == -1)
    {
        std::cerr << "Error listening on socket" << std::endl;
        return -1;
    }

    // Accept a connection
    clientSocket = accept(serverSocket, NULL, NULL);

    if (clientSocket == -1)
    {
        std::cerr << "Error accepting connection" << std::endl;
        return -1;
    }

    // start the loop
    loop_loco_manipulation.start();

    return 0;
}

void ManipulationUDP::socketSendRecv()
{
    ssize_t bytesRead = recv(clientSocket, &_dataRecv, sizeof(_dataRecv), 0);
    if (bytesRead == -1)
    {
        std::cerr << "Error receiving data" << std::endl;
    }
    else
    {
        std::cout << "Received Force: " << _dataRecv.force[0] << std::endl;
        std::cout << "Received Vel: " << _dataRecv.velocity[1] << std::endl;
        std::cout << "Received Omega: " << _dataRecv.omega[2] << std::endl;

        // Eigen::Vector3d force_body = (Eigen::Vector3d() << _dataRecv.force[0], 0, 0).finished();
        // Highcmd.manipulation_force = rotmat * force_body;

        // Highcmd.omega_cmd[2] = _dataRecv.omega[2];

        // Highcmd.velocity_cmd[0] = 0.05;

        // Highcmd.velocity_cmd[1] = _dataRecv.velocity[1]; // + 3 * (distance_body[1]);
    }
}