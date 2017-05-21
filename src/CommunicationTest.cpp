#include "darkroom/UDPSocket.hpp"

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "CommunicationTest",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);

    boost::shared_ptr<ros::AsyncSpinner> spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));

    if (argc == 2 && strcmp(argv[1], "ping") == 0) {
        UDPSocket pingSocket(8000, false);
        DarkRoomProtobuf::configObject config_message;
        while (!pingSocket.receiveMessage<DarkRoomProtobuf::configObject>(config_message)) {
            ROS_INFO_THROTTLE(1, "waiting for config message from mkr");
        }
    } else {

        string client_IP;
        int client_port, server_port;
        if (argc == 4) {
            server_port = atoi(argv[1]);
            client_IP = argv[2];
            client_port = atoi(argv[3]);
        } else {
            server_port = 8002;
            client_IP = "10.25.13.112";
            client_port = 8000;
        }
        UDPSocket udpSocket(client_IP.c_str(), client_port, server_port);

        while (ros::ok()) {
            unsigned short timestamp;
            int id, lighthouse, rotor;
            uint sweepDuration;
            if(udpSocket.receiveSensorData(timestamp, id, lighthouse, rotor, sweepDuration)){
//            if(lighthouse == 1 && rotor == 1) {

//                printf(          "\nlighthouse:    %d"
//                                 "\nrotor:         %d"
//				 "\nsweep:         %d"
//                                 "\nangle: %f",  lighthouse, rotor, sweepDuration, sweepDuration * 0.0216);
                ROS_INFO("\ntimestamp:    %hu"
                                 "\nid:            %d"
                                 "\nlighthouse:    %d"
                                 "\nrotor:         %d"
                                 "\nsweepDuration: %d"
                                 "\nangle: %f", timestamp, id, lighthouse, rotor, sweepDuration,
                         sweepDuration * 0.0216);
//            }
            }else{
                ROS_WARN_THROTTLE(1,"invalid data");
            }
        }
    }


}
