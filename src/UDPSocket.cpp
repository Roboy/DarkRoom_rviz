#include "darkroom/UDPSocket.hpp"

bool convertByte2Text(uint32_t inet, char *inet_str){
    if(inet_ntop(AF_INET, &inet, inet_str, INET_ADDRSTRLEN) == NULL)
        return false;
    else
        return true;
}

bool convertText2Byte(char *inet_str, uint32_t *inet){
    if(inet_pton(AF_INET, inet_str, inet)<= 0)
        return false;
    else
        return true;
}

UDPSocket::UDPSocket(const char *server_IP, int server_port, const char *client_IP, int client_port, bool exclusive):
exclusive(exclusive){
    ROS_INFO_STREAM("creating socket on " << server_IP << ":" << server_port << " with client " << client_IP << ":"
                                          << client_port);
    int rv;

    struct addrinfo hints, *p;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    char pstr[10];
    snprintf(pstr, 10, "%d", server_port);
    if ((rv = getaddrinfo(server_IP, pstr, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return;
    }

    int yes = 1;
    // loop through all the results, make a socket and bind
    for (p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            ROS_ERROR("talker: socket");
            continue;
        }
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &yes, sizeof(int)) == -1) {
            ROS_ERROR("setsockopt");
            continue;
        };
        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            ROS_ERROR("listener: bind");
            continue;
        }
        break;
    }

    if (p == NULL) {
        ROS_ERROR("talker: failed to bind socket");
        return;
    }

    // set 100ms timeout
    setTimeOut(100000);

    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr(client_IP);
    client_addr.sin_port = htons(client_port);
    client_addr_len = sizeof(client_addr);

    initialized = true;
}

UDPSocket::UDPSocket(const char *client_IP, int client_port, bool exclusive):exclusive(exclusive) {
    string myIP;
    if (whatsMyIP(myIP)) {
        ROS_INFO_STREAM(
                "creating socket on " << myIP << ":" << 8000 << " with client " << client_IP << ":" << client_port);
        int rv;
        struct addrinfo hints, *p;

        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;

        char pstr[10];
        snprintf(pstr, 10, "%d", 8000);
        if ((rv = getaddrinfo(myIP.c_str(), pstr, &hints, &servinfo)) != 0) {
            ROS_ERROR("getaddrinfo: %s", gai_strerror(rv));
            return;
        }

        int yes = 1;
        // loop through all the results, make a socket and bind
        for (p = servinfo; p != NULL; p = p->ai_next) {
            if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
                ROS_ERROR("talker: socket");
                continue;
            }
            if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &yes, sizeof(int)) == -1) {
                ROS_ERROR("setsockopt");
                continue;
            };
            if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
                close(sockfd);
                ROS_ERROR("listener: bind");
                continue;
            }
            break;
        }

        if (p == NULL) {
            ROS_ERROR("talker: failed to bind socket");
            return;
        }

        // set 100ms timeout
        setTimeOut(100000);

        client_addr.sin_family = AF_INET;
        client_addr.sin_addr.s_addr = inet_addr(client_IP);
        client_addr.sin_port = htons(client_port);
        client_addr_len = sizeof(client_addr);

        initialized = true;
    }else{
        ROS_ERROR("could not create UDP socket");
    }
}

UDPSocket::UDPSocket(const char *client_IP, int client_port, int server_port, bool exclusive):exclusive(exclusive) {
    string myIP;
    if (whatsMyIP(myIP)) {
        ROS_INFO_STREAM(
                "creating socket on " << myIP << ":" << server_port << " with client " << client_IP << ":" << client_port);
        int rv;
        struct addrinfo hints, *p;

        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;

        char pstr[10];
        snprintf(pstr, 10, "%d", server_port);
        if ((rv = getaddrinfo(myIP.c_str(), pstr, &hints, &servinfo)) != 0) {
            ROS_ERROR("getaddrinfo: %s", gai_strerror(rv));
            return;
        }

        int yes = 1;
        // loop through all the results, make a socket and bind
        for (p = servinfo; p != NULL; p = p->ai_next) {
            if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
                ROS_ERROR("talker: socket");
                continue;
            }
            if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &yes, sizeof(int)) == -1) {
                ROS_ERROR("setsockopt");
                continue;
            };
            if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
                close(sockfd);
                ROS_ERROR("listener: bind");
                continue;
            }
            break;
        }

        if (p == NULL) {
            ROS_ERROR("talker: failed to bind socket");
            return;
        }

        // set 100ms timeout
        setTimeOut(100000);

        client_addr.sin_family = AF_INET;
        client_addr.sin_addr.s_addr = inet_addr(client_IP);
        client_addr.sin_port = htons(client_port);
        client_addr_len = sizeof(client_addr);

        initialized = true;
    }else{
        ROS_ERROR("could not create UDP socket");
    }
}

UDPSocket::UDPSocket(int port, bool broadcaster) {
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(port);
    broadcast_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    broadcast_addr_len = sizeof(broadcast_addr);

    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(port);
    client_addr.sin_addr.s_addr = INADDR_ANY;
    client_addr_len = sizeof(client_addr);

    // creat UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        ROS_ERROR("talker: socket");
        return;
    }

    // Allow broadcasts
    int yes = true;
    if  (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (const void *)&yes, sizeof(int)) == -1) {
        ROS_ERROR("broadcasting not allowed");
        return;
    }

    // set 100ms timeout
    setTimeOut(100000);

    if(!broadcaster) {
        // Bind an address to our socket, so that client programs can listen to this server
        if (bind(sockfd, (struct sockaddr *) &client_addr, client_addr_len) == -1) {
            close(sockfd);
            ROS_ERROR("broadcaster bind error");
            return;
        }
    }

    initialized = true;
}

UDPSocket::~UDPSocket() {
    close(sockfd);
}

bool UDPSocket::receiveSensorData(unsigned short &timestamp, int &id, int &lighthouse, int &rotor, uint &sweepDuration){
    if(receiveUDPFromClient() && numbytes == 6){
        // the first two byte are the timestamp in milliseconds (overflows approx every 4 min)
        timestamp = buf[1] << 8 | buf[0] & 0xff;
        // decode our 4 byte frame
        uint32_t data = 0;
        data = (uint32_t)((uint8_t)buf[5] << 24 | (uint8_t)buf[4] << 16 | (uint8_t)buf[3] << 8 | (uint8_t)buf[2]);
        id             = data & 0x01FF;
        lighthouse     = (data >> 9) & 0x01;
        rotor          = (data >> 10) & 0x01;
        int valid      = (data >> 12) & 0x01;
        sweepDuration  = (data >> 13) & 0x07FFFF;
        return (valid == 1);
    }else if(numbytes > 6){
        ROS_DEBUG( "received more bytes than expected");
    }else if(numbytes < 6) {
        ROS_DEBUG("received less bytes than expected");
    }
    return false;
}

bool UDPSocket::receiveSensorData(unsigned short &timestamp, vector<uint> &id, vector<uint> &lighthouse, vector<uint> &rotor, vector<uint> &sweepDuration){
    if(receiveUDPFromClient() && (numbytes-2)%4 == 0){ // without the first two bytes, which is the timestamp, each sensor value is 4 bytes
        // the first two byte are the timestamp in milliseconds (overflows approx every 4 min)
        timestamp = buf[1] << 8 | buf[0] & 0xff;
        for(int i=0;i<(numbytes-2)/4;i++){
          // decode our 4 byte frame
          uint32_t data = 0;
          data = (uint32_t)((uint8_t)buf[i*4+5] << 24 | (uint8_t)buf[i*4+4] << 16 | (uint8_t)buf[i*4+3] << 8 | (uint8_t)buf[i*4+2]);
          int valid      = (data >> 12) & 0x01;
          if(valid==1){
            id.push_back(data & 0x01FF);
            lighthouse.push_back((data >> 9) & 0x01);
            rotor.push_back((data >> 10) & 0x01);
            sweepDuration.push_back((data >> 13) & 0x07FFFF);
          }
        }
        return true;
    }
    return false;
}

bool UDPSocket::setTimeOut(int usecs) {
// set 10ms timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = usecs;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        ROS_ERROR("Error setting timeout");
        return false;
    }
    return true;
}

bool UDPSocket::whatsMyIP(string &ip) {
    struct ifaddrs *ifAddrStruct = NULL;
    void *tmpAddrPtr = NULL;

    getifaddrs(&ifAddrStruct);
    char IP[INET_ADDRSTRLEN];

    for (struct ifaddrs *ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;

            inet_ntop(AF_INET, tmpAddrPtr, IP, INET_ADDRSTRLEN);
            string str(ifa->ifa_name);
            if (str.find("wlp") != std::string::npos ||
                str.find("eth") != std::string::npos ||
                str.find("enp") != std::string::npos) { // if wifi or ethernet adapter
                ROS_INFO("%s IP Address %s", ifa->ifa_name, IP);
                ip = string(IP);
                myIP.second = ip;
                return convertText2Byte(IP,&myIP.first);
            }
        }
    }
    return false;
}

bool UDPSocket::receiveUDP() {
    if ((numbytes = recv(sockfd, buf, MAXBUFLENGTH - 1, 0)) ==
        -1) {
        ROS_DEBUG_THROTTLE(5, "received nothing");
        return false;
    }else {
        ROS_DEBUG_THROTTLE(5, "got message: " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " "
                BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " ",
                          BYTE_TO_BINARY(buf[3]), BYTE_TO_BINARY(buf[2]), BYTE_TO_BINARY(buf[1]), BYTE_TO_BINARY(buf[0]));
    }
    return true;
}

bool UDPSocket::receiveUDPFromClient() {
    if ((numbytes = recvfrom(sockfd, buf, MAXBUFLENGTH - 1, 0, (struct sockaddr *) &client_addr, &client_addr_len)) ==
        -1) {
        ROS_DEBUG_THROTTLE(5, "received nothing");
        return false;
    }else {
        ROS_DEBUG_THROTTLE(1, "got message: " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " "
                BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " ",
                          BYTE_TO_BINARY(buf[3]), BYTE_TO_BINARY(buf[2]), BYTE_TO_BINARY(buf[1]), BYTE_TO_BINARY(buf[0]));
    }
    return true;
}

bool UDPSocket::sendUDPToClient() {
    if ((numbytes = sendto(sockfd, buf, numbytes, 0, (struct sockaddr *) &client_addr, client_addr_len) == -1)) {
        ROS_ERROR_THROTTLE(1, "could not send");
        return false;
    }
    return true;
}

bool UDPSocket::broadcastUDP() {
    if ((numbytes = sendto(sockfd, buf, numbytes, 0, (struct sockaddr *) &broadcast_addr, broadcast_addr_len)) ==
        -1) {
        ROS_ERROR_THROTTLE(1, "could not broadcast");
        return false;
    }
    return true;
}
