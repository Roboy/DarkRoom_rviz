#include "darkroom/TrackedObject.hpp"

int TrackedObject::trackeObjectInstance = 0;

TrackedObject::TrackedObject() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "TrackedObject",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);

    if (const char* env_p = getenv("DARKROOM_CALIBRATED_OBJECTS")) {
        path = env_p;
        ROS_INFO_STREAM("using DARKROOM_CALIBRATED_OBJECTS: " << path);
    }else
        ROS_WARN("could not get DARKROOM_CALIBRATED_OBJECTS environmental variable");


//    readConfig(path + "/nerfgun.yaml");

    trackeObjectInstance++;
}

TrackedObject::~TrackedObject() {
    receiveData = false;
    tracking = false;
    calibrating = false;
    if (sensor_thread != nullptr) {
        if (sensor_thread->joinable()) {
            ROS_INFO("Waiting for sensor thread to terminate");
            sensor_thread->join();
        }
    }
    if (tracking_thread != nullptr) {
        if (tracking_thread->joinable()) {
            ROS_INFO("Waiting for tracking thread to terminate");
            tracking_thread->join();
        }
    }
    if (calibrate_thread != nullptr) {
        if (calibrate_thread->joinable()) {
            ROS_INFO("Waiting for calibration thread to terminate");
            calibrate_thread->join();
        }
    }
}

bool TrackedObject::connectWifi(const char *TrackedObjectIP, int sensor_port, int command_port, int logging_port,
                                bool configure) {
    logging_socket.reset();
    logging_socket = UDPSocketPtr(new UDPSocket(TrackedObjectIP, command_port, DEFAULT_LOGGING_PORT, true));
    command_socket.reset();
    command_socket = UDPSocketPtr(new UDPSocket(TrackedObjectIP, command_port, DEFAULT_COMMAND_PORT, true));
    sensor_socket.reset();
    sensor_socket = UDPSocketPtr(new UDPSocket(TrackedObjectIP, command_port, DEFAULT_SENSOR_PORT, true));
    imu_socket.reset();
    imu_socket = UDPSocketPtr(new UDPSocket(TrackedObjectIP, command_port, DEFAULT_IMU_PORT, true));

    // prepare configure message
    DarkRoomProtobuf::configObject msg;
    msg.set_ip(command_socket->myIP.first);
    msg.set_logging_port(DEFAULT_LOGGING_PORT);
    msg.set_sensor_port(DEFAULT_SENSOR_PORT);
    msg.set_imu_port(DEFAULT_IMU_PORT);

    ros::Time start_time = ros::Time::now();
    command_socket->sendMessage<DarkRoomProtobuf::configObject>(msg);

    if (configure) { // send our ports and wait for a logging message
        ROS_INFO("Waiting 5 seconds for sensor data");
        while ((ros::Time::now() - start_time).sec < 5) { // wait for logging message
            DarkRoomProtobuf::loggingObject log_msg;
            if (logging_socket->receiveMessage<DarkRoomProtobuf::loggingObject>(log_msg)) {
                ROS_WARN_STREAM("connection established with " << TrackedObjectIP << ", sensor_port: " << sensor_port <<
                                                               ", command_port: " << command_port << ", logging_port: "
                                                               << logging_port);
                connected = true;
                startReceiveData(true);
                return true;
            }
        }
        ROS_ERROR("did not receive any data aborting connection");
        return false;
    } else {
        connected = true;
        startReceiveData(true);
        return true;
    }
}

void TrackedObject::connectRoboy(){
    receiveData = true;
    sensor_sub =  nh->subscribe("/roboy/middleware/DarkRoom/sensors", 1, &TrackedObject::receiveSensorDataRoboy, this);
}

void TrackedObject::startReceiveData(bool start) {
    if (start) {
        if (!receiveData) {
            ROS_INFO("starting sensor thread");
            receiveData = true;
            sensor_thread = boost::shared_ptr<thread>(new thread(&TrackedObject::receiveSensorData, this));
            sensor_thread->detach();
            imu_thread = boost::shared_ptr<thread>(new thread(&TrackedObject::receiveImuData, this));
            imu_thread->detach();
        } else {
            ROS_WARN("sensor thread already running");
        }
    } else {
        if (sensor_thread != nullptr) {
            ROS_INFO("stopping sensor thread");
            receiveData = false;
            if (sensor_thread->joinable()) {
                ROS_INFO("Waiting for sensor thread to terminate");
                sensor_thread->join();
            }
        }
        if (imu_thread != nullptr) {
            ROS_INFO("stopping imu thread");
            receiveData = false;
            if (imu_thread->joinable()) {
                ROS_INFO("Waiting for imu thread to terminate");
                imu_thread->join();
            }
        }
    }
}

void TrackedObject::startTracking(bool start) {
    if(command_socket != nullptr)
        toggleTracking(start);
    if (start) {
        if (!receiveData)
            startReceiveData(true);
        ROS_INFO("starting tracking thread");
        tracking = true;
        tracking_thread = boost::shared_ptr<thread>(new thread(&TrackedObject::trackSensors, this));
        tracking_thread->detach();
    } else {
        if (tracking_thread != nullptr) {
            ROS_INFO("stopping tracking thread");
            tracking = false;
            if (tracking_thread->joinable()) {
                ROS_INFO("Waiting for tracking thread to terminate");
                tracking_thread->join();
            }
        }
    }
}

void TrackedObject::startCalibration(bool start) {
    if (start) {
        ROS_INFO("starting calibration thread");
        calibrating = true;
        calibrate_thread = boost::shared_ptr<thread>(new thread(&TrackedObject::calibrate, this));
        calibrate_thread->detach();
    } else {
        if (calibrate_thread != nullptr) {
            calibrating = false;
            if (calibrate_thread->joinable()) {
                ROS_INFO("Waiting for calibration thread to terminate");
                calibrate_thread->join();
            }
        }
    }
}

void TrackedObject::showRays(bool show) {
    rays = show;
}

void TrackedObject::map_lighthouse_id(bool switchID) {
    ROS_INFO_STREAM("switching lighthouses " << switchID);
    m_switch = switchID;
    for (auto &sensor : sensors) {
        sensor.second.switchLighthouses(switchID);
    }
}

bool TrackedObject::distanceEstimation(bool lighthouse, vector<int> *specificIds) {
    vector<Vector3d> relPos;
    vector<double> elevations, azimuths;
    vector<double> distanceToLighthouse;
    vector<int> ids;
    if (specificIds == nullptr) {
        // let's see who is active
        for (auto &sensor : sensors) {
            // skip inactive sensors
            if (sensor.second.isActive(lighthouse)) {
                ids.push_back(sensor.first);
                sensor.second.get(lighthouse, elevations, azimuths);
                sensor.second.getRelativeLocation(relPos);
                distanceToLighthouse.push_back(sensor.second.getDistance(lighthouse));
            }
        }
    } else {
        uint sensor_counter = 0;
        // wait until all requested sensors are active
        ros::Time start_time = ros::Time::now();
        while ((ros::Time::now() - start_time) < ros::Duration(5) && sensor_counter < specificIds->size()) {
            sensor_counter = 0;
            for (uint i = 0; i < specificIds->size(); i++) {
                // skip inactive sensors
                if (sensors[specificIds->at(i)].isActive(lighthouse)) {
                    ROS_INFO("sensor%d active", specificIds->at(i));
                    sensor_counter++;
                } else {
                    ROS_WARN("sensor%d inactive", specificIds->at(i));
                }
            }
            ROS_INFO_THROTTLE(1, "waiting for specific sensors to become active");
        }
        if (sensor_counter < specificIds->size()) {
            ROS_WARN("time out waiting for specific sensors");
            return false;
        }
        // get the values now that all requested sensors are active
        for (uint i = 0; i < specificIds->size(); i++) {
            ids.push_back(specificIds->at(i));
            sensors[specificIds->at(i)].get(lighthouse, elevations, azimuths);
            sensors[specificIds->at(i)].getRelativeLocation(relPos);
            distanceToLighthouse.push_back(sensors[specificIds->at(i)].getDistance(lighthouse));
        }
    }
    if (ids.size() < 3)
        return false;
    // cost function
    auto f = [](double &R0, double &R1, double &cosine, double &distance) {
        return (pow(R0, 2.0) + pow(R1, 2.0) - 2.0 * R0 * R1 * cosine - pow(distance, 2.0));
    };
    // partial derivative
    auto df = [](double &R0, double &R1, double &cosine) { return (2.0 * R0 - 2 * R1 * cosine); };

    MatrixXd cosineBetween(ids.size(), ids.size()), distanceBetween(ids.size(), ids.size());
    for (uint i = 0; i < ids.size() - 1; i++) {
        for (uint j = i + 1; j < ids.size(); j++) {
            // calculate the cosine between the two sensors
            cosineBetween(i, j) =
                    sin(elevations[i]) * cos(azimuths[i]) * sin(elevations[j]) * cos(azimuths[j]) +
                    sin(elevations[i]) * sin(azimuths[i]) * sin(elevations[j]) * sin(azimuths[j]) +
                    cos(elevations[i]) * cos(elevations[j]);
            // calculate the distance between the sensors
            distanceBetween(i, j) = (relPos[i] - relPos[j]).norm();
            ROS_INFO("distance between sensor %d and %d: %f", ids[i], ids[j], distanceBetween(i, j));
        }
    }

    int iterations = 0;

    uint n = ids.size();
    MatrixXd J(n * (n - 1) / 2, n);
    J = J.setZero();
    VectorXd v(n * (n - 1) / 2), d_old(ids.size());

    double error;
    while (iterations < MAX_ITERATIONS) {

        // construct jacobian and function vector
        int row = 0;
        for (uint i = 0; i < ids.size() - 1; i++) {
            for (uint j = i + 1; j < ids.size(); j++) {
                J(row, i) = df(distanceToLighthouse[i], distanceToLighthouse[j], cosineBetween(i, j));
                J(row, j) = df(distanceToLighthouse[j], distanceToLighthouse[i], cosineBetween(i, j));
                v(row) = f(distanceToLighthouse[i], distanceToLighthouse[j], cosineBetween(i, j),
                           distanceBetween(i, j));
                row++;
            }
        }
        for (uint i = 0; i < ids.size(); i++) {
            d_old(i) = distanceToLighthouse[i];
        }
        // ROS_INFO_STREAM("J\n" << J);
//        ROS_INFO_STREAM("v\n" << v);
//        ROS_INFO_STREAM("d_old\n" << d_old);
        error = v.norm();
        if (error < 0.0001) {
            ROS_INFO_STREAM(
                    "error below threshold in " << iterations << " iterations\n" << "v: " << v << "\nd: " << d_old);
            uint i = 0;
            for (auto id:ids) {
                ROS_INFO_STREAM("sensor:" << id << " distance to lighthouse " << lighthouse << ": " << d_old(i));

                // sweep planes
                Eigen::Vector3d vertic(axis_offset, -cos(elevations[i]), -sin(elevations[i]));
                Eigen::Vector3d horizo(cos(azimuths[i]), axis_offset, -sin(azimuths[i]));

                // calc normals
                Eigen::Vector3d n_horizo = horizo.cross(Eigen::Vector3d::UnitY());
                n_horizo.normalize();
                Eigen::Vector3d n_vertic = vertic.cross(Eigen::Vector3d::UnitX());
                n_vertic.normalize();

                // calc line direction from cross product of hyperplane normals
                Eigen::Vector3d u0 = n_horizo.cross(n_vertic);

                Vector3d relLocation(d_old(i) * u0(0), d_old(i) * u0(1), d_old(i) * u0(2));
                sensors[id].set(lighthouse, relLocation);

                i++;

                publishSphere(relLocation, (lighthouse ? "lighthouse2" : "lighthouse1"), "sensor_location_estimated",
                              getMessageID(DISTANCE, id, lighthouse), COLOR(0, 1, lighthouse ? 0 : 1, 0.8), 0.01f, 100);

                Vector3d pos(0, 0, 0);
                publishRay(pos, relLocation, (lighthouse ? "lighthouse2" : "lighthouse1"), "rays",
                           getMessageID(RAY, id, lighthouse), COLOR(0, 1, lighthouse ? 0 : 1, 1), 100);
            }
            return true;
        }
        // construct distance new vector, sharing data with the stl container
        Map<VectorXd> d_new(distanceToLighthouse.data(), distanceToLighthouse.size());
        d_new = d_old - (J.transpose() * J).inverse() * J.transpose() * v;
        iterations++;
    }
    if (iterations == MAX_ITERATIONS) {
        ROS_WARN_STREAM("maximum number of iterations reached, error: " << error);
        return false;
    }
}

bool TrackedObject::poseEstimation(tf::Transform &tf) {
    vector<int> ids = {0, 1, 3};
    while (!distanceEstimation(0, &ids)) {
        ROS_INFO_THROTTLE(1, "could not estimate relative distance to lighthouse 0");
    }
    while (!distanceEstimation(1, &ids)) {
        ROS_INFO_THROTTLE(1, "could not estimate relative distance to lighthouse 1");
    }

    PoseMinimizer minimizer;

    uint i = 0;
    for (auto id:ids) {
        Vector3d relLocation0, relLocation1;
        sensors[id].get(0, relLocation0);
        sensors[id].get(1, relLocation1);
        minimizer.pos3D_A.block(0, i, 4, 1) << relLocation0(0), relLocation0(1), relLocation0(2), 1;
        minimizer.pos3D_B.block(0, i, 4, 1) << relLocation1(0), relLocation1(1), relLocation1(2), 1;
        i++;
    }

    VectorXd pose(6);
    pose << 0, 0, 0, 0, 0, 0.001;

    Matrix4d RT_0, RT_1;
    getTransform("lighthouse1", "world_vive", RT_0);
    getTransform("lighthouse2", "world_vive", RT_1);
    if (!m_switch) {
        minimizer.pos3D_A = RT_0 * minimizer.pos3D_A;
        minimizer.pos3D_B = RT_1 * minimizer.pos3D_B;
    } else {
        minimizer.pos3D_A = RT_1 * minimizer.pos3D_A;
        minimizer.pos3D_B = RT_0 * minimizer.pos3D_B;
    }

    NumericalDiff<PoseMinimizer> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseMinimizer>, double> *lm;
    numDiff = new NumericalDiff<PoseMinimizer>(minimizer);
    lm = new LevenbergMarquardt<NumericalDiff<PoseMinimizer>, double>(*numDiff);
    lm->parameters.maxfev = 100;
    lm->parameters.xtol = 1.0e-10;
    int ret = lm->minimize(pose);

    minimizer.getTFtransform(pose, tf);
}

bool TrackedObject::record(bool start) {
    if (start) {
        file.open("record.log");
        if (file.is_open()) {
            ROS_INFO("start recording");
            recording = true;
            return true;
        } else {
            ROS_ERROR("could not open file");
            return false;
        }
    } else {
        ROS_INFO("saving to file recording");
        recording = false;
        file.close();
        return true;
    }
}

bool TrackedObject::readConfig(string filepath) {
    YAML::Node config = YAML::LoadFile(filepath);
    objectID = config["ObjectID"].as<int>();
    name = config["name"].as<string>();
    mesh = config["mesh"].as<string>();
    vector<float> zero_pose_vector = config["zero_pose"].as<vector<float>>();
    zero_pose[0] = zero_pose_vector[0];
    zero_pose[1] = zero_pose_vector[1];
    zero_pose[2] = zero_pose_vector[2];
    zero_pose[3] = zero_pose_vector[3];
    vector<vector<float>> relative_locations = config["sensor_relative_locations"].as<vector<vector<float>>>();
    sensors.clear();
    for (int i = 0; i < relative_locations.size(); i++) {
        sensors[relative_locations[i][0]].setRelativeLocation(
                Vector3d(relative_locations[i][1], relative_locations[i][2],
                         relative_locations[i][3]));
    }
    return true;
}

bool TrackedObject::writeConfig(string filepath) {
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        ROS_WARN_STREAM("Could not write config " << filepath);
        return false;
    }

    YAML::Node config;
    config["ObjectID"] = objectID;
    config["name"] = name;
    config["mesh"] = mesh;
    YAML::Node node = YAML::Load("[0, 0, 0, 0]");
    node[0] = zero_pose[0];
    node[1] = zero_pose[1];
    node[2] = zero_pose[2];
    node[3] = zero_pose[3];
    config["zero_pose"] = node;
    for (auto &sensor : sensors) {
        Vector3d relative_location;
        sensor.second.getRelativeLocation(relative_location);

        // first number is the sensor id
        node[0] = sensor.first;
        for (int i = 1; i <= 3; i++) {
            node[i] = relative_location(i - 1);
        }
        config["sensor_relative_locations"].push_back(node);
    }
    fout << config;
    return true;
}

bool TrackedObject::rebootESP() {
    DarkRoomProtobuf::commandObject msg;
    msg.set_command(RESET);
    return command_socket->sendMessage<DarkRoomProtobuf::commandObject>(msg);
}

bool TrackedObject::toggleMPU6050(bool toggle) {
    DarkRoomProtobuf::commandObject msg;
    int command = MPU;
    command |= toggle << 4;
    msg.set_command(command);
    return command_socket->sendMessage<DarkRoomProtobuf::commandObject>(msg);
}

bool TrackedObject::toggleTracking(bool toggle) {
    DarkRoomProtobuf::commandObject msg;
    int command = TRACKING;
    command |= toggle << 4;
    msg.set_command(command);
    return command_socket->sendMessage<DarkRoomProtobuf::commandObject>(msg);
}

void TrackedObject::receiveSensorData() {
    while (receiveData && connected) {
        unsigned short timestamp;
        vector<uint> id, lighthouse, rotor, sweepDuration;
        if (sensor_socket->receiveSensorData(timestamp, id, lighthouse, rotor,
                                             sweepDuration)) { // if we receive new data
            for (uint i = 0; i < id.size(); i++) {
                sensors[id[i]].update(lighthouse[i], rotor[i], timestamp, uSecsToRadians(sweepDuration[i]));
                ROS_INFO_THROTTLE(1, "received sensor data");
                if (recording) {
                    file << "\n---------------------------------------------\n"
                         << "timestamp:     " << timestamp << endl
                         << "id:            " << id[i] << endl
                         << "lighthouse:    " << lighthouse[i] << endl
                         << "rotor:         " << rotor[i] << endl
                         << "sweepDuration: " << id[i];
                }
            }
        }
    }
}

void TrackedObject::receiveSensorDataRoboy(const roboy_communication_middleware::DarkRoom::ConstPtr &msg){
    unsigned short timestamp = (unsigned short)(ros::Time::now().sec&0xFF);
    for(uint32_t const &data:msg->sensor_value) {
        static uint id, lighthouse, rotor, sweepDuration;
//        ROS_INFO("\nsync0:  %d\nsync1:  %d\n",(int)(data&0xff),(int)((data>>16)&0xff));
//        if (recording) {
//            file << (int)(data&0xff) << " " <<  (int)((data>>16)&0xff) << endl;
//        }

//        ROS_INFO("lighthouse %d",data);

//        uint32_t val = data;
//        if(val>8400) {
//            val %= 8333;
//            if(val > 5000)
//                lighthouse = 0;
//            else {
//                val-=8333;
//                if(val>300)
//                    lighthouse = 1;
//                else
//                    lighthouse = 0;
//            }
//        }else if( val < 8000){
//            lighthouse = 0;
//        }
//        ROS_INFO("lighthouse %d          %d", lighthouse, data);

//        std::bitset<32> x(data);
//        ROS_INFO_STREAM(x);
        int valid = (data >> 12) & 0x01;
        id = (data & 0x01FF);
        lighthouse = (data >> 9) & 0x01;
        rotor = (data >> 10) & 0x01;
        sweepDuration = (data >> 13) & 0x07FFFF;
        ROS_INFO_STREAM_THROTTLE(1,"timestamp:     " << timestamp << endl <<
                "valid:         " << valid << endl <<
                "id:            " << id << endl <<
                "lighthouse:    " << lighthouse << endl <<
                "rotor:         " << rotor << endl <<
                "sweepDuration: " << sweepDuration);
        if (valid == 1) {
            if (recording) {
                file << "\n---------------------------------------------\n"
                     << "timestamp:     " << timestamp << endl
                     << "id:            " << id << endl
                     << "lighthouse:    " << lighthouse << endl
                     << "rotor:         " << rotor << endl
                     << "sweepDuration: " << sweepDuration;
            }
            ROS_WARN_THROTTLE(5,"receiving sensor data");
            sensors[id].update(lighthouse,rotor,timestamp,uSecsToRadians(sweepDuration));
        }else{
            ROS_WARN_THROTTLE(5,"receiving sensor data, but it's not valid");
        }
    }
}

void TrackedObject::receiveImuData() {
    while (receiveData && connected) {
        DarkRoomProtobuf::imuObject msg;
        if (imu_socket->receiveMessage<DarkRoomProtobuf::imuObject>(msg)) { // if we receive new data
            ROS_INFO_THROTTLE(1, "received imu data");
//            ROS_INFO_STREAM(msg.DebugString());
            Vector3d pos(0,0,0);
            sensors[0].getPosition3D(pos);
            pose = Vector4d(msg.quaternion(0),msg.quaternion(1),msg.quaternion(2),msg.quaternion(3));
            publishMesh(pos, pose, mesh.c_str(), "world", name.c_str(), 66666, 0);
        }
    }
}

void TrackedObject::trackSensors() {
    ros::Duration duration(2);
    duration.sleep();

    ros::Time timestamp0_new[2], timestamp1_new[2];
    map<int, ros::Time[2]> timestamps0_old, timestamps1_old;
    while (tracking) {
        for (auto &sensor : sensors) {
            if (sensor.second.isActive(0) && sensor.second.isActive(1)) {
                Vector2d lighthouse0_angles;
                Vector2d lighthouse1_angles;
                sensor.second.get(0, lighthouse0_angles, timestamp0_new);
                sensor.second.get(1, lighthouse1_angles, timestamp1_new);

                // check if this is actually new data
                if (timestamps0_old[sensor.first][0] != timestamp0_new[0] &&
                    timestamps0_old[sensor.first][1] != timestamp0_new[1] &&
                    timestamps1_old[sensor.first][0] != timestamp1_new[0] &&
                    timestamps1_old[sensor.first][1] != timestamp1_new[1]) {

                    timestamps0_old[sensor.first][0] = timestamp0_new[0];
                    timestamps0_old[sensor.first][1] = timestamp0_new[1];
                    timestamps1_old[sensor.first][0] = timestamp1_new[0];
                    timestamps1_old[sensor.first][1] = timestamp1_new[1];

                    Vector3d triangulated_position, ray0, ray1;
                    triangulateFromLighthousePlanes(lighthouse0_angles, lighthouse1_angles, triangulated_position, ray0,
                                                    ray1);

                    sensor.second.set(triangulated_position);

                    if(!isnan(triangulated_position[0]) && !isnan(triangulated_position[1]) && !isnan(triangulated_position[2]))
                        publishSphere(triangulated_position, "world", "sensor_location",
                                  getMessageID(TRIANGULATED, sensor.first), COLOR(0, 1, 0, 1), 0.05f, 1);

                    if (rays) {
                        Vector3d pos(0, 0, 0);
                        ray0 *= 5;
                        publishRay(pos, ray0, "lighthouse1", "rays", getMessageID(RAY, sensor.first, 0),
                                   COLOR(1, 0, 0, 1.0), 1);
                        ray1 *= 5;
                        publishRay(pos, ray1, "lighthouse2", "rays", getMessageID(RAY, sensor.first, 1),
                                   COLOR(1, 0, 0, 1.0), 1);
                    }
                }
            } else {
                ROS_INFO_THROTTLE(1, "sensor %d not active", sensor.first);
            }
        }
    }
}

void TrackedObject::calibrate() {
    // stop tracking
    startTracking(false);

    map<int, vector<Vector3d>> sensorPosition3d;
    map<int, int> number_of_samples;

    // measure the sensor location for a couple of seconds
    ros::Time start_time = ros::Time::now();
    clearAll();

    ros::Time timestamp0_new[2], timestamp1_new[2];
    map<int, ros::Time[2]> timestamps0_old, timestamps1_old;

    clearAll();
    ROS_INFO("measuring mean sensor positions for 30 seconds");
    int message_counter = 0;
    while ((ros::Time::now() - start_time) < ros::Duration(30) && calibrating) {
        uint done_counter = 0;
        for (auto &sensor : sensors) {
            // if the sensor is visible for both lighthouses and it is active
            if (sensor.second.isActive(0) && sensor.second.isActive(1)) {
                Vector2d lighthouse0_angles;
                Vector2d lighthouse1_angles;
                // triangulate the locations
                sensor.second.get(0, lighthouse0_angles, timestamp0_new);
                sensor.second.get(1, lighthouse1_angles, timestamp1_new);

                // check if this is actually new data
                if (timestamps0_old[sensor.first][0] != timestamp0_new[0] &&
                    timestamps0_old[sensor.first][1] != timestamp0_new[1] &&
                    timestamps1_old[sensor.first][0] != timestamp1_new[0] &&
                    timestamps1_old[sensor.first][1] != timestamp1_new[1]) {

                    timestamps0_old[sensor.first][0] = timestamp0_new[0];
                    timestamps0_old[sensor.first][1] = timestamp0_new[1];
                    timestamps1_old[sensor.first][0] = timestamp1_new[0];
                    timestamps1_old[sensor.first][1] = timestamp1_new[1];

                    Vector3d position, ray0, ray1;
                    triangulateFromLighthousePlanes(lighthouse0_angles, lighthouse1_angles, position, ray0, ray1);

                    sensorPosition3d[sensor.first].push_back(position);
                    number_of_samples[sensor.first]++;

                    publishSphere(position, "world", "sensor_calibration", message_counter++, COLOR(1, 0, 0, 0.1),
                                  0.005f, 100);

                    printf(".");
                }
            }
        }
    }

    toggleMPU6050(true);
    while ((ros::Time::now() - start_time) < ros::Duration(5) && calibrating){
        ROS_INFO_THROTTLE(1,"waiting a couple of seconds for fresh imu data");
    }
    zero_pose = pose;
    toggleMPU6050(false);

    // start tracking again
    startTracking(true);

    // calculate the mean and covariance for each sensor and origin of the object (the origin is the average center)
    origin = Vector3d(0, 0, 0);
    map<int, Vector3d> mean;
    map<int, Vector3d> variance;
    int active_sensors = 0;
    for (auto const &sensor : sensors) {
        if (number_of_samples[sensor.first] > 0) {
            mean[sensor.first] = Vector3d(0.0, 0.0, 0.0);
            for (auto pos:sensorPosition3d[sensor.first]) {
                mean[sensor.first] += pos;
            }
            mean[sensor.first] /= number_of_samples[sensor.first];
            variance[sensor.first] = Vector3d(0.0, 0.0, 0.0);
            for (auto pos:sensorPosition3d[sensor.first]) {
                variance[sensor.first](0) += pow(pos.x() - mean[sensor.first].x(), 2.0);
                variance[sensor.first](1) += pow(pos.y() - mean[sensor.first].y(), 2.0);
                variance[sensor.first](2) += pow(pos.z() - mean[sensor.first].z(), 2.0);
            }
            variance[sensor.first] /= number_of_samples[sensor.first];
            ROS_INFO_STREAM("\nsensor " << sensor.first << "\nmean " << mean[sensor.first] << "\nvariance "
                                        << variance[sensor.first]);
            active_sensors++;
        }
        origin += mean[sensor.first];
    }
    origin /= active_sensors;
    publishSphere(origin, "world", "origin", message_counter++, COLOR(0, 0, 1, 1.0), 0.01f);
    // set the relative sensor location for each sensor wrt the origin
    for (auto &sensor : sensors) {
        if (number_of_samples[sensor.first] > 0) {
            Vector3d rel;
            rel = mean[sensor.first] - origin;
            sensor.second.setRelativeLocation(rel);
            ROS_INFO_STREAM(rel.norm());
            publishSphere(mean[sensor.first], "world", "mean", message_counter++, COLOR(0, 0, 1, 1.0), 0.01f);
            publishRay(origin, rel, "world", "relative", message_counter++, COLOR(1, 1, 0, 1.0));
        }
    }
    writeConfig(path + "/protoType3.yaml");
}

void TrackedObject::publishRelativeFrame() {
    ros::Rate rate(10);
    while (publishingRelativeFrame) {
        tf_broadcaster.sendTransform(tf::StampedTransform(relativeFrame, ros::Time::now(), "world_vive", name.c_str()));
        rate.sleep();
    }
}

bool
TrackedObject::triangulateFromLighthousePlanes(Vector2d &angles0, Vector2d &angles1, Vector3d &triangulated_position,
                                               Vector3d &ray0, Vector3d &ray1) {
    Matrix4d RT_0, RT_1;
    if (!getTransform("world", "lighthouse1", RT_0))
        return false;
    if (!getTransform("world", "lighthouse2", RT_1))
        return false;

    // generate the projection matrices
    Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
    proj_matrix0 = RT_0.topLeftCorner(3, 4);
    proj_matrix1 = RT_1.topLeftCorner(3, 4);

    double azimuth0 = angles0(1), azimuth1 = angles1(1), elevation0 = angles0(0), elevation1 = angles1(0);
    Eigen::Vector3d vec0_vertic(axis_offset, -cos(elevation0), -sin(elevation0));
    Eigen::Vector3d vec0_horizo(cos(azimuth0), axis_offset, -sin(azimuth0));

    Eigen::Vector3d vec1_vertic(axis_offset, -cos(elevation1), -sin(elevation1));
    Eigen::Vector3d vec1_horizo(cos(azimuth1), axis_offset, -sin(azimuth1));
    // calc normals lighthouse 0
    Eigen::Vector3d n0_horizo = vec0_horizo.cross(Eigen::Vector3d::UnitY());
    n0_horizo.normalize();
    Eigen::Vector3d n0_vertic = vec0_vertic.cross(Eigen::Vector3d::UnitX());
    n0_vertic.normalize();
    // calc normals lighthouse 1
    Eigen::Vector3d n1_horizo = vec1_horizo.cross(Eigen::Vector3d::UnitY());
    n1_horizo.normalize();
    Eigen::Vector3d n1_vertic = vec1_vertic.cross(Eigen::Vector3d::UnitX());
    n1_vertic.normalize();
    // calc line direction from cross product of hyperplane normals
    Eigen::Vector3d u0 = n0_horizo.cross(n0_vertic);
    Eigen::Vector3d u1 = n1_horizo.cross(n1_vertic);

    ray0 = Eigen::Vector3d(u0(0), u0(1), u0(2));
    ray1 = Eigen::Vector3d(u1(0), u1(1), u1(2));

    // project onto image plane
    Eigen::Vector2d projected_image_location0 = Eigen::Vector2d(ray0(0) / ray0(2), ray0(1) / ray0(2));
    Eigen::Vector2d projected_image_location1 = Eigen::Vector2d(ray1(0) / ray1(2), ray1(1) / ray1(2));

    triangulated_position = triangulate_point(proj_matrix0, proj_matrix1,
                                              projected_image_location0, projected_image_location1);

    return true;
}

bool TrackedObject::getTransform(const char *from, const char *to, Matrix4d &transform) {
    tf::StampedTransform trans;
    try {
        tf_listener.lookupTransform(to, from, ros::Time(0), trans);
    }
    catch (tf::TransformException ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    Eigen::Affine3d trans_;
    tf::transformTFToEigen(trans, trans_);
    transform = trans_.matrix();
    return true;
}

void TrackedObject::clearAll() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETEALL;
    visualization_pub.publish(marker);
}

int TrackedObject::getMessageID(int type, int sensor, bool lighthouse) {
    return trackeObjectInstance * 5 * sensors.size() +
           type * (sensors.size() * NUMBER_OF_LIGHTHOUSES) + sensors.size() * lighthouse + sensor;
}

void TrackedObject::publishMesh(Vector3d &pos, Vector4d& orientation, const char* modelname,
                                const char *frame, const char *ns, int message_id, int duration) {
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = frame;
    mesh.ns = ns;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = 1.0;
    mesh.scale.y = 1.0;
    mesh.scale.z = 1.0;
    mesh.lifetime = ros::Duration(duration);
    mesh.header.stamp = ros::Time::now();
    mesh.action = visualization_msgs::Marker::ADD;
    mesh.id = message_id;
    mesh.pose.position.x = pos[0];
    mesh.pose.position.y = pos[1];
    mesh.pose.position.z = pos[2];
    mesh.pose.orientation.x = orientation[0];
    mesh.pose.orientation.y = orientation[1];
    mesh.pose.orientation.z = orientation[2];
    mesh.pose.orientation.w = orientation[3];
    char meshpath[200];
    sprintf(meshpath,"package://darkroom/calibrated_objects/models/%s.dae", modelname);
    mesh.mesh_resource = meshpath;
    visualization_pub.publish(mesh);
}

void TrackedObject::publishSphere(Vector3d &pos, const char *frame, const char *ns, int message_id, COLOR color,
                                  float radius, int duration) {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = frame;
    sphere.ns = ns;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.color.r = color.r;
    sphere.color.g = color.g;
    sphere.color.b = color.b;
    sphere.color.a = color.a;
    sphere.lifetime = ros::Duration(duration);
    sphere.scale.x = radius;
    sphere.scale.y = radius;
    sphere.scale.z = radius;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.header.stamp = ros::Time::now();
    sphere.id = message_id;
    sphere.pose.position.x = pos(0);
    sphere.pose.position.y = pos(1);
    sphere.pose.position.z = pos(2);
    visualization_pub.publish(sphere);
}

void TrackedObject::publishCube(Vector3d &pos, Vector4d &quat, const char *frame, const char *ns, int message_id, COLOR color,
                                  float radius, int duration) {
    visualization_msgs::Marker cube;
    cube.header.frame_id = frame;
    cube.ns = ns;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.color.r = color.r;
    cube.color.g = color.g;
    cube.color.b = color.b;
    cube.color.a = color.a;
    cube.lifetime = ros::Duration(duration);
    cube.scale.x = radius;
    cube.scale.y = radius;
    cube.scale.z = radius;
    cube.action = visualization_msgs::Marker::ADD;
    cube.header.stamp = ros::Time::now();
    cube.id = message_id;
    cube.pose.position.x = pos(0);
    cube.pose.position.y = pos(1);
    cube.pose.position.z = pos(2);
    cube.pose.orientation.x = quat(0);
    cube.pose.orientation.y = quat(1);
    cube.pose.orientation.z = quat(2);
    cube.pose.orientation.w = quat(3);
    visualization_pub.publish(cube);
}

void
TrackedObject::publishRay(Vector3d &pos, Vector3d &dir, const char *frame, const char *ns, int message_id, COLOR color,
                          int duration) {
    visualization_msgs::Marker arrow;
    arrow.ns = ns;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.r = color.r;
    arrow.color.g = color.g;
    arrow.color.b = color.b;
    arrow.color.a = color.a;
    arrow.lifetime = ros::Duration(duration);
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.03;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.header.stamp = ros::Time::now();

    arrow.header.frame_id = frame;
    arrow.id = message_id;
    arrow.points.clear();
    geometry_msgs::Point p;
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    arrow.points.push_back(p);
    p.x += dir(0);
    p.y += dir(1);
    p.z += dir(2);
    arrow.points.push_back(p);
    visualization_pub.publish(arrow);
}
