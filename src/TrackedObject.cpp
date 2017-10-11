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

    sensor_location_pub = nh->advertise<roboy_communication_middleware::DarkRoomSensor>(
            "/roboy/middleware/DarkRoom/sensor_location", 1);
    visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);
    lighthouse_pose_correction = nh->advertise<roboy_communication_middleware::LighthousePoseCorrection>(
            "/roboy/middleware/DarkRoom/LighthousePoseCorrection", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();


    if (const char *env_p = getenv("DARKROOM_CALIBRATED_OBJECTS")) {
        path = env_p;
        ROS_INFO_STREAM("using DARKROOM_CALIBRATED_OBJECTS: " << path);
        readConfig(path + "/protoType4.yaml");
    } else
        ROS_WARN("could not get DARKROOM_CALIBRATED_OBJECTS environmental variable");

    trackeObjectInstance++;

}

TrackedObject::~TrackedObject() {
    receiveData = false;
    tracking = false;
    calibrating = false;
    poseestimating = false;
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
    if (poseestimation_thread != nullptr) {
        if (poseestimation_thread->joinable()) {
            ROS_INFO("Waiting for pose estimation thread to terminate");
            poseestimation_thread->join();
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

void TrackedObject::connectRoboy() {
    receiveData = true;
    sensor_sub = nh->subscribe("/roboy/middleware/DarkRoom/sensors", 1, &TrackedObject::receiveSensorDataRoboy, this);
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
    if (command_socket != nullptr)
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

void TrackedObject::startPoseestimation(bool start) {
    if (start) {
        ROS_INFO("starting pose estimation thread");
        poseestimating = true;
        poseestimation_thread = boost::shared_ptr<thread>(new thread(&TrackedObject::poseEstimation4, this));
        poseestimation_thread->detach();
    } else {
        if (poseestimation_thread != nullptr) {
            poseestimating = false;
            if (poseestimation_thread->joinable()) {
                ROS_INFO("Waiting for pose estimation thread to terminate");
                poseestimation_thread->join();
            }
        }
    }
}

void TrackedObject::showRays(bool show) {
    rays = show;
}

void TrackedObject::showDistances(bool show) {
    distances = show;
}

void TrackedObject::map_lighthouse_id(bool switchID) {
    ROS_INFO_STREAM("switching lighthouses " << switchID);
    m_switch = switchID;
    for (auto &sensor : sensors) {
        sensor.second.switchLighthouses(switchID);
    }
}

bool TrackedObject::distanceEstimation(bool lighthouse, vector<int> &specificIds) {
    ROS_INFO_STREAM("estimating distance of sensors to lighthouse " << lighthouse + 1);
    vector<Vector3d> relPos;
    vector<double> elevations, azimuths;
    vector<double> distanceToLighthouse;
    vector<int> ids;
    if (specificIds.empty()) {
        // let's see who is active
        for (auto &sensor : sensors) {
            // skip inactive sensors
            if (sensor.second.isActive(lighthouse)) {
                ids.push_back(sensor.first);
                sensor.second.get(lighthouse, elevations, azimuths);
                sensor.second.getRelativeLocation(relPos);
                distanceToLighthouse.push_back(sensor.second.getDistance(lighthouse));
                cout << sensor.first << endl;
            }
        }
    } else {
        uint sensor_counter = 0;
        for (uint i = 0; i < specificIds.size(); i++) {
            // skip inactive sensors
            if (sensors[specificIds.at(i)].isActive(lighthouse)) {
                sensor_counter++;
            } else {
                ROS_WARN_THROTTLE(1, "sensor%d inactive", specificIds.at(i));
            }
        }
        if (sensor_counter < specificIds.size()) {
            ROS_WARN("time out waiting for specific sensors");
            return false;
        }
        // get the values now that all requested sensors are active
        for (uint i = 0; i < specificIds.size(); i++) {
            ids.push_back(specificIds.at(i));
            sensors[specificIds.at(i)].get(lighthouse, elevations, azimuths);
            sensors[specificIds.at(i)].getRelativeLocation(relPos);
            distanceToLighthouse.push_back(sensors[specificIds.at(i)].getDistance(lighthouse));
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

            ROS_INFO("cosine between %d and %d: %f", i, j, cosineBetween(i, j));
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
        if(iterations%100==0){
//            ROS_INFO_STREAM("J\n" << J);
//            ROS_INFO_STREAM("v\n" << v);
//            ROS_INFO_STREAM("d_old\n" << d_old);
        }

        error = v.norm() / (double) ids.size();
        if (error < ERROR_THRESHOLD) {
            ROS_INFO_STREAM(
                    "mean squared error " << error << " below threshold " << ERROR_THRESHOLD << " in " << iterations
                                          << " iterations");
            break;
        }
        // construct distance new vector, sharing data with the stl container
        Map<VectorXd> d_new(distanceToLighthouse.data(), distanceToLighthouse.size());
        d_new = d_old - (J.transpose() * J).inverse() * J.transpose() * v;
        iterations++;
    }

    uint i = 0;
    for (auto id:ids) {
        ROS_DEBUG_STREAM("sensor:" << id << " distance to lighthouse " << lighthouse << ": " << d_old(i));

        Vector2d angles(azimuths[i], elevations[i]);
        Eigen::Vector3d u0;
        rayFromLighthouseAngles(angles,u0);

        Vector3d relLocation(d_old(i) * u0(0), d_old(i) * u0(1), d_old(i) * u0(2));
        sensors[id].set(lighthouse, relLocation);

        i++;

        char str[100];
        sprintf(str, "sensor_%d_estimated", id);

        publishSphere(relLocation, (lighthouse ? "lighthouse2" : "lighthouse1"), str,
                      getMessageID(DISTANCE, id, lighthouse), COLOR(0, 1, lighthouse ? 0 : 1, 0.3), 0.01f, 100);

        sprintf(str, "ray_%d", id);
        Vector3d pos(0, 0, 0);
        publishRay(pos, relLocation, (lighthouse ? "lighthouse2" : "lighthouse1"), str,
                   getMessageID(RAY, id, lighthouse), COLOR(0, 1, lighthouse ? 0 : 1, 0.3), 100);

        sprintf(str, "%d", id);
        publishText(relLocation, str, (lighthouse ? "lighthouse2" : "lighthouse1"), "sensor_id", rand(),
                    COLOR(1, 0, 0, 0.5), 0, 0.04f);
    }

    for (auto id:ids) {
        for (auto id2:ids) {
            if (id2 != id) {
                Vector3d pos1, pos2, dir;
                sensors[id].get(lighthouse, pos1);
                sensors[id2].get(lighthouse, pos2);
                dir = pos2 - pos1;
                publishRay(pos1, dir, (lighthouse ? "lighthouse2" : "lighthouse1"), "distance",
                           rand(), COLOR(0, 1, lighthouse ? 0 : 1, 0.5));

                if(distances){
                    char str[100];
                    sprintf(str, "%.3f", dir.norm());
                    Vector3d pos = pos1 + dir / 2.0;
                    publishText(pos, str, (lighthouse ? "lighthouse2" : "lighthouse1"), "distance", rand(),
                                COLOR(1, 0, 0, 0.5), 0, 0.02f);
                }
            }
        }
    }

    ROS_WARN_STREAM("distance estimation terminated after " << iterations << " error: " << error);
//        return true;
//    }
    return true;
}

bool TrackedObject::poseEstimation(tf::Transform &tf) {
    ros::Time start_time = ros::Time::now();
    while (!distanceEstimation(0, calibrated_sensors)) {
        ROS_INFO_THROTTLE(1,
                          "could not estimate relative distance to lighthouse 0, are the sensors visible to lighthouse 0?!");
        if ((ros::Time::now() - start_time).sec > 10) {
            ROS_WARN("time out");
            return false;
        }
    }
    start_time = ros::Time::now();
    while (!distanceEstimation(1, calibrated_sensors)) {
        ROS_INFO_THROTTLE(1,
                          "could not estimate relative distance to lighthouse 1, are the sensors visible to lighthouse 1?!");
        if ((ros::Time::now() - start_time).sec > 10) {
            ROS_WARN("time out");
            return false;
        }
    }

    int number_of_sensors = 0;
    if (calibrated_sensors.empty())
        number_of_sensors = sensors.size();
    else
        number_of_sensors = calibrated_sensors.size();

    PoseMinimizer minimizer(number_of_sensors);


    if (calibrated_sensors.empty()) {
        uint i = 0;
        for (auto &sensor:sensors) {
            Vector3d relLocation0, relLocation1;
            sensors[sensor.first].get(0, relLocation0);
            sensors[sensor.first].get(1, relLocation1);
            minimizer.pos3D_A.block(0, i, 4, 1) << relLocation0(0), relLocation0(1), relLocation0(2), 1;
            minimizer.pos3D_B.block(0, i, 4, 1) << relLocation1(0), relLocation1(1), relLocation1(2), 1;
            i++;
        }
    } else {
        uint i = 0;
        for (auto &sensor:calibrated_sensors) {
            Vector3d relLocation0, relLocation1;
            sensors[sensor].get(0, relLocation0);
            sensors[sensor].get(1, relLocation1);
            minimizer.pos3D_A.block(0, i, 4, 1) << relLocation0(0), relLocation0(1), relLocation0(2), 1;
            minimizer.pos3D_B.block(0, i, 4, 1) << relLocation1(0), relLocation1(1), relLocation1(2), 1;
            i++;
        }
    }

    VectorXd pose(6);
    pose << 0, 0, 0, 0, 0, 0.001;

    Matrix4d RT_0, RT_1;
    getTransform("lighthouse1", "world", RT_0);
    getTransform("lighthouse2", "world", RT_1);
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
    lm->parameters.maxfev = MAX_ITERATIONS;
    lm->parameters.xtol = 1.0e-10;
    int ret = lm->minimize(pose);
    ROS_INFO("pose estimation finished after %ld iterations, with an error of %f", lm->iter, lm->fnorm);

    minimizer.getTFtransform(pose, tf);
}

bool TrackedObject::poseEstimation2() {
    ros::Time start_time = ros::Time::now();
    int numberOfSamples = 0;
    ros::Rate rate(20);

    MatrixXd rays0_A(3, NUMBER_OF_SAMPLES), rays1_A(3, NUMBER_OF_SAMPLES),
            rays0_B(3, NUMBER_OF_SAMPLES), rays1_B(3, NUMBER_OF_SAMPLES);

    while (numberOfSamples < NUMBER_OF_SAMPLES) {
        Vector2d angles0, angles1;
        Vector3d ray0, ray1;

        // get the rays for the first sensor
        sensors[17].get(0, angles0);
        sensors[17].get(1, angles1);
        rayFromLighthouseAngles(angles0, ray0);
        rayFromLighthouseAngles(angles1, ray1);

        rays0_A(0, numberOfSamples) = ray0(0);
        rays0_A(1, numberOfSamples) = ray0(1);
        rays0_A(2, numberOfSamples) = ray0(2);

        rays0_B(0, numberOfSamples) = ray1(0);
        rays0_B(1, numberOfSamples) = ray1(1);
        rays0_B(2, numberOfSamples) = ray1(2);

        Vector3d zero(0, 0, 0);
        publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 1 + numberOfSamples, COLOR(0, 1, 0, 0.1));
        publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 2 + numberOfSamples, COLOR(0, 1, 0, 0.1));

        // get the rays for the second sensor
        sensors[18].get(0, angles0);
        sensors[18].get(1, angles1);
        rayFromLighthouseAngles(angles0, ray0);
        rayFromLighthouseAngles(angles1, ray1);

        rays1_A(0, numberOfSamples) = ray0(0);
        rays1_A(1, numberOfSamples) = ray0(1);
        rays1_A(2, numberOfSamples) = ray0(2);

        rays1_B(0, numberOfSamples) = ray1(0);
        rays1_B(1, numberOfSamples) = ray1(1);
        rays1_B(2, numberOfSamples) = ray1(2);

        publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 3 + numberOfSamples, COLOR(0, 0, 1, 0.1));
        publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 4 + numberOfSamples, COLOR(0, 0, 1, 0.1));

        numberOfSamples++;
        rate.sleep();
    }

//    if (!m_switch) {
//        ray0_A = RT_0 * ray0_A;
//        ray0_B = RT_1 * ray0_B;
//        ray1_A = RT_0 * ray1_A;
//        ray1_B = RT_1 * ray1_B;
//    } else {
//        ray0_A = RT_1 * ray0_A;
//        ray0_B = RT_0 * ray0_B;
//        ray1_A = RT_1 * ray1_A;
//        ray1_B = RT_0 * ray1_B;
//    }

    LighthousePoseEstimator::LighthousePoseMinimizer minimizer(NUMBER_OF_SAMPLES, 0.237, rays0_A, rays0_B, rays1_A,
                                                               rays1_B);
    Matrix4d RT_0, RT_1;
    getTransform("world", "lighthouse1", RT_0);
    getTransform("world", "lighthouse2", RT_1);
    if (!m_switch) {
        minimizer.RT_A = RT_0;
        minimizer.RT_B = RT_1;
    } else {
        minimizer.RT_A = RT_1;
        minimizer.RT_B = RT_0;
    }

    VectorXd pose(6);
    pose << 0, 0, 0, 0, 0, 0;

    NumericalDiff<LighthousePoseEstimator::LighthousePoseMinimizer> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<LighthousePoseEstimator::LighthousePoseMinimizer>, double> *lm;
    numDiff = new NumericalDiff<LighthousePoseEstimator::LighthousePoseMinimizer>(minimizer);
    lm = new LevenbergMarquardt<NumericalDiff<LighthousePoseEstimator::LighthousePoseMinimizer>, double>(*numDiff);
    lm->parameters.maxfev = MAX_ITERATIONS;
    lm->parameters.xtol = 1.0e-10;
    int ret = lm->minimize(pose);

    Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
    proj_matrix0 = RT_0.topLeftCorner(3, 4);

    Matrix4d RT_1_corrected = MatrixXd::Identity(4, 4);
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(pose(0), 2.0) + pow(pose(1), 2.0) + pow(pose(2), 2.0), 2.0);
    Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                  2.0 * pose(0) / (alpha_squared + 1),
                  2.0 * pose(1) / (alpha_squared + 1),
                  2.0 * pose(2) / (alpha_squared + 1));
    q.normalize();
    // construct RT matrix
    RT_1_corrected.topLeftCorner(3, 3) = q.toRotationMatrix();
    RT_1_corrected.topRightCorner(3, 1) << pose(3), pose(4), pose(5);
    RT_1_corrected = RT_1_corrected * RT_1;
    proj_matrix1 = RT_1_corrected.topLeftCorner(3, 4);

//        cout << "fvec" << endl;
    for (int i = 0; i < numberOfSamples; i++) {
        // project onto image plane
        Eigen::Vector2d projected_image_location0 = Eigen::Vector2d(rays0_A(0, i) / rays0_A(2, i),
                                                                    rays0_A(1, i) / rays0_A(2, i));
        Eigen::Vector2d projected_image_location1 = Eigen::Vector2d(rays0_B(0, i) / rays0_B(2, i),
                                                                    rays0_B(1, i) / rays0_B(2, i));

        Vector3d pos0 = triangulate_point(proj_matrix0, proj_matrix1,
                                          projected_image_location0, projected_image_location1);

        projected_image_location0 = Eigen::Vector2d(rays1_A(0, i) / rays1_A(2, i),
                                                    rays1_A(1, i) / rays1_A(2, i));
        projected_image_location1 = Eigen::Vector2d(rays1_B(0, i) / rays1_B(2, i),
                                                    rays1_B(1, i) / rays1_B(2, i));

        Vector3d pos1 = triangulate_point(proj_matrix0, proj_matrix1,
                                          projected_image_location0, projected_image_location1);
        publishSphere(pos0, "world", "tri", rand(), COLOR(1, 0, 0, 1));
        publishSphere(pos1, "world", "tri", rand(), COLOR(1, 0, 0, 1));
        ROS_INFO_STREAM((pos1 - pos0).norm());
    }


    tf::Transform tf;
    minimizer.getTFtransform(pose, tf);

    roboy_communication_middleware::LighthousePoseCorrection msg;
    msg.id = 1;
    tf::transformTFToMsg(tf, msg.tf);
    lighthouse_pose_correction.publish(msg);

    ROS_INFO("pose estimation finished with %d", ret);
}

bool TrackedObject::poseEstimation3() {
    MatrixXd ray_A(4, 3), ray_B(4, 3);

    Vector2d angles0, angles1;
    Vector3d ray0, ray1;

    // get the rays for the first sensor
    sensors[16].get(0, angles0);
    sensors[16].get(1, angles1);
    rayFromLighthouseAngles(angles0, ray0);
    rayFromLighthouseAngles(angles1, ray1);

    ray_A(0, 0) = ray0(0);
    ray_A(1, 0) = ray0(1);
    ray_A(2, 0) = ray0(2);
    ray_A(3, 0) = 1;

    ray_B(0, 0) = ray1(0);
    ray_B(1, 0) = ray1(1);
    ray_B(2, 0) = ray1(2);
    ray_B(3, 0) = 1;

    Vector3d zero(0, 0, 0);
    publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 1, COLOR(1, 0, 0, 0.1));
    publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 2, COLOR(1, 0, 0, 0.1));

    // get the rays for the second sensor
    sensors[17].get(0, angles0);
    sensors[17].get(1, angles1);
    rayFromLighthouseAngles(angles0, ray0);
    rayFromLighthouseAngles(angles1, ray1);

    ray_A(0, 1) = ray0(0);
    ray_A(1, 1) = ray0(1);
    ray_A(2, 1) = ray0(2);
    ray_A(3, 1) = 1;

    ray_B(0, 1) = ray1(0);
    ray_B(1, 1) = ray1(1);
    ray_B(2, 1) = ray1(2);
    ray_B(3, 1) = 1;

    publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 3, COLOR(0, 1, 0, 0.1));
    publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 4, COLOR(0, 1, 0, 0.1));

    // get the rays for the third sensor
    sensors[18].get(0, angles0);
    sensors[18].get(1, angles1);
    rayFromLighthouseAngles(angles0, ray0);
    rayFromLighthouseAngles(angles1, ray1);

    ray_A(0, 2) = ray0(0);
    ray_A(1, 2) = ray0(1);
    ray_A(2, 2) = ray0(2);
    ray_A(3, 2) = 1;

    ray_B(0, 2) = ray1(0);
    ray_B(1, 2) = ray1(1);
    ray_B(2, 2) = ray1(2);
    ray_B(3, 2) = 1;

    publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 5, COLOR(0, 0, 1, 0.1));
    publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 6, COLOR(0, 0, 1, 0.1));

    LighthousePoseEstimator2::LighthousePoseMinimizer minimizer(3, ray_A, ray_B);

    VectorXd pose(6);
    pose << 0, 0, 0, 0, 0, -2;

    NumericalDiff<LighthousePoseEstimator2::LighthousePoseMinimizer> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<LighthousePoseEstimator2::LighthousePoseMinimizer>, double> *lm;
    numDiff = new NumericalDiff<LighthousePoseEstimator2::LighthousePoseMinimizer>(minimizer);
    lm = new LevenbergMarquardt<NumericalDiff<LighthousePoseEstimator2::LighthousePoseMinimizer>, double>(*numDiff);
    lm->parameters.maxfev = 100;
    lm->parameters.xtol = 1.0e-10;
    int ret = lm->minimize(pose);

    tf::Transform tf;
    minimizer.getTFtransform(pose, tf);

    roboy_communication_middleware::LighthousePoseCorrection msg;
    msg.id = 1;
    tf::transformTFToMsg(tf, msg.tf);
    lighthouse_pose_correction.publish(msg);

    ROS_INFO("pose estimation finished with %d", ret);
}

bool TrackedObject::poseEstimation4() {
    ros::Time start_time = ros::Time::now();
    while (!distanceEstimation(0, calibrated_sensors)) {
        ROS_INFO_THROTTLE(1,
                          "could not estimate relative distance to lighthouse 0, are the sensors visible to lighthouse 0?!");
        if ((ros::Time::now() - start_time).sec > 10) {
            ROS_WARN("time out");
            return false;
        }
    }

    epnp PnP;
    PnP.reset_correspondences();
    PnP.set_internal_parameters(0, 0, 1, 1);
    PnP.set_maximum_number_of_correspondences(calibrated_sensors.size());
    for(uint id:calibrated_sensors){
        Vector3d pos, ray;
        Vector2d angles;
        sensors[id].get(0,pos);
        sensors[id].get(0,angles);
        rayFromLighthouseAngles(angles,ray);
        PnP.add_correspondence(pos(0),pos(2),-pos(1),ray(0)/ray(1), ray(2)/ray(1));
    }

    double R_est[3][3], t_est[3];
    double err2 = PnP.compute_pose(R_est, t_est);
    double rot_err, transl_err;

    PnP.print_pose(R_est, t_est);
    cout << "epnp error: " << err2 << endl;

    tf::Transform tf;
    tf.setOrigin(tf::Vector3(t_est[0], t_est[1], t_est[2]));
    tf.setRotation(tf::Quaternion(1,0,0,0));
    tf_broadcaster.sendTransform(tf::StampedTransform(tf,ros::Time::now(),"lighthouse1","object"));

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
    vector<vector<float>> relative_locations = config["sensor_relative_locations"].as<vector<vector<float>>>();
    sensors.clear();
    calibrated_sensors.clear();
    cout << "using calibrated sensors: ";
    for (int i = 0; i < relative_locations.size(); i++) {
        Vector3d relLocation(relative_locations[i][1], relative_locations[i][2], relative_locations[i][3]);
        sensors[relative_locations[i][0]].setRelativeLocation(relLocation);
        calibrated_sensors.push_back((int) relative_locations[i][0]);
        cout << "\t" << calibrated_sensors.back();
    }
    cout << endl;

    return true;
}

bool TrackedObject::rebootESP() {
    DarkRoomProtobuf::commandObject msg;
    msg.set_command(RESET);
    return command_socket->sendMessage<DarkRoomProtobuf::commandObject>(msg);
}

bool TrackedObject::toggleMPU6050(bool toggle) {
    if (command_socket != nullptr) {
        DarkRoomProtobuf::commandObject msg;
        int command = MPU;
        command |= toggle << 4;
        msg.set_command(command);
        return command_socket->sendMessage<DarkRoomProtobuf::commandObject>(msg);
    } else {
        return false;
    }
}

bool TrackedObject::toggleTracking(bool toggle) {
    if (command_socket != nullptr) {
        DarkRoomProtobuf::commandObject msg;
        int command = TRACKING;
        command |= toggle << 4;
        msg.set_command(command);
        return command_socket->sendMessage<DarkRoomProtobuf::commandObject>(msg);
    } else {
        return false;
    }
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

void TrackedObject::receiveSensorDataRoboy(const roboy_communication_middleware::DarkRoom::ConstPtr &msg) {
    ROS_WARN_THROTTLE(5, "receiving sensor data");
    unsigned short timestamp = (unsigned short) (ros::Time::now().sec & 0xFF);
    uint id = 0;
    for (uint32_t const &data:msg->sensor_value) {
        uint lighthouse, rotor, sweepDuration;
        lighthouse = (data >> 31) & 0x1;
        rotor = (data >> 30) & 0x1;
        int valid = (data >> 29) & 0x1;
        sweepDuration = (data & 0x1fffffff); // raw sensor duration is 50 ticks per microsecond
//        ROS_INFO_STREAM_THROTTLE(1,"timestamp:     " << timestamp << endl <<
//                "valid:         " << valid << endl <<
//                "id:            " << id << endl <<
//                "lighthouse:    " << lighthouse << endl <<
//                "rotor:         " << rotor << endl <<
//                "sweepDuration: " << sweepDuration);
        if (valid == 1) {
            if (recording) {
                file << "\n---------------------------------------------\n"
                     << "timestamp:     " << timestamp << endl
                     << "id:            " << id << endl
                     << "lighthouse:    " << lighthouse << endl
                     << "rotor:         " << rotor << endl
                     << "sweepDuration: " << sweepDuration << endl;
            }

            sensors[id].update(lighthouse, rotor, timestamp, ticksToRadians(sweepDuration));
        }
        id++;
    }
}

void TrackedObject::receiveImuData() {
    while (receiveData && connected) {
        DarkRoomProtobuf::imuObject msg;
        if (imu_socket->receiveMessage<DarkRoomProtobuf::imuObject>(msg)) { // if we receive new data
            ROS_INFO_THROTTLE(1, "received imu data");
//            ROS_INFO_STREAM(msg.DebugString());
            Vector3d pos(0, 0, 0);
            sensors[0].getPosition3D(pos);
            pose = Vector4d(msg.quaternion(0), msg.quaternion(1), msg.quaternion(2), msg.quaternion(3));
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
        roboy_communication_middleware::DarkRoomSensor msg;

        Matrix4d RT_0, RT_1;
        if (!getTransform((!m_switch?"lighthouse1":"lighthouse2"), "world",RT_0))
            continue;
        if (!getTransform((!m_switch?"lighthouse2":"lighthouse1"),"world", RT_1))
            continue;

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
                    triangulateFromLighthousePlanes(lighthouse0_angles, lighthouse1_angles, RT_0, RT_1,
                                                    triangulated_position, ray0,
                                                    ray1);

                    sensor.second.set(triangulated_position);

                    if (!triangulated_position.hasNaN()) {
                        char str[100], str2[2];
                        sprintf(str, "sensor_%d", sensor.first);
                        publishSphere(triangulated_position, "world", str,
                                      getMessageID(TRIANGULATED, sensor.first), COLOR(0, 1, 0, 0.8), 0.01f, 0.1);
                        sprintf(str2, "%d", sensor.first);
                        publishText(triangulated_position, str2, "world", str, getMessageID(SENSOR_NAME, sensor.first),
                                    COLOR(1, 0, 0, 0.5), 0.1, 0.04f);
                        msg.ids.push_back(sensor.first);
                        geometry_msgs::Vector3 v;
                        v.x = triangulated_position[0];
                        v.y = triangulated_position[1];
                        v.z = triangulated_position[2];
                        msg.position.push_back(v);
                    }

                    if (rays) {
//                        {
//
//                            Vector3d origin0, origin1;
//                            origin0 = RT_0.topRightCorner(3, 1);
//                            origin1 = RT_1.topRightCorner(3, 1);
//
//                            Vector3d ray0_worldFrame, ray1_worldFrame;
//
//                            ray0_worldFrame = RT_0.topLeftCorner(3, 3)*ray0;
//                            ray1_worldFrame = RT_1.topLeftCorner(3, 3)*ray1;
//
//                            publishRay(origin0, ray0_worldFrame, "world", "rays_lighthouse_1", rand(),
//                                       COLOR(1, 0, 1, 1.0), 1);
//                            publishRay(origin1, ray1_worldFrame, "world", "rays_lighthouse_2", rand(),
//                                       COLOR(1, 0, 1, 1.0), 1);
//                        }
                        Vector3d pos(0, 0, 0);
                        ray0 *= 5;
                        publishRay(pos, ray0, "lighthouse1", "rays_lighthouse_1", getMessageID(RAY, sensor.first, 0),
                                   COLOR(1, 0, 0, 1.0), 1);
                        ray1 *= 5;
                        publishRay(pos, ray1, "lighthouse2", "rays_lighthouse_2", getMessageID(RAY, sensor.first, 1),
                                   COLOR(1, 0, 0, 1.0), 1);

                        for (auto &sensor_other : sensors) {
                            if (sensor.first != sensor_other.first &&
                                (sensor_other.second.isActive(0) && sensor_other.second.isActive(1))) {
                                Vector3d pos1, pos2, dir;
                                sensor_other.second.getPosition3D(pos2);
                                sensor.second.getPosition3D(pos1);
                                dir = pos2 - pos1;
                                publishRay(pos1, dir, "world", "distance",
                                           rand(), COLOR(0, 1, 1, 1.0), 0.01);
                            }
                        }
                    }

                    if (distances) {
                        for (auto &sensor_other : sensors) {
                            if (sensor.first != sensor_other.first &&
                                (sensor_other.second.isActive(0) && sensor_other.second.isActive(1))) {
                                Vector3d pos1, pos2, dir;
                                sensor_other.second.getPosition3D(pos2);
                                sensor.second.getPosition3D(pos1);
                                dir = pos2 - pos1;
                                if (!rays)
                                    publishRay(pos1, dir, "world", "distance",
                                               rand(), COLOR(0, 1, 1, 1.0), 0.01);

                                char str[100];
                                sprintf(str, "%.3f", dir.norm());
                                Vector3d pos = pos1 + dir / 2.0;
                                publishText(pos, str, "world", "distance", rand(),
                                            COLOR(1, 0, 0, 0.5), 0.05, 0.02f);
                            }
                        }
                    }
                }
            } else {
                ROS_INFO_THROTTLE(1, "sensor %d not active", sensor.first);
            }

        }
        if (msg.ids.size() > 0)
            sensor_location_pub.publish(msg);
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
    ROS_INFO("measuring mean sensor positions for 10 seconds");
    int message_counter = 0;

    // get the lighthouse poses
    Matrix4d RT_0, RT_1;
    if (!getTransform("world", "lighthouse1", RT_0))
        return;
    if (!getTransform("world", "lighthouse2", RT_1))
        return;

    while ((ros::Time::now() - start_time) < ros::Duration(10) && calibrating) {
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
                    triangulateFromLighthousePlanes(lighthouse0_angles, lighthouse1_angles, RT_0, RT_1, position, ray0,
                                                    ray1);

                    if (sensorPosition3d[sensor.first].size() > 0) {
                        Vector3d diff = sensorPosition3d[sensor.first].back() - position;
                        if (diff.norm() < 0.1) { // reject outliers
                            sensorPosition3d[sensor.first].push_back(position);
                            number_of_samples[sensor.first]++;
                        }
                    } else {
                        sensorPosition3d[sensor.first].push_back(position);
                        number_of_samples[sensor.first]++;
                    }

                    publishSphere(position, "world", "sensor_calibration", message_counter++, COLOR(1, 0, 0, 0.1),
                                  0.005f, 100);

                }
            }
        }
    }

//    toggleMPU6050(true);
//    while ((ros::Time::now() - start_time) < ros::Duration(5) && calibrating){
//        ROS_INFO_THROTTLE(1,"waiting a couple of seconds for fresh imu data");
//    }
//    zero_pose = pose;
//    toggleMPU6050(false);

    // start tracking again
    startTracking(true);

    // calculate the mean and covariance for each sensor and origin of the object (the origin is the average center)
    origin = Vector3d(0, 0, 0);
    map<int, Vector3d> mean;
    map<int, Vector3d> variance;
    map<int, bool> sensor_accepted;
    int active_sensors = 0;

    // set the relative sensor location for each sensor wrt the origin
    for (auto const &sensor : sensors) {
        if (number_of_samples[sensor.first] > 200) {
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
            ROS_INFO_STREAM("sensor " << sensor.first << " mean("
                                      << mean[sensor.first][0] << ", " << mean[sensor.first][1] << ", "
                                      << mean[sensor.first][2] << ") variance("
                                      << variance[sensor.first][0] << ", " << variance[sensor.first][1] << ", "
                                      << variance[sensor.first][2] << ")"
                                      << " " << "number of samples" << number_of_samples[sensor.first]);
            if (variance[sensor.first].norm() > 0) {
                sensor_accepted[sensor.first] = true;
                active_sensors++;
                origin += mean[sensor.first];
            } else {
                sensor_accepted[sensor.first] = false;
            }
        } else {
            ROS_INFO("rejecting sensor %d, because it does not have enough samples (%d)", sensor.first,
                     number_of_samples[sensor.first]);
        }
    }
    if (active_sensors == 0) {
        ROS_WARN("no active sensors, aborting");
        return;
    }
    origin /= (double) active_sensors;
    if (origin.hasNaN()) {
        ROS_WARN("origin not finite, aborting");
        return;
    }
    publishSphere(origin, "world", "origin", message_counter++, COLOR(0, 0, 1, 1.0), 0.01f);
    for (auto &sensor : sensors) {
        if (sensor_accepted[sensor.first]) {
            Vector3d rel;
            rel = mean[sensor.first] - origin;
            sensor.second.setRelativeLocation(rel);
            ROS_INFO_STREAM("origin(" << origin[0] << ", " << origin[1] << ", " << origin[2] << ")");
            publishSphere(mean[sensor.first], "world", "mean", message_counter++, COLOR(0, 0, 1, 1.0), 0.01f);
            publishRay(origin, rel, "world", "relative", message_counter++, COLOR(1, 1, 0, 1.0));
        }
    }
    writeConfig(path + "/protoType3.yaml");
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
    for (auto &sensor : sensors) {
        if (!sensor.second.sensorCalibrated())
            continue;
        YAML::Node node = YAML::Load("[0, 0, 0, 0]");
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

void TrackedObject::publishRelativeFrame() {
    ros::Rate rate(10);
    while (publishingRelativeFrame) {
        tf_broadcaster.sendTransform(tf::StampedTransform(relativeFrame, ros::Time::now(), "world", name.c_str()));
        rate.sleep();
    }
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
