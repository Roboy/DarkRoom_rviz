#pragma once

// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <roboy_communication_middleware/DarkRoomSensor.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <roboy_communication_middleware/DarkRoom.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// mavmap
#include "base3d/triangulation.h"
#include "base3d/projection.h"

// std
#include <map>
#include <fstream>
#include <deque>
#include <thread>
#include <mutex>
#include <bitset>

// yaml-cpp
#include "yaml-cpp/yaml.h"

#include "darkroom/UDPSocket.hpp"
#include "darkroom/Sensor.hpp"
#include "darkroom/PoseMinimizer.hpp"

#include <common_utilities/rviz_visualization.hpp>

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

#define uSecsToRadians(ticks) (degreesToRadians(ticks * 0.0216))
#define MAX_ITERATIONS 100
#define ERROR_THRESHOLD 0.001
//#define DEBUG
// #define KALMAN

#define DEFAULT_CONFIG_PORT  8001
#define DEFAULT_SENSOR_PORT  8002
#define DEFAULT_IMU_PORT     8003
#define DEFAULT_COMMAND_PORT 8004
#define DEFAULT_LOGGING_PORT 8005

using namespace std;

static vector<int> DEFAULT_VECTOR;

class TrackedObject:public rviz_visualization {
public:
    TrackedObject();

    ~TrackedObject();

    /**
     * This function attempts to establish the connection to the tracked object via wifi
     * @param IP IP of the object
     * @param sensor_port port to be used for sensor value
     * @param command_port port to be used for commands
     * @param logging_port port to be used for logging
     * @param configure if true sends our ports to client and waits for a logging message
     * @return success (if configure is false, this returns true even if there is no data coming from clientbool lighthouse)
     */
    bool connectWifi(const char *IP, int sensor_port, int command_port, int logging_port, bool configure=true);

    /**
     * This function initializes a subscriber for roboy darkroom
     */
    void connectRoboy();

    void startReceiveData(bool start);
    /**
     * toggles tracking (starts reception of sensor data and triangulation threads)
     * @param start toggle flag
     */
    void startTracking(bool start);

    /**
     * Toggles calibration thread
     * @param start bool
     */
    void startCalibration(bool start);

    /**
     * Toggles visualization of ligthhouse rays
     * @param show bool
     */
    void showRays(bool show);

    /**
     * This function  switches the lighthouse ids for all objects
     */
    void map_lighthouse_id(bool switchID);

    /**
     * Estimates the sensor distances of all active sensors (or a vector of specified sensor ids)
     * using the known (ie. calibrated) relative distance between the sensors and the lighthouse angles
     * @param lighthouse for which lighthouse
     * @param specificIds if defined, waits until the specified sensors become active
     * @return
     */
    bool distanceEstimation(bool lighthouse, vector<int> &specificIds = DEFAULT_VECTOR);

    /**
     * Estimates the pose correction between ligthhouse 1 and 2, such that the squared distances between sensor positions
     * estimated for both lighthouses is minimized.
     * @param tf the corrective transform for lighthouse 2
     * @return success
     */
    bool poseEstimation(tf::Transform &tf);

    /**
     * Triggers recording of sensor data to a sensor.log file
     * @param start
     * @return success (if writing to the file is possible)
     */
    bool record(bool start);

    /**
     * Reads a yaml tracked object file
     * @param filepath to
     * @return success
     */
    bool readConfig(string filepath);

    /**
     * Writes a tracked object to file
     * @param filepath
     * @return success
     */
    bool writeConfig(string filepath);

    /**
     * Sends a command which triggers ESP reboot
     * @return success
     */
    bool rebootESP();

    /**
     * Toggles the IMU MPU6050
     * @param toggle true/false
     * @return successully sent
     */
    bool toggleMPU6050(bool toggle);

    /**
     * Toggles the tracking on fpga
     * @param toggle true/false
     * @return successully sent
     */
    bool toggleTracking(bool toggle);

    static int trackeObjectInstance; //! a unique object instance (helps with unique rviz marker ids)
private:
    /**
     * Continuesly receiving, decoding and updating the sensor data
     */
    void receiveSensorData();
    /**
     * Continuesly receiving, decoding and updating the sensor data
     */
    void receiveSensorDataRoboy(const roboy_communication_middleware::DarkRoom::ConstPtr &msg);

    /**
     * Continuesly receiving imu data
     */
    void receiveImuData();

    /**
     * Triangulates the sensor positions (the transform between lighthouse 1 and 2 needs to be known, otherwise the
     * triangulated position is not correct)
     */
    void trackSensors();

    /**
     * Measures triangulated sensor locations for 30 seconds. Calculates mean sensor locations and generates
     * relative sensor positions which are saved to a yaml file
     */
    void calibrate();

    /**
     * Publishes information about the relative frame of the Object (as estimated via calibration)
     */
    void publishRelativeFrame();

    /**
    * This function triangulates the position of a sensor using the horizontal and vertical angles from two ligthouses
    * via planes
    * @param angles0 vertical/horizontal angles form first lighthouse
    * @param angles1 vertical/horizontal angles form second lighthouse
    * @param triangulated_position the triangulated position
    * @param ray0 ligthhouse ray
    * @param ray1 ligthhouse ray
    * @return success (this can fail, if we don't get the pose of the lighthouses via tf)
    */
    bool triangulateFromLighthousePlanes(Vector2d &angles0, Vector2d &angles1, Vector3d &triangulated_position,
                                         Vector3d &ray0, Vector3d &ray1);

    /**
     * Queries the tf listener for the specified transform
     * @param from we whant the transformation from this frame
     * @param to another frame
     * @param transform the transform if available
     * @return true if available
     */
    bool getTransform(const char *from, const char *to, Matrix4d &transform);

    /**
     * clear all markers
     */
    void clearAll();

    /**
     * Returns a unique id for #MESSAGE_ID sensor and lighthouse
     * @param type the message type #MESSAGE_ID
     * @param sensor the sensor id
     * @param lighthouse the lighthouse
     * @return a unique id
     */
    int getMessageID(int type, int sensor, bool lighthouse = false);
public:
    vector<int> pose_correction_sensors;
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher visualization_pub, sensor_location_pub;
    ros::Subscriber sensor_sub;
    tf::Transform relativeFrame;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    int objectID = 0;
    string path;
    string name = "bastiisdoff";
    string mesh = "pimmel";
    vector<Eigen::Vector3f> object;
    double axis_offset = -0.015;
    Vector3d poseIMU;
    int sensordata_port;
    UDPSocketPtr sensor_socket, command_socket, logging_socket, imu_socket;
    boost::shared_ptr<thread> sensor_thread = nullptr, tracking_thread = nullptr, calibrate_thread = nullptr, imu_thread = nullptr;
    bool receiveData = false, tracking = false, calibrating = false, connected = false, rays = false, recording = false,
            publishingRelativeFrame = false;
    map<int,Sensor> sensors;
    Vector3d origin;
    Vector4d pose;
    Vector4d zero_pose;
    bool m_switch = false;
    ofstream file;
    enum MESSAGE_ID{
        TRIANGULATED = 0,      // for each sensor
        DISTANCE = 1,           // for each sensor and lighthouse
        RAY = 2,   // for each sensor and lighthouse
        SENSOR_NAME = 3   // for each sensor
    };
};

typedef boost::shared_ptr<TrackedObject> TrackedObjectPtr;
