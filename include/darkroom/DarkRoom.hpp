#pragma once

#ifndef Q_MOC_RUN
// qt
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QComboBox>
#include <QTimer>
#include <QScrollArea>
#include <QListWidget>
#include <QStyledItemDelegate>
#include "darkroom/TrackedObjectDelegate.hpp"
// std
#include <map>
#include <thread>
#include <mutex>
// ros
#include <ros/ros.h>
#include <rviz/panel.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "darkroom/TrackedObject.hpp"
#include "darkroom/UDPSocket.hpp"

#endif

using namespace std;
using namespace Eigen;

class DarkRoom : public rviz::Panel{
    Q_OBJECT

public:
    DarkRoom(QWidget *parent = 0);

    ~DarkRoom();

    /**
     * Load all configuration data for this panel from the given Config object.
     * @param config rviz config file
     */
    virtual void load(const rviz::Config &config);

    /**
     * Save all configuration data from this panel to the given
     * Config object.  It is important here that you call save()
     * on the parent class so the class id and panel name get saved.
     * @param config rviz config file
     */
    virtual void save(rviz::Config config) const;

public Q_SLOTS:
    /**
     * Connects to sensor_port and IP given via GUI
     */
    void connectTo();

    /**
     * Clears all visualization markers
     */
    void clearAll();

    /**
     * Resets the lighthouse poses to inital values
     */
    void resetLighthousePoses();

    /**
     * Toggles recording sensor values for all tracked objects
     */
    void record();

    /**
     * Toggles tracking of all objects (ie triangulation)
     */
    void track();

    /**
     * Toggles visualization of lighthouse rays
     */
    void showRays();

    /**
     * Toggles calibration
     */
    void calibrate();

    /**
     * Toggles distance estimatio for both lighthouses
     */
    void estimateDistance();

    /*
     * Toggles pose correction
     */
    void estimatePose();

    void pingThread();

    void switch_lighthouses();

    void startObjectListener();

    void rebootESP();

    void toggleMPU6050();

private:
    /**
     * Is regularily publishing the tf frames (world_vive, lighthouse1, lighthouse2)
     */
    void transformPublisher();

    /**
     * Ping thread regularily listening for new objects
     */
    void objectListener();

    /**
     * This functions looks through trackedObjects ListWidget if the object is already being tracked
     * @param IP IP of new object
     * @return true (object already exists), false (new object)
     */
    bool checkIfObjectAlreadyExists(const string &IP);
    bool checkIfObjectAlreadyExists(const uint32_t &IP);

    boost::shared_ptr<std::thread> transform_thread = nullptr;
    boost::shared_ptr<std::thread> object_listener_thread;
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher visualization_pub;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform lighthouse1, lighthouse2, tf_world;
    bool lighthouse_switch = true, publish_transform = true, add_new_objects = true;
    int message_counter = 0;
    int object_counter = 0;
    map<int, TrackedObjectPtr> trackedObjects;
    map<int, string> IPs;
    string server_IP;
    int server_port;
    UDPSocketPtr ping_socket;
    mutex m_lockMutex;
};
