#include "include/darkroom/DarkRoom.hpp"

DarkRoom::DarkRoom(QWidget *parent)
        : rviz::Panel(parent) {

    GOOGLE_PROTOBUF_VERIFY_VERSION;
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QTabWidget *tabPage = new QTabWidget;

    // ################################################### connect TAB
    QWidget *connectWidget = new QWidget;
    connectWidget->setLayout(new QVBoxLayout);
    tabPage->addTab(connectWidget, "connect");

   QListWidget *listWidget = new QListWidget;
   listWidget->setObjectName("trackedObjects");
   connectWidget->layout()->addWidget(listWidget);

    // QTableWidget *tableWidget = new QTableWidget(4, 4);
    //
    // tableWidget->setItemDelegate(new TrackedObjectDelegate);
    // tableWidget->setEditTriggers(QAbstractItemView::DoubleClicked
    //                             | QAbstractItemView::SelectedClicked);
    // tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    //
    // tableWidget->resize(500, 300);
    // TrackedObjectDelegate *item = new TrackedObjectDelegate;
    // populateTableWidget(tableWidget);
    // connectWidget->layout()->addWidget(tableWidget);

    QLabel *IP_label = new QLabel("IP:");
    connectWidget->layout()->addWidget(IP_label);
    QLineEdit *IP_line = new QLineEdit("192.168.0.107");
    IP_line->setObjectName("IP");
    connectWidget->layout()->addWidget(IP_line);

    QLabel *sensor_port_label = new QLabel("client_port:");
    connectWidget->layout()->addWidget(sensor_port_label);
    QLineEdit *sensor_port_line = new QLineEdit("4210");
    sensor_port_line->setObjectName("client_port");
    connectWidget->layout()->addWidget(sensor_port_line);

    QLabel *lighthousedistance_label = new QLabel("lighthouse distance:");
    connectWidget->layout()->addWidget(lighthousedistance_label);
    QLineEdit *lighthouse_distance = new QLineEdit("2,0");
    lighthouse_distance->setObjectName("lighthouse_distance");
    connectWidget->layout()->addWidget(lighthouse_distance);

    QPushButton *connect_button = new QPushButton(tr("connect"));
    connect(connect_button, SIGNAL(clicked()), this, SLOT(connectTo()));
    connectWidget->layout()->addWidget(connect_button);

    QCheckBox *listen = new QCheckBox(tr("listen for new objects"));
    listen->setObjectName("listen");
    connect(listen, SIGNAL(clicked()), this, SLOT(startObjectListener()));
    connectWidget->layout()->addWidget(listen);

    QCheckBox *trackIT = new QCheckBox(tr("track"));
    trackIT->setObjectName("track");
    connect(trackIT, SIGNAL(clicked()), this, SLOT(track()));
    connectWidget->layout()->addWidget(trackIT);

    QCheckBox *rays = new QCheckBox(tr("rays"));
    rays->setObjectName("rays");
    connect(rays, SIGNAL(clicked()), this, SLOT(showRays()));
    connectWidget->layout()->addWidget(rays);

    QPushButton *switch_lighthouses_button = new QPushButton(tr("switch lighthouses"));
    connect(switch_lighthouses_button, SIGNAL(clicked()), this, SLOT(switch_lighthouses()));
    connectWidget->layout()->addWidget(switch_lighthouses_button);

    QPushButton *calibrate_button = new QPushButton(tr("calibrate"));
    connect(calibrate_button, SIGNAL(clicked()), this, SLOT(calibrate()));
    connectWidget->layout()->addWidget(calibrate_button);

    QPushButton *estimateDistance_button = new QPushButton(tr("estimateDistance"));
    connect(estimateDistance_button, SIGNAL(clicked()), this, SLOT(estimateDistance()));
    connectWidget->layout()->addWidget(estimateDistance_button);

    QPushButton *estimatePose_button = new QPushButton(tr("estimatePose"));
    connect(estimatePose_button, SIGNAL(clicked()), this, SLOT(estimatePose()));
    connectWidget->layout()->addWidget(estimatePose_button);

    QPushButton *estimatePose2_button = new QPushButton(tr("estimatePose2"));
    connect(estimatePose2_button, SIGNAL(clicked()), this, SLOT(estimatePose2()));
    connectWidget->layout()->addWidget(estimatePose2_button);

    QPushButton *clearall_button = new QPushButton(tr("clear all"));
    connect(clearall_button, SIGNAL(clicked()), this, SLOT(clearAll()));
    connectWidget->layout()->addWidget(clearall_button);

    QPushButton *reset_ligthouse_poses_button = new QPushButton(tr("reset lighthouse poses"));
    connect(reset_ligthouse_poses_button, SIGNAL(clicked()), this, SLOT(resetLighthousePoses()));
    connectWidget->layout()->addWidget(reset_ligthouse_poses_button);

    QCheckBox *recordIT = new QCheckBox(tr("record"));
    recordIT->setObjectName("record");
    connect(recordIT, SIGNAL(clicked()), this, SLOT(record()));
    connectWidget->layout()->addWidget(recordIT);

    QPushButton *rebootESP_button = new QPushButton(tr("reboot ESP"));
    connect(rebootESP_button, SIGNAL(clicked()), this, SLOT(rebootESP()));
    connectWidget->layout()->addWidget(rebootESP_button);

    QCheckBox *MPU6050_checkbox = new QCheckBox(tr("MPU6050"));
    MPU6050_checkbox->setObjectName("MPU6050");
    connect(MPU6050_checkbox, SIGNAL(clicked()), this, SLOT(toggleMPU6050()));
    connectWidget->layout()->addWidget(MPU6050_checkbox);
    // ################################################### objects TAB
    QWidget *objectsWidget = new QWidget;
    objectsWidget->setLayout(new QVBoxLayout);
    QScrollArea *objects_scrollArea = new QScrollArea;
    objects_scrollArea->setWidgetResizable(true);

//    QListWidget *listWidget = new QListWidget;
//    listWidget->setObjectName("trackedObjects");
//    objects_scrollArea->setWidget(listWidget);
//    objectsWidget->layout()->addWidget(objects_scrollArea);
//    tabPage->addTab(objectsWidget, "objects");

    frameLayout->addWidget(tabPage);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    // initialize ros

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "DarkRoomPlugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));

    visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);

    pose_correction_sub = nh->subscribe("/roboy/middleware/DarkRoom/LighthousePoseCorrection",1,&DarkRoom::correctPose, this);

    resetLighthousePoses();

    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    newObject->connectRoboy();
    object_counter++;
    trackedObjects[object_counter] = newObject;
    QListWidgetItem *item = new QListWidgetItem;
    item->setText("Roboy");
    this->findChild<QListWidget *>("trackedObjects")->addItem(item);
    ROS_INFO_STREAM("added Roboy");

    trackIT->click();
}

DarkRoom::~DarkRoom() {
    publish_transform = false;
    if(transform_thread->joinable()) {
        ROS_INFO("waiting for transform thread to shut down");
        transform_thread->join();
    }
    add_new_objects = false;
    if(object_listener_thread->joinable()) {
        ROS_INFO("waiting for object listener thread to shut down");
        object_listener_thread->join();
    }
    google::protobuf::ShutdownProtobufLibrary();
}

void DarkRoom::save(rviz::Config config) const {
    QLineEdit* w = this->findChild<QLineEdit*>("IP");
    config.mapSetValue(w->objectName(), w->text());
    w = this->findChild<QLineEdit*>("client_port");
    config.mapSetValue(w->objectName(), w->text());
    rviz::Panel::save(config);
}

void DarkRoom::load(const rviz::Config &config) {
    rviz::Panel::load(config);
    QLineEdit* w = this->findChild<QLineEdit*>("IP");
    QVariant text;
    config.mapGetValue(w->objectName(), &text);
    w->setText(text.toString());
    w = this->findChild<QLineEdit*>("client_port");
    config.mapGetValue(w->objectName(), &text);
    w->setText(text.toString());
}

void DarkRoom::startObjectListener(){
    QCheckBox *w = this->findChild<QCheckBox *>("listen");
    if(w->isChecked()){
        add_new_objects = true;
        object_listener_thread = boost::shared_ptr<std::thread>(new std::thread(&DarkRoom::objectListener, this));
        object_listener_thread->detach();
    }else{
        add_new_objects = false;
        if(object_listener_thread->joinable()){
            ROS_INFO("waiting for object listener thread to termincate");
            object_listener_thread->join();
        }
    }
}

void DarkRoom::rebootESP(){
    for(auto const &object:trackedObjects){
        if(!object.second->rebootESP()){
            ROS_WARN("could not send reboot message");
        }
    }
}

void DarkRoom::toggleMPU6050(){
    QCheckBox *w = this->findChild<QCheckBox *>("MPU6050");
    for(auto const &object:trackedObjects){
        if(!object.second->toggleMPU6050(w->isChecked())){
            ROS_WARN("could not send MPU6050 message");
        }
    }
}

void DarkRoom::connectTo() {
    QLineEdit *w = this->findChild<QLineEdit *>("IP");
    string IP = w->text().toStdString();
    w = this->findChild<QLineEdit *>("client_port");
    bool ok;
    int client_port = w->text().toInt(&ok);
    if (!ok) {
        w->setText("invalid port");
        return;
    }
    if(!checkIfObjectAlreadyExists(IP)) {
        TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
        // if we receive data from the sensor, add it to our tracked objects
        if (newObject->connectWifi(IP.c_str(), DEFAULT_SENSOR_PORT, client_port, DEFAULT_LOGGING_PORT, false)) {
            trackedObjects[object_counter] = newObject;
            QListWidgetItem *item = new QListWidgetItem;
            item->setText(IP.c_str());
            this->findChild<QListWidget *>("trackedObjects")->addItem(item);
            object_counter++;
        }
    }
}

void DarkRoom::clearAll() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.id = message_counter++;
    message_counter = 0;
    marker.action = visualization_msgs::Marker::DELETEALL;
    visualization_pub.publish(marker);
}

void DarkRoom::resetLighthousePoses(){
    QLineEdit *w = this->findChild<QLineEdit *>("lighthouse_distance");
    float ligthouse_distance = atof(w->text().toStdString().c_str());

    tf_world.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0, 0);
    tf_world.setRotation(quat);
    quat.setRPY(0, 0, 0);
    lighthouse1.setOrigin(tf::Vector3(0, 0, 0));
    lighthouse1.setRotation(quat);
    quat.setRPY(M_PI, M_PI, 0);
    lighthouse2.setOrigin(tf::Vector3(0, -ligthouse_distance, 0));
    lighthouse2.setRotation(quat);
}

void DarkRoom::record() {
    QCheckBox *w = this->findChild<QCheckBox *>("record");
    if(trackedObjects.empty())
        w->setChecked(false);
    for(auto const &object:trackedObjects){
        object.second->record(w->isChecked());
    }
}

void DarkRoom::track(){
    QCheckBox *w = this->findChild<QCheckBox *>("track");
    if(trackedObjects.empty()){
        w->setChecked(false);
    }else{
        for(auto const &object:trackedObjects){
            object.second->startTracking(w->isChecked());
        }
        publish_transform = true;
        if(transform_thread==nullptr){
          transform_thread = boost::shared_ptr<std::thread>(new std::thread(&DarkRoom::transformPublisher, this));
          transform_thread->detach();
        }
    }
}

void DarkRoom::showRays(){
    QCheckBox *w = this->findChild<QCheckBox *>("rays");
    for(auto const &object:trackedObjects){
        object.second->showRays(w->isChecked());
    }
}

void DarkRoom::calibrate() {
    for(auto const &object:trackedObjects){
        object.second->startCalibration(true);
    }
}

void DarkRoom::estimateDistance() {
    for(auto const &object:trackedObjects){
        object.second->startTracking(false);
        object.second->distanceEstimation(0, object.second->calibrated_sensors);
        object.second->distanceEstimation(1, object.second->calibrated_sensors);
        object.second->startTracking(true);
    }
}

void DarkRoom::estimatePose() {
    for(auto const &object:trackedObjects){
        tf::Transform correction;
        object.second->poseEstimation(correction);
        lighthouse2 = correction*lighthouse2;
    }
}

void DarkRoom::estimatePose2() {
    for(auto const &object:trackedObjects){
        object.second->startPoseestimation(true);
    }
}

void DarkRoom::switch_lighthouses(){
    lighthouse_switch=!lighthouse_switch;
    for(auto const &object:trackedObjects){
        object.second->map_lighthouse_id(lighthouse_switch);
    }
}

void DarkRoom::transformPublisher(){
    ros::Rate rate(5);
    while(publish_transform){
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse1,ros::Time::now(),"world","lighthouse1"));
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse2,ros::Time::now(),"world","lighthouse2"));
//        tf::Matrix3x3 rot(lighthouse2.getRotation());
//
//        ROS_INFO_STREAM("rot: \n" << rot.getRow(0).getX() << "\t" << rot.getRow(0).getY() << "\t" << rot.getRow(0).getZ() << endl <<
//                                rot.getRow(1).getX() << "\t" << rot.getRow(1).getY() << "\t" << rot.getRow(1).getZ() << endl <<
//                                rot.getRow(2).getX() << "\t" << rot.getRow(2).getY() << "\t" << rot.getRow(2).getZ() << endl);
        rate.sleep();
    }
}

void DarkRoom::objectListener(){
    // create a ping socket
    ping_socket = UDPSocketPtr(new UDPSocket(DEFAULT_CONFIG_PORT, false));
    ros::Rate rate(1);
    DarkRoomProtobuf::trackedObjectConfig msg;
    while(add_new_objects){
        ROS_INFO_THROTTLE(5,"listening to new objects");
        // listen for trackedObjectConfig messages
        if(ping_socket->receiveMessage<DarkRoomProtobuf::trackedObjectConfig>(msg)){
            lock_guard<std::mutex> lock(m_lockMutex);
            // if its a new object add it
            char IP[INET_ADDRSTRLEN];
            convertByte2Text(msg.ip(),IP);
            if(!checkIfObjectAlreadyExists(msg.ip())) {
                ROS_INFO("thats a new object, trying to connect...");
                TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
                // if we receive data from the sensor, add it to our tracked objects
                if (newObject->connectWifi(IP, DEFAULT_SENSOR_PORT, msg.command_port(), DEFAULT_LOGGING_PORT)) {
                    object_counter++;
                    trackedObjects[object_counter] = newObject;
                    QListWidgetItem *item = new QListWidgetItem;
                    item->setText(IP);
                    this->findChild<QListWidget *>("trackedObjects")->addItem(item);
                    ROS_INFO_STREAM("added new object " << IP);
                    ros::Time start_time = ros::Time::now();
                    ROS_INFO("sleeping for a few seconds, to give the esp time for setting up stuff");
                    while ((ros::Time::now() - start_time).sec < 5) {
                        usleep(100000);
                    }
                }
            }else{
                trackedObjects[object_counter].reset();
                TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
                // if we receive data from the sensor, add it to our tracked objects
                if (newObject->connectWifi(IP, DEFAULT_SENSOR_PORT, msg.command_port(), DEFAULT_LOGGING_PORT, false)) {
                    trackedObjects[object_counter] = newObject;
                    ROS_INFO_STREAM("object " << IP << " reconnected");
                }
            }
        }
        rate.sleep();
    }
    ping_socket.reset();
}

void DarkRoom::correctPose(const roboy_communication_middleware::LighthousePoseCorrection &msg){
    tf::Transform tf;
    tf::transformMsgToTF(msg.tf, tf);
    if(msg.id == 0){
        lighthouse1 = tf*lighthouse1;
    }else{
        lighthouse2 = tf*lighthouse2;
    }
}

bool DarkRoom::checkIfObjectAlreadyExists(const string &IP){
    QListWidget *wt = this->findChild<QListWidget *>("trackedObjects");
    for(int i = 0; i < wt->count(); ++i)
    {
        QListWidgetItem* item = wt->item(i);
        if(item->text().compare(IP.c_str())==0){
            ROS_WARN_STREAM("object is already connected with " << IP);
            return true;
        }
    }
    return false;
}

bool DarkRoom::checkIfObjectAlreadyExists(const uint32_t &IP){
    QListWidget *wt = this->findChild<QListWidget *>("trackedObjects");
    char ip[INET_ADDRSTRLEN];
    convertByte2Text(IP,ip);
    for(int i = 0; i < wt->count(); ++i)
    {
        QListWidgetItem* item = wt->item(i);
        if(item->text().compare(ip)==0){
            ROS_WARN_STREAM("object is already connected with " << IP);
            return true;
        }
    }
    return false;
}

PLUGINLIB_EXPORT_CLASS(DarkRoom, rviz::Panel)
