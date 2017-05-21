#include "darkroom/PoseMinimizer.hpp"

PoseMinimizer::PoseMinimizer(int numberOfSensors):
        Functor<double>(6,3*numberOfSensors), numberOfSensors(numberOfSensors) {
    pose = VectorXd(6);
    pos3D_A = MatrixXd(4,numberOfSensors);
    pos3D_B = MatrixXd(4,numberOfSensors);
}

int PoseMinimizer::operator()(const VectorXd &x, VectorXd &fvec) const
{
    Matrix4d RT = MatrixXd::Identity(4,4);
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(x(0),2.0)+pow(x(1),2.0)+pow(x(2),2.0),2.0);
    Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                  2.0*x(0)/(alpha_squared+1),
                  2.0*x(1)/(alpha_squared+1),
                  2.0*x(2)/(alpha_squared+1));
    // construct RT matrix
    RT.topLeftCorner(3,3) = q.toRotationMatrix();
    RT.topRightCorner(3,1) << x(3), x(4), x(5);

    MatrixXd new_position3d(4,numberOfSensors);
    new_position3d = RT * pos3D_B;

    MatrixXd difference(4,numberOfSensors);
    difference = new_position3d - pos3D_A;

    uint j = 0;
    for(uint i=0; i<numberOfSensors; i++){
        fvec(j++)   = difference(0,i);
        fvec(j++)   = difference(1,i);
        fvec(j++)   = difference(2,i);
    }

    // cout << "RT\n" << RT << endl;
    // cout << "position3D frame A\n" <<  pos3D_A << endl;
    // cout << "position3D frame B\n" <<  pos3D_B << endl;
    // cout << "new position3D frame A\n" <<  new_position3d << endl;
    // cout << "error : " << difference.squaredNorm() <<endl;
    // cout << "x : " << x <<endl;
    return 0;
}

void PoseMinimizer::getRTmatrix(VectorXd &x, Matrix4d &RT){
    RT = MatrixXd::Identity(3,4);
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(x(0),2.0)+pow(x(1),2.0)+pow(x(2),2.0),2.0);
    Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                  2.0*x(0)/(alpha_squared+1),
                  2.0*x(1)/(alpha_squared+1),
                  2.0*x(2)/(alpha_squared+1));
    // construct RT matrix
    RT.topLeftCorner(3,3) = q.toRotationMatrix();
    RT.topRightCorner(3,1) << x(3), x(4), x(5);
}

void PoseMinimizer::getRTmatrix(Matrix4d &RT){
    RT = Matrix4d::Identity();
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(pose(0),2.0)+pow(pose(1),2.0)+pow(pose(2),2.0),2.0);
    Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                  2.0*pose(0)/(alpha_squared+1),
                  2.0*pose(1)/(alpha_squared+1),
                  2.0*pose(2)/(alpha_squared+1));
    // construct RT matrix
    RT.topLeftCorner(3,3) = q.toRotationMatrix();
    RT.topRightCorner(3,1) << pose(3), pose(4), pose(5);
}

void PoseMinimizer::getTFtransform(VectorXd &x, tf::Transform &tf){
    tf.setOrigin(tf::Vector3(x(3), x(4), x(5)));

    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(x(0),2.0)+pow(x(1),2.0)+pow(x(2),2.0),2.0);
    Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                  2.0*x(0)/(alpha_squared+1),
                  2.0*x(1)/(alpha_squared+1),
                  2.0*x(2)/(alpha_squared+1));
    Matrix3d rot = q.toRotationMatrix();
    tf::Quaternion quat;
    tf::Matrix3x3 rot_matrix(rot(0,0), rot(0,1), rot(0,2),
                             rot(1,0), rot(1,1), rot(1,2),
                             rot(2,0), rot(2,1), rot(2,2));

    rot_matrix.getRotation(quat);
    tf.setRotation(quat);
}