#include "darkroom/LighthousePoseMinimizer.hpp"

namespace LighthousePoseEstimator {

    int LighthousePoseMinimizer::counter = 0;

    LighthousePoseMinimizer::LighthousePoseMinimizer(int numberOfSamples, double distanceBetweenSensors,
                                                     MatrixXd &rays0_A, MatrixXd &rays0_B,
                                                     MatrixXd &rays1_A, MatrixXd &rays1_B) :
            Functor<double>(6, 1 * numberOfSamples), numberOfSamples(numberOfSamples),
            distanceBetweenSensors(distanceBetweenSensors), rays0_A(rays0_A),
            rays0_B(rays0_B), rays1_A(rays1_A), rays1_B(rays1_B) {
        pose = VectorXd(6);
    }

    int LighthousePoseMinimizer::operator()(const VectorXd &x, VectorXd &fvec) const {
        Matrix4d RT = MatrixXd::Identity(4, 4);
        // construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(x(0), 2.0) + pow(x(1), 2.0) + pow(x(2), 2.0), 2.0);
        Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                      2.0 * x(0) / (alpha_squared + 1),
                      2.0 * x(1) / (alpha_squared + 1),
                      2.0 * x(2) / (alpha_squared + 1));
        q.normalize();
        // construct RT matrix
        RT.topLeftCorner(3, 3) = q.toRotationMatrix();
        RT.topRightCorner(3, 1) << x(3), x(4), x(5);


        Matrix4d RT_B_corrected;
        RT_B_corrected = RT*RT_B;

        Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
        proj_matrix0 = RT_A.topLeftCorner(3, 4);
        proj_matrix1 = RT_B_corrected.topLeftCorner(3, 4);

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
            Vector3d distance = pos1 - pos0;
            fvec(i) = distance.norm() - distanceBetweenSensors;
//            printf("pos0: %f\t%f\t%f    pos1: %f\t%f\t%f\n", pos0(0), pos0(1), pos0(2), pos1(0), pos1(1), pos1(2));
//            printf("%f\t", distance.norm());
        }
//        cout << endl;
        counter++;

        cout << "iteration: " << counter << endl;
        cout << "RT_B_corrected\n" << RT_B_corrected << endl;
        cout << "error : " << fvec.squaredNorm() << endl;
        cout << "x : " << x << endl;
        return 0;
    }

    void LighthousePoseMinimizer::getRTmatrix(VectorXd &x, Matrix4d &RT) {
        RT = MatrixXd::Identity(3, 4);
        // construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(x(0), 2.0) + pow(x(1), 2.0) + pow(x(2), 2.0), 2.0);
        Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                      2.0 * x(0) / (alpha_squared + 1),
                      2.0 * x(1) / (alpha_squared + 1),
                      2.0 * x(2) / (alpha_squared + 1));
        // construct RT matrix
        RT.topLeftCorner(3, 3) = q.toRotationMatrix();
        RT.topRightCorner(3, 1) << x(3), x(4), x(5);
    }

    void LighthousePoseMinimizer::getRTmatrix(Matrix4d &RT) {
        RT = Matrix4d::Identity();
        // construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(pose(0), 2.0) + pow(pose(1), 2.0) + pow(pose(2), 2.0), 2.0);
        Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                      2.0 * pose(0) / (alpha_squared + 1),
                      2.0 * pose(1) / (alpha_squared + 1),
                      2.0 * pose(2) / (alpha_squared + 1));
        // construct RT matrix
        RT.topLeftCorner(3, 3) = q.toRotationMatrix();
        RT.topRightCorner(3, 1) << pose(3), pose(4), pose(5);
    }

    void LighthousePoseMinimizer::getTFtransform(VectorXd &x, tf::Transform &tf) {
        tf.setOrigin(tf::Vector3(x(3), x(4), x(5)));

        // construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(x(0), 2.0) + pow(x(1), 2.0) + pow(x(2), 2.0), 2.0);
        Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                      2.0 * x(0) / (alpha_squared + 1),
                      2.0 * x(1) / (alpha_squared + 1),
                      2.0 * x(2) / (alpha_squared + 1));
        q.normalize();
        Matrix3d rot = q.toRotationMatrix();
        tf::Quaternion quat;
        tf::Matrix3x3 rot_matrix(rot(0, 0), rot(0, 1), rot(0, 2),
                                 rot(1, 0), rot(1, 1), rot(1, 2),
                                 rot(2, 0), rot(2, 1), rot(2, 2));

        rot_matrix.getRotation(quat);
        tf.setRotation(quat);
    }

}