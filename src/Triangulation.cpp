#include "darkroom/Triangulation.hpp"

void triangulateFromLighthousePlanes(Vector2d &angles0, Vector2d &angles1, Matrix4d &RT_0, Matrix4d &RT_1,
                                     Vector3d &triangulated_position, Vector3d &ray0, Vector3d &ray1) {
    ;

    // generate the projection matrices
    Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
    proj_matrix0 = RT_0.topLeftCorner(3, 4);
    proj_matrix1 = RT_1.topLeftCorner(3, 4);

    double azimuth0 = angles0(1), azimuth1 = angles1(1), elevation0 = angles0(0), elevation1 = angles1(0);
    Eigen::Vector3d vec0_vertic(AXIS_OFFSET, -cos(elevation0), -sin(elevation0));
    Eigen::Vector3d vec0_horizo(cos(azimuth0), AXIS_OFFSET, -sin(azimuth0));

    Eigen::Vector3d vec1_vertic(AXIS_OFFSET, -cos(elevation1), -sin(elevation1));
    Eigen::Vector3d vec1_horizo(cos(azimuth1), AXIS_OFFSET, -sin(azimuth1));
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
}

void triangulateFromRays(Vector3d &ray0, Vector3d &ray1, Matrix4d &RT_0, Matrix4d &RT_1, Vector3d &triangulated_position){
    // generate the projection matrices
    Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
    proj_matrix0 = RT_0.topLeftCorner(3, 4);
    proj_matrix1 = RT_1.topLeftCorner(3, 4);
    // project onto image plane
    Eigen::Vector2d projected_image_location0 = Eigen::Vector2d(ray0(0) / ray0(2), ray0(1) / ray0(2));
    Eigen::Vector2d projected_image_location1 = Eigen::Vector2d(ray1(0) / ray1(2), ray1(1) / ray1(2));

    triangulated_position = triangulate_point(proj_matrix0, proj_matrix1,
                                              projected_image_location0, projected_image_location1);
}

void triangulateFromRays(MatrixXd &ray0, MatrixXd &ray1, Matrix4d &RT_0, Matrix4d &RT_1, MatrixXd &triangulated_position){
    // generate the projection matrices
    Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
    proj_matrix0 = RT_0.topLeftCorner(3, 4);
    proj_matrix1 = RT_1.topLeftCorner(3, 4);

    for(int i=0;i<ray0.cols(); i++){
        // project onto image plane
        Eigen::Vector2d projected_image_location0 = Eigen::Vector2d(ray0(0,i) / ray0(2,i), ray0(1,i) / ray0(2,i));
        Eigen::Vector2d projected_image_location1 = Eigen::Vector2d(ray1(0,i) / ray1(2,i), ray1(1,i) / ray1(2,i));

        Vector3d pos = triangulate_point(proj_matrix0, proj_matrix1,
                                         projected_image_location0, projected_image_location1);

        triangulated_position(0,i) = pos(0);
        triangulated_position(1,i) = pos(1);
        triangulated_position(2,i) = pos(2);
    }
}

void rayFromLighthouseAngles(Vector2d &angles, Vector3d &ray){
    double azimuth = angles(1), elevation = angles(0);
    Eigen::Vector3d vec_vertic(AXIS_OFFSET, -cos(elevation), -sin(elevation));
    Eigen::Vector3d vec_horizo(cos(azimuth), AXIS_OFFSET, -sin(azimuth));

    // calc normals lighthouse
    Eigen::Vector3d n_horizo = vec_horizo.cross(Eigen::Vector3d::UnitY());
    n_horizo.normalize();
    Eigen::Vector3d n_vertic = vec_vertic.cross(Eigen::Vector3d::UnitX());
    n_vertic.normalize();

    // calc line direction from cross product of hyperplane normals
    ray = n_horizo.cross(n_vertic);
}