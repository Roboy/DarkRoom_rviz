#include "darkroom/Triangulation.hpp"

// dist3D_Line_to_Line(): get the 3D minimum distance between 2 lines
//    Input:  two 3D lines L1 and L2
//    Return: the shortest distance between L1 and L2
double dist3D_Line_to_Line( Vector3d &pos0, Vector3d &dir1,
                            Vector3d &pos1, Vector3d &dir2,
                            Vector3d &tri0, Vector3d &tri1) {
    Vector3d   u = dir1;
    Vector3d   v = dir2;
    Vector3d   w = pos0 - pos1;
    double    a = u.dot(u);         // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);         // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;        // always >= 0
    double    sc, tc;

    // compute the line parameters of the two closest points
    if (D < 0.0000001) {          // the lines are almost parallel
        sc = 0.0;
        tc = (b>c ? d/b : e/c);    // use the largest denominator
    }
    else {
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }

    // get the difference of the two closest points
    Vector3d   dP = w + (sc * u) - (tc * v);  // =  L1(sc) - L2(tc)
    tri0 = sc*dir1;
    tri1 = tc*dir2;
    return dP.norm();   // return the closest distance
}

double triangulateFromLighthousePlanes(Vector2d &angles0, Vector2d &angles1, Matrix4d &RT_0, Matrix4d &RT_1,
                                     Vector3d &triangulated_position, Vector3d &ray0, Vector3d &ray1) {
    ;

    Matrix3d rot0, rot1;
    rot0 = RT_0.topLeftCorner(3, 3);
    rot1 = RT_1.topLeftCorner(3, 3);

    double theta0 = angles0(1), theta1 = angles1(1), beta0 = angles0(0), beta1 = angles1(0);
    Eigen::Vector3d vec0_vertic(AXIS_OFFSET, sin(beta0), cos(beta0));
    Eigen::Vector3d vec0_horizo(cos(theta0), sin(theta0), AXIS_OFFSET);

    Eigen::Vector3d vec1_vertic(AXIS_OFFSET, sin(beta1), cos(beta1));
    Eigen::Vector3d vec1_horizo(cos(theta1), sin(theta1), AXIS_OFFSET);
    // calc normals lighthouse 0
    Eigen::Vector3d n0_horizo = vec0_horizo.cross(Eigen::Vector3d::UnitZ());
    n0_horizo.normalize();
    Eigen::Vector3d n0_vertic = vec0_vertic.cross(Eigen::Vector3d::UnitX());
    n0_vertic.normalize();
    // calc normals lighthouse 1
    Eigen::Vector3d n1_horizo = vec1_horizo.cross(Eigen::Vector3d::UnitZ());
    n1_horizo.normalize();
    Eigen::Vector3d n1_vertic = vec1_vertic.cross(Eigen::Vector3d::UnitX());
    n1_vertic.normalize();
    // calc line direction from cross product of hyperplane normals
    Eigen::Vector3d u0 = n0_horizo.cross(n0_vertic);
    Eigen::Vector3d u1 = n1_horizo.cross(n1_vertic);

    ray0 = Vector3d(u0(0), u0(1), u0(2));
    ray1 = Vector3d(u1(0), u1(1), u1(2));

    Vector3d ray0_worldFrame, ray1_worldFrame;

    ray0_worldFrame = rot0*ray0;
    ray1_worldFrame = rot1*ray1;

    Vector3d origin0 = RT_0.topRightCorner(3,1), origin1 = RT_1.topRightCorner(3,1);

    Vector3d l0, l1;

    double distance = dist3D_Line_to_Line(origin0, ray0_worldFrame, origin1, ray1_worldFrame, l0, l1);
    triangulated_position = l0+origin0 + (l1+origin1-l0-origin0)/2.0;

    return distance;
}

double triangulateFromRays(Vector3d &ray0, Vector3d &ray1,
                           Matrix4d &RT_0, Matrix4d &RT_1,
                           Vector3d &triangulated_position) {
    Matrix3d rot0, rot1;
    rot0 = RT_0.topLeftCorner(3, 3);
    rot1 = RT_1.topLeftCorner(3, 3);

    Vector3d ray0_worldFrame, ray1_worldFrame;

    ray0_worldFrame = rot0*ray0;
    ray1_worldFrame = rot1*ray1;

    Vector3d origin0 = RT_0.topRightCorner(3,1), origin1 = RT_1.topRightCorner(3,1);

    Vector3d l0, l1;

    double distance = dist3D_Line_to_Line(origin0, ray0_worldFrame, origin1, ray1_worldFrame, l0, l1);
    triangulated_position = l0+origin0 + (l1+origin1-l0-origin0)/2.0;

    return distance;
}

void rayFromLighthouseAngles(Vector2d &angles, Vector3d &ray) {
    double beta0 = angles(1), theta0 = angles(0);
    Eigen::Vector3d vec0_vertic(AXIS_OFFSET, sin(beta0), cos(beta0));
    Eigen::Vector3d vec0_horizo(cos(theta0), sin(theta0), AXIS_OFFSET);

    // calc normals lighthouse
    Eigen::Vector3d n_horizo = vec0_horizo.cross(Eigen::Vector3d::UnitZ());
    n_horizo.normalize();
    Eigen::Vector3d n_vertic = vec0_vertic.cross(Eigen::Vector3d::UnitX());
    n_vertic.normalize();

    // calc line direction from cross product of hyperplane normals
    ray = n_horizo.cross(n_vertic);
}