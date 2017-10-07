#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// mavmap
#include "base3d/triangulation.h"
#include "base3d/projection.h"

// offset from center of laser rotation
#define AXIS_OFFSET -0.015

using namespace Eigen;

/**
    * This function triangulates the position of a sensor using the horizontal and vertical angles from two ligthouses
    * via planes
    * @param angles0 vertical/horizontal angles form first lighthouse
    * @param angles1 vertical/horizontal angles form second lighthouse
    * @param RT_0 pose matrix of first lighthouse
    * @param RT_1 pose matrix of second lighthouse
    * @param triangulated_position the triangulated position
    * @param ray0 ligthhouse ray
    * @param ray1 ligthhouse ray
    */
void triangulateFromLighthousePlanes(Vector2d &angles0, Vector2d &angles1, Matrix4d &RT_0, Matrix4d &RT_1,
                                     Vector3d &triangulated_position, Vector3d &ray0, Vector3d &ray1);

void triangulateFromRays(Vector3d &ray0, Vector3d &ray1, Matrix4d &RT_0, Matrix4d &RT_1, Vector3d &triangulated_position);

void triangulateFromRays(MatrixXd &ray0, MatrixXd &ray1, Matrix4d &RT_0, Matrix4d &RT_1, Vector3d &triangulated_position);

void rayFromLighthouseAngles(Vector2d &angles, Vector3d &ray);