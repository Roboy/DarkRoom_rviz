#pragma once

#include <tf/tf.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// std
#include <iostream>

using namespace Eigen;
using namespace std;

// Generic functor for Eigen Levenberg-Marquardt minimizer
template<typename _Scalar, int NX = Dynamic, int NY = Dynamic>
struct Functor {
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}

    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }

    int values() const { return m_values; }
};

struct PoseMinimizer : Functor<double> {
    /**
     * Default amount of sensors needed for Eigen templated structure
     * @param numberOfSensors you can however choose any number of sensors here
     */
    PoseMinimizer(int numberOfSensors = 4);

    /**
     * This is the function that is called in each iteration
     * @param x the pose vector (3 rotational 3 translational parameters)
     * @param fvec the error function (the difference between the sensor positions)
     * @return
     */
    int operator()(const VectorXd &x, VectorXd &fvec) const;

    /**
     * Constructs a Transform matrix from the given pose vector
     * @param x the pose vector
     * @param RT the corresponding transform
     */
    void getRTmatrix(VectorXd &x, Matrix4d &RT);

    /**
     * Constructs a Transform matrix from the optimized pose
     * @param RT the corresponding transform
     */
    void getRTmatrix(Matrix4d &RT);

    /**
     * Constructs a tf transform from the given pose vector
     * @param x the pose vector
     * @param tf the corresponding tf transform
     */
    void getTFtransform(VectorXd &x, tf::Transform &tf);

    VectorXd pose;
    MatrixXd pos3D_A, pos3D_B; // 3d relative sensor positions wrt a common frame (eg worl_vive frame)
    int numberOfSensors = 4;
};