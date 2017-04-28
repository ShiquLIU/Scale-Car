#include <eigen3/Eigen/Dense>

using namespace Eigen;

#ifndef KALMAN_H
#define KALMAN_H

void kalman(Vector4d & x, Matrix4d & P, const Vector2d & y, const double & u, const double & dt);

#endif
