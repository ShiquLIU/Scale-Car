#include <iostream>
#include <cmath>

#include "kalman.h"

using namespace Eigen;
using namespace std;

void kalman(Vector4d & x, Matrix4d & P, const Vector2d & y, const double & u, const double & dt) {

	// specify dynamics
	double f0_bar = 0.9060;
	double f1_bar = 0.0060;
	double mass = 9.0700;

	double kappa = f1_bar/mass;
	double ekt = exp(-kappa * dt);

	//
	// Model: x(k+1) = Ax(k) + Bu(k) + K + w
	//        y(k) = C*x(k) + v
	//  where w ~ N(0,Q), v ~ N(0,R)
	// 

	Matrix4d A;
	A << ekt, 0, 0, 0,
		(ekt-1)/kappa, 1, dt, pow(dt, 2)/2,
		0, 0, 1, dt,
		0, 0, 0, 1;

	Vector4d B; 
	B << f1_bar*(1-ekt), mass*(1-ekt) - dt*f1_bar, 0, 0;
	B *= 1 / pow(f1_bar,2);

	Vector4d K;
	K << (f0_bar/f1_bar)*(ekt-1), (f0_bar/( pow(f1_bar, 2)) )*(dt*f1_bar + mass*(ekt-1)),  0, 0;

	Matrix<double, 2, 4> C;
	C << 1, 0, 0, 0,
	     0, 1, 0, 0;

	Vector4d Qvec;
	Qvec << 0, sqrt(pow(dt, 3) / 3), sqrt(pow(dt, 2) / 2), sqrt(dt);
	Matrix4d Q = Qvec * Qvec.transpose();
	Matrix2d R = Matrix2d::Zero();
	R(0,0) = 0.1*dt;
	R(1,1) = 0.1*dt;

    // Do Kalman updates
    Vector4d x_next_pred = A * x + B * u + K;
    Matrix4d P_next_pred = A * P * A.transpose() + Q;

    Vector2d inn = y - C * x_next_pred;
    Matrix2d inn_convar = C * P_next_pred * C.transpose() + R;

    Matrix<double, 4, 2> gain = P_next_pred * C.transpose() * inn_convar.inverse();

    x = x_next_pred + gain*inn;
    P = (Matrix4d::Identity() - gain * C) * P_next_pred;

    // Ugly hacks to keep speeds positive
    x(0) = fmax(x(0), 0);
    x(2) = fmax(x(2), 0);
}
