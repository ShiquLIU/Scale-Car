#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>

#include "ros/ros.h"
#include "um_acc/ComputeControl.h"
#include "eiquadprog/eiquadprog.hpp"

#include "qphelper.h"

using namespace std;
using namespace Eigen;
using namespace eiquadprog;

// QP parameters
double equality_tolerance = 1e-8;
uint iter_max = 100;
uint iters = 0;
VectorXd u_actual;
double obj_actual;

// Car constants
double car_vlin;
double car_mass;
double car_f0;
double car_f1;
double car_f2;
double tau_des;
double v_des;
double h_min;

// Computes QP weights from loaded info
QPHelper qp_helper;
QPInfo qpinfo;

// QP weights and scaling value
Matrix<double, DIM_N, DIM_N> Rx;
Matrix<double, DIM_N, 1> rx;
Matrix<double, DIM_M, DIM_M> Ru;
Matrix<double, DIM_M, 1> ru;

// Set info
int current_set;
int current_poly;

// Constant variables
Matrix<double, DIM_N, 1> x0;

void update_weights(const Matrix<double, DIM_N, 1> & x0, Matrix<double, DIM_N, DIM_N> & Rx, Matrix<double, DIM_N, 1> & rx, Matrix<double, DIM_M, DIM_M> & Ru, Matrix<double, DIM_M, 1> & ru) {
 
  double v = x0(0);
  double v_lead = x0(2);

  // weights on different criteria
  double con_v_weight = 1;
  double con_h_weight = 8;
  double con_u_weight = 15;
  double ramp_lim = 0.5; 
  double ramp_delta = 1;

  /////// Do not change below /////////

  double v_goal = min(v_des, v_lead);
  double h_goal = fmax(h_min, tau_des*v);

  double ramp = fmax(0, fmin(1, 0.5+abs(v-v_lead)/ramp_delta-ramp_lim/ramp_delta));

  double v_weight      = con_v_weight;
  double h_weight      = con_h_weight*(1-ramp);
  double u_weight      = con_u_weight;

  Rx(0,0) = v_weight;
  Rx(1,1) = h_weight;
  rx(0) = v_weight*(-v_goal);
  rx(1) = h_weight*(-h_goal);

  Ru(0,0) = u_weight;
  ru(0) = 0;
}

bool compute(um_acc::ComputeControl::Request  &req,
	     um_acc::ComputeControl::Response &res) {

  x0(0) = req.v;
  x0(1) = req.h;
  x0(2) = req.vl;

  qp_helper.update_current_poly(x0, current_set, current_poly);
  int next_set = current_set > 0 ? current_set-1 : current_set;
  int next_poly = current_set > 0 || current_poly == 0 ? current_poly : current_poly-1;

  update_weights(x0, Rx, rx, Ru, ru);
  qpinfo = qp_helper.get_qp_info(next_set, next_poly, x0, Rx, rx, Ru, ru);
  obj_actual = eiquadprog::solve_quadprog(qpinfo.H, qpinfo.f, 
                                    -qpinfo.A_eq.transpose(), -qpinfo.b_eq.transpose(), 
                                    -qpinfo.A_ineq.transpose(), qpinfo.b_ineq, 
                                    u_actual, equality_tolerance, iter_max, &iters);
  res.u = u_actual(0)/qpinfo.u_scale + car_f2*pow(req.v-car_vlin,2);
  return true;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "compute_control_server");


  ifstream infile;
  string filename = argv[1];

  // Load sets and dyn model
  qp_helper.load_controller(argv[1]);

  // Load some constants
  infile.open(filename + "/car_constants.txt");
  infile >> car_vlin;
  infile >> car_mass;
  infile >> car_f0;
  infile >> car_f1;
  infile >> car_f2;
  infile >> tau_des;
  infile >> v_des;
  infile >> h_min;

  infile.close();
  
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("compute_control", compute);
  ROS_INFO("Control signal computation initialized");
  ros::spin();

  return 0;
}
