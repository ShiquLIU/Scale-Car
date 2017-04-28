#include <vector>
#include "eiquadprog.hpp"
#include <fstream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using namespace Eigen;

class qp_var
{
	public:

		struct Polyhedron {
			MatrixXd A;
			VectorXd b;
		};

		struct Dynamics {
			MatrixXd A;
			MatrixXd B;
			MatrixXd E;
			VectorXd K;
			//MatrixXd Em;

			//Polyhedron XU_set;  //using above Polyhedron struct
			MatrixXd XD_plus;
			MatrixXd XD_minus;
		};

		qp_var(); //constructor
		void read_matlab();
		VectorXd get_input(VectorXd x0, MatrixXd Rx, VectorXd rx, MatrixXd Ru, VectorXd ru);
		VectorXd get_feedback(VectorXd x0);
		VectorXd update_state(VectorXd x0, VectorXd u, VectorXd d);

	protected:

	private:
		template <typename Type>
		void read_item(std::ifstream & file, Type &);
		//void read_sets(std::ifstream & file, int number);

		// Sets of input constraints
		Dynamics dyn;

		// Define pre-feedback matrix
		MatrixXd k_feedback;

		// Define Polyhedron to steer to
		MatrixXd Hx;
		MatrixXd hx;

		// Input constraints
		MatrixXd Hu;
		VectorXd hu;

		// Disturbance assumptions
		MatrixXd XDplus, XDminus;

		// Sets
		//vector<vector<Polyhedron> > Sets;
};
