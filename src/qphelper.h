#include <vector>
#include <stdexcept>
#include "eigen3/Eigen/Dense"
#include <fstream>

// declare sizes that are decided at compile time
#define DIM_N 3
#define DIM_M 1
#define DIM_P 1
#define DIM_PM 0

#ifndef QP_HELPER_H
#define QP_HELPER_H

using namespace Eigen;
using namespace std;

struct QPInfo {
	Matrix<double, DIM_M, DIM_M> H;
	Matrix<double, DIM_M, 1> f;
	Matrix<double, Dynamic, DIM_M> A_ineq;
	Matrix<double, Dynamic, 1> b_ineq;
	Matrix<double, Dynamic, DIM_M> A_eq;
	Matrix<double, Dynamic, 1> b_eq;
  double u_scale;
};

template <int dim>
struct Polyhedron {
	int num_ineq() const { return A.rows(); }
	Matrix<double, Dynamic, dim> A;
	Matrix<double, Dynamic, 1> b;
};

struct Dynamics {
	Matrix<double, DIM_N, DIM_N> A;
	Matrix<double, DIM_N, 1> K;
	Matrix<double, DIM_N, DIM_M> B;
	Matrix<double, DIM_N, DIM_P> E;
	Matrix<double, DIM_N, DIM_PM> Em;

	Polyhedron<DIM_N+DIM_M> XU_set;  //using above Polyhedron struct
	Matrix<double, Dynamic, DIM_N + DIM_P> XD_plus;
	Matrix<double, Dynamic, DIM_N + DIM_P> XD_minus;

	double u_scale;
};

struct PwDynamics {

	// This function return the dynamics that are active in the region of a given point
	const Dynamics & get_dynamics(const VectorXd & x) const {  
		for (int i = 0; i != regions.size(); ++i) {
			if ( ( (regions[i].A * x).array() <= regions[i].b.array() ).all() )
				return dynamics[i];
		}
		return dynamics[0]; // ugly hack
	}

	vector<Polyhedron<DIM_N> > regions;
	vector<Dynamics> dynamics;
};


class QPHelper
{
	public:
		void load_controller(const string &);

		QPInfo get_qp_info(int setid, int polyid, VectorXd x0, MatrixXd Rx, VectorXd rx, MatrixXd Ru, VectorXd ru) const;
		void update_current_poly(const Matrix<double, DIM_N, 1> & x0, int & set, int & poly);
	private:
		template <typename Type>
		void read_item(std::ifstream & file, Type &);

		void read_sets(std::ifstream & file);

		// Sets of input constraints
		PwDynamics pwdyn;

		// Sets
		vector<vector<Polyhedron<DIM_N> > > sets;
};

ostream & operator<<(ostream &, const Dynamics &);

#endif
