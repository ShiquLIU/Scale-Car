#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include "qphelper.h"

using namespace Eigen;
using namespace std;

void QPHelper::load_controller(const string & dir) {
	ifstream infile;
	string filename;

	int i = 1;
	while (true) {
		filename = dir + "/dynamics_" + to_string(i) + ".txt";

		infile.open(filename);

		if(infile.is_open()) {
			Polyhedron<DIM_N> region;
			Dynamics dyn;
			read_item(infile, region.A);
			read_item(infile, region.b);
			read_item(infile, dyn.A);
			read_item(infile, dyn.K);
			read_item(infile, dyn.B);
			read_item(infile, dyn.E);
			read_item(infile, dyn.Em);
			read_item(infile, dyn.XU_set.A);
			read_item(infile, dyn.XU_set.b);
			read_item(infile, dyn.XD_plus);
			read_item(infile, dyn.XD_minus);
			
			// ugly hack
			Matrix<double, 1, 1> number;
			read_item(infile, number);
			dyn.u_scale = number(0,0);

			pwdyn.regions.push_back(move(region));
			pwdyn.dynamics.push_back(move(dyn));
			infile.close();
		} else {
		  if(i == 1) {
			  cout << "could not open any dynamics file" << endl;
				throw 5;
		  }
			i--;
			break;
		}

		i++;
	}

	cout << "Loaded " << i << " dynamics" << endl;

	i = 1;
	while (true) {
		filename = dir + "/sets_" + to_string(i) + ".txt";

		infile.open(filename);

		if(infile.is_open()) {
			read_sets(infile);
			infile.close();
		} else {
		  if(i == 1) {
		    cout << "could not open any set file" << endl;
				throw 5;
		  }
			i--;
			break;
		}
		i++;
	}

	cout << "Loaded " << i << " sets." << endl;
}

template <typename Type>
void QPHelper::read_item(std::ifstream & file, Type & data_out) {
	int rows, cols;
	float data_in;
	file >> rows >> cols;
	data_out.resize(rows, cols);	// resize matrix or vector to correct dimensions

	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols; j++)
		{
			file >> data_in;
			data_out(i,j) = data_in;	// put data into matrix or vector
		}
	}
}

void QPHelper::read_sets(std::ifstream & file) {
	int i = 0;

	vector<Polyhedron<DIM_N> > new_set;

	int num_sets;
	file >> num_sets;

	for (int i = 0; i != num_sets; ++i)
	{
		Polyhedron<DIM_N> new_data;

		read_item(file, new_data.A);		// get data for the new set
		read_item(file, new_data.b);

		new_set.push_back(new_data);		// place new data into new set
	}

	sets.push_back(move(new_set));	// add new set to vector of sets
}

QPInfo QPHelper::get_qp_info(int setid, int polyid, VectorXd x0, MatrixXd Rx, VectorXd rx, 
													    MatrixXd Ru, VectorXd ru) const {

	const Dynamics & dyn = pwdyn.get_dynamics(x0);

	const Polyhedron<DIM_N> & final_poly = sets[setid][polyid];

	int n = dyn.A.cols();
	int m = dyn.B.cols();
	int p = dyn.E.cols();

	QPInfo res;

	res.A_ineq.resize(dyn.XU_set.num_ineq() + 2 * final_poly.num_ineq(), m);
	res.b_ineq.resize(dyn.XU_set.num_ineq() + 2 * final_poly.num_ineq());
	res.H.resize(m,m);
	res.f.resize(m);

	res.A_ineq.block(0, 0, dyn.XU_set.num_ineq(), m) = dyn.XU_set.A.block(0,n,dyn.XU_set.A.rows(), m);
	res.A_ineq.block(dyn.XU_set.num_ineq(), 0, final_poly.num_ineq(), m) = final_poly.A * dyn.B;
	res.A_ineq.block(dyn.XU_set.num_ineq() + final_poly.num_ineq(), 0, final_poly.num_ineq(), m) = final_poly.A * dyn.B;

	res.b_ineq.segment(0, dyn.XU_set.num_ineq()) = dyn.XU_set.b -  dyn.XU_set.A.block(0, 0, dyn.XU_set.A.rows(), n) * x0;
	res.b_ineq.segment(dyn.XU_set.num_ineq(), final_poly.num_ineq()) = 
		final_poly.b - final_poly.A * ( 
			dyn.E * dyn.XD_plus.block(0,n,dyn.XD_plus.rows(),p)
			+ dyn.K + dyn.A * x0 + dyn.E * dyn.XD_plus.block(0,0,dyn.XD_plus.rows(),n) * x0
		);
	res.b_ineq.segment(dyn.XU_set.num_ineq() + final_poly.num_ineq(), final_poly.num_ineq()) = 
		final_poly.b - final_poly.A * ( 
			dyn.E * dyn.XD_minus.block(0,n,dyn.XD_minus.rows(),p)
			+ dyn.K + dyn.A * x0 + dyn.E * dyn.XD_minus.block(0,0,dyn.XD_minus.rows(),n) * x0
		);

	res.H = Ru + dyn.B.transpose() * Rx * dyn.B;
	res.f = ru + dyn.B.transpose() * Rx * (dyn.A * x0 + dyn.K) + dyn.B.transpose() * rx;
	res.u_scale = dyn.u_scale; // copy u_scale of current dynamics
	return move(res);
}

void QPHelper::update_current_poly(const Matrix<double, DIM_N, 1> & x0, int & set, int & poly) {
	// See if we are in the same one!
	if ( ( (sets[set][poly].A * x0).array() <= sets[set][poly].b.array() ).all() ) {
		return; // do nothing
	} else if (set > 0 && ( ( (sets[poly][set-1].A * x0).array() <= sets[poly][set-1].b.array() ).all() )) {
		--set;
		return;
	} else if (poly > 0 && ( ( (sets[poly-1][set].A * x0).array() <= sets[poly-1][set].b.array() ).all() )) {
		--poly;
		return;
	}
	// do exhaustive search
	for (int s = 0; s != sets.size(); ++s) {
		for (int p = 0; p != sets[s].size(); ++p) {
			if ( ( (sets[s][p].A * x0).array() <= sets[s][p].b.array() ).all() ) {
				set = s;
				poly = p;
				return;
			}
		}
	}
}

template <int dim>
ostream & operator<<(ostream & os, const Polyhedron<dim> & poly) {
	IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	os << "A: " <<  endl << poly.A.format(OctaveFmt) << endl << endl;
	os << "b: " << endl << poly.b.format(OctaveFmt) << endl << endl;
	return os;
}

ostream & operator<<(ostream & os, const Dynamics & dyn) {
	IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	os << "======================================" << endl;
	os << "===========  Dynamics: ===============" << endl;
	os << "======================================" << endl;
	os << "A: " <<  endl << dyn.A.format(OctaveFmt) << endl << endl;
	os << "K: " << endl << dyn.K.format(OctaveFmt) << endl << endl;
	os << "B: " << endl << dyn.B.format(OctaveFmt) << endl << endl;
	os << "E: " << endl << dyn.E.format(OctaveFmt) << endl << endl;
	os << "Em: " << endl << dyn.Em.format(OctaveFmt) << endl << endl;
	os << "XU_set: " << endl << dyn.XU_set << endl << endl;
	os << "XD_plus: " << endl << dyn.XD_plus.format(OctaveFmt) << endl << endl;
	os << "XD_minus: " << endl << dyn.XD_minus.format(OctaveFmt) << endl;
	os << "======================================" << endl;
	return os;
}
