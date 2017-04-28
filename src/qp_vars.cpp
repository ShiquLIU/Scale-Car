#include "qp_vars.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

using Eigen::MatrixXd;
using namespace std;

qp_var::qp_var()
{
	dyn.A = MatrixXd::Zero(1,1);
	dyn.B = MatrixXd::Zero(1,1);
	Hx = MatrixXd::Zero(1,1);
	Hu = MatrixXd::Zero(1,1);
	hu = MatrixXd::Zero(1,1);
};

void qp_var::read_matlab()
{
	string file;
	int i = 1;

	ifstream infile("output_file2.txt");
	try
	{

		if(infile.is_open()) // check if file is opened
		{
			read_item(infile, dyn.A);
			read_item(infile, dyn.B);
			read_item(infile, dyn.E);
			read_item(infile, dyn.K);  //this order could be changed as needed
			read_item(infile, Hx);
			read_item(infile, hx);
			read_item(infile, k_feedback);
			read_item(infile, Hu);
			read_item(infile, hu);
			read_item(infile, XDplus);
			read_item(infile, XDminus);

			infile.close();
		}
		else throw "error";

/*
		while(1)
		{
			file = "sets_" + to_string(i) + ".txt";

			ifstream infile2(file);

			if(infile2.is_open()) read_sets(infile2, i);
			else
			{
				if(i == 1) throw 5;
				i--;
				break;
			}

			i++;
		}

		cout << "MATLAB data loaded successfully." << endl;
		cout << "Loaded a total of " << i << " set files." << endl << endl;

		cout << "Sets[1][3].A =" << endl << "---" << endl << Sets[1][3].A << endl << "---" << endl;

		cout << "Check line 200 of sets_2.txt to verify" << endl;*/
	}
	catch (string e){
		cout << "Error opening file (Expected output_file.txt)." << endl;  // output error if file did not open
	}

	//catch (int e){
	//	cout << "Error opening file (Expected set_i.txt where i is an integer)." << endl;  // output error if file did not open
	//}
}

VectorXd qp_var::get_feedback(VectorXd x0)
{
	return k_feedback*x0;
}

VectorXd qp_var::update_state(VectorXd x0, VectorXd u, VectorXd d)
{
	return dyn.A*x0 + dyn.B*u + dyn.E*d;
}

VectorXd qp_var::get_input(VectorXd x0, MatrixXd Rx, VectorXd rx, MatrixXd Ru, VectorXd ru)
{
	// min f(u) = x_1^T*Rx*x_1 + rx^T*x_1 + u^T*Ru*u + ru^T*u
	// s.t. x_1 = A*x_0 + B*u
	// u is in U
	// A*x_0 + B*u + E*d is in C_0

	// => min f(x) = 1/2*u^T*H*u + u^T*f
	// s.t. CE*x0 = ce0
	// and CI*x0 >= ci0

	VectorXd input(1, 1);
	input << 1;

	MatrixXd H = dyn.B.transpose()*Rx*dyn.B + Ru;
	VectorXd f = ru.transpose() + dyn.B.transpose()*Rx*dyn.A*x0 + dyn.B.transpose()*rx;

	MatrixXd CE = VectorXd::Zero(1, 1);
	VectorXd ce0 = VectorXd::Zero(1, 1);

	MatrixXd CI(Hu.transpose().rows(), 2*(Hx*dyn.B).transpose().cols() + Hu.transpose().cols());
	CI << -(Hx*dyn.B).transpose(), -(Hx*dyn.B).transpose(), -Hu.transpose();
	VectorXd ci0(2*(hx.rows()) + hu.rows(), hu.cols());
	ci0 << -hx + Hx*dyn.A*x0 + Hx*dyn.E*XDplus,
		-hx + Hx*dyn.A*x0 + Hx*dyn.E*XDminus,
		-hu;

	solve_quadprog(H, f,  CE, ce0,  CI, ci0, input);

	return input;
}
template <typename Type>
void qp_var::read_item(std::ifstream & file, Type & data_out)
{
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

/*
void qp_var::read_sets(std::ifstream & file, int number)
{
	int i = 0;

	vector<Polyhedron> new_set;

	while(!file.eof())
	{
		Polyhedron new_data;

		read_item(file, new_data.A);		// get data for the new set
		read_item(file, new_data.b);

		new_set.emplace_back(new_data);		// place new data into new set

		i++;
	}

	Sets.emplace_back(new_set);	// add new set to vector of sets
}*/
