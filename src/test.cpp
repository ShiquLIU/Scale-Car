#include "qp_vars.h"
#include <iostream>
using namespace std;

int main()
{
	qp_var test;

	VectorXd x0(4,1);
	x0 <<	.1,
		.1,
		0,
		.1;

	MatrixXd Rx(4,4);
	Rx << 	1, 2, 1, 1,
		2, 2, 2, 1,
		1, 2, 1, 1,
		1, 2, 1, 1;

	VectorXd rx(4,1);
	rx <<	1,
		2,
		1,
		1;

	VectorXd Ru(1,1);
	Ru <<	2;

	VectorXd ru(1,1);
	ru <<	1;

	VectorXd u(1,1);

	test.read_matlab();
	u = test.get_input(x0, Rx, rx, Ru, ru);

	cout << "Control response: " << u << endl;	

}