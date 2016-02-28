#include "lsqrfit.h"


std::vector <cv::Point2f> crossing_points_g;

/*****************************************************************************
   If MODE = 0, compute  y = y + A*x,
   If MODE = 1, compute  x = x + A^T*y.
*****************************************************************************/
static void     
lsqr_eval_for_opt_(long mode, dvec *x, dvec *y, void *userdata)
{
//	int i, j, nparms = x->length;

	if (mode == 0) {
      // compute y = y + A*x
		for (auto j = 0; j < y->length; ++j) {
			double x_val = crossing_points_g[j].x;

			y->elements[j] += x->elements[0] * std::pow(x_val, 2) + x->elements[1] * x_val
								+ x->elements[2];
			//y->elements[j] += (1) * x->elements[j + 1]
			//					+ (-0.5 * lambda_i) * (x->elements[j] + x->elements[j + 2]);

//			y->elements[j+0] += _fromVector[j/3].x()*x->elements[0];
//			y->elements[j+0] += _fromVector[j/3].y()*x->elements[1];
//			y->elements[j+0] += _fromVector[j/3].z()*x->elements[2];
//
//			y->elements[j+1] += _fromVector[j/3].x()*x->elements[3];
//			y->elements[j+1] += _fromVector[j/3].y()*x->elements[4];
//			y->elements[j+1] += _fromVector[j/3].z()*x->elements[5];
//
//			y->elements[j+2] += _fromVector[j/3].x()*x->elements[6];
//			y->elements[j+2] += _fromVector[j/3].y()*x->elements[7];
//			y->elements[j+2] += _fromVector[j/3].z()*x->elements[8];
		}

	} else if (mode == 1) {
      // compute x = x + A^T*y
		for (auto i = 0; i < y->length; ++i) {
			double x_val = crossing_points_g[i].x;
			//double y_val = crossing_points_g[i].y;
			x->elements[0] += (std::pow(x_val, 2) * y->elements[i]);
			x->elements[1] += (x_val * y->elements[i]);
			x->elements[2] += ( y->elements[i]);
		}

//		for (auto i = 0; i < x->length; ++i) {

//			if (i == 0) {
//				x->elements[i] += (-0.5) * lambdas_g[i] * y->elements[i];
//			} else if (i == 1) {
//				x->elements[i] += (lambdas_g[i - 1] + 1) * y->elements[i - 1]
//							+ (-0.5) * lambdas_g[i] * y->elements[i];
//			} else if (i == (x->length - 2)) {
//				x->elements[i] += (lambdas_g[i - 1] + 1) * y->elements[i - 1]
//							+ (-0.5) * lambdas_g[i - 2] * y->elements[i - 2];
//			} else if (i == (x->length - 1)) {
//				x->elements[i] += (-0.5) * lambdas_g[i - 2] * y->elements[i - 2];
//			} else {
//				x->elements[i] += (-0.5) * lambdas_g[i - 2] * y->elements[i - 2]
//						+ (lambdas_g[i - 1] + 1) * y->elements[i - 1]
//						+ (-0.5) * lambdas_g[i] * y->elements[i];
//			}
//
//
//
//			x->elements[0] += _fromVector[j/3].x()*y->elements[j+0];
//			x->elements[1] += _fromVector[j/3].y()*y->elements[j+0];
//			x->elements[2] += _fromVector[j/3].z()*y->elements[j+0];
//
//			x->elements[3] += _fromVector[j/3].x()*y->elements[j+1];
//			x->elements[4] += _fromVector[j/3].y()*y->elements[j+1];
//			x->elements[5] += _fromVector[j/3].z()*y->elements[j+1];
//
//			x->elements[6] += _fromVector[j/3].x()*y->elements[j+2];
//			x->elements[7] += _fromVector[j/3].y()*y->elements[j+2];
//			x->elements[8] += _fromVector[j/3].z()*y->elements[j+2];
//		}
	} else {
		assert(false);
	}
}


/*****************************************************************************
*****************************************************************************/
double find_optimal_edge_zero_crossing(std::vector<cv::Point2f>& crossing_points) 
{
	// copy to globals
	//assert(fromVector.size() == toVector.size());
	//assert(fromVector.size() >= 3);
	//_fromVector = fromVector;
//	worlds_pts.resize(3);

	if (crossing_points.size() < 3) {
		// too little points to opitimize
		return -1;
	}



#ifdef DEBUG
	//for (auto i = 0u; i < lambdas.size(); ++i) {
	//	std::cout << std::setprecision(15) << lambdas[i] << std::endl;
	//}
#endif
	
	// allocate to globals
	crossing_points_g = crossing_points;

	// allocate structures for sparse linear least squares
	//printf("\tallocating for sparse linear least squares "
			 //"(%i vectors)...\n", fromVector.size());
	int num_rows = crossing_points.size();
	int num_cols = 3;

	lsqr_input *input = NULL;
	lsqr_output *output = NULL;
	lsqr_work *work = NULL;
	lsqr_func *func = NULL;
	alloc_lsqr_mem(&input, &output, &work, &func, num_rows, num_cols);
	input->num_rows = num_rows;
	input->num_cols = num_cols;
	input->damp_val = 0.0;
	input->rel_mat_err = 0.0;
	input->rel_rhs_err = 0.0;
	input->cond_lim = 0.0;
	input->max_iter = 10*input->num_cols;
	input->lsqr_fp_out = NULL;
	func->mat_vec_prod = lsqr_eval_for_opt_;

	// set rhs vec
	for (auto j = 0; j < num_rows; ++j) {
//	   input->rhs_vec->elements[j] = (1.0 - lambdas[j]) * worlds_pts[j+1][1];
	   input->rhs_vec->elements[j] = crossing_points[j].y;
	}

	// set initial sol vec
	for (auto i = 0u; i<num_cols; i++) {
//		input->sol_vec->elements[i] = worlds_pts[i][1];
		input->sol_vec->elements[i] = 0;
	}

	// call sparse linear least squares!
	//printf("\t\tstarting (rows=%i, cols=%i)...\n", num_rows, num_cols);
	lsqr(input, output, work, func, NULL);
	double error = output->resid_norm;
	//printf("\t\ttermination reason = %i\n", output->term_flag);
	//printf("\t\tnum function calls = %i\n", output->num_iters);
	//printf("\t\tremaining error = %lf\n", error);

	double a = input->sol_vec->elements[0];
	double b = input->sol_vec->elements[1];
	double c = input->sol_vec->elements[2];

	// solving for y = 0

	double solution = 0.0;
	double discriminant = std::pow(b, 2) - (4 * a * c);
	if (discriminant < 0) {
		// something went wrong
		solution = -1;
	} else {
		double delta = std::sqrt(discriminant);
		double sol_1 = ((-1 * b) + delta) / (2 * a);
		double sol_2 = ((-1 * b) - delta) / (2 * a);

		if (sol_1 <= crossing_points[crossing_points.size() - 1].x &&
			sol_1 >= crossing_points[0].x) {
			solution = sol_1;
		} else if (sol_2 <= crossing_points[crossing_points.size() - 1].x &&
			sol_2 >= crossing_points[0].x) {
			solution = sol_2;
		} else {
			// something wrong happened
			solution = -1;
		}

	}


	// free memory
   free_lsqr_mem(input, output, work, func);

	return (solution);
}

	
