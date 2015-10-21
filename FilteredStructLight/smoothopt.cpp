#include "smoothopt.h"
#include <assert.h>


// for use by lsqr_eval_for_opt
static std::vector<double> lambdas_g;



/*****************************************************************************
   If MODE = 0, compute  y = y + A*x,
   If MODE = 1, compute  x = x + A^T*y.
*****************************************************************************/
static void     
lsqr_eval_for_opt(long mode, dvec *x, dvec *y, void *userdata)
{
//	int i, j, nparms = x->length;

	if (mode == 0) {
      // compute y = y + A*x
		for (auto j = 0; j < y->length; ++j) {
			double lambda_i = lambdas_g[j];
			y->elements[j] += (lambda_i + 1) * x->elements[j + 1]
								+ (-0.5 * lambda_i) * (x->elements[j] + x->elements[j + 2]);

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
		for (auto i = 0; i < x->length; ++i) {
			if (i == 0) {
				x->elements[i] += (-0.5) * lambdas_g[i] * y->elements[i];
			} else if (i == 1) {
				x->elements[i] += (lambdas_g[i - 1] + 1) * y->elements[i - 1]
							+ (-0.5) * lambdas_g[i] * y->elements[i];
			} else if (i == (x->length - 2)) {
				x->elements[i] += (lambdas_g[i - 1] + 1) * y->elements[i - 1]
							+ (-0.5) * lambdas_g[i - 2] * y->elements[i - 2];
			} else if (i == (x->length - 1)) {
				x->elements[i] += (-0.5) * lambdas_g[i - 2] * y->elements[i - 2];
			} else {
				x->elements[i] += (-0.5) * lambdas_g[i - 2] * y->elements[i - 2]
						+ (lambdas_g[i - 1] + 1) * y->elements[i - 1]
						+ (-0.5) * lambdas_g[i] * y->elements[i];
			}
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
		}
	} else {
		assert(false);
	}
}


/*****************************************************************************
*****************************************************************************/
float
optimize_smoothness(WPt& worlds_pts, const IntensityPerImage& left_intensities, const IntensityPerImage& right_intensities)
{
	// copy to globals
	//assert(fromVector.size() == toVector.size());
	//assert(fromVector.size() >= 3);
	//_fromVector = fromVector;

	double adjustment_rate = 0.01;

	std::vector<double> lambdas;
	lambdas.resize(worlds_pts.size() - 2);
	for (auto w = 0; w < worlds_pts.size();++w) {
		if (((w - 1) >= 0) && ((w + 1) < (worlds_pts.size() - 1))) {
			double z_top = worlds_pts[w - 1][2];
			double z_bottom = worlds_pts[w + 1][2];
			double z_0 = worlds_pts[w][2];
			double delta_z = (z_top - z_bottom) + (z_0 - z_top) - (z_bottom - z_0);

			double omega_i = std::min(left_intensities[w], right_intensities[w]);
			omega_i /= 255.0;

			double lambda_i = (1.0 - delta_z) * omega_i * adjustment_rate;
			lambdas[w - 1] = lambda_i;


			double y_top = worlds_pts[w - 1][1];
			double y_bottom = worlds_pts[w + 1][1];
			double y_0 = worlds_pts[w][1];
			
			double y = worlds_pts[w][1];
			double y_prime = w;
		}
	}
	
	// allocate to globals
	lambdas_g = lambdas;

	// allocate structures for sparse linear least squares
	//printf("\tallocating for sparse linear least squares "
			 //"(%i vectors)...\n", fromVector.size());
	int num_rows = worlds_pts.size() - 2;
	int num_cols = worlds_pts.size();
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
	input->max_iter = 4*input->num_cols;
	input->lsqr_fp_out = NULL;
	func->mat_vec_prod = lsqr_eval_for_opt;

	// set rhs vec
	for (auto j = 1u; j < num_rows - 1; ++j) {
	   input->rhs_vec->elements[j] = worlds_pts[j][1];
	}

	// set initial sol vec
	for (auto i = 0u; i<num_cols; i++) {
		input->sol_vec->elements[i] = worlds_pts[i][1];
	}

	// call sparse linear least squares!
	printf("\t\tstarting (rows=%i, cols=%i)...\n", num_rows, num_cols);
	lsqr(input, output, work, func, NULL);
	double error = output->resid_norm;
	printf("\t\ttermination reason = %i\n", output->term_flag);
	printf("\t\tnum function calls = %i\n", output->num_iters);
	printf("\t\tremaining error = %lf\n", error);

	for (auto i = 0u; i < worlds_pts.size(); ++i) {
		worlds_pts[i][1] = output->sol_vec->elements[i];
	}

	// free memory
   free_lsqr_mem(input, output, work, func);

	return (error);
}
