#include "swarmopt.h"
#include <math.h>
#include <malloc.h>


extern "C" int mylmdif_(int (*fcn)(int *, int *, double *, double *, int *), int *m, int *n, double *x, double *fvec, double *ftol, double *xtol, double *gtol, int *maxfev, 
	double *epsfcn, double *diag, int *mode, double *factor, int *nprint, int *info, int *nfev, double *fjac, int *ldfjac, int *ipvt, 
	double *qtf, double *wa1, double *wa2, double *wa3, double *wa4);
//
//
///*****************************************************************************
//*****************************************************************************/

double min_time_taken_g;
double max_coverage_g;
double max_multi_sampling_g;

SwarmViewer* SwarmOptimizer::swarm_viewer_g;
double SwarmOptimizer::separation_constant_;

int
SwarmOptimizer::swarm_sim_opt_error_(int *m_ptr, int *n_ptr, double *params, double *error, int *)
{
	int nparms = *n_ptr;
	int nerrors = *m_ptr;


	 // 0 - separation constant
	 // 1 - alignment constant
	 // 2 - cluster constant
	 // 3 - perimeter constant
	 // 4 - explore constant
	 // 5 - separation range max (min = 0)
	 // 6 - alignment range max ( min = separation range max )
	 // 7 - cluster range min 
	 // 8 - cluster range max
	 // perimeter range - 0 - 0.5 of diagonal in square
	 // explore range - 0 - diagonal of square
	 // obstacle avoid near range - 0 - 0.5
	 // obstacle avoid far range - 0.5 - 2
	 // sensor range - measure input - 0 - 6
	 // discovery range - measured input - 0 - 1
	 // 9 - preferred neighbor count

	
	double scaling_constant = 1000.f;
	double current_separation_constant = (separation_constant_ + scaling_constant * params[0]);
	current_separation_constant = (current_separation_constant > 0) ? current_separation_constant : 0;
	swarm_viewer_g->set_separation_constant(current_separation_constant);
	//swarm_viewer_g->set_alignment_constant(params[1]);
	//swarm_viewer_g->set_cluster_constant(params[2]);
	//swarm_viewer_g->set_perimeter_constant(params[3]);
	//swarm_viewer_g->set_exploration_constant(params[4]);

	//Range separation_range(0, params[5]);
	//Range alignment_range(params[5], params[6]);
	//Range cluster_range(params[7], params[8]);

	//swarm_viewer_g->set_separation_range(separation_range.min_, separation_range.max_);
	//swarm_viewer_g->set_alignment_range(alignment_range.min_, alignment_range.max_);
	//swarm_viewer_g->set_cluster_range(cluster_range.min_, cluster_range.max_);


	//swarm_viewer_g->set_preferred_neighbor_count(params[9]);
	emit swarm_viewer_g->optimizer_reset_sim();
	double current_time_taken;
	double current_multi_sampling;
	double current_coverage;

	swarm_viewer_g->get_sim_results(current_time_taken, current_multi_sampling, current_coverage);

	std::cout << "Time taken for step : " << current_time_taken << std::endl;
	std::cout << "Multi sampling for step : " << current_multi_sampling << std::endl;
	std::cout << "Current coverage : " << current_coverage << std::endl;

	std::cout << "Separation Constant : " << swarm_viewer_g->separation_constant_ << std::endl;

	// minimize time taken
	// maximize multi sampling
	// maximize coverage

	error[0] = std::pow(current_time_taken, 2);
	error[1] = std::pow(current_multi_sampling - max_multi_sampling_g, 2);
	error[2] = std::pow(current_coverage - max_coverage_g, 2);

	return 1;
}

void SwarmOptimizer::set_optimize_swarm_params(SwarmViewer* swarm_viewer, 
	double& seperation_constant, double& alignment_constant, double& cluster_constant, double& perimeter_constant, 
	double& explore_constant, Range& separation_range, Range& alignment_range, 
	Range& cluster_range, int& preferred_neighborhood_count) {

	separation_constant_ = seperation_constant;
	swarm_viewer_g = swarm_viewer;
	//double& seperation_constant, double& alignment_constant, double& cluster_constant, double& perimeter_constant, 
	//double& explore_constant, Range& separation_range, Range& alignment_range, 
	//Range& cluster_range, int& preferred_neighborhood_count


}


/*****************************************************************************
*****************************************************************************/
/* Parameters controlling MINPACK's lmdif() optimization routine. */
/* See the file lmdif.f for definitions of each parameter.        */
#define REL_SENSOR_TOLERANCE_ftol    1.0E-6      /* [pix] */
#define REL_PARAM_TOLERANCE_xtol     1.0E-7
#define ORTHO_TOLERANCE_gtol         0.0
#define MAXFEV                       (1000*n)
#define EPSFCN                       1.0E-10 /* was E-16 Do not set to 0! */
#define MODE                         2       /* variables scaled internally */
#define FACTOR                       100.0 

void 
SwarmOptimizer::optimize_swarm_params() {

    /* Parameters needed by MINPACK's lmdif() */
	int     n = 1;
	int     m = 3;
    double *x;
    double *fvec;
    double  ftol = REL_SENSOR_TOLERANCE_ftol;
    double  xtol = REL_PARAM_TOLERANCE_xtol;
    double  gtol = ORTHO_TOLERANCE_gtol;
    int     maxfev = MAXFEV;
    double  epsfcn = EPSFCN;
    double *diag;
    int     mode = MODE;
    double  factor = FACTOR;
    int     ldfjac = m;
    int     nprint = 0;
    int     info;
    int     nfev;
    double *fjac;
    int    *ipvt;
    double *qtf;
    double *wa1;
    double *wa2;
    double *wa3;
    double *wa4;
	 //double worldSize = non_zero_vals.rows;

	 const int num = 1;
	 double params[num];

	 // 0 - separation constant
	 // 1 - alignment constant
	 // 2 - cluster constant
	 // 3 - perimeter constant
	 // 4 - explore constant
	 // 5 - separation range max (min = 0)
	 // 6 - alignment range max ( min = separation range max )
	 // 7 - cluster range min 
	 // 8 - cluster range max
	 // perimeter range - 0 - 0.5 of diagonal in square
	 // explore range - 0 - diagonal of square
	 // obstacle avoid near range - 0 - 0.5
	 // obstacle avoid far range - 0.5 - 2
	 // sensor range - measure input - 0 - 6
	 // discovery range - measured input - 0 - 1
	 // 9 - preferred neighbor count


	 params[0] = 0.0;
	 //params[1] = alignment_constant;
	 //params[2] = cluster_constant;
	 //params[3] = perimeter_constant;
	 //params[4] = explore_constant;

	 //params[5] = separation_range.max_;
	 //params[6] = alignment_range.max_;
	 //params[7] = cluster_range.min_;
	 //params[8] = cluster_range.max_;
	 //params[9] = preferred_neighborhood_count;

	 /* copy to globals */

	 min_time_taken_g = 0;
	 max_coverage_g = std::pow(swarm_viewer_g->grid_resolution_per_side_, 2);
	 max_multi_sampling_g = swarm_viewer_g->no_of_robots_;

	 //perimeter_range_g = perimeter_range;
	 //explore_range_g = explore_range;
	 //obstacle_avoidance_near_range_g = obstacle_avoidance_near_range;
	 //obstacle_avoidance_far_range_g = obstacle_avoidance_far_range;
	 //sensor_range_g = sensor_range;
	 //discovery_range;


    /* allocate stuff dependent on n */
    x    = (double *)calloc(n, sizeof(double));
    diag = (double *)calloc(n, sizeof(double));
    qtf  = (double *)calloc(n, sizeof(double));
    wa1  = (double *)calloc(n, sizeof(double));
    wa2  = (double *)calloc(n, sizeof(double));
    wa3  = (double *)calloc(n, sizeof(double));
    ipvt = (int    *)calloc(n, sizeof(int));

    /* allocate some workspace */
    if (( fvec = (double *) calloc ((unsigned int) m, 
                                    (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace fvec\n");
       exit(-1);
    }

    if (( fjac = (double *) calloc ((unsigned int) m*n,
                                    (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace fjac\n");
       exit(-1);
    }

    if (( wa4 = (double *) calloc ((unsigned int) m, 
                                   (unsigned int) sizeof(double))) == NULL ) {
       fprintf(stderr,"calloc: Cannot allocate workspace wa4\n");
       exit(-1);
    }


    /* copy parameters in as initial values */
	//std::cout << "initial val. :";
	for (int i = 0; i < n; i++) {
       x[i] = params[i];
	    //std::cout << x[i] << ", ";
	}
	//std::cout << std::endl;

    /* define optional scale factors for the parameters */
    if ( mode == 2 ) {
		for (int offset = 0; offset<n; offset++) {
			diag[offset] = 1.0;
		}
    }

    /* perform the optimization */ 
    printf("Starting optimization step...\n");
    mylmdif_ (&SwarmOptimizer::swarm_sim_opt_error_,
            &m, &n, x, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn,
            diag, &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac,
            ipvt, qtf, wa1, wa2, wa3, wa4);
    double totalerror = 0;
    for (int i=0; i<m; i++) {
       totalerror += fvec[i];
	}
    printf("\tnum function calls = %i\n", nfev);
    printf("\tremaining total error value = %f\n", totalerror);
    printf("\tor %1.2f per point\n", std::sqrt(totalerror) / m);
    printf("...ended optimization step.\n");

    /* copy result back to parameters array */
	std::cout << "final val. :";
    for (int i=0; i<n; i++) {
       params[i] = x[i];
	   std::cout << x[i] << ", ";
	}
	std::cout << std::endl;



    /* release allocated workspace */
    free (fvec);
    free (fjac);
    free (wa4);
    free (ipvt);
    free (wa1);
    free (wa2);
    free (wa3);
    free (qtf);
    free (diag);
    free (x);

	emit finished();
	 //return (1);
	
}

void SwarmMCMCOptimizer::set_viewer(SwarmViewer* swarm_viewer) {
	swarm_viewer_ = swarm_viewer;
}

double SwarmMCMCOptimizer::run_simulation(double separation_constant) {

	 // 0 - separation constant
	 // 1 - alignment constant
	 // 2 - cluster constant
	 // 3 - perimeter constant
	 // 4 - explore constant
	 // 5 - separation range max (min = 0)
	 // 6 - alignment range max ( min = separation range max )
	 // 7 - cluster range min 
	 // 8 - cluster range max
	 // perimeter range - 0 - 0.5 of diagonal in square
	 // explore range - 0 - diagonal of square
	 // obstacle avoid near range - 0 - 0.5
	 // obstacle avoid far range - 0.5 - 2
	 // sensor range - measure input - 0 - 6
	 // discovery range - measured input - 0 - 1
	 // 9 - preferred neighbor count

	
	//double scaling_constant = 100.f;
	separation_constant = (separation_constant > 0) ? separation_constant : 0;
	swarm_viewer_->set_separation_constant(separation_constant);
	//swarm_viewer_g->set_alignment_constant(params[1]);
	//swarm_viewer_g->set_cluster_constant(params[2]);
	//swarm_viewer_g->set_perimeter_constant(params[3]);
	//swarm_viewer_g->set_exploration_constant(params[4]);

	//Range separation_range(0, params[5]);
	//Range alignment_range(params[5], params[6]);
	//Range cluster_range(params[7], params[8]);

	//swarm_viewer_g->set_separation_range(separation_range.min_, separation_range.max_);
	//swarm_viewer_g->set_alignment_range(alignment_range.min_, alignment_range.max_);
	//swarm_viewer_g->set_cluster_range(cluster_range.min_, cluster_range.max_);


	//swarm_viewer_g->set_preferred_neighbor_count(params[9]);
	emit swarm_viewer_->optimizer_reset_sim();
	double current_time_taken;
	double current_multi_sampling;
	double current_coverage;

	swarm_viewer_->get_sim_results(current_time_taken, current_multi_sampling, current_coverage);

	//std::cout << "Time taken for step : " << current_time_taken << std::endl;
	//std::cout << "Multi sampling for step : " << current_multi_sampling << std::endl;
	//std::cout << "Current coverage : " << current_coverage << std::endl;

	//std::cout << "Separation Constant : " << swarm_viewer_->separation_constant_ << std::endl;

	// minimize time taken
	// maximize multi sampling
	// maximize coverage

	double score = 0.0;
	score += std::pow(10000 - current_time_taken, 2);
	//score += std::pow(current_multi_sampling - max_multi_sampling_, 2);
	//score += std::pow(current_coverage - max_coverage_, 2);

	return score;
}
	

void SwarmMCMCOptimizer::optimize_swarm_params() {

	//Algorithm.MCMC(params)

	//	1. initialize params(just randomly select values)

	//	2. current_score = Score(params)

	//	3. best_score = current_score
	//	4. Repeat for a lot of iterations

	//	5.     obtain next_params by randomly changing params

	//	(One approach is to select one param and add a small delta to change its value.)
	//	6.     next_score = Score(next_params)

	//	7.     If next_score > best_score

	//	8.         best_score = next_score
	//	9.         best_params = params
	//	10.   If next_score >= score or next_score / score >= uniform_rand(0, 1)

	//	11.       params = next_params

	//	12.       score = next_score

	//	13. Return best_params


	// initialize params
	double current_separation_constant = swarm_viewer_->separation_constant_;
	double scaling_constant = 100.f;
	
	// current score
	double current_score = run_simulation(current_separation_constant);

	double best_score = current_score;
	double best_separation_constant = current_separation_constant;

	int no_of_iterations = 1000000;

	std::random_device rd;
	std::mt19937 mt(rd());
	std::normal_distribution<> normal_distribution(1, 0.5);
	std::uniform_real_distribution<> uniform_real_distribution(0, 1);
	

	for (int i = 0; i < no_of_iterations; ++i) {
		double separation_delta = normal_distribution(mt);
		double next_separation_constant = (current_separation_constant + scaling_constant * separation_delta);

		double next_score = run_simulation(next_separation_constant);

		if (next_score > best_score) {
			best_score = next_score;
			best_separation_constant = next_separation_constant;
			std::cout << "Best time step : " << best_score << " best separation constant : " << best_separation_constant << std::endl;
		}

		double uniform_random_value = uniform_real_distribution(mt);

		if ((next_score >= current_score)
			|| ((next_score / current_score) >= uniform_random_value)) {

			current_score = next_score;
			current_separation_constant = next_separation_constant;
			//std::cout << "Accepted time step : " << current_score << " accepted separation constant : " << current_separation_constant << std::endl;
		}
	}




}
