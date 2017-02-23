#include "swarmopt.h"
#include <math.h>
#include <malloc.h>
#include <QtCore/qeventloop.h>
#include <chrono>
#include <fstream>
#include <QtCore/qcoreapplication.h>
#include <qthreadpool.h>
#include "simulatorthread.h"
#include "swarmutils.h"
#include <iomanip>


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

//double ParallelMCMCOptimizer::MAX_SEPARATION_VALUE = 30.0;
//double ParallelMCMCOptimizer::MAX_ALIGNMENT_VALUE = 10.0;
//double ParallelMCMCOptimizer::MAX_CLUSTER_VALUE = 50.0;
//double ParallelMCMCOptimizer::MAX_EXPLORE_VALUE = 10.0;
//double ParallelMCMCOptimizer::MAX_BOUNCE_MULTIPLIER_VALUE = 50.0;

double ParallelMCMCOptimizer::MAX_SEPARATION_VALUE = 1.0;
double ParallelMCMCOptimizer::MAX_ALIGNMENT_VALUE = MAX_SEPARATION_VALUE;
double ParallelMCMCOptimizer::MAX_CLUSTER_VALUE = MAX_SEPARATION_VALUE;
double ParallelMCMCOptimizer::MAX_EXPLORE_VALUE = MAX_SEPARATION_VALUE;
double ParallelMCMCOptimizer::MAX_BOUNCE_MULTIPLIER_VALUE = MAX_SEPARATION_VALUE;

double ParallelMCMCOptimizer::MIN_SEPARATION_VALUE = 0.0;
double ParallelMCMCOptimizer::MIN_ALIGNMENT_VALUE = 0.0;
double ParallelMCMCOptimizer::MIN_CLUSTER_VALUE = 0.0;
double ParallelMCMCOptimizer::MIN_EXPLORE_VALUE = 0.0;
double ParallelMCMCOptimizer::MIN_BOUNCE_MULTIPLIER_VALUE = 0.0;

//int
//SwarmOptimizer::swarm_sim_opt_error_(int *m_ptr, int *n_ptr, double *params, double *error, int *)
//{
//	int nparms = *n_ptr;
//	int nerrors = *m_ptr;
//
//
//	 // 0 - separation constant
//	 // 1 - alignment constant
//	 // 2 - cluster constswarm_paramsant
//	 // 3 - perimeter constant
//	 // 4 - explore constant
//	 // 5 - separation range max (min = 0)
//	 // 6 - alignment range max ( min = separation range max )
//	 // 7 - cluster range min 
//	 // 8 - cluster range max
//	 // perimeter range - 0 - 0.5 of diagonal in square
//	 // explore range - 0 - diagonal of square
//	 // obstacle avoid near range - 0 - 0.5
//	 // obstacle avoid far range - 0.5 - 2
//	 // sensor range - measure input - 0 - 6
//	 // discovery range - measured input - 0 - 1
//	 // 9 - preferred neighbor count
//
//	
//	double scaling_constant = 1000.f;
//	double current_separation_constant = (separation_constant_ + scaling_constant * params[0]);
//	current_separation_constant = (current_separation_constant > 0) ? current_separation_constant : 0;
//	swarm_viewer_g->set_separation_constant(current_separation_constant);
//	//swarm_viewer_g->set_alignment_constant(params[1]);
//	//swarm_viewer_g->set_cluster_constant(params[2]);
//	//swarm_viewer_g->set_perimeter_constant(params[3]);
//	//swarm_viewer_g->set_exploration_constant(params[4]);
//
//	//Range separation_range(0, params[5]);
//	//Range alignment_range(params[5], params[6]);
//	//Range cluster_range(params[7], params[8]);
//
//	//swarm_viewer_g->set_separation_range(separation_range.min_, separation_range.max_);
//	//swarm_viewer_g->set_alignment_range(alignment_range.min_, alignment_range.max_);
//	//swarm_viewer_g->set_cluster_range(cluster_range.min_, cluster_range.max_);
//
//
//	//swarm_viewer_g->set_preferred_neighbor_count(params[9]);
//	emit swarm_viewer_g->optimizer_reset_sim();
//	double current_time_taken;
//	double current_multi_sampling;
//	double current_coverage;
//	double occlusion;
//
//	swarm_viewer_g->get_sim_results(current_time_taken, current_multi_sampling, current_coverage, occlusion);
//
//	std::cout << "Time taken for step : " << current_time_taken << std::endl;
//	std::cout << "Multi sampling for step : " << current_multi_sampling << std::endl;
//	std::cout << "Current coverage : " << current_coverage << std::endl;
//
//	std::cout << "Separation Constant : " << swarm_viewer_g->separation_constant_ << std::endl;
//
//	// minimize time taken
//	// maximize multi sampling
//	// maximize coverage
//
//	error[0] = std::pow(current_time_taken, 2);
//	error[1] = std::pow(current_multi_sampling - max_multi_sampling_g, 2);
//	error[2] = std::pow(current_coverage - max_coverage_g, 2);
//
//	return 1;
//}
//
//void SwarmOptimizer::set_optimize_swarm_params(SwarmViewer* swarm_viewer, 
//	double& seperation_constant, double& alignment_constant, double& cluster_constant, double& perimeter_constant, 
//	double& explore_constant, Range& separation_range, Range& alignment_range, 
//	Range& cluster_range, int& preferred_neighborhood_count) {
//
//	separation_constant_ = seperation_constant;
//	swarm_viewer_g = swarm_viewer;
//	//double& seperation_constant, double& alignment_constant, double& cluster_constant, double& perimeter_constant, 
//	//double& explore_constant, Range& separation_range, Range& alignment_range, 
//	//Range& cluster_range, int& preferred_neighborhood_count
//
//
//}
//
//
///*****************************************************************************
//*****************************************************************************/
///* Parameters controlling MINPACK's lmdif() optimization routine. */
///* See the file lmdif.f for definitions of each parameter.        */
//#define REL_SENSOR_TOLERANCE_ftol    1.0E-6      /* [pix] */
//#define REL_PARAM_TOLERANCE_xtol     1.0E-7
//#define ORTHO_TOLERANCE_gtol         0.0
//#define MAXFEV                       (1000*n)
//#define EPSFCN                       1.0E-10 /* was E-16 Do not set to 0! */
//#define MODE                         2       /* variables scaled internally */
//#define FACTOR                       100.0 
//
//void 
//SwarmOptimizer::optimize_swarm_params() {
//
//    /* Parameters needed by MINPACK's lmdif() */
//	int     n = 1;
//	int     m = 3;
//    double *x;
//    double *fvec;
//    double  ftol = REL_SENSOR_TOLERANCE_ftol;
//    double  xtol = REL_PARAM_TOLERANCE_xtol;
//    double  gtol = ORTHO_TOLERANCE_gtol;
//    int     maxfev = MAXFEV;
//    double  epsfcn = EPSFCN;
//    double *diag;
//    int     mode = MODE;
//    double  factor = FACTOR;
//    int     ldfjac = m;
//    int     nprint = 0;
//    int     info;
//    int     nfev;
//    double *fjac;
//    int    *ipvt;
//    double *qtf;
//    double *wa1;
//    double *wa2;
//    double *wa3;
//    double *wa4;
//	 //double worldSize = non_zero_vals.rows;
//
//	 const int num = 1;
//	 double params[num];
//
//	 // 0 - separation constant
//	 // 1 - alignment constant
//	 // 2 - cluster constant
//	 // 3 - perimeter constant
//	 // 4 - explore constant
//	 // 5 - separation range max (min = 0)
//	 // 6 - alignment range max ( min = separation range max )
//	 // 7 - cluster range min 
//	 // 8 - cluster range max
//	 // perimeter range - 0 - 0.5 of diagonal in square
//	 // explore range - 0 - diagonal of square
//	 // obstacle avoid near range - 0 - 0.5
//	 // obstacle avoid far range - 0.5 - 2
//	 // sensor range - measure input - 0 - 6
//	 // discovery range - measured input - 0 - 1
//	 // 9 - preferred neighbor count
//
//
//	 params[0] = 0.0;
//	 //params[1] = alignment_constant;
//	 //params[2] = cluster_constant;
//	 //params[3] = perimeter_constant;
//	 //params[4] = explore_constant;
//
//	 //params[5] = separation_range.max_;
//	 //params[6] = alignment_range.max_;
//	 //params[7] = cluster_range.min_;
//	 //params[8] = cluster_range.max_;
//	 //params[9] = preferred_neighborhood_count;
//
//	 /* copy to globals */
//
//	 min_time_taken_g = 0;
//	 //max_coverage_g = std::pow(swarm_viewer_g->grid_resolution_per_side_, 2);
//	 //max_multi_sampling_g = swarm_viewer_g->no_of_robots_;
//
//	 //perimeter_range_g = perimeter_range;
//	 //explore_range_g = explore_range;
//	 //obstacle_avoidance_near_range_g = obstacle_avoidance_near_range;
//	 //obstacle_avoidance_far_range_g = obstacle_avoidance_far_range;
//	 //sensor_range_g = sensor_range;
//	 //discovery_range;
//
//
//    /* allocate stuff dependent on n */
//    x    = (double *)calloc(n, sizeof(double));
//    diag = (double *)calloc(n, sizeof(double));
//    qtf  = (double *)calloc(n, sizeof(double));
//    wa1  = (double *)calloc(n, sizeof(double));
//    wa2  = (double *)calloc(n, sizeof(double));
//    wa3  = (double *)calloc(n, sizeof(double));
//    ipvt = (int    *)calloc(n, sizeof(int));
//
//    /* allocate some workspace */
//    if (( fvec = (double *) calloc ((unsigned int) m, 
//                                    (unsigned int) sizeof(double))) == NULL ) {
//       fprintf(stderr,"calloc: Cannot allocate workspace fvec\n");
//       exit(-1);
//    }
//
//    if (( fjac = (double *) calloc ((unsigned int) m*n,
//                                    (unsigned int) sizeof(double))) == NULL ) {
//       fprintf(stderr,"calloc: Cannot allocate workspace fjac\n");
//       exit(-1);
//    }
//
//    if (( wa4 = (double *) calloc ((unsigned int) m, 
//                                   (unsigned int) sizeof(double))) == NULL ) {
//       fprintf(stderr,"calloc: Cannot allocate workspace wa4\n");
//       exit(-1);
//    }
//
//
//    /* copy parameters in as initial values */
//	//std::cout << "initial val. :";
//	for (int i = 0; i < n; i++) {
//       x[i] = params[i];
//	    //std::cout << x[i] << ", ";
//	}
//	//std::cout << std::endl;
//
//    /* define optional scale factors for the parameters */
//    if ( mode == 2 ) {
//		for (int offset = 0; offset<n; offset++) {
//			diag[offset] = 1.0;
//		}
//    }
//
//    /* perform the optimization */ 
//    printf("Starting optimization step...\n");
//    mylmdif_ (&SwarmOptimizer::swarm_sim_opt_error_,
//            &m, &n, x, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn,
//            diag, &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac,
//            ipvt, qtf, wa1, wa2, wa3, wa4);
//    double totalerror = 0;
//    for (int i=0; i<m; i++) {
//       totalerror += fvec[i];
//	}
//    printf("\tnum function calls = %i\n", nfev);
//    printf("\tremaining total error value = %f\n", totalerror);
//    printf("\tor %1.2f per point\n", std::sqrt(totalerror) / m);
//    printf("...ended optimization step.\n");
//
//    /* copy result back to parameters array */
//	std::cout << "final val. :";
//    for (int i=0; i<n; i++) {
//       params[i] = x[i];
//	   std::cout << x[i] << ", ";
//	}
//	std::cout << std::endl;
//
//
//
//    /* release allocated workspace */
//    free (fvec);
//    free (fjac);
//    free (wa4);
//    free (ipvt);
//    free (wa1);
//    free (wa2);
//    free (wa3);
//    free (qtf);
//    free (diag);
//    free (x);
//
//	emit finished();
//	 //return (1);
//	
//}
//
//void SwarmMCMCOptimizer::set_viewer(SwarmViewer* swarm_viewer) {
//	swarm_viewer_ = swarm_viewer;
//}
//
//double SwarmMCMCOptimizer::run_simulation(double separation_constant, double cluster_constant) {
//
//	
//	separation_constant = (separation_constant > 0) ? separation_constant : 0;
//	swarm_viewer_->set_separation_constant(separation_constant);
//
//	cluster_constant = (cluster_constant > 0) ? cluster_constant : 0;
//	swarm_viewer_->set_cluster_constant(cluster_constant);
//
//	double current_time_taken;
//	double current_multi_sampling;
//	double current_coverage;
//	double occlusion;
//
//
//	swarm_viewer_->sim_results_updated_ = false;
//	swarm_viewer_->optimizer_reset_sim();
//	QCoreApplication::processEvents();
//
//	swarm_viewer_->get_sim_results(current_time_taken, current_multi_sampling, current_coverage, occlusion);
//
//	// minimize time taken
//	// maximize multi sampling
//	// maximize coverage
//
//	double score = 0.0;
//	score += ((max_time_taken_ + 100) - current_time_taken) / (max_time_taken_ + 100);
//	score += (current_multi_sampling) / static_cast<double>(swarm_viewer_->no_of_robots_);
//	//score += std::pow(current_coverage - max_coverage_, 2);
//	return score;
//}
//
//double SwarmMCMCOptimizer::run_simulation(double alignment_constant, double cluster_constant, 
//	double square_radius, int no_of_robots, OPTIMIZE_CASE case_no) {
//
//	if (no_of_robots < 4) {
//		swarm_viewer_->set_discovery_range(2);
//	} else {
//		swarm_viewer_->set_discovery_range(1);
//	}
//	
//	swarm_viewer_->set_alignment_constant(alignment_constant);
//	swarm_viewer_->set_cluster_constant(cluster_constant);
//	swarm_viewer_->set_square_formation_radius(square_radius);
//	swarm_viewer_->set_no_of_robots(no_of_robots);
//
//	double current_time_taken;
//	double current_multi_sampling;
//	double current_coverage;
//	double occlusion;
//
//	swarm_viewer_->sim_results_updated_ = false;
//	swarm_viewer_->optimizer_reset_sim();
//	QCoreApplication::processEvents();
//
//	swarm_viewer_->get_sim_results(current_time_taken, current_multi_sampling, current_coverage, occlusion);
//
//	// minimize time taken
//	// maximize multi sampling
//	// maximize coverage
//
//	double score = 0.0;
//	switch (case_no) {
//	case TIME_ONLY: {
//		score += ((max_time_taken_ + 100) - current_time_taken) / (max_time_taken_ + 100);
//		break;
//	}
//	case SIMUL_SAMPLING_ONLY: {
//		score += (current_multi_sampling) / static_cast<double>(swarm_viewer_->no_of_robots_);
//		break;
//	}
//	case TIME_AND_SIMUL_SAMPLING: {
//		score += ((max_time_taken_ + 100) - current_time_taken) / (max_time_taken_ + 100);
//		score += (current_multi_sampling) / static_cast<double>(swarm_viewer_->no_of_robots_);
//		break;
//	}
//	default: break;
//	}
//	//score += std::pow(current_coverage - max_coverage_, 2);
//	return score;
//}
//
//void SwarmMCMCOptimizer::set_max_time_taken(double max_time) {
//	max_time_taken_ = max_time;
//}
//
//void SwarmMCMCOptimizer::optimize_brute_force() {
//
//	// initialize params
//	double current_separation_constant = swarm_viewer_->separation_constant_;
//	double current_cluster_constant = swarm_viewer_->cluster_constant_;
//	double scaling_constant = 1.f;
//	
//
//	std::ofstream file("optimize.csv");
//	//file << "Separation Constant,Cluster Constant,Score\n";
//
//	int iterations_per_constant = 20;
//	double next_separation_constant = 0.0;
//	double next_cluster_constant = 0.0;
//	double max_constant = 10.0;
//
//	file << ",";
//	for (int j = 0; j < iterations_per_constant; ++j) {
//		next_cluster_constant = max_constant * j / (double)iterations_per_constant;
//		file << next_cluster_constant << ",";
//	}
//	file << "\n";
//
//	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//
//	int no_of_iterations = 0;
//	
//	double max_score = 0.0;
//	double max_separation_constant = 0;
//	double max_cluster_constant = 0;
//
//	for (int i = 1; i < iterations_per_constant + 1; ++i) {
//		next_separation_constant = max_constant * i / (double)iterations_per_constant;
//
//		file << next_separation_constant << ",";
//
//		for (int j = 1; j < iterations_per_constant + 1; ++j) {
//			//next_cluster_constant = max_constant * j / (double)iterations_per_constant;
//			next_cluster_constant = 0.0;
//			//if (j < (iterations_per_constant / 2)) {
//			//	next_cluster_constant = next_separation_constant * j / (double)(iterations_per_constant / 2);
//			//} else {
//			//	next_cluster_constant = next_separation_constant * (j - (double)(iterations_per_constant / 2));
//			//}
//
//
//			next_cluster_constant = next_separation_constant * j / (double)(iterations_per_constant);
//
//			double next_score = run_simulation(next_separation_constant, next_cluster_constant);
//
//			if (next_score > max_score) {
//				max_score = next_score;
//				max_cluster_constant = next_cluster_constant;
//				max_separation_constant = next_separation_constant;
//				
//			}
//
//			std::cout << "iteration " << no_of_iterations << " : " 
//				<< next_separation_constant << "," << next_cluster_constant << ","
//				<< next_cluster_constant / next_separation_constant << "," << next_score << "\n";
//
//			file << next_score;
//			if (j != (iterations_per_constant)) {
//				file << ",";
//			}
//			no_of_iterations++;
//		}
//		file << "\n";
//		file.flush();
//	}
//	file.close();
//
//	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//
//	std::cout << "Time taken (s) : " << 
//		std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / (1000 * static_cast<double>(no_of_iterations)) << std::endl;
//
//	std::cout 
//		<< max_separation_constant << "," << max_cluster_constant << ","
//		"," << max_score << "\n";
//}
//
//
//void SwarmMCMCOptimizer::optimize_experimental_brute_force() {
//
//	std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
//	std::string timestamp_str = std::to_string(timestamp.time_since_epoch().count());
//	std::ofstream file("optimize-" + timestamp_str + ".csv");
//	//file << "Separation Constant,Cluster Constant,Score\n";
//
//	int iterations_per_constant = 10;
//	Range alignment_range(0.0, 5.0);
//	Range cluster_range(0.0, 10.0);
//
//	std::vector<double> alignment_vals;
//	std::vector<double> cluster_vals;
//
//	for (int i = 0; i < iterations_per_constant; ++i) {
//		double alignment_value = alignment_range.min_ + i * (alignment_range.max_ - alignment_range.min_) / (double)iterations_per_constant;
//		alignment_vals.push_back(alignment_value);
//		
//		double cluster_value = cluster_range.min_ + i * (cluster_range.max_ - cluster_range.min_) / (double)iterations_per_constant;
//		cluster_vals.push_back(cluster_value);
//	}
//	//alignment_vals.push_back(2.5);
//	//cluster_vals.push_back(1.0);
//	//cluster_vals.push_back(2.0);
//
//	std::vector<double> square_radius_values;
//	square_radius_values.push_back(4.0);
//	square_radius_values.push_back(16.0);
//
//	std::vector<int> number_of_bots;
//	//number_of_bots.push_back(1);
//	//number_of_bots.push_back(10);
//	number_of_bots.push_back(100);
//
//	file << "alignment,cluster,square_radius,noofbots,optimize_case,score" << "\n";
//
//	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//
//	double max_score = 0.0;
//	double max_cluster_constant = 0.0;
//	double max_alignment_constant = 0.0;
//	double max_square_radius_constant = 0.0;
//	int max_no_of_bots = 0;
//	int no_of_iterations = 0;
//
//	for (auto optimize_case = 0u; optimize_case < 3; ++optimize_case) {
//		for (auto& botsno : number_of_bots) {
//			for (auto& radius : square_radius_values) {
//				for (auto& alignment : alignment_vals) {
//					for (auto& cluster : cluster_vals) {
//						double next_score = run_simulation(alignment, cluster, radius, botsno, static_cast<OPTIMIZE_CASE>(optimize_case));
//						if (next_score > max_score) {
//							max_score = next_score;
//							max_cluster_constant = cluster;
//							max_alignment_constant = alignment;
//							max_square_radius_constant = radius;
//							max_no_of_bots = botsno;
//						}
//						no_of_iterations++;
//
//						std::cout << no_of_iterations << " : " << alignment << ", " << cluster << ", " << radius << ", " << botsno << ", " << optimize_case << ", " << next_score << "\n";
//						file << alignment << "," << cluster << "," << radius << "," << botsno << "," << optimize_case << ","  << next_score << "\n";
//						file.flush();
//					}
//				}
//			}
//
//			std::cout << max_alignment_constant << "," << max_cluster_constant << ","
//				"," << max_square_radius_constant << "," << max_no_of_bots << "," << max_score << "\n";
//
//		}
//	}
//	file.close();
//
//
//	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//
//	std::cout << "Time taken (s) : " << 
//		std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / (1000 * static_cast<double>(no_of_iterations)) << std::endl;
//
//}

ParallelMCMCOptimizer::ParallelMCMCOptimizer(const SwarmParams& swarm_params, const OptimizationParams& optimization_params, std::string& optimizer_filename) 
	: swarm_params_(swarm_params), optimization_params_(optimization_params), optimizer_filename_(optimizer_filename), 
	current_working_threads_(0), cull_threshold_(0.2f) {
	VisibilityQuadrant::visbility_quadrant(swarm_params_.sensor_range_ * 2);
}

double ParallelMCMCOptimizer::init_value(double min, double max) {
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> uniform_real_distribution(min, max);

	return uniform_real_distribution(mt);
}

double ParallelMCMCOptimizer::perturb_value(double current_value, double temperature, double min, double max) {

	std::random_device rd;
	std::mt19937 mt(rd());
	std::normal_distribution<> normal_distribution(current_value, temperature);

	double perterbed_val;
	perterbed_val = normal_distribution(mt);

	perterbed_val = std::max(min, std::min(max, perterbed_val));

	return perterbed_val;
}

SimulatorThread* ParallelMCMCOptimizer::init_mcmc_thread(int temperature, int thread_id, int iteration, const MCMCParams& mcmc_params) {

	//seperation_constant = perturb_value(current_params.separation_constant, temperature, 0.0, 10.0);
	//alignment_constant = perturb_value(current_params.alignment_constant, temperature, 0.0, 10.0);
	//cluster_constant = perturb_value(current_params.cluster_constant, temperature, 0.0, 10.0);
	//explore_constant = perturb_value(current_params.explore_constant, temperature, 0.0, 10.0);

	//Params next_params;
	//next_params.separation_constant = swarm_viewer_->separation_constant_;
	//next_params.alignment_constant = swarm_viewer_->alignment_constant_;
	//next_params.cluster_constant = swarm_viewer_->cluster_constant_;
	//next_params.explore_constant = swarm_viewer_->explore_constant_;

	MCMCParams next_params = mcmc_params;

	next_results_map_[temperature][thread_id] = next_params;

	SimulatorThread *simulator_thread = new SimulatorThread(temperature, thread_id, iteration, next_params.swarm_params);

	//SimulatorThread *simulator_thread = new SimulatorThread(bridge_, temperature, thread_id, swarm_viewer_->no_of_robots_,
	//	swarm_viewer_->grid_resolution_, swarm_viewer_->grid_length_,
	//	swarm_viewer_->interior_model_filename_, swarm_viewer_->interior_scale_, swarm_viewer_->interior_offset_, swarm_viewer_->model_rotation_,
	//	swarm_viewer_->max_time_taken_,
	//	next_params.separation_constant, next_params.alignment_constant, next_params.cluster_constant, next_params.explore_constant,
	//	swarm_viewer_->sensor_range_, swarm_viewer_->discovery_range_, swarm_viewer_->separation_distance_,
	//	(Formation)swarm_viewer_->formation_, swarm_viewer_->square_radius_, swarm_viewer_->bounce_function_power_,
	//	swarm_viewer_->bounce_function_multiplier_, iteration
	//	);

	return simulator_thread;
}


SimulatorThread* ParallelMCMCOptimizer::init_mcmc_thread(int temperature, int thread_id, int iteration) {

	//seperation_constant = perturb_value(current_params.separation_constant, temperature, 0.0, 10.0);
	//alignment_constant = perturb_value(current_params.alignment_constant, temperature, 0.0, 10.0);
	//cluster_constant = perturb_value(current_params.cluster_constant, temperature, 0.0, 10.0);
	//explore_constant = perturb_value(current_params.explore_constant, temperature, 0.0, 10.0);

	MCMCParams next_params;
	next_params.swarm_params = swarm_params_;
	//next_params.swarm_params.coverage_needed_ = swarm_params_.coverage_needed_;
	next_params.swarm_params.separation_constant_ = init_value(MIN_SEPARATION_VALUE, MAX_SEPARATION_VALUE);
	next_params.swarm_params.alignment_constant_ = init_value(MIN_ALIGNMENT_VALUE, MAX_ALIGNMENT_VALUE);
	next_params.swarm_params.cluster_constant_ = init_value(MIN_CLUSTER_VALUE, MAX_CLUSTER_VALUE);
	next_params.swarm_params.explore_constant_ = init_value(MIN_EXPLORE_VALUE, MAX_EXPLORE_VALUE);
	//next_params.swarm_params.bounce_function_multiplier_ = init_value(0.0, 100.0);
	//next_params.swarm_params.separation_range_max_ = init_value(0.0, 5.0);


	auto simulator_thread = init_mcmc_thread(temperature, thread_id, iteration, next_params);

	return simulator_thread;
}

SimulatorThread* ParallelMCMCOptimizer::get_next_mcmc(int temperature, int thread_id, int iteration) {

	double next_score = next_results_map_[temperature][thread_id].score;
	double best_score = best_results_map_[temperature][thread_id].score;
	double current_score = current_results_map_[temperature][thread_id].score;


	if (next_score < best_score) {
		best_results_map_[temperature][thread_id] = next_results_map_[temperature][thread_id];
		result_progression_map_[temperature][thread_id].push_back(next_results_map_[temperature][thread_id]);
	}

	double uniform_random_value = init_value(0.0, 1.0);

	if ((next_score <= current_score)
		|| ((next_score / current_score) >= uniform_random_value)) {
		current_results_map_[temperature][thread_id] = next_results_map_[temperature][thread_id];
	}

	double seperation_constant, alignment_constant, cluster_constant, explore_constant;

	MCMCParams current_mcmc_params = current_results_map_[temperature][thread_id];


	// randomly pick a parameter
	int no_of_params_to_perturb;
	std::random_device rd;
	std::mt19937 mt(rd());
	//std::uniform_int_distribution<> uniform_int_distribution(0, 5);
	//std::uniform_int_distribution<> uniform_int_distribution(0, 3);
	std::uniform_int_distribution<> uniform_int_distribution(0, 4);
	
	int param_index = uniform_int_distribution(mt);

	//auto& current_params = current_mcmc_params.swarm_params;
	//seperation_constant = current_params.separation_constant_;
	//alignment_constant = current_params.alignment_constant_;
	//cluster_constant = current_params.cluster_constant_;
	//explore_constant = current_params.explore_constant_;

	auto next_mcmc_params = current_mcmc_params;


	//print_result(current_params, std::cout);

	switch (param_index) {
	case 0: {
		next_mcmc_params.swarm_params.separation_constant_ = perturb_value(next_mcmc_params.swarm_params.separation_constant_, 
			temperatures_[temperature], MIN_SEPARATION_VALUE, MAX_SEPARATION_VALUE);
		break;
	}
	case 1: {
		next_mcmc_params.swarm_params.alignment_constant_ = perturb_value(next_mcmc_params.swarm_params.alignment_constant_, 
			temperatures_[temperature], MIN_ALIGNMENT_VALUE, MAX_ALIGNMENT_VALUE);
		break;
	}
	case 2: {
		next_mcmc_params.swarm_params.cluster_constant_ = perturb_value(next_mcmc_params.swarm_params.cluster_constant_, 
			temperatures_[temperature], MIN_CLUSTER_VALUE, MAX_CLUSTER_VALUE);
		break;
	}
	case 3: {
		next_mcmc_params.swarm_params.explore_constant_ = perturb_value(next_mcmc_params.swarm_params.explore_constant_,
			temperatures_[temperature], MIN_EXPLORE_VALUE, MAX_EXPLORE_VALUE);
		break;
	}
	case 4: {
		next_mcmc_params.swarm_params.bounce_function_multiplier_ = perturb_value(next_mcmc_params.swarm_params.bounce_function_multiplier_,
			temperatures_[temperature], MIN_BOUNCE_MULTIPLIER_VALUE, MAX_BOUNCE_MULTIPLIER_VALUE);
		break;
	}
	//case 5: {
	//	next_mcmc_params.swarm_params.separation_range_max_ = perturb_value(next_mcmc_params.swarm_params.separation_range_max_,
	//		temperatures_[temperature], 0.0, 5.0);
	//	break;
	//}

	}



	//SwarmParams& next_params = next_mcmc_params.swarm_params;
	//next_params.separation_constant_ = seperation_constant;
	//next_params.alignment_constant_ = alignment_constant;
	//next_params.cluster_constant_ = cluster_constant;
	//next_params.explore_constant_ = explore_constant;

	//print_result(next_params, std::cout);
	//std::cout << " current and next\n";

	next_results_map_[temperature][thread_id] = next_mcmc_params;

	//SimulatorThread *simulator_thread = new SimulatorThread(temperature, thread_id, iteration,  swarm_viewer_->no_of_robots_,
	//	swarm_viewer_->grid_resolution_, swarm_viewer_->grid_length_,
	//	swarm_viewer_->interior_model_filename_, swarm_viewer_->interior_scale_, swarm_viewer_->interior_offset_, swarm_viewer_->model_rotation_,
	//	swarm_viewer_->max_time_taken_,
	//	next_params.separation_constant, next_params.alignment_constant, next_params.cluster_constant, next_params.explore_constant,
	//	swarm_viewer_->sensor_range_, swarm_viewer_->discovery_range_, swarm_viewer_->separation_distance_,
	//	(Formation)swarm_viewer_->formation_, swarm_viewer_->square_radius_, swarm_viewer_->bounce_function_power_,
	//	swarm_viewer_->bounce_function_multiplier_, iteration
	//	);
	SimulatorThread *simulator_thread = new SimulatorThread(temperature, thread_id, iteration, next_mcmc_params.swarm_params);


	return simulator_thread;

}

void ParallelMCMCOptimizer::refill_queue_with_single_next_mcmc_thread(int temperature, int thread_id, int iteration) {

	int next_iteration = ++iteration;
	//for (int temperature = 0; temperature < temperatures_.size(); ++temperature) {
	//	// start no_of_threads
	//	for (int thread_id = 0; thread_id < no_of_threads_per_temperature; ++thread_id) {
			auto simulator_thread = get_next_mcmc(temperature, thread_id, next_iteration);
			simulator_threads_work_queue_.push_back(simulator_thread);
	//	}
	//}

}




void ParallelMCMCOptimizer::print_results(std::string swarm_config_filename) {

	std::ofstream file(SwarmUtils::get_optimizer_results_filename(swarm_config_filename, "mcmc_results"));
	SwarmUtils::print_result_header(file);

	std::cout << std::setprecision(17);
	file << std::setprecision(17);

	SwarmUtils::print_result_header(std::cout);

	double best_score = 0.0;
	MCMCParams best_params;
	for (auto& group_entry : best_results_map_) {
		for (auto& thread_entry : group_entry.second) {
			auto params = thread_entry.second;
			if (params.score > best_score) {
				best_score = params.score;
				best_params = params;
			}
			SwarmUtils::print_result(params, std::cout);
			SwarmUtils::print_result(params, file);
			std::cout << "\n";
			file.flush();
		}
	}

	std::cout << "Best params : \n";
	file << "Best params : \n";
	SwarmUtils::print_result(best_params, std::cout);
	SwarmUtils::print_result(best_params, file);
	auto secs_count = std::chrono::duration_cast<std::chrono::seconds>(end_time_ - begin_time_).count();
	std::cout << "Time taken : " << secs_count << "s => " << secs_count / 60 << "m " << secs_count % 60 << "s\n";
	file << "Time taken : " << secs_count << "s => " << secs_count / 60 << "m " << secs_count % 60 << "s\n";
	
}

MCMCParams ParallelMCMCOptimizer::get_best_results() {
	double best_score = DBL_MAX;
	MCMCParams best_params;
	for (auto& group_entry : best_results_map_) {
		for (auto& thread_entry : group_entry.second) {
			auto params = thread_entry.second;
			if (params.score < best_score) {
				best_score = params.score;
				best_params = params;
			}
		}
	}
	return best_params;
}

void ParallelMCMCOptimizer::write_out_best_results() {
	std::ofstream file(optimizer_filename_, std::ios_base::app);
	auto best_mcmc_results = get_best_results();
	SwarmUtils::print_result(best_mcmc_results, file);

	auto swarm_config_optimized_filename = SwarmUtils::get_swarm_config_results_filename(best_mcmc_results.swarm_params.config_name_.toStdString(), "optimized");
	//std::string full_optimized_pathname = std::string("swarm-config/" + swarm_config_optimized_filename);
	std::string full_optimized_pathname = std::string(swarm_config_optimized_filename);
	QString filename_qstring(full_optimized_pathname.c_str());
	SwarmUtils::save_swarm_params(best_mcmc_results.swarm_params, filename_qstring);
}

void ParallelMCMCOptimizer::print_progression_results(std::string swarm_config_filename) {

	std::ofstream file(SwarmUtils::get_optimizer_results_filename(swarm_config_filename, "mcmc_progression_results"));

	SwarmUtils::print_result_header(file);
	//print_result_header(std::cout);

	for (auto& group_entry : result_progression_map_) {
		for (auto& thread_entry : group_entry.second) {
			auto params_vector = thread_entry.second;
			for (auto& params : params_vector) {
				//print_result(params, std::cout);
				SwarmUtils::print_result(params, file);
			}
			std::cout << "\n";
			file.flush();
		}
	}
}

void ParallelMCMCOptimizer::print_progression_results_2(std::string swarm_config_filename) {
	std::ofstream file(SwarmUtils::get_optimizer_results_filename(swarm_config_filename, "mcmc_best_results"));


	int max_param_size = 0;
	for (auto& group_entry : result_progression_map_) {
		for (auto& thread_entry : group_entry.second) {
			max_param_size = std::max((int)thread_entry.second.size(), max_param_size);
			std::stringstream ss;
			ss << group_entry.first << "-" << thread_entry.first << ",";
			file << ss.str();
		}
	}
	file << "\n";


	for (int i = 0; i < max_param_size; ++i) {
		for (auto& group_entry : result_progression_map_) {
			for (auto& thread_entry : group_entry.second) {
				auto params_vector = thread_entry.second;
				auto progression_value = (params_vector.size() - 1) < i ? "" : std::to_string(params_vector[i].score);
				file << progression_value << ",";
			}
		}
		file << "\n";
		file.flush();
	}
}

void ParallelMCMCOptimizer::refill_queue(int temperature, int thread_id, int iteration) {
	if (iteration >= optimization_params_.no_of_iterations) {
		// print best results
		std::cout << "Print best results\n";
	} else if (iteration % optimization_params_.culling_nth_iteration == 0 && current_working_threads_ == 0) {
		cull_and_refill_queue(iteration);
	} else if (iteration % optimization_params_.culling_nth_iteration != 0){
		refill_queue_with_single_next_mcmc_thread(temperature, thread_id, iteration);
	}
}


//void ParallelMCMCOptimizer::set_viewer(SwarmViewer* swarm_viewer) {
//	swarm_viewer_ = swarm_viewer;
//}

ParallelMCMCOptimizer::~ParallelMCMCOptimizer() {
}

void ParallelMCMCOptimizer::start_thread() {
		auto sim_thread = simulator_threads_work_queue_.front();
		//connect(sim_thread,
		//	SIGNAL(send_sim_results(int, int, int, double, double, double, double, double, double, double, double, double)),
		//	this,
		//	SLOT(restart_work(int, int, int, double, double, double, double, double, double, double, double, double)));

		connect(sim_thread,
			SIGNAL(send_sim_results(int, int, int, SwarmParams, OptimizationResults)),
			this,
			SLOT(restart_work(int, int, int, SwarmParams, OptimizationResults)));


		current_working_threads_++;
		sim_thread->reset_sim();
		thread_pool_.start(sim_thread);

		simulator_threads_work_queue_.pop_front();
}

void ParallelMCMCOptimizer::run_optimizer() {
	// decide on temperatures
	// decide on initialization points
	// start simulation with ideal number of threads
	// as thread finishes work, start new thread with perturb, store points in a priority queue
	// if thread performance is bad, start with new init point

	//temperatures_.push_back(0.25f);
	//temperatures_.push_back(0.5f);
	//temperatures_.push_back(1.f);
	//temperatures_.push_back(2.f);
	//temperatures_.push_back(3.f);

	temperatures_.push_back(0.1f);
	temperatures_.push_back(0.2f);
	temperatures_.push_back(0.3f);
	temperatures_.push_back(0.4f);
	temperatures_.push_back(0.5f);


	//no_of_threads_per_temperature = 10;

	//total_iterations_ = 30;
	//culling_iterations_ = 10;
	//cull_threshold_ = 0.2;


	// init threads, data structs
	for (int temperature = 0; temperature < temperatures_.size(); ++temperature) {
		for (int thread_id = 0; thread_id < optimization_params_.no_of_threads; ++thread_id) {
			auto simulator_thread = init_mcmc_thread(temperature, thread_id, 1);
			simulator_threads_work_queue_.push_back(simulator_thread);

			MCMCParams params;
			params.score = DBL_MAX;
			params.group_id = temperature;
			params.thread_id = thread_id;

			current_results_map_[temperature][thread_id] = params;
			best_results_map_[temperature][thread_id] = params;
			result_progression_map_[temperature][thread_id].push_back(best_results_map_[temperature][thread_id]);
		}
	}

	//bridge_ = new BridgeObject();
	//connect(bridge_,
	//	SIGNAL(send_sim_results(int, int, int, double, double, double, double, double, double, double, double, double)),
	//	this,
	//	SLOT(restart_work(int, int, int, double, double, double, double, double, double, double, double, double)));
	//connect(bridge_,
	//	&BridgeObject::send_sim_results,
	//	this,
	//	&ParallelMCMCOptimizer::restart_work);

	begin_time_ = std::chrono::steady_clock::now();

	std::cout << "ideal thread count : " << QThread::idealThreadCount() << "\n";

	auto sim_itr = simulator_threads_work_queue_.begin();

	current_working_threads_ = 0;

	for (int i = 0; i < 2 * QThread::idealThreadCount() && simulator_threads_work_queue_.size() > 0; ++i) {
		start_thread();
	}

	//QEventLoop loop;
	//loop.exec();
	// if iteration == culling iterations, move to the next one

}

void ParallelMCMCOptimizer::cull_and_refill_queue(int iteration) {
	// keep only culling %
	std::vector<MCMCParams> sort_vector;

	for (auto& entry_group_id : best_results_map_) {
		for (auto& entry_thread_id : entry_group_id.second) {
			sort_vector.push_back(entry_thread_id.second);
		}
	}
	std::sort(sort_vector.begin(), sort_vector.end(), [](const MCMCParams& a, const MCMCParams &b)
	{
		return a.score < b.score;
	});

	int no_of_entries_to_keep = sort_vector.size() * cull_threshold_;
	sort_vector.erase(sort_vector.begin() + no_of_entries_to_keep, sort_vector.end());

	auto best_params = sort_vector[0];

	int next_iteration = ++iteration;
	for (int temperature = 0; temperature < temperatures_.size(); ++temperature) {
		// start no_of_threads
		for (int thread_id = 0; thread_id < optimization_params_.no_of_threads; ++thread_id) {

			bool is_high_score = false;
			for (auto& high_score_value : sort_vector) {
				if (high_score_value.group_id == temperature
					&& high_score_value.thread_id == thread_id) {
					is_high_score = true;
					break;
				}
			}


			SimulatorThread* simulator_thread;
			if (is_high_score) {
				// let it continue on MCMC
				simulator_thread = get_next_mcmc(temperature, thread_id, next_iteration);
			} else {
				// reinitialize values
				next_results_map_[temperature][thread_id] = best_params;
				simulator_thread = get_next_mcmc(temperature, thread_id, next_iteration);
				//simulator_thread = init_mcmc_thread(temperature, thread_id, next_iteration);
			}

			simulator_threads_work_queue_.push_back(simulator_thread);
		}
	}
}

void ParallelMCMCOptimizer::print_best_results_progression(const std::string& swarm_config_filename) {
	std::ofstream file(SwarmUtils::get_optimizer_results_filename(swarm_config_filename, "progressive_best_results"));
	SwarmUtils::print_result_header(file);

	std::cout << std::setprecision(17);
	file << std::setprecision(17);


	for (auto& best_results_per_iteration : best_results_per_iteration_map_) {
		auto params = best_results_per_iteration.second;
		SwarmUtils::print_result(params, std::cout);
		SwarmUtils::print_result(params, file);
		file.flush();
	}
}

void ParallelMCMCOptimizer::restart_work(int group_id, int thread_id, int iteration, SwarmParams params, OptimizationResults results) {


	work_queue_lock_.lock();

	current_working_threads_--;
	//MCMCParams next_params = create_params(temperature, thread_id, iteration,
	//	 seperation_constant,  alignment_constant,  cluster_constant,  explore_constant,
	//	 seperation_distance,  simultaneous_sampling,  time_taken,  occlusion,  coverage);

	MCMCParams next_params;
	next_params.group_id = group_id;
	next_params.thread_id = thread_id;
	next_params.iteration = iteration;
	next_params.swarm_params = params;
	next_params.results = results;
	next_params.coeffs = optimization_params_.coefficients;


	OptimizationResults scores;

	next_params.score = SwarmUtils::calculate_score(params, results, optimization_params_.coefficients, TIME_AND_SIMUL_SAMPLING_AND_MULTI_SAMPLING_COVERAGE, scores);
	next_params.scores = scores;
	//next_params.score = calculate_score(next_params, MULTI_SAMPLING_ONLY);

	//std::cout << "score : " << next_params.score << "\n";
	next_results_map_[group_id][thread_id] = next_params;

	auto best_iteration_result = best_results_per_iteration_map_.find(iteration);
	if (best_iteration_result == best_results_per_iteration_map_.end()) {
		if (iteration > 1) {
			MCMCParams previous_params = best_results_per_iteration_map_[iteration - 1];
			double score = previous_params.score;
			if (score < next_params.score) {
				best_results_per_iteration_map_[iteration] = previous_params;
			} else {
				best_results_per_iteration_map_[iteration] = next_params;
			}
		} else {
			best_results_per_iteration_map_[iteration] = next_params;
		}
	} else {
		if (next_params.score < best_iteration_result->second.score) {
			best_iteration_result->second = next_params;
		}
	}

	//result_progression_map_[temperature][thread_id].push_back(next_results_map_[temperature][thread_id]);


	if (iteration >= optimization_params_.no_of_iterations) {
		if (simulator_threads_work_queue_.size() > 0) {
			start_thread();
		}
	} else if (iteration % optimization_params_.culling_nth_iteration == 0) {
		if (current_working_threads_ == 0) {
			cull_and_refill_queue(iteration);
			for (int i = 0; i < 2 * QThread::idealThreadCount() && simulator_threads_work_queue_.size() > 0; ++i) {
				start_thread();
			}
		}
		else {
			if (simulator_threads_work_queue_.size() > 0) {
				start_thread();
			}
		}
	} else if (iteration % optimization_params_.culling_nth_iteration != 0){
		refill_queue_with_single_next_mcmc_thread(group_id, thread_id, iteration);
		start_thread();
	}



	//if (simulator_threads_work_queue_.size() == 0) {
	//	refill_queue(temperature, thread_id, iteration);
	//}

	//if (simulator_threads_work_queue_.size() > 0) {
	//	if (current_working_threads_ == 0) {
	//		for (int i = 0; i < 2 * QThread::idealThreadCount() && simulator_threads_work_queue_.size() > 0; ++i) {
	//			start_thread();
	//		}
	//	} else {
	//		start_thread();
	//	//std::cout << "No. of active threads : " << thread_pool_.activeThreadCount() << "\n";
	//	}
	//} 	
	//if (current_working_threads_ == 0 && simulator_threads_work_queue_.size() == 0) {
	//	print_results();
	//	print_progression_results();
	//	std::cout << "Work done!\n No. of active threads : " << thread_pool_.activeThreadCount() << "\n";
	//}

	if (current_working_threads_ == 0 && simulator_threads_work_queue_.size() == 0) {
		end_time_ = std::chrono::steady_clock::now();
		//print_results(swarm_params_.config_name_.toStdString());
		//print_progression_results(swarm_params_.config_name_.toStdString());
		print_progression_results_2(swarm_params_.config_name_.toStdString());
		print_best_results_progression(swarm_params_.config_name_.toStdString());
		write_out_best_results();
		//std::cout << "Work done!\n No. of active threads : " << thread_pool_.activeThreadCount() << "\n";
		thread_pool_.waitForDone();
		emit finished();
	}	

	//std::cout << "No. of active threads : " << thread_pool_.activeThreadCount() << "\n";
	//std::cout << "Current working threads : " << current_working_threads_ << "\n";
	//std::cout << "Queue size : " << simulator_threads_work_queue_.size() << "\n";
	work_queue_lock_.unlock();



}
//
//Params ParallelMCMCOptimizer::create_params(int group_id, int thread_id, int iteration, 
//	double seperation_constant, double alignment_constant, double cluster_constant, double explore_constant, 
//	double seperation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage) {
//
//	Params params;
//	params.group_id = group_id;
//	params.thread_id = thread_id;
//	params.separation_constant = seperation_constant;
//	params.alignment_constant = alignment_constant;
//	params.cluster_constant = cluster_constant;
//	params.explore_constant = explore_constant;
//	params.seperation_distance = seperation_distance;
//	params.time_taken = time_taken;
//	params.simultaneous_sampling = simultaneous_sampling;
//	params.occlusion = occlusion;
//	params.coverage = coverage;
//
//	return params;
//}

//void ParallelMCMCOptimizer::restart_work(int group_id, int thread_id, int iteration, 
//	double seperation_constant, double alignment_constant, double cluster_constant, double explore_constant, 
//	double seperation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage) {
//
//
//	work_queue_lock_.lock();
//
//	Params next_params = create_params( group_id,  thread_id,  iteration,
//		 seperation_constant,  alignment_constant,  cluster_constant,  explore_constant,
//		 seperation_distance,  simultaneous_sampling,  time_taken,  occlusion,  coverage);
//	next_params.score = calculate_score(next_params, TIME_ONLY);
//	next_results_map_[group_id][thread_id] = next_params;
//
//	if (simulator_threads_work_queue_.size() == 0) {
//		refill_queue(iteration);
//	}
//
//	if (simulator_threads_work_queue_.size() > 0) {
//		auto sim_thread = simulator_threads_work_queue_.front();
//		//connect(bridge_,
//		//	SIGNAL(send_sim_results(int, int, int, double, double, double, double, double, double, double, double, double)),
//		//	this,
//		//	SLOT(restart_work(int, int, int, double, double, double, double, double, double, double, double, double)));
//		sim_thread->reset_sim();
//		thread_pool_.start(sim_thread);
//		simulator_threads_work_queue_.pop_front();
//	} else {
//		std::cout << "Work done!";
//		emit finished();
//	}
//	
//	work_queue_lock_.unlock();
//
//}

//void ParallelMCMCOptimizer::restart_work(int group_id, int thread_id, int iteration) {
//void ParallelMCMCOptimizer::restart_work(int temperature, int thread_id, int iteration, 
//	double seperation_constant, double alignment_constant, double cluster_constant, double explore_constant, 
//	double seperation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage) {
//
//
//
//	work_queue_lock_.lock();
//
//	current_working_threads_--;
//	//Params next_params = create_params(group_id, thread_id, iteration, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//	Params next_params = create_params(temperature, thread_id, iteration,
//		 seperation_constant,  alignment_constant,  cluster_constant,  explore_constant,
//		 seperation_distance,  simultaneous_sampling,  time_taken,  occlusion,  coverage);
//
//	next_params.score = calculate_score(next_params, TIME_ONLY);
//	//next_params.score = calculate_score(next_params, MULTI_SAMPLING_ONLY);
//
//	//std::cout << "score : " << next_params.score << "\n";
//	next_results_map_[temperature][thread_id] = next_params;
//
//	//result_progression_map_[temperature][thread_id].push_back(next_results_map_[temperature][thread_id]);
//
//
//	if (iteration >= total_iterations_) {
//		if (simulator_threads_work_queue_.size() > 0) {
//			start_thread();
//		}
//	} else if (iteration % culling_iterations_ == 0) {
//		if (current_working_threads_ == 0) {
//			cull_and_refill_queue(iteration);
//			for (int i = 0; i < 2 * QThread::idealThreadCount() && simulator_threads_work_queue_.size() > 0; ++i) {
//				start_thread();
//			}
//		}
//		else {
//			if (simulator_threads_work_queue_.size() > 0) {
//				start_thread();
//			}
//		}
//	} else if (iteration % culling_iterations_ != 0){
//		refill_queue_with_single_next_mcmc_thread(temperature, thread_id, iteration);
//		start_thread();
//	}
//
//
//
//	//if (simulator_threads_work_queue_.size() == 0) {
//	//	refill_queue(temperature, thread_id, iteration);
//	//}
//
//	//if (simulator_threads_work_queue_.size() > 0) {
//	//	if (current_working_threads_ == 0) {
//	//		for (int i = 0; i < 2 * QThread::idealThreadCount() && simulator_threads_work_queue_.size() > 0; ++i) {
//	//			start_thread();
//	//		}
//	//	} else {
//	//		start_thread();
//	//	//std::cout << "No. of active threads : " << thread_pool_.activeThreadCount() << "\n";
//	//	}
//	//} 	
//	//if (current_working_threads_ == 0 && simulator_threads_work_queue_.size() == 0) {
//	//	print_results();
//	//	print_progression_results();
//	//	std::cout << "Work done!\n No. of active threads : " << thread_pool_.activeThreadCount() << "\n";
//	//}
//
//	if (current_working_threads_ == 0 && simulator_threads_work_queue_.size() == 0) {
//		end_time_ = std::chrono::steady_clock::now();
//		print_results();
//		print_progression_results();
//		print_progression_results_2();
//		std::cout << "Work done!\n No. of active threads : " << thread_pool_.activeThreadCount() << "\n";
//		thread_pool_.waitForDone();
//		emit finished();
//	}	
//
//	std::cout << "No. of active threads : " << thread_pool_.activeThreadCount() << "\n";
//	std::cout << "Current working threads : " << current_working_threads_ << "\n";
//	std::cout << "Queue size : " << simulator_threads_work_queue_.size() << "\n";
//	work_queue_lock_.unlock();
//
//
//
//
//
//}

//void SwarmMCMCOptimizer::optimize_swarm_params() {
//
//	//Algorithm.MCMC(params)
//	//	1. initialize params(just randomly select values)
//	//	2. current_score = Score(params)
//	//	3. best_score = current_score
//	//	4. Repeat for a lot of iterations
//	//	5.     obtain next_params by randomly changing params
//	//	(One approach is to select one param and add a small delta to change its value.)
//	//	6.     next_score = Score(next_params)
//	//	7.     If next_score > best_score
//	//	8.         best_score = next_score
//	//	9.         best_params = params
//	//	10.   If next_score >= score or next_score / score >= uniform_rand(0, 1)
//	//	11.       params = next_params
//	//	12.       score = next_score
//	//	13. Return best_params
//
//
//	// initialize params
//	double current_separation_constant = swarm_viewer_->separation_constant_;
//	double current_cluster_constant = swarm_viewer_->cluster_constant_;
//	double scaling_constant = 1.f;
//	
//	// current score
//	double current_score = run_simulation(current_separation_constant, current_cluster_constant);
//
//	double best_score = current_score;
//	double best_separation_constant = current_separation_constant;
//	double best_cluster_constant = current_cluster_constant;
//
//	int no_of_iterations = 10;
//
//	std::random_device rd;
//	std::mt19937 mt(rd());
//	std::normal_distribution<> normal_distribution(0.0, 1.0);
//	std::uniform_real_distribution<> uniform_real_distribution(0, 1);
//	
//
//	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//
//
//	for (int i = 0; i < no_of_iterations; ++i) {
//		double next_separation_constant = -1.0;
//		while (next_separation_constant < 0 || next_separation_constant > 20) {
//			double separation_delta = normal_distribution(mt);
//			next_separation_constant = (current_separation_constant + scaling_constant * separation_delta);
//		}
//		//std::cout << "next separation constant : " << next_separation_constant << "\n";
//
//		double next_cluster_constant = -1.0;
//		while (next_cluster_constant < 0 || next_cluster_constant > 20) {
//			double cluster_delta = normal_distribution(mt);
//			next_cluster_constant = (current_cluster_constant + scaling_constant * cluster_delta);
//		}
//		//std::cout << "next cluster constant : " << next_cluster_constant << "\n";
//
//		double next_score = run_simulation(next_separation_constant, next_cluster_constant);
//
//		if (next_score > best_score) {
//			best_score = next_score;
//			best_separation_constant = next_separation_constant;
//			best_cluster_constant = next_cluster_constant;
//			std::cout << "Best time step : " << best_score << " best separation constant : " << best_separation_constant << " best cluster constant : " << best_cluster_constant << std::endl;
//		}
//
//		double uniform_random_value = uniform_real_distribution(mt);
//
//		if ((next_score >= current_score)
//			|| ((next_score / current_score) >= uniform_random_value)) {
//
//			current_score = next_score;
//			current_separation_constant = next_separation_constant;
//			current_cluster_constant = next_cluster_constant;
//			std::cout << "Iteration : " << i << " Accepted time step : " << current_score << 
//				" accepted separation constant : " << current_separation_constant << std::endl;
//		}
//	}
//
//	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//
//	std::cout << "Time taken (s) : " << 
//		std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / (1000 * static_cast<double>(no_of_iterations)) << std::endl;
//
//
//}
