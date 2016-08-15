#include <fsl_common.h>
#include "swarmviewer.h"
#include <qthreadpool.h>
#include "simulatorthread.h"
#include <chrono>

class SwarmOptimizer : public QObject {
	Q_OBJECT
	
	static SwarmViewer* swarm_viewer_g;
	static double separation_constant_;
	//static int swarm_sim_opt_error_(int *m_ptr, int *n_ptr, double *params, double *error, int *);

	

public:
//void set_optimize_swarm_params(SwarmViewer* swarm_viewer,
//	double& seperation_constant, double& alignment_constant, double& cluster_constant, double& perimeter_constant, double& explore_constant,
//	Range& separation_range, Range& alignment_range, Range& cluster_range,
//	//Range& explore_range,  Range& perimeter_range,
//	//Range& obstacle_avoidance_near_range, Range& obstacle_avoidance_walking_range,
//	int& preferred_neighborhood_count);

signals:
void finished();

public slots:
//void optimize_swarm_params();
		
};


class SwarmMCMCOptimizer : public QObject {
	Q_OBJECT
	
	SwarmViewer* swarm_viewer_;
	double separation_constant_;
	double max_time_taken_;
public:
	//void set_viewer(SwarmViewer* swarm_viewer);
	//double run_simulation(double separation_constant, double cluster_constant);
	//void set_max_time_taken(double max_time);
	//enum OPTIMIZE_CASE {
	//	TIME_ONLY = 0,
	//	SIMUL_SAMPLING_ONLY = 1,
	//	TIME_AND_SIMUL_SAMPLING = 2
	//};
	//double run_simulation(double alignment_constant, double cluster_constant, double square_radius, int no_of_robots, OPTIMIZE_CASE optimize_case);

signals:
	void finished();

public slots:
//void optimize_swarm_params();
//void optimize_brute_force();
//void optimize_experimental_brute_force();
};


class ParallelMCMCOptimizer : public QObject {
	Q_OBJECT

	enum OPTIMIZE_CASE {
		TIME_ONLY = 0,
		MULTI_SAMPLING_ONLY = 1,
		TIME_AND_SIMUL_SAMPLING = 2,
		TIME_AND_SIMUL_SAMPLING_AND_COVERAGE = 3
	};

	//SwarmViewer* swarm_viewer_;
	QMutex work_queue_lock_;
	QThreadPool thread_pool_;
	std::deque<SimulatorThread*> simulator_threads_work_queue_;

	//int total_iterations_;
	//int culling_iterations_;
	//int no_of_threads_per_temperature;

	std::unordered_map<int, std::unordered_map<int, MCMCParams>> best_results_map_;
	std::unordered_map<int, std::unordered_map<int, MCMCParams>> current_results_map_;
	std::unordered_map<int, std::unordered_map<int, MCMCParams>> next_results_map_;
	std::unordered_map<int, std::unordered_map<int, std::vector<MCMCParams>>> result_progression_map_;
	std::unordered_map<int, MCMCParams> best_results_per_iteration_map_;

	float cull_threshold_;
	std::vector<float> temperatures_;
	//BridgeObject* bridge_;
	int current_working_threads_;
	std::chrono::high_resolution_clock::time_point begin_time_;
	std::chrono::high_resolution_clock::time_point end_time_;
	SwarmParams swarm_params_;
	OptimizationParams optimization_params_;
	std::string optimizer_filename_;
public:
	ParallelMCMCOptimizer(const SwarmParams& swarm_params, const OptimizationParams& optimization_params, std::string& optimizer_filename);

	double init_value(double min, double max);
	double perturb_value(double current_value, double temperature, double min, double max);
	SimulatorThread* init_mcmc_thread(int temperature, int thread_id, int iteration, const MCMCParams& next_params);
	SimulatorThread* init_mcmc_thread(int temperature, int thread_id, int iteration);
	SimulatorThread* get_next_mcmc(int temperature, int thread_id, int iteration);
	void refill_queue_with_single_next_mcmc_thread(int temperature, int thread_id, int iteration);
	void print_results(std::string swarm_config_filename);
	MCMCParams get_best_results();
	void write_out_best_results();
	void print_progression_results(std::string swarm_config_filename);
	void print_progression_results_2(std::string swarm_config_filename);
	void refill_queue(int temperature, int thread_id, int iteration);
	double calculate_score(OptimizationResults results, int case_no, OptimizationResults& scores);

	//void set_viewer(SwarmViewer* swarm_viewer);
	virtual ~ParallelMCMCOptimizer();
	void start_thread();

public slots:
	void run_optimizer();
	void cull_and_refill_queue(int iteration);
	//Params create_params(int group_id, int thread_id, int iteration, double seperation_constant, double alignment_constant, double cluster_constant, double explore_constant, double seperation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage);
	//void restart_work(int group_id, int thread_id, int iteration,
	//	double seperation_constant, double alignment_constant, double cluster_constant, double explore_constant,
	//	double seperation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage);
	//void restart_work(int group_id, int thread_id, int iteration);

	void print_best_results_progression(const std::string& swarm_config_filename);
	void restart_work(int group_id, int thread_id, int iteration, SwarmParams params, OptimizationResults results);


signals:
	void finished();

};