#include <fsl_common.h>
#include "swarmviewer.h"

class SwarmOptimizer : public QObject {
	Q_OBJECT
	
	static SwarmViewer* swarm_viewer_g;
	static double separation_constant_;
	static int swarm_sim_opt_error_(int *m_ptr, int *n_ptr, double *params, double *error, int *);

	

public:
void set_optimize_swarm_params(SwarmViewer* swarm_viewer,
	double& seperation_constant, double& alignment_constant, double& cluster_constant, double& perimeter_constant, double& explore_constant,
	Range& separation_range, Range& alignment_range, Range& cluster_range,
	//Range& explore_range,  Range& perimeter_range,
	//Range& obstacle_avoidance_near_range, Range& obstacle_avoidance_walking_range,
	int& preferred_neighborhood_count);

signals:
void finished();

public slots:
void optimize_swarm_params();
		
};


class SwarmMCMCOptimizer : public QObject {
	Q_OBJECT
	
	SwarmViewer* swarm_viewer_;
	double separation_constant_;
	double max_time_taken_;
public:
	void set_viewer(SwarmViewer* swarm_viewer);
	double run_simulation(double separation_constant, double cluster_constant);
	void set_max_time_taken(double max_time);

signals:
	void finished();

public slots:
void optimize_swarm_params();
void optimize_brute_force();
		
};


