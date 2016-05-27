#include "planefit.h"
#include <math.h>
#include <malloc.h>


extern "C" int mylmdif_(int (*fcn)(int *, int *, double *, double *, int *), int *m, int *n, double *x, double *fvec, double *ftol, double *xtol, double *gtol, int *maxfev, 
	double *epsfcn, double *diag, int *mode, double *factor, int *nprint, int *info, int *nfev, double *fjac, int *ldfjac, int *ipvt, 
	double *qtf, double *wa1, double *wa2, double *wa3, double *wa4);
//
//static std::vector <glm::vec3> _worldPts;
//static std::vector <glm::vec2> _imgPts;
//
///*****************************************************************************
//*****************************************************************************/

std::vector<cv::Point3f>* line_points_g;

std::vector<Ray>* rays_g;

static int
line_fit_error_(int *m_ptr, int *n_ptr, double *params, double *error, int *)
{
	int nparms = *n_ptr;
	int nerrors = *m_ptr;


	// 0 - ax, 1 - ay, 2 - az
	// 3 - bx, 4 - by, 5 - bz

	cv::Vec3f a(params[0], params[1], params[2]);
	cv::Vec3f b(params[3], params[4], params[5]);

	cv::Vec3f v = b - a;
	double v_length = cv::norm(v);

	for (int i = 0; i < nerrors; ++i) {
		cv::Vec3f x = (*line_points_g)[i];
		cv::Vec3f u = x - a;

		double d = cv::norm(v.cross(u)) / v_length;

		error[i] = d;
	}

	return 1;
}


static int
plane_fit_error_(int *m_ptr, int *n_ptr, double *params, double *error, int *)
{
	int nparms = *n_ptr;
	int nerrors = *m_ptr;


	// 0 - nx, 1 - ny, 2 - nz, 3 - d
	 // n.x = d

	cv::Vec3d n = cv::Vec3d(params[0], params[1], params[2]);
	double n_length = cv::norm(n);
	double d = params[3];


	for (int i = 0; i < nerrors; ++i) {
		Ray ray = (*rays_g)[i];

		double distance_1  = (n.dot(ray.a) - d) / n_length ;
		double distance_2  = (n.dot(ray.b) - d) / n_length ;

		double distance = pow(distance_1, 2);
		distance += pow(distance_2, 2);

		error[i] = sqrt(distance);
	}

	return 1;
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


int
fit_line(std::vector<cv::Point3f>& line_stripe_points, Ray& ray) {
    /* Parameters needed by MINPACK's lmdif() */
	int     n = 6;
	int     m = line_stripe_points.size();
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

	 const int num = 6;
	 double params[num];

	// 0 - ax, 1 - ay, 2 - az
	// 3 - bx, 4 - by, 5 - bz
	 cv::Vec3f a = line_stripe_points[0];
	 cv::Vec3f b = line_stripe_points[line_stripe_points.size() - 1];

	 params[0] = a[0];
	 params[1] = a[1];
	 params[2] = a[2];

	 params[3] = b[0];
	 params[4] = b[1];
	 params[5] = b[2];

	 /* copy to globals */
	 line_points_g = &line_stripe_points;

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
		int offset = 0;
		for (int offset = 0; offset<n; offset++) {
			diag[offset] = 1.0;
		}
    }

    /* perform the optimization */ 
    //printf("Starting optimization step...\n");
    mylmdif_ (line_fit_error_,
            &m, &n, x, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn,
            diag, &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac,
            ipvt, qtf, wa1, wa2, wa3, wa4);
    double totalerror = 0;
    for (int i=0; i<m; i++) {
       totalerror += fvec[i];
	}
    //printf("\tnum function calls = %i\n", nfev);
    //printf("\tremaining total error value = %f\n", totalerror);
    //printf("\tor %1.2f per point\n", std::sqrt(totalerror) / m);
    //printf("...ended optimization step.\n");

    /* copy result back to parameters array */
	//std::cout << "final val. :";
    for (int i=0; i<n; i++) {
       params[i] = x[i];
	    //std::cout << x[i] << ", ";
	}
	//std::cout << std::endl;

	//double mean = params[0];
	//double std_dev = params[1];
	//double k = params[2];
	//double c = params[3];

	// returning mid point of gaussian, which is the highest x value
	ray.a = cv::Vec3d(params[0], params[1], params[2]);
	ray.b = cv::Vec3d(params[3], params[4], params[5]);



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

	 return (1);
	
}


int
fit_plane(std::vector<Ray>& rays, Plane& plane) {
    /* Parameters needed by MINPACK's lmdif() */
	int     n = 4;
	int     m = rays.size();
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

	 const int num = 4;
	 double params[num];

	// 0 - nx, 1 - ny, 2 - nz, 3 - d
	 // n.x = d
	 cv::Vec3d a = rays[0].a;
	 cv::Vec3d b = rays[0].b;
	 cv::Vec3d c = rays[rays.size() - 1].a;

	 cv::Vec3d ab = b - a;
	 cv::Vec3d bc = c - b;

	 cv::Vec3d normal = cv::normalize(ab.cross(bc));
	 double d = normal.dot(a);

	 params[0] = normal[0];
	 params[1] = normal[1];
	 params[2] = normal[2];
	 params[3] = d;

	 /* copy to globals */
	 rays_g = &rays;

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
		int offset = 0;
		for (int offset = 0; offset<n; offset++) {
			diag[offset] = 1.0;
		}
    }

    /* perform the optimization */ 
    //printf("Starting optimization step...\n");
    mylmdif_ (plane_fit_error_,
            &m, &n, x, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn,
            diag, &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac,
            ipvt, qtf, wa1, wa2, wa3, wa4);
    double totalerror = 0;
    for (int i=0; i<m; i++) {
       totalerror += fvec[i];
	}
    //printf("\tnum function calls = %i\n", nfev);
    //printf("\tremaining total error value = %f\n", totalerror);
    //printf("\tor %1.2f per point\n", std::sqrt(totalerror) / m);
    //printf("...ended optimization step.\n");

    /* copy result back to parameters array */
	//std::cout << "final val. :";
    for (int i=0; i<n; i++) {
       params[i] = x[i];
	    //std::cout << x[i] << ", ";
	}
	//std::cout << std::endl;


	// returning mid point of gaussian, which is the highest x value
	plane.n = cv::Vec3d(params[0], params[1], params[2]);
	plane.d = params[3];



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

	return (1);
	
}
