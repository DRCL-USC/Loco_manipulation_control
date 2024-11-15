#ifndef _mpc_solver
#define _mpc_solver

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <sys/time.h>
#include "common_types.h"
#include <qpOASES.hpp>
#include <common/Utilities/Timer.h>
#include <common/cppTypes.h>
#include "RobotState.h"

using Eigen::Dynamic;
using Eigen::Quaterniond;
using Eigen::Quaternionf;
using std::cout;
using std::endl;
#define BIG_NUMBER 5e10
#define K_MAX_GAIT_SEGMENTS 36

struct problem_setup
{
  float dt;
  float mu;
  float f_max;
  int horizon;
};

struct update_data_t
{
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[12];
  float linear_accel[3];
  float angular_accel[3];
  float yaw;
  float weights[12];
  float traj[12 * K_MAX_GAIT_SEGMENTS];
  float alpha;
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
  int use_jcqp;
};

class MPCSolver
{
public:
  MPCSolver(double dt, int horizon, double mu, double f_max, Quadruped *quad);
  void setup_problem(double dt, int horizon, double mu, double f_max);
  void setup_adaptive(int horizon, int _state_num, float *_mu_2);
  void setup_LocoManipulation(int horizon, int _state_num);
  void update_problem_data(double *p, double *v, double *q, double *w, double *r, double yaw, double *weights, double *state_trajectory,
                           double alpha, int *gait);
  void set_acclerations(double *_ang_accel, double *_lin_accel);
  void set_manipulation_force(float *force);
  double get_solution(int index);
  RobotState rs;

protected:
  void initialize_mpc();
  void solve_mpc();
  void c2qp(Eigen::Matrix<fpt, 3, 3> I_world, fpt m, Eigen::Matrix<fpt, 3, 4> r_feet, Eigen::Matrix<fpt, 3, 3> R_yaw, fpt dt, s16 horizon);
  void resize_qp_mats(s16 horizon);

  bool first_run;
  problem_setup setup;
  u8 gait_data[K_MAX_GAIT_SEGMENTS];
  pthread_mutex_t problem_cfg_mt;
  pthread_mutex_t update_mt;
  update_data_t update;
  pthread_t solve_thread;
  int has_solved;
  int state_num;
  int control_num;
  float mu_2[6];
  float manipulation_force[3];
  float linear_accel[3];
  float angular_accel[3];

  Eigen::Matrix<fpt, Dynamic, Dynamic> A_qp;
  Eigen::Matrix<fpt, Dynamic, Dynamic> B_qp;
  Eigen::Matrix<fpt, Dynamic, Dynamic> Bdt;
  Eigen::Matrix<fpt, Dynamic, Dynamic> Adt;
  Eigen::Matrix<fpt, Dynamic, Dynamic> ABc, expmm;
  Eigen::Matrix<fpt, Dynamic, 1> x_0;
  Eigen::Matrix<fpt, Dynamic, Dynamic> S;
  Eigen::Matrix<fpt, Dynamic, 1> X_d;
  Eigen::Matrix<fpt, Dynamic, 1> U_b;
  Eigen::Matrix<fpt, Dynamic, Dynamic> fmat;
  Eigen::Matrix<fpt, Dynamic, Dynamic> A;
  Eigen::Matrix<fpt, Dynamic, Dynamic> B;
  Eigen::Matrix<fpt, Dynamic, Dynamic> powerMats[20];
  Eigen::Matrix<fpt, Dynamic, 1> full_weight;
  Eigen::Matrix<fpt, Dynamic, Dynamic> qH;
  Eigen::Matrix<fpt, Dynamic, 1> qg;
  Eigen::Matrix<fpt, Dynamic, Dynamic> eye_12h;
  Eigen::Matrix<fpt, 5, 3> f_block;
  fpt mu;

  qpOASES::real_t *H_qpoases;
  qpOASES::real_t *g_qpoases;
  qpOASES::real_t *A_qpoases;
  qpOASES::real_t *lb_qpoases;
  qpOASES::real_t *ub_qpoases;
  qpOASES::real_t *q_soln;
  qpOASES::real_t *H_red;
  qpOASES::real_t *g_red;
  qpOASES::real_t *A_red;
  qpOASES::real_t *lb_red;
  qpOASES::real_t *ub_red;
  qpOASES::real_t *q_red;
  u8 real_allocated = 0;

  char var_elim[2000];
  char con_elim[2000];
};
#endif
