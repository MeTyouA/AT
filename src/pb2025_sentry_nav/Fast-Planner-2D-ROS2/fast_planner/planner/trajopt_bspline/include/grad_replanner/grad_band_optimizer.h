#ifndef _GRAD_BAND_OPTIMIZER_H_
#define _GRAD_BAND_OPTIMIZER_H_

// #include <ros/ros.h>
#include <Eigen/Eigen>
#include "../grad_replanner/utils/font_color.h"
#include "sdf_map.h"
#include "rc_esdf.h"
#include <memory>
#include "../quadrotor_msgs/time_.h"


// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point

class GradBandOptimizer {
 private:
  /* data */
  // SDFMap sdf_map;
  shared_ptr<SDFMap> sdf_map;
  Eigen::MatrixXd control_points;  // nx3
  Eigen::MatrixXd path_points;     // 3 x n-2
  Eigen::MatrixXd path_acc;        // 3 x n-2
  Eigen::Vector3d end_pt;

  /* optimization parameters */
  double lamda1_init;  // initial curvature weight
  double lamda1;       // curvature weight consider pts num
  double lamda2;       // distance weight
  double lamda3;       // barrier function weight
  double lamda3p;      // barrier function weight
  double lamda4;       // imitate weight
  double alpha, beta;  // exp barrier param
  double dist0;
  double rc_esdf_lambda;
  double rc_esdf_margin;
  double rc_esdf_query_radius;
  double max_vel, max_acc;  // constrains parameters
  int variable_num;
  int algorithm;
  int max_iteration_num;
  int rc_esdf_obstacle_sample_step;
  int rc_esdf_max_obstacles_per_point;
  std::vector<double> best_variable;
  double min_cost;
  int start_id, end_id, origin_pts_num;
  bool rc_esdf_enabled;
  bool rc_esdf_ready;
  RcEsdfMap rc_esdf_map;
  std::vector<Eigen::Vector2d> rc_esdf_obstacles;

  /* bspline */
  double bspline_interval;  // ts
  int order;                // bspline order

 public:
  GradBandOptimizer() {}
  ~GradBandOptimizer() {}

  /* main API */
  void setControlPoints(Eigen::MatrixXd points);
  void setBSplineInterval(double ts);
  void setDistanceField(const shared_ptr<SDFMap>& map);
  void setParameterAuto(double max_v, double max_a);
  void setEsdfPenalty(double distance_weight, double safe_distance);
  void setRcEsdfConfig(bool enabled, double lambda, double margin, double map_width,
                       double map_height, double resolution, double query_radius,
                       int max_obstacles_per_point, int obstacle_sample_step,
                       const std::vector<Eigen::Vector2d>& footprint);
  void setRcEsdfObstacles(const std::vector<Eigen::Vector3d>& obstacles);
  //void setParameter(double lamda1_, double lamda2_, double lamda3_, double max_vel_, double max_acc_);
  void renewLambda1(double lamda1_);
  void renewLambda2(double lamda2_);
  void resetLambda2();

  void setOptimizationRange(int start, int end);

  void optimizeTrajFixedEnd();  // NLopt, all at one time, currently use
  void optimizeTrajFreeEnd();
  Eigen::MatrixXd getControlPoints();

 private:
  /* NLopt cost */
  static double costFuncMinCurvatureParaFreeEnd(const std::vector<double>& x,
                                                std::vector<double>& grad,
                                                void* func_data);
  static double costFuncMinCurvaturePara(const std::vector<double>& x,
                                         std::vector<double>& grad,
                                         void* func_data);
  static double costFuncMinJerkPara(const std::vector<double>& x,
                                    std::vector<double>& grad, void* func_data);
  static double costFuncMinCurvatureExp(const std::vector<double>& x,
                                        std::vector<double>& grad,
                                        void* func_data);
  static double velConstraint(const std::vector<double>& x,
                              std::vector<double>& grad, void* data);
  static double accConstraint(const std::vector<double>& x,
                              std::vector<double>& grad, void* data);
  /* helper function */
  void getDistanceAndGradient(Eigen::Vector3d& pos, double& dist,
                              Eigen::Vector3d& grad);
  double getRcEsdfCostAndGradient(const Eigen::Vector3d& q1,
                                  const Eigen::Vector3d& q2,
                                  const Eigen::Vector3d& q3,
                                  Eigen::Vector3d& grad_q2) const;

 public:
  /* for evaluation */
  vector<double> vec_cost;
  vector<double> vec_time;
  quadrotor_msgs::Time time_start;

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost;
    time = vec_time;
  }
};

#endif
