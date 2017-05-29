#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define STEER_LIMIT 0.436332


using namespace std;

double polyeval(Eigen::VectorXd coeffs, double x);

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order);

template <typename T>
Eigen::VectorXd convert_vec_to_eigen(const std::vector<T>& vec) {
  Eigen::VectorXd eigen_vec(vec.size());
  for(int i = 0; i < vec.size(); ++i) {
    eigen_vec[i] = vec[i];
  }
  return eigen_vec;
}

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return all x, y, v, psi, cte, epsi, delta and accel. values
  void Solve(Eigen::VectorXd x0,
             Eigen::VectorXd coeffs,
             vector<double>& mpc_x_vals,
             vector<double>& mpc_y_vals,
             vector<double>& mpc_psi_vals,
             vector<double>& mpc_v_vals,
             vector<double>& mpc_cte_vals,
             vector<double>& mpc_epsi_vals,
             vector<double>& mpc_delta_vals,
             vector<double>& mpc_a_vals );
};

#endif /* MPC_H */
