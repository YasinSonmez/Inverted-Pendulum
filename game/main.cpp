#include <SFML/Graphics.hpp>
#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include "inverted_pendulum.h"
#include "tools.h"
#include "pid.h"
#include "lqr.h"
#include "/home/yasin/scarab/utils/scarab_markers.h"
#include "mpc/NLMPC.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main()
{
  // Open the parameters file
  std::ifstream file("../game/params.json");

  // Read the contents of the file into a string
  std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  // Parse the JSON string
  json params = json::parse(str);

  // Access the parameters and create variables for them
  std::string controller = params["controller"];
  const double control_frequecy = params["control_frequency"];
  double simulation_length = params["simulation_length"];
  const double p_0 = params["p_0"];
  const double theta_0 = params["theta_0"];
  // Constants
  double M = params["M"];
  double m = params["m"];
  double J = params["J"];
  double l = params["l"];
  double c = params["c"];
  double gamma = params["gamma"];
  double g = params["g"];

  Eigen::VectorXd x_0(4);
  x_0 << p_0, to_radians(theta_0), 0, 0;
  // Create a model with default parameters
  InvertedPendulum *ptr = new InvertedPendulum(M, m, J, l, c, gamma, x_0);

  // Design MPC controller
  // MPC Parameters
  constexpr int num_states = 4;
  constexpr int num_output = 0;
  constexpr int num_inputs = 1;
  constexpr int pred_hor = 25;
  constexpr int ctrl_hor = 5;
  constexpr int ineq_c = 0;
  constexpr int eq_c = 0;
  double sampling_time = params["MPC"]["sampling_time"];
  double input_cost_weight = params["MPC"]["input_cost_weight"];

  mpc::NLMPC<num_states, num_inputs, num_output, pred_hor, ctrl_hor, ineq_c, eq_c> optsolver;

  optsolver.setLoggerLevel(mpc::Logger::log_level::NORMAL);
  optsolver.setContinuosTimeModel(sampling_time);

  auto stateEq = [&](
                     mpc::cvec<num_states> &x_dot_,
                     const mpc::cvec<num_states> &x_,
                     const mpc::cvec<num_inputs> &u)
  {
    // Constants
    double M_t = M + m;
    double J_t = J + m * std::pow(l, 2);
    // Recover state parameters
    double x = x_(0);     // position of the base
    double theta = x_(1); // angle of the pendulum
    double vx = x_(2);    // velocity of the base
    double omega = x_(3); // angular rate of the pendulum

    // Compute common terms
    double s_t = std::sin(theta);
    double c_t = std::cos(theta);
    double o_2 = std::pow(omega, 2);
    double l_2 = std::pow(l, 2);

    // Calculate derivatives
    x_dot_(0) = vx;
    x_dot_(1) = omega;
    x_dot_(2) = (-m * l * s_t * o_2 + m * g * (m * l_2 / J_t) * s_t * c_t -
                 c * vx - (gamma / J_t) * m * l * c_t * omega + u(0)) /
                (M_t - m * (m * l_2 / J_t) * c_t * c_t);
    x_dot_(3) =
        (-m * l_2 * s_t * c_t * o_2 + M_t * g * l * s_t - c * l * c_t * vx -
         gamma * (M_t / m) * omega + l * c_t * u(0)) /
        (J_t * (M_t / m) - m * (l * c_t) * (l * c_t));
  };

  optsolver.setStateSpaceFunction([&](
                                      mpc::cvec<num_states> &dx,
                                      const mpc::cvec<num_states> &x,
                                      const mpc::cvec<num_inputs> &u,
                                      const unsigned int &)
                                  { stateEq(dx, x, u); });
  optsolver.setObjectiveFunction([&](
                                     const mpc::mat<pred_hor + 1, num_states> &x,
                                     const mpc::mat<pred_hor + 1, num_output> &,
                                     const mpc::mat<pred_hor + 1, num_inputs> &u,
                                     double)
                                 { return x.array().square().sum() + u.array().square().sum() * input_cost_weight; });

  // MPC
  mpc::cvec<num_states> modelX, modeldX;
  modelX.resize(num_states);
  modeldX.resize(num_states);

  modelX(0) = p_0;
  modelX(1) = to_radians(theta_0);
  modelX(2) = 0.0;
  modelX(3) = 0.0;

  auto r = optsolver.getLastResult();

  // Design PID controller
  const double kp = params["PID"]["kp"];
  const double ki = params["PID"]["ki"];
  const double kd = params["PID"]["kd"];
  PID *c_ptr = new PID();
  c_ptr->Init(kp, ki, kd);

  // Design LQR controller
  LQR optimal;
  ptr->Linearize();
  optimal.A_ = ptr->A_;
  optimal.B_ = ptr->B_;
  optimal.Q_ = Eigen::MatrixXd::Identity(4, 4);
  optimal.Q_(0, 0) = 10;
  optimal.R_ = Eigen::MatrixXd::Identity(1, 1);
  optimal.Compute();

  // Create a clock to run the simulation
  sf::Clock clock;
  float time = clock.getElapsedTime().asSeconds();
  float last_input_update_time = 0;
  int roi_count = 0;
  double u = 0;

  // Simulation loop
  while (time < simulation_length)
  {
    // Update the simulation
    time = clock.getElapsedTime().asSeconds();

    // Get state
    Eigen::VectorXd x = ptr->GetState();

    // Control calculations every 1/CONTROL_REQUENCY seconds
    if (time - last_input_update_time > 1.0 / control_frequecy)
    {
      // Control calculations starts here
      scarab_roi_dump_begin();
      if (controller == "PID")
      {
        double angle = x(1);
        double error = 0.0F - angle;
        c_ptr->UpdateError(time, error);
        u = c_ptr->TotalError();
      }
      else if (controller == "LQR")
      {
        u = optimal.Control(x)(0, 0);
      }
      else if (controller == "MPC")
      {
        modelX = x;
        r = optsolver.step(modelX, r.cmd);
        u = r.cmd(0);
      }
      // Control calculations ends here
      scarab_roi_dump_end();
      roi_count++;
      std::cout << "ROI Count: " << roi_count << std::endl;
      last_input_update_time = time;
    }
    // Apply input to the system
    ptr->Update(time, u);
  }
  return 0;
}