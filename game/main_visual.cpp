#include <SFML/Graphics.hpp>
#include <iostream>

#include "Eigen/Dense"
#include "inverted_pendulum.h"
#include "tools.h"
#include "pid.h"
#include "lqr.h"
#include "/home/yasin/scarab/utils/scarab_markers.h"
#include "mpc/NLMPC.hpp"

#define CONTROL_FREQUENCY 25.0 // hz

int main()
{
  // Choose controller MPC, LQR or PID
  std::string controller = "MPC";
  // Set initial conditions
  const double p_0 = 0;
  const double theta_0 = 40;
  Eigen::VectorXd x_0(4);
  x_0 << p_0, to_radians(theta_0), 0, 0;

  // MPC Parameters
  constexpr int num_states = 4;
  constexpr int num_output = 0;
  constexpr int num_inputs = 1;
  constexpr int pred_hor = 25;
  constexpr int ctrl_hor = 5;
  constexpr int ineq_c = 0;
  constexpr int eq_c = 0;

  double ts = 0.1;

  mpc::NLMPC<
      num_states, num_inputs, num_output,
      pred_hor, ctrl_hor,
      ineq_c, eq_c>
      optsolver;

  optsolver.setLoggerLevel(mpc::Logger::log_level::NORMAL);
  optsolver.setContinuosTimeModel(ts);

  auto stateEq = [&](
                     mpc::cvec<num_states> &x_dot_,
                     const mpc::cvec<num_states> &x_,
                     const mpc::cvec<num_inputs> &u)
  {
    // Constants
    double M_ = 1.0;
    double m_ = 1.0;
    double J_ = 1.0;
    double l_ = 1.0;
    double c_ = 1.0;
    double gamma_ = 1.0;
    double g_ = 9.81;
    double M_t_ = M_ + m_;
    double J_t_ = J_ + m_ * std::pow(l_, 2);

    // Recover state parameters
    double x = x_(0);     // position of the base
    double theta = x_(1); // angle of the pendulum
    double vx = x_(2);    // velocity of the base
    double omega = x_(3); // angular rate of the pendulum

    // Compute common terms
    double s_t = std::sin(theta);
    double c_t = std::cos(theta);
    double o_2 = std::pow(omega, 2);
    double l_2 = std::pow(l_, 2);

    // Calculate derivatives
    x_dot_(0) = vx;
    x_dot_(1) = omega;
    x_dot_(2) = (-m_ * l_ * s_t * o_2 + m_ * g_ * (m_ * l_2 / J_t_) * s_t * c_t -
                 c_ * vx - (gamma_ / J_t_) * m_ * l_ * c_t * omega + u(0)) /
                (M_t_ - m_ * (m_ * l_2 / J_t_) * c_t * c_t);
    x_dot_(3) =
        (-m_ * l_2 * s_t * c_t * o_2 + M_t_ * g_ * l_ * s_t - c_ * l_ * c_t * vx -
         gamma_ * (M_t_ / m_) * omega + l_ * c_t * u(0)) /
        (J_t_ * (M_t_ / m_) - m_ * (l_ * c_t) * (l_ * c_t));
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
                                 { return x.array().square().sum() + u.array().square().sum() / 100; });

  // MPC
  mpc::cvec<num_states> modelX, modeldX;
  modelX.resize(num_states);
  modeldX.resize(num_states);

  modelX(0) = p_0;
  modelX(1) = to_radians(theta_0);
  modelX(2) = 0.0;
  modelX(3) = 0.0;

  auto r = optsolver.getLastResult();

  // Set PID constants
  const double kp = 100.0F;
  const double ki = 50.0F;
  const double kd = 10.0F;

  // Create a model with default parameters
  InvertedPendulum *ptr = new InvertedPendulum(x_0);
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

  ///*
  // Load font
  sf::Font font;
  if (!font.loadFromFile("Roboto-Regular.ttf"))
  {
    std::cout << "Failed to load font!\n";
  }
  sf::RenderWindow window(sf::VideoMode(640, 480), "Inverted Pendulum");

  // Create text to display simulation time
  sf::Text text;
  text.setFont(font);
  text.setCharacterSize(24);
  const sf::Color grey = sf::Color(0x7E, 0x7E, 0x7E);
  text.setFillColor(grey);
  text.setPosition(480.0F, 360.0F);

  // Create text to display controller type
  sf::Text type;
  type.setFont(font);
  type.setCharacterSize(24);
  const sf::Color turquoise = sf::Color(0x06, 0xC2, 0xAC);
  type.setFillColor(turquoise);
  type.setPosition(480.0F, 384.0F);

  // Create a track for the cart
  sf::RectangleShape track(sf::Vector2f(640.0F, 2.0F));
  track.setOrigin(320.0F, 1.0F);
  track.setPosition(320.0F, 240.0F);
  const sf::Color light_grey = sf::Color(0xAA, 0xAA, 0xAA);
  track.setFillColor(light_grey);

  // Create the cart of the inverted pendulum
  sf::RectangleShape cart(sf::Vector2f(100.0F, 100.0F));
  cart.setOrigin(50.0F, 50.0F);
  cart.setPosition(320.0F, 240.0F);
  cart.setFillColor(sf::Color::Black);

  // Create the pole of the inverted pendulum
  sf::RectangleShape pole(sf::Vector2f(20.0F, 200.0F));
  pole.setOrigin(10.0F, 200.0F);
  pole.setPosition(320.0F, 240.0F);
  pole.setRotation(-theta_0);
  const sf::Color brown = sf::Color(0xCC, 0x99, 0x66);
  pole.setFillColor(brown);
  //*/

  // Create a clock to run the simulation
  sf::Clock clock;
  float time = clock.getElapsedTime().asSeconds();
  float last_input_update_time = 0;
  int roi_count = 0;
  double u = 0;
  while (1)
  {
    // Update the simulation
    time = clock.getElapsedTime().asSeconds();
    ///*
    const std::string msg = std::to_string(time);
    text.setString("Time   " + msg.substr(0, msg.find('.') + 2));
    type.setString(controller);
    //*/

    // Control calculations every 1/CONTROL_REQUENCY seconds
    if (time - last_input_update_time > 1.0 / CONTROL_FREQUENCY)
    {
      if (time < 30)
      {
        // Control calculations starts here
        // scarab_roi_dump_begin();
        u = 0;
        if (controller == "PID")
        {
          double angle = ptr->GetState()(1);
          double error = 0.0F - angle;
          c_ptr->UpdateError(time, error);
          u = c_ptr->TotalError();
        }
        else if (controller == "LQR")
        {
          u = optimal.Control(ptr->GetState())(0, 0);
        }
        else if (controller == "MPC")
        {
          r = optsolver.step(modelX, r.cmd);
          auto seq = optsolver.getOptimalSequence();
          (void)seq;
          u = r.cmd(0);
          modelX = ptr->GetState();
          // std::cout << "u: " << u << std::endl;
          // std::cout << "x: " << modelX << std::endl;
        }

        // Control calculations ends here
        // scarab_roi_dump_end();
        // roi_count++;
        // std::cout << "ROI Count: " << roi_count << std::endl;
        last_input_update_time = time;

        // Apply input to the system
        ptr->Update(time, u);
      }
      else
      {
        break; // remove this
        delete ptr;
        delete c_ptr;
        ptr = new InvertedPendulum(x_0);
        c_ptr = new PID();
        c_ptr->Init(kp, ki, kd);
        clock.restart();
        last_input_update_time = 0;
        // pid = (pid) ? false : true;
      }
    }
    else
    {
      ptr->Update(time, u);
    }
    Eigen::VectorXd x = ptr->GetState();

    ///*
    // Update SFML drawings
    cart.setPosition(320.0F + 100 * x(0), 240.0F);
    pole.setPosition(320.0F + 100 * x(0), 240.0F);
    pole.setRotation(to_degrees(-x(1)));

    window.clear(sf::Color::White);
    window.draw(track);
    window.draw(cart);
    window.draw(pole);
    window.draw(text);
    window.draw(type);
    window.display();
    //*/
  }
  return 0;
}
