// #include <SFML/Graphics.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "Eigen/Dense"
#include "inverted_pendulum.h"
#include "tools.h"
#include "pid.h"
#include "lqr.h"
#include "mpc/NLMPC.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

void saveToCSV(const Eigen::MatrixXd &matrix, const std::string &filename)
{
    std::ofstream file(filename, std::ios::app); // Append mode

    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (int i = 0; i < matrix.rows(); ++i)
    {
        for (int j = 0; j < matrix.cols(); ++j)
        {
            if (j > 0)
                file << ",";
            file << matrix(i, j);
        }
        file << "\n";
    }

    file.close();
}

void removeLastRows_saveToCSV(const Eigen::MatrixXd &matrix, const std::string &filename, int start_idx)
{
    // Read existing content (first start_idx rows)
    std::ifstream existingFile(filename);
    std::vector<std::string> existingContent;

    if (existingFile.is_open())
    {
        std::string line;
        for (size_t i = 0; i < start_idx; ++i)
        {
            std::getline(existingFile, line);
            existingContent.push_back(line);
        }
        existingFile.close();
    }

    // Open the file in truncate mode to clear existing content
    std::ofstream file(filename, std::ios::trunc);

    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write back the existing content (first start_idx rows)
    if (existingContent.size() > 0)
    {
        for (size_t i = 0; i < existingContent.size(); ++i)
        {
            file << existingContent[i] << "\n";
        }
    }

    // Write the new matrix content to the file
    for (int i = 0; i < matrix.rows(); ++i)
    {
        for (int j = 0; j < matrix.cols(); ++j)
        {
            if (j > 0)
                file << ",";
            file << matrix(i, j);
        }
        file << "\n";
    }

    file.close();
}

Eigen::MatrixXd loadFromCSV(const std::string &filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return Eigen::MatrixXd(0, 0);
    }

    std::vector<std::vector<double>> values;
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::vector<double> row;
        std::string value;

        while (std::getline(ss, value, ','))
        {
            row.push_back(std::stod(value));
        }

        values.push_back(row);
    }

    int rows = values.size();
    int cols = (rows > 0) ? values[0].size() : 0;

    Eigen::MatrixXd matrix(rows, cols);

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            matrix(i, j) = values[i][j];
        }
    }

    file.close();
    return matrix;
}

int main(int argc, char *argv[])
{
    // Open the parameters file
    std::ifstream file("../params.json");

    // Read the contents of the file into a string
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Parse the JSON string
    json params = json::parse(str);

    // filename to save matrix
    std::string filename = "../state_and_input_matrix.csv";
    std::string global_filename = "../../state_and_input_matrix.csv";
    // Access the parameters and create variables for them
    std::string controller = params["controller"];
    bool record_trace = params["record_trace"];
    const double control_frequecy = params["control_frequency"];
    int simulation_steps = params["simulation_steps"];
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

    // Create a vector to store x_t, u_t in each iteration
    std::vector<Eigen::VectorXd> xu_vector;
    Eigen::VectorXd x_0(4);
    int start_idx = 0;
    double u_start = 0;
    if (argc == 1)
    {
        x_0 << p_0, to_radians(theta_0), 0, 0;
    }
    else if (argc >= 2)
    {
        Eigen::MatrixXd xu_matrix_tmp;
        if (std::stoi(argv[1]) == 0)
        {
            x_0 << p_0, to_radians(theta_0), 0, 0;
        }
        else
        {
            xu_matrix_tmp = loadFromCSV(global_filename);
            start_idx = std::stoi(argv[1]);
            x_0 << xu_matrix_tmp(start_idx, 1), xu_matrix_tmp(start_idx, 2),
                xu_matrix_tmp(start_idx, 3), xu_matrix_tmp(start_idx, 4);
        }

        if (argc >= 3)
        {
            simulation_steps = std::stoi(argv[2]);
        }
        if (argc == 4)
        {
            // If a 4th argument is given, use the last control input
            u_start = xu_matrix_tmp(start_idx - 1, 5);
        }
    }
    std::cout << "Doing simulation for this number of timesteps: " << simulation_steps << std::endl;
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
                                   { 
                                    std::vector<double> w = params["MPC"]["state_cost_weights"].get<std::vector<double>>();
                                    Eigen::VectorXd state_cost_weights = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(w.data(), w.size()); // create a weight vector
                                    return (x * state_cost_weights.asDiagonal()).array().square().sum() + u.array().square().sum() * input_cost_weight; });

    // MPC
    mpc::cvec<num_states> modelX, modeldX;
    modelX.resize(num_states);
    modeldX.resize(num_states);

    modelX(0) = p_0;
    modelX(1) = to_radians(theta_0);
    modelX(2) = 0.0;
    modelX(3) = 0.0;

    auto r = optsolver.getLastResult();

    //  Design PID controller
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

    // Load font
    // sf::Font font;
    // if (!font.loadFromFile("Roboto-Regular.ttf"))
    // {
    //     std::cout << "Failed to load font!\n";
    // }
    // sf::RenderWindow window(sf::VideoMode(640, 480), "Inverted Pendulum");

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
    type.setString(controller);

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

    // Create a clock to run the simulation
    sf::Clock clock;
    float time = clock.getElapsedTime().asSeconds();
    float last_input_update_time = 0;
    int roi_count = 0;
    double u = 0;

    // Simulation loop
    while (roi_count < simulation_steps)
    {
        // Update the simulation
        time = clock.getElapsedTime().asSeconds();

        // Get state
        Eigen::VectorXd x = ptr->GetState();

        // Control calculations every 1/CONTROL_REQUENCY seconds
        if (time - last_input_update_time > 1.0 / control_frequecy || roi_count == 0)
        {
            // Control calculations starts here
            if (argc == 4 && roi_count == 0)
            {
                u = u_start;
            }
            else if (controller == "PID")
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
            roi_count++;
            std::cout << "ROI Count: " << roi_count << std::endl;
            last_input_update_time = time;

            // Record state and input x_t,u_t
            Eigen::VectorXd xu(x.size() + 1);
            xu << x, u;
            xu_vector.push_back(xu);
        }

        // Apply input to the system
        ptr->Update(time, u);

        const std::string msg = std::to_string(time);
        text.setString("Time   " + msg.substr(0, msg.find('.') + 2));
        // Update SFML drawings
        // cart.setPosition(320.0F + 100 * x(0), 240.0F);
        // pole.setPosition(320.0F + 100 * x(0), 240.0F);
        // pole.setRotation(to_degrees(-x(1)));

        // window.clear(sf::Color::White);
        // window.draw(track);
        // window.draw(cart);
        // window.draw(pole);
        // window.draw(text);
        // window.draw(type);
        // window.display();
    }
    // Record the last state
    // Get state
    Eigen::VectorXd x = ptr->GetState();
    Eigen::VectorXd xu(x.size() + 1);
    xu << x, 0.0;
    xu_vector.push_back(xu);
    Eigen::MatrixXd xu_matrix(xu_vector.size(), xu_vector[0].size() + 1); // +1 for the timestep column

    for (size_t i = 0; i < xu_vector.size(); ++i)
    {
        xu_matrix(i, 0) = static_cast<double>(start_idx + i); // Writing row index at the first column
        xu_matrix.block(i, 1, 1, xu_vector[0].size()) = xu_vector[i].transpose();
    }
    // Print the resulting matrix
    std::cout << "Resulting matrix:\n"
              << xu_matrix << std::endl;

    // Save to CSV
    removeLastRows_saveToCSV(xu_matrix, filename, start_idx);
    removeLastRows_saveToCSV(xu_matrix, global_filename, start_idx);
    return 0;
}