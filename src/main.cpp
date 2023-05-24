#include <iostream>
#include <sciplot/sciplot.hpp>

#include "Eigen/Dense"
#include "inverted_pendulum.h"
#include "tools.h"

int main()
{
  // Set initial conditions
  Eigen::VectorXd x_0(4);
  x_0 << 0, M_PI / 4, 0, 0;

  // Create a model with default parameters
  InvertedPendulum model(x_0);

  // Define simulation time
  sciplot::Vec time = sciplot::linspace(0.0, 20.0, 1000);

  // Store data from the simulation
  std::vector<std::vector<double>> data;
  std::vector<double> theta;

  // Simulate model
  for (auto t : time)
  {
    model.Update(t, 0);

    Eigen::VectorXd x = model.GetState();
    theta.push_back(x(1));
    data.push_back({t, x(0), x(1)});
  }

  // Create Plot object
  sciplot::Plot2D plot;

  // Set color palette
  plot.palette("set2");

  // Draw theta over the simulation time
  plot.drawCurve(time, theta).label("theta(t)").lineWidth(4);

  // Put both plots in a "figure" horizontally next to each other
  sciplot::Figure figure = {{plot}};

  // Create a canvas / drawing area to hold figure and plots
  sciplot::Canvas canvas = {{figure}};
  // Set color palette for all Plots that do not have a palette set (plot2) / the default palette
  canvas.defaultPalette("set1");

  // Show the canvas in a pop-up window
  canvas.show();

  // Save the plot to a SVG file
  canvas.save("example.svg");

  // Export data to file
  Export("data.csv", {"time", "position", "angle"}, data);
}
