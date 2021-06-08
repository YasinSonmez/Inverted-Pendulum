#include <SFML/Graphics.hpp>
#include <iostream>

#include "Eigen/Dense"
#include "inverted_pendulum.h"
#include "tools.h"
#include "pid.h"

int main() {
  sf::RenderWindow window(sf::VideoMode(640, 480), "Inverted Pendulum");

  // Set initial conditions
  const double p_0 = 0;
  const double theta_0 = -5;
  Eigen::VectorXd x_0(4);
  x_0 << p_0, to_radians(theta_0), 0, 0;

  // Set PID constants
  const double kp = 100.0F;
  const double ki = 50.0F;
  const double kd = 10.0F;

  // Create a model with default parameters
  InvertedPendulum *ptr = new InvertedPendulum(x_0);
  PID *c_ptr = new PID();
  c_ptr->Init(kp, ki, kd);

  // Load font
  sf::Font font;
  if (!font.loadFromFile("Roboto-Regular.ttf")) {
    std::cout << "Failed to load font!\n";
  }

  // Create text to display simulation time
  sf::Text text;
  text.setFont(font);
  text.setCharacterSize(24);
  const sf::Color grey = sf::Color(0x7E, 0x7E, 0x7E);
  text.setFillColor(grey);
  text.setPosition(480.0F, 360.0F);

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

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      switch (event.type) {
        case sf::Event::Closed:
          window.close();
          break;
      }
    }

    // Update the simulation
    sf::Time elapsed = clock.getElapsedTime();
    const float time = elapsed.asSeconds();
    const std::string msg = std::to_string(time);
    text.setString("Time " + msg.substr(0, msg.find('.') + 2));
    if (time < 15) {
      double angle = ptr->GetState()(1);
      double error = 0.0F - angle;
      std::cout << "angle: " << angle << std::endl;
      c_ptr->UpdateError(time, error);
      ptr->Update(time, c_ptr->TotalError());
    } else {
      delete ptr;
      delete c_ptr;
      ptr = new InvertedPendulum(x_0);
      PID *c_ptr = new PID();
      c_ptr->Init(kp, ki, kd);
      clock.restart();
    }

    Eigen::VectorXd x = ptr->GetState();

    // Update SFML drawings
    cart.setPosition(320.0F + 100 * x(0), 240.0F);
    pole.setPosition(320.0F + 100 * x(0), 240.0F);
    pole.setRotation(to_degrees(-x(1)));

    window.clear(sf::Color::White);
    window.draw(track);
    window.draw(cart);
    window.draw(pole);
    window.draw(text);
    window.display();
  }
  return 0;
}
