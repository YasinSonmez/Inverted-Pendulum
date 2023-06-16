# Inverted Pendulum
![build](https://github.com/jasleon/Inverted-Pendulum/actions/workflows/main.yml/badge.svg)
![GitHub last commit](https://img.shields.io/github/last-commit/jasleon/Inverted-Pendulum)

A simple simulator to test control algorithms in C++. There are currently three algorithms: PID, LQR, and MPC

<p align="center">
  <img src="img/carpole-free.gif" width=50% height=50%/>
</p>
<p align="center">
    Inverted pendulum in open-loop
</p>

<p align="center">
  <img src="img/cartpole-pid.gif" width=50% height=50%/>
</p>
<p align="center">
    Inverted pendulum in closed-loop with PID
</p>

<p align="center">
  <img src="img/cartpole-lqr.gif" width=50% height=50%/>
</p>
<p align="center">
    Inverted pendulum in closed-loop with LQR
</p>

## Dependencies
- cmake
- make
- gcc/g++
- [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (needs to  be copied to /usr/local/include directory)
- [libmpc++](https://github.com/nicolapiccinelli/libmpc) (even though libmpc++ is header only, it needs to be installed using `make install` to work in this project)
- [sfml](https://www.sfml-dev.org/)
- [googletest](https://github.com/google/googletest) (optional)
- [sciplot](https://github.com/sciplot/sciplot) (optional)

## Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./cartpole` or `./cartpole_visual` to see graphics
5. If you need to adjust some of the parameters, simply modify the `game/params.json` file without the need for recompiling.

## Guidelines
- [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html)
- [Udacity Git Commit Message Style Guide](http://udacity.github.io/git-styleguide/)

## References
### Projects
- [CartPole-v1](https://gym.openai.com/envs/CartPole-v1/)
- [Riccati_Solver](https://github.com/TakaHoribe/Riccati_Solver)
- [libmpc++](https://github.com/nicolapiccinelli/libmpc)
### Tutorials
- [Install SFML in Linux](https://www.sfml-dev.org/tutorials/2.5/start-linux.php)
- [Install sciplot](https://sciplot.github.io/installation/)
### Books
- Åström, K. J., & Murray, R. M. (2021). *Feedback systems: An introduction for scientists and engineers* (2nd ed.). Princeton University Press. ([online](https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers))
