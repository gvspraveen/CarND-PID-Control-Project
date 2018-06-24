# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program


## Description of Implementation

In this project I implemented PID controller following lesson by Sebastian on PID controller.

I implemented the algorithm in two step process. In Step1, I manually tuned paramters to get car along track.
In Step2, I implemented twidle algorithm to auto tune parameters to complete a lap without letting car go off track.

### Step1: Manual Tuning

Followed some discussion in Udacity forum, I first tried to manually tune parameters to get a starting point for my twiddle algorithm.
I tuned each parameter one at a time.

1. **P (Proportional) Component**: As explained in lecture videos, P directly effects the cross track error. It tries to directly
negate cross track error at any given time step. It does not consider past history. In project code, P component is represented by `Kp` variable. To run
this parameter, I set `Ki` and `Kd` to 0. And kept tweaking `Kp`. I figured it is impossible to keep car on track just by tweaking this.
Considering that, I decided to tune it to keep it on track at least slightly less curvy sections of road. Fially I settled on `Kp=0.12`.

2. **D (Differential) Component**: Unlike `P`, `D` component tries to take into account rate of change of `CTE`. It helps in cases of car drifting towards edge, my reacting
to change of CTE and ajusts steering based on that. Having preset `Kp` in previous step. I started tweaking `Kd` with goal of keep car on the track during the first few curves.
Finally I found a sweet spot at `Kd=1.28`

3. **I (Integral) Component**: The problem with `Kd` is it is very reactive to difference in error for previous time step. `Ki` counters that
by taking into accout total error in past few steps. Essentially, it is reducing average error over a period of time. Finally I got to sweet spot
of `Ki=0.0003`

So finally manual parameters that I got are:

```
Kp=0.12
Kd=1.28
Ki=0.0003
```


While this tends to give Okay results, one big problem I found is, sometimes on step curves car is entering into yellow lines on track.
Beyond this point, manually tuning is becoming difficult.

Implementation of manual tuning can be found in [Commit 6e9888af9d4557dd68b3d7d20b023471821c0f5f](https://github.com/gvspraveen/CarND-PID-Control-Project/commit/6e9888af9d4557dd68b3d7d20b023471821c0f5f)


### Step2: Implementing Twiddle

Finally I implemented twiddle algorithm as indicated by Sebastian in the lecture videos. I used resources in Udacity
discussion board, to get some insights on how to get started on this.

#### Useful links:

1. https://discussions.udacity.com/t/how-to-tune-parameters/303845/14
2. https://discussions.udacity.com/t/how-to-implement-twiddle-optimisation/279749/5

I started off with parameters manually tuned in Step1. Then every 500 steps, I invoke twiddle algorithm
in `PID.cpp`. After every 500 steps, I reset the simulator to beginning position. And kept running this while
monitoring output of twiddle and total_error.

#### Why 500 steps?
Simulator has some randomness in itself. On average I found that car takes 500 time steps to get on the bridge.
This idea was obtained from discussion forum links above.

#### Monitoring twiddle output
I printed out some console logs in twiddle algorithm, which helped figure out when twiddle finally converged. After about 30 minutes
of running this simulator on loop, I was able to see twiddle converge on a specific set of params.

Sample console logs


```
Connected!!!
Params before twiddle: P: 0.131, I: 0.00025, D: 1.33
total_error: 509.3
==================== Running twiddle =========================
==================== Found best error =========================
Params after twiddle: P: 0.131, I: 0.00025, D: 1.33
==================== Resetting =========================

Connected!!!
Params before twiddle: P: 0.131, I: 0.00025, D: 1.33
total_error: 534.755
==================== Running twiddle =========================
==================== Trying adding to param: 2 =========================
Params after twiddle: P: 0.131, I: 0.000275, D: 1.33
==================== Resetting =========================

Connected!!!
Params before twiddle: P: 0.131, I: 0.000275, D: 1.33
total_error: 663.931
==================== Running twiddle =========================
==================== Trying subtracting from param: 2 =========================
Params after twiddle: P: 0.131, I: 0.000225, D: 1.33
==================== Resetting =========================

Connected!!!
Params before twiddle: P: 0.131, I: 0.000225, D: 1.33
total_error: 606.748
==================== Running twiddle =========================
==================== Resetting param: 2 =========================
Params after twiddle: P: 0.131, I: 0.00025, D: 1.33
==================== Resetting =========================

Connected!!!
Params before twiddle: P: 0.131, I: 0.00025, D: 1.33
total_error: 681.043
==================== Running twiddle =========================
==================== Trying adding to param: 0 =========================
Params after twiddle: P: 0.136895, I: 0.00025, D: 1.33
==================== Resetting =========================
```


#### Final parameters

```
Kp=0.131917
Kd=1.40623
Ki=0.00022975
```

With these parameters, I could get the car to go around the track well, without going
off track.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

