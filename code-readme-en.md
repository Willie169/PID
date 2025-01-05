# PID

This repository implements a PID (Proportional-Integral-Derivative) controller designed to regulate the speed of an auto-following kart with a leader robot. The implementation includes the PID controller logic, a car simulation, and a framework for testing various PID configurations.

## PID.hpp

### Overview

This header file implements a PID controller that includes traditional PID control logic along with enhancements such as adaptive gain adjustment, adaptive integral term, error history management, and additional derivative term. It is also designed to be compatible with both Arduino and non-Arduino environments.

### Arduino and Non-Arduino Compatibility

The header is designed to be compatible with both Arduino and non-Arduino environments. This allows developers to use the same PID controller implementation across different platforms.

Conditional Compilation:
```cpp
#ifdef ARDUINO
#include <Arduino.h>
#include <math.h>
#define to_string(a) String(a)
#else
#include <cmath>
#include <string>
#define String string
using namespace std;
#endif
```

### Traditional PID Controller

The traditional PID controller is defined by the following equation:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt} 
$$

Where:
- $u(t)$ is the control output.
- $e(t)$ is the error at time $t$, defined as the difference between the desired value and the measured value.
- $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains, respectively.

### Adaptive Gain Adjustment

The proportional gain $K_p$ is dynamically adjusted based on the interaction between the current error $e(t)$ and its rate of change (derivative). This adjustment is designed to improve the responsiveness of the controller to changing conditions. The adaptation can be mathematically expressed as follows:

$$
K_p' = K_p \times (1 \pm eDP_a)
$$

Where:
- $eDP = e(t) \times \frac{de(t)}{dt}$
- $eDP_m$ is a threshold that determines when to adjust the proportional gain.
- $eDP_a$ is the adjustment factor that scales the change in $K_p$ based on the error’s derivative. 

The key idea behind this adjustment is to increase the proportional gain when the system is experiencing a large error combined with a significant rate of change, which could indicate an approaching overshoot or oscillation. Conversely, if the error is decreasing but remains large, it may reduce the proportional gain to prevent excessive correction, allowing the system to stabilize more smoothly.

### Adaptive Integral Term

Integral control is essential for eliminating steady-state error; however, it can lead to issues like integral windup, where the integral term accumulates excessively during periods of sustained error. To mitigate this, the integral gain $K_i$ is adaptively computed as follows:

$$
K_i = K_p \times rTiM \times ap
$$

Where:
- $ap = e^{-\frac{\text{amp}}{\text{maxAmp}}}$
- $rTiM$ is a constant that relates to the desired integral time constant.

The variable $amp$ represents the maximum amplitude of error observed over a session. By using the exponential decay factor $ap$, the contribution of the integral term is diminished during periods of high error, effectively reducing the risk of windup. When the error is within an acceptable range, $ap$ approaches 1, allowing the integral term to contribute effectively to the control output.

### Error History Management

Inspired by the **Ziegler–Nichols tuning method**, the implementation maintains a history of past errors and their timestamps to calculate the derivative and integral terms accurately. This history management is crucial for two main reasons:

- **Deriving the Rate of Change**: The derivative term requires knowledge of how fast the error is changing. By storing previous error values in a linked list, we can accurately compute the change in error over time ($\frac{de(t)}{dt}$):
  
$$
  dv = \frac{e(t) - preE}{dt}
$$
  
- **Integral Calculation**: The integral of the error is accumulated over time, enabling the controller to respond to persistent deviations from the setpoint:
  
$$
  intg += e(t) \times dt
$$

Using a linked list allows for efficient management of this error history. As new errors are added, older entries can be pruned to ensure that only relevant data is retained, which helps avoid excessive memory usage and keeps the calculations focused on the most recent behavior of the system.

### Additional Derivative Term

In addition to the standard derivative term $K_d$, an additional derivative term $K_{dd}$ is included in the PID controller. This term represents a second derivative component, which can be defined as follows:

$$
K_{dd} = K_d \times TddM
$$

Where:
- $TddM$ is a multiplier that adjusts the impact of the second derivative term. 

The inclusion of $K_{dd}$ helps the controller anticipate changes in the error more effectively. By accounting for the acceleration of the error (i.e., how the rate of error change is itself changing), this additional term enhances the damping characteristics of the controller, allowing for more refined control responses, especially in systems that are subject to rapid changes or oscillations.

### Code Structure

#### Data Structures

- **Data**: Holds the time difference (`dt`) and error value (`e`).
- **Node**: Represents an element in the linked list, containing `Data` and a pointer to the previous node.
- **List**: Manages the linked list, holding pointers to the head and tail nodes and the size of the list.

#### PID Class

The `PID` class encapsulates the PID control logic. It includes:

- **Member Variables**:
  - Control gains: `Kp`, `Ki`, `Kd`, `Kdd` (additional derivative term).
  - State variables: `preE`, `preDv`, `intg`, `preOut`, and `preT` for storing previous error, derivative, integral term, last output, and last timestamp, respectively.
  - `dtXs`: An instance of `List` to maintain historical error data.

- **Constructor**: Initializes the PID parameters and state variables.

- **Update Method**:
  - Takes the current error and timestamp, computes the control output, and updates the internal state.
  - Implements logic for calculating the adaptive gains, managing error history, and generating the control signal.

## testCar.hpp

### Overview

This header file includes a `Car` class for simulating the motion of a car, a PID controller for adjusting the speed of an auto-following kart based on the distance to a leader car, and several utility functions for data processing and results management.

### Car Class

The `Car` class represents a simple vehicle model with basic motion dynamics. It includes attributes for speed and position, as well as methods to update these attributes.

### PID Debugging Function

The `pidDebug` function is a utility for logging messages related to PID control operations.

### Sum of Last Squared Values
```cpp
double sum_last_squared(const std::vector<double> &v, double prop)
```
Calculates the sum of squares of the last `n * prop` elements of the provided vector `v`.

### Velocity Generation
```cpp
vector<double> velocities(int steps)
```
Generates a vector of velocity values over a specified number of simulation steps, simulating different phases of motion (acceleration, deceleration, and adjustments).

### PID Testing Function
```cpp
vector<double> test(double maxIntTm, double maxAmp, double minKp, double maxKp, double rTiM, double TdM, double TddM, double eDPm, double eDPa, unsigned long session, double Kp, double preE = 0, unsigned long preT = 0, double timeInterval = 10, int steps = 1000)
```
This function simulates the interaction between a leader car and an auto-following kart using a PID controller to adjust the follower's speed based on the distance to the leader.

- **Parameters**:
  - `double maxIntTm`: Maximum integral time constant.
  - `double maxAmp`: Maximum amplitude of control output.
  - `double minKp`: Minimum proportional gain.
  - `double maxKp`: Maximum proportional gain.
  - `double rTiM`: Reference time interval multiplier.
  - `double TdM`: Derivative time multiplier.
  - `double TddM`: Second derivative time multiplier.
  - `double eDPm`: Maximum error derivative proportionality factor.
  - `double eDPa`: Error derivative adjustment factor.
  - `unsigned long session`: Session identifier for logging purposes.
  - `double Kp`: Initial proportional gain.
  - `double preE`: Previous error value (optional, defaults to 0).
  - `unsigned long preT`: Previous time value (optional, defaults to 0).
  - `double timeInterval`: Time interval for each step of simulation (default is 10).
  - `int steps`: Total number of simulation steps (default is 1000).
  
- **Returns**: A vector containing the distances between the leader and follower over time.

### PID Test Data Structure
```cpp
struct pidTest
{
	double maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa;
	unsigned long session;
	double Kp, result;
};
```
The `pidTest` structure is used to store parameters for PID testing, along with the result of the simulation.

### Result Writing Function
```cpp
bool write_results(const vector<pidTest> &data, const string &filename)
```
Writes the results of PID tests to a CSV file for analysis.

### `ParameterRange`

A structure to define the range of parameters for optimization.

- **Attributes**:
  - `double start`: The starting value of the parameter range.
  - `double end`: The ending value of the parameter range.
  - `double step`: The increment step for the parameter.

## optimization.hpp

This header file defines the optimization functions and structures used to optimize the parameters of a PID controller in a car simulation without multi-threaded processing.

### `int optimize(vector<ParameterRange> ranges)`

Optimizes the parameters of the PID controller based on the provided ranges and writes results to a CSV file.

- **Parameters**:
  - `vector<ParameterRange> ranges`: A vector containing the ranges of parameters to optimize.
  
- **Returns**: 
  - `int`: Returns `EXIT_SUCCESS` (0) if optimization completes successfully, otherwise returns `EXIT_FAILURE`.

## optimization_multithread.hpp

This header file defines the optimization functions and structures used to optimize the parameters of a PID controller in a car simulation with multi-threaded processing.

### `int optimize(vector<ParameterRange> ranges)`

Optimizes the parameters of the PID controller based on the provided ranges and writes results to a CSV file.

- **Parameters**:
  - `vector<ParameterRange> ranges`: A vector containing the ranges of parameters to optimize.
  
- **Returns**: 
  - `int`: Returns `EXIT_SUCCESS` (0) if optimization completes successfully, otherwise returns `EXIT_FAILURE`.

## main.cpp

The code file demonstrates the usage of the `optimization.hpp` (or `optimization_multithread.hpp`) header file to perform parameter optimization for a PID controller. It defines a set of parameter ranges and calls the `optimize` function to evaluate different combinations of parameters.

Note that steps can't be zero or the program won't stop.

## main.ino

This Arduino code is for controlling a differential-drive robot using two ultrasonic sensors to measure distance, a PID (Proportional-Integral-Derivative) controller for speed and direction adjustments, and an L293D motor driver to control the motors based on sensor inputs. 

### Pin Definitions
- **L293D motor control pins:**
  - `L293D_LEFT_EN` and `L293D_RIGHT_EN`: Enable pins for the left and right motors.
  - `L293D_LEFT_IN1`, `L293D_LEFT_IN2`, `L293D_RIGHT_IN1`, `L293D_RIGHT_IN2`: Input pins for controlling the direction of the motors.
  
- **Ultrasonic sensor pins:**
  - `TRIG_LEFT` and `TRIG_RIGHT`: Trigger pins for the left and right ultrasonic sensors.
  - `ECHO_LEFT` and `ECHO_RIGHT`: Echo pins for the left and right ultrasonic sensors that return the duration of the reflected sound wave.

### Constants
- **`HALF_SOUND_SPEED`**: Constant used to convert the echo time into a distance. The value `0.1715` is derived from the speed of sound.
- **`TARGET_DISTANCE`**: The target distance that the robot should maintain between the two sensors (in centimeters).
- **`DISTANCE_BETWEEN_ULTRASONIC_SENSORS`**: The distance between the two ultrasonic sensors mounted on the robot, which is used to calculate angular adjustments.

### PID Tuning Parameters
- **PID Controllers (`avgPID` and `angPID`)**: Two PID controllers are used:
  - `avgPID`: Controls the average speed to keep the robot at the target distance.
  - `angPID`: Controls the angular speed to maintain proper alignment based on the sensor readings.
  
  The values provided for the PID controllers (like `461.9`, `517.9`, etc.) are the tuning parameters for each PID term (P, I, D).

### Functions and Code Flow

- **`leftIn()` and `rightIn()`**: These inline functions trigger the ultrasonic sensors by sending a pulse and measuring the time it takes for the sound to return. The return time is converted into a distance using the formula `distance = time * speed of sound / 2`. The resulting distance is then returned.

- **`setup()`**: This function runs once when the program starts. It initializes serial communication, sets pin modes for the motor control and ultrasonic sensors, and initializes the velocity variables (`avgV` and `angV`).

- **`loop()`**: This function runs repeatedly. It does the following:
  1. Reads the distance from the left and right ultrasonic sensors using `leftIn()` and `rightIn()`.
  2. Calculates the average distance (`avg`) and the angular difference (`ang`) between the two sensors.
  3. Applies the PID controllers to adjust the average speed and angular speed to maintain the target distance.
  4. Sets the left and right velocities (`leftV` and `rightV`) based on the PID updates and applies any necessary adjustments to control the motors.
  5. Sends the control signals to the motors using `leftOut()` and `rightOut()`.

### Motor Control (`leftOut()` and `rightOut()`):
- **`leftOut()` and `rightOut()`**: These inline functions are responsible for sending control signals to the motor driver (L293D). The functions adjust the speed based on the calculated velocity and apply any speed multipliers to tune motor behavior. The `analogWrite()` function is used to set the motor speeds, where:
  - Positive velocities drive the motors forward.
  - Negative velocities drive the motors backward.

### Debugging
- If `DEBUG` is set to `1`, additional information about the PID updates and motor velocities is printed to the serial monitor for troubleshooting and performance analysis.


## Wooden Board

The wooden board serves as the support structure of the auto-following kart that connects and props the components such as an Arduino board, an Omni wheel, a breadboard, 2 wheels, and 2 motors.

The 3D design file is [assets/board.step](assets/board.step). 

Here's the photo of the cut wooden board:

![assets/board.jpg](assets/board.jpg)

## Inspiration and Background

The development of this PID (Proportional-Integral-Derivative) controller project is rooted in control theory, a field of engineering that focuses on the behavior of dynamic systems. What makes us take the initiative is the auto-following kart lesson in our living technology class. We aim to make an auto-following kart from scratch with Arduino and wooden boards and make it as quality as possible on a constrained budget.

### Control Theory and PID Controllers

The development of this PID (Proportional-Integral-Derivative) controller project is rooted in control theory, a field of engineering that focuses on the behavior of dynamic systems. PID controllers are a cornerstone of modern control systems. They are designed to automatically adjust system outputs based on feedback to minimize error, allowing for precise control of dynamic systems. The PID controller operates based on three fundamental components:

1. **Proportional Control (P)**: Responds proportionally to the current error value, providing immediate correction based on how far the system is from the desired state.

2. **Integral Control (I)**: Addresses accumulated past errors, ensuring that the system reaches the desired state even if there are persistent biases.

3. **Derivative Control (D)**: Predicts future errors based on the rate of change of the error, providing a damping effect to reduce overshoot and oscillation.

The tuning of these parameters is crucial for achieving optimal performance. The traditional Ziegler–Nichols tuning method has served as a foundational approach for many engineers, providing empirical guidelines for setting PID parameters based on system behavior.

### Arduino Scenarios in Living Technology Classes

Arduino boards offer an accessible platform for students and hobbyists to experiment with hardware and software integration, facilitating hands-on learning experiences in technology classes.

In living technology classes, we engage with Arduino to build auto-following kart projects that require real-time control systems. This scenario highlights the importance of tuning PID controllers to achieve desired behaviors in robotics and automotive systems.

This PID controller serves as an imperative part of our auto-following kart. We use it to achieve a stable distance between the auto-following kart and the leader kart, in our case, a cleaning robot.

## Development History

The development of the PID controller and its associated simulation framework has been an iterative process, marked by continuous enhancement and refinement.

### Initial Implementation

The journey began with a simple implementation of the traditional PID controller. The focus was on creating a basic class structure in C++ that encapsulated the PID logic, allowing for straightforward application in simulations.

- **Key Features**:
  - Basic PID calculations.
  - Initial support for both Arduino and non-Arduino environments through conditional compilation.

### Enhancements and Adaptive Features

As the initial implementation was tested, several limitations became apparent, particularly regarding responsiveness and stability. This prompted the incorporation of advanced features such as:

- **Adaptive Gain Adjustment**: Inspired by the need for dynamic responsiveness, the proportional gain $K_p$ was made adaptable based on the error's rate of change. This enhancement allowed for better handling of transient behaviors in the system.

- **Adaptive Integral Term**: To mitigate integral windup, the integral gain $K_i$ was adjusted based on observed error amplitude. This feature improved system stability during sustained error conditions.

### Error History Management

To accurately compute the integral and derivative terms, a robust error history management system was implemented. This system utilized a linked list structure to store past error values and timestamps, enabling precise calculations for both the integral and derivative components. The integration of error history management was crucial for implementing more advanced tuning strategies, inspired by established control theory practices.

### Additional Derivative Term

Recognizing the need for improved damping characteristics, an additional derivative term $K_{dd}$ was introduced. This term accounted for the acceleration of error changes, allowing the controller to anticipate and react to rapid fluctuations more effectively.

### Testing and Simulation Framework

With the PID logic in place, attention turned to building a comprehensive simulation framework to test various PID configurations. The introduction of the `Car` class for simulating follower and leader vehicles added a practical context for evaluating PID performance in a dynamic environment.

- **PID Testing Function**: A dedicated function was developed to simulate the interaction between the leader and follower karts, providing insights into how different PID parameters affected system behavior.

### Optimization and Multi-threading

To facilitate further exploration of PID parameter tuning, optimization functions were integrated. Utilizing multi-threading allowed for the concurrent evaluation of multiple parameter combinations, greatly enhancing the testing efficiency. The optimization framework also included mechanisms for result logging and progress tracking, making it easier to analyze and visualize outcomes.

### Arduino Auto-Following Kart

We use two ultrasonic sensors to measure the distance on both sides. The difference in distance helps adjust the robot's angular velocity, while the average of both distances adjusts its linear speed. The PID controllers fine-tune these adjustments to ensure the robot stays at the target distance, while adjusting its orientation to keep the two sensors aligned. The L293D motor driver uses these velocity values to control the left and right motors, adjusting the robot's movement accordingly.

## Contributing

Contributions are welcome! If you have ideas for improvements or optimizations, please fork the repository and create a pull request.

## License

This project is licensed under the AGPL License. See the [LICENSE.md](LICENSE.md) file for details.
