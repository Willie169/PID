# PID Control System

## Overview

This repository implements a PID (Proportional-Integral-Derivative) control system using C++. The PID controller is a widely used control loop feedback mechanism that is effective in various control applications. The code simulates a simple car-following scenario where a follower car adjusts its speed based on the distance to a leader car.

## File Structure

The project consists of the following primary files:

1. **PID.hpp**: Contains the PID controller class definition and associated data structures.
2. **testCar.hpp**: Implements the `Car` class and the testing framework to evaluate the PID controller's performance.
3. **optimization.cpp**: The main execution file that runs the tests and outputs results to a CSV file.

## PID.hpp

### Header Guard

```cpp
#ifndef PID_HPP
#define PID_HPP
```
This prevents multiple inclusions of the header file.

### Includes

- **Arduino Compatibility**: The code checks for the `ARDUINO` macro to include the appropriate libraries for Arduino or standard C++.
- **Math and String Utilities**: Includes standard math and string libraries to support various operations.

### Macros

The following macros are defined for utility functions:
- `MAX(a, b)`: Returns the maximum of two values.
- `MIN(a, b)`: Returns the minimum of two values.
- `ABS(a)`: Returns the absolute value of a number.
- `COPYSIGN(a, b)`: Returns the value of `a` with the sign of `b`.

### Data Structures

1. **Data Struct**: Holds the time difference (`dt`) and error value (`e`).
   ```cpp
   struct Data {
       unsigned long dt;
       double e;
   };
   ```

2. **Node Struct**: Represents a node in a linked list, containing `Data` and a pointer to the previous node.
   ```cpp
   struct Node {
       Data data;
       Node* prev;
   };
   ```

3. **List Struct**: A linked list structure to manage multiple `Data` entries.
   ```cpp
   struct List {
       Node* head;
       Node* tail;
       unsigned int size;
   };
   ```

### PID Class

The main class implementing the PID control algorithm.
- **Member Variables**:
  - Constants defining PID parameters and limits.
  - Previous error and derivative values, integral value, and output.
  - A linked list to store time and error data.

- **Constructor**: Initializes PID parameters.
  
- **Methods**:
  - `update(double e, unsigned long timestamp, String* debug = nullptr)`: Updates the PID controller with the current error and timestamp. It computes the PID output based on the provided error, adjusting the integral and derivative terms as needed. Debug information can be outputted if requested.

### Usage Example
```cpp
PID pidController(maxIntTm, maxAmp, minKp, maxKp, rTiM, TdM, TddM, eDPm, eDPa, session, Kp);
double output = pidController.update(currentError, currentTimestamp);
```

## testCar.hpp

### Car Class

Represents a car in the simulation with attributes for speed and position.
- **Methods**:
  - `updatePosition(double timeInterval)`: Updates the car's position based on its speed and a given time interval.
  - `changeSpeed(double newSpeed)`: Sets the car's speed.
  - Getters for position and speed.

### Testing Framework

- **pidDebug**: Outputs debug messages to the console.
  
- **test**: Simulates the interaction between a leader and follower car, using the PID controller to adjust the follower's speed based on the distance to the leader. It returns a vector of distance values over time.

- **average_last**: Computes the average of the last `n` values in a vector.

## optimization.cpp

### Functionality

The main file executes the PID control simulation by:
- Defining various test parameters.
- Running multiple simulations with different PID parameters.
- Storing results for later analysis.

### Results Output

Results are sorted and written to a CSV file for easy analysis.

### Example Usage
To run the tests, compile the `optimization.cpp` and execute it. The results will be saved in the specified output file.

## LICENCE

This repository is licensed under AGPL license.
