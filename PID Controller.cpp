#include <iostream>
#include <algorithm>    // For std::clamp

class PID {
private:
    // PID coefficients
    double kp; // proportional gain
    double ki; // integral gain
    double kd; // derivative gain

    // PID state variables
    double integral;
    double previous_error;
    double output_min; // minimum output limit
    double output_max; // maximum output limit

    // derivative filtering
    double last_derivative;
    double alpha; // smoothing factor for derivative filter

public:
    // constructor
    PID(double kp, double ki, double kd, double output_min, double output_max)
        : kp(kp), ki(ki), kd(kd), integral(0.0), previous_error(0.0), output_min(output_min), output_max(output_max),
          last_derivative(0.0), alpha(0.8) {
            // initialize any other variables if necessary
          }

    // method to calculate the control output
    double calculate(double setpoint, double measured_value, double dt) {
        // calculate error
        double error = setpoint - measured_value;

        // calculate proportional term
        double Pout = kp * error;

        // calculate integral term with anti-windup
        if (output_min < Pout && output_max > Pout) {
            integral += error * dt;
        }
        double Iout = ki * integral;

        // update the last derivative value if using filtering
        double derivative = (error - previous_error) / dt;
        derivative = alpha * last_derivative + (1.0 - alpha) * derivative;
        double Dout = kd * derivative;
        last_derivative = derivative; // placeholder (to be improved)

        double output = Pout + Iout + Dout;

        // clamp output to min and max limits
        output = std::clamp(output, output_min, output_max);

        // update previous error
        previous_error = error;

        return output;
    }

    // method to reset PID terms
    void reset() {
        integral = 0.0;
        previous_error = 0.0;
        last_derivative = 0.0; 
    }
};

int main() {
    // PID controller parameterrs
    double kp = 1.0, ki = 0.1, kd = 0.01;
    double output_min = -10.0;  // example min output
    double output_max = 10.0;   // example max output

    // create pid controller
    PID pid(kp, ki, kd, output_min, output_max);

    // example setpoint and measured value
    double setpoint = 100.0; 
    double measured_value = 90.0;
    double dt = 0.1; // timestep

    // calculate PID output
    double output = pid.calculate(setpoint, measured_value, dt);

    std::cout << "PID output: " << output << std::endl;

    return 0;
}
