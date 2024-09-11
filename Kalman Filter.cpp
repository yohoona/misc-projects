#include <iostream>
#include <vector>


using namespace std;


// Function to simulate measurements (with noise)
vector<double> simulateMeasurements(const vector<double>& truePositions, double measurementNoise) {
    vector<double> measurements;
    for (const auto& position : truePositions) {
        double noisyMeasurement = position + ((rand() % 1000) / 1000.0 - 0.5) * measurementNoise;
        measurements.push_back(noisyMeasurement);
    }
    return measurements;
}


int main() {
    double x_hat = 0;       // initial state estimate
    double P = 1;           // initial estimate covariance
    double Q = 0.1;           // process noise variance
    double R = 1;           // measurement noise variance
    double v = 1;           // constant velocity
    double dt = 1;          // time step duration


    // True positions (for simulation purposes)
    vector<double> truePositions = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    vector<double> measurements = simulateMeasurements(truePositions, R);


    // Kalman Filter Implementation
    for (size_t k = 0; k < measurements.size(); ++k) {
        // Prediction step
        double x_hat_plus = x_hat + v * dt; // predicted state
        double P_plus = P + Q; // predicted covariance
       
        // Measurement update step
        double K_plus = P_plus / (P_plus + R);              // Kalman Gain
        x_hat_plus = x_hat_plus + K_plus * (measurements[k + 1] - x_hat_plus);                 // updated state
        P_plus = (1 - K_plus) * P_plus;                     // updated covariance


        // Update for next iteration
        x_hat = x_hat_plus;
        P = P_plus;


        // Print results
        cout << "Time step: " << k << ", Measurement: " << measurements[k] << ", Estimate: " << x_hat << endl;
    }


    return 0;
}
