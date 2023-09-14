#ifndef SINGLE_KALMAN_H_
#define SINGLE_KALMAN_H_

class SingleKalmanFilter {

  /**
   * Create 1-dimensional kalman filter
   * @param  {double} R -> processNoise
   * @param  {double} Q -> measurementNoise
   * @param  {double} A -> stateVector
   * @param  {double} B -> controlVector
   * @param  {double} C -> measurementVector
   * @return {KalmanFilter}
   */

#define NULL '\0'
private:
  double processNoise;
  double measurementNoise; // variance
  double stateVector;
  double controlVector;
  double measurementVector;
  double cov;
  double estimatedSignal; // x

public:
  SingleKalmanFilter(double R, double Q, double A, double B, double C);

  /**
   * Filter a new value
   * @param  {double} z newSignal Measurement
   * @param  {double} u Control
   * @return {double} estimatedSignal
   */
  double filter(double newSignal, double u);

  /**
   * Return the last filtered measurement
   * @return {Number}
   */
  double lastMeasurement();

  /**
   * Set measurement noise Q
   * @param {double} noise
   */
  void setMeasurementNoise(double noise);

  /**
   * Set the process noise R
   * @param {double} noise
   */
  void setProcessNoise(double noise);
  /**
   * Get measurement noise Q
   * @return {double} noise
   */
  double getMeasurementNoise(void);
  /**
   * Get the process noise R
   * @return {double} noise
   */
  double getProcessNoise(void);
};
#endif // KALMAN_H_