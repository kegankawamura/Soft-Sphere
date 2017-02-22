#include <DCMotorControl.h>


double xD = 0.1; //desired position
double vD = 0; //desired velocity
double aD = 0;

  //define pins used 
  const int sensorPin = A0;
  const int pwmPin = 3;
  const int motorFwd = 5;
  const int motorBck = 6;
  DCMotorControl motor(pwmPin, motorFwd, motorBck);
  
  
  double u = 0;
  double dt = 0.01; //TDescribes the delay between each control update. 
  unsigned long start = millis();
  // Define state space matrices
  double X[2] = {0, 0};
  double A[2][2] = {{1, dt},
                    {0, 1}
  };
  double B[2] = {pow(dt,2) / 2, (0.0288111 * 0.015) * dt}; // calculated from stall torque * spool radius (may not be correct9)
  double C[2] = {1, 0};
  double L[2] = {1.25, 50};    //found with K_finder.py
  double x_p[2]; //predicted state variable

  //for use with L(t), V(t), and a(t) function: referenceUpdate
  double L_t;
  double w_t;
  double alpha_t;
  const unsigned long progBegin = millis();

void setup() {
  Serial.begin(9600);
  initialState();
}

void loop() {
  statePredict(u);
  double distanceMeasurment = irToDist(analogRead(sensorPin));
  stateUpdate(distanceMeasurment);
  controlUpdate();
  start = millis();
  motor.driveMotor(u);

  //debyg
  Serial.println(X[0]);

}






/**************************************************************
***************************************************************/

/*
   Guesses initial state. Intial velcity assumed to be zero here.
   This allows the system to know where it started based on sensor readings. 
*/
  void initialState() {
    X[0] = irToDist(analogRead(sensorPin));
    X[1] = 0.0;
  }



/*
   Predicts state based on state space equations (no sensor data considered).
   First calculates time step since last motor actuation, then uses state space
   matrix eqauation to predict system movement.
*/
  void statePredict(double u) {
    dt = (start - millis()) / 1000; //calculates the time since the motor actuation was last change
    A[0][1] = dt;        //update state space matrices with new value of dt
    B[0] = pow(dt,2) / 2;
    B[1] = (0.0288111 * 0.015) * dt;
    
    x_p[0] = (A[0][0] * X[0]) + (A[0][1] * X[1]) + B[0] * u;
    x_p[1] = (A[1][0] * X[0]) + (A[1][1] * X[1]) + B[1] * u;
  
                                     //debug
                                     //Serial.println(x_p[1]);
                                     //Serial.println(u);
  }



/*
    Updates state with Kalman Filter algorithm
*/
  void stateUpdate(double distanceMeas) {
    X[0] = x_p[0] + L[0] * (distanceMeas - x_p[0]);
    X[1] = x_p[1] + L[1] * (distanceMeas - x_p[0]);
  }



/*
   Converts ir sensor data to distances in cm. Polynomial coefficients found by performing
   a least squares fit in Matlab. 
*/
  double irToDist(double irRead) {
    double irVoltage = irRead * (5.0 / 1023.0); //convert ir data from analogRead value to voltage
    const double x3Term = -3.3183; //polynomial term for x^3
    const double x2Term = 20.9721;
    const double x1Term = -46.8391;
    const double x0Term = 42.5554;
    
    double dist = x3Term * pow(irVoltage, 3) + x2Term * pow(irVoltage, 2) + x1Term * irVoltage + x0Term;
    dist = dist / 100.0; //convert to m
    return dist;
  }


/*
 * Updates control input based on feedback law. 
 */
  void controlUpdate() {
    const double zeta = 0.7;
    const double omegaN = 10;
    //double uEQ = 9; //equilibrium value for input. (input to counter force of gravity in this case)
    u = (-2 * zeta * omegaN * (X[2]) - w_t) - (pow(omegaN, 2) * (X[0] - L_t)) + alpha_t; //updates control command
  }


/*
 * Updated reference position and velocity based on function found to keep controller rotating
 * in circle. n
 * 
 * Calculates the position, velocity, and acceleration based on the analytical funcitons found that will keep controller in same
 * position as the ball rotates around. 
 * 
 * Note that this function should be changed to have whatever reference you want the
 * controller to follow. 
 */
  void refUpdate() {
    const double R = 0.11;
    const double r = R/2;
    const double R_2 = pow(R, 2);
    const double r_2 = pow(r, 2);
    const double omega = 5; //desired angular speed of ball
    const double omega_2 = pow(omega, 2);

    float currT = (millis() - progBegin)/ 1000.0; // current time in seconds
    L_t = sqrt( pow((R*cos(omega*currT)) ,2)  + pow((R*sin(omega*currT)) ,2));
    w_t = 2*R_2*omega*sin(currT * omega) - 2*R*omega*sin(currT * omega) * (R*cos(currT * omega) - r) / (2 * sqrt(pow(R*cos(currT*omega) - r, 2))  + R_2*pow(currT*omega ,2) );
    alpha_t = r*R*omega_2 * abs(sin(currT * omega)) / ((R*cos(currT * omega) - R) * abs(r - R*cos(currT * omega)));
  }
