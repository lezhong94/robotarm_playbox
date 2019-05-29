// ME5245 final project - simulink robot arm with arduino control
// programmed using position control
//
// Group G
// Group member: Lezhong Gao, Zikun Yu, Chunpeng Wang, Jinjie Tong

#include <Wire.h>
#include <stdio.h>
#include <math.h>

#define XMID 528 // middle position of joystick x direction
#define YMID 516 // middle position of joystick z direction
#define XYLIMIT 1024
#define XYSTEP 3
// These 4 parameters are used to tune the input p_ref.x and p_ref.y as a function of joystick input

#define RESOLUTION 96 // resolution of encoder
#define AMP1 10.0 // joint angle amplification
#define AMP2 10.0
#define AMP3 10.0
#define KP1 6.0
#define KP2 6.0
#define KP3 6.0
#define KD 0.1
#define KI 0.0
// These 4 parameters needs to be set for specific simulink model

#define ALPHA 0.1

#define XLIMITLOW -0.2233
#define XLIMITUP 0.2539
#define YLIMITLOW -0.2427
#define YLIMITUP 0.2427
#define ZLIMITLOW 0.09857
#define ZLIMITUP 0.1662

const int JoyStick_X = 0; //x
const int JoyStick_Y = 1; //y
const int JoyStick_Z = 4; //key
const int Button_Z_Up = 5; //z+
const int Button_Z_Down = 6; //z-

double L1 = 0.10414, L2 = 0.10795, L3 = 0.03429, D1 = 0.132195; // link length (m)

unsigned long currentTime;
unsigned long lastTime = 0;

struct Joint
{
    double theta1, theta2, theta3; // joint space param (rad)
};
Joint j_output = {PI, PI, PI};
Joint j_output_last = {0.0, 0.0, 0.0};
Joint j_control = {PI, PI, PI};
Joint j_control_last = {0.0, 0.0, 0.0};

struct Velocity
{
    double v1, v2, v3; // joint velocity param (rad/s)
};
Velocity v_output = {0.0, 0.0, 0.0};
Velocity v_output_last = {0.0, 0.0, 0.0};
Velocity v_control = {0.0, 0.0, 0.0};
Velocity v_control_last = {0.0, 0.0, 0.0};

struct Encoder
{
  int enc1, enc2, enc3;
};
Encoder encoder = {0, 0, 0};

struct Position
{
  double x, y, z;
};
//Position p_ref = {(XLIMITLOW + XLIMITUP) / 2, (YLIMITLOW + YLIMITUP) / 2, (ZLIMITLOW + ZLIMITUP) / 2};
//Position p_ref = {0, L1+L2+L3, D1 };
Position p_ref = {0.0, 0.0, 0.0 };
Position p_output = {0.0, 0.0, 0.0};

struct Torque
{
  double t1, t2, t3;
};
Torque t_input = {0.0, 0.0, 0.0};
Torque t_input_integral = {0.0, 0.0, 0.0};

// joint angle to x,y,z
void FWKsolver (double theta1, double theta2, double theta3)
{
  double X1, Y1, Z1;
  X1 = L1 * cos(theta1) + (L2 + L3 * cos(theta3)) * cos(theta1 + theta2);
  Y1 = L1 * sin(theta1) + (L2 + L3 * cos(theta3)) * sin(theta1 + theta2);
  Z1 = D1 + L3 * sin(theta3);
  p_output.x = X1;
  p_output.y = Y1;
  p_output.z = Z1;
}

// x,y,z to joint angle
void IVKsolver (double X, double Y, double Z)
{
  double X1 = X, Y1 = Y, Z1 = Z; // end-effector coord for kinematics (m)
  double psi = 0.0, phi = 0.0; 
  double a = 0.0;
  if(!isnan(asin(Z1 / L3))) j_control.theta3 = asin(Z1 / L3);
  a = sqrt(pow(X1, 2) + pow(Y1, 2));
  phi = acos((-pow(a, 2) - pow(L1, 2) + pow(L2+L3*cos(j_control.theta3), 2)) / (-2 * L1 * a));
  psi = atan(Y1 / X1);
  if((!isnan(phi)) && (!isnan(psi))) j_control.theta1 = psi - phi;
  if(!isnan(atan((a * sin(phi)) / (a * cos(phi) - L1)))) j_control.theta2 = atan((a * sin(phi)) / (a * cos(phi) - L1));
}

// encoder reading to joint output
void encToJoint(void)
{
  j_output.theta1 = encoder.enc1 * 2.0 * PI / RESOLUTION + PI;
  j_output.theta2 = encoder.enc2 * 2.0 * PI / RESOLUTION + PI;
  j_output.theta3 = encoder.enc3 * 2.0 * PI / RESOLUTION + PI;
}

// get position from joystick
void getPos()
{
  /*
  p_ref.x = p_ref.x + ((analogRead(JoyStick_X) - XMID) * XYSTEP / XYLIMIT) / 100.0;
  p_ref.x = min(p_ref.x, XLIMITUP);
  p_ref.x = max(p_ref.x, XLIMITLOW);
  p_ref.y = p_ref.y + ((analogRead(JoyStick_Y) - YMID) * XYSTEP / XYLIMIT) / 100.0;
  p_ref.y = min(p_ref.y, YLIMITUP);
  p_ref.y = max(p_ref.y, YLIMITLOW);
  if(digitalRead(Button_Z_Up)) p_ref.z = p_ref.z + 1.0 / 100;
  else if(digitalRead(Button_Z_Down)) p_ref.z = p_ref.z - 1.0 / 100;
  p_ref.z = min(p_ref.z, ZLIMITUP);
  p_ref.z = max(p_ref.z, ZLIMITLOW);*/
  j_control.theta1 += ((analogRead(JoyStick_X) - XMID) * XYSTEP / XYLIMIT) / 100.0;
  j_control.theta1 = max(j_control.theta1, 0);
  j_control.theta1 = min(j_control.theta1, 2*PI);
  j_control.theta2 += ((analogRead(JoyStick_Y) - YMID) * XYSTEP / XYLIMIT) / 100.0;
  j_control.theta2 = max(j_control.theta2, 0);
  j_control.theta2 = min(j_control.theta2, 2*PI);
  if(digitalRead(Button_Z_Up)) j_control.theta3 += 1.0/100;
  else if(digitalRead(Button_Z_Down)) j_control.theta3 += -1.0/100;
  j_control.theta3 = max(j_control.theta3, 0);
  j_control.theta3 = min(j_control.theta3, 2*PI);
}

// PID controller
void controller (double error1, double error2, double error3, unsigned long t)
{
  double delta_t = double(t);
  //t_input_integral.t1 = t_input_integral.t1 + KI * error1 * delta_t;
  t_input.t1 = KP1 * error1 + KD * (v_control.v1 - v_output.v1) + t_input_integral.t1;
  //t_input_integral.t2 = t_input_integral.t2 + KI * error2 * delta_t;
  t_input.t2 = KP2 * error2 + KD * (v_control.v2 - v_output.v2) + t_input_integral.t2;
  //t_input_integral.t3 = t_input_integral.t3 + KI * error3 * delta_t;
  t_input.t3 = KP3 * error3 + KD * (v_control.v3 - v_output.v3) + t_input_integral.t3;
}

void outputVelocityCalculator (double error1, double error2, double error3, double delta_t)
{
  v_output.v1 = (1.0 - ALPHA) * v_output_last.v1 + ALPHA * error1 / delta_t;
  v_output.v2 = (1.0 - ALPHA) * v_output_last.v2 + ALPHA * error2 / delta_t;
  v_output.v3 = (1.0 - ALPHA) * v_output_last.v3 + ALPHA * error3 / delta_t;
  v_output_last.v1 = v_output.v1;
  v_output_last.v2 = v_output.v2;
  v_output_last.v3 = v_output.v3;
}

void controlVelocityCalculator (double error1, double error2, double error3, double delta_t)
{
  v_control.v1 = (1 - ALPHA) * v_control_last.v1 + ALPHA * error1 / delta_t;
  v_control.v2 = (1 - ALPHA) * v_control_last.v2 + ALPHA * error2 / delta_t;
  v_control.v3 = (1 - ALPHA) * v_control_last.v3 + ALPHA * error3 / delta_t;
  v_control_last.v1 = v_control.v1;
  v_control_last.v2 = v_control.v2;
  v_control_last.v3 = v_control.v3;
}

void setup() 
{
  pinMode(JoyStick_Z, INPUT);
  pinMode(Button_Z_Up, INPUT);
  pinMode(Button_Z_Down, INPUT); 
  Serial.begin(9600);
  Wire.begin(10);                // join i2c bus with address #10
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent);
  currentTime = millis();
  delay(100);
}

void loop() 
{
  encToJoint();
  getPos();
  //IVKsolver(p_ref.x, p_ref.y, p_ref.z);
  lastTime = currentTime;
  currentTime = millis();
  outputVelocityCalculator(j_output.theta1 - j_output_last.theta1, j_output.theta2 - j_output_last.theta2, j_output.theta3 - j_output_last.theta3, double(currentTime - lastTime) / 1000.0);
  controlVelocityCalculator(j_control.theta1 - j_control_last.theta1, j_control.theta2 - j_control_last.theta2, j_control.theta3 - j_control_last.theta3, double(currentTime - lastTime) / 1000.0);
  j_output_last.theta1 = j_output.theta1;
  j_output_last.theta2 = j_output.theta2;
  j_output_last.theta3 = j_output.theta3;
  j_control_last.theta1 = j_control.theta1;
  j_control_last.theta2 = j_control.theta2;
  j_control_last.theta3 = j_control.theta3;
  controller(j_control.theta1 - j_output.theta1, j_control.theta2 - j_output.theta2, j_control.theta3 - j_output.theta3, double(currentTime - lastTime) / 1000.0);
  delay(10);
  Serial.print(v_control.v1); Serial.print(","); Serial.print(v_control.v2); Serial.print(","); Serial.print(v_control.v3); Serial.print(",");
  //Serial.print(encoder.enc1); Serial.print(","); Serial.print(encoder.enc2); Serial.print(","); Serial.print(encoder.enc3); Serial.print(",");
  //Serial.print(j_output.theta1); Serial.print(","); Serial.print(j_output.theta2); Serial.print(","); Serial.print(j_output.theta3); Serial.print(",");
  Serial.print(j_control.theta1); Serial.print(","); Serial.print(j_control.theta2); Serial.print(","); Serial.println(j_control.theta3);
  //Serial.print(int(t_input.t1 * AMP1)); Serial.print(","); Serial.print(int(t_input.t2 * AMP2)); Serial.print(","); Serial.println(int(t_input.t3 * AMP3));
}

// function that executes whenever data is requested by master
void requestEvent() {
  Wire.write(int(t_input.t1 * AMP1));
  Wire.write(int(t_input.t2 * AMP2));
  Wire.write(int(t_input.t3 * AMP3));
}

void receiveEvent(int num) {
  long c, c1, c2, c3;
  byte r1 = Wire.read();
  byte r2 = Wire.read();
  byte r3 = Wire.read();
  byte r4 = Wire.read();
  c = long(r4)*256*65536 + long(r3)*65536 + long(r2)*256 + long(r1);
  c1 = c/10000;
  c2 = (c - c1*10000)/100;
  c3 = c - c1*10000 - c2*100;
  encoder.enc1 = int(c1) - 48;
  encoder.enc2 = int(c2) - 48;
  encoder.enc3 = int(c3) - 48;
}