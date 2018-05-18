/*
 * Ronald Rey Lovera (reyronald@gmail.com)
 * Smailyn Peña Núñez (smailyn.p@gmail.com)
 * Última modificación: Viernes 30 de agosto de 2013
 * Rover bot, Hands-On Competition, Intec
 * Versión Final
 * Ingeniería Mecatrónica
 */

#include <Servo.h>
#include <QTRSensors.h>
#include <stdio.h>
#include <math.h>

// Represents a duty cycle of 100%
//#define MAX_SPEED 255
const int MAX_SPEED = 255;
const int REDUCED_SPEED = 40;

// Output pins for motors
#define RIGHT_MOTOR_FWD 8
#define RIGHT_MOTOR_RV  7
#define LEFT_MOTOR_RV   9
#define LEFT_MOTOR_FWD  10
#define BIAS            20

// Servo
#define SERVO_PIN 3
#define OPEN      90
#define CLOSED    45
Servo gripper;

// Box management logic variables
#define SAMPLE_AMOUNT        10
#define SENSOR_THRESHOLD     (1<<10)/2
boolean stopReached  = false;
boolean boxGrabbed   = false;
boolean whiteBox     = false;
boolean directions[] = {};
int directinosCount  = 0;

// Constants for the PID Control
float Kp = 125, Kd = 14, Ki = 0;
#define INIT_DELAY 0

// Sensors PINS
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2
#define BOX_SENSOR              8
unsigned int sensorValues[NUM_SENSORS];
const float SENSOR_POS[] = {0, 1, 2, 3, 4, 5, 6, 7};
const int SENSOR_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7};
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7}, // sensors 0 through 7 are connected to analog inputs 0 through 7, respectively
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

void TurnLeft()
{
  // Set LEFT to REVERSE and
  // RIGHT to FORWARD
  analogWrite(LEFT_MOTOR_RV, MAX_SPEED - BIAS - REDUCED_SPEED);
  analogWrite(LEFT_MOTOR_FWD, 0);
  analogWrite(RIGHT_MOTOR_FWD, MAX_SPEED - REDUCED_SPEED);
  analogWrite(RIGHT_MOTOR_RV, 0);
}

void TurnRight()
{
  // Set RIGHT to REVERSE and 
  // LEFT to FORWARD	
  analogWrite(LEFT_MOTOR_FWD, MAX_SPEED - BIAS - REDUCED_SPEED);
  analogWrite(LEFT_MOTOR_RV, 0);
  analogWrite(RIGHT_MOTOR_RV, MAX_SPEED - REDUCED_SPEED);
  analogWrite(RIGHT_MOTOR_FWD, 0);
}

void Stop()
{
  analogWrite(LEFT_MOTOR_FWD, 0);
  analogWrite(LEFT_MOTOR_RV, 0);
  analogWrite(RIGHT_MOTOR_RV, 0);
  analogWrite(RIGHT_MOTOR_FWD, 0);
}

void Push()
{
  analogWrite(LEFT_MOTOR_FWD, MAX_SPEED);
  analogWrite(RIGHT_MOTOR_FWD, MAX_SPEED);
  analogWrite(LEFT_MOTOR_RV, 0);
  analogWrite(RIGHT_MOTOR_RV, 0);
}

void Retreat()
{
  analogWrite(LEFT_MOTOR_RV, MAX_SPEED);
  analogWrite(RIGHT_MOTOR_RV, MAX_SPEED);
  analogWrite(LEFT_MOTOR_FWD, 0);
  analogWrite(RIGHT_MOTOR_FWD, 0);
}

// Adjusts the speed of the left and right motors by
// varying the PWM duty cycle according to the control signal.
// The control signal represents how far right or left the
// robot has to turn, not the speed of of the motors.
void Drive(float u, boolean showSpeeds = false)
{
  // The PWM frequency is approx. 490 Hz
  if (u < 0)
  {
    analogWrite(RIGHT_MOTOR_FWD, MAX_SPEED + u);
    analogWrite(LEFT_MOTOR_FWD, MAX_SPEED - BIAS);
    analogWrite(LEFT_MOTOR_RV, 0);
    analogWrite(RIGHT_MOTOR_RV, 0);
    if (showSpeeds) Serial.print("RIGHT = "), Serial.println(MAX_SPEED + u);
  }
  else
  {
    analogWrite(RIGHT_MOTOR_FWD, MAX_SPEED);
    analogWrite(LEFT_MOTOR_FWD, MAX_SPEED - u - BIAS);
    analogWrite(LEFT_MOTOR_RV, 0);
    analogWrite(RIGHT_MOTOR_RV, 0);
    if (showSpeeds) Serial.print("LEFT = "), Serial.println(MAX_SPEED - u - BIAS);
  }
}

float Sense(boolean show = false)
{
  qtra.read(sensorValues);
  float sum_sensorWeight = 0.0;
  float sum_sensorVal = 0.0;
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] < SENSOR_THRESHOLD) sensorValues[i] = 0;
    sum_sensorWeight += sensorValues[i]*SENSOR_POS[i];
    sum_sensorVal += sensorValues[i];

    if (show) Serial.print(sensorValues[i]), Serial.print('\t');
  }

  if ( sum_sensorVal / NUM_SENSORS > SENSOR_THRESHOLD && millis() > 1000)
  {
    stopReached = true;
    // Check if box is white or black
    // take the average of many samples
    float sensorAvg = 0.0;
    for (int i = 0; i < SAMPLE_AMOUNT; i++)
      sensorAvg += analogRead( BOX_SENSOR );
    sensorAvg /= SAMPLE_AMOUNT;
 
   /*
    if (sensorAvg > SENSOR_THRESHOLD) whiteBox = false;
    else                              whiteBox = true;
    */

    if (whiteBox) whiteBox = false;
    else whiteBox = true;

    return 3.5; 
  }

  float pos = sum_sensorWeight / sum_sensorVal;
  if (show) Serial.print(" | "), Serial.println(pos);
  return (3.5 - pos);
}

void Control()
{
  static float INTEGRATION = 0;
  static float DERIVATION = 0;
  static float PREVERROR = Sense();
  static float DELTATIME = 0;
  static long PREVTIME = millis();

  // Control
  float e = Sense();

  // Box management logic
  if (stopReached)
  {
    // ESTE IF SE ENCARGA DE AGARRAR LA CAJA Y REGRESAR CUANDO SE ENCUENTRA
    // CON UNA LINEA CRUZADA LA PRIMERA VEZ
    if (!boxGrabbed)
    {
      Stop();
      delay(250);
      gripper.write(CLOSED);
      delay(500);
      //boxGrabbed = true;
      TurnRight();

      // The first while is to keep the rover turning until 
      // all the sensors are all above white surface.
      // The second is to keep it turning until it finds the line again.
      delay(100);
      while (!isnan(Sense()));
      delay(100);
      while (isnan(Sense()));
    }
    // ESTE ELSE SE ENCARGA DE DEJAR LA CAJA A UN LADO U OTRO, RETROCEDER,
    // Y LUEGO GIRAR Y REGRESAR LA SEGUNDA VEZ QUE SE ENCUENTRE CON UNA LINEA CRUZADA
    else
    {
      Stop();
      delay(500);
      gripper.write(OPEN);
      delay(500);
      //boxGrabbed = false;

      // Turn for 0.25 secs, move backwards for 0.5 secs,
      // and then turn again until finds the line
      whiteBox ? TurnLeft() : TurnRight();
      delay(250);
      Retreat();
      delay(500);
      whiteBox ? TurnLeft() : TurnRight();
      delay(500);
    }

    INTEGRATION = 0;
    DERIVATION = 0;
    PREVERROR = Sense();
    e = PREVERROR;
    DELTATIME = 0;
    PREVTIME = millis() - PREVTIME;
  }

  long currTime = millis();
  DELTATIME = (currTime - PREVTIME)/1000.0;
  PREVTIME = currTime;

  if (isnan(PREVERROR)) PREVERROR = e;
  DERIVATION = (e - PREVERROR)/DELTATIME;
  PREVERROR = e;

  INTEGRATION = (INTEGRATION + e*DELTATIME);

  if (isnan(INTEGRATION)) INTEGRATION = 0;

  float u = Kp*e;
  float D = Kd*DERIVATION;
  float I = Ki*INTEGRATION;
  if (!isnan(D)) u += D;
  if (!isnan(I)) u += I;

  // Limit control signal u to [-MAX_SPEED, MAX_SPEED]
  if (u > MAX_SPEED - BIAS) u = MAX_SPEED - BIAS;
  else if (u < -MAX_SPEED) u = -MAX_SPEED;

  if (stopReached) boxGrabbed = !boxGrabbed;
  stopReached = false;
  if (!isnan(u)) Drive(u, true);
}

void setup()
{
  // For testing
  Serial.begin(9600);

  // Attaching gripper's servo motor
  gripper.attach(SERVO_PIN);
  gripper.write(OPEN);
}

void loop()
{
  Control();
}