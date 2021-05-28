// INCLUDED LIBRARIES
#include <I2Cdev.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

// DECLARE PINS
int leftDir1 = 6;
int leftDir2 = 9;
int rightDir1 = 10;
int rightDir2 = 11;

// DECLARE INSTANCE OF AN MPU OBJECT
MPU6050 mpu;

// DECLARE MPU CONTROL AND STATUS VARIBALES
bool dmpReady = false;  // DIGITAL MOTION PROCESSOR IS SET TO TRUE IF DMP INITIALIZATION WAS SUCCESSFUL
uint8_t mpuIntStatus;   // CONTAINS THE INTERUPT STATUS BYTE FROM THE MPU
uint8_t devStatus;      // DEVICE STATUS THAT RETURN A STATUS AFTER EACH DEVICE OPERATION (0 = SUCCESS, !0 = ERROR)
uint16_t packetSize;    // EXPECTED PACKET SIZE FOR DIGITAL MOTION PROCESSOR (DMP) (DEFAULT IS 42 BYTES)
uint16_t fifoCount;     // BYTES CURRENTLY IN FIFO
uint8_t fifoBuffer[64]; // FIFO STORAGE BUFFER

// DECLARE ORIENTATION AND MOTINO VARIABLES
Quaternion q;           // [w, x, y, z]         QUATERNION CONTAINER
VectorFloat gravity;    // [x, y, z]            GRAVITY VECTOR
float ypr[3];           // [yaw, pitch, roll]   CONTAINS THE YAW PITCH AND ROLL

// PROPORTAIONAL INTEGRAL DERIVATIVE (PID) CONTROLLER VALUES
double setpoint = 230;  // SETPOINT VALUE OF ROBOT
double Kp = 30;         // PROPORTIONAL VALUE
double Kd = .7;        // DERIVATIVE VALUE
double Ki = 130;        // INTEGRAL VALUE

// INITIALIZE THE PID ALGORITHM
double input, output; // INPUT AND OUTPUT VALUES FOR PID ALGORITHM
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// INTERRUPT FLAG FOR THE MPU6050
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // BEGIN SERIAL MONITOR FOR DEBUGGING PURPOSES
  Serial.begin(115200);

  // INITIALIZE DEVICE
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // VERIFY CONNECTIONS AND PRINT WHETHER SUCCESS OR FAIL
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // INITIALIZE THE DIITAL MOTION PROCESSOR
  devStatus = mpu.dmpInitialize();

  // DECLARE VALUES FOR THE GYRO OFFSETS
  mpu.setXGyroOffset(309);
  mpu.setYGyroOffset(-55);
  mpu.setZGyroOffset(36);
  mpu.setZAccelOffset(3231);

  // CHECK DEVICE STATUS FLAG TO INSURE ALL STEPS WERE A SUCCESS
  if (devStatus == 0) // IF SUCCESSFUL OPERATIONS
  {
    // ACTIVATE THE DIGITAL MOTION PROCESSOR
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // ENABLE THE ARDUINO INTERRUPT DETECTION
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // SET THE DMP READY FLAG
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // RETRIEVE THE PACKET SIZE OF THE DMP
    packetSize = mpu.dmpGetFIFOPacketSize();

    // PID SETUP
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else // IF NON-SUCCESSFUL OPERATIONS
  {
    // DISPLAY ERROR CODE ON THE DMP
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // DECLARE MOTOR DIRECTION PINS AS OUTPUT
  pinMode (leftDir1, OUTPUT);
  pinMode (leftDir2, OUTPUT);
  pinMode (rightDir1, OUTPUT);
  pinMode (rightDir2, OUTPUT);

  // INITIALLY HAVE MOTORS STOPPED
  Stop();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  // IF THE DMP READY FLAG WAS NEVER SET TO TRUE THE THE PROGRAM FAILED
  if (!dmpReady) return;

  // WAIT FOR AN INTERRUPT OR FOR A PACKET IN FIFO TO BECOME AVAILABLE
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // PERFORM PID CALCULATIONS AND OUTPUT TO MOTORS
    pid.Compute();

    // PRINT INPUT AND OUTPUT VALUES TO SERIAL MONITOR
    Serial.print(" Input: "); Serial.print(input); Serial.print(" Output: "); Serial.println(output);

    // IF THE ROBOT IS FALLING
    if (input > (setpoint-40) && input < (setpoint+40)) 
    {
      // FALLING FORWARDS
      if (output > 0) 
        Forward();

      // FALLINF BACKWARDS
      else if (output < 0) 
        Reverse(); 
    }

    // IF THE ROBOT IS NOT FALLING
    else
      Stop();
  }

  // RESET THE MPU INTERUPT FLAG AND GET THE INTERUPT STATUS
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // GET THE CUURENT FIFO COUNT
  fifoCount = mpu.getFIFOCount();

  // CHECK FOR OVER FLOW
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // RESET FIFO
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }

  // IF NO OVERFLOW CHECK FOR DMP DATA READY INTERRUPT
  else if (mpuIntStatus & 0x02)
  {
    // WAIT FOR A FULL PACKET TO BECOME AVAIBLE
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // READ A PACKET FROM FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // KEEP FIFO COUNT UPDATED IN CASE THERE IS MORE THAN MORE PACKETS AVAILABLE
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);       // GET QUATERNION VALUES
    mpu.dmpGetGravity(&gravity, &q);            // GET GRAVITY VECTOR VALUES
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  // GET THE YAW, PITCH, AND ROLL VALUES

    // CALCULATE THE NEW INOUT FIR THE PID CONTROLLER
    input = ypr[1] * 180 / M_PI + 180;          
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// DRIVES WHEELS FOROWARD
void Forward()
{
  analogWrite(leftDir1, output);
  analogWrite(leftDir2, 0);

  analogWrite(rightDir1, output);
  analogWrite(rightDir2, 0);

  Serial.print("F"); // PRINT F TO SERIAL MONITOR FOR DEBUGGING
}

// DRIVES WHEELS IN REVERSE
void Reverse()
{
  analogWrite(leftDir1, 0);
  analogWrite(leftDir2, output * -1);

  analogWrite(rightDir1, 0);
  analogWrite(rightDir2, output * -1);

  Serial.print("R"); // PRINT R TO SERIAL MONITOR FOR DEBUGGING
}

// STOPS BOTH WHEELS
void Stop()
{
  analogWrite(leftDir1, 0);
  analogWrite(leftDir2, 0);

  analogWrite(rightDir1, 0);
  analogWrite(rightDir2, 0);

  Serial.print("S"); // PRINT S TO SERIAL MONITOR FOR DEBUGGING
}
