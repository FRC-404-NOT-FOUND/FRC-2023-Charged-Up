//Modified from MPU6050_DMP6 at https://github.com/jrowberg/i2cdevlib, by Jeff Rowberg

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

//Uncomment if you want original functionality
#define TEXT_PROMPT_INPUT
//Uncomment if you want to pull in a continuous stream of data
//#define CONTINUOUS_INPUT

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vectors

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(9600);
    Serial.setTimeout(10);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices...; "));
    mpu.initialize();
    // pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections... "));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful; ") : F("MPU6050 connection failed; "));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        //set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        Serial.println("READY");

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    //I tried checking for an input, but it took to long to loop through everything.
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    #ifdef CONTINUOUS_INPUT
         if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("y: ");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print(" p: ");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(" r: ");
            Serial.println(ypr[2] * 180/M_PI);
         }
    #endif

    #ifdef TEXT_PROMPT_INPUT
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            char message[100]; //Read the latest string.
            int availableBytes = Serial.available();
            for(int i = 0; i < availableBytes; i++) {
                message[i] = Serial.read();
            } 
            
            message[availableBytes] = '\0'; // Append a null
            //Sorry for the if-elseif-elseif-elseif-elseif-elseif :(((
            if(message[0] == 'g'){
                switch (message[1]) {
                    case 'Y': // yaw
                        Serial.print(ypr[0] * 180/M_PI);
                        Serial.println(";");
                        break;

                    case 'P': // pitch
                        Serial.print(ypr[1] * 180/M_PI);
                        Serial.println(";");
                        break;

                    case 'R': // roll
                        Serial.print(ypr[2] * 180/M_PI);
                        Serial.println(";");
                        break;

                    case 'A': // all
                        Serial.print(ypr[0] * 180/M_PI);
                        Serial.print(ypr[1] * 180/M_PI);
                        Serial.print(ypr[2] * 180/M_PI);
                        break;

                    case 'C': // calibrate
                        Serial.println("Calibrating IMU...");
                        mpu.CalibrateAccel(6);
                        mpu.CalibrateGyro(6);
                        Serial.println("");
                        Serial.println("DONE!");
                        break;
                    
                    default: // unreachable but you know just in case someone does something really stupid at least we'll know
                        break;
                }
            }   
            // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
        }
    #endif
}
