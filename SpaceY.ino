/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================================================
*/

//=============================================================================
//=======================MPU 12CDEV Library & Variables========================
//=============================================================================

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

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

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define POWER_PIN 22
#define GROUND_PIN 23


//=============================================================================
//=======================TFT LCD Library & Variables========================
//=============================================================================
// IMPORTANT: Adafruit_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h FOR SETUP.
//Technical support:goodtft@163.com

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

//=============================================================================
//======================== Motion Variables ===================================
//=============================================================================

#define ANGLE_DEADBAND 5
#define LEV1_THRES 12
#define LEV2_THRES 90
#define LEV1_V 3
#define LEV2_V 6

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_O[3] = {0, 0, 0};

//=============================================================================
//========================= Image Variables ===================================
//=============================================================================

struct obj{
  int x;
  int y;
  int v;
  int color;
  bool life;
}obj;

struct obj spaceship,creep_1,creep_2;
    
int pesawat [14] [21] = {
  //as many vals as dim1
 {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0},
 {0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0,0},
 {0,0,0,1,1,0,0,0,1,0,1,0,1,0,0,0,1,1,0,0,0},
 {0,0,0,1,1,0,0,0,1,0,1,0,1,0,0,0,1,1,0,0,0},
 {0,0,0,1,1,1,1,1,0,0,1,0,0,1,1,1,1,1,0,0,0},
 {0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0},
 {0,0,0,1,0,1,1,1,0,0,1,0,0,1,1,1,0,1,0,0,0},
 {0,0,0,0,1,0,0,0,1,1,1,1,1,0,0,0,1,0,0,0,0}

};

int creep [14] [21] = {
  //as many vals as dim1
 {0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,1,0,1,1,1,0,1,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,0},
 {0,0,0,0,0,1,1,1,1,0,0,0,1,1,1,1,0,0,0,0,0},
 {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
 {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},
 {0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0},
 {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},
 {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},
 {0,0,0,1,1,1,0,0,0,0,1,0,0,0,0,1,1,1,0,0,0},
 {0,0,0,1,1,1,1,1,0,0,1,0,0,1,1,1,1,1,0,0,0},
 {0,0,0,1,0,0,0,1,0,0,1,0,0,1,0,0,0,1,0,0,0},
 {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0}
};

int hor, ver;
int score = 0;

// ===========================================================================
// ======================INTERRUPT DETECTION ROUTINE==========================
// ===========================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



//=============================================================================
//=======================FreeRTOS Task Initiation==============================
//=============================================================================

// define two tasks for Update Data & Update Data Offset
void TaskUpdateData( void *pvParameters );
void TaskUpdateDataOffset( void *pvParameters );
void TaskDrawSpaceship(void *pvParameters);
void TaskDrawCreep(void *pvParameters);

TaskHandle_t TaskHandleUpdateData ;
TaskHandle_t TaskHandleUpdateDataOffset ;
TaskHandle_t TaskHandleDrawSpaceship ;
TaskHandle_t TaskHandleDrawCreep;
TaskHandle_t TaskHandleDrawLaser;

// Create a Semaphore binary flag for the Serial Port. To ensure only single access.
SemaphoreHandle_t LCDSemaphore;

//=============================================================================
//================================ Setup ======================================
//=============================================================================


void setup() {
  // put your setup code here, to run once:
    // Now set up two tasks to run independently.
  Serial.begin(9600);
  init_read_data();
  init_tft_lcd();
  pinMode(44,INPUT);
  ver = 0;
  hor = 0;
  spaceship.x = 40;
  spaceship.y = 280;
  spaceship.color = BLUE;

  creep_1.x = random(220);
  creep_1.y = 0;
  creep_1.v = 1;
  creep_1.color = RED;
  creep_1.life = true;

  init_frame();
  init_print_score();
    
  xTaskCreate(
    TaskUpdateData
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  512  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandleUpdateData );

  xTaskCreate(
    TaskUpdateDataOffset
    ,  (const portCHAR *) "AnalogRead"
    ,  256  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &TaskHandleUpdateDataOffset );

  xTaskCreate(
    TaskDrawSpaceship
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandleDrawSpaceship );  

  xTaskCreate(
    TaskDrawCreep
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandleDrawCreep );  


  xTaskCreate(
      draw_laser
      ,  (const portCHAR *)"gambar creep"   // A name just for humans
      ,  512  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskHandleDrawLaser );

  if ( LCDSemaphore == NULL )          // Check to see if the Serial Semaphore has not been created.
  {
    LCDSemaphore = xSemaphoreCreateMutex(); // mutex semaphore for Serial Port
    if ( ( LCDSemaphore ) != NULL )
      xSemaphoreGive( ( LCDSemaphore ) );  // make the Serial Port available
  }

  vTaskSuspend(TaskHandleUpdateData);  
  vTaskSuspend(TaskHandleDrawSpaceship);
  vTaskSuspend(TaskHandleDrawCreep);  
  vTaskSuspend(TaskHandleDrawLaser); 
}

//=============================================================================
//=============================== Loop ========================================
//=============================================================================

void loop() {
  // put your main code here, to run repeatedly:

}

//=============================================================================
//================================ Task =======================================
//=============================================================================

void TaskUpdateData(void *pvParameters)  // This is a task.
{
  (void) pvParameters;  
  TickType_t xLastWakeTime; 
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;) // A Task shall never return or exit.
  {
    read_data();
    spaceship_position_adjust();
    vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );
  }
}

void TaskUpdateDataOffset(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  for (;;) // A Task shall never return or exit.
  {
    read_data_offset();
    vTaskDelay(   200 / portTICK_PERIOD_MS  );
  }
}

void TaskDrawSpaceship(void *pvParameters)
{
  (void) pvParameters;
  TickType_t xLastWakeTime; 

  xLastWakeTime = xTaskGetTickCount();

  
  for (;;) // A Task shall never return or exit.
  {  
        
    // See if we can obtain the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks to see if it becomes free.
    if ( xSemaphoreTake( LCDSemaphore, ( TickType_t ) 5 / portTICK_PERIOD_MS ) == pdTRUE )
    {
      // We were able to obtain the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      spaceship.x +=  hor;
      spaceship.y +=  ver; 
        
      spaceship = draw_img(pesawat,spaceship);
      vTaskDelay(8 / portTICK_PERIOD_MS);
      clear_img(pesawat, spaceship); 
      
      xSemaphoreGive( LCDSemaphore ); // Now free the Serial Port for others.
    } 
    vTaskDelay(4 / portTICK_PERIOD_MS);
    //vTaskDelay(5 / portTICK_PERIOD_MS);
    
  }
  
}

void TaskDrawCreep(void *pvParameters)
{
  (void) pvParameters;
  TickType_t xLastWakeTime; 

  xLastWakeTime = xTaskGetTickCount();
  
  for (;;) // A Task shall never return or exit.
  { 
    // See if we can obtain the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks to see if it becomes free.
    if((creep_1.life == false))
    {
      creep_1.y =  5;
      creep_1.x = random(5,215);
      creep_1.v = random((1+score/10),(6+score/10));
     
      creep_1.life = true;
      vTaskDelay( 300 / portTICK_PERIOD_MS );
    }
    
    if ( xSemaphoreTake( LCDSemaphore, ( TickType_t ) 5 / portTICK_PERIOD_MS ) == pdTRUE )
    {
      // We were able to obtain the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
          creep_1.y +=  creep_1.v;
          
        if((creep_1.y >= 300))
        {
          creep_1.y =  0;
          creep_1.x = random(220);
          creep_1.v = random(1,5);
          score = 0;
          print_KO();
          print_score(score);
          vTaskDelay( 800 / portTICK_PERIOD_MS );
          creep_1.life = false;
          clear_KO();
          //creep_1.life = true;
        }


        
        if(creep_1.life == true)
        {
          //vTaskSuspendAll();
          creep_1 = draw_img(creep,creep_1);
          vTaskDelay( 9 / portTICK_PERIOD_MS ); // wait for one second
          clear_img(creep,creep_1);
          //vTaskDelay( 1 / portTICK_PERIOD_MS ); // wait for one second
          //xTaskResumeAll();
        }
        else
        {
          clear_img(creep,creep_1);      
        }
        
        xSemaphoreGive( LCDSemaphore ); // Now free the Serial Port for others.
    }   
    //vTaskDelayUntil( &xLastWakeTime, ( 3 / portTICK_PERIOD_MS ) );
    vTaskDelay(4 / portTICK_PERIOD_MS);

  }
}

void draw_laser(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    if(digitalRead(44)==0)
    {    
      if ( xSemaphoreTake( LCDSemaphore, ( TickType_t ) 5 / portTICK_PERIOD_MS ) == pdTRUE )
      {
        draw_laser(spaceship);
        clear_laser(spaceship);
        clear_img(pesawat,creep_1);
        xSemaphoreGive( LCDSemaphore );
      }
      vTaskDelay( 5 / portTICK_PERIOD_MS );
    }
  }
}


//=============================================================================
//=================================== IMU Function ============================
//=============================================================================
void init_read_data()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        
        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();
        
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else 
    {
      
      
    } 
}


void read_data()
{   
    //Serial.println("Start Update Data");
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr[0] = ypr[0] - ypr_O[0];
        ypr[1] = ypr[1] - ypr_O[1];
        ypr[2] = ypr[2] - ypr_O[2];     

    }  
}


void read_data_offset()
{   
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr_O, &q, &gravity);
        #endif
        
       vTaskResume(TaskHandleUpdateData); 
       vTaskResume(TaskHandleDrawSpaceship);
       vTaskResume(TaskHandleDrawCreep); 
       vTaskResume(TaskHandleDrawLaser); 
       vTaskSuspend(NULL );

    }
}



void spaceship_position_adjust()
{
  //Mapping Pitch reading to Vertical movement
  
  if (-1*ypr[1]* 180/M_PI > -1*ANGLE_DEADBAND && -1*ypr[1]* 180/M_PI<=ANGLE_DEADBAND)
  {
    ver = 0;  
  }
  else if(-1*ypr[1]* 180/M_PI > ANGLE_DEADBAND && -1*ypr[1]* 180/M_PI<=LEV1_THRES)
  {
    ver = LEV1_V;  
  }
  else if(-1*ypr[1]* 180/M_PI > LEV1_THRES && -1*ypr[1]* 180/M_PI<=LEV2_THRES)
  {
    ver = LEV2_V;  
  }
    else if(-1*ypr[1]* 180/M_PI > LEV1_THRES*-1 && -1*ypr[1]* 180/M_PI<=ANGLE_DEADBAND*-1)
  {
    ver = -1 * LEV1_V;  
  }
  else if(-1*ypr[1]* 180/M_PI > LEV2_THRES*-1 && -1*ypr[1]* 180/M_PI<=LEV1_THRES*-1)
  {
    ver = -1 * LEV2_V;  
  }
  else
  {
    ver = 0;  
  }

  //Mapping Roll reading to Horizontal movement
  if (-1*ypr[2]* 180/M_PI > -1*ANGLE_DEADBAND && -1*ypr[2]* 180/M_PI<=ANGLE_DEADBAND)
  {
    hor = 0;  
  }
  else if(-1*ypr[2] * 180/M_PI> ANGLE_DEADBAND && -1*ypr[2]* 180/M_PI<=LEV1_THRES)
  {
    hor = LEV1_V;  
  }
  else if(-1*ypr[2] * 180/M_PI> LEV1_THRES && -1*ypr[2]* 180/M_PI<=LEV2_THRES)
  {
    hor = LEV2_V;  
  }
    else if(-1*ypr[2]* 180/M_PI > LEV1_THRES*-1 && -1*ypr[2]* 180/M_PI<=ANGLE_DEADBAND*-1)
  {
    hor = -1 * LEV1_V;  
  }
  else if(-1*ypr[2]* 180/M_PI > LEV2_THRES*-1 && -1*ypr[2]* 180/M_PI<=LEV1_THRES*-1)
  {
    hor = -1 * LEV2_V;  
  }
  else
  {
    hor = 0;  
  }
  
}

//=============================================================================
//=================================== Draw Function ===========================
//=============================================================================

void init_tft_lcd()
{      
      tft.reset();
    
      uint16_t identifier = tft.readID();
      if(identifier==0x0101)
          identifier=0x9341;
  
      tft.begin(identifier);
      tft.fillScreen(BLACK);
}

void init_frame()
{
    tft.drawLine(2, 2, 200, 2, WHITE);
    tft.drawLine(2, 2, 2, 320, WHITE);
    tft.drawLine(2, 318, 200, 318, WHITE);
    tft.drawLine(200, 318, 200, 2, WHITE);
    tft.setCursor(210, 2);
    tft.setTextColor(WHITE);  tft.setTextSize(1);
    tft.println("SCORE");
    tft.setCursor(220, 20);
    tft.setTextColor(GREEN);  tft.setTextSize(1);
    tft.println(score);
    tft.setCursor(212, 50);
    tft.setTextColor(RED);  tft.setTextSize(4);
    tft.println("S");
    tft.setCursor(212, 80);
    tft.setTextColor(RED);  tft.setTextSize(4);
    tft.println("P");
    tft.setCursor(212, 110);
    tft.setTextColor(RED);  tft.setTextSize(4);
    tft.println("A");
    tft.setCursor(212, 140);
    tft.setTextColor(RED);  tft.setTextSize(4);
    tft.println("C");
    tft.setCursor(212, 170);
    tft.setTextColor(RED);  tft.setTextSize(4);
    tft.println("E");
    tft.setCursor(212, 210);
    tft.setTextColor(GREEN);  tft.setTextSize(4);
    tft.println("Y");
}

void init_print_score()
{
    tft.setTextColor(WHITE);  tft.setTextSize(1);  
}

void print_score(int score)
{
    tft.setCursor(220, 20);
    tft.setTextColor(GREEN);  tft.setTextSize(1);
    for(int i=0; i<=20; i++)
    {
      for(int j=0; j<=20; j++)
      {
        tft.drawPixel(i+220, j+20, BLACK);
      }
    }  
    tft.println(score);  
}


void clear_img(int draw[14][21],struct obj spaceship)
{
    //hapus gambar pesawat
        tft.drawPixel(3+spaceship.x, 8+spaceship.y, BLACK);
        tft.drawPixel(3+spaceship.x, 9+spaceship.y, BLACK);
        tft.drawPixel(3+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(3+spaceship.x, 11+spaceship.y, BLACK);
        tft.drawPixel(3+spaceship.x, 12+spaceship.y, BLACK);
        tft.drawPixel(4+spaceship.x, 8+spaceship.y, BLACK);
        tft.drawPixel(4+spaceship.x, 9+spaceship.y, BLACK);
        tft.drawPixel(4+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(4+spaceship.x, 13+spaceship.y, BLACK);
        tft.drawPixel(5+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(5+spaceship.x, 12+spaceship.y, BLACK);
        tft.drawPixel(6+spaceship.x, 3+spaceship.y, BLACK);
        tft.drawPixel(6+spaceship.x, 4+spaceship.y, BLACK);
        tft.drawPixel(6+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(6+spaceship.x, 12+spaceship.y, BLACK);
        tft.drawPixel(7+spaceship.x, 3+spaceship.y, BLACK);
        tft.drawPixel(7+spaceship.x, 5+spaceship.y, BLACK);
        tft.drawPixel(7+spaceship.x, 6+spaceship.y, BLACK);
        tft.drawPixel(7+spaceship.x, 7+spaceship.y, BLACK);
        tft.drawPixel(7+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(7+spaceship.x, 12+spaceship.y, BLACK);
        tft.drawPixel(8+spaceship.x, 2+spaceship.y, BLACK);
        tft.drawPixel(8+spaceship.x, 8+spaceship.y, BLACK);
        tft.drawPixel(8+spaceship.x, 9+spaceship.y, BLACK);
        tft.drawPixel(8+spaceship.x, 13+spaceship.y, BLACK);
        tft.drawPixel(9+spaceship.x, 1+spaceship.y, BLACK);
        tft.drawPixel(9+spaceship.x, 13+spaceship.y, BLACK);
        tft.drawPixel(10+spaceship.x, 0+spaceship.y, BLACK);
        tft.drawPixel(10+spaceship.x, 1+spaceship.y, BLACK);
        tft.drawPixel(10+spaceship.x, 2+spaceship.y, BLACK);
        tft.drawPixel(10+spaceship.x, 4+spaceship.y, BLACK);
        tft.drawPixel(10+spaceship.x, 5+spaceship.y, BLACK);
        tft.drawPixel(10+spaceship.x, 12+spaceship.y, BLACK);
        tft.drawPixel(10+spaceship.x, 13+spaceship.y, BLACK);
        tft.drawPixel(11+spaceship.x, 1+spaceship.y, BLACK);
        tft.drawPixel(11+spaceship.x, 13+spaceship.y, BLACK);
        tft.drawPixel(12+spaceship.x, 2+spaceship.y, BLACK);
        tft.drawPixel(12+spaceship.x, 8+spaceship.y, BLACK);
        tft.drawPixel(12+spaceship.x, 9+spaceship.y, BLACK);
        tft.drawPixel(12+spaceship.x, 13+spaceship.y, BLACK);
        tft.drawPixel(13+spaceship.x, 3+spaceship.y, BLACK);
        tft.drawPixel(13+spaceship.x, 5+spaceship.y, BLACK);
        tft.drawPixel(13+spaceship.x, 6+spaceship.y, BLACK);
        tft.drawPixel(13+spaceship.x, 7+spaceship.y, BLACK);
        tft.drawPixel(13+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(13+spaceship.x, 12+spaceship.y, BLACK);
        tft.drawPixel(14+spaceship.x, 3+spaceship.y, BLACK);
        tft.drawPixel(14+spaceship.x, 4+spaceship.y, BLACK);
        tft.drawPixel(14+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(14+spaceship.x, 12+spaceship.y, BLACK);
        tft.drawPixel(15+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(15+spaceship.x, 12+spaceship.y, BLACK);
        tft.drawPixel(16+spaceship.x, 8+spaceship.y, BLACK);
        tft.drawPixel(16+spaceship.x, 9+spaceship.y, BLACK);
        tft.drawPixel(16+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(16+spaceship.x, 13+spaceship.y, BLACK);
        tft.drawPixel(17+spaceship.x, 8+spaceship.y, BLACK);
        tft.drawPixel(17+spaceship.x, 9+spaceship.y, BLACK);
        tft.drawPixel(17+spaceship.x, 10+spaceship.y, BLACK);
        tft.drawPixel(17+spaceship.x, 11+spaceship.y, BLACK);
        tft.drawPixel(17+spaceship.x, 12+spaceship.y, BLACK);
}

struct obj draw_img(int draw[14][21],struct obj spaceship)
{
      if(spaceship.x <4)
        spaceship.x = 4;
      else if (spaceship.x > 180)
        spaceship.x = 180;

      if(spaceship.y <4)
        spaceship.y = 4;
      else if (spaceship.y > 300)
        spaceship.y = 300;
          
      //gambar pesawat
    tft.drawPixel(3+spaceship.x, 8+spaceship.y, spaceship.color);
      tft.drawPixel(3+spaceship.x, 9+spaceship.y, spaceship.color);
      tft.drawPixel(3+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(3+spaceship.x, 11+spaceship.y, spaceship.color);
      tft.drawPixel(3+spaceship.x, 12+spaceship.y, spaceship.color);
      tft.drawPixel(4+spaceship.x, 8+spaceship.y, spaceship.color);
      tft.drawPixel(4+spaceship.x, 9+spaceship.y, spaceship.color);
      tft.drawPixel(4+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(4+spaceship.x, 13+spaceship.y, spaceship.color);
      tft.drawPixel(5+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(5+spaceship.x, 12+spaceship.y, spaceship.color);
      tft.drawPixel(6+spaceship.x, 3+spaceship.y, spaceship.color);
      tft.drawPixel(6+spaceship.x, 4+spaceship.y, spaceship.color);
      tft.drawPixel(6+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(6+spaceship.x, 12+spaceship.y, spaceship.color);
      tft.drawPixel(7+spaceship.x, 3+spaceship.y, spaceship.color);
      tft.drawPixel(7+spaceship.x, 5+spaceship.y, spaceship.color);
      tft.drawPixel(7+spaceship.x, 6+spaceship.y, spaceship.color);
      tft.drawPixel(7+spaceship.x, 7+spaceship.y, spaceship.color);
      tft.drawPixel(7+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(7+spaceship.x, 12+spaceship.y, spaceship.color);
      tft.drawPixel(8+spaceship.x, 2+spaceship.y, spaceship.color);
      tft.drawPixel(8+spaceship.x, 8+spaceship.y, spaceship.color);
      tft.drawPixel(8+spaceship.x, 9+spaceship.y, spaceship.color);
      tft.drawPixel(8+spaceship.x, 13+spaceship.y, spaceship.color);
      tft.drawPixel(9+spaceship.x, 1+spaceship.y, spaceship.color);
      tft.drawPixel(9+spaceship.x, 13+spaceship.y, spaceship.color);
      tft.drawPixel(10+spaceship.x, 0+spaceship.y, spaceship.color);
      tft.drawPixel(10+spaceship.x, 1+spaceship.y, spaceship.color);
      tft.drawPixel(10+spaceship.x, 2+spaceship.y, spaceship.color);
      tft.drawPixel(10+spaceship.x, 4+spaceship.y, spaceship.color);
      tft.drawPixel(10+spaceship.x, 5+spaceship.y, spaceship.color);
      tft.drawPixel(10+spaceship.x, 12+spaceship.y, spaceship.color);
      tft.drawPixel(10+spaceship.x, 13+spaceship.y, spaceship.color);
      tft.drawPixel(11+spaceship.x, 1+spaceship.y, spaceship.color);
      tft.drawPixel(11+spaceship.x, 13+spaceship.y, spaceship.color);
      tft.drawPixel(12+spaceship.x, 2+spaceship.y, spaceship.color);
      tft.drawPixel(12+spaceship.x, 8+spaceship.y, spaceship.color);
      tft.drawPixel(12+spaceship.x, 9+spaceship.y, spaceship.color);
      tft.drawPixel(12+spaceship.x, 13+spaceship.y, spaceship.color);
      tft.drawPixel(13+spaceship.x, 3+spaceship.y, spaceship.color);
      tft.drawPixel(13+spaceship.x, 5+spaceship.y, spaceship.color);
      tft.drawPixel(13+spaceship.x, 6+spaceship.y, spaceship.color);
      tft.drawPixel(13+spaceship.x, 7+spaceship.y, spaceship.color);
      tft.drawPixel(13+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(13+spaceship.x, 12+spaceship.y, spaceship.color);
      tft.drawPixel(14+spaceship.x, 3+spaceship.y, spaceship.color);
      tft.drawPixel(14+spaceship.x, 4+spaceship.y, spaceship.color);
      tft.drawPixel(14+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(14+spaceship.x, 12+spaceship.y, spaceship.color);
      tft.drawPixel(15+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(15+spaceship.x, 12+spaceship.y, spaceship.color);
      tft.drawPixel(16+spaceship.x, 8+spaceship.y, spaceship.color);
      tft.drawPixel(16+spaceship.x, 9+spaceship.y, spaceship.color);
      tft.drawPixel(16+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(16+spaceship.x, 13+spaceship.y, spaceship.color);
      tft.drawPixel(17+spaceship.x, 8+spaceship.y, spaceship.color);
      tft.drawPixel(17+spaceship.x, 9+spaceship.y, spaceship.color);
      tft.drawPixel(17+spaceship.x, 10+spaceship.y, spaceship.color);
      tft.drawPixel(17+spaceship.x, 11+spaceship.y, spaceship.color);
      tft.drawPixel(17+spaceship.x, 12+spaceship.y, spaceship.color);


      return spaceship;
}

void draw_laser(struct obj spaceship)
{
    if(((spaceship.x+10) <= (creep_1.x+21))&&((spaceship.x+10) >= (creep_1.x))&&(creep_1.life==true)&&((spaceship.y+20) >= (creep_1.y)))
    {
      creep_1.y = 0;//creep mati
      creep_1.life = false;//creep mati
      for(int b=spaceship.y; b>creep_1.y; b=b-4) //gambar sampe creep
        tft.drawPixel(spaceship.x+10, b, GREEN);
      vTaskDelay( 2 / portTICK_PERIOD_MS );
      clear_img(pesawat,creep_1); 
      score+=1;
      print_score(score); 
    }
    else
    {
       for(int b=spaceship.y; b>8; b=b-4) //gambar sampe ujung
        tft.drawPixel(spaceship.x+10, b, GREEN);
       vTaskDelay( 2 / portTICK_PERIOD_MS );  
    }
    

}

void clear_laser(struct obj spaceship)
{
    for(int b=spaceship.y; b>=4; b=b-4)
    {
        tft.drawPixel(spaceship.x+10, b, BLACK);  
    }
    
}

void print_KO()
{
    tft.setCursor(212, 250);
    tft.setTextColor(WHITE);  tft.setTextSize(3);
    tft.println("K");
    tft.setCursor(212, 280);
    tft.setTextColor(WHITE);  tft.setTextSize(3);
    tft.println("O");
}

void clear_KO()
{
    tft.setCursor(212, 250);
    tft.setTextColor(BLACK);  tft.setTextSize(3);
    tft.println("K");
    tft.setCursor(212, 280);
    tft.setTextColor(BLACK);  tft.setTextSize(3);
    tft.println("O");
}

