
#include <Arduino_LSM9DS1.h> 

void read_N_Accel(unsigned int N, float& averX, float& averY, float& averZ);
void read_N_Gyro(unsigned int N, float& averX, float& averY, float& averZ);
void read_N_Magnet(unsigned int N, float& averX, float& averY, float& averZ);

void setup() {
  Serial.begin(9600); 
  while (!Serial);
  pinMode(LED_BUILTIN,OUTPUT); 
  delay(10);
  if (!IMU.begin()) { Serial.println(F("Failed to initialize IMU!")); while (1);  }


/*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     ****************
********************         Copy/Replace the lines below by the code output of the program              ****************/
//   IMU.setAccelFS(3);           
//   IMU.setAccelODR(5);           // 
//   IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
//   IMU.setAccelSlope (1, 1, 1);  //   uncalibrated
   IMU.setAccelFS(0);
   IMU.setAccelODR(5);
   IMU.setAccelOffset(-0.005113, -0.000879, 0.014134);
   IMU.setAccelSlope (1.005486, 1.000644, 1.009336);
/***********************************************************************************************************************************
*******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
*******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******
************************************************************************************************************************************/
   IMU.accelUnit=  GRAVITY;    // or  METERPERSECOND2    

/*******   The gyroscope needs to be calibrated. Offset controls drift and Slope scales the measured rotation angle  *********
*****************   Copy/Replace the lines below by the output of the DIY_Calibration_Gyroscope sketch   ********************/
//   IMU.setAccelFS(3);    
//   IMU.setAccelODR(3);
//   IMU.setGyroOffset (0, 0, 0);  // = uncalibrated
//   IMU.setGyroSlope  (1, 1, 1);  // = uncalibrated
   IMU.setGyroFS(0);
   IMU.setGyroODR(5);
   IMU.setGyroOffset (-0.168854, 0.844375, -1.256459);
   IMU.setGyroSlope (1.172718, 1.138811, 1.167088);
/*****************************************************************************************************************************     
*********  FS  Full Scale       setting 0: ±245°/s | 1: ±500°/s | 2: ±1000°/s | 3: ±2000°/s       ****************************
*********  ODR Output Data Rate setting 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (not working 6:952Hz)   *******
*****************************************************************************************************************************/     
   IMU.gyroUnit= DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  

/*****************   For a proper functioning of the compass the magnetometer needs to be calibrated    ********************
*****************   Replace the lines below by the output of the DIY_Calibration_Magnetometer sketch   ********************/
//   IMU.setMagnetFS(0);  
//   IMU.setMagnetODR(8); 
//   IMU.setMagnetOffset(0, 0, 0);  //  uncalibrated
//   IMU.setMagnetSlope (1, 1, 1);  //  uncalibrated */
   IMU.setMagnetFS(0);
   IMU.setMagnetODR(8);
   IMU.setMagnetOffset(-7.506714, 70.541382, -44.193726);
   IMU.setMagnetSlope (1.334641, 1.382020, 1.255356);

/******************************************************************************************************************************     
****  FS  Full Scale        range (0=±400 | 1=±800 | 2=±1200 | 3=±1600  (µT)                                              *****     
****  ODR Output Data Rate  range (6,7,8)=(40,80,400)Hz | not available on all chips (0..5): (0.625,1.25,2.5,5.0,10,20)Hz *****
*******************************************************************************************************************************/     
   IMU.magnetUnit = MICROTESLA;  //   GAUSS   MICROTESLA   NANOTESLA
}

void loop(){
   int nrOfAccelSamples=50;
   int nrOfGyroSamples=50;
   int nrOfMagnetSamples=50;

   float measureAccelTime = nrOfAccelSamples/(IMU.getAccelODR());
   float measureGyroTime = nrOfGyroSamples/(IMU.getGyroODR());
   float measureMagnetTime = nrOfMagnetSamples/(IMU.getMagnetODR());

   float ax, ay, az, gx, gy, gz, mx, my, mz;

   read_N_Accel(nrOfAccelSamples,ax, ay, az);
   read_N_Gyro(nrOfGyroSamples,gx, gy, gz);
   read_N_Magnet(nrOfMagnetSamples,mx, my, mz);

   Serial.print(ax);
   Serial.print('\t');
   Serial.print(ay);
   Serial.print('\t');
   Serial.print(az);
   Serial.print('\t');
   Serial.print(gx);
   Serial.print('\t');
   Serial.print(gy);
   Serial.print('\t');
   Serial.print(gz);
   Serial.print('\t');
   Serial.print(mx);
   Serial.print('\t');
   Serial.print(my);
   Serial.print('\t');
   Serial.println(mz);
 }

void read_N_Accel(unsigned int N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     averX=0; averY =0;averZ =0;
     for (int i=1;i<=N;i++)
     {  while (!IMU.accelAvailable());
        IMU.readAccel(x, y, z);
        averX += x;    averY += y;     averZ += z;
     } 
     averX /= N;    averY /= N;     averZ /= N;
}

void read_N_Gyro(unsigned int N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     averX=0; averY =0;averZ =0;
     for (int i=1;i<=N;i++)
     {  while (!IMU.gyroAvailable());
        IMU.readGyro(x, y, z);
        averX += x;    averY += y;     averZ += z;
     } 
     averX /= N;    averY /= N;     averZ /= N;
}

void read_N_Magnet(unsigned int N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     averX=0; averY =0;averZ =0;
     for (int i=1;i<=N;i++)
     {  while (!IMU.magnetAvailable());
        IMU.readMagnet(x, y, z);
        averX += x;    averY += y;     averZ += z;
     } 
     averX /= N;    averY /= N;     averZ /= N;
}
