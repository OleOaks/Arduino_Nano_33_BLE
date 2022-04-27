//**********************************************************************************************************************************
//*********************************************              Accelerometer            **********************************************
//**********************************************************************************************************************************

#include <Arduino.h>
#include <Arduino_LSM9DS1.h> 
#include "functions.h"

const float accelCriterion = 0.1;
char xyz[3]= {'X','Y','Z'};
float maxAX = 1, maxAY=1, maxAZ=1, minAX=-1, minAY=-1, minAZ=-1; // Accel Slope
float zeroAX1 =0,zeroAX2 =0,zeroAY1 =0,zeroAY2 =0,zeroAZ1 =0,zeroAZ2 =0;  //Accel Offset
boolean accelOK=false;
uint8_t acceMMlOK=0; // bit 0..2 maxXYZ bit 3..5 minXYZ


void printParam(char txt[], float param[3])
{   for (int i= 0; i<=2 ; i++) 
    {  Serial.print(txt);Serial.print("[");
       Serial.print(i);Serial.print("] = "); 
       Serial.print(param[i],6);Serial.print(";");
    }
}
void printSetParam(char txt[], float param[3])
{   Serial.print(txt);Serial.print("(");
    Serial.print(param[0],6);Serial.print(", ");
    Serial.print(param[1],6);Serial.print(", ");
    Serial.print(param[2],6);Serial.print(");");
}

void calibrateAccelMenu(uint8_t accelFSindex, uint8_t accelODRindex)
{char incomingByte = 0;
 byte b;
 uint16_t NofCalibrationSamples = 1000;
 while (1)  //(incomingByte!='X') 
 {  Serial.println(F("\n\n")); 
    Serial.println(F(" Calibrate Accelerometer Offset and Slope"));
    Serial.println(F(" Before calibrating choose the Full Scale (FS) setting and Output Data Rate (ODR). The accelerometer and the"));
    Serial.println(F(" gyroscope share their ODR, so the setting here must be the same as in the DIY_Calibration_Gyroscope sketch."));
    Serial.println(F(" Place the board on a horizontal surface with one of its axes vertical and hit enter to start a calibration"));
    Serial.println(F(" measurement. Each of the axes must be measured pointing up and pointing down, so a total of 6 measurements."));
    Serial.println(F(" The program recognises which axis is vertical and shows which were measured successfully. If the angle is to"));
    Serial.println(F(" far oblique the measurement is not valid.\n  ")); 
    Serial.println(F(" (enter)  Start a calibration measurement. "));
    Serial.print  (F("   (N)    Number of calibration samples "));Serial.println(NofCalibrationSamples);
    Serial.print  (F("   (F)    Full Scale setting "));Serial.print(accelFSindex);Serial.print(" = ");Serial.print(IMU.getAccelFS(),0);Serial.println(F("g"));
    Serial.print  (F("   (R)    Output Data Rate (ODR) setting "));Serial.print(accelODRindex);Serial.print(" = ");Serial.print(IMU.getAccelODR(),0);Serial.println(F("Hz (actual value)"));
    
//    Serial.println("Press (X) to exit \n");

    Serial.print(F(" Measured status of axis \n "));
    for (int i=0;i<=2;i++){  Serial.print(xyz[i]); if (bitRead(acceMMlOK,i)==1)Serial.print("+ = ( -OK- ) "); else Serial.print("+ = not done "); }
    Serial.print("\n ");
    for (int i=0;i<=2;i++){  Serial.print(xyz[i]); if (bitRead(acceMMlOK,i+3)==1)Serial.print("- = ( -OK- ) "); else Serial.print("- = not done "); }
   
 //   Serial.println(F("\n\nCurrent  accelerometer calibration values (copy/paste-able)\n"));
    Serial.println(F("\n\n   // Accelerometer code"));
    Serial.print(F("   IMU.setAccelFS(")); Serial.print(accelFSindex);
    Serial.print(F(");\n   IMU.setAccelODR("));Serial.print(accelODRindex);Serial.println(");");

    printSetParam("   IMU.setAccelOffset",IMU.accelOffset);
    Serial.println();
    printSetParam ("   IMU.setAccelSlope ",IMU.accelSlope); 
    Serial.println("\n\n");
    incomingByte= readChar();                          //wait for and get keyboard input
    switch (incomingByte)
    { case  'F': { Serial.print (F("\n\nEnter new FS nr  0: ±2g ; 1: ±24g ; 2: ±4g ; 3: ±8g > ")); 
                  b= readChar()-48; Serial.println(b);
                  if (b!=accelFSindex && b >=0 && b<=3) accelFSindex=b;
                  IMU.setAccelFS(accelFSindex); 
                  Serial.print("\n\n\n\n\n\n\n\n\n"); 
                  break;}
      case 'R': { Serial.print (F("\n\nEnter new ODR nr   1:10,2:50 3:119,4:238,5:476 Hz > ")); 
                  b= readChar()-48; //Serial.println(b);
                  if (b!=accelODRindex && b>=1 && b<=5) accelODRindex=b;
                  IMU.setAccelODR(accelODRindex);
                  Serial.print("\n\n\n\n\n\n\n\n\n"); 
                  break;
                }  
      case 'N': { readAnswer("\n\n\n\n\n\nThe number of calibration samples ", NofCalibrationSamples);
                  break;}
      case 'C': {};        
      default :   calibrateAccel(NofCalibrationSamples);
    }  
  }
}

void calibrateAccel(uint16_t NofSamples)
{  boolean validMmt=false;
   float x,y,z;
   Serial.println(F("\n\n\n\n\n\n\n\n\n\n\n")); 
   Serial.println(F("measuring \n")); 
//   IMU.setAccelODR(5);  //476 Hz
   raw_N_Accel(NofSamples,x,y,z);
   if (abs(x)>max(abs(y),abs(z)))
   {    Serial.println(F("X detected"));  
       if (sqrt(y*y+z*z)/x<accelCriterion)
       {  validMmt= true;
          if (x>0) {maxAX=x; 
                    acceMMlOK=acceMMlOK | 0b00000001 ;}
          else     {minAX=x; 
                    acceMMlOK=acceMMlOK | 0b00001000 ; }
       }
   }
   if (abs(y)>max(abs(x),abs(z)))
   {  Serial.println(F("Y detected"));  
      if (sqrt(x*x+z*z)/y<accelCriterion)
       {  validMmt= true;
          if (y>0) {maxAY=y; 
                    acceMMlOK=acceMMlOK | 0b00000010 ; }
          else     {minAY=y; 
                    acceMMlOK=acceMMlOK | 0b00010000 ; }
       }  
   }
   if (abs(z)>max(abs(x),abs(y)))
   {  Serial.println(F("Z detected"));  
      if ( sqrt(x*x+y*y)/z<accelCriterion)
       {  validMmt= true;
          if (z>0) {maxAZ=z; 
                    acceMMlOK=acceMMlOK | 0b00000100 ; }
          else     {minAZ=z;
                    acceMMlOK=acceMMlOK | 0b00100000 ; }
       }  
   }
   IMU.setAccelOffset((maxAX+minAX)/2,(maxAY+minAY)/2,(maxAZ+minAZ)/2);
   IMU.setAccelSlope ((maxAX-minAX)/2,(maxAY-minAY)/2,(maxAZ-minAZ)/2);
   if (acceMMlOK==0b00111111) accelOK = true;
   
   if ( !validMmt ) 
    { Serial.print(F("\n\n\nNot a valid measurement!  "));
      Serial.println(" x=");Serial.print(x);Serial.print("  y=");Serial.print(y);Serial.println("  z=");Serial.print(z);
    }
}

char readChar()
{  char ch;
   while (!Serial.available()) ;             // wait for character to be entered
   ch= toupper(Serial.read());   
   delay(10);
   while (Serial.available()){Serial.read();delay(1);} // empty readbuffer 
   return ch;
}

void readAnswer(char msg[], uint16_t& param)
{ char ch=0;
  byte count=0;
  const byte NofChars = 8;
  char ans[NofChars];
  while (Serial.available()){Serial.read();}  //empty read buffer
  Serial.print(msg); 
  Serial.print(param); 
  Serial.print(F(" Enter new value ")); 
  while (byte(ch)!=10 && byte(ch)!=13 && count<(NofChars-1)   )
  {  if (Serial.available())
     {  ch= Serial.read();
        ans[count]=ch;
        count++;
     }
  }      
  ans[count]=0;
  Serial.println(ans);
  if (count>1) param= atoi(ans);
  while (Serial.available()){Serial.read();}
     Serial.println(F("\n\n\n\n\n\n\n")); 
}

void raw_N_Accel(uint16_t N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     averX=0; averY =0;averZ =0;
     for (int i=1;i<=N;i++)
     {  while (!IMU.accelAvailable());
        IMU.readRawAccel(x, y, z);
        averX += x;    averY += y;     averZ += z;
        digitalWrite(LED_BUILTIN, (millis()/125)%2);       // blink onboard led every 250ms
        if ((i%30)==0)Serial.print('.'); 
     } 
     averX /= N;    averY /= N;     averZ /= N;
     digitalWrite(LED_BUILTIN,0);                          // led off
}

