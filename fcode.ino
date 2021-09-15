/*
----------------------------------------------------------------------------------------------------------------------------------------------------
													
													ARDUINO CODE FOR PARACHUTE DEPLOYMENT
													
----------------------------------------------------------------------------------------------------------------------------------------------------													

    pin configuration
 *  1. BMP280
 *        scl-7
 *        MISO(SDA)-6
 *        MOSI(SDD)-5
 *        CSB-3
 *  2.MPU6500
 *        sck-13
 *        MISO(SDA)-12
 *        MOSI(ADO)-11
 *        CS-10
 *   3.SD card module
 *        sck -13
 *        MISO-12
 *        MOSI -11
 *        CS-4
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>

// Declaring pins for BMP280

#define BMP_SCK  (7)
#define BMP_MISO (6)
#define BMP_MOSI (5)
#define BMP_CS   (3)


const int chipSelectForSDcard = 4;//for SD card module
const int chipSelectForMPU = 10;//for MPU 6500

double accelX,accelY,accelZ;
float gForceX,gForceY,gForceZ;

Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
void matrixAdd(int r,int c,double m1[][2],double m2[][2],double res[][2])
{
  int i,j;
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=0;
    }
  }
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=m1[i][j]+m2[i][j];
    }
  }
  return ; 
}

void matrixSubtract(int r,int c,double m1[][2],double m2[][2],double res[][2])
{
  int i,j;
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=m1[i][j]-m2[i][j];
    }
  }  
}

void matrixMultiply(int r1, int c1, int r2, int c2,double m1[][2], double m2[][2], double res[][2])
{
  int i, j, k;
  for(i = 0; i < r1; ++i){
     for(j = 0; j < c2; ++j){
        res[i][j] = 0;
     }
  }
  for(i = 0; i < r1; ++i){
    for(j = 0; j < c2; ++j){
      for(k=0; k<c1; ++k){
        res[i][j] += m1[i][k] * m2[k][j];
      }
    }
  }

}
void matrixTranspose(int r, int c, double m1[][2],double m2[][2])
{
    int i,j;
    for(i = 0; i < r; ++i){
     for(j = 0; j < c; ++j){
        m2[i][j] = 0;
     }
  }
    for(i=0;i<r;i++){
      for(j=0;j<c;j++){
        m2[j][i]=m1[i][j];
      }
    }  
}
double temp1[2][2]={0,0,0,0};
double temp2[2][2]={0,0,0,0};
double temp3[2][2]={0,0,0,0};
double temp4[2][2]={0,0,0,0};
double temp5[2][2]={0,0,0,0};
double temp6[2][2]={0,0,0,0};
double temp7[2][2]={0,0,0,0};
double temp8[2][1]={0,0};
double temp9[2][1]={0,0};
double temp10[2][2]={0,0,0,0};
double temp11[2][2]={0,0,0,0};
double dt = 0.02;
double x[2][2]={0.74,0,0,0};
double F[2][2]={1,dt,0,1};
double Ft[2][2]={0,0,0,0};
double B[2][2]={0.5*dt*dt,0,dt,0};
double H[2][2]={1,0,0,0};
double Ht[2][2]={0,0,0,0};
double Kt[2][2]={0,0,0,0};
double Q[2][2]={0.4,0.4,0.4,4};
double R[2][2]={0.4,0,0,0};
double y[2][2]={0,0,0,0};
double S[2][2]={0,0,0,0};
double I[2][2]={{1,0},{0,1}};
double K[2][2]={0,0,0,0};
double verticalAccln[2][2]={0,0,0,0};
double currentAltitude[2][2]={0,0,0,0};
double P[2][2]={{1,0},{0,1}};

void setup(){
  Serial.begin(115200);
  Serial.println(F("Kalman filter code test"));

  // Activating IMU
  SPI.begin();

  // Checking for BMP
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  
  // Checking for SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelectForSDcard)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");

  pinMode(chipSelectForMPU, OUTPUT);

  //Initialising BMP
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500
                  ); /* Standby time. */

  // Initialising IMU registers
  writeRegister(0x1C, 0x18);
  writeRegister(0x6B, 0x00);
  delay(100);

  for(int numberOfIteration=0;numberOfIteration<=1000;numberOfIteration++){
    //Temperature
    Serial.print(bmp.readTemperature());
    Serial.print(" ");

    //Pressure  
    Serial.print(bmp.readPressure());
    Serial.print(" ");
    
    //Altitude
    Serial.print(bmp.readAltitude(1013.25));
    Serial.print(" ");
    writeRegister(0x6A, 0x10);
    
    //Acceleration data
    accelX = readRegister(0x3B)<<8|readRegister(0x3C); //Store first two bytes into accelX
    accelY = readRegister(0x3D)<<8|readRegister(0x3E); //Store middle two bytes into accelY
    accelZ = readRegister(0x3F)<<8|readRegister(0x40); //Store last two bytes into accelZ
    
    //processing Accelerometer data
    processAccelData();

    //Accln along x  
    Serial.print(gForceX);
    Serial.print(" ");
    
    //Accln along y 
    Serial.print(gForceY);
    Serial.print(" ");

    //Accln along z 
    Serial.print(gForceZ);
    Serial.print(" ");

    //---------------------------------------------------------------------------------------------------
    //                                            KALMAN FILTERING                            
    //---------------------------------------------------------------------------------------------------

    // Initialising variables
    
    verticalAccln[0][0]=gForceZ;
    currentAltitude[0][0]=bmp.readAltitude(1013.25);
    
    // STEP 1 Prediction
    
    matrixMultiply(2,2,2,1,F,x,temp9);
    matrixMultiply(2,1,1,1,B,verticalAccln,temp8);
    matrixAdd(2,1,temp9,temp8,x);
    matrixMultiply(2,2,2,2,F,P,temp1);
    matrixTranspose(2,2,F,Ft);
    matrixMultiply(2,2,2,2,temp1,Ft,temp2);
    matrixAdd(2,2,temp2,Q,P);
    
    //STEP 2:Updation
    
    matrixMultiply(1,2,2,1,H,x,temp11);
    matrixSubtract(1,1,currentAltitude,temp11,y);
    matrixTranspose(1,2,H,Ht);
    matrixMultiply(2,2,2,1,P,Ht,temp9);
    matrixMultiply(1,2,2,1,H,temp9,temp11);
    matrixAdd(1,1,R,temp11,S);
    S[0][0]=1/S[0][0];
    matrixMultiply(2,1,1,1,temp9,S,K);
    matrixMultiply(2,1,1,1,K,y,temp8);
    matrixAdd(2,1,x,temp8,temp9);
    x[0][0]=temp9[0][0];
    x[0][1]=temp9[0][1];
    matrixMultiply(2,1,1,2,K,H,temp1);
    matrixSubtract(2,2,I,temp1,temp2);
    matrixMultiply(2,2,2,2,temp2,P,temp3);
    matrixMultiply(2,1,1,2,K,H,temp4);
    matrixSubtract(2,2,I,temp4,temp5);
    matrixTranspose(2,2,temp5,temp7);
    matrixMultiply(2,2,2,2,temp3,temp7,temp6);
    matrixMultiply(2,1,1,1,K,R,temp9);
    matrixTranspose(2,1,K,Kt);
    matrixMultiply(2,1,1,2,temp9,Kt,temp10);
    matrixAdd(2,2,temp6,temp10,P);
    Serial.print(" ");
    //Serial.print(".........");
    Serial.print(x[0][0]);
    Serial.print(" ");
    //Serial.print(".........");
    Serial.println(x[0][0]*3.28084);
    


    String dataString = "";
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.print(bmp.readTemperature());
      dataFile.print(" ");
      dataFile.print(bmp.readPressure());
      dataFile.print(" ");
      
      dataFile.print(bmp.readAltitude(1013.25));
      dataFile.print(" ");
      dataFile.print(gForceX);
      dataFile.print(" ");
      dataFile.print(gForceY);
      dataFile.print(" ");
      dataFile.print(gForceZ);
      dataFile.print(" ");
      dataFile.println(x[0][0]);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    }
  }  
}


// No Code in loop
void loop()
{
  
}
void processAccelData()
{
  if(accelX>32768)
  {
    gForceX = 65535 - accelX;
    gForceX /= 2048.0;
   }
   else
  {
    gForceX = -accelX/2048.0;
  }
  if(accelY>32768)
  {
    gForceY = 65535 - accelY;
    gForceY /= 2048.0;
  }
  else
  {
    gForceY = -accelY/2048.0;
  }
  if(accelZ>32768)
  {
    gForceZ = 65535 - accelZ;
    gForceZ /= 2048.0;
  }
  else
  {
    gForceZ= -accelZ/2048.0;
  }
}
//Read from or write to register from the SCP1000:
unsigned int readRegister(byte thisRegister)
{
  byte dataToSend = thisRegister | 0x80;
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  // take the chip select low to select the device:
  digitalWrite(chipSelectForMPU, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  byte result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  // take the chip select high to de-select:
  digitalWrite(chipSelectForMPU, HIGH);
  // release control of the SPI port
  SPI.endTransaction();
  return(result);
}

//Sends a write command to MPU6500
void writeRegister(byte thisRegister, byte thisValue) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  // take the chip select low to select the device:
  digitalWrite(chipSelectForMPU, LOW);
  SPI.transfer(thisRegister); //Send register location
  SPI.transfer(thisValue); //Send value to record into register
  // take the chip select high to de-select:
  digitalWrite(chipSelectForMPU, HIGH);
  // release control of the SPI port
  SPI.endTransaction();
}
