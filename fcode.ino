/* pin configuration
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
#define BMP_SCK  (7)
#define BMP_MISO (6)
#define BMP_MOSI (5)
#define BMP_CS   (3)
const int chipSelect = 4;//for SD card module
const int chipSelectPin = 10;//for MPU 6500
double accelX,accelY,accelZ;
float gForceX,gForceY,gForceZ;
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
void Add1(int r,int c,double m1[][1],double m2[][1],double res[][1])
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
void Add2(int r,int c,double m1[][2],double m2[][2],double res[][2])
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
void Sub1(int r,int c,double m1[][1],double m2[][1],double res[][1])
{
  
  int i,j;
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=0;
    }
  }
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=m1[i][j]-m2[i][j];
    }
  }

}
void Sub2(int r,int c,double m1[][2],double m2[][2],double res[][2])
{
  int i,j;
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=m1[i][j]-m2[i][j];
    }
  }  
}

void MUL1(int r1, int c1, int r2, int c2,double m1[][1], double m2[][1], double res[][1])
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
void MUL3(int r1, int c1, int r2, int c2,double m1[][2], double m2[][1], double res[][1])
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
void MUL2(int r1, int c1, int r2, int c2,double m1[][1], double m2[][2], double res[][2])
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

void MUL4(int r1, int c1, int r2, int c2,double m1[][2], double m2[][2], double res[][2])
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
void transpose1(int r, int c, double m1[][1],double m2[][2])
{
    int i,j;
    for(i=0;i<r;i++){
      for(j=0;j<c;j++){
        m2[i][j]=0;
      }
    }
    for(i=0;i<r;i++){
      for(j=0;j<c;j++){
        m2[i][j]=m1[j][i];
      }
    }  
}
void transpose2(int r, int c, double m1[][2],double m2[][1])
{
    int i,j;
    
    for(i=0;i<r;i++){
      for(j=0;j<c;j++){
        m2[j][i]=m1[i][j];
      }
    }  
}
void transpose3(int r, int c, double m1[][2],double m2[][2])
{
    int i,j;
    
    for(i=0;i<r;i++){
      for(j=0;j<c;j++){
        m2[j][i]=m1[i][j];
      }
    }  
}
static double altitude=0,acceleration=0,altitude_new=0;
    int a=0;
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
double temp11[1][1]={0};
double dt = 0.02;
double x[2][1]={{0.74},{0}};
double F[2][2]={1,0.02,0,1};
double Ft[2][2]={0,0,0,0};
double B[2][1]={{0.5*dt*dt},{dt}};
double H[1][2]={1,0};
double Ht[2][1]={0,0};
double Kt[1][2]={0,0};
double Q[2][2]={0.4,0.4,0.4,4};
double R[1][1]={0.4};
double y[1][1]={0};
double S[1][1]={0};
double I[2][2]={{1,0},{0,1}};
double K[2][1]={0,0};
double u[1][1]={0};
double z[1][1]={0};
double P[2][2]={{1,0},{0,1}};

void setup(){
  Serial.begin(115200);
  Serial.println(F("Kalman filter code test"));
  SPI.begin();
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  pinMode(chipSelectPin, OUTPUT);
  //Configure SCP1000 for low noise configuration:
  writeRegister(0x1C, 0x18);
  writeRegister(0x6B, 0x00);
  // writeRegister(0x68, 0x07);
  // writeRegister(0x1D, 0x08);
  // give the sensor time to set up:
  delay(100);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    for(int b=0;b<=1000;b++){
    //Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.print(" ");
    //Serial.println(" *C");
    //Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.print(" ");
    //Serial.print(" Pa");
    Serial.print(bmp.readAltitude(1013.25));
    writeRegister(0x6A, 0x10);
    // if ((readRegister(0x3A)&(0x01)) == 0x01)
    // {
    accelX = readRegister(0x3B)<<8|readRegister(0x3C); //Store first two bytes into accelX
    accelY = readRegister(0x3D)<<8|readRegister(0x3E); //Store middle two bytes into accelY
    accelZ = readRegister(0x3F)<<8|readRegister(0x40); //Store last two bytes into accelZ
    processAccelData();
    //Serial.print(" Accel (g)");
    //Serial.print(" X=");
    
    Serial.print(gForceX);
    Serial.print(" ");
    //Serial.print(" Y=");
    Serial.print(gForceY);
    //Serial.print(" Z=");
    Serial.print(" ");
    Serial.print(gForceZ);
    //step 1:Prediction
    u[0][0]=gForceX;
    z[0][0]=bmp.readAltitude(1013.25);
    MUL3(2,2,2,1,F,x,temp9);
    MUL1(2,1,1,1,B,u,temp8);
    Add1(2,1,temp9,temp8,x);
    MUL4(2,2,2,2,F,P,temp1);
    transpose3(2,2,F,Ft);
    MUL4(2,2,2,2,temp1,Ft,temp2);
    Add2(2,2,temp2,Q,P);
  
  //step 2:UpdationM
  MUL3(1,2,2,1,H,x,temp11);
  Sub1(1,1,z,temp11,y);
  transpose2(1,2,H,Ht);
  MUL3(2,2,2,1,P,Ht,temp9);
  MUL3(1,2,2,1,H,temp9,temp11);
  Add1(1,1,R,temp11,S);
  S[0][0]=1/S[0][0];
  MUL1(2,1,1,1,temp9,S,K);
  MUL1(2,1,1,1,K,y,temp8);
  Add1(2,1,x,temp8,temp9);
  x[0][0]=temp9[0][0];
  x[0][1]=temp9[0][1];
  MUL2(2,1,1,2,K,H,temp1);
  Sub2(2,2,I,temp1,temp2);
  MUL4(2,2,2,2,temp2,P,temp3);
  MUL2(2,1,1,2,K,H,temp4);
  Sub2(2,2,I,temp4,temp5);
  transpose3(2,2,temp5,temp7);
  MUL4(2,2,2,2,temp3,temp7,temp6);
  MUL1(2,1,1,1,K,R,temp9);
  transpose1(2,1,K,Kt);
  MUL2(2,1,1,2,temp9,Kt,temp10);
  Add2(2,2,temp6,temp10,P);
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
digitalWrite(chipSelectPin, LOW);
// send the device the register you want to read:
SPI.transfer(dataToSend);
// send a value of 0 to read the first byte returned:
byte result = SPI.transfer(0x00);
// decrement the number of bytes left to read:
// take the chip select high to de-select:
digitalWrite(chipSelectPin, HIGH);
// release control of the SPI port
SPI.endTransaction();
return(result);
}
//Sends a write command to MPU6500
void writeRegister(byte thisRegister, byte thisValue) {
SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
// take the chip select low to select the device:
digitalWrite(chipSelectPin, LOW);
SPI.transfer(thisRegister); //Send register location
SPI.transfer(thisValue); //Send value to record into register
// take the chip select high to de-select:
digitalWrite(chipSelectPin, HIGH);
// release control of the SPI port
SPI.endTransaction();
}
