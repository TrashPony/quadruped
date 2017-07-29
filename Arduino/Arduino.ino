// BasicLinearAlgebra - Version: Latest 
#include <math.h>
#include <Wire.h>
#include <Multiservo.h>
#define PI 3.1415926535897932384626433832795

Multiservo rBotCenter;
Multiservo rFrontCenter;
Multiservo rBotMid;
Multiservo rFrontMid;

Multiservo rBotPalm;
Multiservo rFrontPalm;
Multiservo lBotCenter;
Multiservo lFrontCenter;

Multiservo lBotMid;
Multiservo lFrontMid;
Multiservo lBotPalm;
Multiservo lFrontPalm;

int rBC=91,rFC=91,lBC=91,lFC=91,rBM=91,rFM=91,lBM=91,lFM=91,rBP=91,rFP=91,lBP=91,lFP=91;


void setup() {
    Serial.begin(9600);
    rBotCenter.attach(0);
    rFrontCenter.attach(1);
    rBotMid.attach(6);
    rFrontMid.attach(7);
    
    rBotPalm.attach(12);
    rFrontPalm.attach(13);
    lBotCenter.attach(2);
    lFrontCenter.attach(3);
    
    lBotMid.attach(8);
    lFrontMid.attach(9);
    lBotPalm.attach(14);
    lFrontPalm.attach(15);
}

void loop() {
  Serial.println("in programm");
  //[x,y,z],[x,y,z],[x,y,z],[x,y,z]
  sentAngle(1,5,6, 1,5,6, 1,5,6, 1,5,6);
  sentAngle(1,13,6, 1,13,6, 1,13,6, 1,13,6);
  sentAngle(9,9,6, 9,9,6, 9,9,6, 9,9,6);
  sentAngle(13,1,6, 13,1,6, 13,1,6, 13,1,6);
  sentAngle(5,1,6, 5,1,6, 5,1,6, 5,1,6);
}

void sentAngle(float x, float y, float z, float x2, float y2, float z2, float x3, float y3, float z3, float x4, float y4, float z4){
  float leg1[] = {x,y,z};
  float leg2[] = {x2,y2,z2};
  float leg3[] = {x3,y3,z3};
  float leg4[] = {x4,y4,z4};
  
  mathIK(leg1[0],leg1[1],leg1[2]);
  mathIK(leg2[0],leg2[1],leg2[2]);
  mathIK(leg3[0],leg3[1],leg3[2]);
  mathIK(leg4[0],leg4[1],leg4[2]);

  int yAngle[] = {int(round(leg1[0])),int(round(leg2[0])),int(round(leg3[0])),int(round(leg4[0]))};
  int aAngle[] = {int(round(leg1[1])),int(round(leg2[1])),int(round(leg3[1])),int(round(leg4[1]))};
  int bAngle[] = {int(round(leg1[2])),int(round(leg2[2])),int(round(leg3[2])),int(round(leg4[2]))};
  writeServo(yAngle,aAngle,bAngle);
}

void mathIK(float &x,float &y, float &zOffset){
  ////////////константы/////////////////
  float coxa    = 4; //(длина таза в см)
  float femur   = 5; //(длина бедра в см)
  float tibia   = 10.5; //(длина голени в см)
  ////////////константы/////////////////

  float xy    = x/y;
  float Y     = atan(xy) * 180/PI;
  float yProj = y/cos(Y * PI/180);
  float yCoxa = yProj-coxa;
  float L     = sqrt(pow(zOffset, 2) + pow((yCoxa), 2));
  float a1    = acos(zOffset/L) * 180/PI;
  float b     = (acos((pow(femur, 2) + pow(tibia, 2) - pow(L, 2))/(2 * tibia * femur )) * 180/PI);
  if(b < 0) b = b+180;
  float lamb  = (acos((pow(L, 2) + pow(tibia, 2) - pow(femur, 2))/(2 * tibia * L )) * 180/PI);
  float a2    = 180 - (abs(b)+lamb);
  float a     = a1+a2;
  if(x < 0) Y = abs(Y)+90;
  //////////////////////////////////////
  float angle[] = {Y, 180 - a, b};
  x = angle[0];
  y = angle[1];
  zOffset = angle[2];
}

void writeServo(int y[],int a[],int b[]) {
  bool flagY1 = false;
  bool flagY2 = false;
  bool flagY3 = false;
  bool flagY4 = false;
  bool flagA1 = false;
  bool flagA2 = false;
  bool flagA3 = false;
  bool flagA4 = false;
  bool flagB1 = false;
  bool flagB2 = false;
  bool flagB3 = false;
  bool flagB4 = false;
  while( flagY1 != true || flagA1 != true || flagB1 != true ||
         flagY2 != true || flagA2 != true || flagB2 != true ||
         flagY3 != true || flagA3 != true || flagB3 != true ||
         flagY3 != true || flagA3 != true || flagB3 != true)
  {
    delay(1);
/////////////////////////////////////////////////////////////////////////////
    if(rBC != y[0]){
      if(rBC < y[0]){
        rBC++;
        rBotCenter.write(rBC);
      }
      if(rBC > y[0]){
        rBC--;
        rBotCenter.write(rBC);
      }
    } else {
        flagY1 = true;
      }

    if(rFC != y[1]){
      if(rFC < y[1]){
        rFC++;
        rFrontCenter.write((rFC - 180) * (-1));
      }
      if(rFC > y[1]){
        rFC--;
        rFrontCenter.write((rFC - 180) * (-1));
      }
    } else {
        flagY2 = true;
      }

    if(lBC != y[2]){
      if(lBC < y[2]){
        lBC++;
        lBotCenter.write((lBC - 180) * (-1));
      }
      if(lBC > y[2]){
        lBC--;
        lBotCenter.write((lBC - 180) * (-1));
      }
    } else {
        flagY3 = true;
      }

    if(lFC != y[3]){
      if(lFC < y[3]){
        lFC++;
        lFrontCenter.write(lFC);
      }
      if(lFC > y[3]){
        lFC--;
        lFrontCenter.write(lFC);
      }
    } else {
        flagY4 = true;
      }
/////////////////////////////////////////////////////////////////////////////
    if(rBM != a[0]){
      if(rBM < a[0]){
        rBM++;
        rBotMid.write(rBM);
      }
      if(rBM > a[0]){
        rBM--;
        rBotMid.write(rBM);
      }
    } else {
        flagA1 = true;
      }

    if(rFM != a[1]){
      if(rFM < a[1]){
        rFM++;
        rFrontMid.write((rFM - 180) * (-1));
      }
      if(rFM > a[1]){
        rFM--;
        rFrontMid.write((rFM - 180) * (-1));
      }
    } else {
        flagA2 = true;
      }

    if(lBM != a[2]){
      if(lBM < a[2]){
        lBM++;
        lBotMid.write((lBM - 180) * (-1));
      }
      if(lBM > a[2]){
        lBM--;
        lBotMid.write((lBM - 180) * (-1));
      }
    } else {
        flagA3 = true;
      }

    if(lFM != a[3]){
      if(lFM < a[3]){
        lFM++;
        lFrontMid.write(lFM);
      }
      if(lFM > a[3]){
        lFM--;
        lFrontMid.write(lFM);
      }
    } else {
        flagA4 = true;
      }
/////////////////////////////////////////////////////////////////////////////
    if(rBP != b[0]){
      if(rBP < b[0]){
        rBP++;
        rBotPalm.write(rBP);
      }
      if(rBP > b[0]){
        rBP--;
        rBotPalm.write(rBP);
      }
    } else {
        flagB1 = true;
      }

    if(rFP != b[1]){
      if(rFP < b[1]){
        rFP++;
        rFrontPalm.write((rFP - 180) * (-1));
      }
      if(rFP > b[1]){
        rFP--;
        rFrontPalm.write((rFP - 180) * (-1));
      }
    } else {
        flagB2 = true;
      }

    if(lBP != b[2]){
      if(lBP < b[2]){
        lBP++;
        lBotPalm.write(lBP);
      }
      if(lBP > b[2]){
        lBP--;
        lBotPalm.write(lBP);
      }
    } else {
        flagB3 = true;
      }

    if(lFP != b[3]){
      if(lFP < b[3]){
        lFP++;
        lFrontPalm.write(lFP);
      }
      if(lFP > b[3]){
        lFP--;
        lFrontPalm.write(lFP);
      }
    } else {
      flagB4 = true;
    }
  }
}
