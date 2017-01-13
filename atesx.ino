
const int chipsel = 8; //active low cs
const int chipres = 7;
const int sdapin = 2;
const int sclpin = 4;

/*
byte commandkey[8] = {0xED, 0x68, 0x50, 0x4B, 0xC6, 0x44, 0x48, 0x3E};
//byte datakey[8]    = {0xE0, 0x76, 0x83, 0x05, 0x64, 0x80, 0x4F, 0xD1}; //mamboagg
byte datakey[8]    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //cleared datakey by retry counter overflow?
//byte datakey[8]    = {0xE6, 0x7F, 0xE0, 0x2E, 0x47, 0xDB, 0xCB, 0xF5}; //perc3rd mix
byte seskey[8]     = {0x13, 0x70, 0x4D, 0x1B, 0xF9, 0x9C, 0xD5, 0x5E}; //for test only
byte datatemp[8]   = {0x01, 0x02, 0x03, 0x04, 0x50, 0x60, 0x70, 0x80};
byte datatemp2[8]  = {0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x80};
*/

#include "SoftI2CMaster.h"

void setup() {
  SoftI2CMaster i2c = SoftI2CMaster(sclpin, sdapin, false, chipres, chipsel);
  Serial.begin(9600);
  delay(4000);
  Serial.println("Secure chip test ...");
  i2c.readiso();
/*  
  i2c.zsSetPwd(commandkey, datakey, seskey);
  i2c.zsread(0);
  i2c.zsread(1);
  i2c.zsread(2);
  i2c.zsread(3);
  i2c.zsread(0xFC);
  i2c.zsread(0xFD);
  i2c.zsconfread(0xFE);
  i2c.zsconfread(4);
  i2c.zsconfread(5);
  i2c.zsconfread(6);
  i2c.zsconfread(7);
  i2c.zsconfread(8);
  i2c.zsconfread(9);
  i2c.zsconfread(10);
  i2c.zsconfread(11);
  i2c.zsconfread(12);
  i2c.zsconfread(13); 
  */
  //i2c.zsconfwrite(0, datatemp);
  //i2c.zsread(0);
  //i2c.zsconfwrite(0xFF, datatemp);
  //i2c.zsconfread(0xFE);  
  
  
  //i2c.setPwd(0, 0);
  //byte a = i2c.readwithPwd(0);
  //i2c.setPwd(0xBAB51195, 0x3067F085); //DrumMania GQ881
  //i2c.setPwd(0xF698EC9B, 0x73136E84); //Solo 2000 GC905JA
  //i2c.setPwd(0xA6D84E60, 0x71221A93); //Solo 2000 GE905JA
  //i2c.setPwd(0xBCF15C1A, 0x43015D5C); //3rd Mix 887JA
  //i2c.setPwd(0x3E89C319 ,0xE7EB9488); //2nd Mix 895JA
  //i2c.setPwd(0xF3205AE7, 0x788172FE); //1st Mix 845JA
  //i2c.setPwd(0x3E89C319, 0xE7EB9488 ); //4th Mix A33
  //i2c.setPwd(0x0D243B7E, 0x7DB2D271); //GuitarFreak 3rd Mix 949
  //i2c.setPwd(0x5C907431, 0x83609566); //GFreak GQ886
  i2c.setPwd(0x48A4F2CD, 0x02882AFB); //1st Mix 845AA
  //i2c.setPwd(0xBAB51195, 0x3067F085); //Perc3rd mix
  //i2c.setPwd(0x3E89C319, 0xE7EB9488); //3rd Mix GE887JA
  //i2c.setPwd(0x48A4F2CD, 0x02882AFB); //3rd Mix GE887KA
  //i2c.setPwd(0xBCF15C1A, 0x43015D5C); //3rd Mix GC887KA
  
  //byte a = i2c.readwithPwd(0);
  //a = i2c.readwithPwd(0x80);
  //a = i2c.readwithPwd(0x100);
  //a = i2c.readwithPwd(0x180);
  //byte a = i2c.readconfreg();
  byte a = i2c.massprogram();
  Serial.println("Done.");
}

void loop() {
  
}


