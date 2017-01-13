/*
 * SoftI2CMaster.h -- Multi-instance software I2C Master library
 * 
 * 2010-2012 Tod E. Kurt, http://todbot.com/blog/
 * 2014, by Testato: update library and examples for follow Wire’s API of Arduino IDE 1.x
 *
 */

#ifndef SoftI2CMaster_h
#define SoftI2CMaster_h

#include <inttypes.h>

#define _SOFTI2CMASTER_VERSION 13  // software version of this library


class SoftI2CMaster
{

private:
  // per object data
  uint8_t _sclPin;
  uint8_t _sdaPin;
  uint8_t _csPin;
  uint8_t _rstPin;
  uint8_t _sclBitMask;
  uint8_t _sdaBitMask;
  volatile uint8_t *_sclPortReg;
  volatile uint8_t *_sdaPortReg;
  volatile uint8_t *_sclDirReg;
  volatile uint8_t *_sdaDirReg;
  uint8_t _isodata[4];
  uint8_t _pwd1[4];
  uint8_t _pwd2[4];
  char _tempstr[50];
  uint8_t _chipType;
  uint8_t _zsCmdKey[8];
  uint8_t _zsDatKey[8];
  uint8_t _zsSesKey[8];
  uint8_t _zsZeroKey[8];
  uint8_t _zsPrevByte;

  uint8_t usePullups;
  
  // private methods

  uint16_t zscalc_crc(uint8_t* buffer, uint8_t length);
  void zs_encrypt(  uint8_t *destination, uint8_t *source, int length, uint8_t *key, uint32_t previous_byte );
  void zs_encrypt2( uint8_t *destination, uint8_t *source, int length, uint8_t *key, uint32_t previous_byte );
  void zs_decrypt(  uint8_t *destination, uint8_t *source, int length, uint8_t *key, uint8_t previous_byte );
  void zs_decrypt2( uint8_t *destination, uint8_t *source, int length, uint8_t *key, uint8_t previous_byte );
  uint8_t zs_cmd(uint8_t cmd, uint8_t blocknum, uint8_t* databytes);
  
  void i2c_writebit( uint8_t c );
  uint8_t i2c_readbit(void);
  void i2c_init(void);
  void i2c_start(void);
  void i2c_repstart(void);
  void i2c_stop(void);
  uint8_t i2c_write( uint8_t c );
  uint8_t i2c_read( uint8_t ack );
  void i2c_write_noackclk(uint8_t c);
  uint8_t i2c_read_noackclk(void);
  uint8_t sendcmdpwd( uint8_t cmd1, uint8_t cmd2, uint8_t* errctx);
  uint8_t pwd_ackpolling( uint8_t pollcmd, uint8_t* errctx);
  
public:
  // public methods
  SoftI2CMaster();
  SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin);
  SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin, uint8_t usePullups);
  SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin, uint8_t usePullups, uint8_t rstPin, uint8_t csPin);

  void setPins(uint8_t sclPin, uint8_t sdaPin, uint8_t usePullups);

  void readiso();
  void setPwd(uint32_t pwd1, uint32_t pwd2); //64bit pwd
  uint8_t readwithPwd(uint16_t address); //read A byte from address
  uint8_t readconfreg(); //read configuration register using configuration pwd
  uint8_t massprogram(); //set all to 0, all parts shipped mass programmed
  
  void zsSetPwd(uint8_t* cmdkey, uint8_t* datkey, uint8_t* seskey);
  uint8_t zsread(uint8_t blocknum); //read data at, suppose able to read 0 - 3 only
  uint8_t zsconfread(uint8_t blocknum); //read(conf mode) data at, 0 - 13 ?
  uint8_t zsconfwrite(uint8_t blocknum, uint8_t* datablock); //write(conf mode) data at    
  
  uint8_t beginTransmission(uint8_t address);
  uint8_t beginTransmission(int address);
  uint8_t endTransmission(void);
  uint8_t write(uint8_t);
  void write(uint8_t*, uint8_t);
  void write(int);
  void write(char*);
  void begin(void) {return;};
  uint8_t requestFrom(int address);
  uint8_t requestFrom(uint8_t address);
  uint8_t read( uint8_t ack );
  uint8_t read();
  uint8_t readLast();
  uint8_t read8bitMSB();
  uint8_t read8bitLSB();

};

#endif
