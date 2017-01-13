/*
 * SoftI2CMaster.cpp -- Multi-instance software I2C Master library
 * 
 * 
 * 2010-12 Tod E. Kurt, http://todbot.com/blog/
 *
 * This code takes some tricks from:
 *  http://codinglab.blogspot.com/2008/10/i2c-on-avr-using-bit-banging.html
 *
 * 2014, by Testato: update library and examples for follow Wire’s API of Arduino IDE 1.x
 *
 */

#include <stdio.h> // for function sprintf
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "SoftI2CMaster.h"

#include <util/delay.h>
#include <string.h>

#define  i2cbitdelay 50

#define  I2C_ACK  1 
#define  I2C_NAK  0


#define i2c_scl_release()                 \
    *_sclDirReg     &=~ _sclBitMask
#define i2c_sda_release()                 \
    *_sdaDirReg     &=~ _sdaBitMask

// sets SCL low and drives output
#define i2c_scl_lo()                                 \
                     *_sclPortReg  &=~ _sclBitMask;  \
                     *_sclDirReg   |=  _sclBitMask; 

// sets SDA low and drives output
#define i2c_sda_lo()                                 \
                     *_sdaPortReg  &=~ _sdaBitMask;  \
                     *_sdaDirReg   |=  _sdaBitMask;  

// set SCL high and to input (releases pin) (i.e. change to input,turnon pullup)
#define i2c_scl_hi()                                 \
                     *_sclDirReg   &=~ _sclBitMask;  \
    if(usePullups) { *_sclPortReg  |=  _sclBitMask; } 

// set SDA high and to input (releases pin) (i.e. change to input,turnon pullup)
#define i2c_sda_hi()                                 \
                     *_sdaDirReg   &=~ _sdaBitMask;  \
    if(usePullups) { *_sdaPortReg  |=  _sdaBitMask; } 


//
// Constructor
//
SoftI2CMaster::SoftI2CMaster()
{
    // do nothing, use setPins() later
}
//
SoftI2CMaster::SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin) 
{
    setPins(sclPin, sdaPin, true);
    i2c_init();
}

//
SoftI2CMaster::SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin, uint8_t pullups)
{
    setPins(sclPin, sdaPin, pullups);
    i2c_init();
}

SoftI2CMaster::SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin, uint8_t pullups, uint8_t rstPin, uint8_t csPin)
{
  setPins(sclPin, sdaPin, pullups);
  i2c_init();
  _csPin = csPin;
  _rstPin = rstPin;
  _chipType = 0;
  pinMode(_csPin, OUTPUT); digitalWrite(_csPin, HIGH);
  pinMode(_rstPin, OUTPUT); digitalWrite(_rstPin, LOW);
}

//
// Turn Arduino pin numbers into PORTx, DDRx, and PINx
//
void SoftI2CMaster::setPins(uint8_t sclPin, uint8_t sdaPin, uint8_t pullups)
{
    uint8_t port;
    
    usePullups = pullups;

    _sclPin = sclPin;
    _sdaPin = sdaPin;
    
    _sclBitMask = digitalPinToBitMask(sclPin);
    _sdaBitMask = digitalPinToBitMask(sdaPin);
    
    port = digitalPinToPort(sclPin);
    _sclPortReg  = portOutputRegister(port);
    _sclDirReg   = portModeRegister(port);

    port = digitalPinToPort(sdaPin);
    _sdaPortReg  = portOutputRegister(port);
    _sdaDirReg   = portModeRegister(port);
    
}

//
//
//

uint8_t SoftI2CMaster::pwd_ackpolling( uint8_t pollcmd, uint8_t* errctx)
{
  uint8_t pollcnt = 0;
  boolean passmatch = false;
  while((pollcnt < 10) && (!passmatch))
  {
    delay(10);
    i2c_start();
    if (i2c_write(pollcmd) == 0) passmatch = true;
    pollcnt++;
  }
  if (!passmatch) 
  {
    sprintf(_tempstr, "%s : Password not match, no ack recved.\n", errctx);
    Serial.print(_tempstr);
    return(1); 
  }
  return 0;
}

uint8_t SoftI2CMaster::sendcmdpwd( uint8_t cmd1, uint8_t cmd2, uint8_t* errctx)
{
  //
  uint8_t errno = 0, act_cnt = 0;
  while((errno == 0) && (act_cnt < 10) )
  {
    switch(act_cnt)
    {
      case 0: if (i2c_write(cmd1) == 1) errno = 1; break;
      case 1: if (i2c_write(cmd2) == 1) errno = 2; break;
      case 2: if (i2c_write(_pwd1[0]) == 1) errno = 3; break;
      case 3: if (i2c_write(_pwd1[1]) == 1) errno = 4; break;
      case 4: if (i2c_write(_pwd1[2]) == 1) errno = 5; break;
      case 5: if (i2c_write(_pwd1[3]) == 1) errno = 6; break;
      case 6: if (i2c_write(_pwd2[0]) == 1) errno = 7; break;
      case 7: if (i2c_write(_pwd2[1]) == 1) errno = 8; break;
      case 8: if (i2c_write(_pwd2[2]) == 1) errno = 9; break;
      case 9: if (i2c_write(_pwd2[3]) == 1) errno = 10; break;
    }
    act_cnt++;
  }
  if (errno > 0) 
  {
    sprintf(_tempstr, "%s : Error %d occured!\n", errctx, errno);
    Serial.print(_tempstr); 
  }
  return(errno);
}

void SoftI2CMaster::readiso()
{
  i2c_scl_lo();
  digitalWrite(_csPin, LOW);
  _delay_us(i2cbitdelay);

  digitalWrite(_rstPin, HIGH);
  _delay_us(i2cbitdelay);
  i2c_scl_hi();
  _delay_us(i2cbitdelay);
  i2c_scl_lo();
  _delay_us(i2cbitdelay);
  digitalWrite(_rstPin, LOW);
  
  for ( uint8_t j=0;j<4;j++) _isodata[j] = read8bitLSB();
  digitalWrite(_csPin, HIGH);

  //sprintf(_tempstr, "ISO = %02X %02X %02X %02X\n", _isodata[0], _isodata[1], _isodata[2], _isodata[3]);
  //Serial.print(_tempstr);
  
  switch(*((uint32_t*)_isodata))
    {
      case 0x55AA5519: Serial.println("X76F041 detected. Proceed..."); _chipType = 1; break;
      case 0x55AA0019: Serial.println("X76F100 detected. Not handled yet, Halted."); _chipType = 2; while(1); break;
      default: Serial.println("Trying detect ZS01..."); _chipType = 15; break;
    }

  if (_chipType != 15) return;
  //reset ala ZS01
  digitalWrite(_csPin, LOW);  
  _delay_us(i2cbitdelay);
  digitalWrite(_rstPin, HIGH); //zs reset is active LOW
  _delay_us(i2cbitdelay);
  i2c_scl_hi();
  _delay_us(i2cbitdelay);
  i2c_scl_lo();
  _delay_us(i2cbitdelay);
  for ( uint8_t j=0;j<4;j++) _isodata[j] = read8bitMSB();
  _delay_us(i2cbitdelay);
  digitalWrite(_csPin, HIGH);

  //sprintf(_tempstr, "ISO = %02X %02X %02X %02X\n", _isodata[0], _isodata[1], _isodata[2], _isodata[3]);
  //Serial.print(_tempstr);
  if (*((uint32_t*)_isodata) == 0x0100535A) {Serial.println("ZS01 detected. Proceed..."); _chipType = 3; }
    else {Serial.println("Nothing detected. Halted."); while(1); }
  
}

void SoftI2CMaster::setPwd(uint32_t pwd1, uint32_t pwd2)
{
  *((uint32_t*)_pwd1) = pwd1;
  *((uint32_t*)_pwd2) = pwd2;
} //i2c.setPwd(0xBAB51195, 0x3067F085); //tx seq = 95 11 b5 ba 85 f0 67 30

uint8_t SoftI2CMaster::readwithPwd(uint16_t address)
{
  //sequential random read can only take place within 80h block
  //block 0 = 000, block 1 = 080h and so on
  //this read function take 2 kind of address, addr after cmd, and addr after pwd match ack.
  //addr after pwd ack, MUST within 1kbit block/80h of addr after cmd.
  //so addr param should be only one of these 4 values = 000h, 080h, 100h, and 180h ??

  uint16_t cmdaddr = 0x6000; //read with pwd cmd (conf read)
  cmdaddr |= address;

  digitalWrite(_csPin, LOW);
  i2c_init();
  i2c_start();
  
  uint8_t errctx[] = "Read with Pwd";
  if (sendcmdpwd(highByte(cmdaddr), lowByte(cmdaddr), errctx) > 0) return 1;
  if (pwd_ackpolling(0xC0, errctx) == 1) return 2; //ack polling
  
  read8bitMSB();//to be discarded
  
  uint16_t blockcnt = 0;
  i2c_start();
  if (i2c_write(0) == 1) {Serial.println("Read with Pwd : NACK in addr after pwd ack"); return(2);}
  while(blockcnt < 0x7F)
  {
    sprintf(_tempstr, "%02X", i2c_read(1) );
    Serial.print(_tempstr);
    blockcnt++;
    if ((blockcnt % 8) == 0) Serial.println(""); else Serial.print(" ");
  }
  // read last byte differently
  sprintf(_tempstr, "%02X", read8bitMSB() ); Serial.print(_tempstr);
  i2c_sda_lo(); //needed here because readbit sets sda high
  i2c_stop();

  digitalWrite(_csPin, HIGH);
  Serial.println("");
  
  return 0;
}

uint8_t SoftI2CMaster::readconfreg()
{
  digitalWrite(_csPin, LOW);
  i2c_start();

  uint8_t errctx[] = "Read Conf Reg";
  if (sendcmdpwd(0x80, 0x60, errctx) > 0) return 1;
  if (pwd_ackpolling(0xC0, errctx) == 1) return 2; //ack polling
  
  Serial.println("Configuration Registers :");
  sprintf(_tempstr, "ACR1 = %02X\n", i2c_read(1) ); Serial.print(_tempstr);
  sprintf(_tempstr, "ACR2 = %02X\n", i2c_read(1) ); Serial.print(_tempstr);
  sprintf(_tempstr, "CR = %02X\n", i2c_read(1) ); Serial.print(_tempstr);
  sprintf(_tempstr, "RR = %02X\n", i2c_read(1) ); Serial.print(_tempstr);
  sprintf(_tempstr, "RC = %02X\n", i2c_read(1) ); Serial.print(_tempstr);
  i2c_stop();
  delay(10);
  
  digitalWrite(_csPin, HIGH);
}

uint8_t SoftI2CMaster::massprogram()
{
  //
  digitalWrite(_csPin, LOW);
  i2c_start();
  //80 70
  uint8_t errctx[] = "Mass Program";
  if (sendcmdpwd(0x80, 0x70, errctx) > 0) return 1;
  if (pwd_ackpolling(0xC0, errctx) == 1) return 2; //ack polling

  i2c_sda_lo(); //needed here because readbit sets sda high
  i2c_stop();
  delay(10);
  digitalWrite(_csPin, HIGH);
  Serial.println("Mass program chip succeed.");
}

uint8_t SoftI2CMaster::zs_cmd(uint8_t cmd, uint8_t blocknum, uint8_t* databytes)
{
  byte buffer[12], tbuf[12], rbuf[12];
  buffer[0] = cmd;  buffer[1] = blocknum;
  for ( uint8_t i=0;i<8;i++) buffer[2+i] = databytes[i];

  uint16_t bcrc = zscalc_crc(buffer, 10);
  buffer[10] = highByte(bcrc); buffer[11] = lowByte(bcrc);
  
  //sprintf(_tempstr, "\nraw: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", 
  //      buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11]);
  //Serial.print(_tempstr);  
  
  if ((cmd & 4) == 4) 
    {
      zs_encrypt2(&buffer[2], &buffer[2], 8, _zsDatKey, _zsPrevByte);
      //sprintf(_tempstr, "ec2: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X, PB=%02X\n", 
      //  buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11],_zsPrevByte);
      //Serial.print(_tempstr);
    };
    
  zs_encrypt(tbuf, buffer, 12, _zsCmdKey, 0xFF);
  
  //sprintf(_tempstr, "enc: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", 
  //      tbuf[0],tbuf[1],tbuf[2],tbuf[3],tbuf[4],tbuf[5],tbuf[6],tbuf[7],tbuf[8],tbuf[9],tbuf[10],tbuf[11]);
  //Serial.print(_tempstr);  
  
  //send tbuf
  digitalWrite(_csPin, LOW);  
  delay(10); //using _delay_us not enough, causing NACK while send tbuf

  i2c_start();
  for ( uint8_t i=0;i<11;i++) if (i2c_write(tbuf[i]) == 1) {Serial.print("NACK while send tbuf : "); Serial.println(i, HEX); return 0; }; 
  i2c_write_noackclk(tbuf[11]); //send last byte with no ack clk

  //wait or ack poll, but how?
  //Serial.println("sent, wait response...");
  delay(100);
  
  //recv
  i2c_start(); //it's actually not a start, but...
  for ( uint8_t i=0;i<11;i++) rbuf[i] = i2c_read(I2C_ACK);
  rbuf[11] = i2c_read_noackclk();
  i2c_stop();

  _delay_us(i2cbitdelay);
  digitalWrite(_csPin, HIGH);  
  
  //sprintf(_tempstr, "recv: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", 
  //      rbuf[0],rbuf[1],rbuf[2],rbuf[3],rbuf[4],rbuf[5],rbuf[6],rbuf[7],rbuf[8],rbuf[9],rbuf[10],rbuf[11]);
  //Serial.print(_tempstr);
  
  zs_decrypt(buffer, rbuf, 12, databytes, 0xFF);
  bcrc = zscalc_crc(buffer, 10);
  if (!((buffer[10] == highByte(bcrc)) && (buffer[11] == lowByte(bcrc)))) 
    {
      Serial.println("dec: bad crc! trying zerokey ...");
      zs_decrypt(buffer, rbuf, 12, _zsZeroKey, 0xFF); //correct prev byte?
      bcrc = zscalc_crc(buffer, 10);
      if (!((buffer[10] == highByte(bcrc)) && (buffer[11] == lowByte(bcrc)))) 
        {
          Serial.println("dec: bad crc again! give up..");
        
          zs_decrypt(buffer, rbuf, 12, _zsCmdKey, 0xFF); //correct prev byte?
          bcrc = zscalc_crc(buffer, 10);
          if (!((buffer[10] == highByte(bcrc)) && (buffer[11] == lowByte(bcrc)))) 
            {
              Serial.println("dec: bad crc again! give up.");
              return 0;
            }
          
          return 0;
        }
    }
  
  //sprintf(_tempstr, "dec: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", 
  //      buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11]);
  //Serial.print(_tempstr);  
  sprintf(_tempstr, "%02X: %02X %02X %02X %02X %02X %02X %02X %02X ,status = %02X %02X ", 
        blocknum, buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[0],buffer[1]);
  Serial.print(_tempstr);  
  //if (buffer[0] == 0) _zsPrevByte = buffer[1];
  _zsPrevByte = buffer[1];
  switch(buffer[0])
  {
    case 0: Serial.println("Status OK"); break;
    case 2: Serial.println("Wrong datakey/Retry Counter reached?"); break;
    case 5:  break;
  }
}

void SoftI2CMaster::zsSetPwd(uint8_t* cmdkey, uint8_t* datkey, uint8_t* seskey)
{
  for ( uint8_t i=0;i<8;i++)
  {
    _zsCmdKey[i] = cmdkey[i]; 
    _zsDatKey[i] = datkey[i];
    _zsSesKey[i] = seskey[i];
    _zsZeroKey[i] = 0;
  }
}
uint8_t SoftI2CMaster::zsread(uint8_t blocknum)
{
  //01 xx : read data at, suppose able to read 0 - 3 only
  zs_cmd(1, blocknum, _zsSesKey);
}
uint8_t SoftI2CMaster::zsconfread(uint8_t blocknum)
{
  //read(conf mode) data at, 0 - 13
  zs_cmd(5, blocknum, _zsSesKey);
}
uint8_t SoftI2CMaster::zsconfwrite(uint8_t blocknum, uint8_t* datablock)
{
  //write(conf mode) data at   
  //4 FF = write new datakey
  //4 FE = write new conf registers
  //
  zs_cmd(4, blocknum, datablock);
}

uint8_t SoftI2CMaster::beginTransmission(uint8_t address)
{
    i2c_start();
    uint8_t rc = i2c_write((address<<1) | 0); // clr read bit
    return rc;
}

//
uint8_t SoftI2CMaster::requestFrom(uint8_t address)
{
    i2c_start();
    uint8_t rc = i2c_write((address<<1) | 1); // set read bit
    return rc;
}
//
uint8_t SoftI2CMaster::requestFrom(int address)
{
    return requestFrom( (uint8_t) address);
}

//
uint8_t SoftI2CMaster::beginTransmission(int address)
{
    return beginTransmission((uint8_t)address);
}

//
//
//
uint8_t SoftI2CMaster::endTransmission(void)
{
    i2c_stop();
    //return ret;  // FIXME
    return 0;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
uint8_t SoftI2CMaster::write(uint8_t data)
{
    return i2c_write(data);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void SoftI2CMaster::write(uint8_t* data, uint8_t quantity)
{
    for(uint8_t i = 0; i < quantity; ++i){
        write(data[i]);
    }
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void SoftI2CMaster::write(char* data)
{
    write((uint8_t*)data, strlen(data));
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void SoftI2CMaster::write(int data)
{
    write((uint8_t)data);
}

//--------------------------------------------------------------------


void SoftI2CMaster::i2c_writebit( uint8_t c )
{
    if ( c > 0 ) {
        i2c_sda_hi();
    } else {
        i2c_sda_lo();
    }

    i2c_scl_hi();
    _delay_us(i2cbitdelay);

    i2c_scl_lo();
    _delay_us(i2cbitdelay);

    if ( c > 0 ) {
        i2c_sda_lo();
    }
    _delay_us(i2cbitdelay);
}

//
uint8_t SoftI2CMaster::i2c_readbit(void)
{
    i2c_sda_hi();
    i2c_scl_hi();
    _delay_us(i2cbitdelay);

    uint8_t port = digitalPinToPort(_sdaPin);
    volatile uint8_t* pinReg = portInputRegister(port);
    uint8_t c = *pinReg;  // I2C_PIN;

    i2c_scl_lo();
    _delay_us(i2cbitdelay);

    return ( c & _sdaBitMask) ? 1 : 0;
}

// Inits bitbanging port, must be called before using the functions below
//
void SoftI2CMaster::i2c_init(void)
{
    //I2C_PORT &=~ (_BV( I2C_SDA ) | _BV( I2C_SCL ));
    //*_sclPortReg &=~ (_sdaBitMask | _sclBitMask);
    i2c_sda_hi();
    //i2c_scl_hi();
    i2c_scl_lo();
    
    _delay_us(i2cbitdelay);
}

// Send a START Condition
//
void SoftI2CMaster::i2c_start(void)
{
    // set both to high at the same time
    //I2C_DDR &=~ (_BV( I2C_SDA ) | _BV( I2C_SCL ));
    //*_sclDirReg &=~ (_sdaBitMask | _sclBitMask);
    i2c_sda_hi();
    i2c_scl_hi();

    _delay_us(i2cbitdelay);
   
    i2c_sda_lo();
    _delay_us(i2cbitdelay);

    i2c_scl_lo();
    _delay_us(i2cbitdelay);
}

void SoftI2CMaster::i2c_repstart(void)
{
    // set both to high at the same time (releases drive on both lines)
    //I2C_DDR &=~ (_BV( I2C_SDA ) | _BV( I2C_SCL ));
    //*_sclDirReg &=~ (_sdaBitMask | _sclBitMask);
    i2c_sda_hi();
    i2c_scl_hi();

    i2c_scl_lo();                           // force SCL low
    _delay_us(i2cbitdelay);

    i2c_sda_release();                      // release SDA
    _delay_us(i2cbitdelay);

    i2c_scl_release();                      // release SCL
    _delay_us(i2cbitdelay);

    i2c_sda_lo();                           // force SDA low
    _delay_us(i2cbitdelay);
}

// Send a STOP Condition
//
void SoftI2CMaster::i2c_stop(void)
{
    i2c_scl_hi();
    _delay_us(i2cbitdelay);

    i2c_sda_hi();
    _delay_us(i2cbitdelay);
    
    i2c_scl_lo();
    _delay_us(i2cbitdelay);
}

// write a byte to the I2C slave device
//
uint8_t SoftI2CMaster::i2c_write( uint8_t c )
{
    i2c_write_noackclk(c);
    return i2c_readbit();
}

void SoftI2CMaster::i2c_write_noackclk(uint8_t c)
{
    for ( uint8_t i=0;i<8;i++) {
        i2c_writebit( c & 128 );
        c<<=1;
    }
}

uint8_t SoftI2CMaster::i2c_read_noackclk(void)
{
    uint8_t res = 0;
    for ( uint8_t i=0;i<8;i++) {
        res <<= 1;
        res |= i2c_readbit();  
    }
    _delay_us(i2cbitdelay);
    return res;
}

// read a byte from the I2C slave device
//
uint8_t SoftI2CMaster::i2c_read( uint8_t ack )
{
    uint8_t res = 0;
    for ( uint8_t i=0;i<8;i++) {
        res <<= 1;
        res |= i2c_readbit();  
    }

    if ( ack ) i2c_writebit( 0 ); else i2c_writebit( 1 );
    _delay_us(i2cbitdelay);
    return res;
}

// FIXME: this isn't right, surely
uint8_t SoftI2CMaster::read( uint8_t ack )
{
  return i2c_read( ack );
}

//
uint8_t SoftI2CMaster::read()
{
    return i2c_read( I2C_ACK );
}

//
uint8_t SoftI2CMaster::readLast()
{
    return i2c_read( I2C_NAK );
}

uint8_t SoftI2CMaster::read8bitMSB()
{
  uint8_t a = 0;
  for ( uint8_t i=0;i<8;i++) 
  {
    a <<= 1;
    a |= i2c_readbit();
  }
  return a;
}

uint8_t SoftI2CMaster::read8bitLSB()
{
  uint8_t a = 0;
  for ( uint8_t i=0;i<8;i++) {a |= i2c_readbit() << i; }
  return a;
}

uint16_t SoftI2CMaster::zscalc_crc(uint8_t* buffer, uint8_t length)
{
  //
  uint32_t v1, a3, v0, a2;
  v1 = 0xffff;  a3 = 0;
  if( length > 0 )
  {
  do
    {
      v0 = buffer[ a3 ];  a2 = 7;  v0 = v0 << 8;  v1 = v1 ^ v0;  v0 = v1 & 0x8000;
      do
        {
	  if( v0 != 0 )
	    { v0 = v1 << 1;  v1 = v0 ^ 0x1021; } else {v0 = v1 << 1; v1 = v1 << 1;}
	  a2--;
	  v0 = v1 & 0x8000;
        } while( (signed) a2 >= 0 );
      a3++;
      v0 = (signed) a3 < (signed) length;
    } while ( v0 != 0 );
  }
  v0 = ~v1 ;
  //v0 = v0 & 0xffff;
  return (uint16_t)v0;
}

void SoftI2CMaster::zs_encrypt( uint8_t *destination, uint8_t *source, int length, uint8_t *key, uint32_t previous_byte )
{
  uint32_t t0, v0, v1, a0, a1;
  length--;
  if( length >= 0 )
    {
      do
	{
	  t0 = 1; v0 = source[ length ]; v1 = previous_byte; a0 = key[ 0 ]; v0 ^= v1; a0 += v0;
	  do
	    {
	      a1 = key[ t0 ]; t0++;  a0 &= 0xff;  v0 = a1 >> 5;  v1 = a0 << v0;
	      v0 = 8 - v0;  v0 &= 7;  a0 = (signed) a0 >> v0;
	      v1 |= a0;  v1 &= 0xff;  a1 &= 0x1f;  v1 += a1;
	      v0 = (signed) t0 < 8; a0 = v1;
            } while( v0 != 0 );

	  previous_byte = v1; destination[ length ] = a0; length--;
	} while( length >= 0 );
    }
}

void SoftI2CMaster::zs_encrypt2( uint8_t *destination, uint8_t *source, int length, uint8_t *key, uint32_t previous_byte )
{
	uint32_t t0, v0, v1, a0, a1, t2; t2 = 0;
	if( length >= 0 )
	{
		do
		{
			t0 = 1;	v0 = source[ t2 ]; v1 = previous_byte; a0 = key[ 0 ]; v0 ^= v1; a0 += v0;
			do
			{
				a1 = key[ t0 ];	t0++; a0 &= 0xff; v0 = a1 >> 5;	v1 = a0 << v0;
				v0 = 8 - v0; v0 &= 7; a0 = (signed) a0 >> v0;	v1 |= a0;
				v1 &= 0xff; a1 &= 0x1f;	v1 += a1; v0 = (signed) t0 < 8; a0 = v1;
			} while( v0 != 0 );

			previous_byte = v1; destination[ t2 ] = a0; t2++;
		} while( t2 < length );
	}
}

void SoftI2CMaster::zs_decrypt( uint8_t *destination, uint8_t *source, int length, uint8_t *key, uint8_t previous_byte )
{
	uint32_t a0, v1, v0, a1, t1, t0;
	length--;
	if( length >= 0 )
	{
		do
		{
			t1 = source[ length ];	a1 = 7;	t0 = t1;
			do
			{
				v1 = key[ a1 ];	a1--; v0 = v1 & 0x1f; v0 = t0 - v0;
				v1 >>= 5; v0 &= 0xff; a0 = (signed)v0 >> v1; v1 = 8 - v1; v1 &= 7;
				v0 = (signed)v0 << v1; t0 = a0 | v0;
			} while( a1 > 0 );

			v1 = key[ 0 ]; a0 = previous_byte; v0 = t0 & 0xff; previous_byte = t1;
			v0 = v0 - v1; v0 = v0 ^ a0; destination[ length ] = v0;	length--;
		}
		while( length >= 0 );
	}
}

void SoftI2CMaster::zs_decrypt2( uint8_t *destination, uint8_t *source, int length, uint8_t *key, uint8_t previous_byte )
{
	uint32_t a0, v1, v0, a1, t2, t1, t0;	t2 = 0;
	if( length >= 0 )
	{
		do
		{
			t1 = source[ t2 ]; a1 = 7; t0 = t1;
			do
			{
				v1 = key[ a1 ];	a1--;	v0 = v1 & 0x1f;	v0 = t0 - v0;
				v1 >>= 5; v0 &= 0xff;	a0 = (signed)v0 >> v1;	v1 = 8 - v1; v1 &= 7;
				v0 = (signed)v0 << v1;	t0 = a0 | v0;
			} while( a1 > 0 );
			v1 = key[ 0 ]; a0 = previous_byte; v0 = t0 & 0xff; previous_byte = t1;	v0 = v0 - v1;	v0 = v0 ^ a0;
			destination[ t2 ] = v0;	t2++;
		} while( t2 < length );
	}
}

