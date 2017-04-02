#include <SPI.h>

byte mcp3912_cs = 10;
byte mcp3912_dr = 2;

#define MCP3912_CH0     0x00
#define MCP3912_CH1     0x01
#define MCP3912_CH2     0x02
#define MCP3912_CH3     0x03
#define MCP3912_MOD     0x08
#define MCP3912_PHASE   0x0A
#define MCP3912_GAIN    0x0B
#define MCP3912_STATCOM 0x0C
#define MCP3912_CONFIG0 0x0D
#define MCP3912_CONFIG1 0x0E
#define MCP3912_OFF0    0x0F
#define MCP3912_GC0     0x10
#define MCP3912_OFF1    0x11
#define MCP3912_GC1     0x12
#define MCP3912_OFF2    0x13
#define MCP3912_GC2     0x14
#define MCP3912_OFF3    0x15
#define MCP3912_GC3     0x16
#define MCP3912_LOCK    0x1F

void mcp3912_write_reg(byte reg, long val24)
{
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (reg<<1) | 0;
  SPI.transfer(treg);
  SPI.transfer((val24>>16)&0xFF);
  SPI.transfer((val24>>8)&0xFF);
  SPI.transfer(val24&0xFF);
  digitalWrite(mcp3912_cs, 1);
}

long mcp3912_read_reg(byte reg)
{
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (reg<<1) | 1;
  SPI.transfer(treg);
  byte b1 = 0;//SPI.transfer(0);
  byte b2 = SPI.transfer(0);
  byte b3 = SPI.transfer(0);
  byte b4 = SPI.transfer(0);
  digitalWrite(mcp3912_cs, 1);
  return (b1<<24) | (b2<<16) | (b3<<8) | b4;
}
long mcp3912_read_reg32(byte reg)
{
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (reg<<1) | 1;
  SPI.transfer(treg);
  byte b1 = SPI.transfer(0);
  byte b2 = SPI.transfer(0);
  byte b3 = SPI.transfer(0);
  byte b4 = SPI.transfer(0);
  digitalWrite(mcp3912_cs, 1);
  return (b1<<24) | (b2<<16) | (b3<<8) | b4;
}

void mcp3912_init()
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  pinMode(mcp3912_cs, OUTPUT);
  pinMode(mcp3912_dr, INPUT);
  digitalWrite(mcp3912_cs, 1);

  long statcom, conf0, conf1, lock;
//  statcom = (0b10111000<<16) | (0b10<<8) | 0b0; //types group, auto inc write, dr high, dr linked, 16crc, 16 bit resolution
  statcom = (0b10111001<<16) | (0b0<<8) | 0b0; //types group, auto inc write, dr high, dr linked, 16crc, 16 bit resolution
//  conf0 = (0b00111110<<16) | (0b10100000<<8) | 0x50; //976 sps
  conf0 = (0b00111110<<16) | (0b10100000<<8) | 0x50; 
  conf1 = 0;
  lock = 0xA5<<16;
//  mcp3912_write_reg(MCP3912_LOCK, lock);
  mcp3912_write_reg(MCP3912_STATCOM, statcom);
  mcp3912_write_reg(MCP3912_CONFIG0, conf0);
  mcp3912_write_reg(MCP3912_CONFIG1, conf1);
}

int adc_ch0 = 0, adc_ch1 = 0, adc_ch2 = 0, adc_ch3 = 0;

void get_mcp_data()
{
  long v1 = mcp3912_read_reg32(MCP3912_CH0);
  long v2 = mcp3912_read_reg32(MCP3912_CH1);
  long v3 = mcp3912_read_reg32(MCP3912_CH2);
  long v4 = mcp3912_read_reg32(MCP3912_CH3);
  adc_ch0 = v1;//(v1>>8)&0xFFFF;
  adc_ch1 = v2;//(v2>>8)&0xFFFF;
  adc_ch2 = v3;//(v3>>8)&0xFFFF;
  adc_ch3 = v4;//(v4>>8)&0xFFFF;
}

void get_mcp_data_t()
{
//  mcp3912_read_reg(MCP3912_STATCOM);
  digitalWrite(mcp3912_cs, 0);
  byte treg = ((0b01)<<6) | (MCP3912_CH0<<1) | 1;
  SPI.transfer(treg);
  byte b1 = SPI.transfer(0);
  byte b2 = SPI.transfer(0);
  byte b24 = 0;
  if(b24) SPI.transfer(0);
  adc_ch0 = (b1<<8) | b2;
  b1 = SPI.transfer(0);
  b2 = SPI.transfer(0);
  if(b24) SPI.transfer(0);
  adc_ch1 = (b1<<8) | b2;
  b1 = SPI.transfer(0);
  b2 = SPI.transfer(0);
  if(b24) SPI.transfer(0);
  adc_ch2 = (b1<<8) | b2;
  b1 = SPI.transfer(0);
  b2 = SPI.transfer(0);
  if(b24) SPI.transfer(0);
  adc_ch3 = (b1<<8) | b2;
  digitalWrite(mcp3912_cs, 1);
  return ;  
}

byte mcp_data_ready()
{
  long st = mcp3912_read_reg(MCP3912_STATCOM);
  byte str = st&0b01111;
  return (str == 0);
}

void setup() {
  mcp3912_init();
  Serial.begin(115200);
}

byte cnt = 0;

byte record[32];

void loop() {
  cnt++;
  byte data_cnt = 0;
  while(mcp_data_ready() < 1) ;

  get_mcp_data();

  byte v1_h = adc_ch0>>8;
  byte v1_l = adc_ch0&0xFF;
  byte v2_h = adc_ch1>>8;
  byte v2_l = adc_ch1&0xFF;
  byte v3_h = adc_ch2>>8;
  byte v3_l = adc_ch2&0xFF;
  byte v4_h = adc_ch3>>8;
  byte v4_l = adc_ch3&0xFF;

  int idx = 1;
  record[idx+0] = v1_h;
  record[idx+1] = v1_l;
  record[idx+2] = v2_h;
  record[idx+3] = v2_l;
  record[idx+4] = v3_h;
  record[idx+5] = v3_l;
  record[idx+6] = v4_h;
  record[idx+7] = v4_l;
  record[0] = 200;
  record[idx+8] = 201;
  Serial.write(record, idx+9);
  
}
