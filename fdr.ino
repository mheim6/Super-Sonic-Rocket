
#include <SPI.h>

#include <Adafruit_BMP280.h>

/*
 * flight data recorder
 * 
 * on startup: start collecting data into a rolling buffer in RAM
 *   to avoid wearing out NVM.
 * 
 * during setup: accumulate an average altitude used for launch trigger
 *   go to setup once enough data is collecte for the average altitude
 *   
 * during prelaunch: continue filling buffer in RAM
 *   once altitude is above the launch trigger for enough sequential samples
 *   go to post launch
 *   
 * during post launch: continue collecting data into the rolling buffer.
 *   as long as there is enough data to fill a 1k buffer
 *   copy it from the rolling buffer to NVM
 *   once NVM is full, go to FULL
 *   
 * during FULL: continue collecting data, but do not save it
 * 
 * during manual stop : stop storing data
 *   
 * Command Line:
 *   the usb serial port is monitored for command (general single characters)
 *   Importsant commands:
 *   i -- display some interesting information (also checks computer status)
 *   C -- clear NVM
 *   L -- clear NVM and go to startup mode (Launch)
 *   M -- dump data from NVM over the serial port in CSV format
 *   D -- toggle display of "fast" data (press, altitude, accel, oritntation
 *   G -- toggle display of GPS data
 *   R -- toggle display of radio status
 *   
 *   others:
 *   x80000yb -- check if page1 of NVM is blank (should see FFFF at FFFFC)
 *   
 *   
 * need to calibrate BNO055 on each power up
 * 
 * consider calculating base pressure at launch 
 * instead of fixed 103500 value use measured pressure + 1000 
 */

#include <Adafruit_Sensor.h>

#include <Adafruit_BNO055.h>

// #include <Adafruit_BMP280.h>


Adafruit_BNO055 bno = Adafruit_BNO055(25);
Adafruit_BMP280 bmp;

// actual Teensy3.6 board mounted LED is on pin 13
// however that is used for SPI clock to LoRa module
// might be able to share function
// However for now just redirect it to a spare pin
int led = 14;
 
// operating modes
enum mode {STARTUP, SETUP, PRE_LAUNCH,POST_LAUNCH, FULL,
            MANUAL_STOP};
int mode = STARTUP;

#define SETUP_SAMPLES 20
// 5*51 (5 1k blocks, see below)
#define NUM_PRE_SAMPLES 255
// note buffer must fit within 1K byte
#define NUM_BUF_SAMPLES 51
// number of 1k buffers to save in NVM
#define NUM_POST_BLKS 200
#define FAST_SAMPLE_PERIOD 50

// define data block pointer to first 1k of FlexRAM
# define FLEX_RAM ((dblk*)0x14000000)

# define address of NVM page 1
#define NVM (0x80000)
unsigned nvmReady = 0;

//NVM memory performance

unsigned int nvmWriteStart;
unsigned int nvmWriteMaxTime;
unsigned int nvmCount;
unsigned int nvmAvg;

float alt, ax, ay, az, ox, oy, oz;

// calculating launch trigger
float setup_sum = 0;
int num_setup = 0;
float launch_trigger = 100000; // set high until setup complete
float launch_offset = 50;  // offset from setup altitude
#define LAUNCH_CONFIRM_COUNT 5
unsigned int launch_time = 0;
// base_press (nominally 103500)
int base_press = 103500;


//store data from fast sensors (10-20 hz)
struct fastBlk {
  int tag; // 'F', ms since launch (24 bit)
  short alt; // 0.1 meters
  short p; // base_press - p(pascal)

  short ax; // 0.1 meter/sec^2 about 0.1 g
  short ay;
  short az;

  short ox; // 0.1 deg/sec
  short oy;
  short oz;
};


void show_fastBlk(struct fastBlk* blk) {
  Serial.print("fast block time: ");
  Serial.print((int)(0xFFFFFF & (blk->tag >> 8)));
  Serial.print(" ms  altitude: ");
  Serial.print(blk->alt/10.0,1);
  Serial.println(" m");
  Serial.print("press: ");
  Serial.print(base_press-(blk->p));
  Serial.println(" pascal");
  Serial.print("acceleration m/s^2 x: ");
  Serial.print(blk->ax/10.0,1);
  Serial.print("  ay: ");
  Serial.print(blk->ay/10.0,1);
  Serial.print("  az:");
  Serial.println(blk->az/10.0,1);
  Serial.print("roll x: ");
  Serial.print(blk->ox/10.0,2);
  Serial.print(" deg/sec   y: ");
  Serial.print(blk->oy/10.0,2);
  Serial.print(" deg/sec  z: ");
  Serial.print(blk->oz/10.0,2);
  Serial.println(" deg/sec");
}

void dump_fastData(struct fastBlk* blk){
  Serial.print("fast,");
  Serial.print((int)(0xFFFFFF & (blk->tag >> 8)));
  Serial.print(",");
  Serial.print(blk->alt/10.0,1);
  Serial.print(",");
  Serial.print(base_press-(blk->p));
  Serial.print(",");
  Serial.print(blk->ax/100.0,2);
  Serial.print(",");
  Serial.print(blk->ay/100.0,2);
  Serial.print(",");
  Serial.print(blk->az/100.0,2);
  Serial.print(",");
  Serial.print(blk->ox/10.0,2);
  Serial.print(",");
  Serial.print(blk->oy/10.0,2);
  Serial.print(",");
  Serial.println(blk->oz/10.0,2);
}



// store data from slow sensors
struct slowBlk {
  int tag;
  float lat;
  float lon;
  float alt;
  float gpstim;
};


void show_slowBlk (struct slowBlk* blk) {
  Serial.print("slow block time: ");
  Serial.print((int)(0xFFFFFF & (blk->tag >> 8)));
  Serial.print(" ms  Lat: ");
  Serial.print(blk->lat,4);
  Serial.print(" min  Long: ");
  Serial.print(blk->lon);
  Serial.print(" min   altutude: ");
  Serial.print(blk->alt,1);
  Serial.print(" m  gps time: ");
  Serial.print(blk->gpstim);
  Serial.println(" sec");
}


void dump_slowData(struct slowBlk* blk){
 Serial.print("slow,");
  Serial.print((int)(0xFFFFFF & (blk->tag >> 8)));
  Serial.print(",");
  Serial.print(blk->lat,4);
  Serial.print(",");
  Serial.print(blk->lon,4);
  Serial.print(",");
  Serial.print(blk->alt,1);
  Serial.print(",");
  Serial.println(blk->gpstim);

}


  
union dblk {
  fastBlk fast;
  slowBlk gps;
};


// BNO055 stuff


void bnoPage0() {
  bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 0);
}

void bnoPage1() {
  bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 1);
}

// read one byte from a reg (bit 8 is page number)
int bnoRead(int reg) {
  unsigned char val;
  if (256 <= reg) {
    bnoPage1();
  }
  val = bno.read8((Adafruit_BNO055::adafruit_bno055_reg_t)(0xFF & reg));
  if (256 <= reg) {
    bnoPage0();
  }
  return val;
}

// GPS stuff

char gps_str[100]; // gps message buffer
int gps_i;
int show_gpsData =1;
int gps_ms = 0; // milliseconds at first gps message char

// rolling buffer (sized to hold pre launch data)
union dblk pre[NUM_PRE_SAMPLES];
// index of the next entry to be written in pre
int pre_i = 0;
// index of next entry in pre to be written to NVM
int post_i = 0;
// index of next 1k block to be written in NVM
int blk_i = 0;

// get next index in pre rolling buffer
int nextPreBlk(int i) {
  ++i;
  if (NUM_PRE_SAMPLES <= i) {
    return 0;
  }
  return i;
}


// display data
int show_data = 0;

// show a data block record
// using show function for block type specified in first byte
void show_dblk(union dblk* blk) {
  static char c;
  c = *(char*)blk;
  // debug !!!
  //Serial.print(c);
  //Serial.print(*(int*)blk);
  switch (c) {
    case 'F' : show_fastBlk(&(blk->fast)); break;
    case 'S' : show_slowBlk(&(blk->gps)); break;
    default : 
      Serial.print("unknown block type: ");
      Serial.print(c);
      Serial.print(" at address: ");
      Serial.println((int)blk,16);
  }
}


void dump_dataBlk(union dblk* blk) {
  static char c;
  c = *(int*)blk &0xFF;
  switch (c) {
    case 'F' : dump_fastData(&(blk->fast)); break;
    case 'S' : dump_slowData(&(blk->gps)); break;
    default :
      // perhaps should just ignore these? 
      Serial.print("unknown block type: ");
      Serial.print(c);
      Serial.print(" at address: ");
      Serial.println((int)blk,16);
  }
}


// output all data in CSV format over USB serial port
void dumpAll(){
  //static int p;
  static int i, j;
  //static int size  = sizeof(union dblk);

  // disable all data display
  show_data = 0;
  show_gpsData = 0;
  mode = MANUAL_STOP;

  flushCache();
  
  // mark start of data dump
  Serial.println(" ");
  Serial.println("Dump All Data");
  // some interesting data
  Serial.print("0,info,");
  Serial.print(launch_time);
  Serial.print(',');
  Serial.print(launch_trigger/10);
  Serial.print(',');
  // base pressure pascal
  Serial.print(base_press);
  Serial.print(nvmWriteMaxTime);
  Serial.print(',');
  Serial.print(nvmAvg);
  Serial.print(',');
  Serial.println(nvmCount);

  //for each 1k block of NVM
  for (i=0; i<NUM_POST_BLKS; ++i) {
    // for each data block in a 1k block
    for (j = 0; j < NUM_BUF_SAMPLES; ++j) {
      Serial.print(i*NUM_BUF_SAMPLES + j);
      Serial.print(',');
      dump_dataBlk((union dblk*)(NVM + 1024*i + j*sizeof(dblk))); 
    }
  }
}


void dumpPre() {
  static int i;
  static int n;
  Serial.println("Rolling Buffer Data:");
  for (i = pre_i, n = 0; n<NUM_PRE_SAMPLES; i = nextPreBlk(i), ++n) {
    dump_dataBlk(&(pre[i]));
  }
}


/*
// LoRa radio module interface


// LoRa radio pins

int lrCS = 15;
int lrIRQ = 16;
int lrRST = 17;
int lrEnable = 20;

int show_LoRa = 0;

// spi interface
// #define SPI_CONFIG 10000000,MSBFIRST,SPI_MODE0

#define LR_RF_CARRIER_FREQ 906500000

#define LR_BUF_SIZE 256

#define LR_FIFO 0

#define LR_MODE_REG 0x01
#define LR_SLEEP 0x80
#define LR_STDBY 0x81
#define LR_TX 0X83
#define LR_RX_CONT 0x85
#define LR_RX_SINGLE 0x86


#define LR_FIFO_TX_BASE_ADDR_REG 0X0E
#define LR_FIFO_SPI_PTR_REG 0x0D

#define LR_IRQ_FLAGS_REG 0X12
#define LR_RX_TIMEOUT 0X80
#define LR_RX_DONE 0x40
#define LR_PAYLOAD_CRC_ERROR 0x20
#define LR_VALID_HEADER 0x10
#define LR_TX_DONE 0x08
#define LR_CAD_DONE 0x04
#define LR_FHSS_CHANGE_CHANNEL 0x02
#define LR_CAD_DETECTED 0x01

#define LR_PAYLOAD_LENGTH_REG 0x21

// define an initial 8 byte message in buffer
unsigned char lr_buf[LR_BUF_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7};


// Notes:
// need 1 microsecond delay after /cs written high or writes appear to fail
// added delay on all transactions

void lrBegin() {
  SPI.beginTransaction(SPISettings(10000000,MSBFIRST,SPI_MODE0));
}

void lrEnd() {
  SPI.endTransaction();
}

unsigned char lrWriteReg(int reg, int val) {
  static unsigned char c;
  lrBegin();
  digitalWrite(lrCS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x80 | reg);
  c = SPI.transfer(val);
  delayMicroseconds(1);
  digitalWrite(lrCS, HIGH);
  delayMicroseconds(1);
  lrEnd();
  return c;
}

void lrWriteRegs(int reg, unsigned char* vals, int num) {
  lrBegin();
  digitalWrite(lrCS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x80 | reg);
  SPI.transfer(vals, num);
  delayMicroseconds(1);
  digitalWrite(lrCS, HIGH);
  delayMicroseconds(1);
  lrEnd();
}


unsigned char lrReadReg(int reg) {
  static unsigned char c;
  lrBegin();
  digitalWrite(lrCS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x7F & reg);
  c = SPI.transfer(0);
  delayMicroseconds(1);
  digitalWrite(lrCS, HIGH);
  delayMicroseconds(1);
  lrEnd();
  return c;
}


void lrReadRegs(int reg, char* buf, int num) {
  lrBegin();
  digitalWrite(lrCS, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x7F & reg);
  SPI.transfer(buf, num);
  delayMicroseconds(1);
  digitalWrite(lrCS, HIGH);
  delayMicroseconds(1);
  lrEnd();
}


void lrShow (const char* msg) {
  static int i;
  Serial.print("LoRA info: ");
  Serial.print(msg);
  Serial.print(" at ");
  Serial.print(millis()/1000.0,3);
  Serial.print(" mode: 0x");
  Serial.print(lrReadReg(LR_MODE_REG),16);
  Serial.print("  IRQ flags:0x");
  Serial.println(lrReadReg(LR_IRQ_FLAGS_REG),16);
  for (i = 0; i<7; ++i) {
    Serial.print(" ");
    Serial.print(lr_buf[i],16);
  }
  Serial.println("");
}



void setupLoRa() {

  // configure spi
  
  SPI.begin();
  //stopAndBlink(3);


  // configure LoRa module

  pinMode(lrEnable, OUTPUT);
  pinMode(lrCS, OUTPUT);
  pinMode(lrRST, INPUT_PULLUP);
  pinMode(lrIRQ, INPUT_PULLUP);
  // power on
  digitalWrite(lrEnable, HIGH);
  digitalWrite(lrCS, HIGH);
  // wait for radio powerup reset
  delay(100);
  if (LOW == digitalRead(lrRST)) {
    stopAndBlink(2);
  }
  // force reset
  pinMode(lrRST, OUTPUT);
  digitalWrite(lrRST, LOW);
  delayMicroseconds(200);
  digitalWrite(lrRST, HIGH);
  pinMode(lrRST, INPUT_PULLUP);
  delay(10);

  //SPI.beginTransaction(SPISettings(SPI_CONFIG));
  //SPI.beginTransaction(SPISettings(10000000,MSBFIRST,SPI_MODE0));
  // stopAndBlink(4);
  

  // configure LoRa module
  // set to sleep mode
  lrWriteReg(LR_MODE_REG,lrReadReg(0Xf8 & LR_MODE_REG));
  delayMicroseconds(10);
  // LoRa sleep mode
  lrWriteReg(LR_MODE_REG,LR_SLEEP);
  //stopAndBlink(2);
  delayMicroseconds(10);
  lrShow("to lr sleep ");
  // set rest of mode reg
  lrWriteReg(LR_MODE_REG,LR_SLEEP);
  delayMicroseconds(10);
  lrShow("to lr sleep  again");
  // set RF carrier frequency
  int rf_carrier = int(LR_RF_CARRIER_FREQ / (32e6/pow(2,19)));
  lr_buf[0] = (unsigned char)(rf_carrier/65536);
  lr_buf[1] = (unsigned char)((rf_carrier/256) & 0xFF);
  lr_buf[2] =(unsigned char)(rf_carrier & 0xFF);
  // select RFO pin (not PA)
  // select max power for non PA pin (13.8 dBm which is default)
  lr_buf[3] = 0x4F;
  // PaRamp set to default
  lr_buf[6] = 0x09;
  lrShow("before write config starting at 0x6");
  lrWriteRegs(0x06, lr_buf, 7);
  // configure TxDone interrupt on module DIO0, breakout board G0, teensy pin16
  lrWriteReg(0x40,0x40);
  delay(10);
  //stopAndBlink(3);
  //SPI.endTransaction();
}




void loopLoRa(unsigned int ms, unsigned int show) {
  static unsigned char c;

  // send message once a second
  // start after 5 seconds
  static unsigned char lr_mode = 0;
  static unsigned int msg_next_ms = 5000;
  
  if (msg_next_ms <= ms) {
    lr_mode = lrReadReg(LR_MODE_REG);
    if ((LR_SLEEP == lr_mode) || (LR_STDBY == lr_mode)) {
      msg_next_ms +=200;
      lrWriteReg(LR_FIFO_SPI_PTR_REG,0); 
      lrWriteRegs(LR_FIFO,lr_buf,8);
      lrWriteReg(LR_FIFO_TX_BASE_ADDR_REG, 0);
      lrWriteReg(LR_PAYLOAD_LENGTH_REG, 8);
      // clear interupts
      lrWriteReg(LR_IRQ_FLAGS_REG, 0xFF);
      // transmit data
      lrWriteReg(LR_MODE_REG, LR_TX);
      if (show) {
        lrShow("transmit ");
      }
    } else {
      if (show) {
        Serial.print("Not ready to transmit. mode= 0x");
        Serial.println(lr_mode,16);
      }
      // wait 1 second before trying again
      msg_next_ms += 1000;
    }
  }

  // check LoRa IRQ
  if (HIGH == digitalRead(lrIRQ)) {
    if (show) {
      lrShow(" LoRa IRQ ");
    }
    lrWriteReg(LR_IRQ_FLAGS_REG,0xFF);
    //lrShow("   after IRQ reset ");
  }

  // check interrupt flags
  if (0 != lrReadReg(0x12)) {
    c = digitalRead(lrIRQ);
    if (show) {
      Serial.print(c);
      lrShow(" irq active");
    }
    lrWriteReg(0x12,0xFF);
    delayMicroseconds(1);
    if (show) {
      Serial.print(digitalRead(lrIRQ));
      Serial.println(" after IRQ reset");
    }
  }

}

// end LoRa stuff
*/

// return a tag word based on one char and a 24 bit time
// in one 32 bit value
// Used to identify slow (S) and fast (F) data blocks
// and track sample time

int tag(char tag, int ms) {
  // first (least significant byte) is a character
  // next 3 bytes are time in milliseconds since power up
  return ((ms & 0xFFFFFF)<<8) | (tag & 0xFF);
}


// convert least significant 4 bits of an integer
// to a single hex digit character.
char toHexChar(int n){
  n = 0xF & n;
  if(0<=n && n<10){
    return n+'0';
  }else{
    return n-10+'A';
  }
}


// send a command to gps module
// wrap command string in "$" and *<check sum><CR><LF>
void gpsSend(const char str[]){
  static int checkSum;
  checkSum = 0;
  static unsigned int i;
  for(i = 0; i<strlen(str); i++)
  {
    checkSum ^= str[i];
  }

  // for debugging
  //Serial.println(checkSum, 16);

  // gps module serial
  Serial1.write("$");
  Serial1.write(str);
  Serial1.write("*");
  Serial1.write(toHexChar(checkSum/16));
  Serial1.write(toHexChar(checkSum%16));
  Serial1.write(13);
  Serial1.write(10);
}

// parse gps GGA message and fill a temporary buffer with
//   time in seconds since midnight
//   latitude (minutes and fractions)
//   longitude (minutes and fractions)
//   altitde (meters)
// Use given millisecond count for the tag
void parse_gga(char str[], int ms){
  static dblk temp;
  static int tmp;
  static int sec;
  static int min;
  static int hr;
  static float fsec;
  static float lat;
  static float lon;
  static float alt;
  static char* p1;
  static char* p2;

  // skip '$GPGGA,'
  p1 = &str[7];
  // read hhmmss as decimal integer
  sec = strtol(p1,&p2,10);
  if (('.' != *p2) || (p1 == p2)) {
    Serial.println("gps: trouble parsing time in:");
    Serial.println(str);
    return;
  }
  p1 = p2;
  // get hours * 3600
  hr = int(sec/10000);
  fsec = 3600.0*hr;
  // get min * 60 + sec
  sec = sec - 10000*hr;
  min = int(sec/100);
  sec = sec - 100*min;
  p1 = p2;
  fsec = sec + 60*min + 3600*hr + strtod(p1, &p2);
  if ((',' != *p2) || (p1 == p2)) {
    Serial.println("gps: trouble parsing fractional minutes in time in:");
    Serial.println(str);
    Serial.print(" sec: ");
    Serial.print(sec);
    Serial.print(" min: ");
    Serial.print(min);
    Serial.print(" hr: ");
    Serial.print(hr);
    Serial.print(" fsec: ");
    Serial.println(fsec);
    Serial.print(" *p1: ");
    Serial.println(p1);
    Serial.print(" *p2: ");
    Serial.println(p2);
    return;
  }
  // get integer part of minutes of latitude as ddmm.mmmm
  p1 = ++p2;
  tmp = strtol(p1, &p2, 10);
  if (('.' != *p2) || (p1 == p2)) {
    Serial.println("gps: trouble parsing latitude in:");
    Serial.println(str);
    return;
  }
  lat = float(tmp - 100*int(tmp/100));
  p1 = p2;
  // add in fractional minutes
  lat += strtod(p1, &p2);
  if ((',' != *p2) || (p1 == p2)) {
    Serial.println("gps: trouble parsing fractional minutes of latitude in:");
    Serial.println(str);

    return;
  }
  // skip ',N/S,'
  p1 = p2+3;
  // get integer part of minutes of longitude as dddmm.mmmm
  tmp = strtol(p1, &p2, 10);
    if (('.' != *p2) || (p1 == p2)) {
    Serial.println("gps: trouble parsing longitude in:");
    Serial.println(str);
    return;
  }
  lon = float(tmp - 100*int(tmp/100));
  p1 = p2;
  lon += strtod(p1, &p2);
  if ((',' != *p2) || (p1 == p2)) {
    Serial.println("gps: trouble parsing fractional minutes of longitude in:");
    Serial.println(str);
    return;
  }
  // skip ,E/W,1/2/3, look for next ',' to skip satellites used
  p1 = strchr(p2+5, ',');
  if (NULL == p1) {
    Serial.println("gps: trouble skipping satellites used in:");
    Serial.println(str);

    return;
  }
  // skip HDOP
  p1 = strchr(p1+1, ',');
    if (NULL == p1) {
    Serial.println("gps: trouble skipping HDOP in:");
    Serial.println(str);
    return;
  }
  // read altitude as float
  alt = float(strtod(p1+1, &p2));
  if ((',' != *p2) || (p1 == p2)) {
    Serial.println("gps: trouble parsing altitude in:");
    Serial.println(str);
    return;
  }
  
  // write gps data block
  temp.gps.tag = tag('S', ms); 
  temp.gps.lat = lat;
  temp.gps.lon = lon;
  temp.gps.alt = alt;
  temp.gps.gpstim = fsec;
  saveBlk(&temp);
  // debug !!!
  //Serial.print("parse_gga after saveBlk");
  //show_dblk(&temp);
}


void save1k(int blk_num) {
  // write 1k from FlexRam to program page 1
                 
  // teensy 3.6 uses MK66FX1M0VMD18 cpu
  // from datasheet has:
  //   1m prog memory + 256K FlexNVM (data) flash
  //   256k data RAM         
  // see RM (reference manual)
  //   sec 4 (restrictions) must run at 120 MHz or below
  //   sec 5 (mem map)
  //   sec 32 (FTFE flash memory module) erase and write

  FTFL_FSTAT = 0x70; // reset errors
     
  // check that prog or data addr is in a safe area to write
  if (NUM_POST_BLKS <= blk_num) {
    mode = FULL;
    Serial.println("full");
    return;
  }
 
  // fill command buffer
  // see sec 32.3.4 for register addresses
  // ignore 32.4.4 (eeprom not used in this project)
  // see sec 32.4.10 for flash command operation
  // see sec 32.4.12.8 for program section (max 1k bytes) command

  setFlashCmdAddress(0x80000+ blk_num*1024);
  // write section cmd
  FTFL_FCCOB0 = (unsigned char)0xB;
  FTFL_FCCOB4 = 0;
  FTFL_FCCOB5 = (unsigned char)64; //64 dbl phrases = 1024 bytes
  flushCache();
      
  FTFL_FSTAT = (unsigned char)0x80; // execute command
  nvmReady = 0;
  nvmWriteStart = millis(); // track NVM write performance
}


// save 16 byte aligned address into flash memory command buffer
void setFlashCmdAddress (unsigned int p) {
  // write addr b23=prog 0, data 1 (sec 32.4.12)
  FTFL_FCCOB1 = (unsigned char)(0xFF & (p >> 16));
  FTFL_FCCOB2 = (unsigned char)(0xFF & (p >> 8));
  FTFL_FCCOB3 = (unsigned char)(0xF0 & p); // aligned 16 byte address
}


// save a data block in memory
// mode selects type of memory (pre launch, post launch)

void saveBlk(dblk* blkPtr) {
  
  switch (mode) {
    
    case SETUP :
    case PRE_LAUNCH :
    case POST_LAUNCH :
      pre[pre_i]=*blkPtr;
      pre_i = nextPreBlk(pre_i);
      break;
    
    // full, no data is stored
  }

}



int flushCache () {
  // brute force cache flush
  // access at least as much memory (8k bytes) as cache has
  // the resut is never used
  // hopefully the compiler does not figure this out
  
  static volatile int *x;
  static int tmp;
  
  tmp = 0;
  for (x = (int*)0x80000; x<(int*)0x82000; ++x) {
    tmp ^= *x;
  }
  
  return tmp;
}


void clearPostData(){
  static int x;

  // clear flash controller status
  FTFL_FSTAT = 0x70;
  
  Serial.print("Clearing Post Launch Data ... ");
  x = 0x80000;
  while(x<0x100000){
      // erase a block of prog or data NVM, addr in x
      // see sec 32.4.12.7 for erase flash sector (4k byte) command
      if (x < 0x80000
         || (0xFF000 < x && x < 0x800000)
         || (0x83F000 < x)) {
           Serial.print("0X");
           Serial.print(x,16);
           Serial.println(" is not in data loging memory");
           return;
      }
      
      // erase sector cmd
      FTFL_FCCOB0 = (unsigned char)0x9;
      // write addr b23=prog 0, data 1 (sec 32.4.12)
      setFlashCmdAddress(x);

      // execute command
      FTFL_FSTAT = (unsigned char)0x80;

      // wait for command to complete
      // delay(120);
      while(0==(0x80 & FTFL_FSTAT));
      // check for errors
      if (0 != (FTFL_FSTAT & 0x70)) {
        Serial.println("trouble erasing post launch data memory");
        Serial.print(" status: 0x");
        Serial.print(FTFL_FSTAT,16);
        Serial.print("  at address: 0x");
        Serial.println(x,16);
        stopAndBlink(5);
      }

      // move to next sector
      x += 4096;

  }

  // reset post data and post buffer indexes to 0
  post_i = 0;
  //buf_i = 0;
  blk_i = 0;

  flushCache();
  
  Serial.println(" done");
  return;
}


void showMode(int mode) {
  switch(mode){
    case SETUP:
    Serial.print("setup");
    break;

    case PRE_LAUNCH:
    Serial.print("pre_launch");
    break;

    case POST_LAUNCH:
    Serial.print("post_launch");
    break;

    case FULL:
    Serial.print("full");
    break;

    case MANUAL_STOP:
    Serial.print("manual_stop");
    break;
  }
}


#define TEENSY36
//#define TEENSY32

// command interface state
int x = 0; // new values are written to x
int y = 0; // secondary value y - save x to y,  Y - recall y to x
char cli_mode = 'c'; // command line mode, initialy = command char

// blinker state
unsigned int led_next_ms = 0;



// Serial command interface

void cmdLine () {
  static char c = ' ';
  static int n = 0;
  static int i = 0;
  static unsigned int p;
  static unsigned int size;
  // BNO055 calibration status
  static uint8_t sysCal, gyroCal, accCal, magCal;

  
  // if a new character is not available
  if ((Serial.available() == 0)) {
    return;
  }
  c = Serial.read();
  // just echo characters for 0.5 sec
  if (500 > millis()) {
    Serial.print(c);
    return;
  }
    // if entering  decimal or hex number
    // and c is a digit, add it to x and return
    // otherwise fall through to check if it is a command
    switch (cli_mode) {
    case 'd' : // decimal number
      if (isDigit(c)) {
        x = x*10 + c - '0';
        return;
      } else if ('_' == c) {
        return;
      }
      break;
        
    case 'x' : // hexidecimal number
      if (isDigit(c)) {
         x = x*16 + c - '0';
         return;
      } else {
        n = c - 'A';
        if ((0 <= n) && (n < 6)) {
          x = x*16 + 10 + n;
          return;
        } else {
          n = c - 'a';
          if ((0 <= n) && (n < 6)) {
            x = x*16 + 10 + n;
            return;
          } else if ('_' == c) {
            return;
          }
        }
      }
      break;
    }
  
    // new char was not part of a number being already being entered
    cli_mode = 'c';
  
    // process a command character
    switch (c) {
    case '0' : // any digit starts a decimal number
    case '1' :
    case '2' :
    case '3' :
    case '4' :
    case '5' :
    case '6' :
    case '7' :
    case '8' :
    case '9' : x = c - '0';
      cli_mode = 'd';
      break;

    case 'x' : // hex number follows
    x = 0;
      cli_mode = 'x';
      break;

    case ' ' : break; // ignore spaces, (can end hex numbers)

    case 'y' : // copy x to y
      y = x;
      break;

    case 'Y' : // copy y to x
      x = y;
      break;
    
    case '.' : // show hex number in x
      Serial.println(x, 16);
      break;

    case 'h' : // show hex number in memory at x, inc x
      Serial.print(*(volatile unsigned long*)x, 16);
      Serial.print(" at 0x");
      Serial.println(x, 16);
      x += 4;
      break;

    case 'd' : // show decimal number at x, inc x
      Serial.println(*(volatile unsigned long*)x);
      x += 4;
      break;

    case 'i' : // show misc info
      // mostly used to check if the teensy is listening
      Serial.print("mode :");
      showMode(mode);
      Serial.print(" prelaunch index : ");
      Serial.print(pre_i);
      Serial.print(" nvm index : ");
      Serial.println(blk_i);
      Serial.print(" post (saved) index : ");
      Serial.println(post_i);
      Serial.print(millis());
      Serial.print(" milliseconds, next led change at ");
      Serial.println(led_next_ms);
      Serial.print(" launch trigger: ");
      Serial.print(launch_trigger);
      Serial.print(" size: ");
      Serial.println(sizeof(union dblk));
      // show address of NVM
      Serial.print("  nvm[0] at 0x");
      Serial.println(NVM, 16);
      Serial.print("  nvm[2] at 0x");
      Serial.println((unsigned long)((dblk*)(NVM + 2*sizeof(dblk))), 16);
      // show address of FlexRAM
      Serial.print("  flex_ram[0] at 0x");
      Serial.println((int)(&FLEX_RAM[0]), 16);
      Serial.print("  flex_ram[2] at 0x");
      Serial.println((unsigned long)(&FLEX_RAM[2]), 16);
      // show address of pre buffer
      Serial.print("pre first int at: 0X");
      Serial.print((int)pre,16);
      Serial.print(" first entry ");
      Serial.print((int)(&pre[0]),16);
      // show address of cmdLine function
      Serial.print(" cmdLine at 0x");
      Serial.println((unsigned long)cmdLine, 16);

   /* used to debug writing to NVM
      // show info about NVM
      Serial.print("FCNFG ");
      Serial.print(FTFL_FCNFG,16);
      
      Serial.print("   FSEC ");
      Serial.print(FTFL_FSEC,16);
      
      Serial.print("   FOPT ");
      Serial.println(FTFL_FOPT,16);
       
      Serial.print("FPROT 0x");
      Serial.print(FTFL_FPROT0, 16);
      Serial.print(FTFL_FPROT1, 16);
      Serial.print(FTFL_FPROT2, 16);
      Serial.print(FTFL_FPROT3, 16);
        
      Serial.print("   FEPROT 0x");
      Serial.print(FTFL_FEPROT,16);
        
      Serial.print("   FOPT 0x");
      Serial.println(FTFL_FDPROT,16);

#ifdef TEENSY36     
      Serial.print("execute only=0 data+exec=1 0x");
      Serial.print(*(volatile unsigned long*)0x40020018,16);
      Serial.print("_");
      Serial.println(*(volatile unsigned long*)0x4002001c,16);
       
      Serial.print("super only=0 user+super=1 0x");
      Serial.print(*(volatile unsigned long*)0x40020020,16);
      Serial.print("_");
      Serial.println(*(volatile unsigned long*)0x40020024,16);
#endif
      */

      // display BNO055 calibration status (from examples/sensorapi)
      sysCal = gyroCal = accCal = magCal = 0;
      bno.getCalibration(&sysCal, &gyroCal, &accCal, &magCal);
      Serial.print("imu calibration 0..3 System:");
      Serial.print(sysCal, DEC);
      Serial.print(" Gyro:");
      Serial.print(gyroCal, DEC);
      Serial.print(" accel:");
      Serial.print(accCal, DEC);
      Serial.print(" magnetic:");
      Serial.println(magCal, DEC);
      
      Serial.print("nvm write performance ");
      Serial.print(nvmCount);
      Serial.print(" ");
      Serial.print(nvmAvg);
      Serial.println(nvmWriteMaxTime);

      break;

    case 'b' : // check for x blank (FF) bytes starting at y
      // x rounded down to even word count
      for (i = y; i < (y + x - 4); i += 4) {
        if (0xFFFFFFFF != *(volatile unsigned long*)i) {
          break;
        }
      }
      // show first non blank word, or last word
      Serial.print(*(volatile unsigned long*)i, 16);
      Serial.print(" at 0x");
      Serial.println((unsigned long)i, 16);
      break;

    case 'B' : // set base pressure for altitude calculations
      base_press = x;
      Serial.print("base pressure (pascal): ");
      Serial.print(base_press);
      break;

    case 's' : // show x words starting at y
      for (i = y; i < (y + 4 * x); i += 4) {
        Serial.print(*(volatile unsigned long*)i, 16);
        Serial.print(" at 0x");
        Serial.println((unsigned long)i, 16);
      }
      break;

    case 'u' : // show one byte at x, inc x
      Serial.print(*(volatile unsigned char*)x, 16);
      Serial.print(" at 0x");
      Serial.println((unsigned long)x, 16);
      x += 1;
      break;

    case 'W' : // write one word from x to memory at y, inc y by 4
      *(volatile long*)y = x;
      y += 4;
      break;

    case 'V' : // write half word (16 bits) from x
      // to memory at y (little endian), then inc y by 2
      *(volatile short*)y = (short)x;
      y += 2;
      break;

    case 'U' : // write a byte from x to memory at y, inc y
      *(volatile char*)y = (char) x;
      y += 1;
      break;

     case 'p':
      for(i=y; i<y+x; ++i)
        {
          show_dblk(&pre[i]);
        }
        break;

      case 'q':
          for(i=y; i<y+x; ++i)
        {
           size = sizeof(union dblk);
           p = 0x80000+ i*size+i/int(124/size)*(1024%size);                  
           show_dblk((union dblk*)p);
        }
        break;


      case 'S':
        mode = MANUAL_STOP;
        Serial.println("Manual Stop");
        break;

#ifdef TEENSY36
      case 'P' : // write 1k from FlexRam to prog or data addr in x
      // teensy 3.6 uses MK66FX1M0VMD18 cpu
      // from datasheet has:
      //   1m prog memory + 256K FlexNVM (data) flash
      //   256k data RAM         
      // see RM (reference manual)
      //   sec 4 (restrictions) must run at 120 MHz or below
      //   sec 5 (mem map)
      //   sec 32 (FTFE flash memory module) erase and write
      // check that prog or data addr is in a safe area to write
        if (x < 0x80000
            || (0xFFC00 < x && x < 0x800000)
            || (0x83FC00 < x)) {
              Serial.print("0X");
              Serial.print(x,16);
              Serial.println(" is not in data loging memory");
              break;
         }
      // see sec 32.3.4 for register addresses
      // ignore 32.4.4 (eeprom not used in this project)
      // see sec 32.4.10 for flash command operation
      // see sec 32.4.12.8 for program section (max 1k bytes) command  
        FTFL_FSTAT = 0x70; // reset errors
        setFlashCmdAddress(x); // write addr b23=prog 0, data 1 (sec 32.4.12)
        FTFL_FCCOB0 = (unsigned char)0xB; // write section cmd
        FTFL_FCCOB4 = 0;
        FTFL_FCCOB5 = 64; //64 dbl phrases = 1024 bytes
      // comment out for now
      // use x40020000yx80U on command interface to execute the write for now
      // *(unsigned char*)0x40020000 = (unsigned char)0x80; // execute
        break;

      case 'E' : // erase a block of prog or data NVM, addr in x
        Serial.print("E ");
        // see sec 32.4.12.7 for erase flash sector (4k byte) command
        if (x < 0x80000
           || (0xFF000 < x && x < 0x800000)
           || (0x83F000 < x)) {
             Serial.print("0X");
             Serial.print(x,16);
             Serial.println(" is not in data loging memory");
             break;
        }
        FTFL_FSTAT = 0x70; // reset errors
        setFlashCmdAddress(x); // write addr b23=prog 0, data 1
        FTFL_FCCOB0 = 9; // erase sector
        Serial.print("00: ");
        Serial.print(*(volatile unsigned int *)0x40020000);
        Serial.print(" 04: ");
        Serial.print(*(volatile unsigned int *)0x40020004);
        Serial.print(" 08: ");
        Serial.println(*(volatile unsigned int *)0x40020008);
        // comment out for now
        // use x40020000yx80U on command interface to execute the erase for now
        // *(unsigned char*)0x40020000 = (unsigned char)0x80; // execute
        break;

        case 'L' : // prepare for launch
        mode = STARTUP;
        launch_time = 0;

        
        // falls through to clear post launch data
        
        case 'C': //clears post data from command line
          clearPostData(); break;
#endif

        case 'D' : // Toggle the data printing inside the loop

          show_data = !show_data;
          break;


        case 'G': // toggle display of GPS data
          show_gpsData = !show_gpsData;
          break;

        /* remove radio stuff
        case 'R' : // toggle display of radio info
          show_LoRa = !show_LoRa;
          break;
        */

        case 'R' : // read register from BNO055 page 0

        case 'r' : // read BNO055 register from page 1
          Serial.print("reg 0X");
          Serial.print(x,16);
          Serial.print(" is 0X");
          Serial.println(bnoRead(x),16);
          break;
          
        case 'M' : // write all data as a csv file.
          dumpAll();
          break;

        case 'm' : // write rolling buffer as a .csv file
          dumpPre();
          break;

        case 'O' : // set launch offset to x/10
          launch_offset = x/10.0;
          Serial.print("launch offset: ");
          Serial.println(launch_offset);
          break;  

        case 'l' : // dump x data  blocks at y, increment y
          for (i = 0; i < x; ++i) {
            dump_dataBlk((dblk*)y);
            y += sizeof(dblk);
          }
          break;


        default : Serial.print(c); Serial.println("?"); break;
    }
    return;

}


int digitalToggle(int pin) {
  static int value;
  value = digitalRead(pin);
  if (HIGH == value) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
  return(value);
}


// used to indicate major faults
// 1 - could not find BMP280
// 2 - could not find BNO055 IMU
// 3 - error on last program write, program memory was not erased?
// 5 - trouble during erase
// 6 - EEPROM function in use
// 7 - CPU not in normal run mode (set clock speed 120 Mhz or less)
void stopAndBlink(int num) {
  static int i;
  while (1) {
    for (i=0; i<num; ++i) {
      digitalWrite(led, HIGH);
      delay(250);
      digitalWrite(led, LOW);
      delay(250);
    }
    delay(1000);
  }
}


void setup() {
   Serial.begin(9600);
  // check if CPU not in normal run mode
  if (0x60 & SMC_PMCTRL) {
    Serial.println("CPU not in normal run mode");
    Serial.println("Check that speed is 120 MHz or less and recompile");
    stopAndBlink(7);
  }
  Serial.println("CPU OK");
  // check that FlexRam is not used for EEPROM
  if (0x2 != FTFL_FCNFG) {
    Serial.println("OOPS FlexRam is being used for EEPROM");
    stopAndBlink(6);
  }
  Serial.println("FLEX RAM OK");
  // clear FTFE flash memory error flags
  FTFL_FSTAT = 0x70;
  
  Serial.begin(9600);
  delay(100);
  Serial.println("hi .  ");
  pinMode(led, OUTPUT);
  // put your setup code here, to run once:
  if (!bmp.begin()) {
    Serial.println("could not find bmp280");
    digitalWrite(led,HIGH);
    stopAndBlink(1);
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X1, /* temp oversampling */
      Adafruit_BMP280::SAMPLING_X1,  /* press oversampling */
      Adafruit_BMP280::FILTER_OFF, /* filterig */
      Adafruit_BMP280::STANDBY_MS_1); /* standby time */
  }
   if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_AMG))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    stopAndBlink(2);
  }
  
      delay(1000);

    
  bno.setExtCrystalUse(true);

  Serial1.begin(9600);
  Serial.print("configure gps");
  

  /*
  Serial.print("TEST");
  Serial1.println("$PMTK010,001*EE");
 */
  // set gps update rate to 5 hz
  Serial1.println("$PMTK220,200*2C");
  delay(100);
  // select only gga messages
  gpsSend("PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
  
  digitalWrite(led, HIGH);  
  show_data = 0;
  show_gpsData = 0;
  //Serial.print("TEST 2");
  //while(1);

  //setupLoRa();
}



void loop() {
  // put your main code here, to run repeatedly:
  static char gps_char;
  static float alt, p;
  static union dblk blk;
  static int launch_cnt = 0;
  static unsigned int next_fast_sample = 0;
  static unsigned int ms;
  static unsigned dt;
  volatile static unsigned int* mp;
  static unsigned int led_period = 1000;
  static int i;
  static int nblks;
  // buffer for 3 element vectors from IMU
  imu::Vector<3> v3;
  
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1000);               // wait for a second
  //digitalWrite(led2, LOW);    // turn the LED off by making the voltage LOW
  //delay(100);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW

  ms = millis();
  
  // collect GPS data
  if(Serial1.available()){
    //Serial.print((char)Serial1.read());
    gps_char=Serial1.read();
    if(mode != MANUAL_STOP) {
    //Serial.println(gps_str);
    switch (gps_char) {

      case '$' : // start of gps message
        gps_str[0] = '$';
        gps_i = 1;
        gps_ms = ms;
        break;

      case '*' : // end of gps message, start of checksum
      case 10 : // end of gps message string
        // if gps string looks sane
        if (60 < gps_i && gps_i < 98) { 
          gps_str[gps_i]=0;
          if (show_gpsData) {
            Serial.print(gps_str);
            Serial.println(gps_i);
          }
          parse_gga(gps_str, gps_ms);
        }
        // get next gps string
        gps_i = 0;
        break;
        
      default : // save gps char if it fits in gps buffer
        if (gps_i < 98) {
          gps_str[gps_i] = gps_char;
          gps_i++;
        }
      }
    }
  }

  
  // sample fast data
  if (next_fast_sample < ms) {
    next_fast_sample += FAST_SAMPLE_PERIOD;
    p = bmp.readPressure();
    alt = bmp.readAltitude(base_press/100.0); 
    //Serial.println("Have Barometric ");
  
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);

    //Serial.println("Have IMU ");

    blk.fast.tag = tag('F', ms);
    blk.fast.alt = (short)(0xFFFF & int(alt*10));
    blk.fast.p = (short)(0xFFFF & int(base_press-p));

    // (degrees/second) *1 0
    v3 = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    blk.fast.ox = (short)int(v3[0]*10);
    blk.fast.oy = (short)int(v3[1]*10);
    blk.fast.oz = (short)int(v3[2]*10);

    // m/sec^2 * 100
    v3 = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    blk.fast.ax = (short)int(v3[0]*100);
    blk.fast.ay = (short)int(v3[1]*100);
    blk.fast.az = (short)int(v3[2]*100);
    
    switch (mode) {

      case STARTUP:
        // start collecting setup data (again)
        // note this does not erase post launch data
        led_period = 1000;
        launch_cnt = 0;
        setup_sum = 0;
        num_setup = 0;
        // initalize nvm write performance monitoring
        nvmWriteMaxTime = 0;
        nvmWriteStart = 0;
        nvmCount = 0;
        nvmAvg = 0;
        
        Serial.println("Setup");
        // check if first batch of post launch is not clear
        for (mp = (unsigned int*)0x80000; mp < (unsigned int *)0x80400; ++mp) {
          if (0 != ~(*mp)) {
            Serial.println("post data memory is not empty !! use C or L ?");
            show_data = 0;
            show_gpsData = 0;
            break;
          }
        }
        mode = SETUP;
        
        // fall through to setup
    
      case SETUP :
        // launch trigger is set above average altitude
        // at power on
        setup_sum += alt;
        ++num_setup;
        if (num_setup > SETUP_SAMPLES) {
          launch_trigger = (setup_sum/num_setup) + launch_offset;
          mode = PRE_LAUNCH;
          Serial.println("To Prelaunch");
        }
        // fall through to prelaunch
    
      case PRE_LAUNCH :
      // detect launch
        if (launch_trigger <= alt) {
          ++launch_cnt;
          if (launch_cnt > LAUNCH_CONFIRM_COUNT) {
            // launch detected
            mode = POST_LAUNCH;
            launch_time = ms;
            led_period = 500;
            // start saving To NVM at what will be the oldest data block in pre buffer
            // after current block is saved (just below in POST_LAUNCH)
            post_i = nextPreBlk(pre_i);
            Serial.println("Launched");            
            // start saving at first block in NVM
            blk_i = 0;
          }
        } else {
          // reset launch count for any measurement below launch level
          launch_cnt = 0;
        }
        // fall through to save data in rolling buffer
    
      case POST_LAUNCH :
        saveBlk(&blk);
        break;

      // FULL : no data is stored
    }
    
    /* Display the fast data */
    if (show_data) {

      Serial.print(" INDEX PRE: ");
      Serial.print(pre_i);
      Serial.print(" NVM Index: ");
      Serial.print(blk_i);
      Serial.print(" INDEX POST: ");
      Serial.println(post_i);
      Serial.print(" MODE: ");
      showMode(mode);
      show_dblk(&blk);
  
      Serial.print(" Launch trigger: ");
      Serial.println(launch_trigger);
      Serial.print(" SETUP_SUM: ");  
      Serial.print(setup_sum,1);
      Serial.print(" NUM_SETUP: ");
      Serial.println(num_setup);
    }
  }

  // copy data to NVM after launch
  if (POST_LAUNCH == mode) {
    // if NVM ready
    if (0x80 & FTFL_FSTAT) {
      // if there are enough blocks to fill a 1k buffer
      nblks = pre_i - post_i;
      // Note if pre_i == post_i all entries in rolling buffer(pre) need to be saved in NVM
      if (nblks <= 0) {
        nblks += NUM_PRE_SAMPLES;
      }
      if (NUM_BUF_SAMPLES < nblks) {
        // copy blocks to FlexRAM buffer
        Serial.print("pre_i ");
        Serial.print(pre_i);
        Serial.print("Copy from pre at ");
        Serial.print(post_i);
        Serial.print(" to NVM at ");
        Serial.println(blk_i);
        for (i=0; i<NUM_BUF_SAMPLES; ++i) {
          FLEX_RAM[i] = pre[post_i];
          post_i = nextPreBlk(post_i);
        }
        // save buffer to page 1 of NVM
        save1k(blk_i);
        ++blk_i;
      }
    }
  }
  
  // send messages
  //loopLoRa(ms, show_LoRa);

  //Monitor nvmWrite Performance
  if (nvmWriteStart >0){
    if(0x80 & FTFL_FSTAT){
      dt=millis()-nvmWriteStart;
      nvmWriteMaxTime = max(nvmWriteMaxTime,dt);
      nvmAvg += dt;
      nvmCount++;
      nvmWriteStart = 0;
    }
  }




      

  cmdLine();
  
  // blinker, only good for 5 days !!
  if (led_next_ms <= millis()) {
    digitalToggle(led);
    led_next_ms += led_period;
  }

}
