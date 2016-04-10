// Pins
#define SCLK 2
#define SDIO 3

// Registers
#define REG_CONFIG_BITS 0x00
#define REG_STATUS_BITS 0x01
#define REG_DELTA_Y 0x02
#define REG_DELTA_X 0x03
#define REG_SQUAL 0x04
#define REG_MAXIMUM_PIXEL 0x05
#define REG_MINIMUM_PIXEL 0x06
#define REG_SUM_PIXEL 0x07
#define REG_PIXEL_DATA 0x08
#define REG_SHUTTER_UPPER 0x09
#define REG_SHUTTER_LOWER 0x0A
#define REG_PRODUCT_ID 0x11

// Image timing debug switch
// #define DEBUGIMAGETIME
#ifdef DEBUGIMAGETIME
  #define DOMOTION false
#else
  #define DOMOTION true
#endif

// Array size and storage
#define PIXWIDTH 18
#define DUMPWIDTH (PIXWIDTH*PIXWIDTH) // Number of pixels to read for each frame.
#define DUMPBYTES ((DUMPWIDTH*6)/8) // Total (packed storage) in bytes

// Motion buffer size and rates
#define MOTIONUPDATERATE_PIX 6
#define MOTIONUPDATERATE_MOT 4
#define MOTIONBUFFERLENGTH ((PIXWIDTH*PIXWIDTH*10)/(15*MOTIONUPDATERATE_PIX)+8)

// Frame and motion buffers
byte frame[DUMPBYTES];
byte motionStore[MOTIONBUFFERLENGTH][2];
unsigned long timeStore[MOTIONBUFFERLENGTH];

// Global flags
unsigned long lastMTime = 0;
unsigned long motionRate = MOTIONUPDATERATE_MOT;
boolean firstSetup = true;
boolean sendPixels = false;
boolean sendSQual = false;

void setup()
{
  if(firstSetup) {
    Serial.begin(115200);
    start();
    delay(100);
    writeRegister(REG_CONFIG_BITS, 0x80);
    delay(5);
    firstSetup = false;
    Serial.println("Reset.");
  }

  byte productId = readRegister(REG_PRODUCT_ID) & 0x0F;
  Serial.print("Found productId : ");
  Serial.println(productId, HEX);

  byte status = readRegister(REG_STATUS_BITS) & 0xE1;
  Serial.print("Status register read : ");
  Serial.println(status, HEX);

  Serial.print("STARTED");

  delay(1000);
}

void start()
{
  pinMode(SCLK, OUTPUT);
  pinMode(SDIO, OUTPUT);
  digitalWrite(SCLK, HIGH);
  digitalWrite(SDIO, HIGH);
  delay(5);
}

void loop()
{
  if(Serial.available()) {
    parseSerial();
    return;
  }

  byte *mStore = (byte*)motionStore;
  unsigned long *tStore = timeStore;
  if(storeMotion(&mStore, &tStore) && DOMOTION) {
    writeMotionBuffer((byte*)motionStore, (unsigned long*)timeStore, 1);
    if(sendSQual) {
      char sQual = readRegister(REG_SQUAL);
      Serial.print('Q', BYTE);
      Serial.print(sQual, BYTE);
    }
  } else if(sendPixels) {
    dumpFrame();
  }
}

void parseSerial()
{
  int val = Serial.read() - 'a';
  if(val == ('x'-'a')) { // Die on 'x'
    do{
      delay(1000);
    } while(true);
    return;
  }
  if(val == ('u'-'a')) { // Toggle sleep bit on 'u'
    byte config = readRegister(REG_CONFIG_BITS);
    config ^= 0x01; // Sleep mode toggle
    writeRegister(REG_CONFIG_BITS, config);
    return;
  }
  if(val == ('p'-'a')) { // Toggle pixel capture on 'p'
    sendPixels = !sendPixels;
    if(sendPixels) {
      motionRate = MOTIONUPDATERATE_PIX;
    } else {
      motionRate = MOTIONUPDATERATE_MOT;
    }
    return;
  }
  if(val == ('s'-'a')) { // SQual capture on 's'
    sendSQual = !sendSQual;
    return;
  }
  if(val == ('w'-'a')) { // Write 'w' to register 'a-o' with next byte. Follow with read from same address
    while(!Serial.available()) {
    }
    int reg = Serial.read()-'a';
    while(!Serial.available()) {
    }
    int data = Serial.read();
    writeRegister(reg, data);
    val = reg;
  }
  if(val >= 0 && val <= REG_PRODUCT_ID) { // Read register 'a-o' and print result
    Serial.print('R', BYTE);
    Serial.print(val, BYTE);
    Serial.print(readRegister(val), BYTE);
  }
}

boolean storeMotion(byte **motion, unsigned long **time)
{
  char dx = 0;
  char dy = 0;
  unsigned long mTime = millis();

  if((mTime - lastMTime) >= motionRate) {
    lastMTime = mTime;
    dx = readRegister(REG_DELTA_X);
    dy = readRegister(REG_DELTA_Y);

    if(dx || dy) {
      *(*motion)++ = dx;
      *(*motion)++ = dy;
      *(*time)++ = mTime;
      return true;
    }
  }
  return false;
}

void writeMotionBuffer(byte *motion, unsigned long *time, unsigned int motionLength)
{
  for(int i = 0; i < motionLength; i++) {
    writeTime(*time++);
    Serial.print('M', BYTE);
    Serial.print(*motion++, BYTE);
    Serial.print(*motion++, BYTE);
  }
}

void writeTime(unsigned long mTime)
{
  Serial.print('T', BYTE);
  Serial.print((byte)((mTime >> 0) & 0x000000FF), BYTE);
  Serial.print((byte)((mTime >> 8) & 0x000000FF), BYTE);
  Serial.print((byte)((mTime >> 16) & 0x000000FF), BYTE);
  Serial.print((byte)((mTime >> 24) & 0x000000FF), BYTE);
}

void dumpFrame()
{
  int idx = 0;
  int pixNum = 0;
  int nMotion = 0;
  boolean gotPixel = false;

  byte *motionPos = (byte*)motionStore;
  unsigned long *timePos = timeStore;

  unsigned long imgTimeBeg;
  unsigned int imgTimeEnd;

  writeRegister(REG_PIXEL_DATA, 0x00);
  do {
    byte data = readRegister(REG_PIXEL_DATA);
    if(data & 0x80) { // Start of frame found
      idx = 0;
      pixNum = 0;
      nMotion = 0;
      motionPos = (byte*)motionStore;
      timePos = timeStore;

      imgTimeBeg = millis();
    }
    if(data & 0x40) { // Data is valid
      switch(pixNum%(8/2)) {
        case 0:
          frame[idx] = data & 0x3F; // all 6 bits
          break;
        case 1:
          frame[idx] |= (data << 2) & 0xC0; // 2 high bits
          idx++;
          frame[idx] = data & 0x0F; // 4 low bits
          break;
        case 2:
          frame[idx] |= (data << 2) & 0xF0; // 4 high bits
          idx++;
          frame[idx] = data & 0x03; // 2 low bits
          break;
        case 3:
          frame[idx] |= (data << 2) & 0xFC; // all six bits 
          idx++;
          break;
      }
      pixNum++;
      gotPixel = true;
    } else { // Pixel  not valid
      if(gotPixel) {
        gotPixel = false;
        if(storeMotion(&motionPos, &timePos)) {
          nMotion++;
        }
      }
    }
  } while (pixNum != DUMPWIDTH);
  imgTimeEnd = millis();

  #ifdef DEBUGIMAGETIME
    Serial.print("Time :");
    Serial.print(imgTimeEnd-imgTimeBeg);
    Serial.print(" Motion :");
    Serial.println(nMotion);
  #else
    writeMotionBuffer((byte*)motionStore, (unsigned long*)timeStore, nMotion);
    writeTime(imgTimeBeg);
    Serial.print('F', BYTE);
    Serial.print((((unsigned int)DUMPBYTES) & 0x00FF), BYTE);
    Serial.print((((unsigned int)DUMPBYTES) & 0xFF00), BYTE);
    for(int i = 0; i < DUMPBYTES; i++) {
      Serial.print(frame[i], BYTE);
    }
    writeTime(imgTimeEnd);
  #endif
}

byte readRegister(byte address)
{
  byte res = 0;
  pinMode (SDIO, OUTPUT);
  for(byte i = 0; i < 7; i++) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, address & 0x80);
    digitalWrite (SCLK, HIGH);
    address <<= 1;
  }
  digitalWrite (SCLK, LOW);
  digitalWrite (SDIO, address & 0x80);
  digitalWrite (SCLK, HIGH);
  delayMicroseconds(1);
  pinMode (SDIO, INPUT);
  delayMicroseconds(99);
  for (byte i = 0; i < 7; i++) {
    digitalWrite(SCLK, LOW);
    digitalWrite(SCLK, HIGH);
    res |= digitalRead(SDIO);
    res <<= 1;
  }
  digitalWrite(SCLK, LOW);
  digitalWrite(SCLK, HIGH);
  res |= digitalRead(SDIO);
  return res;
}

void writeRegister(byte address, byte data)
{
  address |= 0x80; // MSB indicates write mode.
  pinMode (SDIO, OUTPUT);
  for(byte i = 0; i < 7; i++) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, address & 0x80);
    digitalWrite (SCLK, HIGH);
    address <<= 1;
  }
  digitalWrite (SCLK, LOW);
  digitalWrite (SDIO, address & 0x80);
  digitalWrite (SCLK, HIGH);
  for(byte i = 0; i < 7; i++) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, data & 0x80);
    digitalWrite (SCLK, HIGH);
    data <<= 1;
  }
  digitalWrite (SCLK, LOW);
  digitalWrite (SDIO, data & 0x80);
  digitalWrite (SCLK, HIGH);

  delayMicroseconds(100);
}
