// RACE - Race Telemetry v6

// Arduino Nano | ST7789 240x320 portrait | SSD1306 128x64 (own tiny driver)

// MPU6050 (full gyro + accel calibration) | DS18B20 (raw OneWire) | Buzzer on Timer2

// MPU mounting: X = forward, Y = left, Z = up

// POWER ON with the bike UPRIGHT and STILL (this sets 0 deg lean / 0 deg pitch)



#include <SPI.h>

#include <Wire.h>

#include <Adafruit_GFX.h>

#include <Adafruit_ST7789.h>

#include <OneWire.h>

// Arduino Nano | ST7789 240x320 portrait | SSD1306 128x64 (own tiny driver)

// MPU6050 (full gyro + accel calibration) | DS18B20 (raw OneWire) | Buzzer on Timer2

// MPU mounting: X = forward, Y = left, Z = up

// POWER ON with the bike UPRIGHT and STILL (this sets 0 deg lean / 0 deg pitch)



#include <SPI.h>

#include <Wire.h>

#include <Adafruit_GFX.h>

#include <Adafruit_ST7789.h>

#include <OneWire.h>



\#define TFT_CS       10

\#define TFT_DC       9

\#define TFT_RST      8

\#define ONE_WIRE_PIN 2

\#define BUZZER_PIN   3

\#define MPU_ADDR     0x68

\#define OLED_ADDR    0x3C



\#define LEAN_SIGN   1.0    // +lean = RIGHT (flip if reversed)

\#define PITCH_SIGN -1.0    // +pitch = nose UP (flip if reversed)



\#define COL_BG     0x0000

\#define COL_WHITE  0xFFFF

\#define COL_CYAN   0x07FF

\#define COL_YELLOW 0xFFE0

\#define COL_GREEN  0x07E0

\#define COL_RED    0xF800

\#define COL_GRAY   0x7BEF

\#define COL_DIM    0x39E7



const float TEMP_WARN = 50.0;

const float TEMP_CRIT = 56.0;



const uint16_t MPU_INTERVAL  = 20;

const uint16_t TFT_INTERVAL  = 120;

const uint16_t OLED_INTERVAL = 130;   // one OLED row per tick

const uint16_t TEMP_PERIOD   = 1000;

const uint16_t TEMP_CONV_MS  = 220;



Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

OneWire ds(ONE_WIRE_PIN);



float gAx, gAy, gAz, gGx, gGy, gGz, gTot;

float gRoll = 0, gPitch = 0, gLong = 0, gLat = 0;

float maxL = 0, maxR = 0;

float gxOff = 0, gyOff = 0, gzOff = 0;   // gyro offsets (deg/s)

float axOff = 0, ayOff = 0, azOff = 0;   // accel offsets (g)

bool  imuOk = false, filterInit = false;

uint8_t imuFail = 0;

uint32_t lastUs = 0, lastImuRetry = 0;



float tempC = 0;

bool  tempOk = false, tempReq = false;

uint8_t tempFail = 0;



bool beeping = false;

uint32_t beepEnd = 0;



uint32_t tMpu = 0, tTft = 0, tOled = 0, tTemp = 0, tBuz = 0;



enum { F_LEAN, F_LSIDE, F_RSIDE, F_MAXL, F_MAXR, F_PITCH, F_YAW,

       F_LONG, F_LAT, F_TOT, F_GX, F_GY, F_GZ, F_TEMP, F_STAT, F_COUNT };

char prevTxt[F_COUNT][11];

uint16_t prevCol[F_COUNT];

int16_t prevPx = 999;

uint16_t prevBarCol = 0;



// 5x7 font, ASCII '-' (0x2D) .. 'Z' (0x5A), uppercase only

const uint8_t font5x7[] PROGMEM = {

  0x08,0x08,0x08,0x08,0x08, // -

  0x00,0x60,0x60,0x00,0x00, // .

  0x20,0x10,0x08,0x04,0x02, // /

  0x3E,0x51,0x49,0x45,0x3E, // 0

  0x00,0x42,0x7F,0x40,0x00, // 1

  0x42,0x61,0x51,0x49,0x46, // 2

  0x21,0x41,0x45,0x4B,0x31, // 3

  0x18,0x14,0x12,0x7F,0x10, // 4

  0x27,0x45,0x45,0x45,0x39, // 5

  0x3C,0x4A,0x49,0x49,0x30, // 6

  0x01,0x71,0x09,0x05,0x03, // 7

  0x36,0x49,0x49,0x49,0x36, // 8

  0x06,0x49,0x49,0x29,0x1E, // 9

  0x00,0x36,0x36,0x00,0x00, // :

  0x00,0x56,0x36,0x00,0x00, // ;

  0x08,0x14,0x22,0x41,0x00, // <

  0x14,0x14,0x14,0x14,0x14, // =

  0x00,0x41,0x22,0x14,0x08, // >

  0x02,0x01,0x51,0x09,0x06, // ?

  0x32,0x49,0x79,0x41,0x3E, // @

  0x7E,0x11,0x11,0x11,0x7E, // A

  0x7F,0x49,0x49,0x49,0x36, // B

  0x3E,0x41,0x41,0x41,0x22, // C

  0x7F,0x41,0x41,0x22,0x1C, // D

  0x7F,0x49,0x49,0x49,0x41, // E

  0x7F,0x09,0x09,0x09,0x01, // F

  0x3E,0x41,0x49,0x49,0x7A, // G

  0x7F,0x08,0x08,0x08,0x7F, // H

  0x00,0x41,0x7F,0x41,0x00, // I

  0x20,0x40,0x41,0x3F,0x01, // J

  0x7F,0x08,0x14,0x22,0x41, // K

  0x7F,0x40,0x40,0x40,0x40, // L

  0x7F,0x02,0x0C,0x02,0x7F, // M

  0x7F,0x04,0x08,0x10,0x7F, // N

  0x3E,0x41,0x41,0x41,0x3E, // O

  0x7F,0x09,0x09,0x09,0x06, // P

  0x3E,0x41,0x51,0x21,0x5E, // Q

  0x7F,0x09,0x19,0x29,0x46, // R

  0x46,0x49,0x49,0x49,0x31, // S

  0x01,0x01,0x7F,0x01,0x01, // T

  0x3F,0x40,0x40,0x40,0x3F, // U

  0x1F,0x20,0x40,0x20,0x1F, // V

  0x3F,0x40,0x38,0x40,0x3F, // W

  0x63,0x14,0x08,0x14,0x63, // X

  0x07,0x08,0x70,0x08,0x07, // Y

  0x61,0x51,0x49,0x45,0x43  // Z

};



const uint8_t oledInitSeq[] PROGMEM = {

  0xAE, 0xD5,0x80, 0xA8,0x3F, 0xD3,0x00, 0x40, 0x8D,0x14, 0x20,0x02,

  0xA1, 0xC8, 0xDA,0x12, 0x81,0xCF, 0xD9,0xF1, 0xDB,0x40, 0xA4, 0xA6, 0xAF

};



// ---------------- math helpers ----------------

float wrap180(float a) {

  while (a > 180.0) a -= 360.0;

  while (a < -180.0) a += 360.0;

  return a;

}



float atan2d(float y, float x) {          // degrees, error < 0.3

  float ax = fabs(x), ay = fabs(y);

  float mx = (ax > ay) ? ax : ay;

  float mn = (ax > ay) ? ay : ax;

  if (mx < 1e-6) return 0;

  float a = mn / mx;

  float r = a / (1.0 + 0.28125 \* a \* a) \* 57.29578;

  if (ay > ax) r = 90.0 - r;

  if (x < 0) r = 180.0 - r;

  if (y < 0) r = -r;

  return r;

}



float sinD(float d) {                     // valid -180..180

  float s = 1.0;

  if (d < 0) { d = -d; s = -1.0; }

  float p = d \* (180.0 - d);

  return s \* 4.0 \* p / (40500.0 - p);

}



float cosD(float d) { return sinD(90.0 - fabs(d)); }



// float -> text, prec 0..2

void fmt(char \*b, float v, uint8_t prec) {

  uint16_t sc = (prec == 0) ? 1 : (prec == 1 ? 10 : 100);

  float lim = (prec == 2) ? 99.0 : 999.0;

  bool neg = v < 0;

  if (neg) v = -v;

  if (v > lim) v = lim;

  uint16_t n = (uint16_t)(v \* sc + 0.5);

  if (n == 0) neg = false;

  if (neg) \*b++ = '-';

  uint16_t ip = n / sc;

  uint16_t fp = n % sc;

  if (ip >= 100) \*b++ = '0' + ip / 100;

  if (ip >= 10)  \*b++ = '0' + (ip / 10) % 10;

  \*b++ = '0' + ip % 10;

  if (prec) {

    \*b++ = '.';

    if (prec == 2) \*b++ = '0' + fp / 10;

    \*b++ = '0' + fp % 10;

  }

  \*b = 0;

}



// ---------------- buzzer (Timer2 on D3 / OC2B) ----------------

// f = 250000 / (2 \* (top + 1)) Hz

void beepOn(uint8_t top, uint16_t ms, uint32_t now) {

  OCR2A = top;

  OCR2B = top;

  TCNT2 = 0;

  TCCR2A = \_BV(COM2B0) | \_BV(WGM21);

  TCCR2B = \_BV(CS22);

  beepEnd = now + ms;

  beeping = true;

}



void beepOff() {

  TCCR2B = 0;

  TCCR2A = 0;

  digitalWrite(BUZZER_PIN, LOW);

  beeping = false;

}



void updateBuzzer(uint32_t now) {

  if (beeping && (int32_t)(now - beepEnd) >= 0) beepOff();

  if (!tempOk) return;

  if (tempC > TEMP_CRIT) {

    if (now - tBuz >= 300) { tBuz = now; beepOn(34, 150, now); }    // \~3.6 kHz

  } else if (tempC >= TEMP_WARN) {

    if (now - tBuz >= 2000) { tBuz = now; beepOn(55, 100, now); }   // \~2.2 kHz

  }

}



// ---------------- OLED 128x64 (tiny SSD1306 text driver) ----------------

void oledCmd(uint8_t c) {

  Wire.beginTransmission(OLED_ADDR);

  Wire.write((uint8_t)0x00);

  Wire.write(c);

  Wire.endTransmission();

}



void oledPos(uint8_t page, uint8_t col) {

  oledCmd(0xB0 | page);

  oledCmd(col & 0x0F);

  oledCmd(0x10 | (col >> 4));

}



void oledInit() {

  for (uint8_t i = 0; i < sizeof(oledInitSeq); i++)

    oledCmd(pgm_read_byte(oledInitSeq + i));

}



void oledClear() {

  for (uint8_t p = 0; p < 8; p++) {

    oledPos(p, 0);

    for (uint8_t k = 0; k < 8; k++) {

      Wire.beginTransmission(OLED_ADDR);

      Wire.write((uint8_t)0x40);

      for (uint8_t j = 0; j < 16; j++) Wire.write((uint8_t)0);

      Wire.endTransmission();

    }

  }

}



// Centered text, 21 chars per line (6 px each), uppercase only

void oledLine(uint8_t page, const char \*s) {

  uint8_t n = strlen(s);

  if (n > 21) n = 21;

  uint8_t lead = (21 - n) >> 1;

  oledPos(page, 1);

  for (uint8_t i = 0; i < 21; i++) {

    char c = (i >= lead && i < lead + n) ? s[i - lead] : ' ';

    Wire.beginTransmission(OLED_ADDR);

    Wire.write((uint8_t)0x40);

    if (c >= '-' && c <= 'Z') {

      const uint8_t \*g = font5x7 + (c - '-') \* 5;

      for (uint8_t k = 0; k < 5; k++) Wire.write(pgm_read_byte(g + k));

    } else {

      for (uint8_t k = 0; k < 5; k++) Wire.write((uint8_t)0);

    }

    Wire.write((uint8_t)0);

    Wire.endTransmission();

  }

}



void oledLineP(uint8_t page, const \_\_FlashStringHelper \*s) {

  char t[22];

  strncpy_P(t, (PGM_P)s, 21);

  t[21] = 0;

  oledLine(page, t);

}



// ---------------- TFT text helpers ----------------

void labelAt(int cx, int y, uint8_t sz, uint16_t col, const \_\_FlashStringHelper \*s) {

  PGM_P p = (PGM_P)s;

  uint8_t len = strlen_P(p);

  int x = cx - (len \* 6 \* sz) / 2;

  for (uint8_t i = 0; i < len; i++)

    tft.drawChar(x + i \* 6 \* sz, y, pgm_read_byte(p + i), col, col, sz);  // bg==col => transparent

}



void drawField(uint8_t idx, int cx, int y, uint8_t sz, uint8_t w, uint16_t col, const char \*s) {

  char o[11];

  uint8_t n = strlen(s);

  if (n > w) n = w;

  uint8_t l = (w - n) / 2;

  for (uint8_t i = 0; i < w; i++) o[i] = ' ';

  memcpy(o + l, s, n);

  int x0 = cx - (w \* 6 \* sz) / 2;

  bool force = (prevCol[idx] != col);

  prevCol[idx] = col;

  for (uint8_t i = 0; i < w; i++) {

    if (force || prevTxt[idx][i] != o[i]) {

      prevTxt[idx][i] = o[i];

      tft.drawChar(x0 + i \* 6 \* sz, y, o[i], col, COL_BG, sz);

    }

  }

}



void showVal(uint8_t idx, int cx, int y, uint8_t sz, uint8_t w, uint16_t col,

             bool valid, float v, uint8_t prec) {

  char b[14];

  if (valid) fmt(b, v, prec);

  else { strcpy_P(b, PSTR("--")); col = COL_GRAY; }

  drawField(idx, cx, y, sz, w, col, b);

}



// ---------------- MPU6050 ----------------

bool mpuWriteReg(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(reg);

  Wire.write(val);

  return Wire.endTransmission() == 0;

}



bool mpuWake() {

  if (!mpuWriteReg(0x6B, 0x00)) return false;

  mpuWriteReg(0x1A, 0x03);   // DLPF \~44 Hz

  mpuWriteReg(0x1B, 0x00);   // gyro  +-250 dps

  mpuWriteReg(0x1C, 0x00);   // accel +-2 g

  return true;

}



int16_t rd16() {

  uint8_t h = Wire.read();

  uint8_t l = Wire.read();

  return (int16_t)((h << 8) | l);

}



// r[0..2]=accel, r[3]=temp, r[4..6]=gyro

bool mpuRead(int16_t \*r) {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write((uint8_t)0x3B);

  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;

  bool nz = false;

  for (uint8_t i = 0; i < 7; i++) {

    r[i] = rd16();

    if (r[i] != 0) nz = true;

  }

  return nz;

}



// Gyro bias + accel level calibration. Bike must be upright and still.

// Rejects the batch and retries (max 4 tries) if the gyro shows motion.

bool calibrateIMU() {

  int16_t r[7], r0[3];

  for (uint8_t attempt = 0; attempt < 4; attempt++) {

    float sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0};

    uint8_t n = 0;

    int dev = 0;

    for (uint8_t i = 0; i < 100; i++) {

      if (mpuRead(r)) {

        if (n == 0) { r0[0] = r[4]; r0[1] = r[5]; r0[2] = r[6]; }

        for (uint8_t k = 0; k < 3; k++) {

          int d = abs((int)r[4 + k] - (int)r0[k]);

          if (d > dev) dev = d;

          sa[k] += r[k];

          sg[k] += r[4 + k];

        }

        n++;

      }

      delay(5);

    }

    if (n < 50) return false;                 // sensor not answering

    if (dev > 400 && attempt < 3) continue;   // moved (>\~3 deg/s): try again

    gxOff = sg[0] / n / 131.0;

    gyOff = sg[1] / n / 131.0;

    gzOff = sg[2] / n / 131.0;

    axOff = sa[0] / n / 16384.0;

    ayOff = sa[1] / n / 16384.0;

    azOff = sa[2] / n / 16384.0 - 1.0;

    return true;

  }

  return false;

}



void updateIMU(uint32_t nowMs) {

  int16_t r[7];

  if (!mpuRead(r)) {

    if (imuFail < 255) imuFail++;

    if (imuFail >= 5) {

      imuOk = false;

      if (nowMs - lastImuRetry >= 1000) { lastImuRetry = nowMs; mpuWake(); }

    }

    return;

  }

  imuFail = 0;

  uint32_t nowUs = micros();

  if (!imuOk) { imuOk = true; filterInit = false; lastUs = nowUs; }

  float dt = (nowUs - lastUs) \* 1e-6;

  lastUs = nowUs;

  if (dt <= 0.0 || dt > 0.2) dt = 0.02;



  gAx = r[0] / 16384.0 - axOff;

  gAy = r[1] / 16384.0 - ayOff;

  gAz = r[2] / 16384.0 - azOff;

  gGx = r[4] / 131.0 - gxOff;

  gGy = r[5] / 131.0 - gyOff;

  gGz = r[6] / 131.0 - gzOff;

  gTot = sqrt(gAx \* gAx + gAy \* gAy + gAz \* gAz);



  float accRoll  = atan2d(gAy, gAz);

  float accPitch = atan2d(-gAx, sqrt(gAy \* gAy + gAz \* gAz));



  if (!filterInit) {

    gRoll = accRoll; gPitch = accPitch; filterInit = true;

  } else {

    gRoll  += gGx \* dt;

    gPitch += gGy \* dt;

    if (fabs(gTot - 1.0) < 0.25) {   // trust accel only near 1 g

      gRoll  += 0.02 \* wrap180(accRoll - gRoll);

      gPitch += 0.02 \* wrap180(accPitch - gPitch);

    }

    gRoll = wrap180(gRoll);

    gPitch = wrap180(gPitch);

  }



  float lean = LEAN_SIGN \* gRoll;

  float c = cosD(lean);

  gLong = gAx + 0.017453 \* gPitch;                 // + accelerating, - braking

  gLat  = (c > 0.2) ? sinD(lean) / c : 0;          // corner G \~ tan(lean), + right



  if (lean > maxR) maxR = lean;

  if (-lean > maxL) maxL = -lean;

}



// ---------------- DS18B20 via raw OneWire (non-blocking) ----------------

void dsStart() {

  ds.reset();

  ds.skip();

  ds.write(0x44, 0);

}



void dsSetRes10() {

  if (!ds.reset()) return;

  ds.skip();

  ds.write(0x4E);

  ds.write(0);

  ds.write(0);

  ds.write(0x3F);   // 10-bit

}



void readTemp() {

  uint8_t d[9];

  bool ok = false;

  if (ds.reset()) {

    ds.skip();

    ds.write(0xBE);

    for (uint8_t i = 0; i < 9; i++) d[i] = ds.read();

    if (OneWire::crc8(d, 8) == d[8]) {

      float t = (int16_t)((d[1] << 8) | d[0]) / 16.0;

      if (t > -55.0 && t < 125.0 && t != 85.0) { tempC = t; ok = true; }

    }

  }

  if (ok) { tempOk = true; tempFail = 0; }

  else {

    if (tempFail < 255) tempFail++;

    if (tempFail >= 3) tempOk = false;

  }

}



void updateTemp(uint32_t now) {

  if (!tempReq) {

    if (now - tTemp >= TEMP_PERIOD) { dsStart(); tempReq = true; tTemp = now; }

  } else if (now - tTemp >= TEMP_CONV_MS) {

    readTemp();

    tempReq = false;

  }

}



// ---------------- TFT UI ----------------

void drawBar(float lean, uint16_t col) {

  int16_t px = imuOk ? (int16_t)(lean \* 1.8333) : 0;

  if (px > 108) px = 108;

  if (px < -108) px = -108;

  if (px == prevPx && col == prevBarCol) return;

  prevPx = px; prevBarCol = col;

  tft.fillRect(11, 117, 218, 12, COL_BG);

  if (px > 0) tft.fillRect(120, 117, px, 12, col);

  else if (px < 0) tft.fillRect(120 + px, 117, -px, 12, col);

  tft.drawFastVLine(65, 117, 12, COL_GRAY);

  tft.drawFastVLine(175, 117, 12, COL_GRAY);

  tft.drawFastVLine(120, 114, 18, COL_WHITE);

}



void drawStatic() {

  tft.fillScreen(COL_BG);



  // Header

  tft.fillRect(0, 0, 240, 4, COL_RED);

  labelAt(120, 8, 2, COL_CYAN, F("RACE"));

  labelAt(120, 26, 1, COL_YELLOW, F("MMS # ItsRacedition"));

  tft.fillRect(0, 36, 240, 2, COL_CYAN);



  // Bike number strip

  tft.drawRect(4, 40, 232, 28, COL_RED);

  labelAt(98, 47, 2, COL_CYAN, F("BIKE NO."));

  labelAt(172, 44, 3, COL_YELLOW, F("94"));



  // Lean

  tft.drawRect(4, 72, 232, 76, COL_GRAY);

  labelAt(120, 75, 1, COL_YELLOW, F("LEAN ANGLE (DEG)"));

  tft.drawRect(10, 116, 220, 14, COL_GRAY);



  // Pitch / yaw rate

  tft.drawRect(4, 152, 232, 40, COL_GRAY);

  tft.drawFastVLine(120, 154, 36, COL_GRAY);

  labelAt(62, 155, 1, COL_CYAN, F("PITCH (+UP)"));

  labelAt(178, 155, 1, COL_CYAN, F("YAW RATE"));



  // G forces

  tft.drawRect(4, 196, 232, 38, COL_GRAY);

  tft.drawFastVLine(81, 198, 34, COL_GRAY);

  tft.drawFastVLine(159, 198, 34, COL_GRAY);

  labelAt(42, 199, 1, COL_CYAN, F("ACC/BRK G"));

  labelAt(120, 199, 1, COL_CYAN, F("CORNER G"));

  labelAt(198, 199, 1, COL_CYAN, F("TOTAL G"));



  // Gyro

  tft.drawRect(4, 238, 232, 38, COL_GRAY);

  tft.drawFastVLine(81, 240, 34, COL_GRAY);

  tft.drawFastVLine(159, 240, 34, COL_GRAY);

  labelAt(42, 241, 1, COL_CYAN, F("GX DEG/S"));

  labelAt(120, 241, 1, COL_CYAN, F("GY DEG/S"));

  labelAt(198, 241, 1, COL_CYAN, F("GZ DEG/S"));



  // Temp / status

  tft.drawRect(4, 280, 232, 38, COL_GRAY);

  tft.drawFastVLine(108, 282, 34, COL_GRAY);

  labelAt(56, 283, 1, COL_CYAN, F("TEMP"));

  labelAt(172, 283, 1, COL_CYAN, F("STATUS"));

}



void updateTft() {

  char b[14];

  float lean = LEAN_SIGN \* gRoll;

  float al = fabs(lean);

  uint16_t lc = (al < 35.0) ? COL_GREEN : (al < 50.0 ? COL_YELLOW : COL_RED);



  showVal(F_LEAN, 120, 85, 4, 6, lc, imuOk, lean, 1);

  drawField(F_LSIDE, 22, 88, 3, 1, (imuOk && lean < -3.0) ? lc : COL_DIM, "L");

  drawField(F_RSIDE, 218, 88, 3, 1, (imuOk && lean > 3.0) ? lc : COL_DIM, "R");

  drawBar(lean, lc);



  strcpy_P(b, PSTR("MAX L ")); fmt(b + 6, maxL, 1);

  drawField(F_MAXL, 62, 136, 1, 10, COL_YELLOW, b);

  strcpy_P(b, PSTR("MAX R ")); fmt(b + 6, maxR, 1);

  drawField(F_MAXR, 178, 136, 1, 10, COL_YELLOW, b);



  showVal(F_PITCH, 62, 166, 3, 6, COL_WHITE, imuOk, PITCH_SIGN \* gPitch, 1);

  showVal(F_YAW, 178, 166, 3, 6, COL_WHITE, imuOk, -gGz, 1);   // + = turning right



  uint16_t gc = (gLong > 0.1) ? COL_GREEN : (gLong < -0.1 ? COL_RED : COL_WHITE);

  showVal(F_LONG, 42, 211, 2, 6, gc, imuOk, gLong, 2);

  showVal(F_LAT, 120, 211, 2, 6, COL_WHITE, imuOk, gLat, 2);

  showVal(F_TOT, 198, 211, 2, 6, COL_CYAN, imuOk, gTot, 2);



  showVal(F_GX, 42, 253, 2, 6, COL_WHITE, imuOk, gGx, 0);

  showVal(F_GY, 120, 253, 2, 6, COL_WHITE, imuOk, gGy, 0);

  showVal(F_GZ, 198, 253, 2, 6, COL_WHITE, imuOk, gGz, 0);



  if (tempOk) {

    uint16_t tc = (tempC > TEMP_CRIT) ? COL_RED : (tempC >= TEMP_WARN ? COL_YELLOW : COL_GREEN);

    fmt(b, tempC, 1);

    strcat_P(b, PSTR(" C"));

    drawField(F_TEMP, 56, 296, 2, 7, tc, b);

  } else {

    strcpy_P(b, PSTR("ERR"));

    drawField(F_TEMP, 56, 296, 2, 7, COL_RED, b);

  }



  const \_\_FlashStringHelper \*st;

  uint16_t sc;

  if (tempOk && tempC > TEMP_CRIT)  { st = F("OVER TEMP");  sc = COL_RED; }

  else if (!imuOk)                  { st = F("IMU ERROR");  sc = COL_RED; }

  else if (!tempOk)                 { st = F("TEMP ERROR"); sc = COL_RED; }

  else if (tempC >= TEMP_WARN)      { st = F("TEMP HIGH");  sc = COL_YELLOW; }

  else                              { st = F("SYSTEM OK");  sc = COL_GREEN; }

  strcpy_P(b, (PGM_P)st);

  drawField(F_STAT, 172, 296, 2, 10, sc, b);

}



// ---------------- OLED UI (one row per call) ----------------

void updateOled() {

  static uint8_t row = 0;

  char b[22], t[10];



  if (row == 0) {

    if (!tempOk) oledLineP(2, F("TEMP ERR"));

    else {

      fmt(b, tempC, 1);

      strcat_P(b, PSTR(" C"));

      oledLine(2, b);

    }

  } else if (row == 1) {

    if (!tempOk)                 oledLineP(4, F("CHECK SENSOR"));

    else if (tempC > TEMP_CRIT)  oledLineP(4, F("OVER TEMP"));

    else if (tempC >= TEMP_WARN) oledLineP(4, F("TEMP HIGH"));

    else                         oledLineP(4, F("TEMP OK"));

  } else {

    if (!imuOk) oledLineP(6, F("IMU ERROR"));

    else {

      float lean = LEAN_SIGN \* gRoll;

      if (lean < 0) strcpy_P(b, PSTR("LEAN L "));

      else          strcpy_P(b, PSTR("LEAN R "));

      fmt(t, fabs(lean), 1);

      strcat(b, t);

      oledLine(6, b);

    }

  }

  if (++row > 2) row = 0;

}



// ---------------- Setup / Loop ----------------

void setup() {

  pinMode(BUZZER_PIN, OUTPUT);



  Wire.begin();

  Wire.setClock(400000);



  tft.init(240, 320);

  tft.setRotation(0);

  tft.fillScreen(COL_BG);



  oledInit();

  oledClear();



  labelAt(120, 70, 3, COL_CYAN, F("TEAM"));

  labelAt(120, 105, 3, COL_CYAN, F("RACE"));

  labelAt(120, 180, 2, COL_YELLOW, F("CALIBRATING"));

  labelAt(120, 210, 2, COL_RED, F("KEEP STILL"));

  oledLineP(1, F("RACE"));

  oledLineP(3, F("CALIBRATING"));

  oledLineP(5, F("KEEP STILL"));



  dsSetRes10();

  dsStart();



  beepOn(82, 0, millis());   // \~1.5 kHz

  delay(120);

  beepOn(51, 0, millis());   // \~2.4 kHz

  delay(160);

  beepOff();



  bool wake = mpuWake();

  imuOk = wake && calibrateIMU();

  imuFail = 0;



  readTemp();



  drawStatic();

  oledClear();

  oledLineP(0, F("RACE"));

  updateOled();

  updateOled();

  updateOled();



  lastUs = micros();

  uint32_t now = millis();

  tMpu = tTft = tOled = tTemp = tBuz = now;

}



void loop() {

  uint32_t now = millis();



  if (now - tMpu >= MPU_INTERVAL) { tMpu = now; updateIMU(now); }

  updateTemp(now);

  updateBuzzer(now);



  if (now - tTft >= TFT_INTERVAL)   { tTft = now; updateTft(); }

  if (now - tOled >= OLED_INTERVAL) { tOled = now; updateOled(); }

}
