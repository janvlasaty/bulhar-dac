#include "Arduino.h"
#include "esp_timer.h"
#include <Servo.h>
#include <FastLED.h>
#include "PCF8574.h"

// The 8-bit values in our sound sample
#include "track.h"
#include "pacman_fail.h"
#include "pacman_sirene.h"
#include "pacman_intro.h"
#include "pacman_steps.h"
#include "sound_success.h"

#define DEBUG true
// board pinout
/*
  3V3           [                     ]  GND
  RESET         [ reset?          sz  ]  GPIO 23
  x             [                SCL  ]  GPIO 22
  x             [                     ]  x
  GPIO 34 ionly [ moneyIR             ]  x
  GPIO 35 ionly [ priceIR        SDA  ]  GPIO 21
  GPIO 32       [ sy+                 ]  GND
  GPIO 33       [ sy-            bx+  ]  GPIO 19
  GPIO 25 dac   [ sound          bx-  ]  GPIO 18
  GPIO 26 dac   [ sx+            by+  ]  GPIO 5
  GPIO 27       [                by-  ]  GPIO 17
  GPIO 14       [                bz+  ]  GPIO 16
  GPIO 12       [                bz-  ]  GPIO 4
  GND           [              servo  ]  GPIO 0
  GPIO 13       [                led  ]  GPIO 2
  x             [                sx-  ]  GPIO 15
  x             [                     ]  x
  x             [                     ]  x
  5V            [                     ]  x
*/
// define switch inputs
#define PIN_SX_PLUS 26
#define PIN_SX_MINUS 15
#define PIN_SY_PLUS 33
#define PIN_SY_MINUS 32
#define PIN_SZ 23
// define break inputs
#define PIN_BX_PLUS 19
#define PIN_BX_MINUS 18
#define PIN_BY_PLUS 5
#define PIN_BY_MINUS 17
#define PIN_BZ_PLUS 16
#define PIN_BZ_MINUS 4
// define IR inputs
#define MONEY_IR 34
#define PRICE_IR 35
// define I2C bus pin
#define PIN_SDA 21
#define PIN_SCL 22
// define motor inputs
#define BUS_MX_PLUS 0
#define BUS_MX_MINUS 1
#define BUS_MY_PLUS 2
#define BUS_MY_MINUS 3
#define BUS_MZ_PLUS 4
#define BUS_MZ_MINUS 5

// define motor outputs
// #define PIN_MX_PLUS 27
// #define PIN_MX_MINUS 14
// #define PIN_MY_PLUS 12
// #define PIN_MY_MINUS 13
// #define PIN_MZ_PLUS 23
// #define PIN_MZ_MINUS 22

// define sound outputs
#define PIN_SOUND 25
// define led outputs
#define PIN_LED 2
// define servo outputs
#define PIN_SERVO 0

// i2c bus
PCF8574 PCF_20(0x20);

// state variables
// game
/*
  0 = off
  1 = preparing
  2 = playing
  3 = craying
  4 = releasing cray & game resolution
  5 = win
  6 = lose
*/
int gameMode = 0;
int previousGameMode = -1;
uint32_t gameStartTime = 0;
int gameDuration = 60000;          // ms
int gameResolutionDuration = 1000; // ms

// breaks
bool bXPlus = false;
bool bXMinus = false;
bool bYPlus = false;
bool bYMinus = false;
bool bZPlus = false;
bool bZMinus = false;

// motors state
/*
  0 = stop
  1 = going +
  2 = going -
*/
int mXState = 0;
int previousMXState = 0;
int mYState = 0;
int previousMYState = 0;
int mZState = 0;
int previousMZState = 0;

// craying
/*
  0 = stop motors XY
  1 = going down
  2 = craying
  3 = going up
  4 = moving over the hole
*/
int crayingMode = 0;
int previousCrayingMode = -1;

int logNumber = 0;

// servo
Servo servo = Servo();
void setServo(int degrees)
{
  servo.write(PIN_SERVO, degrees);
}

// motor functions
void setMotor(char axis, int state)
{
  switch (axis)
  {
  case 'x':
    mXState = state;
    break;
  case 'y':
    mYState = state;
    break;
  case 'z':
    mZState = state;
    break;
  }
}
void setMotorIfPlaying(char axis, int state)
{
  if (gameMode == 2)
    setMotor(axis, state);
}
void stopMotorsXY()
{
  if (DEBUG)
    Serial.println("stopMotorsXY");
  // stop motors
  setMotor('x', 0);
  setMotor('y', 0);
}

// interrupt handlers
void IRAM_ATTR sXPChange()
{
  if (digitalRead(PIN_SX_PLUS) == LOW)
  {
    if (DEBUG)
      logNumber = 1;
    if (!bXPlus)
      setMotorIfPlaying('x', 1);
  }
  else
  {
    if (DEBUG)
      logNumber = 2;
    setMotorIfPlaying('x', 0);
  }
}
void IRAM_ATTR sXMChange()
{
  if (digitalRead(PIN_SX_MINUS) == LOW)
  {
    if (DEBUG)
      logNumber = 3;
    if (!bXMinus)
      setMotorIfPlaying('x', 2);
  }
  else
  {
    if (DEBUG)
      logNumber = 4;
    setMotorIfPlaying('x', 0);
  }
}
void IRAM_ATTR sYPChange()
{
  if (digitalRead(PIN_SY_PLUS) == LOW)
  {
    if (DEBUG)
      logNumber = 5;
    if (!bYPlus)
      setMotorIfPlaying('y', 1);
  }
  else
  {
    if (DEBUG)
      logNumber = 6;
    setMotorIfPlaying('y', 0);
  }
}
void IRAM_ATTR sYMChange()
{
  if (digitalRead(PIN_SY_MINUS) == LOW)
  {
    if (DEBUG)
      logNumber = 7;
    if (!bYMinus)
      setMotorIfPlaying('y', 2);
  }
  else
  {
    if (DEBUG)
      logNumber = 8;
    setMotorIfPlaying('y', 0);
  }
}
void IRAM_ATTR sZChange()
{
  if (digitalRead(PIN_SZ) == LOW)
  {
    if (DEBUG)
      logNumber = 9;
    if (gameMode == 2)
      // move to craying
      gameMode = 3;
  }
}
void IRAM_ATTR bXPChange()
{
  if (digitalRead(PIN_BX_PLUS) == LOW)
  {
    if (DEBUG)
      logNumber = 10;
    bXPlus = true;
    setMotor('x', 0);
  }
  else
  {
    if (DEBUG)
      logNumber = 11;
    bXPlus = false;
  }
}
void IRAM_ATTR bXMChange()
{
  if (digitalRead(PIN_BX_MINUS) == LOW)
  {
    if (DEBUG)
      logNumber = 12;
    bXMinus = true;
    setMotor('x', 0);
  }
  else
  {
    if (DEBUG)
      logNumber = 13;
    bXMinus = false;
  }
}
void IRAM_ATTR bYPChange()
{
  if (digitalRead(PIN_BY_PLUS) == LOW)
  {
    if (DEBUG)
      logNumber = 15;
    bYPlus = true;
    setMotor('y', 0);
  }
  else
  {
    if (DEBUG)
      logNumber = 16;
    bYPlus = false;
  }
}
void IRAM_ATTR bYMChange()
{
  if (digitalRead(PIN_BY_MINUS) == LOW)
  {
    if (DEBUG)
      logNumber = 17;
    bYMinus = true;
    setMotor('y', 0);
  }
  else
  {
    if (DEBUG)
      logNumber = 18;
    bYMinus = false;
  }
}
void IRAM_ATTR bZPChange()
{
  if (digitalRead(PIN_BZ_PLUS) == LOW)
  {
    if (DEBUG)
      logNumber = 19;
    bZPlus = true;
  }
  else
  {
    if (DEBUG)
      logNumber = 20;
    bZPlus = false;
  }
}
void IRAM_ATTR bZMChange()
{
  if (digitalRead(PIN_BZ_MINUS) == LOW)
  {
    if (DEBUG)
      logNumber = 21;
    bZMinus = true;
  }
  else
  {
    if (DEBUG)
      logNumber = 22;
    bZMinus = false;
  }
}

void priceIRDetected()
{
  if (DEBUG)
    logNumber = 23;
  if (gameMode == 4)
    gameMode = 5;
}
void moneyIRDetected()
{
  if (DEBUG)
    logNumber = 24;
  if (gameMode == 0)
  {
    // start a game
    gameStartTime = millis();
    gameMode = 1;
  }
}

void crayClose()
{
  if (DEBUG)
    Serial.println("crayClose");
  for (int posDegrees = 0; posDegrees <= 179; posDegrees++)
  {
    setServo(posDegrees);
    delay(10);
  }
  delay(1000);
}
void crayOpen()
{
  if (DEBUG)
    Serial.println("crayOpen");
  for (int posDegrees = 179; posDegrees >= 0; posDegrees--)
  {
    setServo(posDegrees);
    delay(10);
  }
  delay(1000);
}
void crayAction()
{
  if (previousCrayingMode != crayingMode)
  {
    if (DEBUG)
      Serial.printf("crayingMode: %i\n", crayingMode);
    previousCrayingMode = crayingMode;
  }
  switch (crayingMode)
  {
  case 0:
    // stop motors XY
    stopMotorsXY();
    setMotor('z', 1);
    crayingMode = 1;
    break;
  case 1:
    // going down
    if (bZMinus == true)
    {
      setMotor('z', 0);
      crayingMode = 2;
    }
    break;
  case 2:
    // craying
    crayClose();
    setMotor('z', 2);
    crayingMode = 3;
    break;
  case 3:
    // going up until over the hole
    if (bZPlus == true)
    {
      // stop Z motor and start XY motors
      setMotor('z', 0);
      setMotor('x', 1);
      setMotor('y', 1);
      crayingMode = 4;
    }
    break;
  case 4:
    // moving over the hole
    if (bXMinus == true && bYMinus == true)
    {
      crayingMode = 0;
      gameMode = 4;
      return;
    }
    break;
  }
}

// sound timer
hw_timer_t *timerSound = NULL;
int soundIndex = 0;
int trackIndex = 0;
bool soundOn = false;
bool loopOn = false;
void IRAM_ATTR onTimerSound()
{
  if (soundOn)
  {
    int soundLength = 0;
    switch (trackIndex)
    {
    case 0:
      dacWrite(PIN_SOUND, acc_sound[soundIndex] / 64);
      soundLength = acc_sound_length;
      break;
    case 3:
      dacWrite(PIN_SOUND, pacman_fail[soundIndex] / 64);
      soundLength = pacman_fail_length;
      break;
    case 4:
      dacWrite(PIN_SOUND, pacman_sirene[soundIndex] / 64);
      soundLength = pacman_sirene_length;
      break;
    case 5:
      dacWrite(PIN_SOUND, pacman_intro[soundIndex] / 64);
      soundLength = pacman_intro_length;
      break;
    case 6:
      dacWrite(PIN_SOUND, pacman_steps[soundIndex] / 128);
      soundLength = pacman_steps_length;
      break;
    case 7:
      dacWrite(PIN_SOUND, sound_success[soundIndex] / 64);
      soundLength = sound_success_length;
      break;
    }
    if (soundIndex < soundLength)
    {
      soundIndex++;
    }
    if (soundIndex >= soundLength && loopOn)
    {
      soundIndex = 0;
    }
  }
}
void playSound(int track, bool loop = false)
{
  soundIndex = 0;
  trackIndex = track;
  loopOn = loop;
  soundOn = true;
}

// led timer
#define NUM_LEDS 50
CRGB leds[NUM_LEDS];
int ledIndex = 0;
unsigned long lastLedShow = millis();
int ledShowInterval = 50;
hw_timer_t *timerLeds = NULL;
void IRAM_ATTR onTimerLeds()
{
  if (ledIndex < NUM_LEDS)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
    }
    leds[ledIndex] = CRGB::Green;
    ledIndex++;
  }
  if (ledIndex >= NUM_LEDS)
  {
    ledIndex = 0;
  }
}

// dummy delay
unsigned long delayStart;
int delayDuration = 5000;
bool delayOn = false;
void startDummyDelay(int duration)
{
  if (delayOn)
  {
    if (millis() - delayStart > delayDuration)
    {
      delayOn = false;
    }
  }
  else
  {
    delayStart = millis();
    delayDuration = duration;
    delayOn = true;
  }
}

// setup pins
void setupPins()
{
  // set pinout buttons

  // sx+
  pinMode(PIN_SX_PLUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SX_PLUS), sXPChange, CHANGE);
  // sx-
  pinMode(PIN_SX_MINUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SX_MINUS), sXMChange, CHANGE);
  // sy+
  pinMode(PIN_SY_PLUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SY_PLUS), sYPChange, CHANGE);
  // sy-
  pinMode(PIN_SY_MINUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SY_MINUS), sYMChange, CHANGE);
  // sz
  pinMode(PIN_SZ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SZ), sZChange, CHANGE);
  // bx+
  pinMode(PIN_BX_PLUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BX_PLUS), bXPChange, CHANGE);
  // bx-
  pinMode(PIN_BX_MINUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BX_MINUS), bXMChange, CHANGE);
  // by+
  pinMode(PIN_BY_PLUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BY_PLUS), bYPChange, CHANGE);
  // by-
  pinMode(PIN_BY_MINUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BY_MINUS), bYMChange, CHANGE);
  // bz+
  pinMode(PIN_BZ_PLUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BZ_PLUS), bZPChange, CHANGE);
  // bz-
  pinMode(PIN_BZ_MINUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BZ_MINUS), bZMChange, CHANGE);

  // // set pinout motors
  // pinMode(PIN_MX_PLUS, OUTPUT);
  // pinMode(PIN_MX_MINUS, OUTPUT);
  // pinMode(PIN_MY_PLUS, OUTPUT);
  // pinMode(PIN_MY_MINUS, OUTPUT);
  // pinMode(PIN_MZ_PLUS, OUTPUT);
  // pinMode(PIN_MZ_MINUS, OUTPUT);

  // // set pinout sound
  pinMode(PIN_SOUND, OUTPUT);

  // // set pinout led
  pinMode(PIN_LED, OUTPUT);

  // // set pinout IR
  pinMode(PRICE_IR, INPUT_PULLDOWN);
  attachInterrupt(PRICE_IR, priceIRDetected, RISING);
  pinMode(MONEY_IR, INPUT_PULLDOWN);
  attachInterrupt(MONEY_IR, moneyIRDetected, RISING);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Setup pins");
  setupPins();

  Serial.println("PCF8574 begin");
  Wire.begin(PIN_SDA, PIN_SCL);
  PCF_20.begin(0x20);
  PCF_20.write(BUS_MX_PLUS, LOW);
  PCF_20.write(BUS_MX_MINUS, LOW);
  PCF_20.write(BUS_MY_PLUS, LOW);
  PCF_20.write(BUS_MY_MINUS, LOW);
  PCF_20.write(BUS_MZ_PLUS, LOW);
  PCF_20.write(BUS_MZ_MINUS, LOW);

  Serial.println("Sound playing");
  timerSound = timerBegin(0, 80, true);
  timerAttachInterrupt(timerSound, &onTimerSound, true);
  timerAlarmWrite(timerSound, 62, true);
  timerAlarmEnable(timerSound);

  Serial.println("LED timing");
  FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  timerLeds = timerBegin(1, 80, true);
  timerAttachInterrupt(timerLeds, &onTimerLeds, true);
  timerAlarmWrite(timerLeds, 1000000, true);
  timerAlarmEnable(timerLeds);

  Serial.println("Cray open");
  crayOpen();
}

void loopLeds()
{
  unsigned long now = millis();
  if (now - lastLedShow > ledShowInterval)
  {
    lastLedShow = now;
    FastLED.show();
  }
}
void loopDebug()
{
  if (DEBUG)
  {
    // int sxplus = digitalRead(PIN_SX_PLUS);
    // Serial.printf("sxplus: %i\n", sxplus);
    // print log
    if (logNumber > 0)
    {
      Serial.printf("logNumber: %i\n", logNumber);
      logNumber = 0;
    }
  }
}
void loopMotors()
{
  if (mXState != previousMXState)
  {
    Serial.printf("mXState: %i\n", mXState);
    PCF_20.write(BUS_MX_PLUS, LOW);
    PCF_20.write(BUS_MX_MINUS, LOW);
    switch (mXState)
    {
    case 1:
      PCF_20.write(BUS_MX_PLUS, HIGH);
      break;
    case 2:
      PCF_20.write(BUS_MX_MINUS, HIGH);
      break;
    }
    previousMXState = mXState;
  }
  if (mYState != previousMYState)
  {
    PCF_20.write(BUS_MY_PLUS, LOW);
    PCF_20.write(BUS_MY_MINUS, LOW);
    switch (mYState)
    {
    case 1:
      PCF_20.write(BUS_MY_PLUS, HIGH);
      break;
    case 2:
      PCF_20.write(BUS_MY_MINUS, HIGH);
      break;
    }
    previousMYState = mYState;
  }
  if (mZState != previousMZState)
  {
    PCF_20.write(BUS_MZ_PLUS, LOW);
    PCF_20.write(BUS_MZ_MINUS, LOW);
    switch (mZState)
    {
    case 1:
      PCF_20.write(BUS_MZ_PLUS, HIGH);
      break;
    case 2:
      PCF_20.write(BUS_MZ_MINUS, HIGH);
      break;
    }
    previousMZState = mZState;
  }
}
void loopGameMode()
{
  bool isNewGameMode = false;
  if (gameMode != previousGameMode)
  {
    if (DEBUG)
      Serial.printf("gameMode: %i\n", gameMode);
    previousGameMode = gameMode;
    isNewGameMode = true;
  }
  switch (gameMode)
  {
  case 0:
    if (isNewGameMode)
    {
      playSound(0, true);
    }
    // off
    break;
  case 1:
    if (isNewGameMode)
    {
      playSound(5, false);
    }
    // preparing
    startDummyDelay(5000);
    if (!delayOn)
      gameMode = 2;
    break;
  case 2:
    // playing
    if (isNewGameMode)
    {
      playSound(6, true);
    }
    startDummyDelay(gameDuration);
    if (!delayOn)
      gameMode = 3;
    break;
  case 3:
    if (isNewGameMode)
    {
      playSound(4, true);
    }
    // craying
    crayAction();
    break;
  case 4:
    // releasing cray & game resolution
    crayOpen();
    if (gameMode != 5)
      gameMode = 6;
    break;
  case 5:
    // win
    if (isNewGameMode)
    {
      playSound(7, false);
    }
    // playSound(0);
    startDummyDelay(10000);
    if (!delayOn)
      gameMode = 0;
    break;
  case 6:
    // lose
    if (isNewGameMode)
    {
      playSound(3, false);
    }
    startDummyDelay(10000);
    if (!delayOn)
      gameMode = 0;
    break;
  }
}

void loop()
{
  // led show
  loopLeds();

  // debug
  loopDebug();

  // set motors
  loopMotors();

  // print gameMode
  loopGameMode();
}
