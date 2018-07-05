// ServoBox

// Richard Sewell, 2018

// Servo tester & monitor
// For :
//  testing servos
//  finding min/max signals to suit a specific installation
//  monitoring servo signals in a running installation (and enforcing min/max)
//  monitoring servo voltage & current


// Runs on a D1 Mini Pro 1.1
// pinout:
// https://wiki.wemos.cc/products:retired:d1_mini_pro_v1.1.0

// Hardware:

// Wemos OLED shield: https://wiki.wemos.cc/products:d1_mini_shields:oled_shield
// Generic click-encoder for UI
// Adafruit INA219 high-side current & voltage monitor: https://www.adafruit.com/product/904


// Uses 
// ArduinoMenu from https://github.com/neu-rah/ArduinoMenu (but you can get it via the LibraryManager)
// ClickEncoder from https://github.com/0xPIT/encoder/tree/arduino (which you download yourself)

#include <Arduino.h>

// required to make the timer to run ClickEncoder on ESP8266
extern "C" {
#include "user_interface.h"
}


#include <menu.h>

// OLED support
#include <menuIO/u8g2Out.h>

#define USE_ENCODER

#ifdef USE_ENCODER
#include <ClickEncoder.h> // you may need to edit this to comment out some AVR-specific includes
#include <menuIO/clickEncoderIn.h>
#endif  

#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>

#include "Servo.h"

using namespace Menu;


// "Boot from Flash" requires some pin state to be right:
// D3/GPIO0 = 1 (high)
// D4/GPIO2 = 1 (high)
// D8/GPIO15 = 0 (low)
// so use those as outputs, or inputs which will be in the right state at boot time


// Pins are
//  I2C
//    D1 SCL
//    D2 SDA
//      wired to OLED and to mux and to servo board

// Encoder is on D5 & D6, switch on D4

// Servo out is on D3
// Servo in is on D7

// i2c addresses:
// Oled   : 0x3C
// INA219 current sensor: 0x40
#define encA D5
#define encB D6
#define encBtn D4

#define SERVO_OUT_PIN D3
#define SERVO_IN_PIN D7


// #define USE_PCD8544
#define USE_SSD1306

#if defined(USE_PCD8544)
// not doing this one
#elif defined(USE_SSD1306)


  #include <Wire.h>
  #define fontName u8g2_font_5x7_mf
  #define fontX 5
  #define fontY 9
  #define offsetX 0
  #define offsetY 0
  #define U8_Width 64
  #define U8_Height 48
  #define USE_HWI2C
  // U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);//, SCL, SDA);
  // U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE, 4, 5);
  // U8G2_SSD1306_128X64_VCOMH0_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE, 4, 5);
  //U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 4, 5);
  U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0); //wemos shield suport: https://github.com/olikraus/u8g2/wiki/gallery#24-mar-2017-wemos-oled-shield
  // which derives from U8G2 in U8g2lib.h
#else
  #error DEFINE YOUR OUTPUT HERE.
#endif


// define menu colors --------------------------------------------------------
//each color is in the format:
//  {{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}
// this is a monochromatic color table
const colorDef<uint8_t> colors[] MEMMODE={
  {{0,0},{0,1,1}},//bgColor
  {{1,1},{1,0,0}},//fgColor
  {{1,1},{1,0,0}},//valColor
  {{1,1},{1,0,0}},//unitColor
  {{0,1},{0,0,1}},//cursorColor
  {{1,1},{1,0,0}},//titleColor
};



volatile long servoInOnUS = -1;
volatile long servoInOffUS = -1;
long servoTimeoutUS = 1000000L;
float servoInDegrees = -359.0;
long servoInUS = -1;

long lastManualUpdateMs = 0;

float current_mA = 0;
float smoothedCurrent_mA = 0;
float loadvoltage = 0;
float currentSmoother = 0.1;

float currentLog[U8_Width];
long currentLogInterval = 500;
long lastCurrentLog = 0;

Servo servoOut;

void setOutVal( float val );

result doAlert(eventMask e, prompt &item);

int test=55;


float fmapConstrained(float x, float in_min, float in_max, float out_min, float out_max)
{
  float f = fmap( x,  in_min, in_max, out_min, out_max);

  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float fconstrain(float f, float out_min, float out_max)
{
  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}


float outMin=0;
float outMax=180;
float outVal = 90;

void updateOutMin() {
  if( outMin > outMax-1 )
    outMin = outMax - 1;

    //Serial.println("updateOutMin");
  if( gotServoIn())
    updateOutVal(); // just constrain it to be in range
  else
    setOutVal(outMin); // move to the new min
}

void updateOutMax() {
  if( outMax < outMin + 1)
    outMax = outMin + 1;

    //Serial.println("updateOutMax");

  if( gotServoIn())
    updateOutVal();
  else
   setOutVal(outMax);
  
}

void setOutValFromInVal( float val )
{
  if( millis() - lastManualUpdateMs > 10000 )
    setOutVal( val );
}


void setOutVal( float val )
{
  //Serial.print("setOutVal ");
  //Serial.println(val);
  outVal = val;
  updateOutVal();  
}

void updateOutValFromMenu()
{
  lastManualUpdateMs = millis();
  updateOutVal();
}


void updateOutVal() {
  if( outVal < outMin )
    outVal = outMin;

  if( outVal > outMax )
    outVal = outMax;

  //Serial.print("updateOutVal ");
  //  Serial.println(outVal);

  // and write to servo
  servoOut.writeMicroseconds(degreesToMicroseconds(outVal));
}

int degreesToMicroseconds( float degrees)
{
  return( fmap( degrees, 0, 180, 1000, 2000 ));
}

float microsecondsToDegrees( long microseconds )
{
  float deg = fmap( microseconds, 1000.0, 2000.0, 0.0, 180.0 );
  deg = fconstrain(deg, -360.0, 360.0);
  return deg;
}

class currentGraphPrompt:public prompt {
public:
  unsigned int t=0;
  unsigned int last=0;
  currentGraphPrompt(constMEM promptShadow& p):prompt(p) {}
  Used printTo(navRoot &root,bool sel,menuOut& out, idx_t idx,idx_t len,idx_t panelNr) override {

    last=t;
    
    gfxOut*g = (gfxOut*)&out;
   u8g2Out*u = (u8g2Out*)&out;

    // look in u8g2lib.h for these
    u8g2.setDrawColor(u->getColor(fgColor,sel,enabledStatus,false));

    // look at rect() in u8g2out.h for the source of this position arithmetic
    const panel p=out.panels[panelNr];
 
    
    int bottom = (p.y+idx+2) * g->resY; // 2 ? Really ?
    int height = g->resY;
    int top = bottom-height;
    
    if( bottom < 0 || top >  u8g2.getDisplayHeight())
    {
      Serial.println("not drawing");
      return 0; // not on screen
    }
    Serial.print(millis()); Serial.println("- drawing");
    for( int x = p.x; x < u8g2.getDisplayWidth();x++)
    {
      int barHeight = 1 + fmap(currentLog[x], 0.0, 1000.0, 0.0, (float)g->resY);
      
      
      int y = bottom-barHeight;
      int h = barHeight;
      /*
      Serial.print("x ");
      Serial.print(x);
      Serial.print(" currentLog[x] ");
      Serial.println(currentLog[x]);
      Serial.print(" barHeight ");
      Serial.println(barHeight);
      */
       
      u8g2.drawVLine(x,y,h);
    }
     
    return 1;
  }
  
  virtual bool changed(const navNode &nav,const menuOut& out,bool sub=true) {

    // is never called!
    
    t=millis()/currentLogInterval;
    if( last!=t )
    {
      Serial.println("changed");
      return true;
    }
    else
    {
      Serial.println("Not changed");
      return false;
    }
  }
};

MENU(mainMenu,"ServoBox",doNothing,noEvent,wrapStyle
  ,FIELD(outVal,"Out","",-360,360,10,1, updateOutValFromMenu, enterEvent, noStyle)
  ,FIELD(servoInDegrees,"In","",-360,360,10,1, NULL, enterEvent, noStyle)
  ,altOP(currentGraphPrompt,"",doNothing,noEvent)
  ,FIELD(outMin,"Min","",-360,360,10,1, updateOutMin, enterEvent, noStyle)
  ,FIELD(outMax,"Max","",-360,360,10,1, updateOutMax, enterEvent, noStyle)
  ,FIELD(smoothedCurrent_mA,"","mA",0,100,10,1, NULL, enterEvent, noStyle)
  ,FIELD(loadvoltage,"","V",-3000,3000,10,1, NULL, enterEvent, noStyle)
);

#define MAX_DEPTH 2

#ifdef USE_ENCODER
ClickEncoder clickEncoder(encA,encB,encBtn,2);
ClickEncoderStream encStream(clickEncoder,1);
MENU_INPUTS(in,&encStream);
os_timer_t myTimer;
void timerCallback(void *)
{
  clickEncoder.service();
  //ignoreCurrentPulse(); // ignore pulses which overlap encoder service calls, we'll probably measure their duraiton incorrectly
  // turns out this ignores all the pulses...
}

#else
serialIn serial(Serial);
MENU_INPUTS(in,&serial);
#endif

MENU_OUTPUTS(out,MAX_DEPTH
  ,U8G2_OUT(u8g2,colors,fontX,fontY,offsetX,offsetY,{0,0,U8_Width/fontX,U8_Height/fontY})
  ,NONE //,SERIAL_OUT(Serial)
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);

result alert(menuOut& o,idleEvent e) {
  if (e==idling) {
    o.setCursor(0,0);
    o.print("alert test");
    o.setCursor(0,1);
    o.print("press [select]");
    o.setCursor(0,2);
    o.print("to continue...");
  }
  return proceed;
}

result doAlert(eventMask e, prompt &item) {
  nav.idleOn(alert);
  return proceed;
}

//when menu is suspended
result idle(menuOut& o,idleEvent e) {
  o.clear();
  switch(e) {
    case idleStart:o.println("suspending menu!");break;
    case idling:o.println("suspended...");break;
    case idleEnd:o.println("resuming menu.");break;
  }
  return proceed;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("ServoBox test");Serial.flush();

  
 
  
  // pinMode(LEDPIN,OUTPUT);//cant use pin 13 when using hw spi
  // and on esp12 i2c can be on pin 2, and that is also led pin
  // so check first if this is adequate for your board
  #if defined(USE_HWSPI)
    SPI.begin();
    u8g2.begin();
  #elif defined(USE_HWI2C)
    Wire.begin();
    u8g2.begin();
  #else
    #error "please choose your interface (I2c,SPI)"
  #endif
  u8g2.setFont(fontName);
  // u8g2.setBitmapMode(0);

  // disable second option
  mainMenu[1].enabled=disabledStatus;
  nav.idleTask=idle;//point a function to be used when menu is suspended

#ifdef USE_ENCODER
  os_timer_setfn(&myTimer, timerCallback, NULL);//set the callback funtion for the encoder
  os_timer_arm(&myTimer, 1, true);//setup the timer tick to call it every ms
#endif

  servoOut.attach(SERVO_OUT_PIN);
  clickEncoder.setAccelerationEnabled(true);

  setupServoIn();

  setupCurrent();
  
  Serial.println("setup done.");Serial.flush();
}

void loop() {
  loopServoIn();
  loopCurrent();
  nav.doInput();
  // digitalWrite(LEDPIN, ledCtrl);
  if (nav.changed(0)) {//only draw if menu changed for gfx device
    //change checking leaves more time for other tasks
    u8g2.firstPage();
    do nav.doOutput(); while(u8g2.nextPage());
  }
  delay(10);//simulate other tasks delay
}


