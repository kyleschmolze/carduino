#include <SPI.h>
#include <mcp2515.h>
#include <FastLED.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

// sources:
// https://github.com/neiman3/scangauge
// https://www.instructables.com/TachometerScan-Gauge-Using-Arduino-OBD2-and-CAN-Bu/

// misc placeholders
#define PID_RPM 0x0C
#define CAN_2515

// CAN frame structures
struct can_frame canMsg; //inbound
struct can_frame canMsgOutgoing;

// settings
long ms_easing_duration = 200; // how long to ease smooth_rpm to a newly-read RPM
long ms_between_can_requests = 100; // how long to wait between CAN requests
const boolean simulation = false; // set to true to simulate RPM values (end of file)
const int NUM_LEDS = 120;
const int LED_STRIP_PIN = 7;
const int spiCSPin = 10;

// vars
long rpm = 0;
long last_rpm = 0;
unsigned long rpm_read_at;
unsigned long last_rpm_request_at;
long smooth_rpm; // the value we ease-in-out and use for LEDs
int hue;
int brightness;

MCP2515 mcp2515(spiCSPin);
CRGB leds[NUM_LEDS];

// function to request OBD data
void requestDataOBD(unsigned long int pid) {
  canMsgOutgoing.can_id  = 0x7DF;   // request
  canMsgOutgoing.can_dlc = 8;       // length of data frame
  canMsgOutgoing.data[0] = 0x02;    // ?
  canMsgOutgoing.data[1] = 0x01;    // ?
  canMsgOutgoing.data[2] = pid;    // OBD PID that we are requesting
  canMsgOutgoing.data[3] = 0x00;   // zeros
  canMsgOutgoing.data[4] = 0x00;
  canMsgOutgoing.data[5] = 0x00;
  canMsgOutgoing.data[6] = 0x00;
  canMsgOutgoing.data[7] = 0x00;
  mcp2515.sendMessage(&canMsgOutgoing);
}

// easing function that looks nice on the eyes when LEDs do it!
double easeInOutCubic(double t) {
  return t<.5 ? 4*t*t*t : (t-1)*(2*t-2)*(2*t-2)+1;
}

void setup() {
  Serial.begin(115200);

  FastLED.addLeds<NEOPIXEL, LED_STRIP_PIN>(leds, NUM_LEDS);

  // init CAN board
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS); // Your vehicle may use a different speed!
  mcp2515.setNormalMode();

  // initial request
  requestDataOBD(PID_RPM);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long diff = millis() - rpm_read_at;
  if(diff >= ms_easing_duration) smooth_rpm = rpm; // past easing function, just set it direct
  else smooth_rpm = last_rpm + ((rpm-last_rpm) * easeInOutCubic((double) diff/ms_easing_duration)); // inside easing function
  
  // min/max values
  if(smooth_rpm < 800) smooth_rpm = 800;
  if(smooth_rpm > 5000) smooth_rpm = 5000;

  // simple hue and brightness controls
  // under 3000 RPM, ramp from yellow to red, and ramp from dim to bright
  // over 3000 RPM, ramp from red to purple, and stay full brightness
  // ref: https://raw.githubusercontent.com/FastLED/FastLED/gh-pages/images/HSV-rainbow-with-desc.jpg
  if(smooth_rpm < 3000) {
    hue = map(smooth_rpm, 800, 3000, 64, 0);
    brightness = map(smooth_rpm, 800, 3000, 120, 255);
  } else {
    hue = map(smooth_rpm, 3000, 5000, 255, 200);
    brightness = 255;
  }
  FastLED.showColor(CHSV(hue, 255, brightness)); 
  // want to track these values nicely in the Serial Plotter? Print 'em like this:
  // Serial.print("rpm: "); Serial.print(rpm);
  // Serial.print(", hue: "); Serial.print(hue);
  // Serial.print(", smooth_rpm: "); Serial.println(smooth_rpm);
  
  // Check for a received RPM message via CAN
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.data[2] == PID_RPM) {
      // Some codes use more than one byte to store the svalue. 
      // The real RPM is a conjugate of two bytes, [3] and [4].
      last_rpm = rpm;
      rpm = (canMsg.data[3]*256 + canMsg.data[4])/4; 
      rpm_read_at = millis();
    }
  }

  // request an rpm update no more than once every ms_between_can_requests milliseconds
  if ((millis() - last_rpm_request_at) > ms_between_can_requests) {
    requestDataOBD(PID_RPM);
    // update counter
    last_rpm_request_at = millis();
    
    // simulation allows us to either move the RPM around semi-randomly,
    // or just linearly cycle up from 0-5000
    if(simulation) {
      last_rpm = rpm;
      if(true) rpm += 800; // simple linear increase simulation
      else { // lovely random revving simulation
        if(rpm < 500) rpm += random(0, 800);
        else if(rpm > 4500) rpm -= random(0, 800);
        else rpm += random(-1000, 1000);
      }
      rpm = rpm % 5000;
      rpm_read_at = millis();
    }
  }
}
