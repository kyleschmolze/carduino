#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

// sources:
// https://github.com/neiman3/scangauge
// https://www.instructables.com/TachometerScan-Gauge-Using-Arduino-OBD2-and-CAN-Bu/


// misc placeholders
#define PID_RPM 0x0C
#define CAN_2515

// frame structures
struct can_frame canMsg; //inbound
struct can_frame canMsgOutgoing;

// settings
long ms_easing_duration = 200;
long ms_between_can_requests = 200;

// vars
unsigned long rpm_read_at;
unsigned long last_rpm_request_at;
long rpm = 0;
long last_rpm = 0;
long smooth_rpm;

const int spiCSPin = 10;
const boolean simulation = false;

MCP2515 mcp2515(spiCSPin);

const int NUM_LEDS = 120;
const int LED_STRIP_PIN = 7;

int red;
int green;
int blue;
int brightness;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

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

double easeInOutCubic(double t) {
  return t<.5 ? 4*t*t*t : (t-1)*(2*t-2)*(2*t-2)+1;
}

double easeInOutSine(double t) {
  return -(cos(PI*t)-1)/2;
}

void setup() {
  Serial.begin(115200);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // init can board
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS); // Your vehicle may use a different speed!
  mcp2515.setNormalMode();

  requestDataOBD(PID_RPM);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long diff = millis() - rpm_read_at;
  if(diff >= ms_easing_duration) smooth_rpm = rpm; // past easing function, just set it direct
  else smooth_rpm = last_rpm + ((rpm-last_rpm) * easeInOutSine((double) diff/ms_easing_duration)); // inside easing function
  
  //Serial.println("diff: "); Serial.println(diff); Serial.println("ms_easing_duration: "); Serial.println(ms_easing_duration);
  Serial.print("rpm: "); Serial.print(rpm); //Serial.print("last_rpm: "); Serial.print(last_rpm);
  Serial.print(", smooth_rpm: "); Serial.println(smooth_rpm);
  
  if(smooth_rpm < 0) smooth_rpm = 0;
  if(smooth_rpm > 8000) smooth_rpm = 8000;

  
  int startingRpm = 800;
  int startingRed = 255;
  int startingGreen = 255;
  int startingBlue = 0;
  int startingBrightness = 50;
  int cutoff1 = 3000;
  int cutoff1Red = 255;
  int cutoff1Green = 0;
  int cutoff1Blue = 0;
  int cutoff1Brightness = 150;
  int cutoff2 = 4000;
  int cutoff2Red = 255;
  int cutoff2Green = 0;
  int cutoff2Blue = 130;
  int cutoff2Brightness = 255;
  int cutoff3 = 5000;
  int cutoff3Red = 255;
  int cutoff3Green = 255;
  int cutoff3Blue = 255;
  int cutoff3Brightness = 255;

  if(smooth_rpm < startingRpm) {
    red = startingRed;
    green = startingGreen;
    blue = startingBlue;
    brightness = startingBrightness;
  } else if(smooth_rpm < cutoff1) {
    red = startingRed + ((double) (smooth_rpm-startingRpm)/(cutoff1-startingRpm)*(cutoff1Red-startingRed));
    green = startingGreen + ((double) (smooth_rpm-startingRpm)/(cutoff1-startingRpm)*(cutoff1Green-startingGreen));
    blue = startingBlue + ((double) (smooth_rpm-startingRpm)/(cutoff1-startingRpm)*(cutoff1Blue-startingBlue));
    brightness = startingBrightness + ((double) (smooth_rpm-startingRpm)/(cutoff1-startingRpm)*(cutoff1Brightness-startingBrightness));
  } else if(smooth_rpm < cutoff2) {
    red = cutoff1Red + ((double) (smooth_rpm-cutoff1)/(cutoff2-cutoff1)*(cutoff2Red-cutoff1Red));
    green = cutoff1Green + ((double) (smooth_rpm-cutoff1)/(cutoff2-cutoff1)*(cutoff2Green-cutoff1Green));
    blue = cutoff1Blue + ((double) (smooth_rpm-cutoff1)/(cutoff2-cutoff1)*(cutoff2Blue-cutoff1Blue));
    brightness = cutoff1Brightness + ((double) (smooth_rpm-cutoff1)/(cutoff2-cutoff1)*(cutoff2Brightness-cutoff1Brightness));
  } else if(smooth_rpm < cutoff3) {
    red = cutoff2Red + ((double) (smooth_rpm-cutoff2)/(cutoff3-cutoff2)*(cutoff3Red-cutoff2Red));
    green = cutoff2Green + ((double) (smooth_rpm-cutoff2)/(cutoff3-cutoff2)*(cutoff3Green-cutoff2Green));
    blue = cutoff2Blue + ((double) (smooth_rpm-cutoff2)/(cutoff3-cutoff2)*(cutoff3Blue-cutoff2Blue));
    brightness = cutoff2Brightness + ((double) (smooth_rpm-cutoff2)/(cutoff3-cutoff2)*(cutoff3Brightness-cutoff2Brightness));

  } else {
    red = cutoff3Red;
    green = cutoff3Green;
    blue = cutoff3Blue;
    brightness = cutoff3Brightness;
  }

  //Serial.print("smooth_rpm: "); Serial.print(smooth_rpm);
  //Serial.print(", red: "); Serial.print(red);
  //Serial.print(", green: "); Serial.print(green);
  //Serial.print(", blue: "); Serial.print(blue);
  //Serial.print(", brightness: "); Serial.println(brightness);

  for(int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(red, green, blue));
    strip.setBrightness(brightness);
  }
  strip.show();
  
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.data[2] == PID_RPM) { // ICE RPM
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
