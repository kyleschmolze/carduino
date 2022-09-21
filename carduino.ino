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
long rpm = 9999;
long last_rpm = 0;
long smooth_rpm;

const int spiCSPin = 10;
const boolean simulation = true;

MCP2515 mcp2515(spiCSPin);

const int NUM_LEDS = 30;
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
  if(rpm != 9999) {
    unsigned long diff = millis() - rpm_read_at;
    if(diff >= ms_easing_duration) smooth_rpm = rpm; // past easing function, just set it direct
    else smooth_rpm = last_rpm + ((rpm-last_rpm) * ((double) diff/ms_easing_duration)); // inside easing function
    
    //Serial.println("diff: "); Serial.println(diff); Serial.println("ms_easing_duration: "); Serial.println(ms_easing_duration);
    Serial.print("rpm: "); Serial.print(rpm); //Serial.print("last_rpm: "); Serial.print(last_rpm);
    Serial.print(", smooth_rpm: "); Serial.println(smooth_rpm);
    
    if(smooth_rpm < 0) smooth_rpm = 0;
    if(smooth_rpm > 8000) smooth_rpm = 8000;

    int red = ((double) smooth_rpm/5000)*255;
    //Serial.println(red);
    if(smooth_rpm < 3000) {
      // yellow 255, 255, 0
      // TO
      // red 255, 0, 0
      red = 255;
      green = 255 - ((double) smooth_rpm/3000)*255;
      blue = 0;
      brightness = ((double) smooth_rpm/5000)*255;
    } else if (smooth_rpm < 4000) {
      // red 255, 0, 0
      // TO
      // purple 255, 0, 130
      red = 255;
      green = 0;
      blue = ((double) (smooth_rpm-3000)/1000)*130;
      brightness = 255;
    } else if (smooth_rpm < 5000) {
      // purple 255, 0, 130
      // TO
      // white 255, 255, 130
      red = 255;
      green = ((double) (smooth_rpm-4000)/1000)*255;
      blue = 130;
      brightness = 255;
    }
    //Serial.print(smooth_rpm);Serial.print(": ");Serial.print(red);Serial.print(", ");
    //Serial.print(green);Serial.print(", ");Serial.print(blue);Serial.print(" - ");Serial.println(brightness);
    for(int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(red, green, blue));
      strip.setBrightness(brightness);
    }
    strip.show();
  }
  
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.data[2] == PID_RPM) { // ICE RPM
      // Some codes use more than one byte to store the svalue. 
      // The real RPM is a conjugate of two bytes, [3] and [4].
      if(rpm != 9999) last_rpm = rpm;
      rpm = (canMsg.data[3]*256 + canMsg.data[4])/4; 
      rpm_read_at = millis();
      //Serial.println(rpm);
    }
  }

  // request an rpm update no more than once every ms_between_can_requests milliseconds
  if ((millis() - last_rpm_request_at) > ms_between_can_requests) {
    requestDataOBD(PID_RPM);
    // update counter
    last_rpm_request_at = millis();
    
    if(simulation) {
      if(rpm == 9999) { rpm = 1000; last_rpm = 1000; }
      last_rpm = rpm;
      if(rpm < 500) rpm += random(0, 400);
      else if(rpm > 4500) rpm -= random(0, 400);
      else rpm += random(-400, 400);
      rpm_read_at = millis();
    }
  }
}
