# Carduino (CarPM?)
A relatively simple Arduino project to connect interior lights in my car to a live-read of the engine's RPM. Since I have a manual transmission, it's fun to see the lights go WOO when I accelerate or change gears.

Note: In addition to lots of online searching, I drew heavily from [this helpful guide](https://www.instructables.com/TachometerScan-Gauge-Using-Arduino-OBD2-and-CAN-Bu/) as well as [this one](https://www.electronicshub.org/arduino-mcp2515-can-bus-tutorial/).

### Instructions
You'll need:
- An Arduino board (I'm using an Arduino Uno)
- A CAN BUS Shield of some kind (Arduino-OBD2 interface)
- An OBD2 plug
- Some LEDs

### Getting car signals to the Arduino
This is the trickiest part for sure, but I had good luck with it! You'll need a CAN BUS Sheild,
[like this one](https://www.amazon.com/Comidox-MCP2515-Receiver-Controller-Development/dp/B07J9KZ4L4/ref=asc_df_B07J9KZ4L4?tag=bngsmtphsnus-20&linkCode=df0&hvadid=80539344142696&hvnetw=&hvqmt=e&hvbmt=be&hvdev=c&hvlocint=&hvlocphy=&hvtargid=pla-4584138871897991&psc=1), and you just solder/wire up the CAN HIGH and CAN LOW from the Shield directly to the pins on the OBD2 port, according to the pin diagram below (apparently this is consistent between cars):

-- image here --

Then wire the CAN BUS Shield to your Arduino like so (ignore the fact that this image has two Arduinos + shields - you'll only need one of each):

-- image here --

Now, using the code from [this helpful guide](https://www.instructables.com/TachometerScan-Gauge-Using-Arduino-OBD2-and-CAN-Bu/), you can check if your Arduino is able to pull the RPM from the engine! First get the [mcp2515](https://github.com/autowp/arduino-mcp2515) library installed (easiest to just download the folder via Github then put it into the Libraries folder in your Arduino folder). Then something as simple as this should work:

```
#include <SPI.h>
#include <mcp2515.h>

// misc placeholders
#define PID_RPM 0x0C
#define CAN_2515

// CAN frame structures
struct can_frame canMsg; //inbound
struct can_frame canMsgOutgoing;

// vars
long rpm = 0;
unsigned long last_rpm_request_at = 0;
long ms_between_can_requests = 200; // how long to wait between CAN requests

MCP2515 mcp2515(spiCSPin);

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

  // init CAN board
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS); // Your vehicle may use a different speed!
  mcp2515.setNormalMode();

  // initial request
  requestDataOBD(PID_RPM);
}

void loop() {
  // Check for a received RPM message via CAN
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.data[2] == PID_RPM) {
      // Some codes use more than one byte to store the svalue. 
      // The real RPM is a conjugate of two bytes, [3] and [4].
      rpm = (canMsg.data[3]*256 + canMsg.data[4])/4; 
      Serial.println(rpm);
    }
  }

  // request an rpm update no more than once every ms_between_can_requests milliseconds
  if ((millis() - last_rpm_request_at) > ms_between_can_requests) {
    requestDataOBD(PID_RPM);
    // update timestamp
    last_rpm_request_at = millis();
  }
}
```

Upload it into the Arduino, open your Serial Monitor (or Serial Plotter) and see if you can read that RPM! If it's not working, keep in mind that some cars use different CAN messages / codes. There's a list of them on Wikipedia [here](https://en.wikipedia.org/wiki/OBD-II_PIDs#Service_01) (though thankfully my Mazda 3 used the most common settings).

Once I got that working, it was just a question of hooking up some LEDs and getting the animations right! I found that a smoothing function was important on the RPM, as I was only sampling the RPM a few times per second (I tried 3, 5, and 10 to see what looked good, ended up using 5). If I just changed the lights based on the most recent RPM, the lights would look super jumpy. 

So instead, I always stored the most-recent RPM in `rpm` as well as the RPM value before that in `last_rpm`. I also tracked a timestamp `rpm_read_at`. That way, on each `loop()`, I could calculate how many milliseconds it had been since we got a new RPM signal. I'd then be able to "ease" from `last_rpm` to `rpm` using a custom function. The final value was stored in `smooth_rpm`, which I then used to control the color and brightness of the LEDs. Finally, a variable `ms_easing_duration` controlled how quickly `smooth_rpm` should catch up with `rpm`. The code ended up like this:

```java
  // inside loop():
  unsigned long diff = millis() - rpm_read_at;
  if(diff >= ms_easing_duration) // past easing function, just "catch up" immediately
    smooth_rpm = rpm;
  else // inside easing function
    smooth_rpm = last_rpm + ((rpm-last_rpm) * easeInOutCubic((double) diff/ms_easing_duration));

  // a simple cubic easing function, maps from 0-1 to 0-1 on nice cubic curves
  double easeInOutCubic(double t) {
    return t<.5 ? 4*t*t*t : (t-1)*(2*t-2)*(2*t-2)+1;
  }
```

Then I just converted `smooth_rpm` into the color values I wanted using [FastLED](http://fastled.io)'s HSV functions, and set the entire LED array to those colors.

And if you print your variables in Arduino nicely, they show up in the Serial Plotter graph really well!
```
  Serial.print("rpm: "); Serial.print(rpm);
  Serial.print(", smooth_rpm: "); Serial.print(smooth_rpm);
  Serial.print(", hue: "); Serial.print(hue);
  Serial.print(", brightness: "); Serial.println(brightness);
```
Produces fun live-updating graphs like this!

-- image here --


My final steps included splitting a USB cable in half so the LEDs could have their own power lines coming in (I found that running 120 LEDs worth of current through the Arduino caused it to stop responding to my computer) and zip-tie-ing the whole rig up under the car dash! (Perhaps a more professional job would be to power the whole thing from the OBD port, but that requires stepping down the power from 12V to 5V, and also my car's OBD port is *always* hooked up to the battery, so I'd need a separate switch for my lights in order to prevent them from killing the battery overnight. Instead, I just wired a long USB cable to my cigarette port USB adapter - this has the advantage of killing power when the ignition is turned off!).

-- final video --
