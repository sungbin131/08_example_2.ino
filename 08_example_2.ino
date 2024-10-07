// Arduino pin assignment
#define PIN_LED  10
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() { 
  float distance;

  // wait until next sampling time (polling)
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // Control LED brightness based on distance
  if ((distance == 0.0) || (distance > _DIST_MAX)) {
      distance = _DIST_MAX + 10.0;    // Set Higher Value
      analogWrite(PIN_LED, 255);      // LED OFF (max value)
  } else if (distance < _DIST_MIN) {
      distance = _DIST_MIN - 10.0;    // Set Lower Value
      analogWrite(PIN_LED, 255);      // LED OFF (min distance)
  } else {
      int brightness = mapDistanceToBrightness(distance);
      analogWrite(PIN_LED, brightness);  // Control LED brightness
  }

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.println("");
  
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}

// Map distance to brightness (0 = brightest, 255 = dimmest)
int mapDistanceToBrightness(float distance) {
    if (distance <= 100.0) {
        return 255;  // Minimum brightness (LED off)
    } else if (distance >= 300.0) {
        return 255;  // Maximum brightness (LED off)
    } else if (distance == 200.0) {
        return 0;    // Maximum LED brightness
    } else if (distance == 150.0 || distance == 250.0) {
        return 128;  // 50% brightness
    } else {
        // For linear interpolation between 100mm and 300mm
        float brightness = (255.0 / 200.0) * (distance - 100.0);
        return (int)brightness;
    }
}
