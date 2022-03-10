#include <Adafruit_MAX31856.h>

#include <AltSoftSerial.h>

const struct {
  int h = 220; // the height of the Nextion graph
  float min_temp[4] = {24, 24, 24, 24}; // the minimum temperature shown on graph
  float max_temp[4] = {150, 300, 600, 100}; // the maximum temperature shown on the graph
  char *names[4] = { "Ambient", "Magnet", "Flue", "Unused" };
  float max_fan_rpm = 3000.; // the maximum fan speed shown on the graph
} graph_config;
      
const struct {
  // The minimum temperature at which fan will run and the PWM value
  float min_temp = 29.;
  int min_pwm = 50;

  // The minimum temperature at which fan will run at full speed
  float max_temp = 34.;
  int max_pwm = 255;

  float deadzone = .2; // hysteresis band: +/- deadzone
} fan_config;

#if INCLUDE_CS
const int one_wire_pin = 4;
#endif

const int pwm_pin = 5;
const int tach_pin = 3;

bool nextion_debug = false;

volatile int fan_pulse_count = 0;
int last_fan_rpm = 0;
unsigned long last_fan_start = 0;
int last_fan_pwm = 0;

enum {TEMP_C, TEMP_F} temp_mode = TEMP_C;
enum {FAN_MIN, FAN_AUTO, FAN_MAX} fan_mode = FAN_AUTO;
enum {MFAN_AUTO, MFAN_LOW, MFAN_MED, MFAN_HIGH} mainfan_mode = MFAN_AUTO;

const unsigned long fan_tach_sample_dt = 1000;
const int fan_ppr = 2; // two pulses per fan revolution
const int display_update_dt = 500;
const int fan_mode_update_dt = 2000;
const int graph_update_dt = 5000;

unsigned long last_display_read = 0;
unsigned long last_display_update = 0;
unsigned long last_graph_update = 0;
unsigned long last_fan_mode_change = 0;
unsigned long last_temp_mode_change = 0;
char nex_buffer[128]; // temp variable holding Nextion command string

AltSoftSerial altSerial; // Pins tx:9 rc:8, Nextion: yellow->9, blue->8
#if INCLUDE_CS
OneWire oneWire(one_wire_pin);
DallasTemperature sensors(&oneWire);
#endif

void isr_fan_pulse();
void find_min_max();
void send_to_nextion(char ss[]);

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 maxthermo[4] = {Adafruit_MAX31856(16, 11, 12, 13),
                                  Adafruit_MAX31856(17, 11, 12, 13),
                                  Adafruit_MAX31856(18, 11, 12, 13),
                                  Adafruit_MAX31856(19, 11, 12, 13) };

// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10);

int ssr0_pin = 2;
int ssr1_pin = 4;
int ssr2_pin = 6;
int ssr3_pin = 7;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  for(int i=0; i<4; i++) {
    maxthermo[i].begin();
    maxthermo[i].setThermocoupleType(MAX31856_TCTYPE_K);
  }

  pinMode(ssr0_pin, OUTPUT);
  pinMode(ssr1_pin, OUTPUT);
  pinMode(ssr2_pin, OUTPUT);
  pinMode(ssr3_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  pinMode(tach_pin, INPUT_PULLUP);

  last_fan_start = millis();
  attachInterrupt(digitalPinToInterrupt(tach_pin), isr_fan_pulse, RISING);

  altSerial.begin(38400);

//  Serial.print("Thermocouple type: ");
//  switch (maxthermo.getThermocoupleType() ) {
//    case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
//    case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
//    case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
//    case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
//    case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
//    case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
//    case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
//    case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
//    case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
//    case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
//    default: Serial.println("Unknown"); break;
//  }

  if (nextion_debug) {
    // Nextion default is to only send a response to a command if there's an error
    send_to_nextion("bkcmd=3");
  }

  send_to_nextion("page 0");
}

void isr_fan_pulse () {
  // Count each fan tach pulse in this interrupt handler
  fan_pulse_count++;
}

int reset_fan_counts() {
  int rv;

  cli();
  rv = fan_pulse_count;
  fan_pulse_count = 0;
  last_fan_start = millis();
  sei();

  return rv;
}

int fan_speed(bool force_update = false) {
  unsigned long cur = millis();

  unsigned long dt = cur - last_fan_start;

  if (force_update && dt < fan_tach_sample_dt) {
    delay(fan_tach_sample_dt - dt + 1);
    cur = millis();
    dt = cur - last_fan_start;
  }

  if (dt > fan_tach_sample_dt) {
    int cnt = reset_fan_counts();

    // Avoid overflow with careful order of operations
    last_fan_rpm = cnt * (60UL * 1000) / (dt * fan_ppr);
  }

  return last_fan_rpm;
}

void set_fan_speed(int fan_pwm) {
  analogWrite(pwm_pin, fan_pwm);
  last_fan_pwm = fan_pwm;
}


//void find_min_max() {
//  for (int fan_pwm = 0; fan_pwm < 256; fan_pwm += 5) {
//    set_fan_speed(fan_pwm);
//
//    delay(500); // Give the fan time to stabilize
//
//    reset_fan_counts();
//
//    int rpm = fan_speed(true); // Block until RPM is read
//
//    Serial.print(fan_pwm);
//    Serial.print(" ");
//    Serial.println(rpm);
//  }
//}

int target_fan_pwm(float temp) {
  int rv = 0;

  switch (fan_mode) {
    case FAN_MIN: rv = fan_config.min_pwm; break;
    case FAN_MAX: rv = fan_config.max_pwm; break;
    case FAN_AUTO:
    default:
      if (temp < fan_config.min_temp) {
        rv = 0;
      } else {
        int tmp = (temp - fan_config.min_temp) / (fan_config.max_temp - fan_config.min_temp) * fan_config.max_pwm;
        rv = min(max(tmp, fan_config.min_pwm), fan_config.max_pwm);
      }
      if (rv != last_fan_pwm) {
        if (fabs(temp - fan_config.min_temp) < fan_config.deadzone || fabs(temp - fan_config.max_temp) < fan_config.deadzone) {
          rv = last_fan_pwm;
        }
      }
      break;
  }

  last_fan_pwm = rv;

  return rv;
}

void loop() {
  int fan_rpm = fan_speed();
  if (fan_mode == FAN_MAX) {
    set_fan_speed(255);
  } else {
    set_fan_speed(0);
  }
  
  switch (mainfan_mode) {
    case MFAN_LOW:
      digitalWrite(ssr1_pin, LOW);
      digitalWrite(ssr2_pin, LOW);
      digitalWrite(ssr0_pin, HIGH);
      break;
    case MFAN_MED:
      digitalWrite(ssr0_pin, LOW);
      digitalWrite(ssr2_pin, LOW);
      digitalWrite(ssr1_pin, HIGH);
      break;
    case MFAN_HIGH:
      digitalWrite(ssr0_pin, LOW);
      digitalWrite(ssr1_pin, LOW);
      digitalWrite(ssr2_pin, HIGH);
      break;    
    case MFAN_AUTO:
    default:
      digitalWrite(ssr0_pin, LOW);
      digitalWrite(ssr1_pin, LOW);
      digitalWrite(ssr2_pin, HIGH);
      break;
  }
     
  float res[4*2];

  for(int i=0; i<4; i++) {
    res[4*0 + i] = maxthermo[i].readCJTemperature();
    res[4*1 + i] = maxthermo[i].readThermocoupleTemperature();
  
    // Check and print any faults
    uint8_t fault = maxthermo[i].readFault();
    if (fault) {
      Serial.print("Fault on sensor ");
      Serial.print(i);
      Serial.print(": ");
      if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
      if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
      if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
      if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
      if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
      if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
      if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
      if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
    }
  }

  for(int i=0; i<8; i++) {
    if (i > 0) {
      Serial.print(",");
    }
    if (i < 4) {
      Serial.print(res[i] - 10);
    } else {
      Serial.print(res[i]);
    }
  }
  Serial.print(",");
  Serial.print(fan_rpm);
  Serial.println("");

  float temps[4] = {res[4], res[5], res[6], res[7]};
  update_display(temps, fan_rpm);

  delay(100);
}

bool nextion_check_return() {
  bool rv = true;

  delay(10); // small delay so that the last command can complete. What is the
  // longest that a command might take to complete?

  // TODO: Needed improvements to make this code more robust:
  //   1) look for the trailing flag 0xff 0xff 0xff;
  //   2) timeout if the trailing flag is not received within a timeout period;
  //   3) reset the response buffer index (i) after parsing a command.

  const int n_buf = 8;
  unsigned char response[n_buf];
  if (altSerial.available()) {
    int i;
    for (i = 0; i < n_buf && altSerial.available(); i++) {
      unsigned char c = altSerial.read();
      response[i] = c;
      if (nextion_debug) {
        Serial.print("0x");
        Serial.print((int)c, HEX);
        Serial.print(" ");
      }
    }
    if (nextion_debug) {
      Serial.print("\n");
    }

    if (i == 6 && response[0] == 0x00 && response[1] == 0x00 && response[2] == 0x00) {
      //  Nextion Startup
    } else if (i > 2 && response[0] == 0x24) { // Serial Buffer Overflow
    } else if (i > 2 && response[0] == 0x65) { // Touch Event
      unsigned char page_num = response[1];
      unsigned char comp_id = response[2];
      unsigned char event = response[3];

      if (page_num == 0x00 && event == 0x00) {
        if (comp_id == 0x09) {
          temp_mode = (temp_mode + 1) % 2;
          if (nextion_debug) {
            Serial.println("Change temp_mode");
          }
          last_temp_mode_change = millis();
        } else if (comp_id == 0x0a) {
          fan_mode = (fan_mode + 1) % 3;
          if (nextion_debug) {
            Serial.println("Change fan_mode");
          }
          last_fan_mode_change = millis();
        } else if (comp_id == 0x0f) {
          mainfan_mode = (mainfan_mode + 1) % 4;
          if (nextion_debug) {
            Serial.println("Change mainfan_mode");
          }
//          last_fan_mode_change = millis();
        }
      }
    } else if (i > 2 && response[0] == 0x66) { // Current Page Number
    } else if (i > 2 && response[0] == 0x67) { // Touch Coordinate (awake)
    } else if (i > 2 && response[0] == 0x68) { // Touch Coordinate (sleep)
    } else if (i > 2 && response[0] == 0x70) { // String Data Enclosed
    } else if (i > 2 && response[0] == 0x71) { // Numeric Data Enclosed
    } else if (i > 2 && response[0] == 0x86) { // Auto Entered Sleep Mode
    } else if (i > 2 && response[0] == 0x87) { // Auto Wake from Sleep
    } else if (i > 2 && response[0] == 0x88) { // Nextion Ready
    } else if (i > 2 && response[0] == 0x89) { // Start microSD Upgrade
    } else if (i > 2 && response[0] == 0xfd) { // Transparent Data Finished
    } else if (i > 2 && response[0] == 0xfe) { // Transparent Data Ready
    } else if (i > 2 && 0 <= response[0] && response[0] <= 0x23) {
      switch (response[0]) {
        case 0x01: // Instruction Successful
          rv = true;
          break;
        case 0x00: // Invalid Instruction
        case 0x02: // Invalid Component ID
        case 0x03: // Invalid Page ID
        case 0x04: // Invalid Picture ID
        case 0x05: // Invalid Font ID
        case 0x06: // Invalid File Operation
        case 0x09: // Invalid CRC
        case 0x11: // Invalid Baud rate Setting
        case 0x12: // Invalid Waveform ID or Channel #
        case 0x1a: // Invalid Variable name or attribute
        case 0x1b: // Invalid Variable Operation
        case 0x1c: // Assignment failed to assign
        case 0x1d: // EEPROM Operation failed
        case 0x1e: // Invalid Quantity of Parameters
        case 0x1f: // IO Operation failed
        case 0x20: // Escape Character Invalid
        case 0x23: // Variable name too long
          rv = false;
          break;
        default:
          Serial.print("Unknown Nextion error code: ");
          Serial.println(response[0], HEX);
          break;
      }
    } else {
      Serial.print("Unknown Nextion code: ");
      for (int j = 0; j < i; j++) {
        unsigned char c = response[j];
        Serial.print("0x");
        Serial.print((int)c, HEX);
        Serial.print(" ");
      }
      Serial.print("\n");
    }
  }

  return rv;
}

void send_to_nextion(char ss[]) {
  if (nextion_debug) {
    Serial.println(ss);
  }

  altSerial.print(ss);
  altSerial.write(0xff);
  altSerial.write(0xff);
  altSerial.write(0xff);

  if (! nextion_check_return()) {
    Serial.print("Potentially failed command: ");
    Serial.println(ss);
  }
}

//void debug_rtc() {
//  const int n = 6;
//  const char *vars[n] = {"rtc2", "rtc3", "rtc4", "rtc5", "sys2", "va1.val"};
//
//  for (int i = 0; i < n; i++) {
//    sprintf(nex_buffer, "covx %s,va4.txt,0,0\xff\xff\xffxstr 0,%d,150,40,0,65000,0,2,1,1,va4.txt", vars[i], i * 40);
//    send_to_nextion(nex_buffer);
//  }
//}

void update_display(float temps[4], int fan_rpm) {
  unsigned long cur = millis();
  bool updated = false;

  if (cur - last_display_update > display_update_dt) {
    // debug_rtc();

    last_display_update = cur;
    updated = true;

    for (int i=0; i<4; i++) {
      int ind[4] = {0, 3, 4, 5};
      
      char unit = 'C';
      float v = temps[i];

      if (temp_mode == TEMP_F) {
        v = temps[i] * (9. / 5.) + 32.;
        unit = 'F';
      }

      char fstr[6];
      dtostrf(v, 0, 1, fstr);
      sprintf(nex_buffer, "b%d.txt=\"%s %c\"", ind[i], fstr, unit);
      send_to_nextion(nex_buffer);
    }

    {
      if (cur - last_fan_mode_change < fan_mode_update_dt) {
        switch (fan_mode) {
          case FAN_MIN: sprintf(nex_buffer, "b1.txt=\"Min RPM\""); break;
          case FAN_MAX: sprintf(nex_buffer, "b1.txt=\"Max RPM\""); break;
          case FAN_AUTO:
          default: sprintf(nex_buffer, "b1.txt=\"Auto RPM\""); break;
        }
      } else {
        switch (fan_mode) {
          case FAN_MIN: sprintf(nex_buffer, "b1.txt=\"%d mRPM\"", fan_rpm); break;
          case FAN_MAX: sprintf(nex_buffer, "b1.txt=\"%d MRPM\"", fan_rpm); break;
          case FAN_AUTO:
          default: sprintf(nex_buffer, "b1.txt=\"%d RPM\"", fan_rpm); break;
        }
      }
      send_to_nextion(nex_buffer);
    }

    {
      switch (mainfan_mode) {
        case MFAN_LOW: sprintf(nex_buffer, "b6.txt=\"Low\""); break;
        case MFAN_MED: sprintf(nex_buffer, "b6.txt=\"Med\""); break;
        case MFAN_HIGH: sprintf(nex_buffer, "b6.txt=\"High\""); break;
        case MFAN_AUTO:
        default: sprintf(nex_buffer, "b6.txt=\"Auto\""); break;
      }
      send_to_nextion(nex_buffer);
    }
  }

  if (cur - last_graph_update > graph_update_dt) {
    last_graph_update = cur;
    updated = true;

    for (int i=0; i<4; i++) {
      int v = (int)((temps[i] - graph_config.min_temp[i]) / (graph_config.max_temp[i] - graph_config.min_temp[i]) * graph_config.h);
      v = min(max(0, v), graph_config.h);
      sprintf(nex_buffer, "add 1,%d,%d", i, v);
      send_to_nextion(nex_buffer);
    }

//    {
//      int v = (int)(fan_rpm / graph_config.max_fan_rpm * graph_config.h);
//      v = min(max(0, v), graph_config.h);
//      sprintf(nex_buffer, "add 1,1,%d", v);
//      send_to_nextion(nex_buffer);
//    }
  }

  if (! updated) {
    nextion_check_return();
  }
}
