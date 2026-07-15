#include <Arduino.h>

// =====================================================================
// FAN CONTROLLER - ESP32-C3 SuperMini / Arduino Nano
//
// Wiring:
//   Blue wire   -> PIN_PWM       (speed control via analogWrite, INVERTED logic - see note)
//   Yellow wire -> PIN_DIR       (direction control)
//   Green wire  -> PIN_FEEDBACK  (tachometer pulse output, optional)
//   Button      -> PIN_BUTTON, other leg to GND (internal pull-up used)
//
// Behavior:
//   - Fan starts OFF (0% speed) at boot.
//   - Long press (>=3s) while OFF -> fan starts at 15% speed.
//   - Short press while RUNNING   -> steps to next of 5 speed levels
//                                    (15 -> 36 -> 58 -> 79 -> 100 -> wraps to 15).
//   - Long press (>=3s) while RUNNING -> fan turns OFF.
// =====================================================================

// ------------------- Pin configuration (per-platform) -------------------
#if defined(ARDUINO_ARCH_ESP32)
  // ESP32-C3 SuperMini wiring
  constexpr int PIN_PWM      = 7;    // Blue wire
  constexpr int PIN_DIR      = 9;    // Yellow wire
  constexpr int PIN_FEEDBACK = 10;   // Green wire (tach, optional use)
  constexpr int PIN_BUTTON   = 3;    // Button leg (other leg to GND)
  constexpr int PIN_LED      = 8;    // Built-in LED, used as speed-level indicator
  constexpr bool LED_ACTIVE_LOW = true;  // SuperMini's onboard LED lights when driven LOW

#elif defined(ARDUINO_ARCH_AVR)
  // Arduino Uno / Nano wiring (both use the ATmega328P, identical pinout) -
  // useful for testing true 5V logic levels
  constexpr int PIN_PWM      = 9;    // Blue wire   - Timer1 PWM-capable pin
  constexpr int PIN_DIR      = 8;    // Yellow wire
  constexpr int PIN_FEEDBACK = 2;    // Green wire  - MUST be pin 2 or 3 (INT0/INT1) for attachInterrupt
  constexpr int PIN_BUTTON   = 4;    // Button leg (other leg to GND)
  constexpr int PIN_LED      = LED_BUILTIN;  // pin 13
  constexpr bool LED_ACTIVE_LOW = false;     // Uno/Nano's built-in LED lights when driven HIGH

#else
  #error "Unsupported board - add PIN_PWM/PIN_DIR/PIN_FEEDBACK/PIN_BUTTON/PIN_LED for your platform"
#endif

// ------------------- Direction configuration -------------------
// Per the motor's own logic: yellow wire tied to GND = CW, floating/HIGH = CCW.
// Flip this constant if you ever need CCW instead. It's read once at startup;
// move the setDirection() call into handleButton()/loop() if you want to be
// able to change direction live while running.
constexpr bool SPIN_CLOCKWISE = true;

// ------------------- PWM configuration -------------------
constexpr int PWM_FREQ_HZ  = 25000;              // 25 kHz, standard for this class of fan PWM input
constexpr int PWM_RES_BITS = 8;                  // 8-bit duty resolution (0-255)
constexpr int PWM_MAX_DUTY = (1 << PWM_RES_BITS) - 1;

// Set this to true once you've added an external single N-MOSFET open-drain
// level-shifter (gate <- PIN_PWM via ~220R, source -> GND, drain -> fan's
// blue wire + pull-up resistor to 5V, plus a 10k gate pull-down to GND for a
// safe power-up default). That stage inverts the signal in hardware, so the
// firmware should then send the duty cycle DIRECTLY (uninverted) rather than
// inverting it in software - the two inversions need to cancel out, not stack.
constexpr bool PWM_LINE_HAS_HW_INVERTER = true;

// NOTE ON INVERTED PWM LOGIC:
// You mentioned the blue wire goes to FULL speed when tied straight to the
// negative rail (i.e. held constantly LOW / 0% high-time). That means this
// fan's PWM input works backwards relative to a typical PC fan header:
//     0% duty (constant LOW)   -> 100% speed
//     100% duty (constant HIGH)-> fan stopped
// Without a hardware inverter, we replicate that by sending
// dutyPercent = 100 - speedPercent directly to the fan.
// With a hardware inverter (PWM_LINE_HAS_HW_INVERTER = true), the MOSFET
// stage already flips the signal, so we send dutyPercent = speedPercent
// and let the hardware do the inversion for us.

// ------------------- Speed levels -------------------
// 5 discrete speeds, evenly spaced between 15% and 100%.
const int speedLevels[] = {15, 36, 58, 79, 100};
constexpr int NUM_SPEEDS = sizeof(speedLevels) / sizeof(speedLevels[0]);
int currentSpeedIndex = 0;  // only meaningful while RUNNING

// ------------------- Button timing -------------------
constexpr unsigned long DEBOUNCE_MS   = 30;
constexpr unsigned long LONG_PRESS_MS = 3000;

// ------------------- Fan state -------------------
enum class FanState { OFF, RUNNING };
FanState fanState = FanState::OFF;

// ------------------- Button state tracking -------------------
bool rawLastReading = HIGH;
bool buttonPressed = false;           // debounced logical state (true = pressed)
unsigned long lastDebounceTime = 0;
unsigned long pressStartTime = 0;
bool longPressHandled = false;        // guards against re-firing while held

// ------------------- Speed-level LED indicator -------------------
// Blinks the built-in LED (currentSpeedIndex + 1) times in quick succession,
// then stays off for the rest of a 10-second window before repeating.
// e.g. speed level 1 (15%)  -> 1 blink every 10s
//      speed level 5 (100%)-> 5 blinks in a row every 10s
constexpr unsigned long LED_CYCLE_MS   = 10000;  // full repeat period
constexpr unsigned long LED_ON_MS      = 150;    // blink on-time
constexpr unsigned long LED_GAP_MS     = 150;    // gap between blinks in a burst

bool ledCycleNeedsReset = true;  // set true whenever fan turns on/off or speed changes

void setLed(bool on) {
  digitalWrite(PIN_LED, (on != LED_ACTIVE_LOW) ? HIGH : LOW);
  // Equivalent to: on ? (LED_ACTIVE_LOW ? LOW : HIGH) : (LED_ACTIVE_LOW ? HIGH : LOW)
}

void updateSpeedIndicatorLed() {
  static unsigned long cycleStart = 0;
  static unsigned long lastToggle = 0;
  static int blinksDone = 0;
  static bool ledOn = false;

  if (fanState != FanState::RUNNING) {
    setLed(false);
    ledCycleNeedsReset = true;  // ensure a clean restart once fan turns back on
    return;
  }

  unsigned long now = millis();

  if (ledCycleNeedsReset) {
    cycleStart = now;
    lastToggle = now;
    blinksDone = 0;
    ledOn = false;
    setLed(false);
    ledCycleNeedsReset = false;
  }

  int requiredBlinks = currentSpeedIndex + 1;  // 1..5

  if (blinksDone < requiredBlinks) {
    if (!ledOn && (now - lastToggle >= LED_GAP_MS)) {
      setLed(true);
      ledOn = true;
      lastToggle = now;
    } else if (ledOn && (now - lastToggle >= LED_ON_MS)) {
      setLed(false);
      ledOn = false;
      blinksDone++;
      lastToggle = now;
    }
  } else {
    // Burst finished; wait out the rest of the 10s window.
    if (now - cycleStart >= LED_CYCLE_MS) {
      cycleStart = now;
      blinksDone = 0;
    }
  }
}

// ------------------- Tachometer feedback (optional) -------------------
#if !defined(ARDUINO_ARCH_ESP32)
  #define IRAM_ATTR   // no-op on AVR; IRAM_ATTR is an ESP32-specific attribute
#endif

volatile unsigned long pulseCount = 0;
void IRAM_ATTR onFeedbackPulse() {
  pulseCount = pulseCount + 1;  // plain assignment avoids the C++20 "volatile ++" deprecation warning
}

// ------------------- Helper functions -------------------
void setDirection(bool cw) {
  // yellow wire: LOW = CW, HIGH = CCW
  digitalWrite(PIN_DIR, cw ? LOW : HIGH);
}

void setSpeedPercent(int speedPercent) {
  speedPercent = constrain(speedPercent, 0, 100);
  int dutyPercent = PWM_LINE_HAS_HW_INVERTER ? speedPercent : (100 - speedPercent);
  int duty = map(dutyPercent, 0, 100, 0, PWM_MAX_DUTY);
  analogWrite(PIN_PWM, duty);
}

void stopFan() {
  fanState = FanState::OFF;
  setSpeedPercent(0);   // drives the line fully HIGH -> motor stopped
  ledCycleNeedsReset = true;
}

void startFan() {
  fanState = FanState::RUNNING;
  currentSpeedIndex = 0;   // always resume at the lowest speed, 15%
  setDirection(SPIN_CLOCKWISE);
  setSpeedPercent(speedLevels[currentSpeedIndex]);
  ledCycleNeedsReset = true;
}

void nextSpeedStep() {
  currentSpeedIndex++;
  if (currentSpeedIndex >= NUM_SPEEDS) {
    currentSpeedIndex = 0;   // wrap from 100% back to 15%
  }
  setSpeedPercent(speedLevels[currentSpeedIndex]);
  ledCycleNeedsReset = true;  // reflect new speed level immediately
}

// ------------------- Button handling (non-blocking) -------------------
void handleButton() {
  bool rawReading = digitalRead(PIN_BUTTON);  // LOW = pressed (pull-up)

  if (rawReading != rawLastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    bool newButtonPressed = (rawReading == LOW);

    if (newButtonPressed != buttonPressed) {
      buttonPressed = newButtonPressed;

      if (buttonPressed) {
        // Just pressed down
        pressStartTime = millis();
        longPressHandled = false;
      } else {
        // Just released
        unsigned long heldTime = millis() - pressStartTime;
        if (!longPressHandled && heldTime < LONG_PRESS_MS) {
          // This was a short press
          if (fanState == FanState::RUNNING) {
            nextSpeedStep();
          }
          // Short press while OFF does nothing, per spec.
        }
      }
    }
  }

  // Check for long press while the button is still held down
  if (buttonPressed && !longPressHandled) {
    if ((millis() - pressStartTime) >= LONG_PRESS_MS) {
      longPressHandled = true;
      if (fanState == FanState::OFF) {
        startFan();
      } else {
        stopFan();
      }
    }
  }

  rawLastReading = rawReading;
}

void setup() {
  Serial.begin(115200);

  // Drive PWM pin to whichever level means "stopped" as early as possible,
  // to minimize any glitch/blip before analogWrite() takes over the pin.
  // Without a hardware inverter: HIGH = stopped. With one: LOW = stopped
  // (the MOSFET stage flips it to a clean 5V HIGH at the fan).
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_PWM, PWM_LINE_HAS_HW_INVERTER ? LOW : HIGH);

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_FEEDBACK, INPUT_PULLUP);

  pinMode(PIN_LED, OUTPUT);
  setLed(false);

#if defined(ARDUINO_ARCH_ESP32)
  // Sets PWM frequency/resolution for the PWM pin on ESP32.
  analogWriteFrequency(PIN_PWM, PWM_FREQ_HZ);
  analogWriteResolution(PIN_PWM, PWM_RES_BITS);
#endif
  // On AVR (Nano), analogWrite() has a fixed ~490Hz frequency and fixed 8-bit
  // resolution that can't be changed without directly touching timer
  // registers. That's lower than the fan's ideal ~25kHz, but is fine for
  // confirming logic-level behavior; most fan PWM inputs still decode duty
  // cycle correctly at this frequency, just possibly with more audible whine.

  attachInterrupt(digitalPinToInterrupt(PIN_FEEDBACK), onFeedbackPulse, FALLING);

  stopFan();  // ensure a clean, known OFF state
  rawLastReading = digitalRead(PIN_BUTTON);
}

void loop() {
  handleButton();
  updateSpeedIndicatorLed();

  // Optional: print state + rough RPM once a second, useful for debugging.
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // Many fans emit 2 tach pulses per revolution; adjust the divisor below
    // if your fan's datasheet specifies a different pulses-per-rev count.
    float rpm = (pulses / 2.0f) * 60.0f;

    int displaySpeed = (fanState == FanState::RUNNING) ? speedLevels[currentSpeedIndex] : 0;
#if defined(ARDUINO_ARCH_ESP32)
    // ESP32 core's Serial supports printf directly.
    Serial.printf("State: %-7s | Speed: %3d%% | Feedback: %.0f RPM\n",
                  fanState == FanState::RUNNING ? "RUNNING" : "OFF",
                  displaySpeed, rpm);
#else
    // AVR core's Serial has no printf() - build the line with plain print/println.
    Serial.print("State: ");
    Serial.print(fanState == FanState::RUNNING ? "RUNNING" : "OFF");
    Serial.print(" | Speed: ");
    Serial.print(displaySpeed);
    Serial.print("% | Feedback: ");
    Serial.print(rpm, 0);
    Serial.println(" RPM");
#endif
  }
}