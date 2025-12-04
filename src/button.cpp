#include "button.h"

ButtonHandler::ButtonHandler() {
  // initialize state machine
  current_state = ButtonState::IDLE;

  // initialize timing variables
  press_start_time = 0;
  release_time = 0;
  
  // initialize debouncing variables
  last_raw_state = false;       // false = not pressed
  last_debounce_time = 0;
  debounced_state = false;      // start assuming not pressed
  previous_debounced = false;   // for edge detection

  // initialize current settings to first preset
  // these will be overwritten by loadSettings() if saved values exist
  current_color_index = 0;
  current_brightness_index = 0;
  current_foam_index = 0;

  // define brightness presets (7 levels)
  brightness_presets[0] = 3;
  brightness_presets[1] = 4;
  brightness_presets[2] = 5;
  brightness_presets[3] = 6;
  brightness_presets[4] = 9;
  brightness_presets[5] = 11;
  brightness_presets[6] = 13;

  // define foam threshold presets (5 levels)
  foam_presets[0] = 30;// no foam
  foam_presets[1] = 5;
  foam_presets[2] = 4;
  foam_presets[3] = 3;
  foam_presets[4] = 2;
}

void ButtonHandler::init() {
  // configure button pin with internal pull-up
  // we read LOW when pressed (active low)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // load saved settings from nvs
  loadSettings();

  Serial.println("Button handler initialized on GPIO 5");
  Serial.print("Loaded settings - Color: ");
  Serial.print(current_color_index);
  Serial.print(", Brightness: ");
  Serial.print(getCurrentBrightness());
  Serial.print(", Foam: ");
  Serial.println(getCurrentFoamThreshold());
}

void ButtonHandler::loadSettings() {
  // open nvs in read-only mode
  preferences.begin(NVS_NAMESPACE, true);

  // load each setting with default fallback if key doesn't exist
  current_color_index = preferences.getInt(NVS_KEY_COLOR, 0);
  current_brightness_index = preferences.getInt(NVS_KEY_BRIGHTNESS, 0);
  current_foam_index = preferences.getInt(NVS_KEY_FOAM, 2);  // default to middle foam level

  // validate loaded values are within range
  if (current_color_index < 0 || current_color_index >= (int)ColorPreset::NUM_PRESETS) {
    current_color_index = 0;
  }
  if (current_brightness_index < 0 || current_brightness_index >= 7) {
    current_brightness_index = 0;
  }
  if (current_foam_index < 0 || current_foam_index >= 5) {
    current_foam_index = 2;
  }

  preferences.end();

  Serial.println("Settings loaded from NVS");
}

void ButtonHandler::saveSettings() {
  // open nvs in read-write mode
  preferences.begin(NVS_NAMESPACE, false);

  // save current settings
  preferences.putInt(NVS_KEY_COLOR, current_color_index);
  preferences.putInt(NVS_KEY_BRIGHTNESS, current_brightness_index);
  preferences.putInt(NVS_KEY_FOAM, current_foam_index);

  preferences.end();

  Serial.println("Settings saved to NVS");
}

bool ButtonHandler::getStableState() {
  // read raw state from pin
  // LOW = pressed (because INPUT_PULLUP, active LOW)
  bool current_raw = (digitalRead(BUTTON_PIN) == LOW);
  
  // check if raw state changed from last reading
  if (current_raw != last_raw_state) {
    // state changed - could be real press/release or noise
    // reset debounce timer to wait for signal to stabilize
    last_debounce_time = millis();
  }
  
  // save raw state for next comparison
  last_raw_state = current_raw;
  
  // check if signal has been stable long enough to trust
  if ((millis() - last_debounce_time) > DEBOUNCE_TIME) {
    // signal stable for DEBOUNCE_TIME ms, update debounced state
    debounced_state = current_raw;
  }
  
  // always return the last known stable state
  // this avoids returning "false" during settling which would
  // cause the state machine to think button was released
  return debounced_state;
}

void ButtonHandler::update() {
  // main state machine - call every loop iteration
  
  // get current stable button state
  bool current = getStableState();
  
  // edge detection - compare current stable state to previous stable state
  // just_pressed = button was not pressed, now is pressed (rising edge)
  // just_released = button was pressed, now is not pressed (falling edge)
  bool just_pressed = current && !previous_debounced;
  bool just_released = !current && previous_debounced;
  
  // save current state for next iteration's edge detection
  previous_debounced = current;

  switch (current_state) {
    case ButtonState::IDLE:
      // waiting for button press
      if (just_pressed) {
        // button just pressed, start tracking press duration
        current_state = ButtonState::PRESSED;
        press_start_time = millis();
        Serial.println("Button pressed, tracking duration...");
      }
      break;

    case ButtonState::PRESSED:
      // button is held down, determine if short or long press
      if (just_released) {
        // released before long press threshold = short press
        // now wait to see if it's a single tap or start of double tap
        current_state = ButtonState::RELEASED_ONCE;
        release_time = millis();
        Serial.println("Short press detected, waiting for possible double-tap...");
      } else if (current && (millis() - press_start_time >= LONG_PRESS_THRESHOLD)) {
        // still held and exceeded long press threshold
        current_state = ButtonState::LONG_PRESS_ACTIVE;
        cycleFoam();
        Serial.println("Long press triggered!");
      }
      break;

    case ButtonState::RELEASED_ONCE:
      // button was pressed briefly and released
      // waiting to see if user taps again (double-tap) or not (single tap)
      if (just_pressed) {
        // second press within window = double tap
        cycleColor();
        // go to wait_release so we don't immediately re-trigger on next loop
        current_state = ButtonState::WAIT_RELEASE;
        Serial.println("Double-tap detected!");
      } else if (millis() - release_time > DOUBLE_TAP_WINDOW) {
        // timeout expired without second tap = single tap
        cycleBrightness();
        current_state = ButtonState::IDLE;
        Serial.println("Single tap detected (timeout)");
      }
      break;

    case ButtonState::LONG_PRESS_ACTIVE:
      // long press action already taken, wait for user to release
      if (just_released) {
        current_state = ButtonState::IDLE;
        Serial.println("Long press released, returning to idle");
      }
      break;
      
    case ButtonState::WAIT_RELEASE:
      // action was taken (e.g. after double-tap), wait for finger to lift
      // this prevents immediately detecting another press on the next loop
      if (just_released) {
        current_state = ButtonState::IDLE;
        Serial.println("Button released, returning to idle");
      }
      break;
  }
}

void ButtonHandler::cycleColor() {
  // advance to next color preset with wraparound
  current_color_index = (current_color_index + 1) % (int)ColorPreset::NUM_PRESETS;

  Serial.print("Color changed to: ");
  Serial.println(current_color_index);

  // save to nvs immediately after change
  saveSettings();
}

void ButtonHandler::cycleBrightness() {
  // advance to next brightness preset with wraparound (7 levels)
  current_brightness_index = (current_brightness_index + 1) % 7;

  int new_brightness = brightness_presets[current_brightness_index];

  Serial.print("Brightness changed to: ");
  Serial.println(new_brightness);

  // save to nvs immediately after change
  saveSettings();
}

void ButtonHandler::cycleFoam() {
  // advance to next foam threshold preset with wraparound (5 levels)
  current_foam_index = (current_foam_index + 1) % 5;

  int new_foam = foam_presets[current_foam_index];

  Serial.print("Foam threshold changed to: ");
  Serial.println(new_foam);

  // save to nvs immediately after change
  saveSettings();
}

ColorPreset ButtonHandler::getCurrentColor() {
  return (ColorPreset)current_color_index;
}

int ButtonHandler::getCurrentBrightness() {
  return brightness_presets[current_brightness_index];
}

int ButtonHandler::getCurrentFoamThreshold() {
  return foam_presets[current_foam_index];
}

void ButtonHandler::getCurrentColorRGB(uint8_t* r, uint8_t* g, uint8_t* b) {
  // convert color preset enum to rgb values
  switch (getCurrentColor()) {
    case ColorPreset::BLUE:
      *r = 0;
      *g = 0;
      *b = 255;
      break;
    case ColorPreset::CYAN:
      *r = 0;
      *g = 255;
      *b = 255;
      break;
    case ColorPreset::GREEN:
      *r = 0;
      *g = 255;
      *b = 0;
      break;
    case ColorPreset::PINK:
      *r = 255;
      *g = 50;
      *b = 150;
      break;
    case ColorPreset::WHITE:
      *r = 255;
      *g = 255;
      *b = 255;
      break;
    default:
      // fallback to blue if somehow invalid
      *r = 0;
      *g = 0;
      *b = 255;
      break;
  }
}