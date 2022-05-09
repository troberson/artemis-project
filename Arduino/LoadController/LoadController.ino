// LoadController.ino
// Artemis Project Wind Turbine Prototype
// Steven Fordham, Tamara Roberson
// Copyright (c) 2022

#include "LoadController.h"
#include "LoadControllerCommand.h"
#include "LoadControllerStatus.h"

#include <SoftwareSerial.h>

#define ENABLE_DEBUG 1

namespace Artemis {
namespace LoadController {

class StopButton {
  int _pin{13};

public:
  StopButton() { pinMode(_pin, INPUT_PULLUP); }

  bool isPressed() { return digitalRead(_pin); }
};

class Controller {
  SoftwareSerial _serial =
      SoftwareSerial(Settings::COMM_PIN_RX, Settings::COMM_PIN_TX);
  int _load_cur = 0;
  StopButton _stop_button = StopButton();

  // ********** Public Functions **********
public:
  Controller() {
    _serial.begin(Settings::COMM_BAUD);
    _serial.listen();
    for (int i = Settings::LOAD_PIN_START; i <= Settings::LOAD_PIN_END; i++) {
      pinMode(i, OUTPUT);
    }
  }

  void run(void) {
    if (_stop_button.isPressed()) {
      Serial.println("### STOP ###");
      disable_load();
      while (_stop_button.isPressed()) {
      }
      return;
    }

    Command input = get_input();
    Status status = handle_input(input);
    Serial.print("Status: ");
    Serial.println((int)status);
    _serial.write((int)status);
    delay(1000);
  }

  void test(void) {
    clear_pins();
    _load_cur = 0;
    disable_load();
    delay(5000);
    while (_load_cur < Settings::LOAD_PIN_END) {
      Serial.print("Current load: ");
      Serial.println(_load_cur);
      Status status = increase_load();
      Serial.print("Status: ");
      Serial.println((int)status);
      delay(5000);
    }
  }

  // ********** Private Functions **********
private:
  void clear_pins(void) {
    for (int i = Settings::LOAD_PIN_START; i <= Settings::LOAD_PIN_END; i++) {
      digitalWrite(i, LOW);
    }
  }

  Command get_input(void) {
    Command input = (Command)_serial.read();
    if ((int)input < 0 || (int)input >= (int)Command::__END__) {
      return Command::NONE;
    }

    debug_received(input);
    return input;
  }

  Status handle_input(Command input) {
    switch (input) {
    case Command::INCREASE:
      return increase_load();
    case Command::DECREASE:
      return decrease_load();
    case Command::DISABLE:
      return disable_load();
    case Command::MAXIMUM:
      return maximize_load();
    default:
      return Status::ERROR_INVALID_CMD;
    }
  }

  Status set_load(int new_pin) {
    Serial.print("set_load(");
    Serial.print(new_pin);
    Serial.println(")");

    if (new_pin < Settings::LOAD_PIN_START && new_pin != 0) {
      Serial.println("ERROR_TOO_LOW");
      return Status::ERROR_TOO_LOW;
    }

    if (new_pin > Settings::LOAD_PIN_END) {
      Serial.println("ERROR_TOO_HIGH");
      return Status::ERROR_TOO_HIGH;
    }

    if (new_pin > 0) {
      digitalWrite(new_pin, HIGH);
    }

    digitalWrite(_load_cur, LOW);
    _load_cur = new_pin;
    return Status::OK;
  }

  Status decrease_load() {
    int new_load = (_load_cur > Settings::LOAD_PIN_START)
                       ? _load_cur - 1
                       : Settings::LOAD_PIN_START;
    return set_load(new_load);
  }

  Status increase_load() {
    int new_load = (_load_cur > 0) ? _load_cur + 1 : Settings::LOAD_PIN_START;
    return set_load(new_load);
  }

  Status disable_load() { return set_load(0); }

  Status maximize_load() { return set_load(Settings::LOAD_PIN_END); }

  void debug_received(Command input) {
    if (!ENABLE_DEBUG) {
      return;
    }

    //    Serial.print((int)Status::DEBUG);
    Serial.print("Received: '");
    Serial.print((int)input);
    Serial.println("'");
  }
};

} // namespace LoadController
} // namespace Artemis

// ********** ARDUINO **********
auto g_load_controller = new Artemis::LoadController::Controller();

void setup(void) { Serial.begin(9600); }

void loop(void) {
  g_load_controller->run();
  // g_load_controller->test();
}
