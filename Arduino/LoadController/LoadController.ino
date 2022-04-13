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

class Controller {
  SoftwareSerial _serial =
      SoftwareSerial(Settings::COMM_PIN_RX, Settings::COMM_PIN_TX);

  int _load_cur = 0;

  // ********** Public Functions **********
public:
  Controller() {
    _serial.begin(Settings::COMM_BAUD);
    for (int i = Settings::LOAD_PIN_START; i <= Settings::LOAD_PIN_END; i++) {
      pinMode(i, OUTPUT);
    }
  }

  void run(void) {
    Command input = get_input();
    Status status = handle_input(input);
    _serial.print((int)status);
  }

  void test(void) {
    clear_pins();
    _load_cur = 0;
    disable_load();
    delay(5000);
    while (_load_cur < Settings::LOAD_PIN_END) {
      _serial.print("Current load: ");
      _serial.println(_load_cur);
      Status status = increase_load();
      _serial.print("Status: ");
      _serial.println((int)status);
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
    debug_received(input);
    return input;
  }

  Status handle_input(Command input) {
    switch (input) {
    case Command::INCREASE:
      return increase_load();
    case Command::DECREASE:
      return decrease_load();
    default:
      return Status::ERROR_INVALID_CMD;
    }
  }

  Status set_load(int new_pin) {
    _serial.print("set_load(");
    _serial.print(new_pin);
    _serial.println(")");

    if (new_pin < Settings::LOAD_PIN_START && new_pin != 0) {
      return Status::ERROR_TOO_LOW;
    }

    if (new_pin > Settings::LOAD_PIN_END) {
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

  void debug_received(Command input) {
    if (!ENABLE_DEBUG) {
      return;
    }

    _serial.print((int)Status::DEBUG);
    _serial.print("Received: '");
    _serial.print((int)input);
    _serial.print("'");
  }
};

} // namespace LoadController
} // namespace Artemis

// ********** ARDUINO **********
auto g_load_controller = new Artemis::LoadController::Controller();

void setup(void) {}
void loop(void) {
  g_load_controller->run();
  // g_load_controller->test();
}
