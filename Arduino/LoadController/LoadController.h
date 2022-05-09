// LoadController.ino
// Artemis Project Wind Turbine Prototype
// Steven Fordham, Tamara Roberson
// Copyright (c) 2022

namespace Artemis {
namespace LoadController {

class Settings {
public:
  // Communication
  static const int COMM_PIN_RX = 11;
  static const int COMM_PIN_TX = 12;
  static const int COMM_BAUD = 9600;
  static const int LOAD_PIN_START = 2;
  static const int LOAD_PIN_END = 9;
};

} // namespace LoadController
} // namespace Artemis
