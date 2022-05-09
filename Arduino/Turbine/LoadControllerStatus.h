// LoadControllerStatus.ino
// Artemis Project Wind Turbine Prototype
// Steven Fordham, Tamara Roberson
// Copyright (c) 2022

namespace Artemis {
namespace LoadController {

enum class Status {
  OK,
  DEBUG,
  ERROR_TOO_LOW,
  ERROR_TOO_HIGH,
  ERROR_INVALID_CMD
};

} // namespace LoadController
} // namespace Artemis
