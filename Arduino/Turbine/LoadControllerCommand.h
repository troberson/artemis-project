// LoadControllerCommands.h
// Artemis Project Wind Turbine Prototype
// Steven Fordham, Tamara Roberson
// Copyright (c) 2022

#ifndef LoadControllerCommands_h
#define LoadControllerCommands_h

namespace Artemis {
namespace LoadController {

// INCREASE will increase the resistance by triggering the next
// higher output port.
//
// DECREASE will decrease the resistance by triggering the previous
// lower output port.
enum class Command {
  NONE,
  INCREASE,
  DECREASE,
  DISABLE,
  MAXIMUM,
};

} // namespace LoadController
} // namespace Artemis

#endif // LoadControllerCommands_h
