// ============================================================
//  control_signal_factory.cpp
//
//  Provides the single definition of ControlSignalFactory::Instance().
//
//  Keeping Instance() in a .cpp file (not in the header) ensures there is
//  exactly one definition in the shared library. If Instance() were defined
//  inline in the header, both the shared library and each executable that
//  includes the header could end up with their own copy of the static local
//  `instance`, causing the shared library's registrations to be invisible
//  to the executable's factory call.
// ============================================================

#include "rv2_control_signal_transport/control_signal_factory.h"

namespace rv2_interfaces
{

ControlSignalFactory& ControlSignalFactory::Instance()
{
    static ControlSignalFactory instance;
    return instance;
}

} // namespace rv2_interfaces
