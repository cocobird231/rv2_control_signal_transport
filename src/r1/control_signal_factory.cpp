/**
 * @file control_signal_factory.cpp
 * @brief Single definition of the r1::ControlSignalFactory singleton
 *        (design draft §7.2). Living in the shared library guarantees one
 *        registry per process, visible across all TUs (F5).
 */

#include "rv2_control_signal_transport/r1/control_signal_factory.h"

namespace rv2_interfaces
{
namespace r1
{

ControlSignalFactory& ControlSignalFactory::Instance()
{
    static ControlSignalFactory instance;
    return instance;
}

} // namespace r1
} // namespace rv2_interfaces
