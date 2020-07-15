// Copyright 2018 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef SERIAL_DRIVER__VISIBILITY_CONTROL_HPP_
#define SERIAL_DRIVER__VISIBILITY_CONTROL_HPP_


////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(SERIAL_DRIVER_BUILDING_DLL) || defined(SERIAL_DRIVER_EXPORTS)
    #define SERIAL_DRIVER_PUBLIC __declspec(dllexport)
    #define SERIAL_DRIVER_LOCAL
  #else  // defined(SERIAL_DRIVER_BUILDING_DLL) || defined(SERIAL_DRIVER_EXPORTS)
    #define SERIAL_DRIVER_PUBLIC __declspec(dllimport)
    #define SERIAL_DRIVER_LOCAL
  #endif  // defined(SERIAL_DRIVER_BUILDING_DLL) || defined(SERIAL_DRIVER_EXPORTS)
#elif defined(__linux__)
  #define SERIAL_DRIVER_PUBLIC __attribute__((visibility("default")))
  #define SERIAL_DRIVER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define SERIAL_DRIVER_PUBLIC __attribute__((visibility("default")))
  #define SERIAL_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // SERIAL_DRIVER__VISIBILITY_CONTROL_HPP_
