// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER__VISIBILITY_CONTROL_H_
#define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_EXPORT __attribute__ ((dllexport))
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_EXPORT __declspec(dllexport)
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_BUILDING_DLL
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_PUBLIC ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_EXPORT
  #else
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_PUBLIC ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_IMPORT
  #endif
  #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_PUBLIC_TYPE ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_PUBLIC
  #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_LOCAL
#else
  #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_PUBLIC
    #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_LOCAL
  #endif
  #define ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER__VISIBILITY_CONTROL_H_
