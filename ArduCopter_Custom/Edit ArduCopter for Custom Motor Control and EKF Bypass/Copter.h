/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
/*
  This is the main Copter class
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>

// Common dependencies
#include <AP_Common/AP_Common.h>            // Common definitions and utility routines for the ArduPilot libraries
#include <AP_Common/Location.h>             // Library having the implementation of location class         
#include <AP_Param/AP_Param.h>              // A system for managing and storing variables that are of general interest to the system.
#include <StorageManager/StorageManager.h>  // library for Management for hal.storage to allow for backwards compatible mapping of storage offsets to available storage

// Application dependencies
#include <AP_Logger/AP_Logger.h>            // ArduPilot Mega Flash Memory Library
#include <AP_Math/AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library
#include <AP_AccelCal/AP_AccelCal.h>        // interface and maths for accelerometer calibration
#include <AP_InertialSensor/AP_InertialSensor.h>                // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS/AP_AHRS.h>                                    // AHRS (Attitude Heading Reference System) interface library for ArduPilot
#include <AP_Mission/AP_Mission.h>                              // Mission command library
#include <AP_Mission/AP_Mission_ChangeDetector.h>               // Mission command change detection library
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>        // Attitude control library
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h>   // 6DoF Attitude control library
#include <AC_AttitudeControl/AC_AttitudeControl_Heli.h>         // Attitude control library for traditional helicopter
#include <AC_AttitudeControl/AC_PosControl.h>                   // Position control library
#include <AC_AttitudeControl/AC_CommandModel.h>                 // Command model library
#include <AP_Motors/AP_Motors.h>            // AP Motors library
#include <Filter/Filter.h>                  // Filter library
#include <AP_Vehicle/AP_Vehicle.h>          // needed for AHRS build
#include <AP_InertialNav/AP_InertialNav.h>  // inertial navigation library
#include <AC_WPNav/AC_WPNav.h>              // ArduCopter waypoint navigation library
#include <AC_WPNav/AC_Loiter.h>             // ArduCopter Loiter Mode Library
#include <AC_WPNav/AC_Circle.h>             // circle navigation library
#include <AP_Declination/AP_Declination.h>  // ArduPilot Mega Declination Helper Library
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library
#include <AP_BattMonitor/AP_BattMonitor.h>  // Battery monitor library
#include <AP_LandingGear/AP_LandingGear.h>  // Landing Gear library
#include <AC_InputManager/AC_InputManager.h>        // Pilot input handling library
#include <AC_InputManager/AC_InputManager_Heli.h>   // Heli specific pilot input handling library
#include <AP_Arming/AP_Arming.h>            // ArduPilot motor arming library
#include <AP_SmartRTL/AP_SmartRTL.h>        // ArduPilot Smart Return To Launch Mode (SRTL) library
#include <AP_TempCalibration/AP_TempCalibration.h>  // temperature calibration library
#include <AC_AutoTune/AC_AutoTune_Multi.h>  // ArduCopter autotune library. support for autotune of multirotors.
#include <AC_AutoTune/AC_AutoTune_Heli.h>   // ArduCopter autotune library. support for autotune of helicopters.
#include <AP_Parachute/AP_Parachute.h>      // ArduPilot parachute release library
#include <AC_Sprayer/AC_Sprayer.h>          // Crop sprayer library
#include <AP_ADSB/AP_ADSB.h>                // ADS-B RF based collision avoidance module library
#include <AP_Proximity/AP_Proximity.h>      // ArduPilot proximity sensor library
#include <AC_PrecLand/AC_PrecLand_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Winch/AP_Winch_config.h>
#include <AP_SurfaceDistance/AP_SurfaceDistance.h>

// Configuration
#include "defines.h"
#include "config.h"

#if FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#else
 #define MOTOR_CLASS AP_MotorsMulticopter
#endif

#if MODE_AUTOROTATE_ENABLED
 #include <AC_Autorotation/AC_Autorotation.h> // Autorotation controllers
#endif

#include "RC_Channel_Copter.h"         // RC Channel Library

#include "GCS_MAVLink_Copter.h"
#include "GCS_Copter.h"
#include "AP_Rally.h"           // Rally point library
#include "AP_Arming_Copter.h"

#include <AP_ExternalControl/AP_ExternalControl_config.h>
#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_ExternalControl_Copter.h"
#endif