#pragma once

 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/


#ifdef __cplusplus
extern "C" {
#endif

#include "psmove_config.h"

#include <stdint.h>

#ifdef _WIN32
#  define ADDCALL __cdecl
#  if defined(BUILDING_SHARED_LIBRARY)
#    define ADDAPI __declspec(dllexport)
#  else /* using shared library */
#    define ADDAPI __declspec(dllimport)
#  endif
#else
#  define ADDAPI
#  define ADDCALL
#endif

/*! Hardware model type for controller.
 **/
enum PSMove_Model_Type {
    Model_Unknown = 0,
    Model_ZCM1,
    Model_ZCM2,
    Model_Count
};

/*! Connection type for controllers.
 * Controllers can be connected via USB or via Bluetooth. The USB connection is
 * required when you want to pair the controller and for saving the calibration
 * blob. The Bluetooth connection is required when you want to read buttons and
 * calibration values. LED and rumble settings can be done with both Bluetooth
 * and USB.
 *
 * Used by psmove_connection_type().
 **/
enum PSMove_Connection_Type {
    Conn_Bluetooth, /*!< The controller is connected via Bluetooth */
    Conn_USB, /*!< The controller is connected via USB */
    Conn_Unknown, /*!< Unknown connection type / other error */
};

/*! Button flags.
 * The values of each button when pressed. The values returned
 * by the button-related functions return an integer. You can
 * use the logical-and operator (&) to check if a button is
 * pressed (in case of psmove_get_buttons()) or if a button
 * state has changed (in case of psmove_get_button_events()).
 *
 * Used by psmove_get_buttons() and psmove_get_button_events().
 **/
enum PSMove_Button {
    /**
     * See comment in psmove_get_buttons() for how this is
     * laid out in the input report.
     *
     * Source:
     * https://github.com/nitsch/moveonpc/wiki/Input-report
     **/
    Btn_TRIANGLE = 1 << 4, /*!< Green triangle */
    Btn_CIRCLE = 1 << 5, /*!< Red circle */
    Btn_CROSS = 1 << 6, /*!< Blue cross */
    Btn_SQUARE = 1 << 7, /*!< Pink square */

    Btn_SELECT = 1 << 8, /*!< Select button, left side */
    Btn_START = 1 << 11, /*!< Start button, right side */

    Btn_PS = 1 << 16, /*!< PS button, front center */
    Btn_MOVE = 1 << 19, /*!< Move button, big front button */
    Btn_T = 1 << 20, /*!< Trigger, on the back */

#if 0
    /* Not used for now - only on Sixaxis/DS3 or nav controller */
    Btn_L2 = 1 << 0x00,
    Btn_R2 = 1 << 0x01,
    Btn_L1 = 1 << 0x02,
    Btn_R1 = 1 << 0x03,
    Btn_L3 = 1 << 0x09,
    Btn_R3 = 1 << 0x0A,
    Btn_UP = 1 << 0x0C,
    Btn_RIGHT = 1 << 0x0D,
    Btn_DOWN = 1 << 0x0E,
    Btn_LEFT = 1 << 0x0F,
#endif
};

/*! Navigation Controller buttons.
 *
 * Note that the button assignment seems to be different on macOS
 * compared to Linux. Untested on Windows, contributions welcome.
 *
 * Use these values to index into e.g. SDL Joystick Buttons.
 **/
enum PSNav_Button {
#if defined(__APPLE__)
    NavBtn_CROSS = 14, /*!< Cross button */
    NavBtn_CIRCLE = 13, /*!< Circle button */

    NavBtn_L1 = 10, /*!< Shoulder button */
    NavBtn_L2 = 8, /*!< Trigger */
    NavBtn_L3 = 1, /*!< Analog stick */

    NavBtn_PS = 16, /*!< PS button */

    NavBtn_UP = 4, /*!< D-Pad UP */
    NavBtn_DOWN = 6, /*!< D-Pad DOWN */
    NavBtn_LEFT = 7, /*!< D-Pad LEFT */
    NavBtn_RIGHT = 5, /*!< D-Pad RIGHT */
#else
    NavBtn_CROSS = 0, /*!< Cross button */
    NavBtn_CIRCLE = 1, /*!< Circle button */

    NavBtn_L1 = 4, /*!< Shoulder button */
    NavBtn_L2 = 5, /*!< Trigger */
    NavBtn_L3 = 7, /*!< Analog stick */

    NavBtn_PS = 6, /*!< PS button */

    NavBtn_UP = 8, /*!< D-Pad UP */
    NavBtn_DOWN = 9, /*!< D-Pad DOWN */
    NavBtn_LEFT = 10, /*!< D-Pad LEFT */
    NavBtn_RIGHT = 11, /*!< D-Pad RIGHT */
#endif
};

/*! Navigation Controller axes.
 *
 * Use these values to index into e.g. SDL Joystick Axes.
 **/
enum PSNav_Axis {
    NavAxis_X = 0,
    NavAxis_Y = 1,
    NavAxis_Trigger = 2, /*!< might not work on macOS */
};

/*! Sensor types.
*
* Used by psmove_get_half_frame().
**/
enum PSMove_Sensor {
	Sensor_Accelerometer = 0,
	Sensor_Gyroscope,
};

/*! Frame of an input report.
 * Each input report sent by the PS Move Controller contains two readings for
 * the accelerometer and the gyroscope. The first one is the older one, and the
 * second one is the most recent one. If you need 120 Hz updates, you can
 * process both frames for each update. If you only need the latest reading,
 * use the second frame and ignore the first frame.
 *
 * Used by psmove_get_accelerometer_frame() and psmove_get_gyroscope_frame().
 **/
enum PSMove_Frame {
    Frame_FirstHalf = 0, /*!< The older frame */
    Frame_SecondHalf, /*!< The most recent frame */
};

/*! Battery charge level.
 * Charge level of the battery. Charging is indicated when the controller is
 * connected via USB, or when the controller is sitting in the charging dock.
 * In all other cases (Bluetooth, not in charging dock), the charge level is
 * indicated.
 *
 * Used by psmove_get_battery().
 **/
enum PSMove_Battery_Level {
    Batt_MIN = 0x00, /*!< Battery is almost empty (< 20%) */
    Batt_20Percent = 0x01, /*!< Battery has at least 20% remaining */
    Batt_40Percent = 0x02, /*!< Battery has at least 40% remaining */
    Batt_60Percent = 0x03, /*!< Battery has at least 60% remaining */
    Batt_80Percent = 0x04, /*!< Battery has at least 80% remaining */
    Batt_MAX = 0x05, /*!< Battery is fully charged (not on charger) */
    Batt_CHARGING = 0xEE, /*!< Battery is currently being charged */
    Batt_CHARGING_DONE = 0xEF, /*!< Battery is fully charged (on charger) */
};

/*! LED update result, returned by psmove_update_leds() */
enum PSMove_Update_Result {
    Update_Failed = 0, /*!< Could not update LEDs */
    Update_Success, /*!< LEDs successfully updated */
    Update_Ignored, /*!< LEDs don't need updating, see psmove_set_rate_limiting() */
};

/*! Boolean type. Use them instead of 0 and 1 to improve code readability. */
enum PSMove_Bool {
    PSMove_False = 0, /*!< False, Failure, Disabled (depending on context) */
    PSMove_True = 1, /*!< True, Success, Enabled (depending on context) */
};

/*! Remote configuration options, for psmove_set_remote_config() */
enum PSMove_RemoteConfig {
    PSMove_LocalAndRemote = 0, /*!< Use both local (hidapi) and remote (moved) devices */
    PSMove_OnlyLocal = 1, /*!< Use only local (hidapi) devices, ignore remote devices */
    PSMove_OnlyRemote = 2, /*!< Use only remote (moved) devices, ignore local devices */
};

enum PSMoveOrientation_Fusion_Type {
	OrientationFusion_None,
	OrientationFusion_MadgwickIMU,
	OrientationFusion_MadgwickMARG,
	OrientationFusion_ComplementaryMARG,
};

/*! Common Calibration Poses */
enum PSMove_CalibrationPose_Type {
    CalibrationPose_Upright,
    CalibrationPose_LyingFlat
};

/*! Coordinate systems to use for the sensor data */
enum PSMove_SensorDataBasis_Type {
    SensorDataBasis_Native,
    SensorDataBasis_OpenGL,
};

#ifndef SWIG
struct _PSMove;
typedef struct _PSMove PSMove; /*!< Handle to a PS Move Controller.
                                    Obtained via psmove_connect_by_id() */
#endif

/*! Size of buffer for holding the extension device's data as reported by the Move */
#define PSMOVE_EXT_DATA_BUF_SIZE 5

/*! Buffer for holding the extension device's data as reported by the Move */
typedef unsigned char PSMove_Ext_Data[PSMOVE_EXT_DATA_BUF_SIZE];

/*! Extension device information */
typedef struct {
    unsigned short dev_id;
    unsigned char dev_info[38];
} PSMove_Ext_Device_Info;

/* A 3d vector - methods in math/psmove_vector.h */
struct _PSMove_3AxisVector
{
	union {
		struct
		{
			float x;
			float y;
			float z;
		};
		float v[3];
	};
};
typedef struct _PSMove_3AxisVector PSMove_3AxisVector;

/* A 3x3 transform matrix - methods in math/psmove_vector.h */
struct _PSMove_3AxisTransform
{
	union {
		struct
		{
			float row0[3];
			float row1[3];
			float row2[3];
		};
		float m[9];
	};
};
typedef struct _PSMove_3AxisTransform PSMove_3AxisTransform;

/*! Library version number */
enum PSMove_Version {
    PSMOVE_CURRENT_VERSION = (PSMOVEAPI_VERSION_MAJOR << 16) |
                             (PSMOVEAPI_VERSION_MINOR << 8) |
                             (PSMOVEAPI_VERSION_PATCH << 0)
};
    
/**
 * \brief Initialize the library and check for the right version
 *
 * This library call should be used at the beginning of each application using
 * the PS Move API to check if the correct version of the library is loaded (for
 * dynamically-loaded versions of the PS Move API).
 *
 * \code
 *    if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
 *        fprintf(stderr, "PS Move API init failed (wrong version?)\n");
 *        exit(1);
 *    }
 * \endcode
 *
 * \param version Should be \ref PSMOVE_CURRENT_VERSION to check for the same
 *                version as was used at compile-time
 *
 * \return \ref PSMove_True on success (version compatible, library initialized)
 * \return \ref PSMove_False otherwise (version mismatch, initialization error)
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_init(enum PSMove_Version version);

/**
 * \brief Enable or disable the usage of local or remote devices
 *
 * By default, both local (hidapi) and remote (moved) controllers will be
 * used. If your application only wants to use locally-connected devices,
 * and ignore any remote controllers, call this function with
 * \ref PSMove_OnlyLocal - to use only remotely-connected devices, use
 * \ref PSMove_OnlyRemote instead.
 *
 * This function must be called before any other PS Move API functions are
 * used, as it changes the behavior of counting and connecting to devices.
 *
 * \param config \ref PSMove_LocalAndRemote, \ref PSMove_OnlyLocal or
 *               \ref PSMove_OnlyRemote
 **/
ADDAPI void
ADDCALL psmove_set_remote_config(enum PSMove_RemoteConfig config);

/**
 * \brief Get the number of available controllers
 *
 * \return Number of controllers available (USB + Bluetooth + Remote)
 **/
ADDAPI int
ADDCALL psmove_count_connected();

/**
 * \brief Connect to the default PS Move controller
 *
 * This is a convenience function, having the same effect as:
 *
 * \code psmove_connect_by_id(0) \endcode
 *
 * \return A new \ref PSMove handle, or \c NULL on error
 **/
ADDAPI PSMove *
ADDCALL psmove_connect();

/**
 * \brief Connect to a specific PS Move controller
 *
 * This will connect to a controller based on its index. The controllers
 * available are usually:
 *
 *  1. The locally-connected controllers (USB, Bluetooth)
 *  2. The remotely-connected controllers (exported via \c moved)
 *
 * The order of controllers can be different on each application start,
 * so use psmove_get_serial() to identify the controllers if more than
 * one is connected, and you need some fixed ordering. The global ordering
 * (first local controllers, then remote controllers) is fixed, and will
 * be guaranteed by the library.
 *
 * \param id Zero-based index of the controller to connect to
 *           (0 .. psmove_count_connected() - 1)
 *
 * \return A new \ref PSMove handle, or \c NULL on error
 **/
ADDAPI PSMove *
ADDCALL psmove_connect_by_id(int id);

/**
 * \brief Get the connection type of a PS Move controller
 *
 * For now, controllers connected via USB can't use psmove_poll() and
 * all related features (sensor and button reading, etc..). Because of
 * this, you might want to check if the controllers are connected via
 * Bluetooth before using psmove_poll() and friends.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return \ref Conn_Bluetooth if the controller is connected via Bluetooth
 * \return \ref Conn_USB if the controller is connected via USB
 * \return \ref Conn_Unknown on error
 **/
ADDAPI enum PSMove_Connection_Type
ADDCALL psmove_connection_type(PSMove *move);

/**
 * \brief Check if the controller is remote (\c moved) or local.
 *
 * This can be used to determine to which machine the controller is
 * connected to, and can be helpful in debugging, or if you need to
 * handle remote controllers differently from local controllers.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return \ref PSMove_False if the controller is connected locally
 * \return \ref PSMove_True if the controller is connected remotely
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_is_remote(PSMove *move);

/**
 * \brief Get the serial number (Bluetooth MAC address) of a controller.
 *
 * The serial number is usually the Bluetooth MAC address of a
 * PS Move controller. This can be used to identify different
 * controllers when multiple controllers are connected, and is
 * especially helpful if your application needs to identify
 * controllers or guarantee a special ordering.
 *
 * The resulting value has the format:
 *
 * \code "aa:bb:cc:dd:ee:ff" \endcode
 *
 * \param move A valid \ref PSMove handle
 *
 * \return The serial number of the controller. The caller must free
 *         the result using \ref psmove_free_mem() when it is not
 *         needed anymore.
 **/
ADDAPI char *
ADDCALL psmove_get_serial(PSMove *move);

/**
 * \brief Get the model type (ZCM1 or ZCM2) of a controller.
 *
 * There are two primary version of the PSMove controller.
 * The ZCM1 is the original PSMove controller that shipped with the PS3.
 * The ZCM2 is the newer PSMove controller that shipped with the PS4.
 * The most important difference is that ZCM2 does not have a magnetometer.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return The \ref PSMove_Model_Type type of the controller
 **/
ADDAPI enum PSMove_Model_Type
ADDCALL psmove_get_model(PSMove *move);

/**
 * \brief Pair a controller connected via USB with the computer.
 *
 * This function assumes that psmove_connection_type() returns
 * \ref Conn_USB for the given controller. This will set the
 * target Bluetooth host address of the controller to this
 * computer's default Bluetooth adapter address. This function
 * has been implemented and tested with the following operating
 * systems and Bluetooth stacks:
 *
 *  * Linux 2.6 (Bluez)
 *  * Mac OS X >= 10.6
 *  * Windows 7 (Microsoft Bluetooth stack)
 *
 * \attention On Windows, this function does not work with 3rd
 * party stacks like Bluesoleil. Use psmove_pair_custom() and
 * supply the Bluetooth host adapter address manually. It is
 * recommended to only use the Microsoft Bluetooth stack for
 * Windows with the PS Move API to avoid problems.
 *
 * If your computer doesn't have USB host mode (e.g. because it
 * is a mobile device), you can use psmove_pair_custom() on a
 * different computer and specify the Bluetooth address of the
 * mobile device instead. For most use cases, you can use the
 * \c psmove pair command-line utility.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return \ref PSMove_True if the pairing was successful
 * \return \ref PSMove_False if the pairing failed
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_pair(PSMove *move);

/**
 * \brief Add an entry for a controller paired on another host.
 *
 * \param addr The Bluetooth address of the PS move to add
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_host_pair_custom(const char *addr);

/**
 * \brief Add an entry for a controller paired on another host.
 *
 * This function behaves the same as psmove_host_pair_custom() but allows you
 * to specify a controller hardware model. Use this to pair a PS4 Move Motion
 * controller (model ZCM2).
 *
 * \param addr The Bluetooth address of the PS move to add
 * \param model The hardware model type of the controller
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_host_pair_custom_model(const char *addr, enum PSMove_Model_Type model);

/**
 * \brief Pair a controller connected via USB to a specific address.
 *
 * This function behaves the same as psmove_pair(), but allows you to
 * specify a custom Bluetooth host address.
 *
 * \param move A valid \ref PSMove handle
 * \param new_host_string The host address in the format \c "aa:bb:cc:dd:ee:ff"
 *
 * \return \ref PSMove_True if the pairing was successful
 * \return \ref PSMove_False if the pairing failed
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_pair_custom(PSMove *move, const char *new_host_string);

/**
 * \brief Enable or disable LED update rate limiting.
 *
 * If LED update rate limiting is enabled, psmove_update_leds() will make
 * ignore extraneous updates (and return \ref Update_Ignored) if the update
 * rate is too high, or if the color hasn't changed and the timeout has not
 * been hit.
 *
 * By default, rate limiting is enabled.
 *
 * \warning If rate limiting is disabled, the read performance might
 *          be decreased, especially on Linux.
 *
 * \param move A valid \ref PSMove handle
 * \param enabled \ref PSMove_True to enable rate limiting,
 *                \ref PSMove_False to disable
 **/
ADDAPI void
ADDCALL psmove_set_rate_limiting(PSMove *move, enum PSMove_Bool enabled);

/**
 * \brief Set the RGB LEDs on the PS Move controller.
 *
 * This sets the RGB values of the LEDs on the Move controller. Usage examples:
 *
 * \code
 *    psmove_set_leds(move, 255, 255, 255);  // white
 *    psmove_set_leds(move, 255, 0, 0);      // red
 *    psmove_set_leds(move, 0, 0, 0);        // black (off)
 * \endcode
 *
 * This function will only update the library-internal state of the controller.
 * To really update the LEDs (write the changes out to the controller), you
 * have to call psmove_update_leds() after calling this function.
 *
 * \param move A valid \ref PSMove handle
 * \param r The red value (0..255)
 * \param g The green value (0..255)
 * \param b The blue value (0..255)
 **/
ADDAPI void
ADDCALL psmove_set_leds(PSMove *move, unsigned char r, unsigned char g,
        unsigned char b);

/**
 * \brief Set the PWM frequency used in dimming the RGB LEDs.
 *
 * The RGB LEDs in the Move controller are dimmed using pulse-width modulation (PWM).
 * This function lets you modify the PWM frequency. The default is around 188 kHz and
 * can also be restored by resetting the controller (using the small reset button on
 * the back).
 *
 * \note Make sure to switch off the LEDs prior to calling this function. If you do
 *       not do this, changing the PWM frequency will switch off the LEDs and keep
 *       them off until the Bluetooth connection has been teared down and then
 *       reestablished.
 *
 * \note Frequency values outside the valid range (see the parameter description) are
 *       treated as errors.
 *
 * \note Even though the controller lets you increase the frequency to several
 *       Megahertz, there is usually not much use in operating at such extreme rates.
 *       Additionally, you will even lose resolution at these rates, i.e. the number
 *       of distinct LED intensities between "off" and "fully lit" decreases. For
 *       example: at 7 MHz there are only 5 different intensities left instead of the
 *       usual 256. This is not a feature of PWM per se but is rather due to
 *       software/hardware limitations of the Move controller.
 *
 * \param freq The PWM frequency in Hertz (range is 733 Hz to 24 MHz)
 *
 * \return \ref PSMove_True on success
 * \return \ref PSMove_False on error
 */
ADDAPI enum PSMove_Bool
ADDCALL psmove_set_led_pwm_frequency(PSMove *move, unsigned long freq);

/**
 * \brief Set the rumble intensity of the PS Move controller.
 *
 * This sets the rumble (vibration motor) intensity of the
 * Move controller. Usage example:
 *
 * \code
 *   psmove_set_rumble(move, 255);  // strong rumble
 *   psmove_set_rumble(move, 128);  // medium rumble
 *   psmove_set_rumble(move, 0);    // rumble off
 * \endcode
 *
 * This function will only update the library-internal state of the controller.
 * To really update the rumble intensity (write the changes out to the
 * controller), you have to call psmove_update_leds() (the rumble value is sent
 * together with the LED updates, that's why you have to call it even for
 * rumble updates) after calling this function.
 *
 * \param move A valid \ref PSMove handle
 * \param rumble The rumble intensity (0..255)
 **/
ADDAPI void
ADDCALL psmove_set_rumble(PSMove *move, unsigned char rumble);

/**
 * \brief Send LED and rumble values to the controller.
 *
 * This writes the LED and rumble changes to the controller. You have to call
 * this function regularly, or the controller will switch off the LEDs and
 * rumble automatically (after about 4-5 seconds). When rate limiting is
 * enabled, you can just call this function in your main loop, and the LEDs
 * will stay on properly (with extraneous updates being ignored to not flood
 * the controller with updates).
 *
 * When rate limiting (see psmove_set_rate_limiting()) is disabled, you have
 * to make sure to not call this function not more often then e.g. every
 * 80 ms to avoid flooding the controller with updates.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return \ref Update_Success on success
 * \return \ref Update_Ignored if the change was ignored (see psmove_set_rate_limiting())
 * \return \ref Update_Failed (= \c 0) on error
 **/
ADDAPI enum PSMove_Update_Result
ADDCALL psmove_update_leds(PSMove *move);

/**
 * \brief Read new sensor/button data from the controller.
 *
 * For most sensor and button functions, you have to call this function
 * to read new updates from the controller.
 *
 * How to detect dropped frames:
 *
 * \code
 *     int seq_old = 0;
 *     while (1) {
 *         int seq = psmove_poll(move);
 *         if ((seq_old > 0) && ((seq_old % 16) != (seq - 1))) {
 *             // dropped frames
 *         }
 *         seq_old = seq;
 *     }
 * \endcode
 *
 * In practice, you usually use this function in a main loop and guard
 * all your sensor/button updating functions with it:
 *
 * \code
 *     while (1) {
 *         if (psmove_poll(move)) {
 *             unsigned int pressed, released;
 *             psmove_get_button_events(move, &pressed, &released);
 *             // process button events
 *         }
 *
 *         // update the application state
 *         // draw the current frame
 *     }
 * \endcode
 *
 * \return a positive sequence number (1..16) if new data is
 *         available
 * \return \c 0 if no (new) data is available or an error occurred
 *
 * \param move A valid \ref PSMove handle
 **/
ADDAPI int
ADDCALL psmove_poll(PSMove *move);

/**
 * \brief Get the extension device's data as reported by the Move.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param data Pointer to store the data, must not be \ref NULL
 *
 * \return \ref PSMove_True on success
 * \return \ref PSMove_False on error
 * 
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_get_ext_data(PSMove *move, PSMove_Ext_Data *data);

/**
 * \brief Send data to a connected extension device.
 *
 * \param move A valid \ref PSMove handle
 * \param data Pointer to the data which to send
 * \param length Number of bytes in \ref data
 *
 * \return \ref PSMove_True on success
 * \return \ref PSMove_False on error
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_send_ext_data(PSMove *move, const unsigned char *data, unsigned char length);

/**
 * \brief Get the current button states from the controller.
 *
 * The status of the buttons is described as a bitfield, with a bit
 * in the result being \c 1 when the corresponding \ref PSMove_Button
 * is pressed.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * Example usage:
 *
 * \code
 *     if (psmove_poll(move)) {
 *         unsigned int buttons = psmove_get_buttons(move);
 *         if (buttons & Btn_PS) {
 *             printf("The PS button is currently pressed.\n");
 *         }
 *     }
 * \endcode
 *
 * \param move A valid \ref PSMove handle
 *
 * \return A bit field of \ref PSMove_Button states
 **/
ADDAPI unsigned int
ADDCALL psmove_get_buttons(PSMove *move);


/**
 * \brief Get new button events since the last call to this fuction.
 *
 * This is an advanced version of psmove_get_buttons() that takes care
 * of tracking the previous button states and comparing the previous
 * states with the current states to generate two bitfields, which is
 * usually more suitable for event-driven applications:
 *
 *  * \c pressed - all buttons that have been pressed since the last call
 *  * \c released - all buttons that have been released since the last call
 *
 * Example usage:
 *
 * \code
 *     if (psmove_poll(move)) {
 *         unsigned int pressed, released;
 *         psmove_get_button_events(move, &pressed, &released);
 *
 *         if (pressed & Btn_MOVE) {
 *             printf("The Move button has been pressed now.\n");
 *         } else if (released & Btn_MOVE) {
 *             printf("The Move button has been released now.\n");
 *         }
 *     }
 * \endcode
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param pressed Pointer to store a bitfield of new press events \c NULL
 * \param released Pointer to store a bitfield of new release events \c NULL
 **/
ADDAPI void
ADDCALL psmove_get_button_events(PSMove *move, unsigned int *pressed,
        unsigned int *released);

/**
 * \brief Check if an extension device is connected to the controller.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return \ref PSMove_True if an extension device is connected
 * \return \ref PSMove_False if no extension device is connected or in case of an error
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_is_ext_connected(PSMove *move);

/**
 * \brief Get information from an extension device connected to the controller.
 *
 * \note Since the information is retrieved from the extension device itself, a
 * noticeable delay may occur when calling this function.
 *
 * \param move A valid \ref PSMove handle
 * \param ext Pointer to a \ref PSMove_Ext_Device_Info that will store the
 *            information. Must not be \ref NULL.
 *
 * \return \ref PSMove_True on success
 * \return \ref PSMove_False on error
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_get_ext_device_info(PSMove *move, PSMove_Ext_Device_Info *info);

/**
 * \brief Get the battery charge level of the controller.
 *
 * This function retrieves the charge level of the controller or
 * the charging state (if the controller is currently being charged).
 *
 * See \ref PSMove_Battery_Level for details on the result values.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \return A \ref PSMove_Battery_Level (\ref Batt_CHARGING when charging)
 **/
ADDAPI enum PSMove_Battery_Level
ADDCALL psmove_get_battery(PSMove *move);

/**
 * \brief Get the current raw device temperature reading of the
 * controller.
 *
 * This gets the raw sensor value of the internal temperature sensor.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return The raw temperature sensor reading
 **/
ADDAPI int
ADDCALL psmove_get_temperature(PSMove *move);

/**
 * \brief Get the current device temperature reading in degree
 * Celsius.
 *
 * This gets the raw temperature sensor value of the internal
 * temperature sensor and then converts it to degree Celsius.
 *
 * The result range is -10..70 °C. Values outside this range will be
 * clipped.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \note This is NOT room temperature, but the temperature of a small
 * thermistor on the controller's PCB. This means that under normal
 * operation the temperature returned by this function will be higher
 * than room temperature due to heat up from current flow.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return The temperature sensor reading in degree Celsius
 **/
ADDAPI float
ADDCALL psmove_get_temperature_in_celsius(PSMove *move);

/**
 * \brief Get the value of the PS Move analog trigger.
 *
 * Get the current value of the PS Move analog trigger. The trigger
 * is also exposed as digital button using psmove_get_buttons() in
 * combination with \ref Btn_T.
 *
 * Usage example:
 *
 * \code
 *     // Control the red LED brightness via the trigger
 *     while (1) {
 *         if (psmove_poll()) {
 *             unsigned char value = psmove_get_trigger(move);
 *             psmove_set_leds(move, value, 0, 0);
 *             psmove_update_leds(move);
 *         }
 *     }
 * \endcode
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return 0 if the trigger is not pressed
 * \return 1-254 when the trigger is partially pressed
 * \return 255 if the trigger is fully pressed
 **/
ADDAPI unsigned char
ADDCALL psmove_get_trigger(PSMove *move);

/**
 * \brief Get the raw accelerometer reading from the PS Move.
 *
 * This function reads the raw (uncalibrated) sensor values from
 * the controller. To read calibrated sensor values, use
 * psmove_get_accelerometer_frame().
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param ax Pointer to store the raw X axis reading, or \c NULL
 * \param ay Pointer to store the raw Y axis reading, or \c NULL
 * \param az Pointer to store the raw Z axis reading, or \c NULL
 **/
ADDAPI void
ADDCALL psmove_get_accelerometer(PSMove *move, int *ax, int *ay, int *az);

/**
 * \brief Get the raw gyroscope reading from the PS Move.
 *
 * This function reads the raw (uncalibrated) sensor values from
 * the controller. To read calibrated sensor values, use
 * psmove_get_gyroscope_frame().
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param gx Pointer to store the raw X axis reading, or \c NULL
 * \param gy Pointer to store the raw Y axis reading, or \c NULL
 * \param gz Pointer to store the raw Z axis reading, or \c NULL
 **/
ADDAPI void
ADDCALL psmove_get_gyroscope(PSMove *move, int *gx, int *gy, int *gz);

/**
 * \brief Get the raw magnetometer reading from the PS Move.
 *
 * This function reads the raw sensor values from the controller,
 * pointing to magnetic north.
 *
 * The result value range is -2048..+2047. The magnetometer is located
 * roughly below the glowing orb - you can glitch the values with a
 * strong kitchen magnet by moving it around the bottom ring of the orb.
 * You can detect if a magnet is nearby by checking if any two values
 * stay at zero for several frames.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param mx Pointer to store the raw X axis reading, or \c NULL
 * \param my Pointer to store the raw Y axis reading, or \c NULL
 * \param mz Pointer to store the raw Z axis reading, or \c NULL
 **/
ADDAPI void
ADDCALL psmove_get_magnetometer(PSMove *move, int *mx, int *my, int *mz);

/**
 * \brief Get the calibrated accelerometer values (in g) from the controller.
 *
 * Assuming that psmove_has_calibration() returns \ref PSMove_True, this
 * function will give you the calibrated accelerometer values in g. To get
 * the raw accelerometer readings, use psmove_get_accelerometer().
 *
 * Usage example:
 *
 * \code
 *     if (psmove_poll(move)) {
 *         float ay;
 *         psmove_get_accelerometer_frame(move, Frame_SecondHalf,
 *                 NULL, &ay, NULL);
 *
 *         if (ay > 0.5) {
 *             printf("Controller is pointing up.\n");
 *         } else if (ay < -0.5) {
 *             printf("Controller is pointing down.\n");
 *         }
 *     }
 * \endcode
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param frame \ref Frame_FirstHalf or \ref Frame_SecondHalf (see \ref PSMove_Frame)
 * \param ax Pointer to store the X axis reading, or \c NULL
 * \param ay Pointer to store the Y axis reading, or \c NULL
 * \param az Pointer to store the Z axis reading, or \c NULL
 **/
ADDAPI void
ADDCALL psmove_get_accelerometer_frame(PSMove *move, enum PSMove_Frame frame,
        float *ax, float *ay, float *az);

/**
 * \brief Get the calibrated gyroscope values (in rad/s) from the controller.
 *
 * Assuming that psmove_has_calibration() returns \ref PSMove_True, this
 * function will give you the calibrated gyroscope values in rad/s. To get
 * the raw gyroscope readings, use psmove_get_gyroscope().
 *
 * Usage example:
 *
 * \code
 *     if (psmove_poll(move)) {
 *         float gz;
 *         psmove_get_gyroscope_frame(move, Frame_SecondHalf,
 *                 NULL, NULL, &gz);
 *
 *         // Convert rad/s to RPM
 *         gz = gz * 60 / (2*M_PI);
 *
 *         printf("Rotation: %.2f RPM\n", gz);
 *     }
 * \endcode
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param frame \ref Frame_FirstHalf or \ref Frame_SecondHalf (see \ref PSMove_Frame)
 * \param gx Pointer to store the X axis reading, or \c NULL
 * \param gy Pointer to store the Y axis reading, or \c NULL
 * \param gz Pointer to store the Z axis reading, or \c NULL
 **/
ADDAPI void
ADDCALL psmove_get_gyroscope_frame(PSMove *move, enum PSMove_Frame frame,
        float *gx, float *gy, float *gz);

/**
 * Get a half-frame from the accelerometer or gyroscope from the
 * PS Move after using psmove_poll() previously.
 *
 * sensor must be Sensor_Accelerometer or Sensor_Accelerometer.
 *
 * frame must be Frame_FirstHalf or Frame_SecondHalf.
 *
 * x, y and z can point to integer locations that will be filled
 * with the readings. If any are NULL, the fields will be ignored.
 **/
ADDAPI void
ADDCALL psmove_get_half_frame(PSMove *move, enum PSMove_Sensor sensor,
	enum PSMove_Frame frame, int *x, int *y, int *z);

/**
 * \brief Get the raw magnetometer reading from the PS Move.
 *
 * This function reads the raw sensor values from the controller,
 * pointing to magnetic north.
 *
 * The result value range is -2048..+2047. The magnetometer is located
 * roughly below the glowing orb - you can glitch the values with a
 * strong kitchen magnet by moving it around the bottom ring of the orb.
 * You can detect if a magnet is nearby by checking if any two values
 * stay at zero for several frames.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param mx Pointer to store the raw X axis reading, or \c NULL
 * \param my Pointer to store the raw Y axis reading, or \c NULL
 * \param mz Pointer to store the raw Z axis reading, or \c NULL
 **/
ADDAPI void
ADDCALL psmove_get_magnetometer(PSMove *move, int *mx, int *my, int *mz);

/**
 * \brief Get the normalized magnetometer vector from the controller.
 *
 * The normalized magnetometer vector is a three-axis vector where each
 * component is in the range [-1,+1], including both endpoints. The range
 * will be dynamically determined based on the highest (and lowest) value
 * observed during runtime. To get the raw magnetometer readings, use
 * psmove_get_magnetometer().
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param mx Pointer to store the X axis reading, or \c NULL
 * \param my Pointer to store the Y axis reading, or \c NULL
 * \param mz Pointer to store the Z axis reading, or \c NULL
 **/
ADDAPI void
ADDCALL psmove_get_magnetometer_vector(PSMove *move, float *mx, float *my, float *mz);

/**
 * \brief Get the normalized magnetometer vector from the controller.
 *
 * The normalized magnetometer vector is a three-axis vector where each
 * component is in the range [-1,+1], including both endpoints. The range
 * will be dynamically determined based on the highest (and lowest) value
 * observed during runtime. To get the raw magnetometer readings, use
 * psmove_get_magnetometer().
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param out_m The output \ref PSMove_3AxisVector
 **/
ADDAPI void
ADDCALL psmove_get_magnetometer_3axisvector(PSMove *move, PSMove_3AxisVector *out_m);

/**
 * \brief Reset the magnetometer calibration state.
 *
 * This will reset the magnetometer calibration data, so they can be
 * re-adjusted dynamically. Used by the calibration utility.
 *
 * \ref move A valid \ref PSMove handle
 **/
ADDAPI void
ADDCALL psmove_reset_magnetometer_calibration(PSMove *move);

/**
 * \brief Save the magnetometer calibration values.
 *
 * This will save the magnetometer calibration data to persistent storage.
 * If a calibration already exists, this will overwrite the old values.
 *
 * \param move A valid \ref PSMove handle
 **/
ADDAPI void
ADDCALL psmove_save_magnetometer_calibration(PSMove *move);

/**
 * \brief Return the raw magnetometer calibration range.
 *
 * The magnetometer calibration is dynamic at runtime - this function returns
 * the raw range of magnetometer calibration. The user should rotate the
 * controller in all directions to find the response range of the controller
 * (this will be dynamically adjusted).
 *
 * \param move A valid \ref PSMove handle
 *
 * \return The smallest raw sensor range difference of all three axes
 **/
ADDAPI float
ADDCALL psmove_get_magnetometer_calibration_range(PSMove *move);

/**
 * \brief Check if calibration is available on this controller.
 *
 * For psmove_get_accelerometer_frame() and psmove_get_gyroscope_frame()
 * to work, the calibration data has to be availble. This usually happens
 * at pairing time via USB. The calibration files are stored in the PS
 * Move API data directory (see psmove_util_get_data_dir()) and can be
 * copied between machines (e.g. from the machine you do your pairing to
 * the machine where you run the API on, which is especially important for
 * mobile devices, where USB host mode might not be supported).
 *
 * If no calibration is available, the two functions returning calibrated
 * values will return uncalibrated values. Also, the orientation features
 * will not work without calibration.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return \ref PSMove_True if calibration is supported, \ref PSMove_False otherwise
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_has_calibration(PSMove *move);

/**
 * \brief Dump the calibration information to stdout.
 *
 * This is mostly useful for developers wanting to analyze the
 * calibration data of a given PS Move controller for debugging
 * or development purposes.
 *
 * The current calibration information (if available) will be printed to \c
 * stdout, including an interpretation of raw values where available.
 *
 * \param move A valid \ref PSMove handle
 **/
ADDAPI void
ADDCALL psmove_dump_calibration(PSMove *move);

/**
 * \brief Copy the raw calibration data to buffer.
 *
 * The current raw calibration data (if available) will be copied to \c
 * supplied buffer.
 *
 * \param move A valid \ref PSMove handle
 * \param data A pointer to a buffer to copy raw calibration data to
 * \param size Size of calibration data buffer, will hold raw calibration size on return
**/
ADDAPI enum PSMove_Bool
ADDCALL psmove_get_calibration_raw(PSMove *move, unsigned char *data, size_t *size);

/**
 * \brief Enable or disable orientation tracking.
 *
 * This will enable orientation tracking and update the internal orientation
 * quaternion (which can be retrieved using psmove_get_orientation()) when
 * psmove_poll() is called.
 *
 * In addition to enabling the orientation tracking features, calibration data
 * and an orientation algorithm (usually built-in) has to be used, too. You can
 * use psmove_has_orientation() after enabling orientation tracking to check if
 * orientation features can be used.
 *
 * \param move A valid \ref PSMove handle
 * \param enabled \ref PSMove_True to enable orientation tracking, \ref PSMove_False to disable
 **/
ADDAPI void
ADDCALL psmove_enable_orientation(PSMove *move, enum PSMove_Bool enabled);

/**
 * \brief Check if orientation tracking is available for this controller.
 *
 * The orientation tracking feature depends on the availability of an
 * orientation tracking algorithm (usually built-in) and the calibration
 * data availability (as determined by psmove_has_calibration()). In addition
 * to that (because orientation tracking is somewhat computationally
 * intensive, especially on embedded systems), you have to enable the
 * orientation tracking manually via psmove_enable_orientation()).
 *
 * If this function returns \ref PSMove_False, the orientation features
 * will not work - check for missing calibration data and make sure that
 * you have called psmove_enable_orientation() first.
 *
 * \param move A valid \ref PSMove handle
 *
 * \return \ref PSMove_True if calibration is supported, \ref PSMove_False otherwise
 **/
ADDAPI enum PSMove_Bool
ADDCALL psmove_has_orientation(PSMove *move);

/**
 * \brief Get the current orientation as quaternion.
 *
 * This will get the current 3D rotation of the PS Move controller as
 * quaternion. You will have to call psmove_poll() regularly in order to have
 * good quality orientation tracking.
 *
 * In order for the orientation tracking to work, you have to enable tracking
 * first using psmove_enable_orientation().  In addition to enabling tracking,
 * an orientation algorithm has to be present, and calibration data has to be
 * available. You can use psmove_has_orientation() to check if all
 * preconditions are fulfilled to do orientation tracking.
 *
 * \param move A valid \ref PSMove handle
 * \param w A pointer to store the w part of the orientation quaternion
 * \param x A pointer to store the x part of the orientation quaternion
 * \param y A pointer to store the y part of the orientation quaternion
 * \param z A pointer to store the z part of the orientation quaternion
 **/
ADDAPI void
ADDCALL psmove_get_orientation(PSMove *move,
        float *w, float *x, float *y, float *z);

/**
 * \brief Reset the current orientation quaternion.
 *
 * This will set the current 3D rotation of the PS Move controller as
 * quaternion. You can use this function to re-adjust the orientation of the
 * controller when it points towards the camera.
 *
 * This function will automatically be called by psmove_enable_orientation(),
 * but you might want to call it manually when the controller points towards
 * the screen/camera for accurate orientation readings relative to a
 * known-good orientation of the controller.
 *
 * \param move A valid \ref PSMove handle
 **/
ADDAPI void
ADDCALL psmove_reset_orientation(PSMove *move);

/**
 * \brief Set the orientation fusion algorithm to use.
 *
 * There are currently the following orientation filter algorithms available:
 * OrientationFusion_MadgwickIMU - Classic MadgwickIMU: Gyro integration + Gravity correction
 *  - 1st Least expensive
 *  - Con: Suffers from pretty bad drift
 * OrientationFusion_MadgwickMARG - Classic MadgwickMARG: Gyro integration + Gravity/Magnetometer correction
 *  - 2nd least expensive
 *  - Con: Suffers from slow drift about the yaw
 * OrientationFusion_ComplementaryMARG - Gyro integration blended with optimized Gravity/Magnetometer alignment
 *  - Suffers no drift
 *  - Con: Most expensive algorithms of the three (but not horrendously so)

 * \param move A valid \ref PSMove handle
 * \param fusion_type The orientation fusion algorithm denoted by the \ref PSMoveOrientation_Fusion_Type enum
 **/
ADDAPI void
ADDCALL psmove_set_orientation_fusion_type(PSMove *move, enum PSMoveOrientation_Fusion_Type fusion_type);

/**
 * \brief Set a common transform used on the calibration data in the psmove_get_transform_<sensor>_... methods
 *
 * This method sets the transform used to modify the calibration vectors returned by:
 * - psmove_orientation_get_magnetometer_calibration_direction()
 * - psmove_orientation_get_gravity_calibration_direction()
 *
 * The transformed calibration data is used by the orientation filter to compute 
 * a quaternion (see \ref psmove_orientation_get_quaternion) that represents 
 * the controllers current rotation from the "identity pose".
 * 
 * Historically, the "identity pose" bas been with the controller laying flat
 * with the controller pointing at the screen. However, now that we have a
 * calibration step that record the magnetic field direction relative to 
 * gravity it makes more sense to make the identity pose with the controller 
 * sitting vertically since it's more stable to record that way. 
 *  
 * In order to maintain reverse compatibility, this transform defaults to rotating
 * the vertically recorded calibration vectors 90 degrees about X axis as if the 
 * controller was laying flat during calibration.
 *
 * Therefore, if you want a different "identity pose" then the default,
 * use this method to set a custom transform.
 *
 * There are the following transforms available:
 * - CalibrationPose_Upright - "identity pose" is the controller standing upright
 * - CalibrationPose_LyingFlat - "identity pose" is the controller laying down pointed at the screen
 *
 * \param orientation_state A valid \ref PSMoveOrientation handle
 * \param transform A \ref PSMove_CalibrationPose_Type common transform to apply to the calibration data
 **/
ADDAPI void
ADDCALL psmove_set_calibration_pose(PSMove *move, enum PSMove_CalibrationPose_Type calibration_pose);

/**
* \brief Set the custom transform used on the calibration data in the psmove_get_transform_<sensor>_... methods
*
* \param orientation_state A valid \ref PSMoveOrientation handle
* \param transform A \ref PSMove_3AxisTransform transform to apply to the calibration data
**/
ADDAPI void
ADDCALL psmove_set_calibration_transform(PSMove *move, const PSMove_3AxisTransform *transform);

/**
* \brief Get the native earth gravity direction.
*
* This returns the native direction of the gravitational field in the identity pose during calibration.
*
* \param move A valid \ref PSMove handle
*
* \return The expected direction of gravity
**/
ADDAPI void
ADDCALL psmove_get_identity_gravity_calibration_direction(PSMove *move, PSMove_3AxisVector *out_a);

/**
* \brief Get the transformed earth gravity direction.
*
* This returns the direction of the gravitational field in the transformed identity pose.
*
* \param move A valid \ref PSMove handle
*
* \return The transformed expected direction of gravity
**/
ADDAPI void
ADDCALL psmove_get_transformed_gravity_calibration_direction(PSMove *move, PSMove_3AxisVector *out_a);

/**
* \brief Get the calibration magnetometer direction.
*
* This returns the direction of the magnetic field in the un-transformed identity pose.
*
* \param move A valid \ref PSMove handle
*
* \return The direction of the magnetic field
**/
ADDAPI void
ADDCALL psmove_get_identity_magnetometer_calibration_direction(PSMove *move, PSMove_3AxisVector *out_m);

/**
* \brief Get the transformed calibration magnetometer direction.
*
* This returns the direction of the magnetic field in the transformed identity pose.
*
* \param move A valid \ref PSMove handle
* \param out_m The output \ref PSMove_3AxisVector
**/
ADDAPI void
ADDCALL psmove_get_transformed_magnetometer_calibration_direction(PSMove *move, PSMove_3AxisVector *out_m);

/**
* \brief Set the calibration magnetometer direction.
*
* This sets the direction of the magnetic field in the identity pose.
* This is typically only set during calibration.
*
* \param move A valid \ref PSMove handle
* \param m A valid \ref PSMoveVector
**/
ADDAPI void
ADDCALL psmove_set_magnetometer_calibration_direction(PSMove *move, PSMove_3AxisVector *m);

/**
 * \brief Set the transform used on the sensor data in the psmove_get_transform_<sensor>_... methods
 *
 * This method sets the transform used to modify the sensor vectors returned by:
 * - psmove_get_transformed_magnetometer_3axisvector()
 * - psmove_get_transformed_accelerometer_frame_3axisvector()
 * - psmove_get_transformed_accelerometer_frame_direction()
 * - psmove_get_transformed_gyroscope_frame_3axisvector()
 *
 * The transformed sensor data is used by the orientation filter to compute 
 * a quaternion (see \ref psmove_orientation_get_quaternion) that represents 
 * the controllers current rotation from the "identity pose".
 * 
 * Historically, the sensor data in the orientation code has been rotated 90 degrees 
 * clockwise about the x-axis. The original Madgwick orientation filter was coded to assume
 * an OpenGL style coordinate system (+x=right, +y=up, +z=out of screen), rather than 
 * than PSMoves coordinate system where:
 *
 * +x = From Select to Start button
 * +y = From Trigger to Move button
 * +z = From glowing orb to USB connector
 *
 * The current default sets the sensor transform to assume an OpenGL style coordinate system
 * in order to maintain reverse compatibility
 *
 * There are the following transforms available:
 * - SensorDataBasis_Native - Keep the sensor data as it was
 * - SensorDataBasis_OpenGL - Rotate 90 degrees about the x-axis (historical default)
 *
 * \param orientation_state A valid \ref PSMoveOrientation handle
 * \param transform A \ref PSMove_SensorDataBasis_Type transform to apply to the sensor data
 **/
ADDAPI void
ADDCALL psmove_set_sensor_data_basis(PSMove *move, enum PSMove_SensorDataBasis_Type basis_type);

/**
* \brief Set the transform used on the sensor data in the psmove_get_transform_<sensor>_... methods
*
* \param orientation_state A valid \ref PSMoveOrientation handle
* \param transform A \ref PSMove_3AxisTransform transform to apply to the sensor data
**/
ADDAPI void
ADDCALL psmove_set_sensor_data_transform(PSMove *move, const PSMove_3AxisTransform *transform);

/**
 * \brief Get the transformed current magnetometer direction.
 *
 * This returns the current normalized direction of the magnetic field with the sensor transform applied.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param out_m The output \ref PSMove_3AxisVector
 **/
ADDAPI void
ADDCALL psmove_get_transformed_magnetometer_direction(PSMove *move, PSMove_3AxisVector *out_m);

/**
 * \brief Get the transformed current accelerometer vector.
 *
 * This returns the current non-normalized vector of the accelerometer with the sensor transform applied.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param out_a The output \ref PSMove_3AxisVector
 **/
ADDAPI void
ADDCALL psmove_get_transformed_accelerometer_frame_3axisvector(PSMove *move, enum PSMove_Frame frame, PSMove_3AxisVector *out_a);

/**
 * \brief Get the transformed normalized current accelerometer direction.
 *
 * This returns the current normalized direction of the accelerometer with the sensor transform applied.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param out_a The output \ref PSMove_3AxisVector
 **/
ADDAPI void
ADDCALL psmove_get_transformed_accelerometer_frame_direction(PSMove *move, enum PSMove_Frame frame, PSMove_3AxisVector *out_a);

/**
 * \brief Get the transformed current gyroscope vector.
 *
 * This returns the current gyroscope vector (omega) with the sensor transform applied.
 *
 * You need to call psmove_poll() first to read new data from the
 * controller.
 *
 * \param move A valid \ref PSMove handle
 * \param out_w The output \ref PSMove_3AxisVector
 **/
ADDAPI void
ADDCALL psmove_get_transformed_gyroscope_frame_3axisvector(PSMove *move, enum PSMove_Frame frame, PSMove_3AxisVector *out_w);

/**
 * \brief Disconnect from the PS Move and release resources.
 *
 * This will disconnect from the controller and release any resources allocated
 * for handling the controller. Please note that this does not disconnect the
 * controller from the system (as the Bluetooth stack of the operating system
 * usually keeps the HID connection alive), but will rather disconnect the
 * application from the controller.
 *
 * To really disconnect the controller from your computer, you can either press
 * the PS button for ~ 10 seconds or use the Bluetooth application of your
 * operating system to disconnect the controller.
 *
 * \param move A valid \ref PSMove handle (which will be invalid after this call)
 **/
ADDAPI void
ADDCALL psmove_disconnect(PSMove *move);

/**
 * \brief Reinitialize the library.
 *
 * Required for detecting new and removed controllers (at least on Mac OS X).
 * Make sure to disconnect all controllers (using psmove_disconnect) before
 * calling this, otherwise it won't work.
 *
 * You do not need to call this function at application startup.
 *
 * \bug It should be possible to auto-detect newly-connected controllers
 *      without having to rely on this function.
 **/
ADDAPI void
ADDCALL psmove_reinit();

/**
 * \brief Get milliseconds since first library use.
 *
 * This function is used throughout the library to take care of timing and
 * measurement. It implements a cross-platform way of getting the current
 * time, relative to library use.
 *
 * \return Time (in ms) since first library use.
 **/
ADDAPI long
ADDCALL psmove_util_get_ticks();

/**
 * \brief Get local save directory for settings.
 *
 * The local save directory is a PS Move API-specific directory where the
 * library and its components will store files such as calibration data,
 * tracker state and configuration files.
 *
 * \return The local save directory for settings.
 *         The returned value is reserved in static memory - it must not be freed!
 **/
ADDAPI const char *
ADDCALL psmove_util_get_data_dir();

/**
 * \brief Get a filename path in the local save directory.
 *
 * This is a convenience function wrapping psmove_util_get_data_dir()
 * and will give the absolute path of the given filename.
 *
 * The data directory will be created in case it doesn't exist yet.
 *
 * \param filename The basename of the file (e.g. \c myfile.txt)
 *
 * \return The absolute filename to the file. The caller must free
 *         the result using \ref psmove_free_mem() when it is not
 *         needed anymore.
 * \return On error, \c NULL is returned.
 **/
ADDAPI char *
ADDCALL psmove_util_get_file_path(const char *filename);

/**
 * \brief Get a filename path in the system save directory.
 *
 * This is a convenience function, which gives the absolute path for
 * a file stored in system-wide data directory.
 *
 * The data directory will NOT be created in case it doesn't exist yet.
 *
 * \param filename The basename of the file (e.g. \c myfile.txt)
 *
 * \return The absolute filename to the file. The caller must free
 *         the result using \ref psmove_free_mem() when it is not
 *         needed anymore.
 * \return On error, \c NULL is returned.
 **/
ADDAPI char *
ADDCALL psmove_util_get_system_file_path(const char *filename);

/**
 * \brief Get an integer from an environment variable
 *
 * Utility function used to get configuration from environment
 * variables.
 *
 * \param name The name of the environment variable
 *
 * \return The integer value of the environment variable, or -1 if
 *         the variable is not set or could not be parsed as integer.
 **/
ADDAPI int
ADDCALL psmove_util_get_env_int(const char *name);

/**
 * \brief Get a string from an environment variable
 *
 * Utility function used to get configuration from environment
 * variables.
 *
 * \param name The name of the environment variable
 *
 * \return The string value of the environment variable, or NULL if the
 *         variable is not set. The caller must free the result using
 *         \ref psmove_free_mem() when it is not needed anymore.
 **/
ADDAPI char *
ADDCALL psmove_util_get_env_string(const char *name);

/**
 * \brief Sleep for a specific amount of milliseconds.
 *
 * \param ms The amount of milliseconds to sleep
 **/
ADDAPI void
ADDCALL psmove_util_sleep_ms(uint32_t ms);

/**
 * \brief Free memory allocated by psmoveapi
 *
 * \param buf Pointer to the memory that was previously allocated and
 *            returned by the library
 **/
ADDAPI void
ADDCALL psmove_free_mem(char *buf);


#ifdef __cplusplus
}
#endif
