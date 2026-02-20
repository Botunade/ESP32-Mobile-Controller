#ifndef CONFIG_H
#define CONFIG_H

// ========== FIRMWARE VERSION ==========
#define FIRMWARE_VERSION "0.3.0"

// ========== WIFI CONFIGURATION ==========
#define AP_SSID     "TankController-AP"
#define AP_PASSWORD "tankwater"

// ========== HARDWARE PIN CONFIGURATION ==========
// Ultrasonic sensor pins
#define ULTRASONIC_TRIG_PIN 5
#define ULTRASONIC_ECHO_PIN 18

// Relay output to pump (GPIO 16)
#define PUMP_RELAY_PIN 16

// Analog output pin (GPIO 26)
#define ANALOG_OUTPUT_PIN 26

// ========== TANK GEOMETRY ==========
#define TANK_HEIGHT_CM   200.0f  // 2 m tank
#define MIN_DISTANCE_CM  3.0f    // near full (sensor to water)
#define MAX_DISTANCE_CM  200.0f  // near empty

// ========== HYSTERESIS CONTROL LIMITS ==========
#define DEFAULT_SETPOINT    80.0f   // Stop pump at 80%
#define DEFAULT_LOWER_LIMIT 40.0f   // Start pump at 40%

// ========== CONTROL TIMING ==========
#define CONTROL_INTERVAL_MS 500      // Run logic every 500 ms

// ========== PID CONTROL ==========
#define PID_KP 2.0f
#define PID_KI 0.5f
#define PID_KD 0.1f

// DAC Output Limits (8-bit: 0-255)
// 0.66V corresponds to (0.66/3.3)*255 ~= 51
// 3.3V corresponds to 255
#define DAC_MIN_VAL 51
#define DAC_MAX_VAL 255

#endif // CONFIG_H
