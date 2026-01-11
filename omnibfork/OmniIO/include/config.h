// SBUS Configuration
const int SBUS_RX_PIN = 12;
const int SBUS_TX_PIN = 13;  // Not used, but required for SoftwareSerial
const int SBUS_DEADBAND = 10;  // ±10% deadband for stick centering

// Maximum motor duty cycle [0 - 255]
uint8_t maxSpeed = 255;

// motor flipping
bool flipM1 = false;
bool flipM2 = false;
bool flipM3 = false;
bool flipM4 = true;
