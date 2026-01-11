// SBUS Configuration
// SBUS uses hardware Serial (D0=RX, D1=TX) - pins are fixed
const int SBUS_DEADBAND = 10;  // ±10% deadband for stick centering

// Maximum motor duty cycle [0 - 255]
uint8_t maxSpeed = 255;

// motor flipping
bool flipM1 = false;
bool flipM2 = false;
bool flipM3 = false;
bool flipM4 = true;
