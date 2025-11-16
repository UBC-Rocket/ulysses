#include <SPI.h>

// -------- Pin Definitions --------
#define CS_ACC 10
#define CS_GYR 9

// -------- Register banks --------
uint8_t acc_reg[256];
uint8_t gyr_reg[256];

volatile uint8_t currentReg;
volatile bool reading;
volatile bool isAcc = false;   // which device was selected

// -------- Synthetic motion timer --------
unsigned long lastT = 0;
const unsigned long DT_US = 1000; // 1 kHz

// -------- SPI ISR --------
ISR(SPI_STC_vect) {
    uint8_t in = SPDR;

    // Check which CS is low
    if (digitalRead(CS_ACC) == LOW) {
        isAcc = true;
    } else if (digitalRead(CS_GYR) == LOW) {
        isAcc = false;
    }

    uint8_t* bank = isAcc ? acc_reg : gyr_reg;

    // First byte = register address with R/W flag
    if (!reading) {
        uint8_t addr = in & 0x7F;  // clear MSB
        currentReg = addr;
        reading = true;

        // For read, output first byte immediately
        if (in & 0x80)
            SPDR = bank[currentReg++];
        else
            SPDR = 0x00; // write not implemented
        return;
    }

    // Subsequent bytes = sequential register reads
    SPDR = bank[currentReg++];
}

// -------- Fill synthetic IMU data --------
void update_fake_sensor() {
    static float t = 0.0f;
    t += 0.001;

    // Example synthetic signals
    float ax = 0.1 * sin(2 * PI * 0.4 * t);
    float ay = 0.05 * sin(2 * PI * 1.0 * t);
    float az = 1.0 + 0.02 * sin(2 * PI * 0.3 * t);

    float gx = 20.0 * sin(2 * PI * 0.7 * t);
    float gy = 10.0 * sin(2 * PI * 0.5 * t);
    float gz = 5.0  * sin(2 * PI * 0.2 * t);

    int16_t ax_r = ax * 16384;
    int16_t ay_r = ay * 16384;
    int16_t az_r = az * 16384;

    int16_t gx_r = gx * 131;
    int16_t gy_r = gy * 131;
    int16_t gz_r = gz * 131;

    // accel little-endian
    acc_reg[0x02] = ax_r & 0xFF;
    acc_reg[0x03] = ax_r >> 8;
    acc_reg[0x04] = ay_r & 0xFF;
    acc_reg[0x05] = ay_r >> 8;
    acc_reg[0x06] = az_r & 0xFF;
    acc_reg[0x07] = az_r >> 8;

    // gyro little-endian
    gyr_reg[0x02] = gx_r & 0xFF;
    gyr_reg[0x03] = gx_r >> 8;
    gyr_reg[0x04] = gy_r & 0xFF;
    gyr_reg[0x05] = gy_r >> 8;
    gyr_reg[0x06] = gz_r & 0xFF;
    gyr_reg[0x07] = gz_r >> 8;
}

void setup() {
    pinMode(CS_ACC, INPUT_PULLUP);
    pinMode(CS_GYR, INPUT_PULLUP);

    pinMode(MISO, OUTPUT);

    Serial.begin(115200);

    // Load WHO_AM_I values
    acc_reg[0x00] = 0x1E;  // BMI088 accel
    gyr_reg[0x00] = 0x0F;  // BMI088 gyro

    // Start SPI slave
    SPCR |= _BV(SPE);
    SPCR |= _BV(SPIE);
    SPI.attachInterrupt();

    lastT = micros();

}

void loop() {
    unsigned long now = micros();
    if (now - lastT >= DT_US) {
        lastT += DT_US;
        update_fake_sensor();
    }

    // Reset state when CS rises
    if (digitalRead(CS_ACC) == HIGH && digitalRead(CS_GYR) == HIGH) {
        reading = false;
    }

    // int64_t test = 105563;
    // int64_t test_lower = test & 0xFF;
    // int64_t test_upper = test >> 8;
    // float acc_x = (test_upper << 8) | test_lower;
    // Serial.println(acc_x);
}