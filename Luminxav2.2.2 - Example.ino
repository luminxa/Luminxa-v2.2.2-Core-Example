#include <WiFi.h>
#include <Wire.h>
#include "axp20x.h"
#include <WS2812FX.h>

#include "MPU6050.h"

#define AXP192_SLAVE_ADDRESS    0x34

#define I2C_SDA             21
#define I2C_SCL             22
#define PMU_IRQ             35
#define WS2812_PIN          33
#define WS2812_PIN1         32

#define LED_COUNT           12

AXP20X_Class axp;
WS2812FX ws2812fx = WS2812FX(LED_COUNT, WS2812_PIN, NEO_RGB + NEO_KHZ800);
WS2812FX ws2812fx1 = WS2812FX(LED_COUNT, WS2812_PIN1, NEO_RGB + NEO_KHZ800);

bool axp192_found = 0;
bool pmu_irq = 0;

int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;
float aRes, gRes; // scale resolutions per LSB for the sensors
MPU6050lib mpu;



void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t b = readByte(devAddr, regAddr);
    printf("rb:%x\n", b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    printf("wb:%x\n", b);
    writeByte(devAddr, regAddr, b);
}

#define PWR1_SLEEP_BIT     6
void setup()
{
    Serial.begin(115200);

    delay(1000);

    Wire.begin(I2C_SDA, I2C_SCL);

    scanI2Cdevice();

    uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    Serial.print("I AM ");
    Serial.print(c, HEX);
    Serial.print(" I Should Be ");
    Serial.println(0x70, HEX);

    if (c == 0x70) { // WHO_AM_I should always be 0x68
        Serial.println("MPU6050 is online...");

        mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
        Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");

        if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            Serial.println("Pass Selftest!");

            mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            mpu.initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

        } else {
            Serial.print("Could not connect to MPU6050: 0x");
            Serial.println(c, HEX);
        }
    }


    ws2812fx.init();
    ws2812fx.setBrightness(255);
    ws2812fx.setSpeed(1000);
    ws2812fx.setColor(0x007BFF);
    ws2812fx.setMode(FX_MODE_STATIC);
    ws2812fx.start();

    ws2812fx1.init();
    ws2812fx1.setBrightness(255);
    ws2812fx1.setSpeed(1000);
    ws2812fx1.setColor(0x007BFF);
    ws2812fx1.setMode(FX_MODE_STATIC);
    ws2812fx1.start();



    axp192_found = 1;
    if (axp192_found) {
        if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
            Serial.println("AXP192 Begin PASS");
        } else {
            Serial.println("AXP192 Begin FAIL");
        }

        // axp.setChgLEDMode(LED_BLINK_4HZ);

        Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

        Serial.println("----------------------------------------");

        // axp.setDCDC1Voltage(3300);
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
        axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
        axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

        Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
            pmu_irq = true;
        }, FALLING);

        axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
        axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
        axp.clearIRQ();

        if (axp.isChargeing()) {
            Serial.println("Charging");
        }
    } else {
        Serial.println("AXP192 not found");
    }
}

unsigned long last_change = 0;
unsigned long now = 0;
unsigned long timestamp = 0;

void loop()
{
    now = millis();

    if (now - timestamp > 10 * 1000) {
        writeBit(MPU6050_ADDRESS, PWR_MGMT_1, PWR1_SLEEP_BIT, 1);
        ws2812fx.strip_off();
        ws2812fx1.strip_off();
        ws2812fx.stop();
        ws2812fx1.stop();
        delay(500);
        esp_sleep_enable_timer_wakeup(10 * 1000000);
        esp_deep_sleep_start();
    }

    ws2812fx1.service();

    ws2812fx.service();
    if (now - last_change > 10000) {
        ws2812fx.setMode((ws2812fx.getMode() + 1) % ws2812fx.getModeCount());
        ws2812fx1.setMode((ws2812fx1.getMode() + 1) % ws2812fx1.getModeCount());
        last_change = now;
    }

    if (axp192_found && pmu_irq) {
        pmu_irq = false;
        axp.readIRQ();
        if (axp.isChargingIRQ()) {
            Serial.println("Charging");
        } else {
            Serial.println("No Charging");
        }
        if (axp.isVbusRemoveIRQ()) {
            Serial.println("No Charging");
        }
        axp.clearIRQ();
    }

}

void scanI2Cdevice(void)
{
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == AXP192_SLAVE_ADDRESS) {
                axp192_found = true;
                Serial.println("axp192 PMU found");
            }
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}
