#include <Arduino.h>
#include <Wire.h>

// ============================================================
// MICA FLIGHT TEST FIRMWARE
// ============================================================
//
// Target: Seeed XIAO ESP32-S3
// Framework: Arduino-ESP32
//
// Motors MUST be physically disconnected during initial
// firmware/receiver testing.
// ============================================================


// ============================================================
// CONFIGURATION
// ============================================================

namespace Config {

    // ---------- I2C ----------
    constexpr int I2C_SDA = -1;       // TODO
    constexpr int I2C_SCL = -1;       // TODO

    // ---------- ELRS / CRSF ----------
    //
    // CRSF normally runs at 420000 baud, 8N1.
    //
    constexpr uint32_t CRSF_BAUD = 420000;

    constexpr int CRSF_RX_PIN = -1;   // TODO
    constexpr int CRSF_TX_PIN = -1;   // Usually unused

    // ---------- Motors ----------
    //
    // Quad X:
    //
    //       FRONT
    //
    //       M1      M2
    //        \      /
    //         \    /
    //          \  /
    //          /  \
    //         /    \
    //        /      \
    //       M3      M4
    //
    constexpr int MOTOR_1_PIN = -1;   // TODO
    constexpr int MOTOR_2_PIN = -1;   // TODO
    constexpr int MOTOR_3_PIN = -1;   // TODO
    constexpr int MOTOR_4_PIN = -1;   // TODO

    // PWM configuration
    constexpr uint32_t MOTOR_PWM_FREQ = 400;
    constexpr uint8_t MOTOR_PWM_BITS = 11;

    // ESC pulse range.
    //
    // If using standard PWM ESCs:
    //   1000 us = minimum
    //   2000 us = maximum
    //
    constexpr uint16_t MOTOR_MIN_US = 1000;
    constexpr uint16_t MOTOR_IDLE_US = 1050;
    constexpr uint16_t MOTOR_MAX_US = 2000;

    // ---------- Control ----------
    constexpr float LOOP_HZ = 500.0f;

    // Maximum commanded angular rates.
    constexpr float MAX_ROLL_RATE  = 360.0f;  // deg/s
    constexpr float MAX_PITCH_RATE = 360.0f;
    constexpr float MAX_YAW_RATE   = 180.0f;

    // Receiver channel mapping.
    //
    // Typical CRSF:
    // CH1 = Roll
    // CH2 = Pitch
    // CH3 = Throttle
    // CH4 = Yaw
    //
    constexpr int CH_ROLL     = 0;
    constexpr int CH_PITCH    = 1;
    constexpr int CH_THROTTLE = 2;
    constexpr int CH_YAW      = 3;
    constexpr int CH_ARM      = 4;

    // Receiver timeout.
    constexpr uint32_t RC_TIMEOUT_MS = 100;

    // Do not arm below this throttle.
    constexpr float ARM_THROTTLE_LIMIT = 0.05f;

    // ---------- PID ----------
    //
    // These are starting values only.
    // They MUST be tuned for the actual frame, motors,
    // propellers and IMU.
    //
    constexpr float ROLL_KP  = 0.8f;
    constexpr float ROLL_KI  = 0.15f;
    constexpr float ROLL_KD  = 0.015f;

    constexpr float PITCH_KP = 0.8f;
    constexpr float PITCH_KI = 0.15f;
    constexpr float PITCH_KD = 0.015f;

    constexpr float YAW_KP   = 0.8f;
    constexpr float YAW_KI   = 0.10f;
    constexpr float YAW_KD   = 0.0f;

    constexpr float PID_OUTPUT_LIMIT = 400.0f;

    // Integral limit prevents runaway accumulation.
    constexpr float PID_INTEGRAL_LIMIT = 100.0f;

    // ---------- Safety ----------
    constexpr bool REQUIRE_ARM_SWITCH = true;

    // Set true only after motor/ESC output has been verified.
    constexpr bool ENABLE_MOTORS = false;

}


// ============================================================
// BASIC UTILITIES
// ============================================================

float clampf(float x, float minValue, float maxValue)
{
    if (x < minValue) return minValue;
    if (x > maxValue) return maxValue;
    return x;
}

float mapFloat(
    float x,
    float inMin,
    float inMax,
    float outMin,
    float outMax
)
{
    return (x - inMin) *
           (outMax - outMin) /
           (inMax - inMin) +
           outMin;
}


// ============================================================
// VECTOR TYPES
// ============================================================

struct Vec3 {
    float x;
    float y;
    float z;
};

struct AttitudeRates {
    float roll;
    float pitch;
    float yaw;
};


// ============================================================
// IMU INTERFACE
// ============================================================
//
// Replace this implementation with the driver for the actual
// IMU on the mica PCB.
//
// The flight controller only needs gyro measurements in deg/s.
// ============================================================

class IMU
{
public:

    bool begin()
    {
        if (Config::I2C_SDA < 0 ||
            Config::I2C_SCL < 0) {

            Serial.println(
                "IMU: I2C pins not configured"
            );

            return false;
        }

        Wire.begin(
            Config::I2C_SDA,
            Config::I2C_SCL
        );

        // TODO:
        //
        // Initialize the actual IMU here.
        //
        // Example:
        //
        // imu.begin();
        // imu.setGyroRange(...);
        // imu.setGyroODR(...);

        Serial.println(
            "IMU: driver placeholder active"
        );

        return true;
    }


    bool readGyro(Vec3 &gyro)
    {
        // TODO:
        //
        // Replace with actual IMU read.
        //
        // gyro.x = ...
        // gyro.y = ...
        // gyro.z = ...

        gyro.x = 0.0f;
        gyro.y = 0.0f;
        gyro.z = 0.0f;

        return true;
    }


    bool calibrateGyro()
    {
        Serial.println(
            "IMU: keep aircraft completely still"
        );

        delay(1000);

        constexpr int SAMPLES = 1000;

        Vec3 sample;
        Vec3 sum = {0, 0, 0};

        for (int i = 0; i < SAMPLES; ++i) {

            if (!readGyro(sample)) {
                return false;
            }

            sum.x += sample.x;
            sum.y += sample.y;
            sum.z += sample.z;

            delay(2);
        }

        gyroBias.x = sum.x / SAMPLES;
        gyroBias.y = sum.y / SAMPLES;
        gyroBias.z = sum.z / SAMPLES;

        Serial.printf(
            "Gyro bias: %.3f %.3f %.3f\n",
            gyroBias.x,
            gyroBias.y,
            gyroBias.z
        );

        return true;
    }


    Vec3 getCorrectedGyro()
    {
        Vec3 raw;

        if (!readGyro(raw)) {
            return {0, 0, 0};
        }

        return {
            raw.x - gyroBias.x,
            raw.y - gyroBias.y,
            raw.z - gyroBias.z
        };
    }


private:

    Vec3 gyroBias = {0, 0, 0};
};


// ============================================================
// CRSF / ELRS RECEIVER
// ============================================================

class CRSF
{
public:

    bool begin()
    {
        if (Config::CRSF_RX_PIN < 0) {

            Serial.println(
                "CRSF: RX pin not configured"
            );

            return false;
        }

        Serial1.begin(
            Config::CRSF_BAUD,
            SERIAL_8N1,
            Config::CRSF_RX_PIN,
            Config::CRSF_TX_PIN
        );

        lastPacket = millis();

        Serial.println("CRSF: initialized");

        return true;
    }


    void update()
    {
        while (Serial1.available()) {

            uint8_t byte = Serial1.read();

            parseByte(byte);
        }
    }


    bool connected() const
    {
        return
            millis() - lastPacket <
            Config::RC_TIMEOUT_MS;
    }


    float channel(int index) const
    {
        if (index < 0 || index >= 16) {
            return 0;
        }

        return channels[index];
    }


private:

    uint8_t packet[64];
    uint8_t packetIndex = 0;
    uint8_t packetLength = 0;

    float channels[16] = {0};

    uint32_t lastPacket = 0;


    void parseByte(uint8_t byte)
    {
        // CRSF packets:
        //
        // [address]
        // [length]
        // [type]
        // [payload]
        // [CRC]

        if (packetIndex == 0) {

            // Typical CRSF device addresses.
            if (byte != 0xC8 &&
                byte != 0xEA &&
                byte != 0xEC &&
                byte != 0xEE) {
                return;
            }

            packet[packetIndex++] = byte;
            return;
        }


        if (packetIndex == 1) {

            packetLength = byte;

            if (packetLength < 2 ||
                packetLength > 62) {

                packetIndex = 0;
                return;
            }

            packet[packetIndex++] = byte;
            return;
        }


        packet[packetIndex++] = byte;


        if (packetIndex >= packetLength + 2) {

            processPacket();

            packetIndex = 0;
        }
    }


    void processPacket()
    {
        uint8_t type = packet[2];

        // CRSF packed RC channels
        constexpr uint8_t TYPE_RC_CHANNELS = 0x16;

        if (type != TYPE_RC_CHANNELS) {
            return;
        }

        // Need 16 channels * 11 bits = 176 bits = 22 bytes.
        if (packetLength < 24) {
            return;
        }

        uint32_t bits = 0;
        int bitCount = 0;
        int byteIndex = 3;

        for (int ch = 0; ch < 16; ch++) {

            uint16_t value = 0;

            for (int b = 0; b < 11; b++) {

                int bitPosition = bitCount + b;

                int sourceByte =
                    byteIndex + bitPosition / 8;

                int sourceBit =
                    bitPosition % 8;

                if (packet[sourceByte] &
                    (1 << sourceBit)) {

                    value |= (1 << b);
                }
            }

            bitCount += 11;

            //
            // CRSF channel range is approximately
            // 172 ... 1811.
            //
            channels[ch] = mapFloat(
                value,
                172,
                1811,
                -1.0f,
                1.0f
            );
        }

        // Throttle is special:
        //
        // Map CH3 from approximately
        // 172 ... 1811 to 0 ... 1.
        channels[Config::CH_THROTTLE] =
            clampf(
                mapFloat(
                    getRawChannel(
                        Config::CH_THROTTLE
                    ),
                    172,
                    1811,
                    0,
                    1
                ),
                0,
                1
            );

        lastPacket = millis();
    }


    float getRawChannel(int index) const
    {
        //
        // This implementation stores normalized values.
        // For a production implementation, retain raw
        // 11-bit values separately.
        //
        return mapFloat(
            channels[index],
            -1,
            1,
            172,
            1811
        );
    }
};


// ============================================================
// PID CONTROLLER
// ============================================================

class PID
{
public:

    PID(
        float kp,
        float ki,
        float kd
    )
        : kp(kp),
          ki(ki),
          kd(kd)
    {}


    float update(
        float target,
        float measurement,
        float dt
    )
    {
        float error =
            target - measurement;

        integral +=
            error * dt;

        integral = clampf(
            integral,
            -Config::PID_INTEGRAL_LIMIT,
            Config::PID_INTEGRAL_LIMIT
        );

        float derivative = 0;

        if (initialized) {

            derivative =
                (error - previousError) / dt;
        }
        else {

            initialized = true;
        }

        previousError = error;

        float output =
            kp * error +
            ki * integral +
            kd * derivative;

        return clampf(
            output,
            -Config::PID_OUTPUT_LIMIT,
            Config::PID_OUTPUT_LIMIT
        );
    }


    void reset()
    {
        integral = 0;
        previousError = 0;
        initialized = false;
    }


private:

    float kp;
    float ki;
    float kd;

    float integral = 0;
    float previousError = 0;

    bool initialized = false;
};


// ============================================================
// MOTOR OUTPUT
// ============================================================

class Motors
{
public:

    void begin()
    {
        pinMode(Config::MOTOR_1_PIN, OUTPUT);
        pinMode(Config::MOTOR_2_PIN, OUTPUT);
        pinMode(Config::MOTOR_3_PIN, OUTPUT);
        pinMode(Config::MOTOR_4_PIN, OUTPUT);

        //
        // Actual LEDC setup should be adapted to the installed
        // Arduino-ESP32 version.
        //
        // For initial development, keep motor output disabled.
        //

        stop();
    }


    void writeMicroseconds(
        uint16_t m1,
        uint16_t m2,
        uint16_t m3,
        uint16_t m4
    )
    {
        if (!Config::ENABLE_MOTORS) {
            return;
        }

        m1 = constrain(
            m1,
            Config::MOTOR_MIN_US,
            Config::MOTOR_MAX_US
        );

        m2 = constrain(
            m2,
            Config::MOTOR_MIN_US,
            Config::MOTOR_MAX_US
        );

        m3 = constrain(
            m3,
            Config::MOTOR_MIN_US,
            Config::MOTOR_MAX_US
        );

        m4 = constrain(
            m4,
            Config::MOTOR_MIN_US,
            Config::MOTOR_MAX_US
        );

        //
        // TODO:
        // Convert microseconds into the PWM representation
        // required by your ESC protocol.
        //
    }


    void stop()
    {
        //
        // TODO:
        // Send the ESC's minimum/disarmed signal.
        //
    }
};


// ============================================================
// FLIGHT CONTROLLER
// ============================================================

class FlightController
{
public:

    void begin()
    {
        rollPID = PID(
            Config::ROLL_KP,
            Config::ROLL_KI,
            Config::ROLL_KD
        );

        pitchPID = PID(
            Config::PITCH_KP,
            Config::PITCH_KI,
            Config::PITCH_KD
        );

        yawPID = PID(
            Config::YAW_KP,
            Config::YAW_KI,
            Config::YAW_KD
        );
    }


    void update(
        float dt,
        const Vec3 &gyro,
        const CRSF &receiver,
        Motors &motors
    )
    {
        bool receiverOK =
            receiver.connected();


        float throttle =
            receiver.channel(
                Config::CH_THROTTLE
            );

        float armChannel =
            receiver.channel(
                Config::CH_ARM
            );


        //
        // ARM CONDITIONS
        //

        bool armRequest =
            armChannel > 0.5f;

        bool throttleLow =
            throttle <
            Config::ARM_THROTTLE_LIMIT;


        if (!receiverOK) {

            armed = false;
        }


        if (Config::REQUIRE_ARM_SWITCH) {

            if (!armRequest) {
                armed = false;
            }
        }


        if (armRequest &&
            throttleLow &&
            receiverOK) {

            armed = true;
        }


        //
        // DISARMED
        //

        if (!armed) {

            rollPID.reset();
            pitchPID.reset();
            yawPID.reset();

            motors.stop();

            return;
        }


        //
        // RECEIVER COMMANDS
        //

        float rollCommand =
            receiver.channel(
                Config::CH_ROLL
            );

        float pitchCommand =
            receiver.channel(
                Config::CH_PITCH
            );

        float yawCommand =
            receiver.channel(
                Config::CH_YAW
            );


        float rollTarget =
            rollCommand *
            Config::MAX_ROLL_RATE;

        float pitchTarget =
            pitchCommand *
            Config::MAX_PITCH_RATE;

        float yawTarget =
            yawCommand *
            Config::MAX_YAW_RATE;


        //
        // RATE PID
        //

        float rollOutput =
            rollPID.update(
                rollTarget,
                gyro.x,
                dt
            );

        float pitchOutput =
            pitchPID.update(
                pitchTarget,
                gyro.y,
                dt
            );

        float yawOutput =
            yawPID.update(
                yawTarget,
                gyro.z,
                dt
            );


        //
        // THROTTLE
        //

        float baseThrottle =
            mapFloat(
                throttle,
                0.0f,
                1.0f,
                Config::MOTOR_IDLE_US,
                Config::MOTOR_MAX_US
            );


        //
        // QUAD-X MIXER
        //
        //
        // Verify this orientation against the actual
        // physical motor positions and rotation directions.
        //
        //
        // M1 = front-left
        // M2 = front-right
        // M3 = rear-right
        // M4 = rear-left
        //

        float m1 =
            baseThrottle
            + rollOutput
            + pitchOutput
            - yawOutput;

        float m2 =
            baseThrottle
            - rollOutput
            + pitchOutput
            + yawOutput;

        float m3 =
            baseThrottle
            - rollOutput
            - pitchOutput
            - yawOutput;

        float m4 =
            baseThrottle
            + rollOutput
            - pitchOutput
            + yawOutput;


        //
        // Prevent mixer saturation from producing values
        // outside the ESC range.
        //

        m1 = clampf(
            m1,
            Config::MOTOR_MIN_US,
            Config::MOTOR_MAX_US
        );

        m2 = clampf(
            m2,
            Config::MOTOR_MIN_US,
            Config::MOTOR_MAX_US
        );

        m3 = clampf(
            m3,
            Config::MOTOR_MIN_US,
            Config::MOTOR_MAX_US
        );

        m4 = clampf(
            m4,
            Config::MOTOR_MIN_US,
            Config::MOTOR_MAX_US
        );


        motors.writeMicroseconds(
            (uint16_t)m1,
            (uint16_t)m2,
            (uint16_t)m3,
            (uint16_t)m4
        );
    }


    bool isArmed() const
    {
        return armed;
    }


private:

    bool armed = false;

    PID rollPID{
        Config::ROLL_KP,
        Config::ROLL_KI,
        Config::ROLL_KD
    };

    PID pitchPID{
        Config::PITCH_KP,
        Config::PITCH_KI,
        Config::PITCH_KD
    };

    PID yawPID{
        Config::YAW_KP,
        Config::YAW_KI,
        Config::YAW_KD
    };
};


// ============================================================
// GLOBAL OBJECTS
// ============================================================

IMU imu;
CRSF receiver;
Motors motors;
FlightController flightController;


// ============================================================
// TIMING
// ============================================================

uint32_t lastLoopMicros = 0;
uint32_t lastDebugMillis = 0;


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("====================");
    Serial.println("       MICA FC      ");
    Serial.println("====================");


    //
    // HARDWARE CHECK
    //

    if (!imu.begin()) {

        Serial.println(
            "ERROR: IMU initialization failed"
        );
    }

    imu.calibrateGyro();


    if (!receiver.begin()) {

        Serial.println(
            "WARNING: CRSF not configured"
        );
    }


    motors.begin();

    flightController.begin();


    //
    // ALWAYS START DISARMED
    //

    motors.stop();


    lastLoopMicros =
        micros();


    Serial.println(
        "MICA firmware ready"
    );

    Serial.println(
        "Motors remain disabled until "
        "ENABLE_MOTORS is changed."
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    //
    // Receiver parsing should run continuously.
    //

    receiver.update();


    //
    // Fixed-rate control loop.
    //

    constexpr uint32_t LOOP_PERIOD_US =
        (uint32_t)(
            1000000.0f /
            Config::LOOP_HZ
        );


    uint32_t now =
        micros();


    if (
        (uint32_t)(
            now - lastLoopMicros
        ) < LOOP_PERIOD_US
    ) {
        return;
    }


    uint32_t elapsed =
        now - lastLoopMicros;

    lastLoopMicros = now;


    float dt =
        elapsed / 1000000.0f;


    //
    // Protect against timing glitches.
    //

    if (dt <= 0 ||
        dt > 0.02f) {

        dt =
            1.0f /
            Config::LOOP_HZ;
    }


    //
    // Read IMU.
    //

    Vec3 gyro =
        imu.getCorrectedGyro();


    //
    // Run flight controller.
    //

    flightController.update(
        dt,
        gyro,
        receiver,
        motors
    );


    //
    // DEBUG TELEMETRY
    //

    if (
        millis() -
        lastDebugMillis > 250
    ) {

        lastDebugMillis =
            millis();


        Serial.printf(
            "RC=%d ARM=%d "
            "T=%.2f "
            "GYRO=%.1f %.1f %.1f "
            "FC=%s\n",

            receiver.connected(),

            flightController.isArmed(),

            receiver.channel(
                Config::CH_THROTTLE
            ),

            gyro.x,
            gyro.y,
            gyro.z,

            flightController.isArmed()
                ? "ARMED"
                : "DISARMED"
        );
    }
}