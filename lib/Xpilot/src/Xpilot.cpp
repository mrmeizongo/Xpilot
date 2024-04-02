#include "Xpilot.h"
#include "mode.h"
#include <utils.cpp>

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral.
// With the MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define Kp 25.0
#define Ki 0.0

// Globals for AHRS loop timing
unsigned long now_us = 0, last_us = 0; // micros() timers
float dt = 0;                          // loop time in seconds
// -------------------------

// Timer variables
unsigned long nowMs, inputLastMs, debugLastMs = 0;
// -------------------------

#if LOOP_DEBUG
unsigned long lastMs = 0;
#endif
// -------------------------

// Aileron variables
volatile long aileronCurrentTime,
    aileronStartTime, aileronPulses = 0;
void aileronInterrupt(void);

// Elevator variables
volatile long elevatorCurrentTIme, elevatorStartTime, elevatorPulses = 0;
void elevatorInterrupt(void);
// -------------------------

Xpilot::Xpilot(void)
{
    aileronPinInt = AILPIN_INT;
    elevatorPinInt = ELEVPIN_INT;
}

void Xpilot::setup(void)
{
#if !defined(AILPIN_INT) || !defined(ELEVPIN_INT) || !defined(MODEPIN_INT)
#error "Ensure interrupt pins are defined"
#endif

    Wire.begin();
#if DEBUG
    Serial.begin(9600);
    while (!Serial)
        ;
#endif

    // initialize device
    imu.initialize();
    // test mpu for connection after initialization
    while (!imu.testConnection())
    {
#if IO_DEBUG
        Serial.println("No MPU found! Check connection");
#endif
        delay(1000);
    }

    // Both ailerons and elevators use hardware interrupts for accuracy and also due to pin limitations on the atmega328p chip
    // Aileron setup
    aileronServo.attach(AILPIN_OUT);
    pinMode(aileronPinInt, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(aileronPinInt), aileronInterrupt, CHANGE);

    // Elevator setup
    elevatorServo.attach(ELEVPIN_OUT);
    pinMode(elevatorPinInt, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(elevatorPinInt), elevatorInterrupt, CHANGE);

    mode.init();
}

// Main execution xpilot execution loop
void Xpilot::loop(void)
{
    // Read input, read imu data, process output to servos
    // Process input at 50Hz, everything else runs at loop speed
    nowMs = millis();
    if (nowMs - inputLastMs >= 20)
    {
        processInput();
        inputLastMs = nowMs;
    }

    if (currentMode != FLIGHT_MODE::MANUAL)
        processIMU();

    processOutput();

#if IO_DEBUG
    if (nowMs - debugLastMs >= 1000)
    {
        print_imu();
        print_input();
        print_output();
        debugLastMs = nowMs;
    }
#endif

#if LOOP_DEBUG
    lastMs = millis();
    Serial.print("Loop time: ");
    Serial.println(lastMs - nowMs / 1000);
#endif
}

void Xpilot::processInput(void)
{
    noInterrupts();
    if (aileronPulses >= RECEIVER_LOW && aileronPulses <= RECEIVER_HIGH)
        aileronPulseWidth = aileronPulses;

    if (elevatorPulses >= RECEIVER_LOW && elevatorPulses <= RECEIVER_HIGH)
        elevatorPulseWidth = elevatorPulses;

    mode.update();
    interrupts();
}

void Xpilot::processIMU(void)
{
    get_imu_scaled();
    now_us = micros();
    dt = (now_us - last_us) * 1e-6; // seconds since last update
    last_us = now_us;

    //  Standard orientation: X North, Y West, Z Up
    //  Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    mahoganyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[1], Mxyz[0], -Mxyz[2], dt);

    // Strictly valid only for approximately level movement
    // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
    // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
    ahrs_roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    ahrs_pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    ahrs_yaw = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
    // to degrees
    ahrs_yaw *= 180.0 / PI;
    ahrs_pitch *= 180.0 / PI;
    ahrs_roll *= 180.0 / PI;

    // http://www.ngdc.noaa.gov/geomag-web/#declination
    // conventional nav, yaw increases CW from North, corrected for local magnetic declination
    ahrs_yaw = -ahrs_yaw + 11.5;

    if (ahrs_yaw < 0)
        ahrs_yaw += 360.0;
    if (ahrs_yaw > 360.0)
        ahrs_yaw -= 360.0;
}

void Xpilot::processOutput(void)
{
    aileron_out = map8(aileronPulseWidth, RECEIVER_LOW, RECEIVER_HIGH, rollDeflectionLim, 180 - rollDeflectionLim);
    elevator_out = map8(elevatorPulseWidth, RECEIVER_LOW, RECEIVER_HIGH, pitchDeflectionLim, 180 - pitchDeflectionLim);

    mode.process();

    aileron_out = constrain(aileron_out, rollDeflectionLim, 180 - rollDeflectionLim);
    elevator_out = constrain(elevator_out, pitchDeflectionLim, 180 - pitchDeflectionLim);

    aileronServo.write(aileron_out);
    elevatorServo.write(elevator_out);
}

void Xpilot::get_imu_scaled(void)
{
    float temp[3];
    int i;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // 250 LSB(d/s) default to radians/s
    Gxyz[0] = ((float)gx - G_off[0]) * gscale;
    Gxyz[1] = ((float)gy - G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - G_off[2]) * gscale;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    // apply offsets (bias) and scale factors from Magneto
    for (i = 0; i < 3; i++)
        temp[i] = (Axyz[i] - A_B[i]);

    Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
    Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
    Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
    vector_normalize(Axyz);

    Mxyz[0] = (float)mx;
    Mxyz[1] = (float)my;
    Mxyz[2] = (float)mz;

    // apply offsets and scale factors from Magneto
    for (i = 0; i < 3; i++)
        temp[i] = (Mxyz[i] - M_B[i]);
    Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
    Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
    Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
    vector_normalize(Mxyz);
}

// Mahogany scheme uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void Xpilot::mahoganyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // Vector to hold integral error for Mahony method
    static float eInt[3] = {0.0, 0.0, 0.0};
    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of the reference vectors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex; // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
        // Apply I feedback
        gx += Ki * eInt[0];
        gy += Ki * eInt[1];
        gz += Ki * eInt[2];
    }

    // Apply P feedback
    gx = gx + Kp * ex;
    gy = gy + Kp * ey;
    gz = gz + Kp * ez;

    // Integrate rate of change of quaternion
    // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
    gx = gx * (0.5 * deltat); // pre-multiply common factors
    gy = gy * (0.5 * deltat);
    gz = gz * (0.5 * deltat);
    float qa = q1;
    float qb = q2;
    float qc = q3;
    q1 += (-qb * gx - qc * gy - q4 * gz);
    q2 += (qa * gx + qc * gz - q4 * gy);
    q3 += (qa * gy - qb * gz + q4 * gx);
    q4 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

// Interrupt functions
void aileronInterrupt(void)
{
    aileronCurrentTime = micros();
    if (aileronCurrentTime > aileronStartTime)
    {
        aileronPulses = aileronCurrentTime - aileronStartTime;
        aileronStartTime = aileronCurrentTime;
    }
}

void elevatorInterrupt(void)
{
    elevatorCurrentTIme = micros();
    if (elevatorCurrentTIme > elevatorStartTime)
    {
        elevatorPulses = elevatorCurrentTIme - elevatorStartTime;
        elevatorStartTime = elevatorCurrentTIme;
    }
}
// ----------------------------

// IO Debug functions
#if IO_DEBUG
void Xpilot::print_imu(void)
{
    Serial.print("Yaw: ");
    Serial.println(ahrs_yaw);
    Serial.print("Roll: ");
    Serial.println(ahrs_roll);
    Serial.print("Pitch: ");
    Serial.println(ahrs_pitch);
    Serial.println();
}

void Xpilot::print_input(void)
{
    Serial.print("Elevator Pulse: ");
    Serial.println(elevatorPulseWidth);
    Serial.print("Aileron Pulse: ");
    Serial.println(aileronPulseWidth);
    Serial.print("Flight Mode: ");
    Serial.println((int)currentMode);
    Serial.println();
}

void Xpilot::print_output(void)
{
    Serial.print("Elevator: ");
    Serial.println(elevator_out);
    Serial.print("Aileron: ");
    Serial.println(aileron_out);
    Serial.println();
}
#endif
// ---------------------------

Xpilot xpilot;