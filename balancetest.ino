#include <SimpleFOC.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- Pin Definitions ---
// Left Motor (motor_L)
#define DRV_L_IN1 32
#define DRV_L_IN2 33
#define DRV_L_IN3 25
#define DRV_L_EN  26
#define ENC_L_CS  5

// Right Motor (motor_R)
#define DRV_R_IN1 27
#define DRV_R_IN2 14
#define DRV_R_IN3 12
#define DRV_R_EN  13
#define ENC_R_CS  4

// Shared SPI Pins
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23

// MPU-6050 I2C Pins
#define MPU_SDA 21
#define MPU_SCL 22

// --- Motor Parameters ---
#define MOTOR_PP 11
#define DRV_VOLTAGE 12.0f

// --- Sensor Objects ---
MagneticSensorSPI motor_sensor_L = MagneticSensorSPI(AS5047_SPI, ENC_L_CS);
MagneticSensorSPI motor_sensor_R = MagneticSensorSPI(AS5047_SPI, ENC_R_CS);
SPIClass sensorSPI = SPI;
Adafruit_MPU6050 mpu;

// --- Driver Objects ---
BLDCDriver3PWM driver_L = BLDCDriver3PWM(DRV_L_IN1, DRV_L_IN2, DRV_L_IN3, DRV_L_EN);
BLDCDriver3PWM driver_R = BLDCDriver3PWM(DRV_R_IN1, DRV_R_IN2, DRV_R_IN3, DRV_R_EN);

// --- Motor Objects ---
BLDCMotor motor_L = BLDCMotor(MOTOR_PP);
BLDCMotor motor_R = BLDCMotor(MOTOR_PP);

// --- MPU6050 Data & Fusion ---
float pitch = 0.0, roll = 0.0, yaw = 0.0; // Angles in degrees
unsigned long mpu_last_time = 0;
float gyro_x_cal = 0.0, gyro_y_cal = 0.0, gyro_z_cal = 0.0;
const int CALIBRATION_SAMPLES_MPU = 1500;
const float MPU_ALPHA = 0.98;
const float MPU_DT_FIXED = 0.004; // Adjusted based on typical MPU read + processing time

// --- Balancing PID Controller ---
float balance_pid_setpoint = 0.0;
float balance_pid_output = 0.0;
float balance_Kp = 0.8;
float balance_Ki = 0.2;
float balance_Kd = 0.5;
float balance_pid_error_sum = 0.0;
float balance_pid_last_error = 0.0;
unsigned long balance_pid_last_time = 0;

float MAX_MOTOR_VELOCITY_CMD = 20.0; // Radians per second
float MAX_PID_ERROR_SUM = 200.0;

// --- Safety ---
bool balancing_active = false;
const float MAX_TILT_ANGLE = 35.0; // Degrees

// --- Serial Communication & Telemetry ---
String serialBuffer = "";
bool commandReceived = false;
unsigned long last_telemetry_time = 0;
const int telemetry_interval_ms = 50;

// Function Declarations
void parseAndExecuteCommand(String cmd);
void sendBalanceTelemetry();
void printBalanceConfirmation(String msg);
void readAndUpdateMPU(float dt_val);
void calibrateMPUgyro();
void setupMotorInstance(BLDCMotor& mtr, MagneticSensorSPI& sensor, BLDCDriver3PWM& drv, const char* motor_name, SPIClass& spi_bus_ref, int cs_pin);
void runBalancingPID();
void applyMotorCommands(float command_L, float command_R);


//==============================================================================
void setup() {
    Serial.begin(115200);
    unsigned long t_start = millis();
    while (!Serial && (millis() - t_start < 3000));
    Serial.println("\n\n--- Self-Balancing Robot V1.1 (Corrected Defines) ---");

    Serial.print("Initializing SPI bus for motor encoders (SCK:");
    Serial.print(SPI_SCK); Serial.print(", MISO:"); Serial.print(SPI_MISO);
    Serial.print(", MOSI:"); Serial.print(SPI_MOSI); Serial.println(")...");
    sensorSPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.println("Motor Encoder SPI Bus Initialized.");

    Serial.println("Initializing MPU6050...");
    Wire.begin(MPU_SDA, MPU_SCL);
    if (!mpu.begin()) {
        Serial.println("!!! CRITICAL: MPU6050 not found. Halting. !!!");
        while (1) delay(100);
    }
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    Serial.println("MPU6050 Gyro calibration... Keep stationary!");
    calibrateMPUgyro();
    Serial.println("MPU6050 Gyro calibration done.");

    Serial.println("Initializing Left Motor...");
    setupMotorInstance(motor_L, motor_sensor_L, driver_L, "Left", sensorSPI, ENC_L_CS);
    Serial.println("Initializing Right Motor...");
    setupMotorInstance(motor_R, motor_sensor_R, driver_R, "Right", sensorSPI, ENC_R_CS);

    motor_L.controller = MotionControlType::velocity;
    motor_R.controller = MotionControlType::velocity;
    Serial.println("Motors set to VELOCITY control mode.");

    motor_L.disable();
    motor_R.disable();
    balancing_active = false;

    serialBuffer.reserve(100);
    mpu_last_time = micros();
    balance_pid_last_time = micros();

    Serial.println("\nSystem Ready. Balancing INACTIVE.");
    Serial.println("Send 'ENABLE' to attempt balancing.");
    Serial.println("Send 'DISABLE' to stop.");
    Serial.println("Send 'P=<val> I=<val> D=<val>' to tune PID (e.g., P=0.5 I=0.1 D=0.2)");
    Serial.println("Send 'SETPOINT=<val>' to adjust balance setpoint (e.g. SETPOINT=-1.5)");
    printBalanceConfirmation("Balancing Inactive");
}

void setupMotorInstance(BLDCMotor& mtr, MagneticSensorSPI& sensor, BLDCDriver3PWM& drv, const char* motor_name, SPIClass& spi_bus_ref, int cs_pin) {
    Serial.print("  Motor ["); Serial.print(motor_name); Serial.print("] CS: "); Serial.println(cs_pin);
    Serial.print("    Sensor init...");
    sensor.init(&spi_bus_ref);
    float angle = sensor.getAngle();
    Serial.print("OK. Initial angle: "); Serial.println(angle, 4);
    if (angle == 0.0f) Serial.println("    WARNING: Sensor angle 0.0 after init. Check hardware/magnet.");

    Serial.print("    Driver init...");
    drv.voltage_power_supply = DRV_VOLTAGE;
    drv.init();
    Serial.println("OK.");

    mtr.linkSensor(&sensor);
    mtr.linkDriver(&drv);

    mtr.PID_velocity.P = 0.2f;
    mtr.PID_velocity.I = 2.0f;
    mtr.PID_velocity.D = 0.0f;
    mtr.PID_velocity.output_ramp = 1000;
    mtr.LPF_velocity.Tf = 0.01f;

    mtr.voltage_limit = DRV_VOLTAGE * 0.8f;
    mtr.velocity_limit = MAX_MOTOR_VELOCITY_CMD * 1.5f;

    Serial.print("    Motor lib init...");
    mtr.init();
    Serial.println("OK.");

    Serial.print("    FOC init (motor may move if powered)...");
    mtr.initFOC();
    Serial.println("Attempted.");
    mtr.disable();
    Serial.print("    Motor ["); Serial.print(motor_name); Serial.println("] setup complete, disabled.");
}

//==============================================================================
void loop() {
    unsigned long current_micros_loop = micros();
    // float dt_actual = (current_micros_loop - mpu_last_time) / 1000000.0f; // Can use if dt_fixed is not desired
    mpu_last_time = current_micros_loop; // Update for next MPU dt calculation if using actual dt

    readAndUpdateMPU(MPU_DT_FIXED);

    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') { commandReceived = true; break; }
        else if (c >= 0) { serialBuffer += c; }
    }

    if (commandReceived) {
        serialBuffer.trim();
        if (serialBuffer.length() > 0) { parseAndExecuteCommand(serialBuffer); }
        serialBuffer = ""; commandReceived = false;
    }

    if (balancing_active) {
        if (abs(pitch) > MAX_TILT_ANGLE) {
            Serial.println("!!! MAX TILT REACHED - DISABLING MOTORS !!!");
            motor_L.disable();
            motor_R.disable();
            balancing_active = false;
            balance_pid_output = 0;
            printBalanceConfirmation("Balancing Disabled - Max Tilt");
        } else {
            runBalancingPID();
            // Assuming positive PID output means correct forward lean -> wheels spin to move robot base forward.
            // This often means a negative velocity command for SimpleFOC if motors are wired identically.
            // If motors are mounted opposite, one might be positive, one negative.
            // THIS IS A CRITICAL TUNING POINT:
            applyMotorCommands(-balance_pid_output, -balance_pid_output); // Example: both motors get inverted PID
            // OR: applyMotorCommands(-balance_pid_output, balance_pid_output); // If one motor needs opposite command
        }
    } else {
        applyMotorCommands(0, 0);
        motor_L.disable(); // Ensure disabled
        motor_R.disable(); // Ensure disabled
    }

    motor_L.loopFOC();
    motor_R.loopFOC();

    if (millis() - last_telemetry_time > telemetry_interval_ms) {
        last_telemetry_time = millis();
        sendBalanceTelemetry();
    }
}

// --- MPU6050 ---
void calibrateMPUgyro() {
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    sensors_event_t a, g, temp_event;
    Serial.print("  Gyro calibration running for samples: ");
    for (int i = 0; i < CALIBRATION_SAMPLES_MPU; i++) {
        if (i > 0 && i % (CALIBRATION_SAMPLES_MPU/10) == 0) Serial.print(".");
        mpu.getEvent(&a, &g, &temp_event);
        gx_sum += g.gyro.x; gy_sum += g.gyro.y; gz_sum += g.gyro.z;
        delay(3);
    }
    Serial.println(" done.");
    gyro_x_cal = (float)gx_sum / CALIBRATION_SAMPLES_MPU;
    gyro_y_cal = (float)gy_sum / CALIBRATION_SAMPLES_MPU;
    gyro_z_cal = (float)gz_sum / CALIBRATION_SAMPLES_MPU;
    Serial.print("  Gyro X offset (rad/s): "); Serial.println(gyro_x_cal, 6);
    Serial.print("  Gyro Y offset (rad/s): "); Serial.println(gyro_y_cal, 6);
    Serial.print("  Gyro Z offset (rad/s): "); Serial.println(gyro_z_cal, 6);
}

void readAndUpdateMPU(float dt_val) {
    sensors_event_t acc_event, gyro_event, temp_event;
    mpu.getEvent(&acc_event, &gyro_event, &temp_event);
    float acc_x = acc_event.acceleration.x, acc_y = acc_event.acceleration.y, acc_z = acc_event.acceleration.z;
    float gyro_x = gyro_event.gyro.x - gyro_x_cal;
    float gyro_y = gyro_event.gyro.y - gyro_y_cal;
    float gyro_z = gyro_event.gyro.z - gyro_z_cal;
    float acc_pitch_rad = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
    float acc_roll_rad  = atan2(acc_y, acc_z);
    float gyro_pitch_change_deg = (gyro_y * RAD_TO_DEG) * dt_val;
    float gyro_roll_change_deg  = (gyro_x * RAD_TO_DEG) * dt_val;
    float gyro_yaw_change_deg   = (gyro_z * RAD_TO_DEG) * dt_val;
    float acc_pitch_deg = acc_pitch_rad * RAD_TO_DEG;
    float acc_roll_deg  = acc_roll_rad * RAD_TO_DEG;
    pitch = MPU_ALPHA * (pitch + gyro_pitch_change_deg) + (1.0 - MPU_ALPHA) * acc_pitch_deg;
    roll  = MPU_ALPHA * (roll  + gyro_roll_change_deg)  + (1.0 - MPU_ALPHA) * acc_roll_deg;
    yaw   = yaw   + gyro_yaw_change_deg;
    if (yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

// --- Balancing PID ---
void runBalancingPID() {
    unsigned long now = micros();
    float dt_pid = (now - balance_pid_last_time) / 1000000.0f;
    balance_pid_last_time = now;

    if (dt_pid <= 0.00001f) dt_pid = MPU_DT_FIXED; // Use fixed dt if calculated is too small or zero

    float error = balance_pid_setpoint - pitch;

    float P_term = balance_Kp * error;

    balance_pid_error_sum += error * dt_pid;
    balance_pid_error_sum = constrain(balance_pid_error_sum, -MAX_PID_ERROR_SUM, MAX_PID_ERROR_SUM);
    float I_term = balance_Ki * balance_pid_error_sum;

    float error_derivative = (dt_pid > 0.00001f) ? (error - balance_pid_last_error) / dt_pid : 0.0f; // Avoid div by zero
    float D_term = balance_Kd * error_derivative;
    balance_pid_last_error = error;

    balance_pid_output = P_term + I_term + D_term;
    balance_pid_output = constrain(balance_pid_output, -MAX_MOTOR_VELOCITY_CMD, MAX_MOTOR_VELOCITY_CMD);
}

void applyMotorCommands(float command_L, float command_R) {
    if (balancing_active) {
        motor_L.move(command_L);
        motor_R.move(command_R);
    } else {
        motor_L.move(0);
        motor_R.move(0);
    }
}

// --- Serial Communication ---
void parseAndExecuteCommand(String cmd) {
    if (cmd.equalsIgnoreCase("ENABLE")) {
        if (!balancing_active) {
            Serial.println("Enabling balancing...");
            balance_pid_error_sum = 0;
            balance_pid_last_error = pitch; // Use current pitch to avoid D-spike
            motor_L.enable();
            motor_R.enable();
            balancing_active = true;
            printBalanceConfirmation("Balancing Enabled");
        } else { Serial.println("Balancing already enabled."); }
    } else if (cmd.equalsIgnoreCase("DISABLE")) {
        Serial.println("Disabling balancing...");
        motor_L.disable(); motor_R.disable(); balancing_active = false;
        balance_pid_output = 0; applyMotorCommands(0,0);
        printBalanceConfirmation("Balancing Disabled");
    } else if (cmd.startsWith("P=") || cmd.startsWith("I=") || cmd.startsWith("D=")) {
        float p_val = balance_Kp, i_val = balance_Ki, d_val = balance_Kd;
        char* str = (char*)cmd.c_str(); char* token = strtok(str, " ");
        while (token != NULL) {
            if (String(token).startsWith("P=")) sscanf(token, "P=%f", &p_val);
            else if (String(token).startsWith("I=")) sscanf(token, "I=%f", &i_val);
            else if (String(token).startsWith("D=")) sscanf(token, "D=%f", &d_val);
            token = strtok(NULL, " ");
        }
        balance_Kp = p_val; balance_Ki = i_val; balance_Kd = d_val;
        Serial.print("Balance PID Updated: Kp="); Serial.print(balance_Kp,3);
        Serial.print(" Ki="); Serial.print(balance_Ki,3);
        Serial.print(" Kd="); Serial.println(balance_Kd,3);
        printBalanceConfirmation("Balance PID Updated");
    } else if (cmd.startsWith("SETPOINT=")) {
        float sp_val = balance_pid_setpoint;
        sscanf(cmd.c_str(), "SETPOINT=%f", &sp_val);
        balance_pid_setpoint = sp_val;
        Serial.print("Balance Setpoint Updated: "); Serial.println(balance_pid_setpoint, 2);
        printBalanceConfirmation("Balance Setpoint Updated");
    }
    else { Serial.print("CMD Unknown: '"); Serial.print(cmd); Serial.println("'"); }
}

void sendBalanceTelemetry() {
    Serial.print("PITCH="); Serial.print(pitch, 2);
    Serial.print(" ROLL="); Serial.print(roll, 2);
    Serial.print(" YAW="); Serial.print(yaw, 2);
    Serial.print(" PID_OUT="); Serial.print(balance_pid_output, 3);
    Serial.print(" MTR_L_VEL="); Serial.print(motor_L.shaftVelocity(), 2);
    Serial.print(" MTR_R_VEL="); Serial.print(motor_R.shaftVelocity(), 2);
    Serial.println();
}

void printBalanceConfirmation(String msg) {
    Serial.print("CMD_RESP: ");
    Serial.println(msg);
}
