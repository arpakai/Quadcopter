#pragma once

// Libraries
#include <stdint.h>
#include <math.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"

#include "QuaternionFilter.h"
#include "EKF.h"
#include "MPU9250Regs.h"

// Constants
constexpr double PI = 3.14159;
constexpr double DEG2RAD = 0.01745;
constexpr double RAD2DEG = 57.29577;
constexpr double KALMAN_UNCERTAINTY_VAL = 0.000256;
constexpr double THRESHOLD = 1e-6;

// Structs
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
};

struct ProcessedData
{
    double ax, ay, az, gx, gy, gz, mx, my, mz;
};

struct GyroCal
{
    double x, y, z;
};

struct Mag
{
    double magX, magY, magZ;
};

struct EulerAngles {
    double roll, pitch, yaw;

    EulerAngles(double r = 0.0, double p = 0.0, double y = 0.0) : roll(r), pitch(p), yaw(y) {}
};

struct Attitude : public EulerAngles {
    using EulerAngles::EulerAngles; // Inherit constructors
};

struct kalmanf : public EulerAngles {
    using EulerAngles::EulerAngles; // Inherit constructors
};

struct madgwickf : public EulerAngles {
    using EulerAngles::EulerAngles; // Inherit constructors
};

struct complementaryf : public EulerAngles {
    using EulerAngles::EulerAngles; // Inherit constructors
};

struct quaternionf : public EulerAngles {
    using EulerAngles::EulerAngles;
};

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

enum magnetometerScale
{
    MFS_14BITS = 0, // 0.6 mG per LSB
    MFS_16BITS      // 0.15 mG per LSB
};

enum class FIFO_SAMPLE_RATE : uint8_t
{
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
};

enum class GYRO_DLPF_CFG : uint8_t
{
    DLPF_250HZ,
    DLPF_184HZ,
    DLPF_92HZ,
    DLPF_41HZ,
    DLPF_20HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_3600HZ,
};

enum class ACCEL_DLPF_CFG : uint8_t
{
    DLPF_218HZ_0,
    DLPF_218HZ_1,
    DLPF_99HZ,
    DLPF_45HZ,
    DLPF_21HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_420HZ,
};

class MPUXX50
{
private:
    // Functions
    void _tune_acc_gyro_impl();
    void _set_acc_gyro_for_calibration();
    void _collect_acc_gyro_data_for_calibration(double *a_bias, double *g_bias);

    void _set_gyro_scale_range(uint8_t gFSR);
    void _set_acc_scale_range(uint8_t aFSR);
    kalmanf _calc_kalman_filter(ProcessedData &process_data);
    madgwickf _calc_madgwick_filter(ProcessedData &process_data);
    complementaryf _calc_complementary_filter(ProcessedData &process_data);
    quaternionf _calc_quaternion_filter(ProcessedData &process_data);
    void _quaternion_update(ProcessedData &processed_data);

    double _clamp_value(double value, double min_val, double max_val);
    ProcessedData _convert_accel_values(RawData &raw_data);
    void _calculate_angles(ProcessedData &processed_data);
    double _signed_sqrt(double value);

    void _start_sensor();
    void _configure_gyro_thermo();
    void _set_gyro_full_scale_range();
    void _set_acc_full_scale_range();
    void _set_acc_sample_rate_configuration();
    void _configure_interrupts();

    void kalman_1d(double kalman_state, double kalman_uncertainty, double kalman_input, double kalman_measurement);
    void _init_mag();

    double acc_bias[3]{0.0};    // acc calibration value in ACCEL_FS_SEL: 2g 
    double gyro_bias[3]{0.0};   // gyro calibration value in GYRO_FS_SEL: 250dps

    //  Quaternion
    float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    //float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
    //float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    float beta = 0.6045998;
    float zeta = 0.0;
    
    double q[4];
    double deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
    uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    uint32_t Now = 0;        // used to calculate integration interval
    double lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
    double a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
    //  Quaternion

    //Variables
    float aScaleFactor, gScaleFactor;
    uint8_t _accel_fchoice, _gyro_fchoice;
    I2C_HandleTypeDef *_pI2Cx;
    UART_HandleTypeDef *_pUARTx;
    uint8_t _addr;

    double _kalman_angle_roll, _kalman_uncertainty_angle_roll;
    double _kalman_angle_pitch, _kalman_uncertainty_angle_pitch;
    double _kalman_1d_output[2];

    uint8_t _gFSR;
    uint8_t _aFSR;
    float _tau;
    float _dt;
    uint8_t Mscale;
    uint8_t Mmode;

    // Structs
    GyroCal _gyro_cal;
    Mag             _mag_data;
    RawData         _raw_data;
    ProcessedData   _processed_data;
    Attitude _attitude;
    kalmanf _kalman;

    uint8_t _read_data;
    uint8_t _write_data;
    uint8_t serialBuffer[100];

    uint8_t _reg_mag_data[7];
    uint8_t buf[14];

    QuaternionFilter quatFilter;
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
    float magnetic_declination = -7.51;    // Japan, 24th June
    float linAcc[3]{0};

public:
    // Init
    MPUXX50(I2C_HandleTypeDef *pI2Cx, UART_HandleTypeDef *pUARTx);

    double _angle_roll;
    double _angle_pitch;

    // Functions
    uint8_t begin();
    void _calibrate_gyro(uint16_t numCalPoints);
    RawData _read_raw_data();
    ProcessedData _process_data();

    template<typename T> T _get_calculated_attitude();
    
    void setGyroFullScaleRange(uint8_t gFSR);
    void setAccFullScaleRange(uint8_t aFSR);
    void setDeltaTime(float dt);
    void setTau(float tau);
};
