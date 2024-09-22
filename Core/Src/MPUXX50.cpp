#include "MPUXX50.h"

#include "string.h"

/// @brief MPUXX50 I2C constructor
/// @param pI2Cx Pointer to I2C structure config
MPUXX50::MPUXX50(I2C_HandleTypeDef *pI2Cx = nullptr, UART_HandleTypeDef *pUARTx = nullptr) : 
                                                                        q{1.0, 0.0, 0.0, 0.0},
                                                                        aScaleFactor{0}, gScaleFactor{0}, 
                                                                        _accel_fchoice{0x01}, _gyro_fchoice{0x03},
                                                                        _kalman_angle_roll{0}, _kalman_uncertainty_angle_roll{2 * 2},
                                                                        _kalman_angle_pitch{0}, _kalman_uncertainty_angle_pitch{2 * 2},
                                                                        _kalman_1d_output{0},
                                                                        _gFSR(GFSR_500DPS), _aFSR(AFSR_4G),
                                                                        _tau(0.98f), _dt(0.004f),
                                                                        Mscale(MFS_16BITS), Mmode(0x02),
                                                                        _gyro_cal{0}, _mag_data{0},
                                                                        _raw_data{0}, _processed_data{0},
                                                                        _attitude{0}, _kalman{0},
                                                                        _read_data{0}, _write_data{0},
                                                                        serialBuffer{0}, _reg_mag_data{0},
                                                                        buf{0}
{
    _pI2Cx = pI2Cx;
    _pUARTx = pUARTx;
    _addr = AD0_LOW << 1;
}

/// @brief Boot up the IMU and ensure we have a valid connection
/// @return Success [1] or fail [0]
uint8_t MPUXX50::begin()
{
    // Initialize variables
    uint8_t check;

    // Confirm device
    HAL_I2C_Mem_Read(_pI2Cx, _addr, WHO_AM_I, 1, &check, 1, I2C_TIMOUT_MS);

    // TODO: If 9250 or 6050 fails could it trigger the opposite check???
    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {

        _start_sensor();   
        _configure_gyro_thermo();
        _set_gyro_full_scale_range();
        _set_acc_full_scale_range();
        _set_acc_sample_rate_configuration();
        _configure_interrupts();
        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
        
        _init_mag();

        // Set the full scale ranges
        _set_acc_scale_range(_aFSR);
        _set_gyro_scale_range(_gFSR);

        quatFilter.select_filter(QuatFilterSel::MAHONY);

        // Calibrate the IMU
        memset(serialBuffer, 0, sizeof(serialBuffer));
        snprintf((char *)serialBuffer, sizeof(serialBuffer), "CALIBRATING...\r\n");
        HAL_UART_Transmit(_pUARTx, serialBuffer, strlen((char *)serialBuffer), HAL_MAX_DELAY);
        _calibrate_gyro(1500);

        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief Set the gyroscope full scale range
/// @param gFSR Desired yroscope full scale range
void MPUXX50::setGyroFullScaleRange(uint8_t gFSR)
{
    _gFSR = gFSR;
}

/// @brief Set the accelerometer full scale range
/// @param aFSR Desired accelerometer full scale range
void MPUXX50::setAccFullScaleRange(uint8_t aFSR)
{
    _aFSR = aFSR;
}

/// @brief Set the sampling duration (delta time) in seconds
/// @param dt Sampling time delta in seconds
void MPUXX50::setDeltaTime(float dt)
{
    _dt = dt;
}

/// @brief Time constant of the complementary filter
/// @param tau Time constant
void MPUXX50::setTau(float tau)
{
    _tau = tau;
}

double MPUXX50::_clamp_value(double value, double min_val, double max_val) 
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

ProcessedData MPUXX50::_convert_accel_values(RawData &raw_data) 
{
    ProcessedData ret_val{0};

    ret_val.ax = raw_data.ax / aScaleFactor - 0.02;
    if (fabs(ret_val.ax) < THRESHOLD) {
        ret_val.ax = (ret_val.ax < 0) ? -THRESHOLD : THRESHOLD;
    }
    ret_val.ax = _clamp_value(ret_val.ax, -0.99, 0.99);

    ret_val.ay = raw_data.ay / aScaleFactor;
    if (fabs(ret_val.ay) < THRESHOLD) {
        ret_val.ay = (ret_val.ay < 0) ? -THRESHOLD : THRESHOLD;
    }
    ret_val.ay = _clamp_value(ret_val.ay, -0.99, 0.99);

    ret_val.az = raw_data.az / aScaleFactor;
    if (fabs(ret_val.az) < THRESHOLD) {
        ret_val.az = (ret_val.az < 0) ? -THRESHOLD : THRESHOLD;
    }
    ret_val.az = _clamp_value(ret_val.az, -0.99, 0.99);

    return ret_val;
}

double MPUXX50::_signed_sqrt(double value) {
    return (signbit(value)) ? -sqrt(-value) : sqrt(value); //signbit(-1) gives 1
}

void MPUXX50::_calculate_angles(ProcessedData &processed_data)
{
    // Calculate _angle_roll
    double multiply = processed_data.ax * processed_data.ax * processed_data.az;
    multiply = _signed_sqrt(multiply);
    _angle_roll = atan(processed_data.ay / multiply) * RAD2DEG;

    // Calculate _angle_pitch
    multiply = processed_data.ay * processed_data.ay * processed_data.az;
    multiply = _signed_sqrt(multiply);
    _angle_pitch = atan(processed_data.ax / multiply) * RAD2DEG;
}

/// @brief Read raw data from IMU
/// @return Structure containing raw accelerometer and gyroscope data
RawData MPUXX50::_read_raw_data()
{
    RawData ret_val{0};
    memset(buf, 0, sizeof(buf));
    // Subroutine for reading the raw data
    HAL_I2C_Mem_Read(_pI2Cx, _addr, ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data
    ret_val.ax = buf[0] << 8 | buf[1];
    ret_val.ay = buf[2] << 8 | buf[3];
    ret_val.az = buf[4] << 8 | buf[5];
    // temperature = buf[6] << 8 | buf[7];
    ret_val.gx = buf[8] << 8 | buf[9];
    ret_val.gy = buf[10] << 8 | buf[11];
    ret_val.gz = buf[12] << 8 | buf[13];

    memset(&_read_data, 0, sizeof(uint8_t));
    HAL_I2C_Mem_Read(_pI2Cx, AK8963_ADDRESS, AK8963_ST1, 1, &_read_data, 1, I2C_TIMOUT_MS);
    if ((_read_data & 0x01) == 0x01)
    {
        HAL_I2C_Mem_Read(_pI2Cx, AK8963_ADDRESS, AK8963_XOUT_L, 1, &_reg_mag_data[0], 7, I2C_TIMOUT_MS);
        uint8_t magB = _reg_mag_data[6];

        if (!(magB & 0x08))
        {
            ret_val.mx = ((int16_t)_reg_mag_data[1] << 8) | _reg_mag_data[0];
            ret_val.my = ((int16_t)_reg_mag_data[3] << 8) | _reg_mag_data[2];
            ret_val.mz = ((int16_t)_reg_mag_data[5] << 8) | _reg_mag_data[4];
        }
    }
    // Result
    return ret_val;
}

/// @brief Process the raw data into real world sensor values
/// @return Structure containing processed accelerometer and gyroscope data
ProcessedData MPUXX50::_process_data()
{
    ProcessedData ret_val{0};

    // Get raw values from the IMU
    _raw_data = _read_raw_data();

    // Convert accelerometer values to g's
    ret_val = _convert_accel_values(_raw_data);
    _calculate_angles(ret_val);

    // Compensate for gyro offset
    ret_val.gx = _raw_data.gx - _gyro_cal.x;
    ret_val.gy = _raw_data.gy - _gyro_cal.y;
    ret_val.gz = _raw_data.gz - _gyro_cal.z;

    // Convert gyro values to deg/s
    ret_val.gx /= gScaleFactor;
    ret_val.gy /= gScaleFactor;
    ret_val.gz /= gScaleFactor;

    ret_val.mx = _raw_data.mx;
    ret_val.my = _raw_data.my;
    ret_val.mz = _raw_data.mz;

    // Return structure
    return ret_val;
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter
/// @return Structure containing sensor attitude data
template <typename T>
T MPUXX50::_get_calculated_attitude()
{
    _processed_data = _process_data();
    static_assert(sizeof(T) == 0, "get_value not implemented for this type");
    return T(); // Default constructor
}

/// @brief No filter _get_calculated_attitude() fn.
template <>
Attitude MPUXX50::_get_calculated_attitude<Attitude>()
{
    Attitude ret_val;
    ret_val.roll = _processed_data.gx;
    ret_val.pitch = _processed_data.gy;
    ret_val.yaw = _processed_data.gz;

    return ret_val;
}

/// @brief Specialization for kalmanf
template <>
kalmanf MPUXX50::_get_calculated_attitude<kalmanf>()
{
    kalmanf ret_val;
    ret_val = _calc_kalman_filter(_processed_data);

    return ret_val;
}

template <>
madgwickf MPUXX50::_get_calculated_attitude<madgwickf>()
{
    madgwickf ret_val;
    _processed_data = _process_data();
    quatFilter.update((-1 * _processed_data.ax), _processed_data.ay, _processed_data.az,
                      (_processed_data.gx * DEG2RAD), (-_processed_data.gy * DEG2RAD), (-_processed_data.gz * DEG2RAD),
                      _processed_data.mx, -_processed_data.my, _processed_data.mz, quat);

    /*QuaternionFilter*/
    double a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components

    a12 = (-2.0 * quat[1] * quat[2]) + (2 * quat[0] * quat[3]);
    a22 = (quat[0] * quat[0]) - (quat[1] * quat[1]) + (quat[2] * quat[2]) - (quat[3] * quat[3]);
    a31 = -2.0 * quat[1] * quat[3] + 2.0 * quat[0] * quat[2];
    a32 = 2.0 * (quat[2] * quat[3] + quat[0] * quat[1]);
    a33 = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3];

    ret_val.roll = atan2f(a31, a33); //0 roll, 1 pitch, 2yaw
    ret_val.pitch = asinf(a32);
    ret_val.yaw = - atan2f(a12, a22);
    ret_val.roll *= 180.0 / PI;
    ret_val.pitch *= 180.0 / PI;
    ret_val.yaw *= 180.0 / PI;
    ret_val.yaw += magnetic_declination;
    if (ret_val.yaw >= +180.f)
        ret_val.yaw -= 360.f;
    else if (ret_val.yaw < -180.f)
        ret_val.yaw += 360.f;

    linAcc[0] = _processed_data.ax + a31;
    linAcc[1] = _processed_data.ay + a32;
    linAcc[2] = _processed_data.az - a33;

    return ret_val;
}

// @brief Specialization for complementaryf
template <>
complementaryf MPUXX50::_get_calculated_attitude<complementaryf>()
{
    complementaryf ret_val;

    /*Complementary filter*/
    // double accelPitch = atan2(_processed_data.ay, _processed_data.az) * RAD2DEG;
    // double accelRoll = atan2(_processed_data.ax, _processed_data.az) * RAD2DEG;

    ret_val.roll = _tau * (ret_val.roll - _processed_data.gy * _dt) + (1 - _tau) * _angle_roll;
    ret_val.pitch = _tau * (ret_val.pitch - _processed_data.gx * _dt) + (1 - _tau) * _angle_pitch;
    ret_val.yaw += (_processed_data.gz * _dt);

    return ret_val;
}

template <>
quaternionf MPUXX50::_get_calculated_attitude<quaternionf>()
{
    quaternionf ret_val;

    _processed_data = _process_data();

    Now = HAL_GetTick();
	deltat = ((Now - lastUpdate)/1000.0); // set integration time by time elapsed since last filter update
	lastUpdate = Now;
	sum += deltat; // sum for averaging filter update rate

    _quaternion_update(_processed_data);

	// Convert quaternions to Euler angles
	a12 =   2.0 * (q[1] * q[2] + q[0] * q[3]);
	a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	a31 =   2.0 * (q[0] * q[1] + q[2] * q[3]);
	a32 =   2.0 * (q[1] * q[3] - q[0] * q[2]);
	a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	ret_val.pitch = -asinf(a32);
	ret_val.roll  = atan2f(a31, a33);
	ret_val.yaw   = atan2f(a12, a22);
	ret_val.pitch *= 180.000 / PI;
	ret_val.yaw   *= 180.000 / PI;
	ret_val.yaw   += 5.53; // Declination

	if(ret_val.yaw < 0) 
        ret_val.yaw   += 360.0; // Ensure yaw stays between 0 and 360
	ret_val.roll  *= 180.000 / PI;
	lin_ax = ax + a31;
	lin_ay = ay + a32;
	lin_az = az - a33;

    return ret_val;
}

void MPUXX50::_quaternion_update(ProcessedData &processed_data){

    processed_data.gx *= DEG2RAD;
    processed_data.gy *= DEG2RAD;
    processed_data.gz *= DEG2RAD;

    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    double norm;
    double hx, hy, _2bx, _2bz;
    double s1, s2, s3, s4;
    double qDot1, qDot2, qDot3, qDot4;
    double multiply{0};

    // Auxiliary variables to avoid repeated arithmetic
    double _2q1mx;
    double _2q1my;
    double _2q1mz;
    double _2q2mx;
    double _4bx;
    double _4bz;
    double _2q1 = 2.0 * q1;
    double _2q2 = 2.0 * q2;
    double _2q3 = 2.0 * q3;
    double _2q4 = 2.0 * q4;
    double _2q1q3 = 2.0 * q1 * q3;
    double _2q3q4 = 2.0 * q3 * q4;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q1q4 = q1 * q4;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q2q4 = q2 * q4;
    double q3q3 = q3 * q3;
    double q3q4 = q3 * q4;
    double q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    multiply = processed_data.ax * processed_data.ax + processed_data.ay * processed_data.ay + processed_data.az * processed_data.az;
    norm = _signed_sqrt(multiply);
    // norm = sqrt(processed_data.ax * processed_data.ax + processed_data.ay * processed_data.ay + processed_data.az * processed_data.az);
    if (norm == 0.0000) return; // handle NaN
    norm = 1.0/norm;
    processed_data.ax *= norm;
    processed_data.ay *= norm;
    processed_data.az *= norm;

    // Normalise magnetometer measurement
    multiply = processed_data.mx * processed_data.mx + processed_data.my * processed_data.my + processed_data.mz * processed_data.mz;
    norm = _signed_sqrt(multiply);
    // norm = sqrt(processed_data.mx * processed_data.mx + processed_data.my * processed_data.my + processed_data.mz * processed_data.mz);
    if (norm == 0.0000) return; // handle NaN
    norm = 1.0/norm;
    processed_data.mx *= norm;
    processed_data.my *= norm;
    processed_data.mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0 * q1 * processed_data.mx;
    _2q1my = 2.0 * q1 * processed_data.my;
    _2q1mz = 2.0 * q1 * processed_data.mz;
    _2q2mx = 2.0 * q2 * processed_data.mx;
    hx = processed_data.mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + processed_data.mx * q2q2 + _2q2 * processed_data.my * q3 + _2q2 * processed_data.mz * q4 - processed_data.mx * q3q3 - processed_data.mx * q4q4;
    hy = _2q1mx * q4 + processed_data.my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - processed_data.my * q2q2 + processed_data.my * q3q3 + _2q3 * processed_data.mz * q4 - processed_data.my * q4q4;
    multiply = hx * hx + hy * hy;
    _2bx = _signed_sqrt(multiply);
    //_2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + processed_data.mz * q1q1 + _2q2mx * q4 - processed_data.mz * q2q2 + _2q3 * processed_data.my * q4 - processed_data.mz * q3q3 + processed_data.mz * q4q4;
    _4bx = 2.0 * _2bx;
    _4bz = 2.0 * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - processed_data.ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - processed_data.ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - processed_data.mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - processed_data.my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - processed_data.mz);
    s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - processed_data.ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - processed_data.ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - processed_data.az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - processed_data.mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - processed_data.my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - processed_data.mz);
    s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - processed_data.ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - processed_data.ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - processed_data.az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - processed_data.mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - processed_data.my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - processed_data.mz);
    s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - processed_data.ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - processed_data.ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - processed_data.mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - processed_data.my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - processed_data.mz);
    multiply = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4;
    norm = _signed_sqrt(multiply);
    // norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5 * (-q2 * processed_data.gx - q3 * processed_data.gy - q4 * processed_data.gz) - beta * s1;
    qDot2 = 0.5 * (q1 * processed_data.gx + q3 * processed_data.gz - q4 * processed_data.gy) - beta * s2;
    qDot3 = 0.5 * (q1 * processed_data.gy - q2 * processed_data.gz + q4 * processed_data.gx) - beta * s3;
    qDot4 = 0.5 * (q1 * processed_data.gz + q2 * processed_data.gy - q3 * processed_data.gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    multiply = q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4;
    norm = _signed_sqrt(multiply);
    // norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0/norm;

    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

/// @brief Find offsets for each axis of gyroscope
void MPUXX50::_calibrate_gyro(uint16_t numCalPoints)
{
    // Init
    RawData _raw_data;
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        _raw_data = _read_raw_data();
        x += _raw_data.gx;
        y += _raw_data.gy;
        z += _raw_data.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    _gyro_cal.x = (double)x / (double)numCalPoints;
    _gyro_cal.y = (double)y / (double)numCalPoints;
    _gyro_cal.z = (double)z / (double)numCalPoints;
}

/// @brief Set the accelerometer full scale range.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
void MPUXX50::_set_acc_scale_range(uint8_t aFSR)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (aFSR)
    {
    case AFSR_2G:
        aScaleFactor = 16384.0;
        select = 0x00;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_8G:
        aScaleFactor = 4096.0;
        select = 0x10;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_16G:
        aScaleFactor = 2048.0;
        select = 0x18;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aScaleFactor = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Set the gyroscope full scale range.
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
void MPUXX50::_set_gyro_scale_range(uint8_t gFSR)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (gFSR)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.00;
        select = 0x00;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.50;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.80;
        select = 0x10;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.40;
        select = 0x18;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 65.50;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

void MPUXX50::_init_mag()
{

    // enable Mag bypass
    _write_data = 0x22;
    HAL_I2C_Mem_Write(_pI2Cx, WHO_AM_I, INT_PIN_CFG, 1, &_write_data, 1, I2C_TIMOUT_MS);

    // read AK8963 WHOAMI
    HAL_I2C_Mem_Read(_pI2Cx, AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &_read_data, 1, I2C_TIMOUT_MS);

    memset(serialBuffer, 0, sizeof(serialBuffer));
    snprintf((char *)serialBuffer, sizeof(serialBuffer), "MAG WHO AM I is (Must return 72): %d\r\n", _read_data);
    HAL_UART_Transmit(_pUARTx, serialBuffer, sizeof(serialBuffer), 100);

    // Init Mag------------------------------------------------------∏------------------------------------------------------
    // Power down magnetometer
    _write_data = 0x00;
    HAL_I2C_Mem_Write(_pI2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &_write_data, 1, I2C_TIMOUT_MS);
    HAL_Delay(100);

    // Enter Fuse ROM access mode
    _write_data = 0x0F;
    HAL_I2C_Mem_Write(_pI2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &_write_data, 1, I2C_TIMOUT_MS);
    HAL_Delay(100);

    // Read the x-, y-, and z-axis calibration values
    uint8_t rawMagCalData[3];
    HAL_I2C_Mem_Read(_pI2Cx, AK8963_ADDRESS, AK8963_ASAX, 1, &rawMagCalData[0], 3, I2C_TIMOUT_MS);
    double calMagX = (double)(rawMagCalData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
    double calMagY = (double)(rawMagCalData[1] - 128) / 256. + 1.;
    double calMagZ = (double)(rawMagCalData[2] - 128) / 256. + 1.;

    memset(serialBuffer, 0, sizeof(serialBuffer));
    snprintf((char *)serialBuffer, sizeof(serialBuffer), "Mag call off X: %f\r\nMag call off Y: %f\r\nMag call off Z: %f\r\n", calMagX, calMagY, calMagZ);
    HAL_UART_Transmit(_pUARTx, serialBuffer, sizeof(serialBuffer), 100);
    HAL_Delay(100);

    // Power down magnetometer
    _write_data = 0x00;
    HAL_I2C_Mem_Write(_pI2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &_write_data, 1, I2C_TIMOUT_MS);
    HAL_Delay(100);

    // Set magnetometer data resolution and sample ODR
    _write_data = Mscale << 4 | 0x02; // _write_data = 0x16;
    memset(serialBuffer, 0, sizeof(serialBuffer));
    snprintf((char *)serialBuffer, sizeof(serialBuffer), "_write_data:%d\r\n", _write_data);
    HAL_UART_Transmit(_pUARTx, serialBuffer, sizeof(serialBuffer), 100);

    HAL_I2C_Mem_Write(_pI2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &_write_data, 1, I2C_TIMOUT_MS);
    HAL_Delay(100);
}

kalmanf MPUXX50::_calc_kalman_filter(ProcessedData &process_data)
{
    kalmanf ret_val;

    kalman_1d(_kalman_angle_roll, _kalman_uncertainty_angle_roll, _processed_data.gx, _angle_pitch);
    _kalman_angle_roll = _kalman_1d_output[0];
    _kalman_uncertainty_angle_roll = _kalman_1d_output[1];
    kalman_1d(_kalman_angle_pitch, _kalman_uncertainty_angle_pitch, _processed_data.gx, _angle_pitch);
    _kalman_angle_pitch = _kalman_1d_output[0];
    _kalman_uncertainty_angle_pitch = _kalman_1d_output[1];

    ret_val.roll = _kalman_angle_roll;
    ret_val.pitch = _kalman_angle_pitch;

    return ret_val;
}

void MPUXX50::kalman_1d(double kalman_state, double kalman_uncertainty, double kalman_input, double kalman_measurement)
{
    kalman_state = kalman_state + 0.004 * kalman_input;
    kalman_uncertainty += KALMAN_UNCERTAINTY_VAL;
    double kalman_gain = kalman_uncertainty * (1 / (kalman_uncertainty + 9));
    kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state);
    kalman_uncertainty = (1 - kalman_gain) * kalman_uncertainty;

    _kalman_1d_output[0] = kalman_state;
    _kalman_1d_output[1] = kalman_uncertainty;
}

void MPUXX50::_start_sensor()
{
    uint8_t select = 0x80;
    HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS); // Write a one to bit 7 reset bit; toggle reset device
    HAL_Delay(100);
    
    // Startup / reset the sensor
    select = 0x00;
    HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);
    HAL_Delay(100);

    select = 0x01;
    HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);
    HAL_Delay(200);
}

/// @brief 
//  Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
/// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
/// be higher than 1 / 0.0059 = 170 Hz
/// GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
/// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
void MPUXX50::_configure_gyro_thermo()
{
        _write_data = (uint8_t)GYRO_DLPF_CFG::DLPF_41HZ;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, MPU_CONFIG, 1, &_write_data, 1, I2C_TIMOUT_MS);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        _write_data = (uint8_t)FIFO_SAMPLE_RATE::SMPL_200HZ;                             // Use a 200 Hz rate; a rate consistent with the filter update rate
        HAL_I2C_Mem_Write(_pI2Cx, _addr, SMPLRT_DIV, 1, &_write_data, 1, I2C_TIMOUT_MS); // determined inset in CONFIG above
}

/// @brief 
// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
void MPUXX50::_set_gyro_full_scale_range()
{
    HAL_I2C_Mem_Read(_pI2Cx, _addr, GYRO_CONFIG, 1, &_read_data, 1, I2C_TIMOUT_MS); // get current GYRO_CONFIG register value
    _read_data = _read_data & ~0xE0;                                    // Clear self-test bits [7:5]
    _read_data = _read_data & ~0x03;                                    // Clear Fchoice bits [1:0]
    _read_data = _read_data & ~0x18;                                    // Clear GYRO_FS_SEL bits [4:3]
    _read_data = _read_data | ((uint8_t(GFSR_2000DPS) << 3) << 3);                             // Set full scale range for the gyro 
    _read_data = _read_data | (uint8_t(_gyro_fchoice) & 0x03);          // Set Fchoice for the gyro
    HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &_read_data, 1, I2C_TIMOUT_MS);

}

/// @brief 
void MPUXX50::_set_acc_full_scale_range()
{
    HAL_I2C_Mem_Read(_pI2Cx, _addr, ACCEL_CONFIG, 1, &_read_data, 1, I2C_TIMOUT_MS);    // get current ACCEL_CONFIG register value
    _read_data = _read_data & ~0xE0;                                                    // Clear self-test bits [7:5]
    _read_data = _read_data & ~0x18;                                                    // Clear ACCEL_FS_SEL bits [4:3]
    _read_data = _read_data | (uint8_t(AFSR_16G) << 3);                                 // Set full scale range for the accelerometer
    HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &_read_data, 1, I2C_TIMOUT_MS); 
}

/// @brief  
//  It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
//  accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
void MPUXX50::_set_acc_sample_rate_configuration()
{
    HAL_I2C_Mem_Read(_pI2Cx, _addr, ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, &_read_data, 1, I2C_TIMOUT_MS);       // get current ACCEL_CONFIG2 register value
    _read_data = _read_data & ~0x0F;                                                        // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    _read_data = _read_data | (~(_accel_fchoice << 3) & 0x08);                              // Set accel_fchoice_b to 1
    _read_data = _read_data | (uint8_t(GYRO_DLPF_CFG::DLPF_41HZ) & 0x07);                   // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, &_read_data, 1, I2C_TIMOUT_MS);      // Write new ACCEL_CONFIG2 register value
}

/// @brief 
// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
// can join the I2C bus and all can be controlled by the STM32 as master
void MPUXX50::_configure_interrupts()
{
    _write_data = 0x22;
    HAL_I2C_Mem_Write(_pI2Cx, _addr, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);
    _write_data = 0x01;
    HAL_I2C_Mem_Write(_pI2Cx, _addr, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &_read_data, 1, I2C_TIMOUT_MS);     // Enable data ready (bit 0) interrupt
}

void MPUXX50::_write_gyro_offset()
{
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    uint8_t gyro_offset_data[6]{0};
    gyro_offset_data[0] = (-(int16_t)_gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    gyro_offset_data[1] = (-(int16_t)_gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
    gyro_offset_data[2] = (-(int16_t)_gyro_bias[1] / 4 >> 8) & 0xFF;
    gyro_offset_data[3] = (-(int16_t)_gyro_bias[1] / 4) & 0xFF;
    gyro_offset_data[4] = (-(int16_t)_gyro_bias[2] / 4 >> 8) & 0xFF;
    gyro_offset_data[5] = (-(int16_t)_gyro_bias[2] / 4) & 0xFF;
    // Push gyro biases to hardware registers
    HAL_I2C_Mem_Write(_pI2Cx, _addr, XG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &gyro_offset_data[0], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, XG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &gyro_offset_data[1], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, YG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &gyro_offset_data[2], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, YG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &gyro_offset_data[3], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, ZG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &gyro_offset_data[4], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, ZG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &gyro_offset_data[5], 1, I2C_TIMOUT_MS);
}

/// @brief     
    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
void MPUXX50::_write_accel_offset()
{
    uint8_t read_data[2] = {0};
    int16_t acc_bias_reg[3] = {0, 0, 0};                                                                    // A place to hold the factory accelerometer trim biases
    HAL_I2C_Mem_Read(_pI2Cx, _addr, XA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &read_data[0], 2, I2C_TIMOUT_MS);   // Read factory accelerometer trim values
    acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
    HAL_I2C_Mem_Read(_pI2Cx, _addr, YA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &read_data[0], 2, I2C_TIMOUT_MS);
    acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
    HAL_I2C_Mem_Read(_pI2Cx, _addr, ZA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &read_data[0], 2, I2C_TIMOUT_MS);
    acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];
    int16_t mask_bit[3] = {1, 1, 1}; // Define array to hold mask bit for each accelerometer bias axis
    for (int i = 0; i < 3; i++)
    {
        if (acc_bias_reg[i] % 2)
        {
            mask_bit[i] = 0;
        }
        acc_bias_reg[i] -= (int16_t)_acc_bias[i] >> 3; // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
        if (mask_bit[i])
        {
            acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i]; // Preserve temperature compensation bit
        }
        else
        {
            acc_bias_reg[i] = acc_bias_reg[i] | 0x0001; // Preserve temperature compensation bit
        }
    }
    uint8_t write_data[6] = {0};
    write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
    write_data[1] = (acc_bias_reg[0]) & 0xFF;
    write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
    write_data[3] = (acc_bias_reg[1]) & 0xFF;
    write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
    write_data[5] = (acc_bias_reg[2]) & 0xFF;
    // Push accelerometer biases to hardware registers
    HAL_I2C_Mem_Write(_pI2Cx, _addr, XA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &write_data[0], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, XA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &write_data[1], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, YA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &write_data[2], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, YA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &write_data[3], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, ZA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &write_data[4], 1, I2C_TIMOUT_MS);
    HAL_I2C_Mem_Write(_pI2Cx, _addr, ZA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &write_data[5], 1, I2C_TIMOUT_MS);
}

void MPUXX50::_collect_acc_gyro_data_for_calibration(double *a_bias, double *g_bias)
{
    // At end of sample accumulation, turn off FIFO sensor read
    uint8_t data[12];                                                                                   // data array to hold accelerometer and gyro x, y, z, data
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, FIFO_EN, 1, &_write_data, 1, I2C_TIMOUT_MS);   // Enable data ready (bit 0) interrupt           // Disable gyro and accelerometer sensors for FIFO
    HAL_I2C_Mem_Read(_pI2Cx, _addr, FIFO_COUNTH, I2C_MEMADD_SIZE_8BIT, &data[0], 2, I2C_TIMOUT_MS);                        // read FIFO sample count
    
    uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
    uint16_t packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (uint16_t ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        HAL_I2C_Mem_Write(_pI2Cx, _addr, FIFO_R_W, I2C_MEMADD_SIZE_8BIT, &data[0], 12, I2C_TIMOUT_MS);  // read data for averaging         
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        a_bias[0] += (double)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        a_bias[1] += (double)accel_temp[1];
        a_bias[2] += (double)accel_temp[2];
        g_bias[0] += (double)gyro_temp[0];
        g_bias[1] += (double)gyro_temp[1];
        g_bias[2] += (double)gyro_temp[2];
    }

    a_bias[0] /= (double)packet_count; // Normalize sums to get average count biases
    a_bias[1] /= (double)packet_count;
    a_bias[2] /= (double)packet_count;
    g_bias[0] /= (double)packet_count;
    g_bias[1] /= (double)packet_count;
    g_bias[2] /= (double)packet_count;

    if (a_bias[2] > 0L)
    {
        a_bias[2] -= 16384.0;
    } // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        a_bias[2] += 16384.0;
    }
}

void MPUXX50::_set_acc_gyro_for_calibration()
{
    // reset device
    _write_data = 0x80; HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS); // Write a one to bit 7 reset bit; toggle reset device
    HAL_Delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    _write_data = 0x01; HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS); 
    _write_data = 0x01; HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);     
    HAL_Delay(200);
    // Configure device for bias calculation
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);    // Disable all interrupts  
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, FIFO_EN, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);       // Disable FIFO
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);    // Turn on internal clock source
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, I2C_MST_CTRL, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);  // Disable I2C master
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);     // Disable FIFO and I2C master modes
    _write_data = 0x0C; HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);    // Reset FIFO and DMP    
    HAL_Delay(15);
    // Configure MPU6050 gyro and accelerometer for bias calculation
    _write_data = 0x01; HAL_I2C_Mem_Write(_pI2Cx, _addr, MPU_CONFIG, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);    // Set low-pass filter to 188 Hz 
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);    // Set sample rate to 1 kHz
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    _write_data = 0x00; HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);  // Set accelerometer full-scale to 2 g, maximum sensitivity
    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    _write_data = 0x40; HAL_I2C_Mem_Write(_pI2Cx, _addr, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);     // Enable FIFO
    _write_data = 0x78; HAL_I2C_Mem_Write(_pI2Cx, _addr, FIFO_EN, I2C_MEMADD_SIZE_8BIT, &_write_data, 1, I2C_TIMOUT_MS);       // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
    HAL_Delay(40);      
}

/// @brief
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// ACCEL_FS_SEL: 2g (maximum sensitivity)
// GYRO_FS_SEL: 250dps (maximum sensitivity)
void MPUXX50::_tune_acc_gyro_impl()
{
    _set_acc_gyro_for_calibration();
    _collect_acc_gyro_data_for_calibration(_acc_bias, _gyro_bias);
    _write_accel_offset();
    _write_gyro_offset();
}
