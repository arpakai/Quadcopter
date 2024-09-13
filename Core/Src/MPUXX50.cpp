#include "MPUXX50.h"

#include "string.h"

/// @brief MPUXX50 I2C constructor
/// @param pI2Cx Pointer to I2C structure config
MPUXX50::MPUXX50(I2C_HandleTypeDef *pI2Cx = nullptr, UART_HandleTypeDef *pUARTx = nullptr) : 
                                                                        aScaleFactor{0}, gScaleFactor{0}, 
                                                                        _accel_fchoice{0x01}, _gyro_fchoice{0x03},
                                                                        _kalman_angle_roll{0}, _kalman_uncertainty_angle_roll{2 * 2},
                                                                        _kalman_angle_pitch{0}, _kalman_uncertainty_angle_pitch{2 * 2},
                                                                        _kalman_1d_output{0},
                                                                        _gFSR(GFSR_500DPS), _aFSR(AFSR_4G),
                                                                        _tau(0.98f), _dt(0.004f),
                                                                        Mscale(MFS_16BITS), Mmode(0x02),
                                                                        _gyro_cal{0}, _magnetometer{0},
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
    uint8_t check, select;

    // Confirm device
    HAL_I2C_Mem_Read(_pI2Cx, _addr, WHO_AM_I, 1, &check, 1, I2C_TIMOUT_MS);

    // TODO: If 9250 or 6050 fails could it trigger the opposite check???
    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {

        select = 0x80;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS); // Write a one to bit 7 reset bit; toggle reset device
        HAL_Delay(100);
        
        // Startup / reset the sensor
        select = 0x00;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);
        HAL_Delay(100);

        select = 0x01;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);
        HAL_Delay(200);


        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        _write_data = (uint8_t)GYRO_DLPF_CFG::DLPF_41HZ;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, MPU_CONFIG, 1, &_write_data, 1, I2C_TIMOUT_MS);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        _write_data = (uint8_t)FIFO_SAMPLE_RATE::SMPL_200HZ;                             // Use a 200 Hz rate; a rate consistent with the filter update rate
        HAL_I2C_Mem_Write(_pI2Cx, _addr, SMPLRT_DIV, 1, &_write_data, 1, I2C_TIMOUT_MS); // determined inset in CONFIG above

        // Set gyroscope full scale range
        // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        HAL_I2C_Mem_Read(_pI2Cx, _addr, GYRO_CONFIG, 1, &_read_data, 1, I2C_TIMOUT_MS); // get current GYRO_CONFIG register value
        _read_data = _read_data & ~0xE0;                                    // Clear self-test bits [7:5]
        _read_data = _read_data & ~0x03;                                    // Clear Fchoice bits [1:0]
        _read_data = _read_data & ~0x18;                                    // Clear GYRO_FS_SEL bits [4:3]
        _read_data = _read_data | (uint8_t(GFSR_500DPS) << 3);              // Set scale range for the gyro
        _read_data = _read_data | (uint8_t(_gyro_fchoice) & 0x03);          // Set Fchoice for the gyro
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &_read_data, 1, I2C_TIMOUT_MS);




        _init_mag();

        // Set the full scale ranges
        //writeAccFullScaleRange(_aFSR);
        //writeGyroFullScaleRange(_gFSR);

        // quatFilter.select_filter(QuatFilterSel::MADGWICK);

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

/// @brief Read raw data from IMU
/// @return Structure containing raw accelerometer and gyroscope data
RawData MPUXX50::readRawData()
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
        uint8_t c = _reg_mag_data[6];

        if (!(c & 0x08))
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
ProcessedData MPUXX50::processData()
{
    ProcessedData ret_val{0};

    // Get raw values from the IMU
    _raw_data = readRawData();

    // Convert accelerometer values to g's
    ret_val.ax = _raw_data.ax / aScaleFactor - 0.01;
    ret_val.ay = _raw_data.ay / aScaleFactor - 0.03;
    ret_val.az = _raw_data.az / aScaleFactor;

    _angle_roll = atan(ret_val.ay / (sqrt(ret_val.ax * ret_val.ax * ret_val.az))) * RAD2DEG;
    _angle_pitch = atan(ret_val.ax / (sqrt(ret_val.ay * ret_val.ay * ret_val.az))) * RAD2DEG;

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
    _processed_data = processData();
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

    quatFilter.update(-_processed_data.ax, _processed_data.ay, _processed_data.az,
                      (_processed_data.gx * DEG_TO_RAD), (-_processed_data.gy * DEG_TO_RAD), (-_processed_data.gz * DEG_TO_RAD),
                      _processed_data.mx, -_processed_data.my, _processed_data.mz, q);

    /*QuaternionFilter*/
    float a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components
    a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    ret_val.roll = atan2f(a31, a33);
    ret_val.yaw = -asinf(a32);
    ret_val.pitch = atan2f(a12, a22);
    ret_val.roll *= 180.0f / PI;
    ret_val.yaw *= 180.0f / PI;
    ret_val.pitch *= 180.0f / PI;
    ret_val.pitch += magnetic_declination;
    if (ret_val.pitch >= +180.f)
        ret_val.pitch -= 360.f;
    else if (ret_val.pitch < -180.f)
        ret_val.pitch += 360.f;

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
    double accelPitch = atan2(_processed_data.ay, _processed_data.az) * RAD2DEG;
    double accelRoll = atan2(_processed_data.ax, _processed_data.az) * RAD2DEG;

    ret_val.roll = _tau * (ret_val.roll - _processed_data.gy * _dt) + (1 - _tau) * accelRoll;
    ret_val.pitch = _tau * (ret_val.pitch - _processed_data.gx * _dt) + (1 - _tau) * accelPitch;
    ret_val.yaw += (_processed_data.gz * _dt);

    return ret_val;
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
        _raw_data = readRawData();
        x += _raw_data.gx;
        y += _raw_data.gy;
        z += _raw_data.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    _gyro_cal.x = (float)x / (float)numCalPoints;
    _gyro_cal.y = (float)y / (float)numCalPoints;
    _gyro_cal.z = (float)z / (float)numCalPoints;
}

/// @brief Set the accelerometer full scale range.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
void MPUXX50::writeAccFullScaleRange(uint8_t aFSR)
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
void MPUXX50::writeGyroFullScaleRange(uint8_t gFSR)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (gFSR)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.00f;
        select = 0x00;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.50f;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.80f;
        select = 0x10;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.40f;
        select = 0x18;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 65.50f;
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
    float calMagX = (float)(rawMagCalData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
    float calMagY = (float)(rawMagCalData[1] - 128) / 256. + 1.;
    float calMagZ = (float)(rawMagCalData[2] - 128) / 256. + 1.;

    snprintf((char *)serialBuffer, sizeof(serialBuffer), "Mag call off X: %f\r\nMag call off Y: %f\r\nMag call off Z: %f\r\n", calMagX, calMagY, calMagZ);
    HAL_UART_Transmit(_pUARTx, serialBuffer, sizeof(serialBuffer), 100);
    HAL_Delay(100);

    // Power down magnetometer
    _write_data = 0x00;
    HAL_I2C_Mem_Write(_pI2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &_write_data, 1, I2C_TIMOUT_MS);
    HAL_Delay(100);

    // Set magnetometer data resolution and sample ODR
    _write_data = Mscale << 4 | 0x02; // _write_data = 0x16;
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
