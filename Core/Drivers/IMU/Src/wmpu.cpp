#include "wmpu.h"

WMPU::WMPU(I2C_HandleTypeDef* i2c_handles[], size_t num_devices, UART_HandleTypeDef* uart_handles):
    _pI2Cx{nullptr}, _pUARTx{uart_handles}, _num_devices{num_devices}, _serial_buf{0}, _attitude{0}, _data_sample_count{0}
{
    for (size_t i = 0; i < _num_devices; ++i) {
        MPUXX50s.push_back(new MPUXX50(i2c_handles[i], uart_handles));
    }
}

// Destructor
WMPU::~WMPU() {
    for (auto mpu : MPUXX50s) {
        delete mpu;
    }
}

void WMPU::_init() {
    for (auto mpu : MPUXX50s) {
        // Check if IMU configured properly and block if it didn't
        uint8_t status;
        do
        {
            status = mpu->_initialize();
            if(status != 1)
            {
                snprintf((char *)_serial_buf, sizeof(_serial_buf), "ERROR!\r\n");
                HAL_UART_Transmit(_pUARTx, _serial_buf, strlen((char *)_serial_buf), HAL_MAX_DELAY);
            }
        }
        while (status != 1);
    }
}

void WMPU::_tune_acc_gyro_offsets() {
    for (auto mpu : MPUXX50s) {
        mpu->_tune_acc_gyro_impl();
    }
}

void WMPU::_get_processed_accel_data(double& x, double& y, double& z) {
    for (auto mpu : MPUXX50s) {
        mpu->_get_processed_accel_data(x, y, z);
    }
}

void WMPU::_get_processed_gyro_data(double& x, double& y, double& z, long&t) {
    for (auto mpu : MPUXX50s) {
        mpu->_get_processed_gyro_data(x, y, z, t);
    }
}

void WMPU::_get_processed_mag_data(double& x, double& y, double& z) {
    for (auto mpu : MPUXX50s) {
        mpu->_get_processed_mag_data(x, y, z);
    }
}

void WMPU::_get_processed_all_data(double& ax, double& ay, double& az, 
                                    double& gx, double& gy, double& gz, long &gt,
                                    double& mx, double& my, double& mz) {
    for (auto mpu : MPUXX50s) {
        mpu->_get_processed_all_data(ax, ay, az, gx, gy, gz, gt, mx, my, mz);
    }
}

void WMPU::_print_roll_pitch_yaw(double& r, double& p, double& y, UART_HandleTypeDef* uart_handle)
{
    sprintf((char *)_serial_buf, "\r\nROT  R:%.2lf, P:%.2lf, Y:%.2lf, Count:%d\n\r",
                r, p, y, _data_sample_count);
    HAL_UART_Transmit(uart_handle, _serial_buf, strlen((char *)_serial_buf), HAL_MAX_DELAY); 
}

void WMPU::_set_computed_average_rpy(uint8_t num_samples, double& r, double& p, double& y)
{
    r = 0.00;
    p = 0.00;
    y = 0.00;

    for (uint8_t i = 0; i < num_samples; ++i) {
        _get_roll_pitch_yaw(r, p, y);
        _attitude.roll += r;
        _attitude.pitch += p;
        _attitude.yaw += y;
    }
    _attitude.roll /= static_cast<double>(num_samples);
    _attitude.pitch /= static_cast<double>(num_samples);
    _attitude.yaw /= static_cast<double>(num_samples);
}

// Get roll, pitch, and yaw from all MPUXX50 devices
void WMPU::_get_roll_pitch_yaw(double& r, double& p, double& y) {
    for (auto mpu : MPUXX50s) {
        _attitude = mpu->_get_calculated_attitude<Attitude>();
        r = _attitude.roll;
        p = _attitude.pitch;
        y = _attitude.yaw;
    }
}
