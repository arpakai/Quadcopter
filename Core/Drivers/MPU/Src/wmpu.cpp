#include "wmpu.h"

WMPU::WMPU(const std::vector<I2C_HandleTypeDef*>& i2c_handles, UART_HandleTypeDef* uart_handles):
    _pUARTx{uart_handles}, _num_devices{0}, _serial_buf{0}, _attitude{0}
{
    _num_devices = i2c_handles.size();
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
            snprintf((char *)_serial_buf, sizeof(_serial_buf), "ERROR!\r\n");
            HAL_UART_Transmit(_pUARTx, _serial_buf, strlen((char *)_serial_buf), HAL_MAX_DELAY);
        }
        while (status != 1);
    }
}

void WMPU::_tune_acc_gyro_offsets() {
    for (auto mpu : MPUXX50s) {
        mpu->_tune_acc_gyro_impl();
    }
}

void WMPU::_print_roll_pitch_yaw(UART_HandleTypeDef& uart_handle)
{
    // Do something
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

// template<typename Filter>
// void WMPU::_get_roll_pitch_yaw(Filter& filter_data) {
//     for (auto mpu : MPUXX50s) {
//         filter_data = mpu->_get_calculated_attitude<Filter>();
//     }
// }
