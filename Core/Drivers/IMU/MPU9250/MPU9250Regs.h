#pragma once

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

constexpr uint32_t I2C_TIMOUT_MS        =100; 

constexpr uint8_t WHO_AM_I_6050_ANS    =0x68;
constexpr uint8_t WHO_AM_I_9250_ANS    =0x71;
constexpr uint8_t WHO_AM_I             =0x75;
constexpr uint8_t AD0_LOW =            =0x68;
constexpr uint8_t AD0_HIGH             =0x69;
constexpr uint8_t XG_OFFSET_H          =0x13;  // User-defined trim values for gyroscope
constexpr uint8_t XG_OFFSET_L          =0x14;
constexpr uint8_t YG_OFFSET_H          =0x15;
constexpr uint8_t YG_OFFSET_L          =0x16;
constexpr uint8_t ZG_OFFSET_H          =0x17;
constexpr uint8_t ZG_OFFSET_L          =0x18;
constexpr uint8_t SMPLRT_DIV           =0x19;
constexpr uint8_t MPU_CONFIG           =0x1A;
constexpr uint8_t GYRO_CONFIG          =0x1B;
constexpr uint8_t ACCEL_CONFIG         =0x1C;
constexpr uint8_t ACCEL_CONFIG2        =0x1D;
constexpr uint8_t LP_ACCEL_ODR         =0x1E;
constexpr uint8_t WOM_THR              =0x1F;
constexpr uint8_t FIFO_EN              =0x23;
constexpr uint8_t I2C_MST_CTRL         =0x24;
constexpr uint8_t I2C_SLV0_ADDR        =0x25;
constexpr uint8_t I2C_SLV0_REG         =0x26;
constexpr uint8_t I2C_SLV0_CTRL        =0x27;
constexpr uint8_t I2C_SLV1_ADDR        =0x28;
constexpr uint8_t I2C_SLV1_REG         =0x29;
constexpr uint8_t I2C_SLV1_CTRL        =0x2A;
constexpr uint8_t I2C_SLV2_ADDR        =0x2B;
constexpr uint8_t I2C_SLV2_REG         =0x2C;
constexpr uint8_t I2C_SLV2_CTRL        =0x2D;
constexpr uint8_t I2C_SLV3_ADDR        =0x2E;
constexpr uint8_t I2C_SLV3_REG         =0x2F;
constexpr uint8_t I2C_SLV3_CTRL        =0x30;
constexpr uint8_t I2C_SLV4_ADDR        =0x31;
constexpr uint8_t I2C_SLV4_REG         =0x32;
constexpr uint8_t I2C_SLV4_DO          =0x33;
constexpr uint8_t I2C_SLV4_CTRL        =0x34;
constexpr uint8_t I2C_SLV4_DI          =0x35;
constexpr uint8_t I2C_MST_STATUS       =0x36;
constexpr uint8_t INT_ENABLE           =0x38;
constexpr uint8_t DMP_INT_STATUS       =0x39;  // Check DMP interrupt
constexpr uint8_t INT_STATUS           =0x3A;
constexpr uint8_t ACCEL_XOUT_H         =0x3B;
constexpr uint8_t ACCEL_XOUT_L         =0x3C;
constexpr uint8_t ACCEL_YOUT_H         =0x3D;
constexpr uint8_t ACCEL_YOUT_L         =0x3E;
constexpr uint8_t ACCEL_ZOUT_H         =0x3F;
constexpr uint8_t ACCEL_ZOUT_L         =0x40;
constexpr uint8_t TEMP_OUT_H           =0x41;
constexpr uint8_t TEMP_OUT_L           =0x42;
constexpr uint8_t GYRO_XOUT_H          =0x43;
constexpr uint8_t GYRO_XOUT_L          =0x44;
constexpr uint8_t GYRO_YOUT_H          =0x45;
constexpr uint8_t GYRO_YOUT_L          =0x46;
constexpr uint8_t GYRO_ZOUT_H          =0x47;
constexpr uint8_t GYRO_ZOUT_L          =0x48;
constexpr uint8_t EXT_SENS_DATA_23     =0x60;
constexpr uint8_t MOT_DETECT_STATUS    =0x61;
constexpr uint8_t I2C_SLV0_DO          =0x63;
constexpr uint8_t I2C_SLV1_DO          =0x64;
constexpr uint8_t I2C_SLV2_DO          =0x65;
constexpr uint8_t I2C_SLV3_DO          =0x66;
constexpr uint8_t I2C_MST_DELAY_CTRL   =0x67;
constexpr uint8_t SIGNAL_PATH_RESET    =0x68;
constexpr uint8_t MOT_DETECT_CTRL      =0x69;
constexpr uint8_t USER_CTRL            =0x6A;    // Bit 7 enable DMP, bit 3 reset DMP
constexpr uint8_t PWR_MGMT_1           =0x6B;    // Device defaults to the SLEEP mode
constexpr uint8_t PWR_MGMT_2           =0x6C;
constexpr uint8_t DMP_BANK             =0x6D;    // Activates a specific bank in the DMP
constexpr uint8_t DMP_RW_PNT           =0x6E;    // Set read/write pointer to a specific start address in specified DMP bank
constexpr uint8_t DMP_REG              =0x6F;    // Register in DMP from which to read or to which to write
constexpr uint8_t DMP_REG_1            =0x70;
constexpr uint8_t DMP_REG_2            =0x71;
constexpr uint8_t FIFO_COUNTH          =0x72;
constexpr uint8_t FIFO_COUNTL          =0x73;
constexpr uint8_t FIFO_R_W             =0x74;
constexpr uint8_t WHO_AM_I_MPU9250     =0x75;    // Should return 0x71
constexpr uint8_t XA_OFFSET_H          =0x77;
constexpr uint8_t XA_OFFSET_L          =0x78;
constexpr uint8_t YA_OFFSET_H          =0x7A;
constexpr uint8_t YA_OFFSET_L          =0x7B;
constexpr uint8_t ZA_OFFSET_H          =0x7D;
constexpr uint8_t ZA_OFFSET_L          =0x7E;

// Magnetometer Registers
constexpr uint8_t AK8963_ADDRESS       =0x0C << 1;
constexpr uint8_t AK8963_WHO_AM_I      =0x00; // should return 0x48
constexpr uint8_t AK8963_INFO          =0x01;
constexpr uint8_t AK8963_ST1           =0x02;    // data ready status bit 0
constexpr uint8_t AK8963_XOUT_L        =0x03; // data
constexpr uint8_t AK8963_XOUT_H        =0x04;
constexpr uint8_t AK8963_YOUT_L        =0x05;
constexpr uint8_t AK8963_YOUT_H        =0x06;
constexpr uint8_t AK8963_ZOUT_L        =0x07;
constexpr uint8_t AK8963_ZOUT_H        =0x08;
constexpr uint8_t AK8963_ST2           =0x09;    // Data overflow bit 3 and data read error status bit 2
constexpr uint8_t AK8963_CNTL          =0x0A;   // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
constexpr uint8_t AK8963_ASTC          =0x0C;   // Self test control
constexpr uint8_t AK8963_I2CDIS        =0x0F; // I2C disable
constexpr uint8_t AK8963_ASAX          =0x10;   // Fuse ROM x-axis sensitivity adjustment value
constexpr uint8_t AK8963_ASAY          =0x11;   // Fuse ROM y-axis sensitivity adjustment value
constexpr uint8_t AK8963_ASAZ          =0x12;   // Fuse ROM z-axis sensitivity adjustment value
constexpr uint8_t INT_PIN_CFG          =0x37;
constexpr uint8_t DATA_READY_MASK      =0x01;
constexpr uint8_t MAGIC_OVERFLOW_MASK  =0x8;