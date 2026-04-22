#include "bmi088.h"
#include "stm32f4xx_hal_spi.h"

#define BMI088_SPI_HANDLE (&hspi1)

#define BMI088_SPI_TIMEOUT_MS 20U

#define BMI088_READ_BIT 0x80U
#define BMI088_WRITE_BIT 0x7FU

#define BMI088_ACC_CHIP_ID_ADDR      0x00U
#define BMI088_ACC_DATA_START_ADDR   0x12U
#define BMI088_ACC_CONF_ADDR         0x40U
#define BMI088_ACC_RANGE_ADDR        0x41U
#define BMI088_ACC_PWR_CONF_ADDR     0x7CU
#define BMI088_ACC_PWR_CTRL_ADDR     0x7DU
#define BMI088_ACC_SOFTRESET_ADDR    0x7EU

#define BMI088_GYRO_CHIP_ID_ADDR     0x00U
#define BMI088_GYRO_DATA_START_ADDR  0x02U
#define BMI088_GYRO_RANGE_ADDR       0x0FU
#define BMI088_GYRO_BANDWIDTH_ADDR   0x10U
#define BMI088_GYRO_SOFTRESET_ADDR   0x14U

#define BMI088_ACC_CHIP_ID_VALUE  0x1EU
#define BMI088_GYRO_CHIP_ID_VALUE 0x0FU

#define BMI088_SOFTRESET_CMD 0xB6U


// cs片选封装函数，增加可读性
static void bmi088_acc_cs_low(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

static void bmi088_acc_cs_high(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

static void bmi088_gyro_cs_low(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

static void bmi088_gyro_cs_high(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}


/* SPI写封装函数 */

static BMI088_Status_e bmi088_spi_write_acc(uint8_t reg, uint8_t value)
{
    // 确保寄存器地址在有效范围内（0x00-0x7F）[清除最高位bit7，确保是写操作]
    uint8_t tx[2] = {reg & BMI088_WRITE_BIT, value};

    bmi088_acc_cs_low();
    if (HAL_SPI_Transmit(BMI088_SPI_HANDLE, tx, sizeof(tx), BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        // 发送失败，拉高片选引脚并返回错误状态
        bmi088_acc_cs_high();
        return BMI088_ERR_SPI;
    }
    bmi088_acc_cs_high();
    return BMI088_OK;
}

static BMI088_Status_e bmi088_spi_write_gyro(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7FU, value};

    bmi088_gyro_cs_low();
    if (HAL_SPI_Transmit(BMI088_SPI_HANDLE, tx, sizeof(tx), BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_gyro_cs_high();
        return BMI088_ERR_SPI;
    }
    bmi088_gyro_cs_high();
    return BMI088_OK;
}

/* SPI读封装函数，实际获取BMI088的原始数据 */

/* ACC SPI read returns one dummy byte before valid data per datasheet. */
static BMI088_Status_e bmi088_spi_read_acc(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr = reg | BMI088_READ_BIT;
    uint8_t rx[7] = {0};
    uint8_t tx_dummy[7] = {0};

    // 确保读取长度不超过6字节（包含一个dummy字节）
    if (len > 6U)
    {
        return BMI088_ERR_SPI;
    }

    bmi088_acc_cs_low();
    if (HAL_SPI_Transmit(BMI088_SPI_HANDLE, &addr, 1U, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_acc_cs_high();
        return BMI088_ERR_SPI;
    }

    /*在进行加速度计部分的读操作时，请求的数据不会立即发送，而是首先发送一个虚拟字节，
    然后在这个虚拟字节之后才传输实际请求的寄存器内容其中第一个接收到的字节可以丢弃，
    而第二个字节包含所需数据。突发读操作也是如此。
    要在 SPI 模式下读取加速度计值，用户必须从地址 0x12（ACC 数据）开始读取 7 个字节。
    在这些字节中，用户必须丢弃第一个字节，并在字节#2 – #7 中找到加速度信息
    （对应于地址 0x12 – 0x17 的内容）。*/
    if (HAL_SPI_TransmitReceive(BMI088_SPI_HANDLE,
                                tx_dummy,
                                rx,
                                (uint16_t)(len + 1U),
                                BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_acc_cs_high();
        return BMI088_ERR_SPI;
    }
    bmi088_acc_cs_high();

    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = rx[i + 1U];
    }

    return BMI088_OK;
}

static BMI088_Status_e bmi088_spi_read_gyro(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr = reg | BMI088_READ_BIT;
    uint8_t tx_dummy[6] = {0};

    if (len > 6U)
    {
        return BMI088_ERR_SPI;
    }

    bmi088_gyro_cs_low();
    if (HAL_SPI_Transmit(BMI088_SPI_HANDLE, &addr, 1U, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_gyro_cs_high();
        return BMI088_ERR_SPI;
    }

    if (HAL_SPI_TransmitReceive(BMI088_SPI_HANDLE,
                                tx_dummy,
                                data,
                                len,
                                BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_gyro_cs_high();
        return BMI088_ERR_SPI;
    }
    bmi088_gyro_cs_high();

    return BMI088_OK;
}

// 合并8位数据为16位数据，并转换为有符号整数
// static int16_t Conver(uint8_t lsb, uint8_t msb)
// {
//     return (int16_t)((uint16_t)lsb | ((uint16_t)msb << 8U));
// }

BMI088_Status_e BMI088_Init(void)
{
 
    uint8_t acc_id = 0U;
    uint8_t gyro_id = 0U;

    bmi088_acc_cs_high();
    bmi088_gyro_cs_high();
    HAL_Delay(2U);

    /* First ACC read is used to switch ACC interface into SPI mode. */
    // bmi088_spi_read_acc(BMI088_ACC_CHIP_ID_ADDR, &acc_id, 1U);

    /* 软重置加速度计 */
    bmi088_spi_write_acc(BMI088_ACC_SOFTRESET_ADDR, BMI088_SOFTRESET_CMD);
    HAL_Delay(2U);

    /* 软重置陀螺仪 */
    bmi088_spi_write_gyro(BMI088_GYRO_SOFTRESET_ADDR, BMI088_SOFTRESET_CMD);
    HAL_Delay(2U);

    /* 配置加速度计电源模式 */
    bmi088_spi_write_acc(BMI088_ACC_PWR_CONF_ADDR, 0x00U);

    /* 使能加速度计 */
    bmi088_spi_write_acc(BMI088_ACC_PWR_CTRL_ADDR, 0x04U);
    
    /* 设置加速度计量程为 ±3g */
    bmi088_spi_write_acc(BMI088_ACC_RANGE_ADDR, 0x00U);

    /*ODR/BW配置*/
    bmi088_spi_write_acc(BMI088_ACC_CONF_ADDR, 0xACU);
    
    /* 配置陀螺仪：±2000dps 量程 */
    bmi088_spi_write_gyro(BMI088_GYRO_RANGE_ADDR, 0x00U);

    /* 设置陀螺仪带宽为 200Hz */
    bmi088_spi_write_gyro(BMI088_GYRO_BANDWIDTH_ADDR, 0x02U);

    /* 验证加速度计芯片ID */
    bmi088_spi_read_acc(BMI088_ACC_CHIP_ID_ADDR, &acc_id, 1U);
    if (acc_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_ERR_ACC_CHIP_ID;
    }

    /* 验证陀螺仪芯片ID */
    bmi088_spi_read_gyro(BMI088_GYRO_CHIP_ID_ADDR, &gyro_id, 1U);
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_ERR_GYRO_CHIP_ID;
    }

    return BMI088_OK;
}
