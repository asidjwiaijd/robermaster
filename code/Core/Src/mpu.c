#include "mpu.h"
#include "string.h"
// 设备地址与寄存器定义
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV  0x19    // 采样率分频寄存器
#define CONFIG      0x1A    // 低通滤波配置寄存器
#define GYRO_CONFIG 0x1B    // 陀螺仪配置寄存器
#define ACCEL_CONFIG 0x02   // 加速度计配置：4G量程（原0x01改为0x02，对应4G）
#define ACCEL_XOUT_H 0x3B   // 加速度X轴高位寄存器
#define ACCEL_XOUT_L 0x3C   // 加速度X轴低位寄存器
#define ACCEL_YOUT_H 0x3D   // 加速度Y轴高位寄存器
#define ACCEL_YOUT_L 0x3E   // 加速度Y轴低位寄存器
#define ACCEL_ZOUT_H 0x3F   // 加速度Z轴高位寄存器
#define ACCEL_ZOUT_L 0x40   // 加速度Z轴低位寄存器
#define TEMP_OUT_H   0x41   // 温度高位寄存器
#define TEMP_OUT_L   0x42   // 温度低位寄存器
#define GYRO_XOUT_H  0x43   // 陀螺仪X轴高位寄存器
#define GYRO_XOUT_L  0x44   // 陀螺仪X轴低位寄存器
#define GYRO_YOUT_H  0x45   // 陀螺仪Y轴高位寄存器
#define GYRO_YOUT_L  0x46   // 陀螺仪Y轴低位寄存器
#define GYRO_ZOUT_H  0x47   // 陀螺仪Z轴高位寄存器
#define GYRO_ZOUT_L  0x48   // 陀螺仪Z轴低位寄存器
#define PWR_MGMT_1   0x6B   // 电源管理寄存器

// 物理单位转换系数（适配4G量程）
#define ACCEL_SCALE_4G 8192.0f  // 4G量程：1G = 8192 LSB（MPU6050手册标准值）
#define GRAVITY        9.80665f // 重力加速度（m/s²）
#define GYRO_SCALE     16.4f    // 陀螺仪2000deg/s量程：1deg/s = 16.4 LSB
#define TEMP_OFFSET    36.53f   // 温度基准偏移（℃）
#define TEMP_SCALE     340.0f   // 温度转换系数（LSB/℃）

int16_t gyro_offset_x = 0;
int16_t gyro_offset_y = 0;
int16_t gyro_offset_z = 0;


void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    // 检查MPU6050设备ID（WHO_AM_I寄存器，地址0x75）
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    if (check == 104)  // 104为MPU6050默认ID，确认设备连接正常
    {
        // 唤醒传感器：PWR_MGMT_1寄存器写0，解除睡眠模式
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);

        // 设置采样率：SMPLRT_DIV=0x07，采样率=1000/(1+7)=125Hz
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, 1000);

        // 配置低通滤波器：CONFIG=0x02，截止频率45Hz（与之前需求一致）
        Data = 0x02;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG, 1, &Data, 1, 1000);

        // 配置陀螺仪：GYRO_CONFIG=0x18，量程2000deg/s（保持不变）
        Data = 0x18;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, 1000);

        // 配置加速度计：ACCEL_CONFIG=0x02，量程4G（核心修改点）
        Data = 0x02;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, 1000);
    }
}



/**
 * @brief 读取加速度（4G量程，转换为m/s²）
 * @param Ax/Ay/Az：输出X/Y/Z轴加速度值（单位：m/s²）
 */
void MPU6050_Read_Accel(float* Ax, float* Ay, float* Az)
{
    uint8_t Rec_Data[6];
    int16_t accel_raw[3];  // 暂存16位原始加速度数据

    // 读取6字节加速度原始数据（从ACCEL_XOUT_H开始，连续读取X/Y/Z轴）
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);

    // 组合高低字节，处理16位有符号数据（MPU6050数据为大端格式）
    accel_raw[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);  // X轴原始值
    accel_raw[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);  // Y轴原始值
    accel_raw[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);  // Z轴原始值

    // 转换为物理单位m/s²：原始值 / 4G量程系数 * 重力加速度
    *Ax = (accel_raw[0] / (ACCEL_SCALE_4G *2)) * GRAVITY;
    *Ay = (accel_raw[1] / (ACCEL_SCALE_4G *2)) * GRAVITY;
    *Az = (accel_raw[2] / (ACCEL_SCALE_4G *2)) * GRAVITY;
}


/**
 * @brief 读取校准后的角速度（2000deg/s量程，转换为deg/s）
 * @param Gx/Gy/Gz：输出X/Y/Z轴校准后角速度值（单位：deg/s）
 * @note 需先调用MPU6050_Gyro_Calibrate()完成校准，否则偏移量为0（无校准效果）
 */
void MPU6050_Read_Gyro(float* Gx, float* Gy, float* Gz)
{
    uint8_t Rec_Data[6];
    int16_t gyro_raw[3];  // 暂存16位原始角速度数据

    // 1. 读取陀螺仪原始数据（
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, 1000);
    gyro_raw[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);  // X轴原始值
    gyro_raw[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);  // Y轴原始值
    gyro_raw[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);  // Z轴原始值

    // 2.原始值 - 零漂偏移量
    int16_t gyro_calibrated_x = gyro_raw[0] - gyro_offset_x;
    int16_t gyro_calibrated_y = gyro_raw[1] - gyro_offset_y;
    int16_t gyro_calibrated_z = gyro_raw[2] - gyro_offset_z;

    // 3. 转换为物理单位deg/s
    *Gx = gyro_calibrated_x / GYRO_SCALE;
    *Gy = gyro_calibrated_y / GYRO_SCALE;
    *Gz = gyro_calibrated_z / GYRO_SCALE;
}

/**
 * @brief 读取温度（转换为℃）
 * @param Temp：输出温度值（单位：℃）
 */
void MPU6050_Read_Temp(float* Temp)
{
    uint8_t Rec_Data[2];
    int16_t temp_raw;  // 暂存16位原始温度数据

    // 读取2字节温度原始数据（从TEMP_OUT_H开始）
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, 1000);

    // 组合高低字节，处理16位有符号数据
    temp_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    // 转换为物理单位℃（公式来自MPU6050手册）
    *Temp = (temp_raw / TEMP_SCALE) + TEMP_OFFSET;
}


/**
 * @brief 串口输出所有传感器数据（带物理单位，适配4G量程）
 * @param huart：目标串口句柄（如&huart1）
 */

/*
void MPU6050_Printf_All(UART_HandleTypeDef* huart)
{
    float Ax, Ay, Az;
    float Gx, Gy, Gz;
    float Temp;
    char buffer[200];

    // 读取转换后的4G量程加速度、角速度、温度数据
    MPU6050_Read_Accel(&Ax, &Ay, &Az);
    MPU6050_Read_Gyro(&Gx, &Gy, &Gz);
    MPU6050_Read_Temp(&Temp);

    // 格式化输出（保留2位小数，数据直观易读）
    sprintf(buffer, "Accel(4G, m/s²): Ax=%.2f, Ay=%.2f, Az=%.2f; "
                    "Gyro(deg/s): Gx=%.2f, Gy=%.2f, Gz=%.2f; "
                    "Temp(℃): %.2f\r\n",
            Ax, Ay, Az, Gx, Gy, Gz, Temp);

    // 串口发送（strlen确保仅发送有效字符，避免乱码）
    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

*/

/**
 * @brief 陀螺仪零漂校准（需将传感器保持绝对静止）
 * @param sample_num：校准采样次数（建议500-2000，次数越多精度越高，耗时越长）
 * @note 校准过程中禁止移动传感器，校准后偏移量存入全局变量gyro_offset_x/y/z
 */
void MPU6050_Gyro_Calibrate(uint16_t sample_num)
{
    uint8_t Rec_Data[6];
    int32_t gyro_sum_x = 0;  // 用32位变量存总和（避免16位溢出）
    int32_t gyro_sum_y = 0;
    int32_t gyro_sum_z = 0;
    int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;

    // 1. 等待传感器稳定（初始化后延迟100ms，避免瞬时波动）
    HAL_Delay(40);

    // 2. 采集sample_num组原始数据，累加求和
    for (uint16_t i = 0; i < sample_num; i++)
    {
        // 读取陀螺仪原始数据（6字节：X_H、X_L、Y_H、Y_L、Z_H、Z_L）
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, 1000);
        
        // 组合高低字节，得到16位原始值
        gyro_raw_x = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        gyro_raw_y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        gyro_raw_z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        
        // 累加各轴原始值（用32位变量避免溢出，如2000组×32767=65534000 < 2^31）
        gyro_sum_x += gyro_raw_x;
        gyro_sum_y += gyro_raw_y;
        gyro_sum_z += gyro_raw_z;
        
        // 等待采样间隔（采样率125Hz，间隔8ms；可根据实际采样率调整）
        HAL_Delay(5);
    }

    // 3. 计算平均值，作为零漂偏移量（四舍五入取整）
    gyro_offset_x = (int16_t)(gyro_sum_x / sample_num);
    gyro_offset_y = (int16_t)(gyro_sum_y / sample_num);
    gyro_offset_z = (int16_t)(gyro_sum_z / sample_num);
}