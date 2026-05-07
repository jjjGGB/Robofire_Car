#include "turn_on_wheeltec_robot/Quaternion_Solution.h"

/**
 * @brief 采样频率，需与底盘实际上传数据的频率保持一致
 */
#define SAMPLING_FREQ 20.0f 

/**
 * @brief 经典的快速平方根倒数算法 (Fast Inverse Square Root)
 * @details 通过位运算快速逼近 1/sqrt(x)，用于四元数归一化处理，提高运算效率。
 * @param number 需要求平方根倒数的浮点数
 * @return float 运算结果 (1/sqrt(number))
 */
float InvSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 ); // 著名的魔术数字
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) ); // 牛顿迭代法提高精度

    return y;
}

/**
 * @brief Mahony互补滤波算法相关全局变量
 */
volatile float twoKp = 1.0f;          // 比例增益 (用于纠正陀螺仪漂移)
volatile float twoKi = 0.0f;          // 积分增益 (用于消除静差)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 初始四元数
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // 积分误差项

/**
 * @brief 四元数姿态解算函数 (Mahony算法)
 * @details 通过加速度计修正陀螺仪的积分误差。
 * @param gx, gy, gz 陀螺仪角速度数据 (单位: rad/s)
 * @param ax, ay, az 加速度计数据 (单位: m/s^2 或 g)
 */
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 仅当加速度计测量值有效时进行反馈修正，避免除零错误
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // 1. 加速度计向量归一化
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;      

        // 2. 提取当前四元数姿态下的重力分量 (机体坐标系下的估计重力向量)
        // 这是方向余弦矩阵 R(q) 的第三行元素
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 3. 误差测量：估计重力向量与测量重力向量的叉积
        // 误差 e = v ⊗ a，反映了当前姿态与重力方向的偏差
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 4. 计算积分反馈项 (如果启用 Ki)
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ);
            integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
            integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
            gx += integralFBx; // 应用积分反馈修正角速度
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // 积分重置，防止积分饱和
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 5. 应用比例反馈修正角速度 (Kp)
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 6. 对四元数进行微分方程积分更新姿态
    // 计算时间步长内的变化量
    gx *= (0.5f * (1.0f / SAMPLING_FREQ));
    gy *= (0.5f * (1.0f / SAMPLING_FREQ));
    gz *= (0.5f * (1.0f / SAMPLING_FREQ));
    
    qa = q0;
    qb = q1;
    qc = q2;
    
    // 一阶 Runge-Kutta 积分更新四元数
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 

    // 7. 四元数归一化，确保模长为 1 (代表纯旋转)
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 8. 将结果存入全局 IMU 消息结构体
    Mpu6050.orientation.w = q0;
    Mpu6050.orientation.x = q1;
    Mpu6050.orientation.y = q2;
    Mpu6050.orientation.z = q3;
}
