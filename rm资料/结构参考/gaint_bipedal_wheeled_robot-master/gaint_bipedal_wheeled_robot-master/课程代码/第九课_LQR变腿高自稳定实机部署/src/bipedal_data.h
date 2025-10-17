#ifndef _BIPEDAL_DATA_h
#define _BIPEDAL_DATA_h
#define SBUSPIN 41 // not completely conform

// int RCValue[6];
#define RCCHANNEL_MIN 600
#define RCCHANNEL_MAX 1400
#define RCCHANNEL3_MIN 200
#define RCCHANNEL3_MID 1000
#define RCCHANNEL3_MAX 1800
// 运动学控制参数结构体
typedef struct
{
    float motorLeftFront;
    float motorLeftRear;
    float motorRightFront;
    float motorRightRear;
    float wheelmotorLeft;
    float wheelmotorRight;
} motorstarget;

typedef struct
{
    float motorLeftFrontangle;
    float motorLeftRearangle;
    float motorRightFrontangle;
    float motorRightRearangle;
    float motorleftsita0_origin;
    float motorleftsita1_origin;
    float motorrightsita0_origin;
    float motorrightsita1_origin;

    float motorleftsita1;
    float motorleftsita2;
    float motorrightsita1;
    float motorrightsita2;
} motorsparam;

typedef struct
{
   // 链长参数
    int l1 = 190; // 第一个关节到第二个关节的长度
    int l2 = 260; // 第二个关节到末端的长度

    // 关节角度
    float sita1 = 0; // 第一个关节角度
    float sita2 = 0; // 第二个关节角度

    // 当前坐标与其他运动学变量
    float l, r;       // 距离和半径
    float x_t, z_t;   // 当前末端坐标
    float X = 0, Z = -180; // 当前关节目标坐标
    float alpha, beta;     // 角度相关变量
    float R;               // 半径或相关量

    float robotl = 312;//机器人宽度
    // 偏导数（运动学雅可比）
    float dx_dsita1, dz_dsita1; // 对sita1的偏导数
    float dx_dsita2, dz_dsita2; // 对sita2的偏导数

    // 力矩和力控变量
    float t1 = 0, t2 = 0; // 力矩变量
    float Fx = 0, Fz = 0; // 作用力

    // 控制增益
    float KP_X = 0.18, KP_Z = 0.105; // 坐标方向的控制增益

    // 期望目标坐标
    float Xe = 0, Ze = -180; 

    // 力矩限制
    float t1_max = 30, t2_max = 30;

} motionControlParams;




#endif //