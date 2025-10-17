#include "Kinematics.h"
#include <Arduino.h>

extern MIT devicesState[4];

float constrainValue(float value, float minValue, float maxValue)
{
  if (value > maxValue)
    return maxValue;
  if (value < minValue)
    return minValue;
  return value;
}


float safe_sqrt(float x) {
    return x >= 0.0f ? sqrtf(x) : 0.0f;
}

float safe_div(float num, float denom) {
    if (fabs(denom) < 1e-6f) denom = (denom < 0 ? -1e-6f : 1e-6f); // 防止分母0
    return num / denom;
}

void inverseKinematics(Node * LeftTarget, Node * RightTarget ,JointAngles * jointAngles)
{
    float alpha1, alpha2, beta1, beta2;
    LeftTarget->y = constrainValue(LeftTarget->y, 110, 300);
    RightTarget->y = constrainValue(RightTarget->y, 110, 300);

    // 右腿逆解运算
    float aRight = 2 * RightTarget->x * L1;
    float bRight = 2 * RightTarget->y * L1;
    float cRight = RightTarget->x * RightTarget->x + RightTarget->y * RightTarget->y + L1 * L1 - L2 * L2;
    float dRight = 2 * L4 * (RightTarget->x - L5);
    float eRight = 2 * L4 * RightTarget->y;
    float fRight = ((RightTarget->x - L5) * (RightTarget->x - L5) + L4 * L4 + RightTarget->y * RightTarget->y - L3 * L3);

    float sqrtAlpha = safe_sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight));
    float denomAlpha = aRight + cRight;
    alpha1 = 2 * atan(safe_div(bRight + sqrtAlpha, denomAlpha));
    alpha2 = 2 * atan(safe_div(bRight - sqrtAlpha, denomAlpha));

    float sqrtBeta = safe_sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight));
    float denomBeta = dRight + fRight;
    beta1 = 2 * atan(safe_div(eRight + sqrtBeta, denomBeta));
    beta2 = 2 * atan(safe_div(eRight - sqrtBeta, denomBeta));

    // 角度转为正
    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    // 右腿角度赋值（加健壮性检查）
    if (!isnan(alpha1) && !isinf(alpha1) && alpha1 >= 0 && alpha1 <= PI)
        jointAngles->alphaRight = alpha1;
    else if (!isnan(alpha2) && !isinf(alpha2) && alpha2 >= 0 && alpha2 <= PI)
        jointAngles->alphaRight = alpha2;

    if (!isnan(beta1) && !isinf(beta1) && beta1 >= 0 && beta1 <= PI / 2)
        jointAngles->betaRight = beta1;
    else if (!isnan(beta2) && !isinf(beta2) && beta2 >= 0 && beta2 <= PI / 2)
        jointAngles->betaRight = beta2;

    // 左腿逆解运算
    float aLeft = 2 * LeftTarget->x * L1;
    float bLeft = 2 * LeftTarget->y * L1;
    float cLeft = LeftTarget->x * LeftTarget->x + LeftTarget->y * LeftTarget->y + L1 * L1 - L2 * L2;

    float dLeft = 2 * L4 * (LeftTarget->x - L5);
    float eLeft = 2 * L4 * LeftTarget->y;
    float fLeft = ((LeftTarget->x - L5) * (LeftTarget->x - L5) + L4 * L4 + LeftTarget->y * LeftTarget->y - L3 * L3);

    float sqrtAlphaL = safe_sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft));
    float denomAlphaL = aLeft + cLeft;
    alpha1 = 2 * atan(safe_div(bLeft + sqrtAlphaL, denomAlphaL));
    alpha2 = 2 * atan(safe_div(bLeft - sqrtAlphaL, denomAlphaL));

    float sqrtBetaL = safe_sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft));
    float denomBetaL = dLeft + fLeft;
    beta1 = 2 * atan(safe_div(eLeft + sqrtBetaL, denomBetaL));
    beta2 = 2 * atan(safe_div(eLeft - sqrtBetaL, denomBetaL));

    // 角度转为正
    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    if (!isnan(alpha1) && !isinf(alpha1) && alpha1 >= 0 && alpha1 <= PI)
        jointAngles->alphaLeft = alpha1;
    else if (!isnan(alpha2) && !isinf(alpha2) && alpha2 >= 0 && alpha2 <= PI)
        jointAngles->alphaLeft = alpha2;

    if (!isnan(beta1) && !isinf(beta1) && beta1 >= 0 && beta1 <= PI / 2)
        jointAngles->betaLeft = beta1;
    else if (!isnan(beta2) && !isinf(beta2) && beta2 >= 0 && beta2 <= PI / 2)
        jointAngles->betaLeft = beta2;
    

}


// 运动学正解函数
void forwardKinematics(Node *LeftTarget, Node *RightTarget, JointAngles *jointAngles)
{
    float  theta1, theta2;
    Node A, C;
    

    //右腿解算 //
  jointAngles->alphaRight = (3*PI/4) - devicesState[2].pos/8  ;
  jointAngles->betaRight  = (3*PI/4) - devicesState[0].pos/8 ;


    A.x = L1 * cosf(jointAngles->alphaRight);
    A.y = L1 * sinf(jointAngles->alphaRight);
    C.x = L5 + L4 * cosf(jointAngles->betaRight);
    C.y = L4 * sinf(jointAngles->betaRight);



  float aRight = 2 * (C.x - A.x) * L2;
  float bRight = 2 * (C.y - A.y) * L2;
  float lRight = sqrt((C.x - A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y));//a点与b点的距离
  float cRight = L2 * L2 + lRight * lRight - L3 * L3;
 
  
  
  theta1 = 2 * atan((bRight + sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight)))/(aRight + cRight));
  theta2 = 2 * atan((bRight - sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight)))/(aRight + cRight));
  

  // 角度解算范围限制
  theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
  theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);

  if (theta1 >= PI / 2)
  {
    jointAngles->thetaRight1 = theta2;
  }
  else
  {
    jointAngles->thetaRight1 = theta1;
  }

  RightTarget->x = A.x + L2 * cos(jointAngles->thetaRight1);
  RightTarget->y = A.y + L2 * sin(jointAngles->thetaRight1);
  RightTarget->polarAngle = atan2f(RightTarget->y, RightTarget->x - 0.5f * L5);
  RightTarget->radius = sqrtf((RightTarget->x - 0.5f * L5) * (RightTarget->x - 0.5f * L5) + RightTarget->y * RightTarget->y);

    

  //左腿  

  jointAngles->alphaLeft = (3*PI/4)- devicesState[3].pos/8 ;
  jointAngles->betaLeft = (3*PI/4)- devicesState[1].pos/8  ;

  A.x = L1 * cosf(jointAngles->alphaLeft);
  A.y = L1 * sinf(jointAngles->alphaLeft);
  C.x = L5 + L4 * cosf(jointAngles->betaLeft);
  C.y = L4 * sinf(jointAngles->betaLeft);


  float aLeft = 2 * (C.x - A.x) * L2;
  float bLeft = 2 * (C.y - A.y) * L2;
  float lLeft = sqrt((C.x -A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y));
  float cLeft = L2 * L2 + lLeft * lLeft - L3 * L3;

  theta1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft)))/(aLeft + cLeft));
  theta2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft *  cLeft)))/(aLeft + cLeft));
   
 
  // 角度解算范围限制
  theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
  theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);
 
  if (theta1 >= PI / 2)
  {
    jointAngles->thetaLeft1 = theta2;
  }
  else
  {
    jointAngles->thetaLeft1 = theta1;
  }
   
  LeftTarget->x = A.x + L2 * cos(jointAngles->thetaLeft1);
  LeftTarget->y = A.y + L2 * sin(jointAngles->thetaLeft1);
  LeftTarget->polarAngle = atan2f(LeftTarget->y, LeftTarget->x - 0.5f * L5);
  LeftTarget->radius = sqrtf((LeftTarget->x - 0.5f * L5) * (LeftTarget->x - 0.5f * L5) + LeftTarget->y * LeftTarget->y);
}

void forwardKinematics_right(Node *RightTarget, JointAngles *jointAngles)
{
  float  theta1, theta2;
    Node A, C;
    
  //右腿解算 //
  // jointAngles->alphaRight = (3*PI/2) - devicesState[2].pos/8  ;
  // jointAngles->betaRight  = (3*PI/2) - devicesState[0].pos/8 ;
  
  jointAngles->alphaRight = (PI/2)+2.87/8 - devicesState[2].pos/8  ;
  jointAngles->betaRight  = (PI/2)+2.67/8 - devicesState[0].pos/8 ;
  
  

  A.x = L1 * cosf(jointAngles->alphaRight);
  A.y = L1 * sinf(jointAngles->alphaRight);
  C.x = L5 + L4 * cosf(jointAngles->betaRight);
  C.y = L4 * sinf(jointAngles->betaRight);



  float aRight = 2 * (C.x - A.x) * L2;
  float bRight = 2 * (C.y - A.y) * L2;
  float lRight = sqrt((C.x - A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y));//a点与b点的距离
  float cRight = L2 * L2 + lRight * lRight - L3 * L3;
 
  
  
  theta1 = 2 * atan((bRight + sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight)))/(aRight + cRight));
  theta2 = 2 *atan((bRight - sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight)))/(aRight + cRight));
  

  // 角度解算范围限制
  theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
  theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);


  if (theta1 >= PI / 2)
  {
    jointAngles->thetaRight1 = theta2;
  }
  else
  {
    jointAngles->thetaRight1 = theta1;
  }

  RightTarget->x = A.x + L2 * cos(jointAngles->thetaRight1);
  RightTarget->y = A.y + L2 * sin(jointAngles->thetaRight1);
  RightTarget->polarAngle = atan2f(RightTarget->y, RightTarget->x - 0.5f * L5);
  RightTarget->radius = sqrtf((RightTarget->x - 0.5f * L5) * (RightTarget->x - 0.5f * L5) + RightTarget->y * RightTarget->y);


}

// void forwardKinematics_right(Node *RightTarget, JointAngles *jointAngles)
// {
// float  theta1, theta2;
//   Node A, C;
//   //左腿  
//   jointAngles->alphaRight = (PI/2)+2.87/8 - devicesState[2].pos/8  ;
//   jointAngles->betaRight  = (PI/2)+2.67/8 - devicesState[0].pos/8 ;

//   // jointAngles->alphaLeft = (3*PI/4)- devicesState[3].pos/8 ;
//   // jointAngles->betaLeft = (3*PI/4)- devicesState[1].pos/8  ;

//   A.x = L1 * cosf(jointAngles->alphaRight);
//   A.y = L1 * sinf(jointAngles->alphaRight);
//   C.x = L5 + L4 * cosf(jointAngles->betaRight);
//   C.y = L4 * sinf(jointAngles->betaRight);


//   float aRight = 2 * (C.x - A.x) * L2;
//   float bRight = 2 * (C.y - A.y) * L2;
//   float lRight = sqrt((C.x -A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y));
//   float cRight = L2 * L2 + lRight * lRight - L3 * L3;

//   theta1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight)))/(aRight + cRight));
//   theta2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight *  cRight)))/(aRight + cRight));
   

//   // 角度解算范围限制
//   theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
//   theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);

//   if (theta1 >= PI / 2)
//   {
//     jointAngles->thetaRight1 = theta2;
//   }
//   else
//   {
//     jointAngles->thetaRight1 = theta1;
//   }
   
//   RightTarget->x = A.x + L2 * cos(jointAngles->thetaRight1);
//   RightTarget->y = A.y + L2 * sin(jointAngles->thetaRight1);
//   RightTarget->polarAngle = atan2f(RightTarget->y, RightTarget->x - 0.5f * L5);
//   RightTarget->radius = sqrtf((RightTarget->x - 0.5f * L5) * (RightTarget->x - 0.5f * L5) + RightTarget->y * RightTarget->y);


// }


void forwardKinematics_left(Node *LeftTarget, JointAngles *jointAngles)
{
  float  theta1, theta2;
  Node A, C;
  //左腿  

  jointAngles->alphaLeft = (PI/2) + 4.03/8- devicesState[3].pos/8 ;
  jointAngles->betaLeft = 1.83/8 + (PI/2)- devicesState[1].pos/8  ;

  // jointAngles->alphaLeft = (3*PI/4)- devicesState[3].pos/8 ;
  // jointAngles->betaLeft = (3*PI/4)- devicesState[1].pos/8  ;

  A.x = L1 * cosf(jointAngles->alphaLeft);
  A.y = L1 * sinf(jointAngles->alphaLeft);
  C.x = L5 + L4 * cosf(jointAngles->betaLeft);
  C.y = L4 * sinf(jointAngles->betaLeft);


  float aLeft = 2 * (C.x - A.x) * L2;
  float bLeft = 2 * (C.y - A.y) * L2;
  float lLeft = sqrt((C.x -A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y));
  float cLeft = L2 * L2 + lLeft * lLeft - L3 * L3;

  theta1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft)))/(aLeft + cLeft));
  theta2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft *  cLeft)))/(aLeft + cLeft));
   

  // 角度解算范围限制
  theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
  theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);

  if (theta1 >= PI / 2)
  {
    jointAngles->thetaLeft1 = theta2;
  }
  else
  {
    jointAngles->thetaLeft1 = theta1;
  }
   
  LeftTarget->x = A.x + L2 * cos(jointAngles->thetaLeft1);
  LeftTarget->y = A.y + L2 * sin(jointAngles->thetaLeft1);
  LeftTarget->polarAngle = atan2f(LeftTarget->y, LeftTarget->x - 0.5f * L5);
  LeftTarget->radius = sqrtf((LeftTarget->x - 0.5f * L5) * (LeftTarget->x - 0.5f * L5) + LeftTarget->y * LeftTarget->y);

}