#include "Kinematics.h"
#include <Arduino.h>



float constrainValue(float value, float minValue, float maxValue)
{
  if (value > maxValue)
    return maxValue;
  if (value < minValue)
    return minValue;
  return value;
}


// 运动学逆解函数 X1x轴左脚  X2x轴右脚   Y1y轴左脚   Y2y轴右脚
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



  alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
  beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);//角度转为正
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 2)
   jointAngles->alphaRight = alpha1;
  else
   jointAngles->alphaRight = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 2)
   jointAngles->betaRight = beta1;
  else
    jointAngles->betaRight = beta2;

  // 左腿逆解运算
  float aLeft = 2 * LeftTarget->x * L1;
  float bLeft = 2 * LeftTarget->y * L1;
  float cLeft = LeftTarget->x * LeftTarget->x + LeftTarget->y * LeftTarget->y + L1 * L1 - L2 * L2;

  float dLeft = 2 * L4 * (LeftTarget->x - L5);
  float eLeft = 2 * L4 * LeftTarget->y;
  float fLeft = ((LeftTarget->x - L5) * (LeftTarget->x - L5) + L4 * L4 + LeftTarget->y * LeftTarget->y - L3 * L3);

  // alpha的计算
  alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
  beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
  

  // 角度解算范围限制
  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 2)
    jointAngles->alphaLeft = alpha1;
  else
    jointAngles->alphaLeft = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 2)
    jointAngles->betaLeft = beta1;
  else
    jointAngles->betaLeft = beta2;

}




// 运动学正解函数
void forwardKinematics(Node * LeftTarget, Node * RightTarget, JointAngles * jointAngles)
{
  float alpha, beta, theta1, theta2, selectedTheta;
  Node A, B, C;
   
  //右腿正解
  alpha = jointAngles->alphaRight;
  beta = jointAngles->betaRight;

  A.x = L1 * cos(alpha);
  A.y = L1 * sin(alpha);
  C.x = L5 + L4 * cos(beta);
  C.y = L4 * sin(beta);
  
  float aRight = 2 * (A.x - C.x) * L2;
  float bRight = 2 * (A.y - C.y) * L2;
  float lRight = sqrt((A.x - C.x) * (A.x - C.x) + (A.y - C.y) * (A.y - C.y));//a点与b点的距离
  float cRight = (L3 * L3 - L2 *L2 - lRight * lRight);
 
  
  
  theta1 = 2 * atan((bRight + sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight)))/(aRight + cRight));
  theta2 = 2 * atan((bRight - sqrt((bRight * bRight) + (aRight * aRight) - (cRight * cRight)))/(aRight + cRight));
  

  // 角度解算范围限制
  theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
  theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);

  if (theta1 >= PI / 2)
  {
    selectedTheta = theta2;
  }
  else
  {
    selectedTheta = theta1;
  }

  RightTarget->x = A.x + L2 * cos(selectedTheta);
  RightTarget->y = A.y + L2 * sin(selectedTheta);


   //左腿正解
   alpha = jointAngles->alphaLeft;
   beta = jointAngles->betaLeft;

   A.x = L1 * cos(alpha);
   A.y = L1 * sin(alpha);
   C.x = L5 + L4 * cos(beta);
   C.y = L4 * sin(beta);
   
   float aLeft = 2 * (A.x - C.x) * L2;
   float bLeft = 2 * (A.y - C.y) * L2;
   float lLeft = sqrt((A.x - C.x) * (A.x - C.x) + (A.y - C.y) * (A.y - C.y));
   float cLeft = (L3 * L3 - L2 *L2 - lLeft * lLeft);
   
   
   
   theta1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft)))/(aLeft + cLeft));
   theta2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft *  cLeft)))/(aLeft + cLeft));
   
 
   // 角度解算范围限制
   theta1 = (theta1 >= 0) ? theta1 : (theta1 + 2 * PI);
   theta2 = (theta2 >= 0) ? theta2 : (theta2 + 2 * PI);
 
   if (theta1 >= PI / 2)
   {
    selectedTheta = theta2;
   }
   else
   {
    selectedTheta = theta1;
   }
   
   LeftTarget->x = A.x + L2 * cos(selectedTheta);
   LeftTarget->y = A.y + L2 * sin(selectedTheta);

}