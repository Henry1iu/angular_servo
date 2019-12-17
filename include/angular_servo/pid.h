#ifndef _PID_H
#define _PID_H

/* 
    EXAMPLE:
  
    float kp = 10;
    float kd = 0.2;
    float ki = 7;
    float deadband = 0.0;
    float u_min = -0.3;
    float u_max = 0.6;
    float e_int_min  = -0.1;
    float e_int_max = 0.1;
    float dt =0.02;
    
    float linear_vel;

    PID xy_PID(kp, ki, kd, deadband, u_min, u_max, e_int_min, e_int_max, dt);
    linear_vel = xy_PID.PID_CalcOutput(YOUR_ERROR);
    
 */

class PID
{
private:
    float mKp;
    float mKi;
    float mKd;
    float mDeadband;
    float mU_min;
    float mU_max;
    float mE_int_min;
    float mE_int_max;
    float mDt;
    
    float mE_cur;
    float mE_old;
    float mE_int;
    float mE_der;
    float mRef;
    float mU;
    
public:
    PID(float kp, float ki, float kd, float deadband, 
	     float u_min, float u_max, 
	     float e_int_min, float e_int_max, float dt);
    
    float PID_CalcOutput(float error);
    
};





PID::PID(float kp, float ki, float kd, float deadband, 
	      float u_min, float u_max, 
	      float e_int_min, float e_int_max, float dt)
{
    mE_cur = 0.0;
    mE_old = 0.0;
    mE_int = 0.0;
    mE_der = 0.0;
    mRef = 0.0;
    mU = 0.0;
    
    mKp = kp;
    mKi = ki;
    mKd = kd;
    
    mDt = dt;
    mDeadband = deadband;
    mE_int_max = e_int_max;
    mE_int_min = e_int_min;
    mU_max = u_max;
    mU_min = u_min;
    
}







float PID::PID_CalcOutput(float error)
{
    mE_cur = mRef - error;
    
    //Deadband
    if(mE_cur <= mDeadband/2 && mE_cur >= -mDeadband/2)
	mE_cur = 0;
    
    mE_der = (mE_cur - mE_old)/mDt;
    mE_int = mE_int + mE_cur*mDt;
    
    //Limits for integral action
    if(mE_int > mE_int_max)
	mE_int = mE_int_max;
    
    if(mE_int < mE_int_min)
	mE_int = mE_int_min;
	
    mU = mKp*mE_cur + mKd*mE_der + mKi*mE_int;

    //Limit controller action
    if(mU > mU_max)
	mU = mU_max;
    
    if(mU <mU_min)
	mU = mU_min;

    mE_old = mE_cur;

    return mU;
    
}



#endif




