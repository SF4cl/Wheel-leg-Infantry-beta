#include "AttitudeThread.h"

uint16_t whoami;
uint16_t pwmimu = 10;

fp32 INS_quat[4] = {0.717f, 0.717f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.Å·À­½Ç µ¥Î» rad
fp32 INS_accel[3],Eaccel[3],Baccel[3],cou;
fp32 paramater[1]={0.1f};
first_order_filter_type_t accel_filter;
int califlag=0,t=0,Gaccel;
float CalcountG[3],CalcountA[3],dir[3]={-1.0f,1.0f,-1.0f};
fp32 paramater7=0.05f,K1=0;
void getRotation(float *Quaternion, float *rt_mat)
{
  rt_mat[0] = 1 - 2 * (Quaternion[2] * Quaternion[2]) - 2 * (Quaternion[3] * Quaternion[3]);
  rt_mat[1] = 2 * Quaternion[1] * Quaternion[2] - 2 * Quaternion[0] * Quaternion[3];
  rt_mat[2] = 2 * Quaternion[1] * Quaternion[3] + 2 * Quaternion[0] * Quaternion[2];
  rt_mat[3] = 2 * Quaternion[1] * Quaternion[2] + 2 * Quaternion[0] * Quaternion[3];
  rt_mat[4] = 1 - 2 * (Quaternion[1] * Quaternion[1]) - 2 * (Quaternion[3] * Quaternion[3]);
  rt_mat[5] = 2 * Quaternion[2] * Quaternion[3] - 2 * Quaternion[0] * Quaternion[1];
  rt_mat[6] = 2 * Quaternion[1] * Quaternion[3] - 2 * Quaternion[0] * Quaternion[2];
  rt_mat[7] = 2 * Quaternion[2] * Quaternion[3] + 2 * Quaternion[0] * Quaternion[1];
  rt_mat[8] = 1 - 2 * (Quaternion[1] * Quaternion[1]) - 2 * (Quaternion[2] * Quaternion[2]);
}


void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}


void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
float Fabs(float a)
{
		if(a<0)	return -a;
		return a;
}
void AttitudeThread(void *argument)
{
		osDelay(100);  
//		for(int i=0;i<100000&&Temperature<44.0f;i++)
//		{
//				whoami = ICM_42688_ReadReg(0x75);
//				ICM_42688_ReadAccel();
//		//		if(Accel[0]==0x00 && Accel[1]==0x00 && Accel[2]==0x00)
//		//			IMU42688_Init();
//		//		if(Accel[0]==-32768 && Accel[1]==-32768 && Accel[2]==-32768)
//		//			IMU42688_Init();		
//					
//				ICM_42688_ReadGyro();
//				ICM_42688_ReadTem();
//				
//				AccelCorrected[0] = ((float)Accel[0] + AccelCal[0]) * LSB_ACC_GYRO[0];
//				AccelCorrected[1] = ((float)Accel[1] + AccelCal[1]) * LSB_ACC_GYRO[0];
//				AccelCorrected[2] = ((float)Accel[2] + AccelCal[2]) * LSB_ACC_GYRO[0];
//				GyroCorrected[0] = ((float)Gyro[0] + GyroCal[0]) * LSB_ACC_GYRO[1];
//				GyroCorrected[1] = ((float)Gyro[1] + GyroCal[1]) * LSB_ACC_GYRO[1];
//				GyroCorrected[2] = ((float)Gyro[2] + GyroCal[2]) * LSB_ACC_GYRO[1];			
//				
//				pwmimu=50;/*
//				if(Temperature>45)	pwmimu=0;
//				else if(Temperature<40)	pwmimu=25;
//				else	pwmimu=25-4*(Temperature-40);
//				__HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, pwmimu);*/
//				//testX=FSqrt(INS_angle[1]);
//				//printf("%f,%f,%f\n",INS_angle[0],INS_angle[1],INS_angle[2]);
//				osDelay(1);   				
//		}
		
//		for(int i=0;i<5000;i++)
//		{
//				whoami = ICM_42688_ReadReg(0x75);
//				ICM_42688_ReadAccel();
//		//		if(Accel[0]==0x00 && Accel[1]==0x00 && Accel[2]==0x00)
//		//			IMU42688_Init();
//		//		if(Accel[0]==-32768 && Accel[1]==-32768 && Accel[2]==-32768)
//		//			IMU42688_Init();		
//					
//				ICM_42688_ReadGyro();
//				ICM_42688_ReadTem();
//				
//				AccelCorrected[0] = ((float)Accel[0] + AccelCal[0]) * LSB_ACC_GYRO[0];
//				AccelCorrected[1] = ((float)Accel[1] + AccelCal[1]) * LSB_ACC_GYRO[0];
//				AccelCorrected[2] = ((float)Accel[2] + AccelCal[2]) * LSB_ACC_GYRO[0];
//				GyroCorrected[0] = ((float)Gyro[0] + GyroCal[0]) * LSB_ACC_GYRO[1];
//				GyroCorrected[1] = ((float)Gyro[1] + GyroCal[1]) * LSB_ACC_GYRO[1];
//				GyroCorrected[2] = ((float)Gyro[2] + GyroCal[2]) * LSB_ACC_GYRO[1];			
//				CalcountG[0]+=GyroCorrected[0];
//				CalcountG[1]+=GyroCorrected[1];
//				CalcountG[2]+=GyroCorrected[2];
//				CalcountA[0]+=GyroCorrected[0];
//				CalcountA[1]+=GyroCorrected[1];
//				CalcountA[2]+=GyroCorrected[2];			
//				
//				if(Temperature>45)	pwmimu=0;
//				else if(Temperature<40)	pwmimu=25;
//				else	pwmimu=25-4*(Temperature-40);
//				__HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, pwmimu);
//				//testX=FSqrt(INS_angle[1]);
//				//printf("%f,%f,%f\n",INS_angle[0],INS_angle[1],INS_angle[2]);
//				osDelay(1);   				
//		}		
		float Rt_mat[9],quat[4];
		uint32_t time_count=0;
		IMU_QuaternionEKF_Init(10, 0.001, 50000000, 1, 0);
		first_order_filter_init(&accel_filter,0.001,&paramater7);		

    while (1)
    {
				whoami = ICM_42688_ReadReg(0x75);
				ICM_42688_ReadAccel();
		//		if(Accel[0]==0x00 && Accel[1]==0x00 && Accel[2]==0x00)
		//			IMU42688_Init();
		//		if(Accel[0]==-32768 && Accel[1]==-32768 && Accel[2]==-32768)
		//			IMU42688_Init();		
					
				ICM_42688_ReadGyro();
				ICM_42688_ReadTem();
				

				AccelCorrected[0] = ((float)Accel[0] + AccelCal[0]) * LSB_ACC_GYRO[0];
				AccelCorrected[1] = ((float)Accel[1] + AccelCal[1]) * LSB_ACC_GYRO[0];
				AccelCorrected[2] = ((float)Accel[2] + AccelCal[2]) * LSB_ACC_GYRO[0];
				//AccelCorrected[0]=Accel[0]*0.0020421+Accel[1]*-0.0027669+Accel[2]*0.0026068+5.2115;
				//AccelCorrected[1]=Accel[0]*-0.00083552+Accel[1]*0.0037683+Accel[2]*0.00109+2.3433;
				//AccelCorrected[2]=Accel[0]*0.0012066+Accel[1]*0.0010014+Accel[2]*0.0036968+-2.2418;			
			
				GyroCorrected[0] = ((float)Gyro[0] + GyroCal[0]) * LSB_ACC_GYRO[1] ;//- CalcountG[0]/5000.0f;
				GyroCorrected[1] = ((float)Gyro[1] + GyroCal[1]) * LSB_ACC_GYRO[1] ;//- CalcountG[1]/5000.0f;
				GyroCorrected[2] = ((float)Gyro[2] + GyroCal[2]) * LSB_ACC_GYRO[1] ;//- CalcountG[2]/5000.0f;			

				
				MahonyAHRSupdateIMU(INS_quat, -GyroCorrected[1], -GyroCorrected[2], GyroCorrected[0], AccelCorrected[1], AccelCorrected[2], -AccelCorrected[0]);
				IMU_QuaternionEKF_Update(GyroCorrected[1],-GyroCorrected[0], GyroCorrected[2], AccelCorrected[1],-AccelCorrected[0], AccelCorrected[2], 0.001f);
				
				quat[0]=INS_quat[1];
				quat[1]=INS_quat[2];
				quat[2]=INS_quat[3];
				quat[3]=INS_quat[0];				
				get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);				
				getRotation(INS_quat,Rt_mat);
				INS_accel[0]=-AccelCorrected[1]*Rt_mat[0]+-AccelCorrected[2]*Rt_mat[1]+AccelCorrected[0]*Rt_mat[2];
				INS_accel[1]=-AccelCorrected[1]*Rt_mat[3]+-AccelCorrected[2]*Rt_mat[4]+AccelCorrected[0]*Rt_mat[5];				
				INS_accel[2]=-AccelCorrected[1]*Rt_mat[6]+-AccelCorrected[2]*Rt_mat[7]+AccelCorrected[0]*Rt_mat[8]+9.81f;			
				Eaccel[0]=INS_accel[0];
				Eaccel[1]=INS_accel[1];
				Eaccel[2]=INS_accel[2];
				Baccel[0]=AccelCorrected[1];
				Baccel[1]=-AccelCorrected[0]-GyroCorrected[2]*GyroCorrected[2]*0.152475f;
				Baccel[2]=AccelCorrected[2];
				float acc;
				BodyFrameToEarthFrame(Baccel,Eaccel,QEKF_INS.q);
				acc=arm_cos_f32(QEKF_INS.Yaw/180*3.14159f)*-Eaccel[1]+arm_sin_f32(QEKF_INS.Yaw/180*3.14159f)*Eaccel[0];
				//EarthFrameToBodyFrame(Eaccel,Baccel,QEKF_INS.q);
				//EarthFrameToBodyFrame(Eaccel,Baccel,INS_quat);
				if(Temperature>40)	pwmimu=0;
				else if(Temperature<35)	pwmimu=27;
				else	pwmimu=27-5*(Temperature-35);
				__HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, pwmimu);
				//testX=FSqrt(INS_angle[1]);
				//printf("%f,%f,%f\n",INS_angle[0],INS_angle[1],INS_angle[2]);
				
				
				Height_KF.MeasuredVector[0]=(dx_left+dx_right)/2.0f;
				first_order_filter_cali(&accel_filter,acc);
				Height_KF.ControlVector[0]=accel_filter.out;			
				//*(Height_KF.R.pData)=K1*Height_KF.MeasuredVector[0]*Height_KF.MeasuredVector[0];
				if(time_count>1000)
					Kalman_Filter_Update(&Height_KF);			
				time_count++;
				
				if(califlag)
				{
						//CalcountG[0]=((float)Gyro[0]*(float)Gyro[0]+CalcountG[0]*(float)t)/(float)(t+1);
						//CalcountG[1]=((float)Gyro[1]*(float)Gyro[1]+CalcountG[1]*(float)t)/(float)(t+1);					
						//CalcountG[2]=((float)Gyro[2]*(float)Gyro[2]+CalcountG[2]*(float)t)/(float)(t+1);
						cou+=Eaccel[1];
						t++;
				}		
				//count_time[4]++;
				osDelay(1);   
		}
}

void AHRS_init(fp32 quat[4], fp32 accel[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

