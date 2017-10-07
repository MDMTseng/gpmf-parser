
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "../GPMF_parser.h"
#include "GPMF_mp4reader.h"
#include "imu_data_process.h"
#include "mtQuaternions.h"


int Extract_META_DATA(GPMF_stream *ms,char *_4CC_Tag,Metadata_Process_CallBack cb,void *param)
{
	if (GPMF_OK != GPMF_FindNext(ms, STR2FOURCC(_4CC_Tag), GPMF_RECURSE_LEVELS) )return;
	uint32_t key = GPMF_Key(ms);
	uint32_t samples = GPMF_Repeat(ms);
	uint32_t elements = GPMF_ElementsInStruct(ms);
	uint32_t buffersize = samples * elements * sizeof(double);
	GPMF_stream find_stream;
	double *ptr, *tmpbuffer = malloc(buffersize);
	uint32_t unit_samples = 1;

	//printf("MP4 Payload time %.3f to %.3f seconds\n", in, out);
	if (!tmpbuffer)
	{
		return -1;
	}

	if (!samples)
	{
		free(tmpbuffer);
		return -1;
	}


	
	uint32_t i, j;

	//Search for any units to display
	GPMF_CopyState(ms, &find_stream);

	if(0)
	{
		char units[10][6] = { "" };

		if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS, GPMF_CURRENT_LEVEL) ||
		GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS, GPMF_CURRENT_LEVEL))
		{
			char *data = (char *)GPMF_RawData(&find_stream);
			int ssize = GPMF_StructSize(&find_stream);
			unit_samples = GPMF_Repeat(&find_stream);

			for (i = 0; i < unit_samples; i++)
			{
				printf("%c%c%c%c ", PRINTF_4CC(key));
				memcpy(units[i], data, ssize);
				units[i][ssize] = 0;
				data += ssize;
			}
		}


	}
	
	//GPMF_FormattedData(ms, tmpbuffer, buffersize, 0, samples); // Output data in LittleEnd, but no scale
	GPMF_ScaledData(ms, tmpbuffer, buffersize, 0, samples, GPMF_TYPE_DOUBLE);  //Output scaled data as floats

    ptr = tmpbuffer;
    
    int ret=cb(param,tmpbuffer,samples,elements,0,0);
    
	free(tmpbuffer);
	GPMF_ResetState(ms);
	return ret;
}

//Front Camera Up      is [2]/Z axis,   Yaw ,heading
//Front Camera left    is [1]/Y axis    Pitch,attitude
//Front camera forward is [0]/X axis    Roll,bank

//Yaw: theta, yaw 
//Pitch:phi  , pitch
//Roll:    psi  , roll
//Rotation direction, right handed XYZ, along axis toward outside counter clock-wise is positive
// Assuming the angles are in radians.

MTQuaternion Conv2Quaternion(double Yaw, double Pitch, double Roll) {
    MTQuaternion q;

	double cy = cos(Yaw * 0.5);
	double sy = sin(Yaw * 0.5);
	double cr = cos(Roll * 0.5);
	double sr = sin(Roll * 0.5);
	double cp = cos(Pitch * 0.5);
	double sp = sin(Pitch * 0.5);

	q.s   = cy * cr * cp + sy * sr * sp;
	q.v.x = cy * sr * cp - sy * cr * sp;
	q.v.y = cy * cr * sp + sy * sr * cp;
	q.v.z = sy * cr * cp - cy * sr * sp;
	
	return q;
}



MTVec3D Conv2Euler(double w, double x, double y,double z) {//Y,Z,X
	MTVec3D e;
	// roll (x-axis rotation)
	double sinr = +2.0 * (w * x + y * z);
	double cosr = +1.0 - 2.0 * (x * x + y * y);
	e.x = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (w * y - z * x);
	if (fabs(sinp) >= 1)
		e.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		e.y = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (w * z + x * y);
	double cosy = +1.0 - 2.0 * (y * y + z * z);  
	e.z = atan2(siny, cosy);
	return e;
}
/*
RTVector3 RTMath::poseFromAccelMag(const RTVector3& accel, const RTVector3& mag)
{
    RTVector3 result;
    RTQuaternion m;
    RTQuaternion q;

    accel.accelToEuler(result);

//  q.fromEuler(result);
//  since result.z() is always 0, this can be optimized a little

    RTFLOAT cosX2 = cos(result.x() / 2.0f);
    RTFLOAT sinX2 = sin(result.x() / 2.0f);
    RTFLOAT cosY2 = cos(result.y() / 2.0f);
    RTFLOAT sinY2 = sin(result.y() / 2.0f);

    q.setScalar(cosX2 * cosY2);
    q.setX(sinX2 * cosY2);
    q.setY(cosX2 * sinY2);
    q.setZ(-sinX2 * sinY2);
//    q.normalize();

    m.setScalar(0);
    m.setX(mag.x());
    m.setY(mag.y());
    m.setZ(mag.z());

    m = q * m * q.conjugate();
    result.setZ(-atan2(m.y(), m.x()));
    return result;
}
*/

MTVec3D mtCreateEulerFromQuaternion(MTQuaternion *q) {
    return Conv2Euler(q->s, q->v.x, q->v.y,q->v.z);
}


MTQuaternion mtCreateQuaternionFromEuler(MTVec3D *euler) {
	return Conv2Quaternion(euler->y,euler->z,euler->x);
}

MTQuaternion TTT={1,{0,0,0}};


int GYRO_Process_CallBack(void *param,const void *rawdata,int samples,int elements,float start_s,float stop_s)
{
	if(elements!=3)return -1;
	MTVec3D angle;
	int i,j;
	MTQuaternion intq=Conv2Quaternion(0,0,0);
	double *ptr=rawdata;
	
    for (i = 0; i < samples; i++,ptr+=3)
	{
		if(i==samples/2)
		{
			
			MTQuaternion nq=Conv2Quaternion(ptr[2],ptr[1],ptr[0]);
			printf("Raw :  %.3f, %.3f, %.3f\n",ptr[2]*180/M_PI,ptr[1]*180/M_PI,ptr[0]*180/M_PI);
			printf("RawQ:  %.3f, %.3f, %.3f, %.3f\n", nq.s,nq.v.x,nq.v.y,nq.v.z);

		}
		MTQuaternion nq=Conv2Quaternion(ptr[2]/samples,ptr[1]/samples,ptr[0]/samples);
		intq=mtMultMTQuaternionMTQuaternion(&intq,&nq);
	}
	mtNormMTQuaternion (&intq);
	MTVec3D euler= mtCreateEulerFromQuaternion(&intq);
	printf("euler:  %.3f, %.3f, %.3f\n", euler.z*180/M_PI,euler.y*180/M_PI,euler.x*180/M_PI);
	printf("Quate:  %.3f, %.3f, %.3f, %.3f\n", intq.s,intq.v.x,intq.v.y,intq.v.z);

	TTT=mtMultMTQuaternionMTQuaternion(&TTT,&intq);
	euler= mtCreateEulerFromQuaternion(&TTT);

	printf("Teuler:  %.3f, %.3f, %.3f\n", euler.z*180/M_PI,euler.y*180/M_PI,euler.x*180/M_PI);
	


	//return MataData_Print_CallBack(param,rawdata,samples,elements,start_s,stop_s);
	return 0;
}



int MataData_Print_CallBack(void *param,const void *rawdata,int samples,int elements,float start_s,float stop_s)
{
    int i,j;
    double *ptr=rawdata;
    for (i = 0; i < samples; i++)
	{
		for (j = 0; j < elements; j++)
			printf("%.3f, ", *ptr++);

		printf("\n");
	}
    return 0;
}