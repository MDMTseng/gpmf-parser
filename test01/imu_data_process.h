
#ifndef _IMU_DATA_PROCESS_H
#define _IMU_DATA_PROCESS_H


#include "../GPMF_parser.h"
#include "GPMF_mp4reader.h"

typedef int (*Metadata_Process_CallBack)(void *param,const void *rawdata,int samples,int elements,double start_s,double stop_s);



typedef struct{
    float *data;
} iMU_dataStore;


void Print_DATA(GPMF_stream *ms,char *_4CC_Tag);


int Extract_META_DATA(GPMF_stream *ms,char *_4CC_Tag,Metadata_Process_CallBack cb,void *param,double time_start,double time_end);

int GYRO_Process_CallBack(void *param,const void *rawdata,int samples,int elements,double start_s,double stop_s);
int MataData_Print_CallBack(void *param,const void *rawdata,int samples,int elements,double start_s,double stop_s);
int MataData_Print_Middle_CallBack(void *param,const void *rawdata,int samples,int elements,double start_s,double stop_s);
int ACCL_Euler_CallBack(void *param,const void *rawdata,int samples,int elements,double start_s,double stop_s);


#endif
