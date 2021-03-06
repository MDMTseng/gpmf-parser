/*! @file GPMF_demo.c
*
*  @brief Demo to extract GPMF from an MP4
*
*  @version 1.0.1
*
*  (C) Copyright 2017 GoPro Inc (http://gopro.com/).
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "../GPMF_parser.h"
#include "GPMF_mp4reader.h"
#include "imu_data_process.h"


extern void PrintGPMF(GPMF_stream *ms);

int main(int argc, char *argv[])
{
	int32_t ret = GPMF_OK;
	GPMF_stream metadata_stream, *ms = &metadata_stream;
	double metadatalength;
	uint32_t *payload = NULL; //buffer to store GPMF samples from the MP4.


	// get file return data
	if (argc != 2)
	{
		printf("usage: %s <file_with_GPMF>\n", argv[0]);
		return -1;
	}


	metadatalength = OpenGPMFSource(argv[1]);

	//metadatalength = OpenGPMFSourceUDTA(argv[1]);

	if (metadatalength > 0.0)
	{
		uint32_t index, payloads = GetNumberGPMFPayloads();
		printf("found %.2fs of metadata, from %d payloads, within %s\n", metadatalength, payloads, argv[1]);

#if 1
		if (payloads == 1) // Printf the contents of the single payload
		{
			uint32_t payloadsize = GetGPMFPayloadSize(0);
			payload = GetGPMFPayload(payload, 0);
			if(payload == NULL)
				goto cleanup;

			ret = GPMF_Init(ms, payload, payloadsize);
			if (ret != GPMF_OK)
				goto cleanup;

			// Output (printf) all the contained GPMF data within this payload
			ret = GPMF_Validate(ms, GPMF_RECURSE_LEVELS); // optional
			if (GPMF_OK != ret)
			{
				printf("Invalid Structure\n");
				goto cleanup;
			}

			GPMF_ResetState(ms);
			do
			{
				PrintGPMF(ms);  // printf current GPMF KLV
			} while (GPMF_OK == GPMF_Next(ms, GPMF_RECURSE_LEVELS));
			GPMF_ResetState(ms);
			printf("\n");

		}
#endif

		for (index = 0; index < payloads; index++)
		{
			uint32_t payloadsize = GetGPMFPayloadSize(index);
			double in = 0.0, out = 0.0; //times
			payload = GetGPMFPayload(payload, index);
			if (payload == NULL)
				goto cleanup;

			ret = GetGPMFPayloadTime(index, &in, &out);
			if (ret != GPMF_OK)
				goto cleanup;

			ret = GPMF_Init(ms, payload, payloadsize);
			if (ret != GPMF_OK)
				goto cleanup;

#if 1		// Find all the available Streams and the data carrying FourCC
			if (index == 0) // show first payload 
			{
				while (GPMF_OK == GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS))
				{
					if (GPMF_OK == GPMF_SeekToSamples(ms)) //find the last FOURCC within the stream
					{
						uint32_t key = GPMF_Key(ms);
						GPMF_SampleType type = (GPMF_SampleType)GPMF_Type(ms);
						uint32_t elements = GPMF_ElementsInStruct(ms);
						//uint32_t samples = GPMF_Repeat(ms);
						uint32_t samples = GPMF_PayloadSampleCount(ms);

						if (samples)
						{
							printf("  STRM of %c%c%c%c ", PRINTF_4CC(key));

							if (type == GPMF_TYPE_COMPLEX)
							{
								GPMF_stream find_stream;
								GPMF_CopyState(ms, &find_stream);

								if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE, GPMF_CURRENT_LEVEL))
								{
									char tmp[64];
									char *data = (char *)GPMF_RawData(&find_stream);
									int size = GPMF_RawDataSize(&find_stream);

									if (size < sizeof(tmp))
									{
										memcpy(tmp, data, size);
										tmp[size] = 0;
										printf("of type %s ", tmp);
									}
								}

							}
							else
							{
								printf("of type %c ", type);
							}

							printf("with %d sample%s ", samples, samples > 1 ? "s" : "");

							if (elements > 1)
								printf("-- %d elements per sample", elements);

							printf("\n");
						}
					}
				}
				GPMF_ResetState(ms);
				printf("\n");
			}
#endif 
			if(index==0)
			{
				//Print_DATA(ms,"GYRO");
				//Print_DATA(ms,"ACCL");
				//Print_DATA(ms,"MAGN");
				//Extract_GYRO_DATA(ms);
				//printf("GYRO>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
				//Extract_META_DATA(ms,"GYRO",GYRO_Process_CallBack,NULL,in,out);

				printf("MAGN>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
				Extract_META_DATA(ms,"MAGN",MataData_Print_CallBack,NULL,in,out);

				printf("ACCL>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
				Extract_META_DATA(ms,"ACCL",MataData_Print_CallBack,NULL,in,out);
				//printf("SHUT>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
				//Extract_META_DATA(ms,"SHUT",MataData_Print_CallBack,NULL,in,out);
			}
		}

#if 1
		// Find all the available Streams and compute they sample rates
		while (GPMF_OK == GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS))
		{
			if (GPMF_OK == GPMF_SeekToSamples(ms)) //find the last FOURCC within the stream
			{
				uint32_t fourcc = GPMF_Key(ms);
				double rate = GetGPMFSampleRate(fourcc, GPMF_SAMPLE_RATE_PRECISE);// GPMF_SAMPLE_RATE_FAST);
				printf("%c%c%c%c sampling rate = %f Hz\n", PRINTF_4CC(fourcc), rate);
			}
		}
#endif


	cleanup:
		if (payload) FreeGPMFPayload(payload); payload = NULL;
		CloseGPMFSource();
	}

	return ret;
}
