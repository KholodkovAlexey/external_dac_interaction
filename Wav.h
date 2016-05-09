#ifndef WAV_H_
#define WAV_H_

#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx.h"

#define CHUNK_ID_RIFF 0x52494646
#define CHUNK_ID_FTM 0x666D7420
#define CHUNK_ID_DATA 0x64617461

#define RIFF_TYPE_WAVE 0x57415645

#define FTM_COPRESSION_CODE_PCM 0x0001

typedef struct {
	uint32_t ChunkID;
	uint32_t ChunkDataSize;
} ChunkTypeDef;

typedef struct {
	uint32_t ChunkID;
	uint32_t ChunkDataSize;
	uint32_t RIFFType;
} RIFF_ChunkTypeDef;

typedef struct {
	uint32_t ChunkID;
	uint32_t ChunkDataSize;
	uint32_t CopressionCode_NumberOfChannels;
	uint32_t SampleRate;
	uint32_t AverageBytesPerSecond;
	uint32_t BlockAlign_SignificantBitsPerSample;
} FTM_ChunkTypeDef;

typedef struct {
	__IO FTM_ChunkTypeDef *ftm;
	__IO ChunkTypeDef *data;
} WavTypeDef;

typedef enum {
	NO_ERROR, NOT_RIFF, NOT_WAV, NOT_PCM, MISS_FTM, MISS_DATA
} Wav_Read_ErrorTypeDef;

uint32_t ReverseByteEndian32(uint32_t x);
uint16_t ReverseByteEndian16(uint16_t x);
Wav_Read_ErrorTypeDef ReadWav(WavTypeDef *wav, uint32_t FileOffset);

#endif /* WAV_H_ */
