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
	uint32_t CompressionCode_NumberOfChannels;
	uint32_t SampleRate;
	uint32_t AverageBytesPerSecond;
	uint32_t BlockAlign_SignificantBitsPerSample;
} FTM_ChunkTypeDef;

typedef struct {
	__IO FTM_ChunkTypeDef *ftm;__IO ChunkTypeDef *data;
} WavTypeDef;

typedef enum {
	WAV_ERROR_NO_ERROR,
	WAV_ERROR_NOT_RIFF_WAV,
	WAV_ERROR_MISS_FTM_OR_DATA,
	WAV_ERROR_WORONG_FORMAT
} Wav_ErrorTypeDef;

uint32_t ReverseByteEndian32(uint32_t x);
uint16_t ReverseByteEndian16(uint16_t x);
Wav_ErrorTypeDef ReadWav(WavTypeDef *wav, uint32_t FileOffset);

#endif /* WAV_H_ */
