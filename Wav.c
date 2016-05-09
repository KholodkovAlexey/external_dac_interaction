#include "Wav.h"

uint32_t ReverseByteEndian32(uint32_t x) {
	return (x >> 24) | ((x >> 8) & 0x0000FF00) | ((x << 8) & 0x00FF0000)
			| (x << 24);
}

uint16_t ReverseByteEndian16(uint16_t x) {
	return (uint16_t) (x >> 8) | (uint16_t) (x << 8);
}

Wav_Read_ErrorTypeDef ReadWav(WavTypeDef *wav, uint32_t FileOffset) {
	Wav_Read_ErrorTypeDef res = NO_ERROR;
	__IO RIFF_ChunkTypeDef *riff = (RIFF_ChunkTypeDef*) FileOffset;
	uint32_t offset = FileOffset;
	wav->ftm = 0;
	wav->data = 0;

	if (riff->ChunkID != ReverseByteEndian32(CHUNK_ID_RIFF)) {
		res = NOT_RIFF;
	} else {
		if (riff->RIFFType != ReverseByteEndian32(RIFF_TYPE_WAVE)) {
			res = NOT_WAV;
		} else {

			//uint32_t data_offset = 20 +  (*((__IO uint32_t*) (offset + 16)));

			/*if (data_offset == 36) {
				GPIOD->BSRR = GPIO_BSRR_BS_13;
			}*/

			/*if (*((__IO uint32_t*) (offset + 12))
					== ReverseByteEndian32(CHUNK_ID_FTM)) {
				GPIOD->BSRR = GPIO_BSRR_BS_13;
			}

			if (*((__IO uint32_t*) (offset + 36))
					== ReverseByteEndian32(CHUNK_ID_DATA)) {
				GPIOD->BSRR = GPIO_BSRR_BS_14;
			}*/

			uint32_t riff_end = offset
					+ ReverseByteEndian32(riff->ChunkDataSize) + 8;
			offset += 12;
			while ((offset < riff_end) && !(wav->ftm && wav->data)) {
				__IO ChunkTypeDef *chunk = (ChunkTypeDef*) offset;
				switch (ReverseByteEndian32(chunk->ChunkID)) {
				case CHUNK_ID_FTM:
					wav->ftm = (FTM_ChunkTypeDef*) offset;
					break;
				case CHUNK_ID_DATA:
					wav->data = (ChunkTypeDef*) offset;
					break;
				}
				offset += chunk->ChunkDataSize + 8;
			}
		}
		if (!wav->ftm) {
			res = MISS_FTM;
		}
		if (!wav->data) {
			res = MISS_DATA;
		}
	}
	return res;
}

