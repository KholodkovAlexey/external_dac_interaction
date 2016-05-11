#include "Wav.h"

uint32_t ReverseByteEndian32(uint32_t x) {
	return (x >> 24) | ((x >> 8) & 0x0000FF00) | ((x << 8) & 0x00FF0000)
			| (x << 24);
}

uint16_t ReverseByteEndian16(uint16_t x) {
	return (uint16_t) (x >> 8) | (uint16_t) (x << 8);
}

Wav_ErrorTypeDef ReadWav(WavTypeDef *wav, uint32_t FileOffset) {
	Wav_ErrorTypeDef res = WAV_ERROR_NO_ERROR;
	__IO RIFF_ChunkTypeDef *riff = (RIFF_ChunkTypeDef*) FileOffset;
	uint32_t offset = FileOffset;
	wav->ftm = 0;
	wav->data = 0;

	if ((riff->ChunkID != ReverseByteEndian32(CHUNK_ID_RIFF))
			|| (riff->RIFFType != ReverseByteEndian32(RIFF_TYPE_WAVE))) {
		res = WAV_ERROR_NOT_RIFF_WAV;
	} else {
		uint32_t riff_end = offset + ReverseByteEndian32(riff->ChunkDataSize)
				+ 8;
		offset += 12;
		while ((offset < riff_end) && !(wav->ftm && wav->data)) {

			__IO ChunkTypeDef *chunk = (ChunkTypeDef*) offset;

			switch (ReverseByteEndian32(chunk->ChunkID)) {
			case CHUNK_ID_FTM:
				wav->ftm = (FTM_ChunkTypeDef*) offset;

				//Неверный способ сжатия или неправилное выравнивание блока
				if ((((uint16_t) wav->ftm->CompressionCode_NumberOfChannels)
						!= FTM_COPRESSION_CODE_PCM)
						|| (((uint16_t) wav->ftm->BlockAlign_SignificantBitsPerSample)
								!= ((uint16_t) (wav->ftm->BlockAlign_SignificantBitsPerSample
										>> 19))
										* ((uint16_t) (wav->ftm->CompressionCode_NumberOfChannels
												>> 16)))) {
					//Завершаем цыкл
					offset = riff_end;
					res = WAV_ERROR_WORONG_FORMAT;
				}
				break;
			case CHUNK_ID_DATA:
				wav->data = (ChunkTypeDef*) offset;
				break;
			}
			offset += chunk->ChunkDataSize + 8;
		}

		if ((!wav->ftm) || (!wav->data)) {
			res = WAV_ERROR_MISS_FTM_OR_DATA;
		}

	}
	return res;
}

