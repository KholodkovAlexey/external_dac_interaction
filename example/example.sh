#!/bin/bash

/usr/local/bin/st-flash write STM32F4_Task.bin 0x8000000

/usr/local/bin/st-flash write test_mono_16000Hz_16bit_PCM.wav 0x800C000
