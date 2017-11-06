#include "codec.h"
#include "wm8731.h"
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include "platform.h"

#include <stdint.h>
#include <string.h>

/// If WM8731_HIGHPASS is defined the codec is configured to remove
/// DC bias automatically, otherwise it's done in software. The noise
/// characteristics are different.
//#define WM8731_HIGHPASS

_Atomic unsigned samplecounter;
_Atomic CodecIntSample peakIn = INT16_MIN;
_Atomic CodecIntSample peakOut = INT16_MIN;

static const uint32_t DAC_DMA_STREAM = DMA_STREAM4; // SPI2_TX
static const uint32_t DAC_DMA_CHANNEL = DMA_SxCR_CHSEL_0;
static const uint32_t ADC_DMA_STREAM = DMA_STREAM3; // I2S2_EXT_RX
static const uint32_t ADC_DMA_CHANNEL = DMA_SxCR_CHSEL_3;

#define BUFFER_SAMPLES ((CODEC_SAMPLES_PER_FRAME)*2)
static int16_t dacBuffer[2][BUFFER_SAMPLES] __attribute__((aligned(4)));
static int16_t adcBuffer[2][BUFFER_SAMPLES] __attribute__((aligned(4)));

static CodecProcess appProcess;

static void codecInitI2C(void)
{
	i2c_reset(I2C2);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO10 | GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO10 | GPIO11);

	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C2);
	i2c_peripheral_disable(I2C2);
	i2c_enable_ack(I2C2);
	i2c_set_dutycycle(I2C2, I2C_CCR_DUTY_DIV2);
	/* HSI is at 8Mhz */
	i2c_set_speed(I2C2, i2c_speed_sm_100k, 8);
	i2c_peripheral_enable(I2C2);
}

static void codecWriteReg(uint8_t reg, uint16_t value)
{
	uint8_t cmd[2];

	/* Assemble 2-byte data in WM8731 format */
	cmd[0] = ((reg << 1) & 0xFE) | ((value >> 8) & 0x01);
	cmd[1] = value & 0xFF;
	i2c_transfer7(I2C2, CODEC_I2C_ADDR, cmd, 2, NULL, 0);
}

static void codecConfig()
{
	codecWriteReg(0x0f, 0b000000000); // Reset!
	codecSetInVolume(0);
	codecSetOutVolume(-10);
	codecWriteReg(0x04, 0b000010010); // Analog path - select DAC, no bypass
#ifdef WM8731_HIGHPASS
	codecWriteReg(0x05, 0b000000000); // Digital path - disable soft mute
#else
	codecWriteReg(0x05, 0b000000001); // Digital path - disable soft mute and highpass
#endif
	codecWriteReg(0x06, 0b001000010); // Power down control - enable everything
	codecWriteReg(0x07, 0b000000010); // Interface format - 16-bit I2S
	codecWriteReg(0x08, 0b000000001); // USB sampling rate, 48x48, on 12Mhz
	codecWriteReg(0x09, 0b000000001); // Active control - engage!

	codecSetInVolume(5);
	codecSetOutVolume(-10);
	i2c_reset(I2C2);
	i2c_peripheral_disable(I2C2);
}

void codecSetInVolume(int vol)
{
	// -23 <= vol <= 8
	const unsigned involume = 0x17 + vol;
	codecWriteReg(0x00, 0x100 | (involume & 0x1f)); // Left line in, unmute
}

void codecSetOutVolume(int voldB)
{
	// -73 <= voldB <= 6
	const unsigned volume = 121 + voldB;
	codecWriteReg(0x02, 0x100 | (volume & 0x7f)); // Left headphone
}

void codecInit(void)
{
	memset(adcBuffer, 0xaa, sizeof(adcBuffer));

	// I2S2 alternate function mapping
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO12 | GPIO13 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF6, GPIO14); // I2S2ext_SD (I2S receive)
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_set_af(GPIOC, GPIO_AF5, GPIO6); // MCLK
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO12 | GPIO13 | GPIO15);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO6);

	codecInitI2C();
	codecConfig();

	// Set up I2S for 48kHz 16-bit stereo.
	// The input to the PLLI2S is 1MHz. Division factors are from
	// table 126 in the data sheet.
	//
	// This gives us 1.536MHz SCLK = 16 bits * 2 channels * 48000 Hz
	// and 12.000 MHz MCLK = 250 * 48000 Hz.
	// With this PLL configuration the actual sampling frequency
	// is nominally 47991 Hz.

	spi_reset(SPI2);

	RCC_PLLI2SCFGR = (3 << 28) | (250 << 6);
	RCC_CR |= RCC_CR_PLLI2SON;
	while (!(RCC_CR & RCC_CR_PLLI2SRDY));

	const unsigned i2sdiv = 3;
	SPI_I2SPR(SPI2) = SPI_I2SPR_MCKOE | SPI_I2SPR_ODD | i2sdiv;
	SPI_I2SCFGR(SPI2) |= SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SE |
	                     (SPI_I2SCFGR_I2SCFG_MASTER_TRANSMIT << SPI_I2SCFGR_I2SCFG_LSB);
	SPI_I2SCFGR(I2S2_EXT_BASE) = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SE |
	                             (SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE << SPI_I2SCFGR_I2SCFG_LSB);

	// Configure the DMA engine to stream data to the DAC.
	dma_stream_reset(DMA1, DAC_DMA_STREAM);
	dma_set_peripheral_address(DMA1, DAC_DMA_STREAM, (intptr_t)&SPI_DR(SPI2));
	dma_set_memory_address(DMA1, DAC_DMA_STREAM, (intptr_t)dacBuffer[0]);
	dma_set_memory_address_1(DMA1, DAC_DMA_STREAM, (intptr_t)dacBuffer[1]);
	dma_set_number_of_data(DMA1, DAC_DMA_STREAM, BUFFER_SAMPLES);
	dma_channel_select(DMA1, DAC_DMA_STREAM, DAC_DMA_CHANNEL);
	dma_set_transfer_mode(DMA1, DAC_DMA_STREAM, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	dma_set_memory_size(DMA1, DAC_DMA_STREAM, DMA_SxCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1, DAC_DMA_STREAM, DMA_SxCR_PSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA1, DAC_DMA_STREAM);
	dma_enable_double_buffer_mode(DMA1, DAC_DMA_STREAM);
	dma_enable_stream(DMA1, DAC_DMA_STREAM);

	/* Configure the DMA engine to stream data from the ADC. */
	dma_stream_reset(DMA1, ADC_DMA_STREAM);
	dma_set_peripheral_address(DMA1, ADC_DMA_STREAM, (intptr_t)&SPI_DR(I2S2_EXT_BASE));
	dma_set_memory_address(DMA1, ADC_DMA_STREAM, (intptr_t) adcBuffer[0]);
	dma_set_memory_address_1(DMA1, ADC_DMA_STREAM, (intptr_t) adcBuffer[1]);
	dma_set_number_of_data(DMA1, ADC_DMA_STREAM, BUFFER_SAMPLES);
	dma_channel_select(DMA1, ADC_DMA_STREAM, ADC_DMA_CHANNEL);
	dma_set_transfer_mode(DMA1, ADC_DMA_STREAM, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	dma_set_memory_size(DMA1, ADC_DMA_STREAM, DMA_SxCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1, ADC_DMA_STREAM, DMA_SxCR_PSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA1, ADC_DMA_STREAM);
	dma_enable_double_buffer_mode(DMA1, ADC_DMA_STREAM);
	dma_enable_stream(DMA1, ADC_DMA_STREAM);

	dma_enable_transfer_complete_interrupt(DMA1, ADC_DMA_STREAM);
	nvic_enable_irq(NVIC_DMA1_STREAM3_IRQ);
	nvic_set_priority(NVIC_DMA1_STREAM3_IRQ, 0x80); // 0 is most urgent

	// Start transmitting data
	spi_enable_rx_dma(I2S2_EXT_BASE);
	spi_enable_tx_dma(SPI2);
}

void dma1_stream3_isr(void)
{
	dma_clear_interrupt_flags(DMA1, ADC_DMA_STREAM, DMA_TCIF);

	AudioBuffer* outBuffer = dma_get_target(DMA1, DAC_DMA_STREAM) ?
	                         (void*)dacBuffer[0] :
	                         (void*)dacBuffer[1];
	AudioBuffer* inBuffer = dma_get_target(DMA1, DAC_DMA_STREAM) ?
	                        (void*)adcBuffer[0] :
	                        (void*)adcBuffer[1];

#ifndef WM8731_HIGHPASS
	// Correct DC offset
	static int32_t correction[2] = { 1230 << 16, 1230 << 16 };
	int average[2] = {};
	for (unsigned n = 0; n < CODEC_SAMPLES_PER_FRAME; n++)
	{
		inBuffer->s[n][0] += correction[0] >> 16;
		inBuffer->s[n][1] += correction[1] >> 16;
		average[0] += inBuffer->s[n][0];
		average[1] += inBuffer->s[n][1];
	}
	correction[0] -= average[0] << 2;
	correction[1] -= average[1] << 2;
#endif

	if (appProcess)
	{
		appProcess((const AudioBuffer*)inBuffer, (AudioBuffer*)outBuffer);
	}

	samplecounter += CODEC_SAMPLES_PER_FRAME;
	CodecIntSample framePeakOut = INT16_MIN;
	CodecIntSample framePeakIn = INT16_MIN;
	for (unsigned s = 0; s < BUFFER_SAMPLES; s++)
	{
		if (outBuffer->m[s] > framePeakOut)
		{
			framePeakOut = outBuffer->m[s];
		}
		if (inBuffer->m[s] > framePeakIn)
		{
			framePeakIn = inBuffer->m[s];
		}
	}
	if (framePeakOut > peakOut)
	{
		peakOut = framePeakOut;
	}
	if (framePeakIn > peakIn)
	{
		peakIn = framePeakIn;
	}

	platformFrameFinishedCB();
}

void codecPeek(const int16_t** buffer, unsigned* buffersamples, unsigned* writepos)
{
	*buffersamples = 2*BUFFER_SAMPLES;
	*buffer = adcBuffer[0];

	unsigned target = dma_get_target(DMA1, ADC_DMA_STREAM);
	unsigned numberOfData = DMA_SNDTR(DMA1, ADC_DMA_STREAM);

	if (target != dma_get_target(DMA1, ADC_DMA_STREAM))
	{
		target = dma_get_target(DMA1, ADC_DMA_STREAM);
		numberOfData = DMA_SNDTR(DMA1, ADC_DMA_STREAM);
	}

	*writepos = (target ? BUFFER_SAMPLES : 0) + (BUFFER_SAMPLES - numberOfData);
}

void codecRegisterProcessFunction(CodecProcess fn)
{
	appProcess = fn;
}
