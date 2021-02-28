#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"

#include "pico/audio_i2s.h"
#include "pico/sd_card.h"

#define START_SECTOR 3200000
#define FILE_LEN_BYTES 36171776

static uint32_t sector_next;

#define SECTORS_PER_READ 6
#define SAMPLES_PER_BUFFER (128 * SECTORS_PER_READ)

static struct audio_buffer_pool *internal_audio_init() {

    static audio_format_t audio_format = {
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,
            .sample_freq = 44100,
            .channel_count = 2,
    };

    static struct audio_buffer_format producer_format = {
            .format = &audio_format,
            .sample_stride = 4
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, PICO_AUDIO_I2S_BUFFERS_PER_CHANNEL,
                                                                      SAMPLES_PER_BUFFER); // todo correct size
    bool __unused ok;
    const struct audio_format *output_format;
    struct audio_i2s_config config = {
            .data_pin = 26,
            .clock_pin_base = 27,
            .dma_channel = 1,
            .pio_sm = 3,
    };

    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    audio_i2s_set_enabled(true);

    return producer_pool;
}

static struct audio_buffer_pool *producer_pool;

void audio_reset()
{
  sector_next = START_SECTOR;
}

void audio_init()
{
  producer_pool = internal_audio_init();
  audio_reset();
}

void audio_transfer()
{
    const int32_t vol = 20;

    {
        // Wait for any previous transfer to stop.
        if (!sd_scatter_read_complete(NULL, NULL)) return;

        struct audio_buffer *buffer = take_audio_buffer(producer_pool, false);
        if (!buffer)
            return;

        sd_set_byteswap_on_read(false);
        sd_readblocks_sync((uint32_t*)buffer->buffer->bytes, sector_next, SECTORS_PER_READ);
        sector_next += SECTORS_PER_READ;
        sd_set_byteswap_on_read(true);

        //__breakpoint();

        int16_t *samples = (int16_t*)buffer->buffer->bytes;
        for (uint i = 0; i < buffer->max_sample_count * 2; i++) {
            samples[i] = (vol * samples[i]) >> 8u;
        }
        buffer->sample_count = buffer->max_sample_count;
        give_audio_buffer(producer_pool, buffer);
    }
}