#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"

#include "pico/audio_i2s.h"
#include "sdring.h"

#define START_SECTOR 3200000
#define FILE_LEN_BYTES 36171776

#define SDRING_INDEX_MASK_HALFWORDS ((1 << (SDRING_BUF_LOG_SIZE_BYTES - 1)) - 1)
#define SAMPLES_PER_BUFFER (128 * 1)

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

    struct audio_buffer_pool *producer_pool = 
        audio_new_producer_pool(&producer_format, 
                                24,
                                SAMPLES_PER_BUFFER);
    
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
  sdring_set_stream(1, START_SECTOR, FILE_LEN_BYTES, true);
}

void audio_init()
{
  producer_pool = internal_audio_init();
}

void audio_transfer()
{
    const int32_t vol = 10;

    if (sdring_words_available(1) < SAMPLES_PER_BUFFER)
        return;

    struct audio_buffer *buffer = take_audio_buffer(producer_pool, false);
    if (!buffer)
        return;

    uint16_t* pcm_data_ptr = (uint16_t*)sdring_buffer_1;

    uint32_t sdring_idx = sdring_get_data_in_ringbuffer_blocking(1, buffer->max_sample_count) << 1;
    assert(buffer->buffer->size >= sizeof(uint16_t) * buffer->max_sample_count * 2);
    int16_t *samples = (int16_t*)buffer->buffer->bytes;
    for (uint i = 0; i < buffer->max_sample_count * 2; i++) {
        int16_t sample = pcm_data_ptr[(sdring_idx + i) & SDRING_INDEX_MASK_HALFWORDS];
        samples[i] = (vol * sample) >> 8u;
    }
    sdring_release_ringbuffer(1);

    buffer->sample_count = buffer->max_sample_count;
    give_audio_buffer(producer_pool, buffer);
}