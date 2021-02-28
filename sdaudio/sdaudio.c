#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"

#include "pico/audio_i2s.h"

#include "sdring.h"

#define START_SECTOR 3200000
#define FILE_LEN_BYTES 36171776

#define SAMPLES_PER_BUFFER 256

#define SDRING_INDEX_MASK_BYTES ((1 << SDRING_BUF_LOG_SIZE_BYTES) - 1)
#define SDRING_INDEX_MASK_HALFWORDS ((1 << (SDRING_BUF_LOG_SIZE_BYTES - 1)) - 1)

struct audio_buffer_pool *init_audio() {

    static audio_format_t audio_format = {
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,
            .sample_freq = 44100,
            .channel_count = 2,
    };

    static struct audio_buffer_format producer_format = {
            .format = &audio_format,
            .sample_stride = 4
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3,
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

unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

int main()
{
    //stdio_init_all();


    sdring_init(false);
    sdring_set_stream(START_SECTOR, FILE_LEN_BYTES, true);
    
    struct audio_buffer_pool *ap = init_audio();

    int32_t vol = 20;

    while (1) 
    {
        struct audio_buffer *buffer = take_audio_buffer(ap, true);
        uint16_t* pcm_data_ptr = (uint16_t*)sdring_buffer;

        uint32_t sdring_idx = sdring_get_data_in_ringbuffer_blocking(buffer->max_sample_count) << 1;
        assert(buffer->buffer->size >= sizeof(uint16_t) * buffer->max_sample_count * 2);
        int16_t *samples = (int16_t*)buffer->buffer->bytes;
        for (uint i = 0; i < buffer->max_sample_count * 2; i++) {
            int16_t sample = pcm_data_ptr[(sdring_idx + i) & SDRING_INDEX_MASK_HALFWORDS];
            samples[i] = (vol * sample) >> 8u;
        }
        sdring_release_ringbuffer();
        buffer->sample_count = buffer->max_sample_count;
        give_audio_buffer(ap, buffer);
    }

    return 0;
}
