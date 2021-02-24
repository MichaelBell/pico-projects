# Library to stream flash data on Pico

A simplified interface to stream a large amount of data (e.g. an image) from the on board flash.

Automatically switches to the faster SSI DMA access mode if compiled with PICO_COPY_TO_RAM.

The simplest way to use it is:
```
  flash_init(false);
  flash_set_stream(data_ptr, data_len, true);
```

Then in a loop you can fetch chunks of data with:
```
  flash_copy_data_blocking(out_ptr, len);
```
Note that data is copied out in 32-bit words and the length is specified in 32-bit words not bytes.

There is also a lower level interface to access data directly from the ringbuffer it is streamed into, in order to avoid having extra copy, see the header.

Currently it uses DMA channel 11 (hard coded).
