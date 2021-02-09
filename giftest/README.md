# Test of animated GIF library on Pico using an ST7789 display

The animated GIF library is from here: https://github.com/bitbank2/AnimatedGIF

I haven't committed duck.h, you can generate it by getting the gif from https://media.giphy.com/media/wsUtUtLR3A2XPvfLVs/giphy-downsized.gif and formatting it with:
```
xxd -i giphy-downsized.gif duck.h
```
Then add const in front of the array name to ensure it goes into flash (it's too big for RAM).
