# SD Audio player

This plays a PCM audio file from an SD card.

To extract audio in the right format from an mp4 do:
```
ffmpeg -i video.mp4 -ac 2 -f s16le -c:a pcm_s16le -ar 44100 audio.pcm
```

Then write the PCM file starting at  a particular sector (in this case 3200000) on the SD card with:
```
dd if=audio.pcm of=/dev/sdc bs=512 seek=3200000
```

Set the sector and file length in the defines in sdaudio.c