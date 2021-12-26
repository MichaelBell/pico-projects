uf2conv.py -f rp2040 -b 0x10010000 $1 -o gifdata.uf2
stat --printf="Data size to set in giftest.c: %s\n" $1
