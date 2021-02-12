f = open("flash_data.h", "w")

print("const uint32_t flash_data[] = {", file=f)

for i in range(256):
    print("  %d," % i, file=f)

print("};", file=f)
