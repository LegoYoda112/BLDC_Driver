from read_hex import convert_to_16bit_aligned
import stm32bootloader

convert_to_16bit_aligned("../../firmware/build/50x50_DRIVE.hex")

# print("Entering bootloader")
# stm32bootloader.enter_bootloader()
print("Flashing")
stm32bootloader.flash_hex_USB()
print()
print("Entering app")
stm32bootloader.enter_app()