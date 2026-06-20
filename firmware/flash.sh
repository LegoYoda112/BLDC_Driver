echo "\033[1;34mFlashing with OpenOCD \033[0m"

HEX_FILE="$1"

openocd -f interface/stlink.cfg -c "transport select hla_swd" \
    -f target/stm32g4x.cfg \
    -c "program $HEX_FILE verify reset exit"