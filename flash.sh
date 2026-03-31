flash_cm4=0
flash_cm7=0

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        cm7) flash_cm7=1 ;;
        cm4) flash_cm4=1 ;;
        *) echo "Usage: $0 [cm7|cm4]" ; exit 1 ;;
    esac
    shift
done

if [[ $flash_cm4 -eq 1 ]]
then
    printf "\n===========================================================================\n"
    printf "Flashing CM4...\n"
    openocd -f interface/stlink.cfg \
            -f target/stm32h7x.cfg \
            -c "program CM4/build/Autonomaus_Sailboat_CM4.elf verify reset exit"
fi

if [[ $flash_cm7 -eq 1 ]]
then
    printf "\n===========================================================================\n"
    printf "Flashing CM7...\n"
    openocd -f interface/stlink.cfg \
            -f target/stm32h7x.cfg \
            -c "program CM7/build/Autonomaus_Sailboat_CM7.elf verify reset exit"
fi

printf "\n"