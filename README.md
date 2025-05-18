# stm32-bm

A project which programs the STM32F103, STM32F401 and STM32F411 bare metal, using C and assembly.

It provides various options for clocking the STM32, and running the main loop from flash or from RAM.

Modify the #defines at the start of include.h to change the configuration.

To build and flash, ensure you have `probe-rs` installed, and a debug probe connected to your PC and your STM32Fxxx, and

```bash
make run
```

If you're not using an STM32F103CBTx - you will need to modify the `probe-rs` commands in the Makefile to use the correct chip.
