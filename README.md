# stm32-bm

A project which programs the STM32F103 bare metal, using C and assembly.

It provides various options for clocking the STM32, and running the main loop from flash or from RAM.

Modify the #defines at the start of include.h to change the configuration.

To build and flash, ensure you have `probe-rs` installed, and a debug probe connected to your PC and your STM32F103, and

```bash
make run
```

