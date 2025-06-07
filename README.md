# stm32-bm

A project which programs the STM32 using C and assembly.  The following variants are supported:

- STM32F103
- STM32F401
- STM32F411
- STM32F405

It provides various options for clocking the STM32, and running the main loop from flash or from RAM.

Modify the #defines at the start of include.h to change the configuration.

To build and flash, ensure you have `probe-rs` installed, and a debug probe connected to your PC and your STM32Fxxx, and

```bash
make TARGET=f103 run # TARGETs f103, f401, f411, f405 supported
```

The probe-rs command assumes you are using a Raspberry Pi Debug Probe.  If you aren't, you may need to change the `probe-rs` command in the Makefile (removing the `--probe xxxx:yyy` option, for example).
