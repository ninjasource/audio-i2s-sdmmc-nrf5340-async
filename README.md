# audio-i2s-sdmmc-nrf5340-async
Asynchronously reads LC3 Encoded audio off an SD card and plays it through the I2S audio peripheral using Embassy on an nrf5340 DK

This application is configured to run on the app core in secure mode.
This means that we do not require a boot loader to unlock the peripherals we need as everything is already unlocked.
There is no performance difference between secure and non-secure mode, only developer pain.

For performance reasons the mcu is set to run at 128 MHz and flash cache is enabled.

## Hardware Setup

![Example Setup](https://github.com/ninjasource/audio-i2s-sdmmc-nrf5340-async/blob/main/nrf5340-sd-i2s.jpg?raw=true)

- `nRF5340-DK` development kit
- `UDA1334A` I2S DAC (for audio)
- Pins (DAC -> dk): 
  - 3.3v -> 3.3v
  - GND  -> GND
  - WSEL -> P0_26
  - DIN  -> P0_25
  - BCLK -> P0_07
- `Deek-Robot DK Data Logging Module` - Spi SD Card reader
- Pins (SD Card -> dk)
  - 5v   -> 3.3v (yes, this module works with 3.3v and I would NOT power it with 5v for the nrf5340's sake)
  - GND  -> GND
  - SCL  -> P1.01
  - MISO -> P1.05
  - MOSI -> P1.04
  - CS   -> P1.06

NOTE: Any SPI compatible SD Card reader should work.

## Software Setup

This does not run out of the box. You need a special version of probe-run that fixes the `erase_all` permission problem for the nrf5340 chip.
You will most likely need to remove your existing version of `probe-run` before running the following:

```
cargo install --git https://github.com/knurling-rs/probe-run --branch erase-all
```

Also, make sure you have the correct target installed:

```
rustup target add thumbv8m.main-none-eabihf
```

Copy the `zane.lc3` file to the root folder of a `FAT32` formatted microSD card which goes in the sd card reader above.

Note that a forked version of the embassy library is used for `i2s` functionality as well as a forked version of the embedded-sdmmc-async library for async SD card reading.
See Cargo.toml for more details.

## Other notes

You can use the `lc3-codec` repo to create your own LC3 encoded files from wav files if you like. See examples section.
Just make sure the filenames are no more than 8 characters long (including the extension) to play nicely with the FAT32 filesystem driver.

## Credits

The audio file is a recording from the late jazz musician, Zane Musa. 
The piece was chosen because it contains both a wind instrument (high spectral resolution) and castanets (high temporal resolution).