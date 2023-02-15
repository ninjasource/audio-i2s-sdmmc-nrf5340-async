# audio-i2s-sdmmc-nrf5340-async
 Asynchronously reads LC3 Encoded audio off an SD card and plays it through the I2S audio peripheral using Embassy on an nrf5340 DK

## Setup

This does not run out of the box. You need a special version of probe-run that fixes the `erase_all` permission problem.

```
cargo install --git https://github.com/knurling-rs/probe-run --branch erase-all
```

Then, you need a special version of embassy to get you `i2s` functionality

That can be found in the `https://github.com/ninjasource/embassy` fork in the `nrf5340-i2s` branch.