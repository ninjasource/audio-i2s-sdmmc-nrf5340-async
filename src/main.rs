#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem;

use bbqueue::BBBuffer;
use byteorder::{ByteOrder, LittleEndian};
use defmt::{error, info, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::gpio::AnyPin;
use embassy_nrf::{
    gpio::{Level, Output, OutputDrive},
    i2s::{self, Channels, Config, DoubleBuffering, MasterClock, SampleWidth, I2S},
    pac,
    peripherals::*,
    spim::{self, Spim},
};
use embassy_time::{Duration, Timer};
use embedded_sdmmc_async as sd;
use lc3_codec::{
    common::{
        complex::Complex,
        config::{FrameDuration, SamplingFrequency},
    },
    decoder::lc3_decoder::Lc3Decoder,
};
use {defmt_rtt as _, panic_probe as _};

use crate::file_reader::FileReader;
use crate::stats::Stats;

mod file_reader;
mod stats;

const AUDIO_FRAME_BYTES_LEN: usize = NUM_SAMPLES * mem::size_of::<Sample>();
const BB_BYTES_LEN: usize = AUDIO_FRAME_BYTES_LEN * 6;
const NUM_SAMPLES: usize = 960; // for both left and right channels
const FILE_FRAME_LEN: usize = 400; // containing 2 channels of audio
static BB: BBBuffer<BB_BYTES_LEN> = BBBuffer::new();
type Sample = i16;

bind_interrupts!(struct Irqs {
    SERIAL3 => spim::InterruptHandler<SERIAL3>;
    I2S0 => i2s::InterruptHandler<I2S0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    info!("Started");

    // change app core clock from 64mhz to 128mhz for improved performance
    let clock: pac::CLOCK = unsafe { mem::transmute(()) };
    clock.hfclkctrl.write(|w| w.hclk().div1());
    info!("Set app core to 128mhz");

    // enable flash cache for improved performance
    let cache: pac::CACHE = unsafe { mem::transmute(()) };
    cache.enable.write(|w| w.enable().enabled());
    info!("Enabled flash cache");

    // used for sending data between tasks
    let (producer, mut consumer) = BB.try_split().unwrap();

    // read SD card over SPI
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    let sdmmc_spi = spim::Spim::new(p.SERIAL3, Irqs, p.P1_01, p.P1_05, p.P1_04, config);
    let cs: AnyPin = p.P1_06.into();
    let sdmmc_cs = Output::new(cs, Level::High, OutputDrive::Standard);

    // setup the I2S peripheral
    let master_clock: MasterClock = i2s::ApproxSampleRate::_48000.into();
    let sample_rate = master_clock.sample_rate();
    info!("Sample rate: {}", sample_rate);
    let mut config = Config::default();
    config.sample_width = SampleWidth::_16bit;
    config.channels = Channels::Stereo;
    let buffers = DoubleBuffering::<Sample, NUM_SAMPLES>::new();

    // NOTE: on the UDA1334A used here tje MCK is not connected even though it is specified below
    let mut output_stream = I2S::new_master(
        p.I2S0,
        Irqs,
        p.P0_06,
        p.P0_07,
        p.P0_26,
        master_clock,
        config,
    )
    .output(p.P0_25, buffers);
    output_stream.start().await.expect("I2S Start");

    // reads and decodes audio on another task
    unwrap!(spawner.spawn(reader(sdmmc_spi, sdmmc_cs, producer)));

    loop {
        // lock free read
        match consumer.split_read() {
            Ok(read_grant) => {
                let (buf0, buf1) = read_grant.bufs();
                let src = if buf0.len() < AUDIO_FRAME_BYTES_LEN {
                    &buf1[..AUDIO_FRAME_BYTES_LEN]
                } else {
                    &buf0[..AUDIO_FRAME_BYTES_LEN]
                };

                LittleEndian::read_i16_into(src, output_stream.buffer());
                read_grant.release(AUDIO_FRAME_BYTES_LEN);
            }
            Err(_) => {
                // decoding cannot keep up with playback speed - play silence instead
                info!("silence");
                for x in output_stream.buffer() {
                    *x = 0;
                }
            }
        };

        // play stream
        if let Err(err) = output_stream.send().await {
            error!("{}", err);
        }
    }
}

#[embassy_executor::task]
async fn reader(
    spi: Spim<'static, SERIAL3>,
    cs: Output<'static, AnyPin>,
    mut producer: bbqueue::Producer<'static, BB_BYTES_LEN>,
) {
    let mut sd_card = sd::SdMmcSpi::new(spi, cs);
    let block_device = sd_card.acquire().await.unwrap();
    let mut file_reader = FileReader::new(block_device, "zane.lc3");
    file_reader.open().await;

    // configure the LC3 decoder
    const NUM_CH: usize = 2;
    const FREQ: SamplingFrequency = SamplingFrequency::Hz48000;
    const DURATION: FrameDuration = FrameDuration::TenMs;

    // create the LC3 decoder
    const SCALER_COMPLEX_LENS: (usize, usize) =
        Lc3Decoder::<NUM_CH>::calc_working_buffer_lengths(DURATION, FREQ);
    let mut complex_buf = [Complex { r: 0., i: 0. }; SCALER_COMPLEX_LENS.1];
    let mut scaler_buf = [0.0; SCALER_COMPLEX_LENS.0];
    let mut decoder = Lc3Decoder::<NUM_CH>::new(DURATION, FREQ, &mut scaler_buf, &mut complex_buf);

    let mut dec_in_buffer = [0; FILE_FRAME_LEN];
    let mut dec_out_buffer = [0; NUM_SAMPLES / NUM_CH];

    info!("decoding");
    let mut stats = Stats::new();

    loop {
        match producer.grant_exact(AUDIO_FRAME_BYTES_LEN) {
            Ok(mut wgr) => {
                stats.start_frame();

                // read a frame of audio data from the sd card
                if !file_reader.read_exact(&mut dec_in_buffer).await {
                    // start reading the file again
                    info!("start reading the file again");
                    continue;
                }

                // set num bytes to be committed (otherwise wgr.buf() may contain the wrong number of bytes)
                wgr.to_commit(AUDIO_FRAME_BYTES_LEN);

                // decode left channel
                decoder
                    .decode_frame(
                        16,
                        0,
                        &dec_in_buffer[..FILE_FRAME_LEN / NUM_CH],
                        &mut dec_out_buffer,
                    )
                    .unwrap();

                encode_to_out_buf(&dec_out_buffer, wgr.buf());

                // decode right channel
                decoder
                    .decode_frame(
                        16,
                        1,
                        &dec_in_buffer[FILE_FRAME_LEN / NUM_CH..],
                        &mut dec_out_buffer,
                    )
                    .unwrap();

                // the pcm buffer (wgr) has L-R audio samples interleved
                encode_to_out_buf(&dec_out_buffer, &mut wgr.buf()[2..]);

                stats.calc_and_print_uptime();
            }
            Err(_) => {
                // input queue full, this is normal
                // the i2s interrupt firing should free up space
                Timer::after(Duration::from_micros(1000)).await;
            }
        }
    }
}

fn encode_to_out_buf(decoder_buf: &[i16], pcm_buf: &mut [u8]) {
    // take 2 bytes at a time and skip every second chunk
    // we do this because this buffer is for stereo audio with L-R samples interleved
    for (src, dst) in decoder_buf
        .iter()
        .zip(pcm_buf.chunks_exact_mut(2).step_by(2))
    {
        LittleEndian::write_i16(dst, *src);
    }
}
