#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem;

use bbqueue::BBBuffer;
use byteorder::{ByteOrder, LittleEndian};
use defmt::{error, info, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::i2s::{self, Channels, Config, DoubleBuffering, MasterClock, SampleWidth, I2S};
use embassy_nrf::peripherals::*;
use embassy_nrf::spim::Spim;
use embassy_nrf::{interrupt, pac, spim};
use embassy_time::{Duration, Instant, Timer};
use embedded_sdmmc_async as sd;
use lc3_codec::common::complex::Complex;
use lc3_codec::common::config::{FrameDuration, SamplingFrequency};
use lc3_codec::decoder::lc3_decoder::Lc3Decoder;
use {defmt_rtt as _, panic_probe as _};

const AUDIO_FRAME_BYTES_LEN: usize = NUM_SAMPLES * mem::size_of::<Sample>();
const BB_BYTES_LEN: usize = AUDIO_FRAME_BYTES_LEN * 6;
static BB: BBBuffer<BB_BYTES_LEN> = BBBuffer::new();
type Sample = i16;

const NUM_SAMPLES: usize = 960;
//const INPUT_FILE: &'static [u8] = include_bytes!("../../48khz_16bit_mono_10ms_150byte_piano.lc3");
const FILE_FRAME_LEN: usize = 300;

mod file_reader;
use file_reader::FileReader;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    info!("Started");

    // change app core clock from 64mhz to 128mhz
    let clock: pac::CLOCK = unsafe { mem::transmute(()) };
    clock.hfclkctrl.write(|w| w.hclk().div1());
    info!("Set app core to 128mhz");

    // enable flash cache
    let cache: pac::CACHE = unsafe { mem::transmute(()) };
    cache.enable.write(|w| w.enable().enabled());
    info!("Enabled flash cache");

    // read SD card over SPI
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    let irq = interrupt::take!(SERIAL3);
    let sdmmc_spi = spim::Spim::new(p.UARTETWISPI3, irq, p.P1_01, p.P1_05, p.P1_04, config);
    let sdmmc_cs = Output::new(p.P1_06, Level::High, OutputDrive::Standard);

    let (producer, mut consumer) = BB.try_split().unwrap();

    unwrap!(spawner.spawn(reader(sdmmc_spi, sdmmc_cs, producer)));

    let master_clock: MasterClock = i2s::ApproxSampleRate::_48000.into();
    let sample_rate = master_clock.sample_rate();
    info!("Sample rate: {}", sample_rate);
    let mut config = Config::default();
    config.sample_width = SampleWidth::_16bit;
    config.channels = Channels::Stereo;
    let irq = interrupt::take!(I2S0);
    let buffers = DoubleBuffering::<Sample, NUM_SAMPLES>::new();
    let mut output_stream =
        I2S::master(p.I2S0, irq, p.P0_06, p.P0_07, p.P0_26, master_clock, config)
            .output(p.P0_25, buffers);
    output_stream.start().await.expect("I2S Start");

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
                // info!("silence");
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
    spi: Spim<'static, UARTETWISPI3>,
    cs: Output<'static, P1_06>,
    mut producer: bbqueue::Producer<'static, BB_BYTES_LEN>,
) {
    let mut sd_card = sd::SdMmcSpi::new(spi, cs);
    let block_device = sd_card.acquire().await.unwrap();
    let mut file_reader = FileReader::new(block_device, "classic.lc3");
    file_reader.open().await;

    // configure the LC3 decoder
    const NUM_CH: usize = 2;
    const FREQ: SamplingFrequency = SamplingFrequency::Hz48000;
    const DURATION: FrameDuration = FrameDuration::TenMs;

    // TODO: upgrade to use heapless pool for buffers (this is horrible)
    const SCALER_COMPLEX_LENS: (usize, usize) =
        Lc3Decoder::<NUM_CH>::calc_working_buffer_lengths(DURATION, FREQ);
    static mut SCALER_BUF: [f32; SCALER_COMPLEX_LENS.0] = [0.0; SCALER_COMPLEX_LENS.0];
    static mut COMPLEX_BUF: [Complex; SCALER_COMPLEX_LENS.1] =
        [Complex { r: 0., i: 0. }; SCALER_COMPLEX_LENS.1];
    let mut decoder =
        Lc3Decoder::<NUM_CH>::new(DURATION, FREQ, unsafe { &mut SCALER_BUF }, unsafe {
            &mut COMPLEX_BUF
        });

    let mut dec_in_buffer = [0; FILE_FRAME_LEN];
    let mut dec_out_buffer = [0; NUM_SAMPLES / NUM_CH];

    info!("decoding");
    let mut stats = Stats::new();

    loop {
        match producer.grant_exact(AUDIO_FRAME_BYTES_LEN) {
            Ok(mut wgr) => {
                stats.start_frame();

                // read a frame of audio data from the sd card
                if !file_reader.read(&mut dec_in_buffer).await {
                    // start reading the file again
                    info!("start reading the file again");
                    continue;
                }

                // set num bytes to be committed
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

pub struct Stats {
    elapsed_micros: [usize; 100],
    elapsed_micros_index: usize,
    num_secs: usize,
    start_frame_micros: u64,
    start_period_micros: u64, // a period of 1 second
    pub frame_index: usize,
}

impl Stats {
    pub fn new() -> Self {
        Self {
            elapsed_micros: [0; 100],
            elapsed_micros_index: 0,
            start_period_micros: uptime().as_micros(),
            start_frame_micros: uptime().as_micros(),
            num_secs: 0,
            frame_index: 0,
        }
    }
    pub fn start_frame(&mut self) {
        self.start_frame_micros = uptime().as_micros();
    }

    pub fn calc_and_print_uptime(&mut self) {
        // calc avg micros
        let elapsed = (uptime().as_micros() - self.start_frame_micros) as usize;
        self.elapsed_micros[self.elapsed_micros_index] = elapsed;
        self.elapsed_micros_index = (self.elapsed_micros_index + 1) % self.elapsed_micros.len();

        // print average number of microseconds needed to decode each 10ms frame of audio
        if elapsed > 15000 {
            info!(
                "expensive frame at index {}: {} micros per frame",
                self.frame_index, elapsed
            );
        }

        if (uptime().as_micros() - self.start_period_micros) > 1000000 {
            self.num_secs += 1;
            let mins = self.num_secs / 60;
            let secs = self.num_secs % 60;
            let avg_micros = self.elapsed_micros.iter().sum::<usize>() / self.elapsed_micros.len();
            let max_micros = self.elapsed_micros.iter().max().unwrap();
            info!(
                "uptime: {} mins {} secs, avg micros: {} max micros: {}",
                mins, secs, avg_micros, max_micros
            );
            self.start_period_micros = uptime().as_micros();
        }

        self.frame_index += 1;
    }
}

pub fn uptime() -> Duration {
    Instant::now().duration_since(Instant::MIN)
}
