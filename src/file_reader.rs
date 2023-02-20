use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

use embedded_sdmmc_async::{
    BlockSpi, Controller, Directory, File, Mode, TimeSource, Timestamp, Volume, VolumeIdx,
};

const SD_CARD_CHUNK_LEN: usize = 512;

pub struct FileReader<'a, SPI, CS>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
{
    file_buffer: [u8; SD_CARD_CHUNK_LEN],
    sd_controller: Controller<BlockSpi<'a, SPI, CS>, DummyTimeSource>,
    file: Option<File>,
    volume: Option<Volume>,
    dir: Option<Directory>,
    read_index: usize,
    file_name: &'static str,
}

struct DummyTimeSource {}
impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_calendar(2022, 1, 1, 0, 0, 0).unwrap()
    }
}

impl<'a, SPI, CS> FileReader<'a, SPI, CS>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
{
    pub fn new(block_device: BlockSpi<'a, SPI, CS>, file_name: &'static str) -> Self {
        let timesource = DummyTimeSource {};
        let sd_controller = Controller::new(block_device, timesource);
        let file_buffer = [0u8; SD_CARD_CHUNK_LEN];

        Self {
            file_buffer,
            sd_controller,
            file: None,
            volume: None,
            dir: None,
            read_index: 0,
            file_name,
        }
    }

    pub async fn open(&mut self) {
        info!("read SD card success");

        let mut volume = match self.sd_controller.get_volume(VolumeIdx(0)).await {
            Ok(volume) => volume,
            Err(e) => {
                panic!("Error getting volume: {:?}", e);
            }
        };

        let dir = self.sd_controller.open_root_dir(&volume).unwrap();
        let mut file = self
            .sd_controller
            .open_file_in_dir(&mut volume, &dir, self.file_name, Mode::ReadOnly)
            .await
            .unwrap();

        // fill the file_buffer
        self.sd_controller
            .read(&volume, &mut file, &mut self.file_buffer)
            .await
            .unwrap();

        self.file = Some(file);
        self.volume = Some(volume);
        self.dir = Some(dir);
        self.read_index = 0;
    }

    // this function fills the `into_buf` with bytes (the exact amount in the buffer)
    // It reads the SD card in 512 KB chunks to prevent unecessary reads and keeps an
    // internal buffer to cache bytes for the next read call
    pub async fn read(&mut self, into_buf: &mut [u8]) -> bool {
        let volume = self.volume.as_ref().expect("file not open");

        if into_buf.len() > self.file_buffer.len() {
            panic!(
                "into_buf len too large. Max len: {}",
                self.file_buffer.len()
            );
        }

        if (self.file_buffer.len() - self.read_index) >= into_buf.len() {
            into_buf.copy_from_slice(
                &self.file_buffer[self.read_index..(self.read_index + into_buf.len())],
            );
            self.read_index += into_buf.len();
        } else {
            let num_bytes = self.file_buffer.len() - self.read_index;
            into_buf[..num_bytes].copy_from_slice(&self.file_buffer[self.read_index..]);

            let mut file = self.file.take().unwrap();

            // fill the file_buffer
            let len = self
                .sd_controller
                .read(volume, &mut file, &mut self.file_buffer)
                .await
                .unwrap();
            self.file = Some(file);

            if len != SD_CARD_CHUNK_LEN {
                // end of file reached, consume the file
                info!("end of file");
                self.close();
                self.open().await;
                return false;
            }

            let num_bytes_new = into_buf.len() - num_bytes;
            into_buf[num_bytes..].copy_from_slice(&self.file_buffer[..num_bytes_new]);
            self.read_index = num_bytes_new;
        }

        return true;
    }

    fn close(&mut self) {
        if let Some(file) = self.file.take() {
            self.sd_controller
                .close_file(self.volume.as_ref().unwrap(), file)
                .unwrap();
        }

        if let Some(dir) = self.dir.take() {
            self.sd_controller
                .close_dir(self.volume.as_ref().unwrap(), dir);
        }
    }
}
