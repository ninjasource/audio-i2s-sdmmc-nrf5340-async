use defmt::info;
use embassy_time::{Duration, Instant};

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
