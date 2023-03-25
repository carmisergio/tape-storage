use indicatif::{ProgressBar, ProgressStyle};
use std::io;
use std::io::prelude::*;
use std::time::Duration;

pub fn wait_for_enter() {
    io::stdout().flush().unwrap();
    let mut stdin = io::stdin();
    // let mut stdout = io::stdout();

    // We want the cursor to stay at the end of the line, so we print without a newline and flush manually.
    // write!(stdout, "Press any key to continue...").unwrap();
    // stdout.flush().unwrap();

    // Read a single byte and discard
    let _ = stdin.read(&mut [0u8]).unwrap();
}

pub fn get_spinner() -> ProgressBar {
    let spinner = ProgressBar::new_spinner();
    spinner.enable_steady_tick(Duration::from_millis(100));
    spinner.set_style(
        ProgressStyle::with_template("{spinner:.white} {msg}")
            .unwrap()
            .tick_chars("\\|/--"),
    );
    spinner
}

pub fn get_progress_bar(blocks: usize) -> ProgressBar {
    let progress_bar = ProgressBar::new(blocks.try_into().unwrap());
    progress_bar.set_style(
        ProgressStyle::with_template(
            "{spinner:.white} {msg} [{elapsed_precise}] [{wide_bar:.white/white}] {pos}/{len} blocks",
        )
        .unwrap()
        .progress_chars("##-")
        .tick_chars("\\|/--")
    );
    progress_bar
}
