//! Times reading a list of unrelated addresses one at a time against reading them as a batch.
//!
//! The addresses are deliberately not contiguous, so `read_32` does not apply and the choice is
//! between `read_word_32` in a loop and `read_words_32`.

use anyhow::{Context, Result};
use clap::Parser;
use probe_rs::probe::{list::Lister, WireProtocol};
use probe_rs::{MemoryInterface, Permissions, config::TargetSelector};
use std::time::{Duration, Instant};

#[derive(clap::Parser)]
struct Cli {
    #[clap(long)]
    chip: String,
    #[clap(long, default_value = "0x20000000")]
    base: String,
    #[clap(long, default_value = "40")]
    count: usize,
    #[clap(long)]
    speed: Option<u32>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let base = u64::from_str_radix(cli.base.trim_start_matches("0x"), 16)?;

    let lister = Lister::new();
    let probes = lister.list_all();
    let mut probe = probes.first().context("no probe")?.open()?;
    probe.select_protocol(WireProtocol::Swd)?;
    if let Some(speed) = cli.speed {
        probe.set_speed(speed)?;
    }

    let mut session = probe.attach(
        TargetSelector::Unspecified(cli.chip.clone()),
        Permissions::default(),
    )?;
    let mut core = session.core(0)?;
    core.halt(Duration::from_millis(200))?;

    // Spread the addresses across the region so no two land in one auto-increment window.
    let addresses: Vec<u64> = (0..cli.count).map(|i| base + (i as u64) * 64).collect();

    let start = Instant::now();
    let mut one_at_a_time = Vec::with_capacity(addresses.len());
    for &address in &addresses {
        one_at_a_time.push(core.read_word_32(address)?);
    }
    let loop_time = start.elapsed();

    let start = Instant::now();
    let mut batched = vec![0u32; addresses.len()];
    core.read_words_32(&addresses, &mut batched)?;
    let batch_time = start.elapsed();

    println!("addresses:   {}", addresses.len());
    println!(
        "one at a time: {:>9.3} ms   ({:.3} ms each)",
        loop_time.as_secs_f64() * 1e3,
        loop_time.as_secs_f64() * 1e3 / addresses.len() as f64
    );
    println!(
        "batched:       {:>9.3} ms   ({:.3} ms each)",
        batch_time.as_secs_f64() * 1e3,
        batch_time.as_secs_f64() * 1e3 / addresses.len() as f64
    );
    println!(
        "values match:  {}",
        if one_at_a_time == batched { "yes" } else { "NO" }
    );
    Ok(())
}
