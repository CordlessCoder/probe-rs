use crate::util::{cli, common_options::ProbeOptions};
use probe_rs_rpc_client::RpcClient;

/// Reset a target's non-volatile configuration to its factory default.
///
/// This is a different operation from `erase`, not a deeper one. It restores the configuration the
/// device boots with, which on some targets decides whether the device can be debugged at all, so
/// it needs `--allow-factory-reset` rather than `--allow-erase-all`.
#[derive(clap::Parser)]
pub struct Cmd {
    #[clap(flatten)]
    common: ProbeOptions,
}

impl Cmd {
    pub async fn run(self, client: RpcClient) -> anyhow::Result<()> {
        let session = cli::attach_probe(&client, self.common, None, false).await?;

        session.factory_reset(async |_event| {}).await?;

        // The device is left with nothing to run, so saying "done" on its own would be misleading:
        // the next thing to touch it will find a part that faults as soon as it is released.
        println!(
            "Factory reset complete. The device has no program on it — flash one before \
             disconnecting, or it may not be reachable again."
        );

        Ok(())
    }
}
