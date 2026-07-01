use std::{
    boxed::Box,
    fs::{create_dir_all, read_dir, remove_file},
    future::Future,
    path::{Path, PathBuf},
    pin::Pin,
    sync::Arc,
    time::Duration,
};

use color_eyre::{
    Result,
    eyre::{Context as _, ensure},
};
use ros_z::{prelude::*, time::Time};
use ros2::sensor_msgs::image::Image;
use serde::{Deserialize, Serialize};
use tokio::task;
use tracing::warn;
use types::time_wrapper::TimeWrapper;

const FILE_PREFIX: &str = "left_image_";
const FILE_EXTENSION: &str = "png";
const MINIMUM_SAVE_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
#[serde(deny_unknown_fields)]
pub struct Parameters {
    pub enabled: bool,
    pub save_interval: Duration,
    pub output_directory: PathBuf,
    pub maximum_saved_images: Option<usize>,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("image_saver").build().await?;

    let parameters = node.bind_parameter_as::<Parameters>("image_saver")?;
    let left_image_sub = node
        .subscriber::<TimeWrapper<Image>>("inputs/left_image")
        .build()
        .await?;

    let mut timer = node.create_timer(effective_save_interval(
        parameters.snapshot().typed().save_interval,
    ));
    let mut latest_image = None;
    let mut last_saved_time = None;

    loop {
        tokio::select! {
            image = left_image_sub.recv() => {
                latest_image = Some(image?);
            }
            _ = timer.tick() => {
                let parameters_snapshot = parameters.snapshot();
                let parameters = parameters_snapshot.typed.clone();
                timer.set_period(effective_save_interval(parameters.save_interval));
                timer.reset();

                if !parameters.enabled {
                    continue;
                }

                let Some(timed_image) = latest_image.clone() else {
                    continue;
                };
                if last_saved_time == Some(timed_image.time) {
                    continue;
                }

                let image_time = timed_image.time;
                if let Err(error) = save_image(timed_image, parameters).await {
                    warn!("failed to save left image: {error:#}");
                    continue;
                }
                last_saved_time = Some(image_time);
            }
        }
    }
}

fn effective_save_interval(save_interval: Duration) -> Duration {
    save_interval.max(MINIMUM_SAVE_INTERVAL)
}

async fn save_image(
    timed_image: TimeWrapper<Image>,
    parameters: Arc<Parameters>,
) -> Result<PathBuf> {
    task::spawn_blocking(move || {
        let output_directory = &parameters.output_directory;
        ensure!(
            !output_directory.as_os_str().is_empty(),
            "output directory must not be empty"
        );

        create_dir_all(output_directory).wrap_err_with(|| {
            format!(
                "failed to create image saver output directory '{}'",
                output_directory.display()
            )
        })?;

        let path = image_path(output_directory, timed_image.time);
        save_png(timed_image.inner, &path)?;
        prune_old_images(output_directory, parameters.maximum_saved_images)?;

        Ok(path)
    })
    .await
    .wrap_err("image saver task failed")?
}

fn image_path(output_directory: &Path, time: Time) -> PathBuf {
    output_directory.join(format!(
        "{FILE_PREFIX}{:019}.{FILE_EXTENSION}",
        time.as_nanos()
    ))
}

fn save_png(image: Image, path: &Path) -> Result<()> {
    image
        .save_to_file(path)
        .wrap_err_with(|| format!("failed to save PNG image file '{}'", path.display()))
}

fn prune_old_images(output_directory: &Path, maximum_saved_images: Option<usize>) -> Result<()> {
    let Some(maximum_saved_images) = maximum_saved_images else {
        return Ok(());
    };

    let mut paths = read_dir(output_directory)
        .wrap_err_with(|| {
            format!(
                "failed to read image saver output directory '{}'",
                output_directory.display()
            )
        })?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| {
            if !path.is_file() {
                return false;
            }
            let Some(file_name) = path.file_name().and_then(|file_name| file_name.to_str()) else {
                return false;
            };
            file_name.starts_with(FILE_PREFIX)
                && path
                    .extension()
                    .is_some_and(|extension| extension.eq_ignore_ascii_case(FILE_EXTENSION))
        })
        .collect::<Vec<_>>();

    let number_to_remove = paths.len().saturating_sub(maximum_saved_images);
    if number_to_remove == 0 {
        return Ok(());
    }

    paths.sort();
    for path in paths.into_iter().take(number_to_remove) {
        remove_file(&path)
            .wrap_err_with(|| format!("failed to remove old image file '{}'", path.display()))?;
    }

    Ok(())
}
