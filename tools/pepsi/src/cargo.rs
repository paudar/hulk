use std::{
    collections::HashMap,
    env::current_dir,
    ffi::{OsStr, OsString},
    path::{Path, PathBuf, absolute},
};

use clap::Args;
use color_eyre::{
    Result,
    eyre::{Context, ContextCompat, bail},
};
use environment::{Environment, EnvironmentArguments};
use lazy_static::lazy_static;
use pathdiff::diff_paths;
use repository::{Repository, cargo::Cargo, sdk::ContainerRuntime};
use serde::Deserialize;
use tokio::{fs::read_to_string, process::Command};
use toml::Table;
use tracing::debug;

pub mod build;
pub mod check;
pub mod clippy;
pub mod common;
pub mod environment;
pub mod install;
pub mod nextest;
pub mod run;
pub mod test;
mod heading {
    pub const PACKAGE_SELECTION: &str = "Package Selection";
    pub const TARGET_SELECTION: &str = "Target Selection";
    pub const FEATURE_SELECTION: &str = "Feature Selection";
    pub const COMPILATION_OPTIONS: &str = "Compilation Options";
    pub const MANIFEST_OPTIONS: &str = "Manifest Options";
}

lazy_static! {
    pub static ref MANIFEST_PATHS: HashMap<&'static str, &'static str> = {
        HashMap::from([
            ("aliveness", "services/aliveness"),
            ("annotato", "tools/annotato"),
            ("depp", "tools/depp"),
            ("pepsi", "tools/pepsi"),
            ("ros-z-cli", "crates/ros-z-cli"),
            ("twix", "tools/twix"),
            ("widget_gallery", "tools/widget_gallery"),
        ])
    };
}

#[derive(Args)]
#[group(skip)]
pub struct Arguments<CargoArguments: Args> {
    pub manifest: Option<OsString>,
    #[command(flatten)]
    pub environment: EnvironmentArguments,
    #[command(flatten, next_help_heading = "Cargo Options")]
    pub cargo: CargoArguments,
}

pub trait CargoCommand {
    const SUB_COMMAND: &'static str;

    fn apply(&self, cmd: &mut Cargo);
    fn profile(&self) -> &str;

    fn selected_packages(&self) -> &[String] {
        &[]
    }
}

pub async fn cargo<CargoArguments: Args + CargoCommand>(
    arguments: Arguments<CargoArguments>,
    repository: &Repository,
    compiler_artifacts: &[impl AsRef<Path>],
) -> Result<()> {
    let mut cargo_command =
        construct_cargo_command(arguments, repository, compiler_artifacts).await?;

    debug!("Running `{cargo_command:?}`");
    let status = cargo_command
        .status()
        .await
        .wrap_err("failed to run cargo")?;

    if !status.success() {
        bail!("cargo failed with {status}");
    }

    Ok(())
}

pub async fn construct_cargo_command<CargoArguments: Args + CargoCommand>(
    arguments: Arguments<CargoArguments>,
    repository: &Repository,
    compiler_artifacts: &[impl AsRef<Path>],
) -> Result<tokio::process::Command, color_eyre::eyre::Error> {
    let manifest_path = match arguments.manifest {
        Some(manifest) => {
            let absolute_manifest = resolve_manifest_path(&manifest, repository)
                .await
                .wrap_err("failed to resolve manifest path")?;
            let relative_manifest = diff_paths(
                absolute_manifest,
                &current_dir().wrap_err("failed to get current directory")?,
            )
            .wrap_err("failed to express manifest relative to repository root")?;

            Some(relative_manifest)
        }
        None => None,
    };
    let environment = match arguments.environment.env {
        Some(environment) => environment,
        None => read_requested_environment(
            &manifest_path,
            CargoArguments::SUB_COMMAND,
            arguments.cargo.selected_packages(),
            repository,
        )
        .await
        .wrap_err("failed to read requested environment")?,
    }
    .resolve(repository)
    .await
    .wrap_err("failed to resolve environment")?;
    let mut cargo = if arguments.environment.remote {
        Cargo::remote(environment)
    } else {
        Cargo::local(environment)
    };
    cargo
        .setup(repository)
        .await
        .wrap_err("failed to set up cargo environment")?;
    cargo.arg(CargoArguments::SUB_COMMAND);
    if let Some(manifest_path) = manifest_path {
        if CargoArguments::SUB_COMMAND == "install" {
            cargo.arg("--path");
            cargo.arg(
                manifest_path
                    .parent()
                    .wrap_err("failed to retrieve package path from manifest path")?,
            );
        } else {
            cargo.arg("--manifest-path");
            cargo.arg(manifest_path);
        }
    }
    arguments.cargo.apply(&mut cargo);
    let cargo_command = cargo
        .command(repository, compiler_artifacts)
        .wrap_err("failed to create cargo command")?;

    Ok(cargo_command)
}

async fn read_requested_environment(
    manifest_path: &Option<PathBuf>,
    cargo_subcommand: &str,
    selected_packages: &[String],
    repository: &Repository,
) -> Result<Environment> {
    if let Some(manifest_path) = manifest_path {
        let manifest = read_to_string(manifest_path).await.wrap_err_with(|| {
            format!(
                "failed to read manifest at {path}",
                path = manifest_path.display()
            )
        })?;
        let manifest: Table = toml::from_str(&manifest).wrap_err("failed to parse manifest")?;

        if manifest_requests_cross_compile(&manifest) {
            return Ok(sdk_environment_for_host());
        }

        if !selected_packages.is_empty() {
            return Ok(
                if selected_packages_request_cross_compile(selected_packages, repository).await? {
                    sdk_environment_for_host()
                } else {
                    Environment::Native
                },
            );
        }

        if manifest.get("package").is_none()
            && manifest.get("workspace").is_some()
            && workspace_command_uses_sdk(cargo_subcommand)
        {
            return Ok(sdk_environment_for_host());
        }

        return Ok(Environment::Native);
    }

    if !selected_packages.is_empty() {
        return Ok(
            if selected_packages_request_cross_compile(selected_packages, repository).await? {
                sdk_environment_for_host()
            } else {
                Environment::Native
            },
        );
    }

    Ok(if workspace_command_uses_sdk(cargo_subcommand) {
        sdk_environment_for_host()
    } else {
        Environment::Native
    })
}

fn manifest_requests_cross_compile(manifest: &Table) -> bool {
    package_metadata(manifest)
        .and_then(|metadata| metadata.get("cross-compile"))
        .and_then(|value| value.as_bool())
        .unwrap_or(false)
}

fn workspace_command_uses_sdk(cargo_subcommand: &str) -> bool {
    cfg!(all(target_os = "macos", target_arch = "aarch64"))
        && matches!(
            cargo_subcommand,
            "build" | "check" | "clippy" | "nextest run" | "test"
        )
}

#[derive(Deserialize)]
struct CargoMetadata {
    packages: Vec<CargoMetadataPackage>,
}

#[derive(Deserialize)]
struct CargoMetadataPackage {
    name: String,
    metadata: serde_json::Value,
}

async fn selected_packages_request_cross_compile(
    selected_packages: &[String],
    repository: &Repository,
) -> Result<bool> {
    let output = Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .current_dir(&repository.root)
        .output()
        .await
        .wrap_err("failed to read Cargo workspace metadata")?;
    if !output.status.success() {
        bail!(
            "cargo metadata failed with {status}: {stderr}",
            status = output.status,
            stderr = String::from_utf8_lossy(&output.stderr),
        );
    }

    let metadata: CargoMetadata = serde_json::from_slice(&output.stdout)
        .wrap_err("failed to parse Cargo workspace metadata")?;

    Ok(metadata.packages.iter().any(|package| {
        selected_packages.contains(&package.name)
            && package
                .metadata
                .get("pepsi")
                .and_then(|metadata| metadata.get("cross-compile"))
                .and_then(serde_json::Value::as_bool)
                .unwrap_or(false)
    }))
}

fn sdk_environment_for_host() -> Environment {
    match ContainerRuntime::default_for_host() {
        ContainerRuntime::Podman => Environment::Podman { image: None },
        ContainerRuntime::Docker => Environment::Docker { image: None },
    }
}

async fn resolve_manifest_path(
    manifest: impl AsRef<OsStr>,
    repository: &Repository,
) -> Result<PathBuf> {
    let manifest = manifest.as_ref();

    let manifest_path = manifest
        .to_str()
        .and_then(|manifest| {
            MANIFEST_PATHS
                .get(manifest)
                .map(|manifest_path| repository.root.join(manifest_path))
        })
        .unwrap_or(manifest.into());

    let manifest_path =
        absolute(manifest_path).wrap_err("failed to get absolute path of manifest")?;

    let metadata = tokio::fs::metadata(&manifest_path)
        .await
        .wrap_err_with(|| {
            format!(
                "failed to retrieve metadata for {manifest_path}",
                manifest_path = manifest_path.to_string_lossy()
            )
        })?;

    Ok(if metadata.is_dir() {
        manifest_path.join("Cargo.toml")
    } else {
        manifest_path
    })
}

fn package_metadata(table: &Table) -> Option<&Table> {
    table
        .get("package")?
        .get("metadata")?
        .get("pepsi")?
        .as_table()
}
