# ARM64 Docker Image Build Guide

This repository includes a GitHub Actions workflow and Dockerfile for building an ARM64 Docker image for Raspberry Pi.

Supported path:
- Build and push via GitHub Actions (recommended for reproducible, cached builds)

See the workflow in [.github/workflows/build-image-arm64.yml](.github/workflows/build-image-arm64.yml) and the Dockerfile in [.devcontainer/Dockerfile.raspi4b](.devcontainer/Dockerfile.raspi4b).

## Prerequisites
- Docker (Docker Desktop on Windows/macOS, or Docker Engine on Linux)
- Docker Hub account (only if you plan to push images)

## Build via GitHub Actions (push to Docker Hub)
This workflow builds an ARM64 image from [.devcontainer/Dockerfile.raspi](.devcontainer/Dockerfile.raspi) and pushes it to your Docker Hub namespace with a timestamp tag.

1) Create repository secrets (Settings → Secrets and variables → Actions):
   - `DOCKERHUB_USERNAME`: Your Docker Hub username
   - `DOCKERHUB_TOKEN`: Docker Hub access token (create at https://hub.docker.com/settings/security)

2) Manually run the workflow:
   - GitHub → Actions → “Build ARM64 Docker Image” → Run workflow

3) Image name and tag:
   - See the `tags:` field in [.github/workflows/build-image-arm64.yml](.github/workflows/build-image-arm64.yml)

Caching is enabled via GitHub Actions cache to speed up subsequent runs.

## Running rc_driver on Raspberry Pi
If Docker is not yet installed on the Raspberry Pi, install it first:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```

Log out and back in (or reboot) for the group membership change to take effect.

First, pull the built image on the Raspberry Pi:

```bash
docker pull DOCKERHUB_USERNAME/<image-name>:<tag>
```

Then use [docker_script/docker-run-for-raspi.sh](docker_script/docker-run-for-raspi.sh) to launch the container. The script mounts the parent directory of wherever it is invoked from, so run it from inside `docker_script/` (this mounts the repository root, which contains `ros2_ws`, to `/home/vscode/work`):

```bash
cd docker_script
./docker-run-for-raspi.sh
```

It starts the container with `--network host` (needed for ROS2 DDS discovery) and `--device=/dev/gpiochip0` plus `--group-add` for the host's `gpio` group, which together let `rc_driver` drive the Raspberry Pi's GPIO pins directly via `gpiozero` from inside the container.

## Troubleshooting
- exec format error: The container image architecture doesn’t match the host. Ensure `--platform linux/arm64` was used during build, and that you run on an arm64 host (or with emulation).
- QEMU/binfmt not registered: Run the `tonistiigi/binfmt` command above, or ensure Docker Desktop’s “Use Rosetta/Emulate architecture” features are enabled as applicable.
- Push denied or unauthorized: Verify `docker login` locally, or check `DOCKERHUB_USERNAME`/`DOCKERHUB_TOKEN` repository secrets for the GitHub Actions workflow.

## References
- Workflow: [.github/workflows/build-image-arm64.yml](.github/workflows/build-image-arm64.yml)
- Dockerfile (ARM64): [.devcontainer/Dockerfile.raspi](.devcontainer/Dockerfile.raspi)
