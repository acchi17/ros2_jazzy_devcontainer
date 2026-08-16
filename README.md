# ARM64 Docker Image Build Guide (ROS 2 Jazzy Devcontainer)

This repository includes a GitHub Actions workflow and a devcontainer Dockerfile for building an ARM64 Docker image (e.g., for Raspberry Pi 4 or any aarch64 host).

Supported path:
- Build and push via GitHub Actions (recommended for reproducible, cached builds)

See the workflow in [.github/workflows/build-image-arm64.yml](.github/workflows/build-image-arm64.yml) and the Dockerfile in [.devcontainer/Dockerfile.raspi4b](.devcontainer/Dockerfile.raspi4b).

## Prerequisites
- Docker (Docker Desktop on Windows/macOS, or Docker Engine on Linux)
- Docker Hub account (only if you plan to push images)

## Build via GitHub Actions (push to Docker Hub)
This workflow builds an ARM64 image from [.devcontainer/Dockerfile.raspi4b](.devcontainer/Dockerfile.raspi4b) and pushes it to your Docker Hub namespace with a timestamp tag.

1) Create repository secrets (Settings → Secrets and variables → Actions):
   - `DOCKERHUB_USERNAME`: Your Docker Hub username
   - `DOCKERHUB_TOKEN`: Docker Hub access token (create at https://hub.docker.com/settings/security)

2) Manually run the workflow:
   - GitHub → Actions → “Build ARM64 Docker Image” → Run workflow

3) Image name and tag:
   - `${DOCKERHUB_USERNAME}/my-jazzy-desktop-image:YYYYMMDD-HHMMSS`

Caching is enabled via GitHub Actions cache to speed up subsequent runs.


## Run the image
On an ARM64 device (e.g., Raspberry Pi 4 running 64-bit OS):

```bash
# Pull from Docker Hub (if pushed)
docker pull <dockerhub-username>/my-jazzy-desktop-image:<tag>

# Start an interactive shell
# (Add flags like --privileged or volume mounts as needed for your use case)
docker run --rm -it <dockerhub-username>/my-jazzy-desktop-image:<tag> bash
```

On x86_64 (developer machine), you can run the arm64 image with emulation (may be slow):

```bash
docker run --rm -it <dockerhub-username>/my-jazzy-desktop-image:<tag> bash
```

## Tagging tips
To also publish a convenient `latest` tag alongside the timestamped tag using GitHub Actions, update the `tags:` in [.github/workflows/build-image-arm64.yml](.github/workflows/build-image-arm64.yml):

```yaml
         - name: Build and push ARM64 image
            uses: docker/build-push-action@v6
            with:
               context: .
               file: .devcontainer/Dockerfile.raspi4b
               platforms: linux/arm64
               push: true
               tags: |
                  ${{ secrets.DOCKERHUB_USERNAME }}/my-jazzy-desktop-image:${{ steps.datetime.outputs.tag }}
                  ${{ secrets.DOCKERHUB_USERNAME }}/my-jazzy-desktop-image:latest
               cache-from: type=gha
               cache-to: type=gha,mode=max
```

## Troubleshooting
- exec format error: The container image architecture doesn’t match the host. Ensure `--platform linux/arm64` was used during build, and that you run on an arm64 host (or with emulation).
- QEMU/binfmt not registered: Run the `tonistiigi/binfmt` command above, or ensure Docker Desktop’s “Use Rosetta/Emulate architecture” features are enabled as applicable.
- Push denied or unauthorized: Verify `docker login` locally, or check `DOCKERHUB_USERNAME`/`DOCKERHUB_TOKEN` repository secrets for the GitHub Actions workflow.

## References
- Workflow: [.github/workflows/build-image-arm64.yml](.github/workflows/build-image-arm64.yml)
- Dockerfile (ARM64): [.devcontainer/Dockerfile.raspi4b](.devcontainer/Dockerfile.raspi4b)
