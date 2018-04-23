The files in this directory are used to construct and deploy Docker
containers that are used to deploy UxAS.

Notes:
- UxAS must be built and the file: `OpenUxAS/build/uxas` must exist.

- The Docker image `uxas_deploy` must be available on the local system or
  downloaded automatically from: the Docker Hub,
  "https://hub.docker.com/".

FILES:

- `01_buildRun_Image.sh` uses Docker commands to construct the
  `uxas_deploy` Docker image

- `02_runUxAS.sh` uses the `uxas_deploy` image to start UxAS in the local
  directory. Optionally takes an argument that points to the UxAS XML
  configuration.

- `03_StopRunContainer.sh` executes the docker command to stop the
  `uxas_deploy` container.

- `cfg_TestUxAS.xml` sample UxAS configuration file used by default in
  `02_runUxAS.sh`

- `Image/Dockerfile.uxas_deploy` is the Dockerfile used to build the
  `uxas_deploy` image.
