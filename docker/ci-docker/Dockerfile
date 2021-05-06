# General guidelines and recommendations for writing 'Dockerfile':
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/

################################################################################

# Build the base image to be used in the 'gitlab-ci' context to build other project-specific Docker images.
# TODO: Automate the process. Currently, this image needs to be pushed manually, using these commands:
# docker build . --tag registry.gitlab.com/o2ac/o2ac-ur/ci-docker:latest
# docker tag ci-docker registry.gitlab.com/o2ac/o2ac-ur/ci-docker:latest
# docker login registry.gitlab.com
# docker push registry.gitlab.com/o2ac/o2ac-ur/ci-docker:latest

# Pull the official Docker base image based on Alpine Linux.
FROM docker:latest

################################################################################

# Install additional tools missing in the official Docker base image.
# These tools are required to excute the project's scripts in GitLab CI.

# Install Docker Compose.
RUN apk update && apk upgrade \
	&& apk add build-base \
  && apk add python2-dev \
	&& apk add python3-dev \
	&& apk add libffi-dev \
	&& apk add openssl-dev \
	&& apk add py3-pip

# Dependencies for cryptography, which is a requirement for docker-compose
RUN apk add --update \
  build-base \
  cairo \
  cairo-dev \
  cargo \
  freetype-dev \
  gcc \
  gdk-pixbuf-dev \
  gettext \
  jpeg-dev \
  lcms2-dev \
  libffi-dev \
  musl-dev \
  openjpeg-dev \
  openssl-dev \
  pango-dev \
  poppler-utils \
  postgresql-client \
  postgresql-dev \
  py-cffi \
  python3-dev \
  rust \
  tcl-dev \
  tiff-dev \
  tk-dev \
  zlib-dev
RUN pip install cryptography

RUN pip install docker-compose

# Install Git.
RUN apk update && apk upgrade \
	&& apk add git

# Install Bash.
RUN apk update && apk upgrade \
	&& apk add bash
