# syntax=docker/dockerfile:1.4
ARG FZF_VERSION=@fzf_version@

FROM @base_image@ AS @builder_stage@

ADD https://github.com/junegunn/fzf.git#master /tmp/fzf

RUN set -euxo pipefail; \
    mkdir -p @builder_output_dir@; \
    cp -a /tmp/fzf @builder_output_dir@/fzf
