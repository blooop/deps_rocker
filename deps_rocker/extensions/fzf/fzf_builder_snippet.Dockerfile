FROM @base_image@ AS @builder_stage@

ADD https://github.com/junegunn/fzf.git#master /tmp/fzf

RUN set -euxo pipefail; \
    mkdir -p @builder_output_dir@; \
    cp -a /tmp/fzf @builder_output_dir@/fzf
