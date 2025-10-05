@(f"FROM {base_image} AS {builder_stage}")

ENV NODE_VERSION=20.0.0

RUN apt-get update && apt-get install -y curl
RUN mkdir -p @(f"{builder_output_dir}")
RUN touch @(f"{builder_output_dir}/node")
RUN touch @(f"{builder_output_dir}/npm")  
RUN touch @(f"{builder_output_dir}/npx")
RUN echo "export PATH=/usr/local/bin:\$PATH" > @(f"{builder_output_dir}/node-env.sh")
