ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV OMNI_KIT_ACCEPT_EULA=YES

RUN --mount=type=cache,target=/root/.cache/pip,id=pip-cache pip install isaacsim[all,extscache]==4.5.0
