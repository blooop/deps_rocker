from typing import Set
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension

# to run:
# isaacsim omni.isaac.sim.python.kit


class IsaacSim(SimpleRockerExtension):
    """Add isaacsim to your docker container"""

    name = "isaacsim"

    def required(self, cliargs) -> Set[str]:
        return {"nvidia", "privileged", "x11"}

    def get_docker_args(self, cliargs):
        isaac_docker_root = Path().home() / "docker/isaac-sim"

        # create the isaac docker cache folder if it does not already exist
        isaac_docker_root.mkdir(exist_ok=True, parents=True)

        volumes = [
            f"{isaac_docker_root}/cache/kit:/isaac-sim/kit/cache:rw",
            f"{isaac_docker_root}/cache/ov:/root/.cache/ov:rw",
            f"{isaac_docker_root}/cache/pip:/root/.cache/pip:rw",
            f"{isaac_docker_root}/cache/glcache:/root/.cache/nvidia/GLCache:rw",
            f"{isaac_docker_root}/cache/computecache:/root/.nv/ComputeCache:rw",
            f"{isaac_docker_root}/logs:/root/.nvidia-omniverse/logs:rw",
            f"{isaac_docker_root}/data:/root/.local/share/ov/data:rw",
            f"{isaac_docker_root}/documents:/root/Documents:rw",
        ]

        # create the commands for mounting all the volume folders
        vols = [f"-v {Path(p).absolute().as_posix()}" for p in volumes]

        args = [
            "--runtime=nvidia",
            "--network",
            "host",
            "--ipc",
            "host",
            # "-e",
            # "DISPLAY",
            "-e",
            "LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib",
            "-e",
            "ROS_DISTRO=humble",
            "-e",
            "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}",
            # "-e",
            # "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
            # "-e",
            # "FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_localhost.xml",
            # "-v",
            # "../../ros:/root/ros:rw",
            # "-v",
            # "../../bin/docker/fastdds_localhost.xml:/fastdds_localhost.xml",
            # SYS_ADMIN capability and FUSE device are required by NVIDIA Container Toolkit
            # to enable GPU support and container runtime features needed by Isaac Sim.
            # These permissions allow:
            # - GPU device mounting and management
            # - Container runtime operations for graphics/simulation
            "--cap-add",
            "SYS_ADMIN",
            "--device",
            "/dev/fuse",
        ] + vols

        run_args = " " + " ".join(args)
        print("isaac run args")
        print(run_args)
        return run_args

    @staticmethod
    def register_arguments(parser, defaults=None):
        SimpleRockerExtension.register_arguments_helper(IsaacSim.name, parser, defaults)
