from typing import Set
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension

# to run:
# isaacsim omni.isaac.sim.python.kit


class IsaacSim(SimpleRockerExtension):
    """Add isaacsim to your docker container""" 

    name = "isaacsim"

    def required(self, cliargs) -> Set[str]:
        return set(["nvidia", "privileged", "x11"])

    def get_docker_args(self, cliargs):
        volumes = [
            "~/.local/share/kinisi/issac_sim/cache/kit:/isaac-sim/kit/cache:rw",
            "~/.local/share/kinisi/issac_sim/cache/ov:/root/.cache/ov:rw",
            "~/.local/share/kinisi/issac_sim/cache/pip:/root/.cache/pip:rw",
            "~/.local/share/kinisi/issac_sim/cache/glcache:/root/.cache/nvidia/GLCache:rw",
            "~/.local/share/kinisi/issac_sim/cache/computecache:/root/.nv/ComputeCache:rw",
            "~/.local/share/kinisi/issac_sim/logs:/root/.nvidia-omniverse/logs:rw",
            "~/.local/share/kinisi/issac_sim/data:/root/.local/share/ov/data:rw",
            "~/.local/share/kinisi/issac_sim/documents:/root/Documents:rw",
        ]

        vols = ["-v " + Path(p).absolute().as_posix() for p in volumes]

        args = [
            "--runtime=nvidia",
            "--network",
            "host",
            "--ipc",
            "host",
            # "-e",
            # "DISPLAY",
            # "-e",
            # "LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib",
            # "-e",
            # "ROS_DISTRO=humble",
            # "-e",
            # "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}",
            # "-e",
            # "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
            # "-e",
            # "FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_localhost.xml",
            # "-v",
            # "../../ros:/root/ros:rw",
            # "-v",
            # "../../bin/docker/fastdds_localhost.xml:/fastdds_localhost.xml",
            "--cap-add",
            "SYS_ADMIN",
            "--device",
            "/dev/fuse",
        ] + vols

        run_args = " " + " ".join(args)
        print(run_args)
        return run_args

    @staticmethod
    def register_arguments(parser, defaults=None):
        SimpleRockerExtension.register_arguments_helper(IsaacSim.name, parser, defaults)
