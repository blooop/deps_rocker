from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class DockerInDocker(SimpleRockerExtension):
    """
    Install Docker CE for Docker-in-Docker support.
    All user/group fixes are handled at runtime in the entrypoint script.
    REQUIRES --privileged mode - will not function without it!
    """

    name = "docker_in_docker"
    depends_on_extension = ("curl",)

    def get_docker_args(self, cliargs):
        """Return Docker arguments required for docker-in-docker to function"""
        # True DinD needs privileged; mount a persistent docker-data dir
        return " --privileged --tmpfs /run --tmpfs /var/run --volume /var/lib/docker"
