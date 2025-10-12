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

    def get_files(self, cliargs):
        """Copy the docker-init.sh and docker-entrypoint.sh scripts into the container"""
        files = {}
        docker_init_content = self.get_config_file("docker-init.sh")
        if not docker_init_content:
            raise FileNotFoundError("Required config file 'docker-init.sh' is missing")
        files["docker-init.sh"] = docker_init_content.decode("utf-8")

        docker_entrypoint_content = self.get_config_file("docker-entrypoint.sh")
        if not docker_entrypoint_content:
            raise FileNotFoundError("Required config file 'docker-entrypoint.sh' is missing")
        files["docker-entrypoint.sh"] = docker_entrypoint_content.decode("utf-8")

        return files
