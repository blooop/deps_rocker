from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class DockerInDocker(SimpleRockerExtension):
    """Install Docker CE for Docker-in-Docker support. REQUIRES --privileged mode - will not function without it!"""

    name = "docker_in_docker"
    depends_on_extension = ("curl",)

    def get_docker_args(self, cliargs):
        """Return Docker arguments required for docker-in-docker to function"""
        del cliargs  # Unused but required by interface

        # Essential arguments for Docker-in-Docker
        return " --privileged --volume /var/lib/docker"

    def get_files(self, cliargs):
        """Copy the docker-init.sh and docker-entrypoint.sh scripts into the container"""
        del cliargs  # Unused but required by interface
        files = {}

        if not (docker_init_content := self.get_config_file("docker-init.sh")):
            raise FileNotFoundError("Required config file 'docker-init.sh' is missing")
        files["docker-init.sh"] = docker_init_content.decode("utf-8")

        if not (docker_entrypoint_content := self.get_config_file("docker-entrypoint.sh")):
            raise FileNotFoundError("Required config file 'docker-entrypoint.sh' is missing")
        files["docker-entrypoint.sh"] = docker_entrypoint_content.decode("utf-8")

        return files
