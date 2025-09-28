from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class DockerInDocker(SimpleRockerExtension):
    """Install Docker CE for Docker-in-Docker support. Requires --privileged and specific mounts to work properly."""

    name = "docker_in_docker"
    depends_on_extension = ("curl",)

    def get_files(self, cliargs):
        """Copy the docker-init.sh and docker-entrypoint.sh scripts into the container"""
        files = {}

        docker_init_content = self.get_config_file("docker-init.sh")
        if docker_init_content:
            files["docker-init.sh"] = docker_init_content.decode("utf-8")

        docker_entrypoint_content = self.get_config_file("docker-entrypoint.sh")
        if docker_entrypoint_content:
            files["docker-entrypoint.sh"] = docker_entrypoint_content.decode("utf-8")

        return files