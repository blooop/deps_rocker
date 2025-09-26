import os
import pwd
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class SshClient(SimpleRockerExtension):
    """Install openssh-client and mount the user's ~/.ssh directory into the container."""

    name = "ssh_client"
    depends = ["ssh", "user"]
    apt_packages = ["openssh-client"]

    def get_docker_args(self, cliargs):
        # Mount the entire ~/.ssh directory from the host to the container

        ssh_dir_host = os.path.expanduser("~/.ssh")
        if not os.path.exists(ssh_dir_host):
            import logging

            logging.warning(
                "Host ~/.ssh directory does not exist. SSH keys will not be mounted into the container."
            )
            return ""

        # Determine the container home directory by checking the same arguments
        # the 'user' extension would use.
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir

        if not container_home:
            print(
                "Warning: Could not determine container home directory. Cannot mount .ssh directory."
            )
            return ""

        target_path = f"{container_home}/.ssh"
        return f' -v "{ssh_dir_host}:{target_path}"'
