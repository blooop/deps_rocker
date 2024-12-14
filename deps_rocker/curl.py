from pathlib import Path
import re
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Curl(SimpleRockerExtension):
    """Adds curl to your docker container"""

    name = "curl"


