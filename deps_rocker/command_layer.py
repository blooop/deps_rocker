import em
import pkgutil
from dataclasses import dataclass, field
from typing import List


@dataclass
class CommandLayer:
    layer: str = None
    command: str = None
    data: set = field(default_factory=set)

    def get_filename(self):
        return f"{self.command}_{self.layer}"

    def update(self, key: str, value: List):
        split = key.split("_")
        self.command = split[0]
        if len(split) > 1:
            self.layer = split[1]
        else:
            self.layer = "default"
        if isinstance(value, list):
            for v in value:
                self.data.add(v)
        else:
            self.data.add(value)

    def to_snippet(self):
        snippet = pkgutil.get_data(
            "deps_rocker", f"templates/{self.command}_snippet.Dockerfile"
        ).decode("utf-8")

        empy_data = dict(
            data_list=list(sorted(self.data)),
            filename=self.get_filename(),
            layer_name=self.layer,
        )
        print("empy_snippet", snippet)
        print("empy_data", empy_data)

        return em.expand(snippet, empy_data)
