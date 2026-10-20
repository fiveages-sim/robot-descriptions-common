"""Serialize ROS parameter files without YAML anchors/aliases unsupported by rcl."""
import yaml


class RosParameterDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def dump_parameter_yaml(parameters, stream, **kwargs):
    # Retain SafeDumper representers; only shared-object alias emission changes.
    yaml.dump(parameters, stream, Dumper=RosParameterDumper, **kwargs)
