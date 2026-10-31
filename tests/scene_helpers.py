"""Explicit initial wall observations for pure geometry regression fixtures."""
from object_tracker import runtime_config as observed_config
from pose_provider import WorldPose


def runtime_config(config, overrides):
    poses={o['id']:WorldPose(*o['center'],o['size'][2]/2,0,0,0)
           for o in config['walls']+config['objects']}
    poses.update(overrides)
    return observed_config(config,poses)
