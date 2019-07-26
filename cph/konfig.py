#!/usr/bin/env python

from typing import Any, Dict, TextIO
import yaml


class Konfig:
    def __init__(self, cfg_file: TextIO):
        self.__global_cfg = yaml.safe_load(cfg_file)
        self.validate_global_config()

    def validate_global_config(self) -> bool:
        # TODO validate yaml file according to a schema
        # Some items to check
        # + Type for each field, e.g., ip, port, bot_type, etc.
        # + 'on_device' for an agent must appear in 'devices'
        # + No 'bot_name' field for all devices
        # + 'motion_automaton' for an agent must be in 'motion_automata' of the assigned device
        return self.__global_cfg is not None

    def gen_all_local_configs(self) \
            -> Dict[str, Dict[str, Any]]:
        assert self.validate_global_config()

        cfg = self.__global_cfg
        device_map = cfg['devices']
        ret = {}
        for agent in cfg['agents']:
            bot_name = agent['on_device']
            assert bot_name in device_map
            local_cfg = cfg.copy()
            local_cfg['num_agents'] = len(cfg['agents'])
            del local_cfg['agents']
            local_cfg['agent'] = agent
            del local_cfg['devices']
            assert 'bot_name' not in device_map[bot_name]
            device = {'bot_name': bot_name}
            device.update(device_map[bot_name])
            local_cfg['device'] = device

            ret[bot_name] = local_cfg

        return ret

    def dump_local_config(self, device_name: str, out_file: TextIO) -> None:
        local_cfg_map = self.gen_all_local_configs()  # FIXME caching local_cfg_map if necessary
        local_cfg = local_cfg_map[device_name]
        yaml.dump(local_cfg, out_file, allow_unicode=True)
