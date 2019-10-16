from src.apps.car_construct_map import BasicFollowApp as AppClass
from src.config.configs import AgentConfig, default_qc_moat_config, default_car_moat_config
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.rrt_drone import RRT as RRT_DRONE
from src.motion.rrt_star import RRT as RRT_CAR
from src.motion.moat_test_drone import MoatTestDrone
from src.motion.moat_withlidar import MoatWithLidar as MoatTestCar


import multiprocessing as mp
import yaml


def run_app(app_class, agent_config, moat_config):
    app = app_class(agent_config, moat_config)
    try:
        app.start()
        app.join()
    except KeyboardInterrupt:
        app.stop()
    finally:
        app.join()


def main(argv):

    cfg_yml = argv[1]

    with open(cfg_yml, 'r') as f:
        cfg = yaml.safe_load(f)

    assert isinstance(cfg, list)

    leader = 0
    plist = [2000 + i for i in range(0, len(cfg))]

    app_list = []
    for pid, device in enumerate(cfg):
        is_leader = (pid == leader)
        port = pid + 2000
        agent_config = AgentConfig(
            pid, len(cfg), "127.255.255.255",
            rport=port, plist=plist, mh=BaseMutexHandler,
            is_leader=is_leader, mhargs=[is_leader, pid]
        )

        botname = device["bot_name"]
        if device["bot_type"] == "QUAD":
            agent_config.moat_class = MoatTestDrone
            moat_config = default_qc_moat_config(botname)
            moat_config.planner = RRT_DRONE()
        elif device["bot_type"] == "CAR":
            agent_config.moat_class = MoatTestCar
            moat_config = default_car_moat_config(botname)
            moat_config.planner = RRT_CAR()

        moat_config.rospy_node = botname + '/waypoint_node'

        app_list.append(
            mp.Process(
                name=botname,
                target=run_app,
                args=(AppClass, agent_config, moat_config)
            )
        )

    for app in app_list:
        app.start()

    try:
        for app in app_list:
            app.join()
    except KeyboardInterrupt:
        print("User sent SIGINT")
        for app in app_list:
            app.join(2.0)  # Wait for a while
    finally:
        for app in app_list:
            if app.is_alive():
                print(app.name, "is still alive. Escalate to SIGTERM")
                app.terminate()


if __name__ == "__main__":
    import sys
    main(sys.argv)
