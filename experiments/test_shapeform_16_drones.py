from src.apps.shapeform import ShapeForm
from src.config.configs import AgentConfig, default_qc_moat_config, default_car_moat_config
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.rrt_drone import RRT as RRT_DRONE
from src.motion.rrt_car import RRT as RRT_CAR
from src.motion.moat_test_drone import MoatTestDrone
from src.motion.moat_test_car import MoatTestCar


import multiprocessing as mp


def run_app(app_class, agent_config, moat_config):
    app = app_class(agent_config, moat_config)
    try:
        app.start()
        app.join()
    except KeyboardInterrupt:
        app.stop()
    finally:
        app.join()


def main():
    app_class = ShapeForm
    leader = 0
    plist = list(range(2000, 2016))

    app_list = []
    for port in plist:
        pid = port - 2000
        is_leader = (pid == leader)
        agent_config = AgentConfig(
            pid, len(plist), "",
            rport=port, plist=plist, mh= BaseMutexHandler,
            is_leader=is_leader, mhargs=[is_leader, pid]
        )
        agent_config.moat_class = MoatTestDrone

        botname = 'drone' + str(pid)
        moat_config = default_qc_moat_config(botname)
        moat_config.planner = RRT_DRONE()
        moat_config.rospy_node = botname + '/waypoint_node'

        app_list.append(
            mp.Process(
                name=botname,
                target=run_app,
                args=(app_class, agent_config, moat_config)
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
    main()
