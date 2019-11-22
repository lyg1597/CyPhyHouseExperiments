from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)

    def initialize_vars(self):
        self.locals['init'] = pos3d(-3., -3., 0.)
        self.locals['dest'] = pos3d(3., -1., 0.)
        self.locals['i'] = 0
        self.locals['trips'] = 0

    def loop_body(self):
        if self.locals['i'] > 200:
            self.trystop()
            return
        self.locals['i'] += 1
        # else:
        if self.locals['trips'] == 0:
            self.agent_gvh.moat.goTo(self.locals['dest'])
            self.locals['trips'] += 1
            return
        if self.agent_gvh.moat.reached:
            if (self.locals['trips'] % 2) == 1:
                self.agent_gvh.moat.goTo(self.locals['init'])
            else:
                self.agent_gvh.moat.goTo(self.locals['dest'])
            self.locals['trips'] += 1
            return
