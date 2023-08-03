class Wrench():
    def __init__(self):
        self.msg = None
        self.fx = None

    def cb(self, msg):
        self.msg = msg
        self.fx = msg.wrench.force.x
