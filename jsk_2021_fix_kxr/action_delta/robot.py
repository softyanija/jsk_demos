class Robot:
    def __init__(self):
        rospy.subscribe('/hoge', Stat, self.sub)
        self.action_service = rospy.service(...)
        # or self.actoin_topic = rospy.Publisher(...)
        pass

    def get_status(self):
        return self.status

    def sub(self, msg):
        self.status = msg

    def execute(self, action):
        self.actoin_service(call)
        # or self.actoin_topic.pub(action)

