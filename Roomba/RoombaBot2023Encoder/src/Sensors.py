import kipr


class Digital(object):
    def __init__(self, port):
        self.port = port

    def digital(self):
        return kipr.digital(self.port)


class Analog(object):
    def __init__(self, port):
        self.port = port

    def analog(self):
        return kipr.analog(self.port)