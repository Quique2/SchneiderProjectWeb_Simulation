"""rospy_stub.py — minimal rospy / std_msgs stand-in for V55 tests.

The V55 aggressive tests exercise the real schneider_state_manager source
without needing a full ROS install.  This stub injects everything the
state_manager / robot_controller / object_manager / conveyor_sim /
gripper_sim source files import from rospy and std_msgs.

It is *only* good enough for the V55 FSM tests:
  - publishers/subscribers are routed through a TopicBus
  - rospy.Time / rospy.Rate are controlled by an advancing fake clock
  - logging just appends to a list captured by the test harness
"""
from __future__ import print_function
import sys
import types
import threading


# ============================================================
# Fake clock + logging capture
# ============================================================
class _Clock(object):
    def __init__(self):
        self.t = 0.0

    def advance(self, dt):
        self.t += float(dt)

    def now(self):
        return _Time(self.t)


CLOCK = _Clock()
LOG = []
LOG_WARN = []
LOG_ERR = []


def reset():
    CLOCK.t = 0.0
    del LOG[:]
    del LOG_WARN[:]
    del LOG_ERR[:]
    BUS.subs.clear()
    BUS.published.clear()


class _Time(object):
    def __init__(self, secs):
        self.secs = float(secs)

    def to_sec(self):
        return self.secs


class _Duration(object):
    def __init__(self, secs):
        self.secs = float(secs)


# ============================================================
# Topic bus
# ============================================================
class _Publisher(object):
    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type

    def publish(self, msg):
        BUS.deliver(self.topic, msg)


class _Subscriber(object):
    def __init__(self, topic, msg_type, cb):
        self.topic = topic
        self.msg_type = msg_type
        self.cb = cb
        BUS.subs.setdefault(topic, []).append(self)


class _TopicBus(object):
    def __init__(self):
        self.subs = {}
        self.published = []

    def deliver(self, topic, msg):
        self.published.append((CLOCK.t, topic,
                               getattr(msg, "data", repr(msg))))
        for s in self.subs.get(topic, []):
            s.cb(msg)


BUS = _TopicBus()


# ============================================================
# Build the fake rospy module
# ============================================================
def init_node(name, anonymous=False):
    return None


def loginfo(fmt, *a):
    msg = (fmt % a) if a else fmt
    LOG.append(("info", CLOCK.t, str(msg)))


def logwarn(fmt, *a):
    msg = (fmt % a) if a else fmt
    LOG_WARN.append(("warn", CLOCK.t, str(msg)))


def logerr(fmt, *a):
    msg = (fmt % a) if a else fmt
    LOG_ERR.append(("err", CLOCK.t, str(msg)))


def logwarn_throttle(period, fmt, *a):
    logwarn(fmt, *a)


def logdebug(*a, **k):
    pass


def Publisher(topic, msg_type, queue_size=1, latch=False):
    return _Publisher(topic, msg_type)


def Subscriber(topic, msg_type, cb):
    return _Subscriber(topic, msg_type, cb)


def Time_now():
    return CLOCK.now()


def Duration(secs):
    return _Duration(secs)


class Rate(object):
    def __init__(self, hz):
        self.hz = float(hz)
        self.dt = 1.0 / self.hz

    def sleep(self):
        CLOCK.advance(self.dt)


def is_shutdown():
    return False


class ROSInterruptException(Exception):
    pass


def on_shutdown(_):
    pass


def install():
    """Install the rospy stub and std_msgs stubs into sys.modules."""
    rospy = types.ModuleType("rospy")
    rospy.init_node = init_node
    rospy.loginfo = loginfo
    rospy.logwarn = logwarn
    rospy.logerr = logerr
    rospy.logwarn_throttle = logwarn_throttle
    rospy.logdebug = logdebug
    rospy.Publisher = Publisher
    rospy.Subscriber = Subscriber
    rospy.Time = types.SimpleNamespace(now=Time_now)
    rospy.Duration = Duration
    rospy.Rate = Rate
    rospy.is_shutdown = is_shutdown
    rospy.ROSInterruptException = ROSInterruptException
    rospy.on_shutdown = on_shutdown
    sys.modules["rospy"] = rospy

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    def _make(name):
        cls = type(name, (), {})

        def __init__(self, data=None):
            self.data = data
        cls.__init__ = __init__
        return cls

    for n in ("Bool", "Empty", "Float32", "String", "UInt8MultiArray"):
        setattr(std_msgs_msg, n, _make(n))
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
