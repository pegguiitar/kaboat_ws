"""L3 전체 흐름 시험용 가짜 토픽 버스 — DDS 없이 노드끼리 메시지를 주고받는다.

각 노드의 publisher 를 BusPublisher 로 바꾸고, 실제 subscription 과 같은 콜백을
버스에 등록한다. 발행하면 같은 토픽의 콜백이 즉시(동기) 불린다.
latched(transient_local) 토픽은 마지막 메시지를 기억했다가 늦게 붙은 구독자에게도 준다.
"""
from collections import defaultdict

LATCHED = {'/mission/state', '/mission/goal', '/detector/enable'}


class Bus:
    def __init__(self):
        self.subs = defaultdict(list)
        self.log = []                 # [(topic, msg)] — 발행 순서 그대로
        self.latched = {}

    def subscribe(self, topic, callback):
        self.subs[topic].append(callback)
        if topic in self.latched:
            callback(self.latched[topic])

    def publish(self, topic, msg):
        self.log.append((topic, msg))
        if topic in LATCHED:
            self.latched[topic] = msg
        for callback in list(self.subs[topic]):
            callback(msg)

    def publisher(self, topic):
        return BusPublisher(self, topic)

    def messages(self, topic):
        return [m for t, m in self.log if t == topic]


class BusPublisher:
    def __init__(self, bus, topic):
        self.bus = bus
        self.topic = topic

    def publish(self, msg):
        self.bus.publish(self.topic, msg)


def wire_mission_manager(bus, node):
    node.pub_state = bus.publisher('/mission/state')
    node.pub_goal = bus.publisher('/mission/goal')
    node.pub_entry_goal = bus.publisher('/mission/entry_goal')
    node.pub_detector_enable = bus.publisher('/detector/enable')
    bus.subscribe('/mission/complete', node.on_complete)
    bus.subscribe('/odom', node.on_odom)


def wire_behavior(bus, node):
    node.cmd_pub = bus.publisher(node.CMD_TOPIC)
    node.complete_pub = bus.publisher('/mission/complete')
    bus.subscribe('/mission/state', node._on_state)
    bus.subscribe('/mission/goal', node._on_goal)
    bus.subscribe('/odom', node._on_odom)
    if hasattr(node, '_on_entry_goal'):
        bus.subscribe('/mission/entry_goal', node._on_entry_goal)


def wire_cmd_mux(bus, node, topics):
    node.pub = bus.publisher('/cmd_vel')
    bus.subscribe('/mission/state', node.on_state)
    for topic in topics:
        bus.subscribe(topic, lambda msg, t=topic: node.on_cmd(t, msg))
