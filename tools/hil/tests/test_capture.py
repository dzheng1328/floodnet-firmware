from pathlib import Path

from floodnet_hil.capture import capture


class FakePort:
    def __init__(self, chunks):
        self.chunks = list(chunks)

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size):
        return self.chunks.pop(0) if self.chunks else b""


class DroppingPort(FakePort):
    """Delivers its chunks, then fails the way a port does when the device resets."""

    def read(self, size):
        if not self.chunks:
            raise OSError("device disconnected")
        return super().read(size)


class FakeClock:
    def __init__(self, step):
        self.now = 0.0
        self.step = step

    def __call__(self):
        self.now += self.step
        return self.now


def test_capture_writes_complete_lines_with_times(tmp_path: Path):
    node = FakePort([b"TX,66,1,0,5,ok\r\nTX,66,1", b",1,9,ok\r\n", b"TX,66,1,2,1"])
    gateway = FakePort([b"floodnet gateway\r\n"])
    counts = capture({"node": node, "gateway": gateway}, tmp_path / "run", 1.0,
                     clock=FakeClock(0.01))
    assert (counts["node"], counts["gateway"]) == (2, 1)
    node_lines = (tmp_path / "run" / "node.log").read_text().splitlines()
    assert [line.split(" ", 1)[1] for line in node_lines] == ["TX,66,1,0,5,ok", "TX,66,1,1,9,ok"]
    assert all(line.split(" ", 1)[0].isdigit() for line in node_lines)
    assert (tmp_path / "run" / "gateway.log").read_text().endswith("floodnet gateway\n")


def test_capture_stops_at_the_deadline(tmp_path: Path):
    endless = FakePort([b"x\n"] * 10000)
    clock = FakeClock(0.1)
    capture({"node": endless, "gateway": FakePort([])}, tmp_path / "run", 1.0, clock=clock)
    assert clock.now < 2.0


def test_capture_reopens_a_port_that_drops(tmp_path: Path):
    node = DroppingPort([b"TX,66,1,0,5,ok\nTX,66,1,1,"])
    reopened = FakePort([b"booted\n", b"TX,66,2,0,5,ok\n"])
    counts = capture(
        {"node": node, "gateway": FakePort([])},
        tmp_path / "run",
        1.0,
        clock=FakeClock(0.01),
        reopen={"node": lambda: reopened, "gateway": lambda: FakePort([])},
    )
    assert counts["node"] == 3
    assert counts["node_reconnects"] == 1
    assert counts["gateway_reconnects"] == 0
    node_log = (tmp_path / "run" / "node.log").read_text().splitlines()
    lines = [line.split(" ", 1)[1] for line in node_log]
    # The partial line cut off by the reset is dropped, not glued to the next one.
    assert lines == ["TX,66,1,0,5,ok", "booted", "TX,66,2,0,5,ok"]


def test_capture_keeps_trying_a_port_that_stays_gone(tmp_path: Path):
    def refuse():
        raise OSError("no such device")

    counts = capture(
        {"node": DroppingPort([]), "gateway": FakePort([b"x\n"])},
        tmp_path / "run",
        1.0,
        clock=FakeClock(0.01),
        reopen={"node": refuse, "gateway": lambda: FakePort([])},
    )
    assert counts["gateway"] == 1
    assert counts["node"] == 0
