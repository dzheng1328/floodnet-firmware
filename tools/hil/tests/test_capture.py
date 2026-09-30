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
    assert counts == {"node": 2, "gateway": 1}
    node_lines = (tmp_path / "run" / "node.log").read_text().splitlines()
    assert [line.split(" ", 1)[1] for line in node_lines] == ["TX,66,1,0,5,ok", "TX,66,1,1,9,ok"]
    assert all(line.split(" ", 1)[0].isdigit() for line in node_lines)
    assert (tmp_path / "run" / "gateway.log").read_text().endswith("floodnet gateway\n")


def test_capture_stops_at_the_deadline(tmp_path: Path):
    endless = FakePort([b"x\n"] * 10000)
    clock = FakeClock(0.1)
    capture({"node": endless, "gateway": FakePort([])}, tmp_path / "run", 1.0, clock=clock)
    assert clock.now < 2.0
