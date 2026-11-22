"""Offline tests of the low-latency Gazebo pose stream."""

from __future__ import annotations

import io
import math

import pytest

import gz_pose_stream as gps

TEXT = """\
header {
  stamp {
    sec: 12
    nsec: 5000000
  }
}
pose {
  name: "floor"
  id: 3
}
pose {
  name: "reely_new1"
  id: 7
  position {
    x: 1.25
    y: -0.5
    z: 0.0675
  }
  orientation {
    z: 0.38268343236508978
    w: 0.92387953251128674
  }
}
pose {
  name: "front_left_wheel"
  position {
    x: 9
  }
}
header {
  stamp {
    sec: 12
    nsec: 15000000
  }
}
pose {
  name: "reely_new1"
  position {
    y: 2
  }
  orientation {
    w: 1
  }
}
"""


def test_poses_stamps_and_omitted_fields():
    out = list(gps.stream_model_poses(TEXT.splitlines(), "reely_new1"))
    assert len(out) == 2
    (t0, x0, y0, yaw0), (t1, x1, y1, yaw1) = out
    assert t0 == pytest.approx(12.005) and (x0, y0) == (1.25, -0.5) and yaw0 == pytest.approx(math.pi / 4)
    assert t1 == pytest.approx(12.015) and (x1, y1, yaw1) == (0.0, 2.0, 0.0)


def test_pose_is_yielded_when_its_block_closes_not_at_the_next_header():
    lines = TEXT.splitlines()
    close = lines.index("pose {", lines.index('  name: "reely_new1"') - 1)  # the car's block
    end = close + lines[close:].index("}")  # its closing brace (the first top-level one after it)
    consumed = []

    def feed():
        for i, line in enumerate(lines):
            consumed.append(i)
            yield line

    first = next(gps.stream_model_poses(feed(), "reely_new1"))
    assert first[0] == pytest.approx(12.005)
    assert consumed[-1] == end  # nothing after the car's block had to be read
    assert consumed[-1] < lines.index("header {", 1)


def test_reader_thread_calls_the_sink():
    got = []
    reader = gps.GzPoseStream("unused", "reely_new1", lambda: 99.0, lambda *a: got.append(a))
    reader.start(io.StringIO(TEXT))
    reader.thread.join(timeout=2)
    assert [g[:3] for g in got] == [(pytest.approx(12.005), 99.0, 1.25), (pytest.approx(12.015), 99.0, 0.0)]
