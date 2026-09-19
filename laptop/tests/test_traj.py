import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import TRAJ_CHUNK_PTS, build_traj_packets, build_traj_start
from send_trajectory import load_trajectory


def test_traj_packets_match_firmware_layout() -> None:
    # Offsets pinned to handle_traj_data()/handle_traj_start() in main/wifi_task.c.
    pts = [(i * 0.01, -i * 0.02, 0.0) for i in range(170)]
    packets = build_traj_packets(7, pts)
    assert [p[7] for p in packets] == [80, 80, 10]
    assert max(len(p) for p in packets) <= 1024          # WIFI_CMD_BUF_SIZE

    got = []
    for p in packets:
        pkt, cmd, tid, total, offset, n = struct.unpack_from("<BBBHHB", p)
        assert (pkt, cmd, tid, total) == (0x02, 0x08, 7, 170)
        assert offset == len(got) and len(p) == 8 + n * 12
        got += [struct.unpack_from("<fff", p, 8 + 12 * i) for i in range(n)]
    assert got == [struct.unpack("<fff", struct.pack("<fff", *q)) for q in pts]
    assert TRAJ_CHUNK_PTS == 80

    assert struct.unpack("<BBBH", build_traj_start(7, 50)) == (0x02, 0x09, 7, 50)


def test_load_trajectory_resamples_relative_to_first_point(tmp_path) -> None:
    csv = tmp_path / "traj.csv"
    csv.write_text("t,x,y,z\n0,1,2,0\n0.1,1.1,2,0\n0.2,1.2,2,0\n")
    pts = load_trajectory(str(csv), 0.05)
    assert pts.shape == (5, 3)
    assert abs(pts[1][0] - 0.05) < 1e-9 and abs(pts[-1][0] - 0.2) < 1e-9
    assert not pts[0].any()
