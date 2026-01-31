from pathlib import Path

from cfdg.io.writer import OutputWriter


def test_output_writer(tmp_path: Path):
    writer = OutputWriter(str(tmp_path))
    writer.write_scene(
        "scene-x",
        trajectory=[{"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.0}],
        labels={"collision": False, "off_road": False, "is_recovered": False, "min_ttc": 1.0},
        meta={"scene_token": "scene-x"},
    )
    assert (tmp_path / "scenes" / "scene-x" / "labels.json").exists()
