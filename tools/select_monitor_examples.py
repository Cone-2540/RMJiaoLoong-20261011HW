import csv
import hashlib
import json
import sys
from pathlib import Path

from monitor_replay import convert_record


SELECTED = [5, 9, 15, 22, 30, 36]


def export_examples(source_dir, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    signature = None
    for order, index in enumerate(SELECTED, 1):
        input_path = source_dir / f"circle_integral_{index}.json"
        record = json.loads(input_path.read_text(encoding="utf-8"))
        state = record["state"]
        parameters = tuple(state[name] for name in (
            "position_pid_params.kp", "position_pid_params.ki", "position_pid_params.kd",
            "speed_pid_params.kp", "speed_pid_params.ki", "speed_pid_params.kd",
            "ff_scale", "current_limit_a", "speed_limit_deg_s",
        ))
        if signature is None:
            signature = parameters
        if parameters != signature:
            raise ValueError("所选记录的参数不一致")
        if not (state["trace_state"] == 2 and state["trace_count"] == 500
                and state["stop_flag"] == 0 and state["settling_time_ms"] <= 100
                and all(point["y"] == 1 for point in record["data"]["feedback_online"])):
            raise ValueError(f"记录 {index} 不满足验收条件")
        reference = record["data"]["target_angle_deg"]
        target = state["target_angle_deg"]
        edge = next(point["x"] for point in reference if point["y"] == target)
        angle = record["data"]["actual_angle_deg"]
        hold = [point["y"] for point in angle if 100 <= point["x"] - edge <= 500]
        hold_error = max(abs(value - target) for value in hold)
        if hold_error > .1:
            raise ValueError(f"记录 {index} 的到位误差超出范围")
        start_deg, target_deg = (index - 1) * 10, index * 10
        settling = int(state["settling_time_ms"])
        prefix = f"Step2_Six_{order:02d}_{start_deg:03d}to{target_deg:03d}_{settling}ms"
        angle_path = output_dir / (prefix + "_angles.stcm")
        error_path = output_dir / (prefix + "_error.stcm")
        convert_record(input_path, angle_path, ["actual_angle_deg", "target_angle_deg"])
        convert_record(input_path, error_path, ["angle_error_deg"])
        angle_origin = min(angle[0]["x"], reference[0]["x"])
        error_origin = record["data"]["angle_error_deg"][0]["x"]
        angle_edge_s = (edge - angle_origin) / 1000
        error_edge_s = (edge - error_origin) / 1000
        rows.append({
            "order": order, "source": input_path.name,
            "physical_start_deg": start_deg, "physical_target_deg": target_deg,
            "actual_start_deg": reference[0]["y"], "actual_target_deg": target,
            "settling_ms": settling, "hold_pp_100_500ms_deg": max(hold)-min(hold),
            "hold_error_max_100_500ms_deg": hold_error,
            "angle_samples": len(angle),
            "angle_edge_s": angle_edge_s, "angle_deadline_s": angle_edge_s + .1,
            "angle_view_start_s": angle_edge_s - .05, "angle_view_end_s": angle_edge_s + .35,
            "error_edge_s": error_edge_s, "error_deadline_s": error_edge_s + .1,
            "error_view_start_s": error_edge_s + .08, "error_view_end_s": error_edge_s + .5,
            "angle_file": angle_path.name, "error_file": error_path.name,
            "source_sha256": hashlib.sha256(input_path.read_bytes()).hexdigest(),
        })
        verify_export(record, angle_path, ["actual_angle_deg", "target_angle_deg"])
        verify_export(record, error_path, ["angle_error_deg"])
    with (output_dir / "six_examples.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return rows


def verify_export(record, path, names):
    messages = [json.loads(line.rstrip(",")) for line in path.read_text(encoding="utf-8").splitlines()]
    for name in names:
        restored = [point for message in messages if message["variablename"] == name
                    for point in message["variabledata"]]
        if restored != record["data"][name]:
            raise ValueError(f"导出记录发生改变：{path.name} / {name}")


if __name__ == "__main__":
    repository = Path(__file__).resolve().parents[1]
    destination = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "log"
    results = export_examples(repository / "tuning/20261013", destination)
    print(json.dumps(results, ensure_ascii=False, indent=2))
