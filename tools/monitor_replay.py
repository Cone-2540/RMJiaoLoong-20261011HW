import argparse
import json
import math
from pathlib import Path


def convert_record(input_path, output_path, names):
    record = json.loads(input_path.read_text(encoding="utf-8"))
    messages = []
    for name in names:
        series = record["data"].get(name, [])
        if not series:
            raise ValueError(f"原始记录没有变量：{name}")
        for point in series:
            if not all(math.isfinite(point[key]) for key in ("x", "y")):
                raise ValueError(f"变量含非有限数据：{name}")
            messages.append({
                "groupname": "myVariables",
                "variablename": name,
                "variabledata": [{"x": point["x"], "y": point["y"]}],
            })
    messages.sort(key=lambda message: message["variabledata"][0]["x"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as stream:
        for message in messages:
            stream.write(json.dumps(message, ensure_ascii=False) + ",\n")
    print(f"已导出 {len(messages)} 个原始采样点：{output_path}")
    print(f"同次固件记录的收敛时间：{record['state'].get('settling_time_ms')} ms")


def main():
    parser = argparse.ArgumentParser(description="将调参 JSON 转为 CubeMonitor 可导入的 STCM 日志")
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--variables", nargs="+", default=["actual_angle_deg", "target_angle_deg"])
    args = parser.parse_args()
    convert_record(args.input, args.output, args.variables)


if __name__ == "__main__":
    main()
