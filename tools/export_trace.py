import argparse
import csv
import math
from pathlib import Path
import struct


SAMPLE = struct.Struct("<I6f")
HEADERS = ["time_ms", "target_deg", "actual_deg", "error_deg",
           "speed_deg_s", "current_a", "feedforward_a"]


def read_hex(path):
    memory = {}
    base = 0
    for line in path.read_text(encoding="ascii").splitlines():
        if not line.strip():
            continue
        if not line.startswith(":"):
            raise ValueError("输入不是 Intel HEX 文件")
        record = bytes.fromhex(line[1:])
        if len(record) != record[0] + 5 or sum(record) % 256:
            raise ValueError("HEX 记录长度或校验和错误")
        address = int.from_bytes(record[1:3], "big")
        kind = record[3]
        data = record[4:-1]
        if kind == 0:
            for offset, value in enumerate(data):
                memory[base + address + offset] = value
        elif kind == 1:
            break
        elif kind == 2:
            base = int.from_bytes(data, "big") << 4
        elif kind == 4:
            base = int.from_bytes(data, "big") << 16
        elif kind not in (3, 5):
            raise ValueError("不支持的 HEX 记录")
    if not memory:
        raise ValueError("HEX 中没有数据")
    start = min(memory)
    end = max(memory) + 1
    if end - start != len(memory):
        raise ValueError("内存不连续，请只导出 trace_samples 数组")
    return bytes(memory[address] for address in range(start, end))


def main():
    parser = argparse.ArgumentParser(description="将 MCU 阶跃记录转换为 CSV")
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--count", type=int, default=500)
    args = parser.parse_args()
    data = read_hex(args.input) if args.input.suffix.lower() == ".hex" else args.input.read_bytes()
    if not 1 <= args.count <= 500 or len(data) != args.count * SAMPLE.size:
        raise ValueError("长度不匹配：每样本28字节，默认500个样本")
    rows = list(SAMPLE.iter_unpack(data))
    for index, row in enumerate(rows):
        if not all(math.isfinite(value) for value in row[1:]):
            raise ValueError("记录含有非有限浮点数")
        if index and row[0] <= rows[index - 1][0]:
            raise ValueError("时间戳未递增，请核对导出地址或采集状态")
    with args.output.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(HEADERS)
        writer.writerows(rows)
    last_outside = max((index for index, row in enumerate(rows)
                        if abs(row[3]) > 0.1), default=-1)
    settling = rows[last_outside + 1][0] if last_outside + 1 < len(rows) else None
    print(f"已导出 {len(rows)} 个样本，观测时长 {rows[-1][0]} ms")
    print(f"观测窗口内收敛时间：{settling if settling is not None else '未收敛'} ms")
    print("参考标准：10°阶跃，100 ms内进入±0.1°并在后续记录中保持")


if __name__ == "__main__":
    main()
