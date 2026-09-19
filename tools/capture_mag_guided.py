#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GY-85 Guided Magnetometer Capture
==================================

Mục đích:
- Tự chọn cổng COM của ESP32.
- Tự gửi lệnh MAG_STREAM 1.
- Hướng dẫn người dùng xoay thiết bị theo từng tư thế 3D.
- Chỉ lưu dữ liệu MAGCSV hợp lệ.
- Tự tạo tools/mag_log.txt.
- Kết thúc bằng MAG_STREAM 0 và in lệnh calibration tiếp theo.

Yêu cầu:
    pip install pyserial

Chạy:
    python capture_mag_guided.py

Có thể chỉ định COM:
    python capture_mag_guided.py --port COM5

Có thể thay thời gian mỗi bước:
    python capture_mag_guided.py --seconds 8

Lưu ý:
- PHẢI đóng Arduino Serial Monitor trước khi chạy, nếu không cổng COM có thể bị chiếm.
- Thiết bị nên được lắp hoàn chỉnh đúng cấu hình sử dụng thật
  (ESP32 + GY-85 + pin + dây + vỏ) trước khi thu dữ liệu.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("\n[LOI] Chua cai pyserial.")
    print("Chay lenh sau trong PowerShell:")
    print("    pip install pyserial")
    print("\nSau do chay lai:")
    print("    python capture_mag_guided.py")
    raise SystemExit(2)


BAUD_DEFAULT = 115200
STEP_SECONDS_DEFAULT = 7
MIN_RECOMMENDED_SAMPLES = 1000


STEPS = [
    (
        "MAT NGUA - XOAY NGANG 360 DO",
        [
            "Dat bo mach gan nam ngang, mat linh kien huong LEN.",
            "Giu do nghieng tuong doi on dinh.",
            "Xoay TOAN BO thiet bi mot vong 360 do quanh phuong thang dung.",
            "Xoay cham va deu, khong giat.",
        ],
    ),
    (
        "MAT UP - XOAY NGANG 360 DO",
        [
            "Lat bo mach de mat linh kien huong XUONG.",
            "Giu bo mach gan nam ngang.",
            "Xoay TOAN BO thiet bi mot vong 360 do quanh phuong thang dung.",
        ],
    ),
    (
        "CANH TRAI XUONG",
        [
            "Dung thiet bi tren CANH TRAI, de mat bo mach gan thang dung.",
            "Giu tu the do va xoay toan bo thiet bi 360 do quanh phuong thang dung.",
            "Neu kho giu dung canh, chi can nghieng gan 90 do va xoay deu.",
        ],
    ),
    (
        "CANH PHAI XUONG",
        [
            "Dung thiet bi tren CANH PHAI.",
            "Giu mat bo mach gan thang dung.",
            "Xoay toan bo thiet bi 360 do quanh phuong thang dung.",
        ],
    ),
    (
        "DAU MUI TEN HUONG LEN",
        [
            "Dung bo mach theo chieu doc.",
            "Cho dau ma ban chon lam MUI TEN vat ly huong LEN.",
            "Trong tu the nay, xoay/chao cham de cam bien di qua nhieu huong.",
        ],
    ),
    (
        "DAU MUI TEN HUONG XUONG",
        [
            "Dao nguoc tu the buoc truoc.",
            "Cho dau MUI TEN vat ly huong XUONG.",
            "Xoay/chao cham de phu tiep cac huong 3D con thieu.",
        ],
    ),
    (
        "HINH SO 8 TRONG KHONG GIAN",
        [
            "Cam thiet bi tu do trong tay.",
            "Ve HINH SO 8 lon trong khong gian.",
            "Dong thoi nghieng va lat thiet bi qua nhieu mat.",
            "Muc tieu la thay doi ca X, Y va Z cua tu truong.",
        ],
    ),
    (
        "QUAY TU DO QUANH CA 3 TRUC",
        [
            "Day la buoc bo sung do phu 3D.",
            "Quay cham quanh truc X, sau do Y, sau do Z.",
            "Uu tien cac tu the ban cam thay chua xoay du o cac buoc truoc.",
            "Khong dat sat dien thoai, loa, nam cham, dong co hoac vat thep lon.",
        ],
    ),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Huong dan thu du lieu HMC5883L va tao mag_log.txt tu dong."
    )
    parser.add_argument(
        "--port",
        help="Cong COM, vi du COM5. Neu bo trong, script se tu liet ke de chon.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=BAUD_DEFAULT,
        help=f"Baud rate, mac dinh {BAUD_DEFAULT}.",
    )
    parser.add_argument(
        "--seconds",
        type=int,
        default=STEP_SECONDS_DEFAULT,
        help=f"So giay ghi moi buoc, mac dinh {STEP_SECONDS_DEFAULT}.",
    )
    parser.add_argument(
        "--output",
        default="mag_log.txt",
        help="Ten file output. Mac dinh luu canh script trong thu muc tools.",
    )
    return parser.parse_args()


def resolve_output_path(output_arg: str) -> Path:
    p = Path(output_arg)
    if p.is_absolute():
        return p
    # Luôn lưu cạnh script, không phụ thuộc PowerShell đang đứng ở thư mục nào.
    return Path(__file__).resolve().parent / p


def choose_port(explicit_port: str | None) -> str:
    if explicit_port:
        return explicit_port

    ports = sorted(list_ports.comports(), key=lambda p: p.device)

    if not ports:
        print("\n[LOI] Khong tim thay cong Serial/COM nao.")
        print("Kiem tra:")
        print("  1. ESP32 da cam USB chua.")
        print("  2. Driver USB-UART da cai chua.")
        print("  3. Thu rut/cam lai cap USB.")
        raise SystemExit(2)

    print("\n=== CAC CONG SERIAL TIM THAY ===")
    for i, p in enumerate(ports, start=1):
        desc = p.description or "Khong co mo ta"
        hwid = p.hwid or ""
        print(f"  [{i}] {p.device:8s} | {desc} | {hwid}")

    if len(ports) == 1:
        selected = ports[0].device
        print(f"\nChi co 1 cong -> tu chon {selected}")
        return selected

    while True:
        raw = input("\nNhap SO thu tu cong cua ESP32: ").strip()
        try:
            idx = int(raw)
        except ValueError:
            print("Vui long nhap mot so.")
            continue

        if 1 <= idx <= len(ports):
            return ports[idx - 1].device

        print(f"Chi chon tu 1 den {len(ports)}.")


def open_serial(port: str, baud: int) -> serial.Serial:
    print(f"\nDang mo {port} @ {baud} baud ...")

    try:
        ser = serial.Serial(port, baud, timeout=0.12)
    except serial.SerialException as exc:
        print(f"\n[LOI] Khong mo duoc {port}:")
        print(f"  {exc}")
        print("\nThu cac buoc sau:")
        print("  1. DONG Arduino Serial Monitor.")
        print("  2. Dong PlatformIO monitor/miniterm neu dang mo.")
        print("  3. Kiem tra dung cong COM.")
        print("  4. Chay lai script.")
        raise SystemExit(2)

    # ESP32 thường reset khi serial được mở do DTR/RTS.
    print("Da mo cong. Cho ESP32 khoi dong 2.5 giay ...")
    time.sleep(2.5)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    return ser


def send_command(ser: serial.Serial, command: str) -> None:
    payload = (command.strip() + "\n").encode("utf-8")
    ser.write(payload)
    ser.flush()


def wait_for_stream_ack(ser: serial.Serial, timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="ignore").strip()
        if "MAG_STREAM,1" in line:
            return True

    return False


def parse_magcsv(line: str) -> tuple[float, float, float] | None:
    if not line.startswith("MAGCSV,"):
        return None

    parts = line.split(",")
    if len(parts) < 4:
        return None

    try:
        x = float(parts[1])
        y = float(parts[2])
        z = float(parts[3])
    except ValueError:
        return None

    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return None

    return x, y, z


class CaptureStats:
    def __init__(self) -> None:
        self.count = 0
        self.min_v = [float("inf")] * 3
        self.max_v = [float("-inf")] * 3
        self.last_sample: tuple[float, float, float] | None = None

    def add(self, sample: tuple[float, float, float]) -> bool:
        # Bỏ mẫu trùng liên tiếp. HMC5883L chạy chậm hơn loop ESP32,
        # nên một measurement có thể bị đọc/in nhiều lần.
        if sample == self.last_sample:
            return False

        self.last_sample = sample
        self.count += 1

        for i, v in enumerate(sample):
            self.min_v[i] = min(self.min_v[i], v)
            self.max_v[i] = max(self.max_v[i], v)

        return True

    def ranges(self) -> tuple[float, float, float]:
        if self.count == 0:
            return 0.0, 0.0, 0.0

        return tuple(
            self.max_v[i] - self.min_v[i]
            for i in range(3)
        )


def print_header(port: str, output_path: Path, seconds: int) -> None:
    print("\n" + "=" * 72)
    print(" GY-85 / HMC5883L - THU DU LIEU CALIBRATION CO HUONG DAN")
    print("=" * 72)
    print(f"Cong Serial       : {port}")
    print(f"File se duoc tao  : {output_path}")
    print(f"Thoi gian moi buoc: {seconds} giay")
    print(f"So buoc           : {len(STEPS)}")
    print("\nQUAN TRONG:")
    print("  - Calibrate voi TOAN BO thiet bi da lap hoan chinh.")
    print("  - Tranh xa dien thoai, loa, nam cham, motor va vat thep lon.")
    print("  - Xoay CHAM, DEU, phu nhieu huong 3D.")
    print("  - Script chi luu dong MAGCSV hop le.")
    print("  - Co the Ctrl+C bat ky luc nao; file da thu van duoc giu lai.")
    print("=" * 72)


def print_step(index: int, title: str, instructions: list[str]) -> None:
    print("\n")
    print("#" * 72)
    print(f" BUOC {index}/{len(STEPS)}: {title}")
    print("#" * 72)

    for n, item in enumerate(instructions, start=1):
        print(f"  {n}. {item}")


def capture_for_duration(
    ser: serial.Serial,
    output_file,
    stats: CaptureStats,
    duration_s: int,
) -> int:
    # Bỏ dữ liệu phát sinh khi người dùng đang đọc hướng dẫn.
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    start_count = stats.count
    start = time.monotonic()
    end = start + duration_s
    last_status_second = -1

    while True:
        now = time.monotonic()
        if now >= end:
            break

        raw = ser.readline()

        if raw:
            line = raw.decode("utf-8", errors="ignore").strip()
            sample = parse_magcsv(line)

            if sample is not None and stats.add(sample):
                output_file.write(
                    f"MAGCSV,{sample[0]:.3f},{sample[1]:.3f},{sample[2]:.3f}\n"
                )

        elapsed = now - start
        sec = int(elapsed)

        if sec != last_status_second:
            last_status_second = sec
            remaining = max(0, int(math.ceil(end - now)))
            collected_this_step = stats.count - start_count
            print(
                f"\r  Dang ghi... con {remaining:2d}s"
                f" | mau buoc nay: {collected_this_step:4d}"
                f" | tong: {stats.count:4d}",
                end="",
                flush=True,
            )

    output_file.flush()
    print()

    return stats.count - start_count


def print_summary(stats: CaptureStats, output_path: Path) -> None:
    rx, ry, rz = stats.ranges()

    print("\n" + "=" * 72)
    print(" HOAN TAT THU DU LIEU")
    print("=" * 72)
    print(f"File       : {output_path}")
    print(f"So mau     : {stats.count}")
    print(f"Range X    : {rx:.1f}")
    print(f"Range Y    : {ry:.1f}")
    print(f"Range Z    : {rz:.1f}")

    if stats.count < MIN_RECOMMENDED_SAMPLES:
        print(
            f"\n[CANH BAO] Moi co {stats.count} mau."
            f" Nen co it nhat {MIN_RECOMMENDED_SAMPLES} mau."
        )
        print("Hay chay lai va xoay cham hon / tang --seconds.")
    else:
        print("\n[OK] So luong mau dat muc khuyen nghi.")

    ranges = [rx, ry, rz]
    if min(ranges) > 0 and max(ranges) / min(ranges) > 4.0:
        print(
            "[CANH BAO] Do phu mot truc kem hon cac truc khac rat nhieu."
        )
        print(
            "Nen chay lai va chu y lat/nghieng quanh truc co do phu kem."
        )

    print("\nBUOC TIEP THEO:")
    print("  Tu thu muc tools chay:")
    print(f'    python calibrate_mag.py "{output_path.name}"')
    print("\nSau do copy NGUYEN dong:")
    print("    MAG_MATRIX ...")
    print("vao Serial Monitor cua ESP32.")
    print("=" * 72)


def main() -> int:
    args = parse_args()

    if args.seconds < 3:
        print("[LOI] --seconds nen >= 3.")
        return 2

    output_path = resolve_output_path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("\nTruoc khi bat dau:")
    print("  1. Cam ESP32 vao USB.")
    print("  2. DONG Arduino Serial Monitor / PlatformIO Monitor.")
    print("  3. Dam bao firmware GY85_Wearable_Compass da nap.")
    input("\nNhan ENTER khi da san sang...")

    port = choose_port(args.port)
    ser = open_serial(port, args.baud)

    print_header(port, output_path, args.seconds)

    print("\nDang gui lenh MAG_STREAM 1 ...")
    send_command(ser, "MAG_STREAM 1")

    if wait_for_stream_ack(ser):
        print("[OK] ESP32 da bat MAG_STREAM.")
    else:
        print(
            "[CANH BAO] Khong thay ACK MAG_STREAM,1 trong 2 giay."
        )
        print(
            "Script van tiep tuc. Neu moi buoc thu duoc 0 mau, hay kiem tra firmware/COM."
        )

    stats = CaptureStats()

    try:
        with output_path.open("w", encoding="utf-8", newline="\n") as f:
            for i, (title, instructions) in enumerate(STEPS, start=1):
                print_step(i, title, instructions)

                answer = input(
                    "\nDat dung tu the tren, sau do nhan ENTER de bat dau "
                    "(go q roi ENTER de dung): "
                ).strip().lower()

                if answer in {"q", "quit", "exit"}:
                    print("\nDung som theo yeu cau.")
                    break

                got = capture_for_duration(
                    ser=ser,
                    output_file=f,
                    stats=stats,
                    duration_s=args.seconds,
                )

                if got == 0:
                    print(
                        "[CANH BAO] Buoc nay khong thu duoc mau MAGCSV nao."
                    )
                    print(
                        "Kiem tra ESP32 co dang chay dung firmware va MAG_STREAM hay khong."
                    )
                else:
                    print(f"  -> Da thu {got} mau hop le o buoc nay.")

    except KeyboardInterrupt:
        print("\n\nNhan Ctrl+C -> dung thu du lieu va luu file hien tai.")

    finally:
        try:
            send_command(ser, "MAG_STREAM 0")
            time.sleep(0.15)
        except Exception:
            pass

        try:
            ser.close()
        except Exception:
            pass

    print_summary(stats, output_path)

    if stats.count == 0:
        print(
            "\n[LOI] File da duoc tao nhung khong co mau."
            " Khong chay calibrate_mag.py voi file nay."
        )
        return 3

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
