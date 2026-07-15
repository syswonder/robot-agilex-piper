#!/usr/bin/env python3
"""Apply or revert the CAN-drain-latest workaround for piper_sdk 0.2.19."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

RAW_CALLBACK_COMMENT = "#\u56de\u8c03\u51fd\u6570\u5904\u7406\u63a5\u6536\u7684\u539f\u59cb\u6570\u636e"

OLD_SNIPPET = """    def ReadCanMessage(self):
        if self.is_can_bus_ok():
            self.rx_message = self.bus.recv()
            if self.rx_message and self.callback_function:
                self.callback_function(self.rx_message) {raw_comment}
        else:
            print(\"CAN bus is not OK, skipping message read\")
""".format(raw_comment=RAW_CALLBACK_COMMENT)

NEW_SNIPPET = """    def ReadCanMessage(self):
        if self.is_can_bus_ok():
            messages = []
            self.rx_message = self.bus.recv()
            if self.rx_message:
                messages.append(self.rx_message)
                while True:
                    queued_message = self.bus.recv(timeout=0.0)
                    if queued_message is None:
                        break
                    messages.append(queued_message)
                self.rx_message = messages[-1]
            if messages and self.callback_function:
                latest_messages = {}
                message_ids = []
                for message in messages:
                    arbitration_id = message.arbitration_id
                    if arbitration_id not in latest_messages:
                        message_ids.append(arbitration_id)
                    latest_messages[arbitration_id] = message
                for arbitration_id in message_ids:
                    self.callback_function(latest_messages[arbitration_id])  # callback handles the newest raw frame per arbitration_id
        else:
            print(\"CAN bus is not OK, skipping message read\")
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=Path, help="Path to can_encapsulation.py. If omitted, auto-detect from piper_sdk.")
    parser.add_argument("--revert", action="store_true", help="Restore the original file from the .bak backup.")
    return parser.parse_args()


def detect_target(explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit.expanduser().resolve()

    import piper_sdk

    package_init = Path(piper_sdk.__file__).resolve()
    target = package_init.parent / "hardware_port" / "can_encapsulation.py"
    if not target.exists():
        raise FileNotFoundError(f"cannot find can_encapsulation.py next to {package_init}")
    return target


def backup_path_for(target: Path) -> Path:
    return target.with_suffix(target.suffix + ".bak")


def ensure_backup(target: Path) -> Path:
    backup = backup_path_for(target)
    if not backup.exists():
        shutil.copy2(target, backup)
    return backup


def apply_patch(target: Path) -> None:
    source = target.read_text(encoding="utf-8")
    if NEW_SNIPPET in source:
        print(f"already patched: {target}")
        return
    if OLD_SNIPPET not in source:
        raise RuntimeError(
            "expected original ReadCanMessage() body not found; unsupported piper_sdk version or local modifications detected"
        )
    ensure_backup(target)
    target.write_text(source.replace(OLD_SNIPPET, NEW_SNIPPET), encoding="utf-8")
    print(f"patched: {target}")
    print(f"backup:  {backup_path_for(target)}")


def revert_patch(target: Path) -> None:
    backup = backup_path_for(target)
    if not backup.exists():
        raise FileNotFoundError(f"backup not found: {backup}")
    shutil.copy2(backup, target)
    print(f"restored: {target}")
    print(f"source:   {backup}")


def main() -> int:
    args = parse_args()
    target = detect_target(args.target)
    if args.revert:
        revert_patch(target)
    else:
        apply_patch(target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
