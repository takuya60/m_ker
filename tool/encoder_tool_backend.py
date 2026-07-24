"""Non-UI helpers for the KER encoder flasher."""

import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

import serial.tools.list_ports


SETTINGS_FILE = Path.home() / ".ker_encoder_tool.json"
ANSI_ESCAPE_RE = re.compile(r"\x1b(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
HEX_ID_RE = re.compile(r"firmware_ID(\d{2})\.hex$")
M5_FIRMWARE_RE = re.compile(r"firmware_(usb|serial|wifi)\.bin$")
STANDARD_FUSES = {
    0: 0x00,  # WDTCFG
    1: 0x00,  # BODCFG
    2: 0x02,  # OSCCFG: 20 MHz internal oscillator
    4: 0xC4,  # SYSCFG0: keep UPDI enabled
    5: 0x06,  # SYSCFG1
    6: 0x00,  # APPEND
    7: 0x00,  # BOOTEND
}


def get_ports():
    return [
        (port.device, f"{port.device} {port.description}")
        for port in serial.tools.list_ports.comports()
    ]


def make_request_packet(device_id):
    header = 0x80 | ((device_id & 0x1F) << 2) | 1
    return bytes([header, 0x00, 0x00, 0x00])


def decode_packet(packet):
    if len(packet) != 4 or not (packet[0] & 0x80):
        return None
    header7 = packet[0] & 0x7F
    response_id = header7 >> 2
    response_command = header7 & 0x03
    data21 = packet[1] | (packet[2] << 7) | (packet[3] << 14)
    return response_id, response_command, data21


def is_platformio_project(path):
    if not path:
        return False
    project = Path(path).expanduser()
    return (
        project.is_dir()
        and (project / "platformio.ini").is_file()
        and (project / "src" / "main.cpp").is_file()
    )


def load_saved_project():
    try:
        data = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
        path = data.get("platformio_project", "")
        return str(Path(path)) if is_platformio_project(path) else ""
    except (OSError, ValueError, TypeError):
        return ""


def save_project(path):
    try:
        SETTINGS_FILE.write_text(
            json.dumps({"platformio_project": str(Path(path))}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except OSError:
        pass


def encoder_project_candidates():
    tool_directory = Path(__file__).resolve().parent
    return (
        tool_directory / "firmware" / "encoder",
        tool_directory.parent / "firmware" / "encoder",
    )


def find_default_project():
    saved = load_saved_project()
    if saved:
        return saved
    for project in encoder_project_candidates():
        if is_platformio_project(project):
            return str(project)
    return ""


def m5_project_candidates():
    tool_directory = Path(__file__).resolve().parent
    return (
        tool_directory / "firmware" / "M5",
        tool_directory.parent / "firmware" / "M5",
    )


def find_default_m5_project():
    for project in m5_project_candidates():
        if is_platformio_project(project):
            return str(project)
    return ""


def find_bundled_hexes():
    tool_directory = Path(__file__).resolve().parent
    firmware_directories = []
    if getattr(sys, "frozen", False):
        firmware_directories.append(
            Path(sys.executable).resolve().parent / "hex" / "encoder")
    else:
        firmware_directories.append(
            tool_directory / "encoder_tool" / "hex" / "encoder")
    firmware_directories.extend(
        project / "firmware" for project in encoder_project_candidates())

    for firmware_directory in firmware_directories:
        if not firmware_directory.is_dir():
            continue
        result = {}
        for path in firmware_directory.glob("firmware_ID*.hex"):
            match = HEX_ID_RE.match(path.name)
            if match:
                result[int(match.group(1))] = path
        if result:
            return result
    return {}


def find_bundled_m5_firmware():
    tool_directory = Path(__file__).resolve().parent
    if getattr(sys, "frozen", False):
        firmware_directory = Path(sys.executable).resolve().parent / "hex" / "m5"
    else:
        firmware_directory = tool_directory / "encoder_tool" / "hex" / "m5"
    if not firmware_directory.is_dir():
        return {}

    result = {}
    for path in firmware_directory.glob("firmware_*.bin"):
        match = M5_FIRMWARE_RE.match(path.name)
        if match:
            result[match.group(1)] = path
    return result


def find_executable(names, candidates=()):
    for name in names:
        found = shutil.which(name)
        if found:
            return found
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return ""


def bundled_flash_cli_path():
    if not getattr(sys, "frozen", False):
        return ""
    executable = Path(sys.executable).resolve().parent / "ker_flash_cli.exe"
    return str(executable) if executable.is_file() else ""


def find_platformio():
    candidates = []
    if os.name == "nt":
        user_profile = Path(os.environ.get("USERPROFILE", Path.home()))
        candidates.append(user_profile / ".platformio" / "penv" / "Scripts" / "pio.exe")
    else:
        candidates.append(Path.home() / ".platformio" / "penv" / "bin" / "pio")
    return find_executable(("pio", "platformio"), candidates)


def find_pymcuprog():
    bundled_cli = bundled_flash_cli_path()
    if bundled_cli:
        return bundled_cli
    names = ("pymcuprog.exe", "pymcuprog") if os.name == "nt" else ("pymcuprog",)
    return find_executable(names)


def find_esptool():
    bundled_cli = bundled_flash_cli_path()
    if bundled_cli:
        return bundled_cli
    names = ("esptool.exe", "esptool.py", "esptool") if os.name == "nt" else (
        "esptool.py", "esptool")
    candidates = []
    platformio_package = Path.home() / ".platformio" / "packages" / "tool-esptoolpy"
    candidates.append(platformio_package / "esptool.py")
    return find_executable(names, candidates)


def make_module_command(executable_path, module_name, arguments):
    if Path(executable_path).name.lower() == "ker_flash_cli.exe":
        return [executable_path, module_name, *arguments]
    module_entrypoint = {
        "pymcuprog": "pymcuprog.pymcuprog",
    }.get(module_name, module_name)
    if not getattr(sys, "frozen", False) and importlib.util.find_spec(module_entrypoint):
        return [sys.executable, "-u", "-m", module_entrypoint, *arguments]

    executable_directory = Path(executable_path).resolve().parent
    python_names = ("python.exe", "python3.exe") if os.name == "nt" else ("python3", "python")
    for python_name in python_names:
        python_path = executable_directory / python_name
        if python_path.is_file():
            return [str(python_path), "-u", "-m", module_name, *arguments]
    return [executable_path, *arguments]


def format_command(command):
    return subprocess.list2cmdline(command) if os.name == "nt" else shlex.join(command)


def strip_terminal_codes(text):
    return ANSI_ESCAPE_RE.sub("", text).replace("\x00", "")


def stream_process_output(process, emit_line):
    line_buffer = bytearray()
    previous_was_carriage_return = False

    def flush_line():
        if line_buffer:
            emit_line(line_buffer.decode("utf-8", errors="replace"))
            line_buffer.clear()

    while True:
        chunk = process.stdout.read(1)
        if not chunk:
            if process.poll() is not None:
                break
            continue
        if chunk == b"\n":
            if not previous_was_carriage_return:
                flush_line()
            previous_was_carriage_return = False
        elif chunk == b"\r":
            flush_line()
            previous_was_carriage_return = True
        elif chunk == b"\b":
            if line_buffer:
                line_buffer.pop()
            previous_was_carriage_return = False
        else:
            line_buffer.extend(chunk)
            previous_was_carriage_return = False
    flush_line()


def run_command(command, *, cwd=None, env=None, emit_line):
    startup_info = None
    if os.name == "nt":
        startup_info = subprocess.STARTUPINFO()
        startup_info.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    emit_line(f"[COMMAND] {format_command(command)}")
    process = subprocess.Popen(
        command,
        cwd=cwd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0,
        startupinfo=startup_info,
    )
    stream_process_output(process, emit_line)
    return process.wait()


def platformio_upload_command(pio_path, updi_port):
    return make_module_command(
        pio_path,
        "platformio",
        ["--no-ansi", "run", "-t", "upload", "--upload-port", updi_port],
    )


def platformio_m5_upload_command(pio_path, transport, upload_port):
    return make_module_command(
        pio_path,
        "platformio",
        [
            "--no-ansi", "run", "-e", transport, "-t", "upload",
            "--upload-port", upload_port,
        ],
    )


def pymcuprog_fuse_commands(pymcuprog_path, updi_port):
    commands = []
    for offset, value in STANDARD_FUSES.items():
        commands.append(make_module_command(
            pymcuprog_path,
            "pymcuprog",
            [
                "write", "-t", "uart", "-u", updi_port, "-d", "attiny1616",
                "-m", "fuses", "-o", str(offset), "-l", hex(value), "-v", "info",
            ],
        ))
    return commands


def pymcuprog_hex_command(pymcuprog_path, updi_port, hex_path):
    return make_module_command(
        pymcuprog_path,
        "pymcuprog",
        [
            "write", "-t", "uart", "-u", updi_port, "-d", "attiny1616",
            "-f", str(hex_path), "--verify", "-v", "info",
        ],
    )


def esptool_m5_bin_command(esptool_path, upload_port, firmware_path):
    arguments = [
        "--chip", "esp32s3",
        "--port", upload_port,
        "--baud", "921600",
        "--before", "default_reset",
        "--after", "hard_reset",
        "write_flash", "-z", "0x0", str(firmware_path),
    ]
    if Path(esptool_path).name.lower() == "ker_flash_cli.exe":
        return [esptool_path, "esptool", *arguments]
    if Path(esptool_path).suffix.lower() == ".py":
        return [sys.executable, "-u", esptool_path, *arguments]
    return [esptool_path, *arguments]
