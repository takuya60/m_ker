"""Desktop flasher and RS-485 tester for KER encoder boards."""

from pathlib import Path
import os
import queue
import sys
import threading
import time

import flet as ft
import serial

from encoder_tool_backend import (
    decode_packet,
    esptool_m5_bin_command,
    find_bundled_hexes,
    find_bundled_m5_firmware,
    find_default_project,
    find_default_m5_project,
    find_esptool,
    find_platformio,
    find_pymcuprog,
    get_ports,
    is_platformio_project,
    make_request_packet,
    platformio_m5_upload_command,
    platformio_upload_command,
    pymcuprog_fuse_commands,
    pymcuprog_hex_command,
    run_command,
    save_project,
    strip_terminal_codes,
)


APP_TITLE = "KER Encoder Flasher & Tester"
MAX_CONSOLE_LINES = 2000
DEFAULT_HEX_SOURCE = "bundled"


def choose_directory(title):
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    path = filedialog.askdirectory(title=title)
    root.destroy()
    return path


def choose_hex_file(title):
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    path = filedialog.askopenfilename(
        title=title,
        filetypes=[("Intel HEX", "*.hex"), ("All files", "*.*")],
    )
    root.destroy()
    return path


def choose_bin_file(title):
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    path = filedialog.askopenfilename(
        title=title,
        filetypes=[("ESP32 merged BIN", "*.bin"), ("All files", "*.*")],
    )
    root.destroy()
    return path


def main(page: ft.Page):
    page.title = APP_TITLE
    page.window.width = 860
    page.window.height = 800
    page.padding = 24

    operation_lock = threading.Lock()
    test_stop_event = threading.Event()
    bundled_hexes = find_bundled_hexes()
    bundled_m5_firmware = find_bundled_m5_firmware()

    def create_console():
        events = queue.Queue()
        stop_event = threading.Event()
        console_list = ft.ListView(expand=True, spacing=1, auto_scroll=True)
        active_line = ft.Text(
            "", size=13, color=ft.Colors.WHITE, font_family="Consolas", selectable=True)
        console_list.controls.append(active_line)
        container = ft.Container(
            content=console_list,
            expand=True,
            bgcolor="#1E1E1E",
            padding=10,
            border_radius=5,
        )

        def enqueue(message):
            clean = strip_terminal_codes(str(message))
            for line in clean.replace("\r\n", "\n").replace("\r", "\n").split("\n"):
                if line:
                    events.put(("line", line))

        def clear(_event=None):
            events.put(("clear", ""))

        def pump():
            nonlocal active_line
            while not stop_event.is_set():
                try:
                    event_type, text = events.get(timeout=0.1)
                except queue.Empty:
                    continue
                if event_type == "clear":
                    console_list.controls.clear()
                    active_line = ft.Text(
                        "", size=13, color=ft.Colors.WHITE,
                        font_family="Consolas", selectable=True)
                    console_list.controls.append(active_line)
                else:
                    active_line.value = text
                    active_line = ft.Text(
                        "", size=13, color=ft.Colors.WHITE,
                        font_family="Consolas", selectable=True)
                    console_list.controls.append(active_line)
                while len(console_list.controls) > MAX_CONSOLE_LINES:
                    console_list.controls.pop(0)
                try:
                    console_list.update()
                except Exception:
                    stop_event.set()

        threading.Thread(target=pump, daemon=True).start()
        return container, enqueue, clear

    source_console, source_log, clear_source_console = create_console()
    hex_console, hex_log, clear_hex_console = create_console()
    m5_source_console, m5_source_log, clear_m5_source_console = create_console()
    m5_bin_console, m5_bin_log, clear_m5_bin_console = create_console()

    ports = get_ports()

    def port_options():
        return [ft.dropdown.Option(key=key, text=text) for key, text in ports]

    updi_dropdown = ft.Dropdown(label="UPDI 烧录口", options=port_options(), width=240)
    rs485_dropdown = ft.Dropdown(label="RS485 测试口", options=port_options(), width=240)
    id_field = ft.TextField(label="目标 ID (1-16)", value="5", width=120)
    hex_updi_dropdown = ft.Dropdown(label="UPDI 烧录口", options=port_options(), width=260)
    hex_rs485_dropdown = ft.Dropdown(label="RS485 测试口", options=port_options(), width=260)
    m5_source_port_dropdown = ft.Dropdown(
        label="M5 烧录串口", options=port_options(), expand=True)
    m5_bin_port_dropdown = ft.Dropdown(
        label="M5 烧录串口", options=port_options(), expand=True)

    project_field = ft.TextField(
        label="PlatformIO 工程目录",
        value=find_default_project(),
        read_only=True,
        expand=True,
        tooltip="目录中必须包含 platformio.ini 和 src/main.cpp",
    )
    project_status = ft.Text(size=12)
    m5_project_field = ft.TextField(
        label="M5 PlatformIO 工程目录",
        value=find_default_m5_project(),
        read_only=True,
        expand=True,
        tooltip="目录中必须包含 platformio.ini 和 src/main.cpp",
    )
    m5_project_status = ft.Text(size=12)
    m5_transport_dropdown = ft.Dropdown(
        label="固件传输模式",
        options=[
            ft.dropdown.Option(key="usb", text="USB Vendor"),
            ft.dropdown.Option(key="wifi", text="WiFi"),
            ft.dropdown.Option(key="serial", text="Serial"),
        ],
        value="usb",
        width=220,
    )

    bundled_hex_dropdown = ft.Dropdown(
        label="内置 HEX",
        options=[
            ft.dropdown.Option(
                key=str(device_id),
                text=f"ID {device_id:02d}  firmware_ID{device_id:02d}.hex")
            for device_id in sorted(bundled_hexes)
        ],
        value="5" if 5 in bundled_hexes else None,
        menu_height=260,
        expand=True,
    )
    hex_source_group = ft.RadioGroup(
        value=DEFAULT_HEX_SOURCE,
        content=ft.Row(
            [
                ft.Radio(value="bundled", label="使用内置 HEX"),
                ft.Radio(value="custom", label="使用自选 HEX"),
            ],
            spacing=20,
        ),
        on_change=lambda event: update_hex_source_state(event),
    )
    custom_hex_field = ft.TextField(
        label="自选 HEX 文件", read_only=True, expand=True, disabled=True)
    hex_source_status = ft.Text(size=12)
    write_fuses_checkbox = ft.Checkbox(
        label="同时写入标准 fuse（推荐）",
        value=True,
        tooltip="写入 encoder/platformio.ini 中定义的 ATtiny1616 fuse",
    )
    m5_bundled_dropdown = ft.Dropdown(
        label="内置 M5 固件",
        options=[
            ft.dropdown.Option(key=transport, text=f"{transport.upper()}  {path.name}")
            for transport, path in sorted(bundled_m5_firmware.items())
        ],
        value="usb" if "usb" in bundled_m5_firmware else None,
        menu_height=220,
        expand=True,
    )
    m5_bin_source_group = ft.RadioGroup(
        value="bundled",
        content=ft.Row(
            [
                ft.Radio(value="bundled", label="使用内置 BIN"),
                ft.Radio(value="custom", label="使用自选 BIN"),
            ],
            spacing=20,
        ),
        on_change=lambda event: update_m5_bin_source_state(event),
    )
    m5_custom_bin_field = ft.TextField(
        label="自选合并 BIN 文件", read_only=True, expand=True, disabled=True)
    m5_bin_source_status = ft.Text(size=12)

    def update_project_status():
        valid = is_platformio_project(project_field.value)
        if valid:
            project_status.value = "工程有效：已找到 platformio.ini 和 src/main.cpp"
            project_status.color = ft.Colors.GREEN_700
        else:
            project_status.value = "请选择 encoder PlatformIO 工程目录"
            project_status.color = ft.Colors.RED_600
        return valid

    def update_m5_project_status():
        valid = is_platformio_project(m5_project_field.value)
        if valid:
            m5_project_status.value = "工程有效：已找到 platformio.ini 和 src/main.cpp"
            m5_project_status.color = ft.Colors.GREEN_700
        else:
            m5_project_status.value = "请选择 M5 PlatformIO 工程目录"
            m5_project_status.color = ft.Colors.RED_600
        return valid

    def parse_target_id():
        value = (id_field.value or "").strip()
        if not value.isdigit() or not 1 <= int(value) <= 16:
            source_log("[ERROR] 请输入有效的 ID (1-16)")
            return None
        return int(value)

    def set_busy(busy, testing=False):
        source_flash_button.disabled = busy
        hex_flash_button.disabled = busy
        source_test_button.disabled = busy
        hex_test_button.disabled = busy
        m5_source_flash_button.disabled = busy
        m5_bin_flash_button.disabled = busy
        source_stop_button.disabled = not testing
        hex_stop_button.disabled = not testing
        project_select_button.disabled = busy
        m5_project_select_button.disabled = busy
        hex_source_group.disabled = busy
        m5_bin_source_group.disabled = busy
        m5_transport_dropdown.disabled = busy
        if busy:
            bundled_hex_dropdown.disabled = True
            custom_hex_field.disabled = True
            hex_select_button.disabled = True
            m5_bundled_dropdown.disabled = True
            m5_custom_bin_field.disabled = True
            m5_bin_select_button.disabled = True
        else:
            update_hex_source_state()
            update_m5_bin_source_state()
        try:
            page.update()
        except Exception:
            pass

    def start_operation(worker, logger, testing=False):
        if not operation_lock.acquire(blocking=False):
            logger("[WARN] 已有操作正在运行")
            return
        set_busy(True, testing=testing)

        def run_worker():
            try:
                worker()
            except Exception as error:
                logger(f"[ERROR] 操作失败: {error}")
            finally:
                operation_lock.release()
                set_busy(False)

        threading.Thread(target=run_worker, daemon=True).start()

    def refresh_ports(_event):
        nonlocal ports
        ports = get_ports()
        devices = {key for key, _text in ports}
        for dropdown in (
                updi_dropdown, rs485_dropdown, hex_updi_dropdown, hex_rs485_dropdown,
                m5_source_port_dropdown, m5_bin_port_dropdown):
            previous = dropdown.value
            dropdown.options = [
                ft.dropdown.Option(key=key, text=text) for key, text in ports]
            dropdown.value = previous if previous in devices else None
        page.update()
        source_log(f"[INFO] 已刷新串口，共发现 {len(ports)} 个设备")
        hex_log(f"[INFO] 已刷新串口，共发现 {len(ports)} 个设备")
        m5_source_log(f"[INFO] 已刷新串口，共发现 {len(ports)} 个设备")
        m5_bin_log(f"[INFO] 已刷新串口，共发现 {len(ports)} 个设备")

    def select_project(_event):
        path = choose_directory("选择 encoder PlatformIO 工程目录")
        if not path:
            return
        if not is_platformio_project(path):
            source_log(f"[ERROR] 不是有效的 PlatformIO 工程: {path}")
            return
        project_field.value = str(Path(path))
        save_project(path)
        update_project_status()
        page.update()
        source_log(f"[INFO] PlatformIO 工程: {path}")

    def select_m5_project(_event):
        path = choose_directory("选择 M5 PlatformIO 工程目录")
        if not path:
            return
        if not is_platformio_project(path):
            m5_source_log(f"[ERROR] 不是有效的 PlatformIO 工程: {path}")
            return
        m5_project_field.value = str(Path(path))
        update_m5_project_status()
        page.update()
        m5_source_log(f"[INFO] M5 PlatformIO 工程: {path}")

    def select_hex(_event):
        path = choose_hex_file("选择要烧录的 Intel HEX 文件")
        if path:
            custom_hex_field.value = str(Path(path))
            update_hex_source_state()
            page.update()
            hex_log(f"[INFO] 自定义 HEX: {path}")

    def select_m5_bin(_event):
        path = choose_bin_file("选择 M5 ESP32-S3 合并 BIN 文件")
        if path:
            m5_custom_bin_field.value = str(Path(path))
            update_m5_bin_source_state()
            page.update()
            m5_bin_log(f"[INFO] 自选 M5 BIN: {path}")

    def flash_platformio(_event):
        target_id = parse_target_id()
        updi_port = updi_dropdown.value
        if target_id is None:
            return
        if not updi_port:
            source_log("[ERROR] 请选择 UPDI 端口")
            return
        if not update_project_status():
            page.update()
            source_log("[ERROR] 请先选择有效的 encoder PlatformIO 工程目录")
            return
        pio_path = find_platformio()
        if not pio_path:
            source_log("[ERROR] 未找到 PlatformIO CLI")
            return

        def worker():
            env = os.environ.copy()
            env["PLATFORMIO_BUILD_FLAGS"] = f"-DDEVICE_ID={target_id}"
            env["PYTHONUNBUFFERED"] = "1"
            env["NO_COLOR"] = "1"
            source_log(f"[INFO] 工程目录: {project_field.value}")
            source_log(f"[INFO] PlatformIO: {pio_path}")
            source_log(f"[INFO] 开始烧录 ID={target_id} 到端口 {updi_port}")
            return_code = run_command(
                platformio_upload_command(pio_path, updi_port),
                cwd=project_field.value,
                env=env,
                emit_line=source_log,
            )
            if return_code == 0:
                source_log(f"[SUCCESS] 烧录成功 (ID={target_id})。可进行 RS485 测试")
            else:
                source_log(f"[FAILED] 烧录失败，退出码 {return_code}")

        start_operation(worker, source_log)

    def update_hex_source_state(_event=None):
        use_custom_hex = hex_source_group.value == "custom"
        bundled_hex_dropdown.disabled = use_custom_hex
        custom_hex_field.disabled = not use_custom_hex
        hex_select_button.disabled = not use_custom_hex

        if use_custom_hex:
            custom_value = (custom_hex_field.value or "").strip()
            if custom_value:
                hex_source_status.value = f"当前烧录来源：自选 HEX | {custom_value}"
            else:
                hex_source_status.value = "当前烧录来源：自选 HEX | 尚未选择文件"
        else:
            selected_id = bundled_hex_dropdown.value
            hex_path = bundled_hexes.get(int(selected_id)) if selected_id else None
            filename = hex_path.name if hex_path else "尚未选择文件"
            hex_source_status.value = f"当前烧录来源：内置 HEX | {filename}"

        if _event is not None:
            page.update()

    def selected_hex_path():
        if hex_source_group.value == "custom":
            custom_path = Path((custom_hex_field.value or "").strip())
            return custom_path if custom_path.is_file() else None
        selected_id = bundled_hex_dropdown.value
        return bundled_hexes.get(int(selected_id)) if selected_id else None

    def flash_hex(_event):
        updi_port = hex_updi_dropdown.value
        hex_path = selected_hex_path()
        hex_source_name = "自选 HEX" if hex_source_group.value == "custom" else "内置 HEX"
        if not updi_port:
            hex_log("[ERROR] 请选择 UPDI 端口")
            return
        if hex_path is None:
            hex_log("[ERROR] 请选择内置 HEX 或自定义 HEX 文件")
            return
        pymcuprog_path = find_pymcuprog()
        if not pymcuprog_path:
            hex_log("[ERROR] 未找到 pymcuprog，无法直接烧录 HEX")
            return

        def worker():
            hex_log(f"[INFO] 烧录来源: {hex_source_name}")
            hex_log(f"[INFO] HEX 文件: {hex_path}")
            hex_log(f"[INFO] pymcuprog: {pymcuprog_path}")
            if write_fuses_checkbox.value:
                hex_log("[INFO] 正在写入标准 fuse")
                for command in pymcuprog_fuse_commands(pymcuprog_path, updi_port):
                    if run_command(command, emit_line=hex_log) != 0:
                        hex_log("[FAILED] fuse 写入失败，已停止 HEX 烧录")
                        return
            return_code = run_command(
                pymcuprog_hex_command(pymcuprog_path, updi_port, hex_path),
                emit_line=hex_log,
            )
            if return_code == 0:
                hex_log(f"[SUCCESS] HEX 烧录并校验成功: {hex_path.name}")
            else:
                hex_log(f"[FAILED] HEX 烧录失败，退出码 {return_code}")

        start_operation(worker, hex_log)

    def flash_m5_platformio(_event):
        upload_port = m5_source_port_dropdown.value
        transport = m5_transport_dropdown.value
        if not upload_port:
            m5_source_log("[ERROR] 请选择 M5 烧录串口")
            return
        if not transport:
            m5_source_log("[ERROR] 请选择固件传输模式")
            return
        if not update_m5_project_status():
            page.update()
            m5_source_log("[ERROR] 请先选择有效的 M5 PlatformIO 工程目录")
            return
        pio_path = find_platformio()
        if not pio_path:
            m5_source_log("[ERROR] 未找到 PlatformIO CLI")
            return

        def worker():
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            env["NO_COLOR"] = "1"
            m5_source_log(f"[INFO] 工程目录: {m5_project_field.value}")
            m5_source_log(f"[INFO] 固件传输模式: {transport}")
            m5_source_log(f"[INFO] 开始烧录到端口 {upload_port}")
            return_code = run_command(
                platformio_m5_upload_command(pio_path, transport, upload_port),
                cwd=m5_project_field.value,
                env=env,
                emit_line=m5_source_log,
            )
            if return_code == 0:
                m5_source_log(f"[SUCCESS] M5 {transport} 固件烧录成功")
            else:
                m5_source_log(f"[FAILED] M5 固件烧录失败，退出码 {return_code}")

        start_operation(worker, m5_source_log)

    def update_m5_bin_source_state(_event=None):
        use_custom_bin = m5_bin_source_group.value == "custom"
        m5_bundled_dropdown.disabled = use_custom_bin
        m5_custom_bin_field.disabled = not use_custom_bin
        m5_bin_select_button.disabled = not use_custom_bin

        if use_custom_bin:
            custom_value = (m5_custom_bin_field.value or "").strip()
            filename = custom_value if custom_value else "尚未选择文件"
            m5_bin_source_status.value = f"当前烧录来源：自选 M5 BIN | {filename}"
        else:
            transport = m5_bundled_dropdown.value
            firmware_path = bundled_m5_firmware.get(transport) if transport else None
            filename = firmware_path.name if firmware_path else "尚无内置固件"
            m5_bin_source_status.value = f"当前烧录来源：内置 M5 BIN | {filename}"

        if _event is not None:
            page.update()

    def selected_m5_bin_path():
        if m5_bin_source_group.value == "custom":
            custom_path = Path((m5_custom_bin_field.value or "").strip())
            return custom_path if custom_path.is_file() else None
        transport = m5_bundled_dropdown.value
        return bundled_m5_firmware.get(transport) if transport else None

    def flash_m5_bin(_event):
        upload_port = m5_bin_port_dropdown.value
        firmware_path = selected_m5_bin_path()
        source_name = (
            "自选合并 BIN" if m5_bin_source_group.value == "custom" else "内置合并 BIN")
        if not upload_port:
            m5_bin_log("[ERROR] 请选择 M5 烧录串口")
            return
        if firmware_path is None:
            m5_bin_log("[ERROR] 请选择内置 M5 固件或自选合并 BIN 文件")
            return
        esptool_path = find_esptool()
        if not esptool_path:
            m5_bin_log("[ERROR] 未找到 esptool，无法直接烧录 M5 固件")
            return

        def worker():
            m5_bin_log(f"[INFO] 烧录来源: {source_name}")
            m5_bin_log(f"[INFO] 合并 BIN: {firmware_path}")
            m5_bin_log(f"[INFO] esptool: {esptool_path}")
            return_code = run_command(
                esptool_m5_bin_command(esptool_path, upload_port, firmware_path),
                emit_line=m5_bin_log,
            )
            if return_code == 0:
                m5_bin_log(f"[SUCCESS] M5 固件烧录成功: {firmware_path.name}")
            else:
                m5_bin_log(f"[FAILED] M5 固件烧录失败，退出码 {return_code}")

        start_operation(worker, m5_bin_log)

    def read_encoder_angle(serial_port, device_id):
        serial_port.reset_input_buffer()
        serial_port.write(make_request_packet(device_id))
        serial_port.flush()
        time.sleep(0.012)
        response = serial_port.read_all()
        for index in range(max(0, len(response) - 3)):
            packet = response[index:index + 4]
            decoded = decode_packet(packet)
            if decoded is None:
                continue
            response_id, response_command, response_data = decoded
            if response_id != device_id or response_command != 1 or response_data == 0:
                continue
            raw15 = (response_data >> 6) & 0x7FFF
            raw15 = raw15 - 32768 if raw15 & 0x4000 else raw15
            return raw15 / 32768.0 * 360.0
        return None

    def run_id_and_angle_test(port, logger):
        test_stop_event.clear()

        def worker():
            logger(f"[SCAN] 正在通过 {port} 扫描 ID 1-16")
            detected_ids = []
            with serial.Serial(port, 2000000, timeout=0.05) as serial_port:
                for device_id in range(1, 17):
                    if test_stop_event.is_set():
                        logger("[INFO] 测试已停止")
                        return
                    angle = read_encoder_angle(serial_port, device_id)
                    if angle is not None:
                        detected_ids.append(device_id)
                        logger(f"[FOUND] ID={device_id:02d}  当前角度={angle:+.2f}°")

                if not detected_ids:
                    logger("[FAILED] 未检测到编码器，请检查端口、A/B、供电和波特率")
                    return

                logger(
                    "[SUCCESS] 检测到 ID: " +
                    ", ".join(f"{device_id:02d}" for device_id in detected_ids))
                logger("[INFO] 开始持续输出角度，点击“停止测试”结束")

                while not test_stop_event.is_set():
                    values = []
                    for device_id in detected_ids:
                        angle = read_encoder_angle(serial_port, device_id)
                        values.append(
                            f"ID{device_id:02d}={angle:+7.2f}°"
                            if angle is not None else f"ID{device_id:02d}=TIMEOUT")
                    logger("[ANGLE] " + "  ".join(values))
                    test_stop_event.wait(0.1)
            logger("[INFO] 测试已停止")

        start_operation(worker, logger, testing=True)

    def source_test(_event):
        if not rs485_dropdown.value:
            source_log("[ERROR] 请选择 RS485 测试端口")
            return
        run_id_and_angle_test(rs485_dropdown.value, source_log)

    def hex_test(_event):
        if not hex_rs485_dropdown.value:
            hex_log("[ERROR] 请选择 RS485 测试端口")
            return
        run_id_and_angle_test(hex_rs485_dropdown.value, hex_log)

    def stop_test(_event):
        test_stop_event.set()

    source_flash_button = ft.ElevatedButton(
        "一键烧录", icon=ft.Icons.DOWNLOAD, on_click=flash_platformio,
        height=45, width=170)
    source_test_button = ft.ElevatedButton(
        "ID/角度测试", icon=ft.Icons.SENSORS, on_click=source_test,
        height=45, width=170)
    source_stop_button = ft.ElevatedButton(
        "停止测试", icon=ft.Icons.STOP, on_click=stop_test,
        height=45, width=150, disabled=True)
    hex_flash_button = ft.ElevatedButton(
        "直接烧录 HEX", icon=ft.Icons.DOWNLOAD, on_click=flash_hex,
        height=45, width=170)
    hex_test_button = ft.ElevatedButton(
        "ID/角度测试", icon=ft.Icons.SENSORS, on_click=hex_test,
        height=45, width=170)
    hex_stop_button = ft.ElevatedButton(
        "停止测试", icon=ft.Icons.STOP, on_click=stop_test,
        height=45, width=150, disabled=True)
    m5_source_flash_button = ft.ElevatedButton(
        "编译并烧录 M5", icon=ft.Icons.DOWNLOAD, on_click=flash_m5_platformio,
        height=45, width=190)
    m5_bin_flash_button = ft.ElevatedButton(
        "直接烧录 M5", icon=ft.Icons.DOWNLOAD, on_click=flash_m5_bin,
        height=45, width=190)
    project_select_button = ft.IconButton(
        icon=ft.Icons.FOLDER_OPEN,
        tooltip="选择 PlatformIO 工程目录",
        on_click=select_project,
    )
    hex_select_button = ft.IconButton(
        icon=ft.Icons.FOLDER_OPEN,
        tooltip="选择自定义 HEX 文件",
        on_click=select_hex,
    )
    m5_project_select_button = ft.IconButton(
        icon=ft.Icons.FOLDER_OPEN,
        tooltip="选择 M5 PlatformIO 工程目录",
        on_click=select_m5_project,
    )
    m5_bin_select_button = ft.IconButton(
        icon=ft.Icons.FOLDER_OPEN,
        tooltip="选择 M5 ESP32-S3 合并 BIN 文件",
        on_click=select_m5_bin,
    )
    bundled_hex_dropdown.on_change = update_hex_source_state
    m5_bundled_dropdown.on_change = update_m5_bin_source_state

    theme_default = ft.Theme(color_scheme_seed=ft.Colors.BLUE)
    theme_sakura = ft.Theme(color_scheme_seed="#C1A0AC")
    page.theme_mode = ft.ThemeMode.LIGHT
    page.theme = theme_sakura
    page.bgcolor = "#FBF5F7"

    def apply_button_colors(sakura):
        if sakura:
            source_flash_button.bgcolor = "#C1A0AC"
            source_flash_button.color = "#16131F"
            hex_flash_button.bgcolor = "#C1A0AC"
            hex_flash_button.color = "#16131F"
            m5_source_flash_button.bgcolor = "#C1A0AC"
            m5_source_flash_button.color = "#16131F"
            m5_bin_flash_button.bgcolor = "#C1A0AC"
            m5_bin_flash_button.color = "#16131F"
            for button in (
                    source_test_button, source_stop_button,
                    hex_test_button, hex_stop_button):
                button.bgcolor = "#806C79"
                button.color = "#16131F"
        else:
            source_flash_button.bgcolor = ft.Colors.ORANGE_700
            source_flash_button.color = ft.Colors.WHITE
            hex_flash_button.bgcolor = ft.Colors.ORANGE_700
            hex_flash_button.color = ft.Colors.WHITE
            m5_source_flash_button.bgcolor = ft.Colors.ORANGE_700
            m5_source_flash_button.color = ft.Colors.WHITE
            m5_bin_flash_button.bgcolor = ft.Colors.ORANGE_700
            m5_bin_flash_button.color = ft.Colors.WHITE
            for button in (
                    source_test_button, source_stop_button,
                    hex_test_button, hex_stop_button):
                button.bgcolor = ft.Colors.BLUE_700
                button.color = ft.Colors.WHITE

    def toggle_theme(_event):
        if page.theme == theme_sakura:
            page.theme = theme_default
            page.theme_mode = ft.ThemeMode.DARK
            page.bgcolor = None
            theme_button.icon = ft.Icons.DARK_MODE
            theme_button.tooltip = "切换到樱花主题"
            apply_button_colors(False)
        else:
            page.theme = theme_sakura
            page.theme_mode = ft.ThemeMode.LIGHT
            page.bgcolor = "#FBF5F7"
            theme_button.icon = ft.Icons.LOCAL_FLORIST
            theme_button.tooltip = "切换到极客主题"
            apply_button_colors(True)
        page.update()

    theme_button = ft.IconButton(
        icon=ft.Icons.LOCAL_FLORIST,
        tooltip="切换到极客主题",
        on_click=toggle_theme,
    )
    apply_button_colors(True)
    update_project_status()
    update_m5_project_status()
    update_hex_source_state()
    update_m5_bin_source_state()

    source_page = ft.Container(
        padding=0,
        content=ft.Column(
            expand=True,
            controls=[
                ft.Text("1. PlatformIO 工程", size=16, weight=ft.FontWeight.W_500),
                ft.Row([project_field, project_select_button]),
                project_status,
                ft.Container(height=8),
                ft.Text("2. 硬件配置", size=16, weight=ft.FontWeight.W_500),
                ft.Row([
                    updi_dropdown,
                    rs485_dropdown,
                    id_field,
                    ft.IconButton(
                        icon=ft.Icons.REFRESH,
                        tooltip="刷新端口",
                        on_click=refresh_ports,
                    ),
                ], spacing=16, wrap=False),
                ft.Container(height=8),
                ft.Text("3. 固件操作", size=16, weight=ft.FontWeight.W_500),
                ft.Row(
                    [source_flash_button, source_test_button, source_stop_button],
                    spacing=16),
                ft.Container(height=8),
                ft.Row([
                    ft.Text("4. 运行日志", size=16, weight=ft.FontWeight.W_500),
                    ft.IconButton(
                        icon=ft.Icons.DELETE_OUTLINE,
                        tooltip="清空日志",
                        on_click=clear_source_console,
                    ),
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),
                ft.Row([source_console], expand=True),
            ],
        ),
    )

    hex_page = ft.Container(
        padding=12,
        content=ft.Column(
            expand=True,
            controls=[
                ft.Text("1. HEX 固件", size=16, weight=ft.FontWeight.W_500),
                ft.Column(
                    spacing=2,
                    controls=[
                        hex_source_group,
                        bundled_hex_dropdown,
                        ft.Row([custom_hex_field, hex_select_button]),
                        hex_source_status,
                    ],
                ),
                ft.Container(height=8),
                ft.Text("2. 硬件配置", size=16, weight=ft.FontWeight.W_500),
                ft.Row([
                    hex_updi_dropdown,
                    hex_rs485_dropdown,
                    ft.IconButton(
                        icon=ft.Icons.REFRESH,
                        tooltip="刷新端口",
                        on_click=refresh_ports,
                    ),
                ], spacing=16, wrap=False),
                write_fuses_checkbox,
                ft.Container(height=8),
                ft.Text("3. 固件操作", size=16, weight=ft.FontWeight.W_500),
                ft.Row(
                    [hex_flash_button, hex_test_button, hex_stop_button],
                    spacing=16),
                ft.Container(height=8),
                ft.Row([
                    ft.Text("4. 运行日志", size=16, weight=ft.FontWeight.W_500),
                    ft.IconButton(
                        icon=ft.Icons.DELETE_OUTLINE,
                        tooltip="清空日志",
                        on_click=clear_hex_console,
                    ),
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),
                ft.Row([hex_console], expand=True),
            ],
        ),
    )

    m5_source_page = ft.Container(
        padding=12,
        content=ft.Column(
            expand=True,
            controls=[
                ft.Text("1. M5 PlatformIO 工程", size=16, weight=ft.FontWeight.W_500),
                ft.Row([m5_project_field, m5_project_select_button]),
                m5_project_status,
                ft.Container(height=8),
                ft.Text("2. 固件与硬件配置", size=16, weight=ft.FontWeight.W_500),
                ft.Row([
                    m5_transport_dropdown,
                    m5_source_port_dropdown,
                    ft.IconButton(
                        icon=ft.Icons.REFRESH,
                        tooltip="刷新端口",
                        on_click=refresh_ports,
                    ),
                ], spacing=16, wrap=False),
                ft.Container(height=8),
                ft.Text("3. 固件操作", size=16, weight=ft.FontWeight.W_500),
                ft.Row([m5_source_flash_button], spacing=16),
                ft.Container(height=8),
                ft.Row([
                    ft.Text("4. 运行日志", size=16, weight=ft.FontWeight.W_500),
                    ft.IconButton(
                        icon=ft.Icons.DELETE_OUTLINE,
                        tooltip="清空日志",
                        on_click=clear_m5_source_console,
                    ),
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),
                ft.Row([m5_source_console], expand=True),
            ],
        ),
    )

    m5_bin_page = ft.Container(
        padding=12,
        content=ft.Column(
            expand=True,
            controls=[
                ft.Text("1. M5 固件", size=16, weight=ft.FontWeight.W_500),
                ft.Text("ESP32-S3 直烧使用合并 BIN（包含引导程序、分区表和应用）", size=12),
                ft.Column(
                    spacing=2,
                    controls=[
                        m5_bin_source_group,
                        m5_bundled_dropdown,
                        ft.Row([m5_custom_bin_field, m5_bin_select_button]),
                        m5_bin_source_status,
                    ],
                ),
                ft.Container(height=8),
                ft.Text("2. 硬件配置", size=16, weight=ft.FontWeight.W_500),
                ft.Row([
                    m5_bin_port_dropdown,
                    ft.IconButton(
                        icon=ft.Icons.REFRESH,
                        tooltip="刷新端口",
                        on_click=refresh_ports,
                    ),
                ], spacing=16, wrap=False),
                ft.Container(height=8),
                ft.Text("3. 固件操作", size=16, weight=ft.FontWeight.W_500),
                ft.Row([m5_bin_flash_button], spacing=16),
                ft.Container(height=8),
                ft.Row([
                    ft.Text("4. 运行日志", size=16, weight=ft.FontWeight.W_500),
                    ft.IconButton(
                        icon=ft.Icons.DELETE_OUTLINE,
                        tooltip="清空日志",
                        on_click=clear_m5_bin_console,
                    ),
                ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),
                ft.Row([m5_bin_console], expand=True),
            ],
        ),
    )

    tabs = ft.Tabs(
        selected_index=1,
        length=4,
        expand=True,
        content=ft.Column([
            ft.TabBar(tabs=[
                ft.Tab(label="编码器｜源码烧录"),
                ft.Tab(label="编码器｜HEX 直烧"),
                ft.Tab(label="M5｜源码烧录"),
                ft.Tab(label="M5｜HEX 直烧"),
            ]),
            ft.TabBarView(
                controls=[source_page, hex_page, m5_source_page, m5_bin_page],
                expand=True,
            ),
        ]),
    )

    page.add(ft.Column(
        expand=True,
        controls=[
            ft.Row(
                [ft.Text(APP_TITLE, size=26, weight=ft.FontWeight.BOLD), theme_button],
                alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
            ),
            ft.Divider(height=16),
            tabs,
        ],
    ))
    hex_source_group.value = DEFAULT_HEX_SOURCE
    custom_hex_field.value = ""
    update_hex_source_state()
    m5_bin_source_group.value = "bundled"
    m5_custom_bin_field.value = ""
    update_m5_bin_source_state()
    page.update()

    if project_field.value:
        source_log(f"[INFO] 自动定位 PlatformIO 工程: {project_field.value}")
    else:
        source_log("[WARN] 未自动找到工程，请点击文件夹按钮选择 encoder 工程")
    pio_path = find_platformio()
    source_log(
        f"[INFO] PlatformIO CLI: {pio_path}"
        if pio_path else "[WARN] 未找到 PlatformIO CLI，烧录前请安装或加入 PATH")
    hex_log(f"[INFO] 已找到 {len(bundled_hexes)} 个内置 HEX")
    pymcuprog_path = find_pymcuprog()
    hex_log(
        f"[INFO] pymcuprog: {pymcuprog_path}"
        if pymcuprog_path else "[WARN] 未找到 pymcuprog，HEX 直烧不可用")
    if m5_project_field.value:
        m5_source_log(f"[INFO] 自动定位 M5 PlatformIO 工程: {m5_project_field.value}")
    else:
        m5_source_log("[WARN] 未自动找到 M5 工程，请点击文件夹按钮选择")
    m5_bin_log(f"[INFO] 已找到 {len(bundled_m5_firmware)} 个内置 M5 固件")
    esptool_path = find_esptool()
    m5_bin_log(
        f"[INFO] esptool: {esptool_path}"
        if esptool_path else "[WARN] 未找到 esptool，M5 直烧不可用")


def select_view():
    requested = os.environ.get("KER_ENCODER_TOOL_VIEW", "").lower()
    if requested == "desktop":
        return ft.AppView.FLET_APP
    if requested == "browser":
        return ft.AppView.WEB_BROWSER
    return ft.AppView.WEB_BROWSER if sys.platform.startswith("linux") else ft.AppView.FLET_APP


if __name__ == "__main__":
    view = select_view()
    options = {"view": view}
    if view == ft.AppView.WEB_BROWSER:
        options["host"] = "127.0.0.1"
        options["port"] = int(os.environ.get("KER_ENCODER_TOOL_PORT", "8550"))
    ft.run(main, **options)
