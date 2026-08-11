#!/usr/bin/env python3
"""Desktop launcher for the robot manipulation project.

Run from the workspace root after building the ROS 2 workspace:

    python3 gui_manager.py

Each long-running ROS task runs in the GUI background. Its complete ROS
output remains available in the built-in task monitor.
"""

from __future__ import annotations

import os
import signal
import shlex
import subprocess
import sys
import threading
import time
import webbrowser
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk


APP_TITLE = "Robot Grasping Control Console"
WORKSPACE = Path(__file__).resolve().parent
KEEP_STILL_GUIDANCE = "KEEP STILL — do not move the selected target."
TRACKING_READY_GUIDANCE = (
    "TRACKING READY — 10 seconds: select the target in MuJoCo and "
    "Ctrl+Shift+right-drag it now."
)
PBVS_FOLLOWING_GUIDANCE = (
    "PBVS FOLLOWING — continue moving, or release the target and keep it still."
)
TARGET_STOPPED_GUIDANCE = (
    "TARGET RELEASED — keep it completely still while the grasp starts."
)
FULL_STACK_LAUNCH_MARKER = "cell_small_full_mujoco_moveit.launch.py"
PROCESS_SHUTDOWN_TIMEOUT_S = 3.0


def full_stack_process_ids(proc_root=Path("/proc")) -> tuple[int, ...]:
    """Return live process IDs whose command line contains the full launch."""
    marker = FULL_STACK_LAUNCH_MARKER.encode()
    matches = []
    try:
        entries = proc_root.iterdir()
    except OSError:
        return ()
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / "cmdline").read_bytes()
        except OSError:
            continue
        if marker in command:
            matches.append(int(entry.name))
    return tuple(sorted(matches))


def terminate_process_groups(processes, timeout_s=PROCESS_SHUTDOWN_TIMEOUT_S):
    """Terminate and reap each live process and its Linux process group."""
    live = [process for process in processes if process.poll() is None]
    errors = []
    for process in live:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except OSError as exc:
            errors.append(f"SIGTERM process group {process.pid}: {exc}")

    deadline = time.monotonic() + max(0.0, float(timeout_s))
    pending = live
    while pending and time.monotonic() < deadline:
        pending = [process for process in pending if process.poll() is None]
        if pending:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    for process in pending:
        if process.poll() is not None:
            continue
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except OSError as exc:
            errors.append(f"SIGKILL process group {process.pid}: {exc}")

    reap_deadline = time.monotonic() + 1.0
    for process in live:
        remaining = max(0.0, reap_deadline - time.monotonic())
        try:
            process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            errors.append(f"Could not reap process group {process.pid}")
    return errors


class ProjectLauncher(tk.Tk):
    """GUI front end for the ROS launch and manipulation commands."""

    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.minsize(940, 690)
        self.geometry("1080x760")

        self.voice_mode = tk.BooleanVar(value=False)
        self.voice_seconds = tk.StringVar(value="5")
        self.voice_language = tk.StringVar(value="Auto")
        self.api_key = tk.StringVar()
        self.status = tk.StringVar(
            value="Ready: start the food-sorting scene, then run the default PBVS task."
        )
        self.task_phase = tk.StringVar(
            value="Waiting: start the food-sorting simulation."
        )
        self.operator_guidance = tk.StringVar(
            value="KEEP STILL — start the food-sorting simulation."
        )
        self.drag_window_active = False
        self.scene_mode: bool | None = None
        self.voice_capture_active = False
        self.active_processes: dict[int, tuple[str, subprocess.Popen[str]]] = {}
        self._closing = False
        self._icon = None

        self._set_icon()
        self._configure_style()
        self._build_layout()
        self._update_input_mode()
        self.protocol("WM_DELETE_WINDOW", self.shutdown)

    def _set_icon(self) -> None:
        icon_path = WORKSPACE / "gui_icon.png"
        if not icon_path.exists():
            return
        try:
            self._icon = tk.PhotoImage(file=str(icon_path))
            self.iconphoto(True, self._icon)
        except tk.TclError:
            # A missing or unsupported icon must not prevent task control.
            pass

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Title.TLabel", font=("Arial", 18, "bold"))
        style.configure("Subtitle.TLabel", foreground="#52606d")
        style.configure("Card.TLabelframe", padding=12)
        style.configure("Card.TLabelframe.Label", font=("Arial", 11, "bold"))
        style.configure("Primary.TButton", font=("Arial", 10, "bold"), padding=8)
        style.configure("Action.TButton", font=("Arial", 10), padding=8)
        style.configure(
            "Guidance.TLabel",
            background="#d8f3dc",
            foreground="#0b3d2e",
            font=("Arial", 11, "bold"),
            padding=10,
        )

    def _build_layout(self) -> None:
        content = ttk.Frame(self, padding=16)
        content.pack(fill=tk.BOTH, expand=True)
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(3, weight=1)

        header = ttk.Frame(content)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text=APP_TITLE, style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            header,
            text=(
                "VLM selection · PBVS tracking · stable grasp · food sorting"
            ),
            style="Subtitle.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(3, 0))
        ttk.Button(
            header, text="Rebuild my_course_pkg", command=self.build_package
        ).grid(row=0, column=1, rowspan=2, sticky="e")

        startup = ttk.LabelFrame(
            content, text="1. Start the Food-Sorting Scene", style="Card.TLabelframe"
        )
        startup.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        startup.columnconfigure(0, weight=1)
        startup.columnconfigure(1, weight=1)
        startup.columnconfigure(2, weight=1)

        ttk.Button(
            startup,
            text="Start Food-Sorting Simulation",
            style="Primary.TButton",
            command=self.start_food_scene,
        ).grid(row=0, column=0, columnspan=2, padx=(0, 8), sticky="ew")
        ttk.Button(
            startup,
            text="Reset Simulation",
            style="Action.TButton",
            command=self.reset_simulation,
        ).grid(row=0, column=2, padx=(8, 0), sticky="ew")
        ttk.Label(
            startup,
            text=(
                "This is the only simulation scene in the GUI: it creates a random "
                "food_bin and enables VLM-based food placement."
            ),
            style="Subtitle.TLabel",
            wraplength=930,
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(9, 0))

        instruction_card = ttk.LabelFrame(
            content, text="2. Natural-Language Instruction", style="Card.TLabelframe"
        )
        instruction_card.grid(row=2, column=0, sticky="nsew", padx=(0, 6), pady=(0, 12))
        instruction_card.columnconfigure(0, weight=1)
        ttk.Label(
            instruction_card,
            text=(
                "Describe the object and location, for example: "
                "pick up the apple near the food bin."
            ),
            style="Subtitle.TLabel",
            wraplength=450,
        ).grid(row=0, column=0, sticky="w")
        self.instruction_text = tk.Text(
            instruction_card,
            height=5,
            wrap=tk.WORD,
            font=("Arial", 11),
            relief=tk.SOLID,
            borderwidth=1,
        )
        self.instruction_text.grid(row=1, column=0, sticky="ew", pady=(8, 10))
        self.instruction_text.bind("<Control-Return>", self._run_default_from_shortcut)

        input_options = ttk.Frame(instruction_card)
        input_options.grid(row=2, column=0, sticky="ew")
        input_options.columnconfigure(4, weight=1)
        ttk.Checkbutton(
            input_options,
            text="Use microphone input",
            variable=self.voice_mode,
            command=self._update_input_mode,
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(input_options, text="Seconds:").grid(
            row=0, column=1, padx=(12, 0)
        )
        self.voice_seconds_entry = ttk.Entry(
            input_options, textvariable=self.voice_seconds, width=4
        )
        self.voice_seconds_entry.grid(row=0, column=2, padx=(2, 8))
        ttk.Label(input_options, text="Language:").grid(row=0, column=3)
        self.voice_language_box = ttk.Combobox(
            input_options,
            textvariable=self.voice_language,
            values=("Auto", "Chinese", "English", "German"),
            width=10,
            state="readonly",
        )
        self.voice_language_box.grid(row=0, column=4, sticky="w", padx=(2, 0))
        self.voice_record_button = ttk.Button(
            input_options,
            text="Record voice now",
            command=self.capture_voice_instruction,
        )
        self.voice_record_button.grid(row=0, column=5, padx=(10, 0), sticky="e")

        ttk.Label(
            instruction_card,
            text=(
                "Text mode sends the instruction directly to the VLM. Voice "
                "mode uses the project's microphone workflow."
            ),
            style="Subtitle.TLabel",
            wraplength=450,
        ).grid(row=3, column=0, sticky="w", pady=(9, 0))
        ttk.Button(
            instruction_card,
            text="Start Complete Grasp-and-Sort Flow",
            style="Primary.TButton",
            command=self.run_dynamic_grasp,
        ).grid(row=4, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(
            instruction_card,
            text="After typing, click the button above or press Ctrl+Enter. In voice mode, it opens the recording page after the robot is ready.",
            style="Subtitle.TLabel",
            wraplength=450,
        ).grid(row=5, column=0, sticky="w", pady=(7, 0))

        flow_card = ttk.LabelFrame(
            content, text="3. Complete Flow", style="Card.TLabelframe"
        )
        flow_card.grid(row=2, column=1, sticky="nsew", padx=(6, 0), pady=(0, 12))
        flow_card.columnconfigure(0, weight=1)
        ttk.Label(
            flow_card,
            text=(
                "1. Capture the home overview RGB-D frame.\n"
                "2. Use the VLM to select and classify one supported object.\n"
                "3. Freeze the food-bin or non-food drop target.\n"
                "4. Lock with SAM2 and follow the moving object with PBVS.\n"
                "5. After a continuous stable window, estimate the 6D pose.\n"
                "6. Execute the pure stable grasp through lift, then place and return home."
            ),
            style="Subtitle.TLabel",
            wraplength=455,
            justify=tk.LEFT,
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            flow_card,
            text=(
                "Food goes to the visually located random food_bin. Non-food goes "
                "to the configured world target. Any unknown identity, category, "
                "mask, pose, stability gate, or placement target fails closed."
            ),
            style="Subtitle.TLabel",
            wraplength=455,
        ).grid(row=1, column=0, sticky="w", pady=(14, 0))

        monitor = ttk.LabelFrame(
            content, text="4. Live Task Monitor — Complete ROS Log", style="Card.TLabelframe"
        )
        monitor.grid(row=3, column=0, columnspan=2, sticky="nsew")
        monitor.columnconfigure(1, weight=1)
        monitor.columnconfigure(3, weight=2)
        monitor.rowconfigure(3, weight=1)

        ttk.Label(monitor, text="VLM / Voice API Key:").grid(
            row=0, column=0, sticky="w"
        )
        self.api_key_entry = ttk.Entry(monitor, textvariable=self.api_key, show="*")
        self.api_key_entry.grid(row=0, column=1, sticky="ew", padx=(8, 18))
        ttk.Label(monitor, text="Robot state:").grid(row=0, column=2, sticky="w")
        ttk.Label(
            monitor,
            textvariable=self.task_phase,
            style="Subtitle.TLabel",
            wraplength=560,
        ).grid(row=0, column=3, sticky="w", padx=(8, 0))
        ttk.Label(
            monitor,
            text=(
                "The full log is retained for this GUI session. Use the state message "
                "above to know when the target may move or must remain still."
            ),
            style="Subtitle.TLabel",
            wraplength=930,
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 6))
        monitor_actions = ttk.Frame(monitor)
        monitor_actions.grid(row=1, column=3, sticky="e", pady=(8, 6))
        ttk.Button(
            monitor_actions,
            text="Open Reset GUI",
            command=self.open_reset_gui,
        ).pack(side=tk.LEFT)
        ttk.Button(
            monitor_actions,
            text="List ROS Topics",
            command=self.list_topics,
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(
            monitor_actions,
            text="Save Log",
            command=self.save_log,
        ).pack(side=tk.LEFT)
        ttk.Button(
            monitor_actions,
            text="Clear",
            command=self.clear_log,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(
            monitor,
            textvariable=self.operator_guidance,
            style="Guidance.TLabel",
            anchor="center",
            justify="center",
            wraplength=990,
        ).grid(
            row=2,
            column=0,
            columnspan=4,
            sticky="ew",
            pady=(2, 8),
        )
        self.log = scrolledtext.ScrolledText(
            monitor,
            height=24,
            state=tk.DISABLED,
            wrap=tk.WORD,
            font=("Consolas", 10),
        )
        self.log.grid(row=3, column=0, columnspan=4, sticky="nsew")

        footer = ttk.Label(content, textvariable=self.status, style="Subtitle.TLabel")
        footer.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(12, 0))

    def _update_input_mode(self) -> None:
        """Enable the controls relevant to the chosen instruction source."""
        voice_enabled = self.voice_mode.get()
        text_state = tk.DISABLED if voice_enabled else tk.NORMAL
        option_state = "normal" if voice_enabled else "disabled"
        self.instruction_text.configure(state=text_state)
        self.voice_seconds_entry.configure(state=option_state)
        self.voice_language_box.configure(state="readonly" if voice_enabled else "disabled")
        self.voice_record_button.configure(
            state="normal" if voice_enabled else "disabled"
        )

    def _run_default_from_shortcut(self, _event: tk.Event) -> str:
        """Submit the typed instruction with the default PBVS workflow."""
        self.run_dynamic_grasp()
        return "break"

    def _instruction_args(self) -> str | None:
        instruction = self.instruction_text.get("1.0", tk.END).strip()
        if not instruction:
            messagebox.showerror(
                APP_TITLE,
                "Enter a natural-language grasp instruction or enable microphone input.",
            )
            return None
        return f"--instruction {shlex.quote(instruction)}"

    def capture_voice_instruction(self, after_capture=None) -> None:
        """Capture and transcribe speech before launching a robot task.

        Recording is intentionally independent of the PBVS startup sequence so
        the user sees the microphone UI immediately after choosing voice.
        """
        if self.voice_capture_active:
            messagebox.showinfo(
                APP_TITLE,
                "Voice recording is already in progress. Complete it in the browser page.",
            )
            return
        if not self.voice_mode.get():
            self.voice_mode.set(True)
            self._update_input_mode()
        if not self._require_api_key():
            return
        try:
            seconds = int(self.voice_seconds.get())
        except ValueError:
            messagebox.showerror(APP_TITLE, "Recording duration must be a positive integer.")
            return
        if seconds <= 0:
            messagebox.showerror(APP_TITLE, "Recording duration must be greater than zero.")
            return

        language_codes = {
            "Auto": None,
            "Chinese": "zh",
            "English": "en",
            "German": "de",
        }
        dialog = self._create_voice_capture_dialog()
        api_key = self.api_key.get().strip()
        self.voice_capture_active = True
        self.status.set("Preparing the browser voice recorder...")
        self._write_log("Preparing browser voice input at http://localhost:8765")
        threading.Thread(
            target=self._capture_voice_worker,
            args=(
                seconds,
                language_codes[self.voice_language.get()],
                api_key,
                after_capture,
                dialog,
            ),
            daemon=True,
        ).start()

    def _capture_voice_worker(
        self,
        seconds: int,
        language: str | None,
        api_key: str,
        after_capture,
        dialog: tk.Toplevel,
    ) -> None:
        """Run the existing browser recorder and return its transcription."""
        package_source = str(WORKSPACE / "src" / "my_course_pkg")
        if package_source not in sys.path:
            sys.path.insert(0, package_source)
        audio_path = None
        previous_values = {
            key: os.environ.get(key)
            for key in ("VLM_API_KEY", "VOICE_API_KEY")
        }
        try:
            if api_key:
                os.environ["VLM_API_KEY"] = api_key
                os.environ["VOICE_API_KEY"] = api_key
            from my_course_pkg.tasks.voice_input.live_input import (
                record_browser_instruction,
                transcribe_audio_instruction,
            )

            audio_path = record_browser_instruction(
                seconds,
                port=8765,
                timeout_sec=180,
                on_ready=lambda url: self._schedule_voice_capture_ready(dialog, url),
                open_browser=False,
            )
            instruction = transcribe_audio_instruction(
                audio_path,
                model=os.environ.get(
                    "VOICE_TRANSCRIPTION_MODEL",
                    "kit.whisper-large-v3",
                ),
                language=language,
            )
        except Exception as exc:
            self._schedule_voice_capture_failed(dialog, str(exc))
        else:
            self._schedule_voice_capture_succeeded(
                dialog,
                instruction,
                after_capture,
            )
        finally:
            if audio_path is not None:
                audio_path.unlink(missing_ok=True)
            for key, value in previous_values.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

    def _create_voice_capture_dialog(self) -> tk.Toplevel:
        dialog = tk.Toplevel(self)
        dialog.title("Voice Input")
        dialog.transient(self)
        dialog.resizable(False, False)
        dialog.protocol("WM_DELETE_WINDOW", dialog.withdraw)
        body = ttk.Frame(dialog, padding=18)
        body.pack(fill=tk.BOTH, expand=True)
        ttk.Label(body, text="Voice input", style="Title.TLabel").pack(anchor="w")
        dialog.voice_status = tk.StringVar(
            value="Starting the browser recording page..."
        )
        ttk.Label(body, textvariable=dialog.voice_status, wraplength=460).pack(
            anchor="w", pady=(8, 10)
        )
        dialog.voice_url = tk.StringVar(value="http://localhost:8765")
        url_entry = ttk.Entry(body, textvariable=dialog.voice_url, width=48)
        url_entry.configure(state="readonly")
        url_entry.pack(fill=tk.X, pady=(0, 12))
        dialog.open_button = ttk.Button(
            body,
            text="Open recording page",
            state="disabled",
            command=lambda: self._open_voice_page(dialog.voice_url.get()),
        )
        dialog.open_button.pack(anchor="w")
        dialog.lift()
        return dialog

    def _schedule_voice_capture_ready(self, dialog: tk.Toplevel, url: str) -> None:
        try:
            self.after(0, self._voice_capture_ready, dialog, url)
        except tk.TclError:
            pass

    def _voice_capture_ready(self, dialog: tk.Toplevel, url: str) -> None:
        if not dialog.winfo_exists():
            return
        dialog.deiconify()
        dialog.voice_url.set(url)
        dialog.voice_status.set(
            "Recording page is ready. Allow microphone access, press Start recording, then speak your instruction."
        )
        dialog.open_button.configure(state="normal")
        self._open_voice_page(url)

    def _schedule_voice_capture_succeeded(
        self,
        dialog: tk.Toplevel,
        instruction: str,
        after_capture,
    ) -> None:
        try:
            self.after(
                0,
                self._voice_capture_succeeded,
                dialog,
                instruction,
                after_capture,
            )
        except tk.TclError:
            pass

    def _voice_capture_succeeded(
        self,
        dialog: tk.Toplevel,
        instruction: str,
        after_capture,
    ) -> None:
        self.voice_capture_active = False
        if dialog.winfo_exists():
            dialog.destroy()
        self.voice_mode.set(False)
        self._update_input_mode()
        self.instruction_text.delete("1.0", tk.END)
        self.instruction_text.insert("1.0", instruction)
        self.instruction_text.focus_set()
        self.status.set("Voice instruction transcribed. Ready to start PBVS grasp.")
        self._write_log(f"Voice instruction: {instruction}")
        if after_capture is not None:
            self.after(0, after_capture)

    def _schedule_voice_capture_failed(self, dialog: tk.Toplevel, error: str) -> None:
        try:
            self.after(0, self._voice_capture_failed, dialog, error)
        except tk.TclError:
            pass

    def _voice_capture_failed(self, dialog: tk.Toplevel, error: str) -> None:
        self.voice_capture_active = False
        if dialog.winfo_exists():
            dialog.destroy()
        self.status.set("Voice input failed. See the message below.")
        self._write_log(f"Voice input failed: {error}")
        messagebox.showerror(APP_TITLE, f"Voice input failed:\n{error}")

    def _require_api_key(self) -> bool:
        """Fail early with an actionable message before starting a VLM task."""
        configured = self.api_key.get().strip() or os.environ.get("VLM_API_KEY")
        configured = configured or os.environ.get("OPENAI_API_KEY")
        if configured:
            return True
        messagebox.showerror(
            APP_TITLE,
            "Natural-language and voice tasks need a VLM API key. Enter it in "
            "the VLM / Voice API Key field, or set VLM_API_KEY before launching the GUI.",
        )
        self.api_key_entry.focus_set()
        return False

    def _ensure_food_scene(self) -> bool:
        """Make the required simulation prerequisite explicit before a task."""
        if self.scene_mode is True:
            return True
        choice = messagebox.askyesnocancel(
            APP_TITLE,
            "The food-sorting simulation has not been started from this GUI.\n\n"
            "Yes: start it now, then wait for the robot and camera to become ready.\n"
            "No: continue because the correct food-sorting scene is already running elsewhere.\n"
            "Cancel: do not start a grasp task.",
        )
        if choice is True:
            self.start_food_scene()
            messagebox.showinfo(
                APP_TITLE,
                "The food-sorting simulation has started. Wait until its ROS logs show "
                "that the robot and camera are ready, then start the PBVS task.",
            )
            return False
        return choice is False

    def _process_environment(self, *, food_mode: bool = False) -> dict[str, str]:
        """Return child-process environment without exposing keys in argv."""
        environment = os.environ.copy()
        if not environment.get("ROS_DOMAIN_ID", "").strip():
            environment.pop("ROS_DOMAIN_ID", None)
        if food_mode:
            environment["MY_COURSE_SINGLE_BIN_MODE"] = "1"
        key = self.api_key.get().strip()
        if key:
            environment["VLM_API_KEY"] = key
            environment["VOICE_API_KEY"] = key
        environment["PYTHONUNBUFFERED"] = "1"
        environment["RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED"] = "1"
        return environment

    def _workspace_command(self, command: str) -> str:
        return (
            f"cd {shlex.quote(str(WORKSPACE))}"
            " && source /opt/ros/humble/setup.bash"
            " && source install/setup.bash"
            f" && {command}"
        )

    def launch_terminal(self, title: str, command: str, *, food_mode: bool = False) -> None:
        """Launch a task with its complete output visible in the GUI monitor."""
        workspace_command = self._workspace_command(command)
        self._launch_in_gui(title, workspace_command, food_mode=food_mode)

    def _launch_in_gui(
        self,
        title: str,
        command: str,
        *,
        food_mode: bool,
    ) -> None:
        """Run a ROS command without a desktop terminal and stream its logs."""
        if self._closing:
            return
        try:
            process = subprocess.Popen(
                ["bash", "-lc", command],
                cwd=WORKSPACE,
                env=self._process_environment(food_mode=food_mode),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except OSError as exc:
            messagebox.showerror(APP_TITLE, f"Could not start the task: {exc}")
            self._write_log(f"Could not start '{title}': {exc}")
            return

        self.status.set(f"Started: {title} (running in the GUI)")
        self._write_log(f"===== STARTED: {title} =====")
        self._write_log(f"Command: {command}")
        self.active_processes[process.pid] = (title, process)
        threading.Thread(
            target=self._stream_process_output,
            args=(title, process),
            daemon=True,
        ).start()

    def _stream_process_output(
        self,
        title: str,
        process: subprocess.Popen[str],
    ) -> None:
        """Forward child output to Tk's main thread until the task exits."""
        if process.stdout is not None:
            for line in process.stdout:
                clean_line = line.rstrip()
                if clean_line:
                    self._schedule_log(f"[{title}] {clean_line}")
        return_code = process.wait()
        self.active_processes.pop(process.pid, None)
        if title == "Food-Sorting Simulation":
            self.scene_mode = False
        self._schedule_log(f"'{title}' finished (exit code {return_code}).")
        try:
            self.after(
                0,
                lambda: self.status.set(
                    f"{title} finished (exit code {return_code})."
                ),
            )
        except tk.TclError:
            # The GUI was already closed while a background task was ending.
            pass

    def _schedule_log(self, message: str) -> None:
        try:
            self.after(0, self._write_log, message)
        except tk.TclError:
            # The GUI was already closed while a background task was ending.
            pass

    def _open_voice_page(self, url: str) -> None:
        try:
            opened = webbrowser.open_new_tab(url)
        except Exception as exc:
            self._write_log(f"Could not open the voice recording page: {exc}")
            return
        if not opened:
            self._write_log(f"Open this voice recording page manually: {url}")

    def start_food_scene(self) -> None:
        if any(
            title == "Food-Sorting Simulation" and process.poll() is None
            for title, process in self.active_processes.values()
        ):
            self.status.set("Food-Sorting Simulation is already running.")
            self._write_log(
                "Food-Sorting Simulation is already running; not starting a "
                "second simulation or a second Robotiq mock server."
            )
            return
        external_pids = full_stack_process_ids()
        if external_pids:
            pid_text = ", ".join(str(pid) for pid in external_pids)
            message = (
                "An existing food-sorting ROS stack is already running outside "
                f"this GUI (PID: {pid_text}). Stop that stack before starting "
                "another one."
            )
            self.status.set("Existing external food-sorting stack detected.")
            self._write_log(message)
            messagebox.showerror(APP_TITLE, message)
            return
        self.scene_mode = True
        self.launch_terminal(
            "Food-Sorting Simulation",
            "ros2 launch ifl_air_ur_launch "
            "cell_small_full_mujoco_moveit.launch.py "
            "scene_mode:=random",
            food_mode=True,
        )

    def shutdown(self) -> None:
        """Close the GUI after terminating every process tree it owns."""
        if self._closing:
            return
        self._closing = True
        owned_processes = [
            process
            for _title, process in list(self.active_processes.values())
        ]
        errors = terminate_process_groups(owned_processes)
        self.active_processes.clear()
        self.scene_mode = False
        for error in errors:
            print(f"GUI shutdown: {error}", file=sys.stderr)
        try:
            self.destroy()
        except tk.TclError:
            pass

    def build_package(self) -> None:
        self.launch_terminal(
            "Build my_course_pkg",
            "colcon build --packages-select my_course_pkg --symlink-install",
        )

    def reset_simulation(self) -> None:
        self.launch_terminal(
            "Reset Simulation",
            "ros2 service call /reset_sim std_srvs/srv/Trigger '{}'",
        )

    def open_reset_gui(self) -> None:
        self.launch_terminal(
            "Simulation Reset GUI", "ros2 run sim_pick_place sim_reset_gui_node"
        )

    def list_topics(self) -> None:
        self.launch_terminal("ROS 2 Topics", "ros2 topic list")

    def run_dynamic_grasp(self) -> None:
        if self.voice_mode.get():
            self.capture_voice_instruction(after_capture=self.run_dynamic_grasp)
            return
        args = self._instruction_args()
        if (
            args is None
            or not self._require_api_key()
            or not self._ensure_food_scene()
        ):
            return
        self.launch_terminal(
            "PBVS Dynamic Grasp",
            f"ros2 run my_course_pkg pbvs_sorting_grasp {args}",
            food_mode=True,
        )

    def _write_log(self, message: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, f"- {message}\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)
        self._update_task_phase(message)

    def _update_task_phase(self, message: str) -> None:
        """Translate the ROS/PBVS state machine into operator guidance."""
        phase = None
        guidance = None
        if "TASK_FAILED" in message:
            reason = message.split("TASK_FAILED:", 1)[-1].strip()
            phase = f"Task failed: {reason}"
            guidance = f"TASK FAILED — {reason}"
            self.drag_window_active = False
        elif "SAFE_STOP" in message or "PBVS_TRACKING_LOST" in message:
            phase = "Safe stop / target lost: inspect the log."
            guidance = "TRACKING LOST — do not move; inspect the log."
            self.drag_window_active = False
        elif "DONE: returned to initial pose" in message:
            phase = "Task complete: robot is back at its initial pose."
            guidance = "TASK COMPLETE — robot returned to its initial pose."
            self.drag_window_active = False
        elif "EXECUTING_GRASP" in message:
            phase = "Grasp executing: do not move the target or robot."
            guidance = KEEP_STILL_GUIDANCE
            self.drag_window_active = False
        elif (
            "FOUNDATIONPOSE_REQUEST" in message
            or "PLANNING_FROM_FOUNDATIONPOSE" in message
        ):
            phase = "Pose estimation / planning: keep the target still."
            guidance = KEEP_STILL_GUIDANCE
            self.drag_window_active = False
        elif "PBVS_RELATIVE_POSE_STABLE" in message:
            phase = "Target stable: grasp planning is starting."
            guidance = KEEP_STILL_GUIDANCE
            self.drag_window_active = False
        elif "PBVS_WAITING_FOR_CONTINUOUS_STOP" in message:
            phase = "Target stopped: keep it still until grasp begins."
            guidance = TARGET_STOPPED_GUIDANCE
            self.drag_window_active = False
        elif "PBVS_FOLLOW_ACTIVE" in message or "PBVS_COMMAND" in message:
            phase = "PBVS following: the robot is tracking the target."
            guidance = PBVS_FOLLOWING_GUIDANCE
            self.drag_window_active = False
        elif "TARGET_LOCKED" in message or "PBVS_OBSERVING_FOR_MOTION" in message:
            phase = "Tracking ready: the 10-second drag window is active."
            guidance = TRACKING_READY_GUIDANCE
            self.drag_window_active = True
        elif "PBVS_MOTION_GATE: state=STABLE" in message:
            if self.drag_window_active:
                phase = "Tracking ready: the 10-second drag window is active."
            else:
                phase = "Target stable: the safety window is filling."
                guidance = KEEP_STILL_GUIDANCE
        elif "SORTING_TARGET_READY" in message:
            phase = "Sorting target ready: SAM2 is locking it."
            guidance = KEEP_STILL_GUIDANCE
            self.drag_window_active = False
        elif "VLM_SELECTION_COMPLETE" in message:
            phase = "Object selected: VLM is classifying it."
            guidance = KEEP_STILL_GUIDANCE
            self.drag_window_active = False
        elif "VLM_SELECTION_REQUEST" in message:
            phase = "VLM is selecting the instructed object."
            guidance = KEEP_STILL_GUIDANCE
            self.drag_window_active = False
        elif "INITIAL_POSE_READY" in message:
            phase = "Initial pose ready: VLM/SAM2 is selecting the target."
            guidance = KEEP_STILL_GUIDANCE
            self.drag_window_active = False
        elif "RETURNING_TO_INITIAL_POSE" in message:
            phase = "Robot is returning to its initial pose."
            guidance = KEEP_STILL_GUIDANCE
            self.drag_window_active = False
        if phase is not None:
            self.task_phase.set(phase)
        if guidance is not None:
            self.operator_guidance.set(guidance)

    def clear_log(self) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.configure(state=tk.DISABLED)

    def save_log(self) -> None:
        log_directory = WORKSPACE / "logs"
        log_directory.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = log_directory / f"gui_ros_log_{timestamp}.txt"
        log_path.write_text(self.log.get("1.0", tk.END), encoding="utf-8")
        self.status.set(f"Saved complete log to {log_path}")
        self._write_log(f"Saved complete log to {log_path}")


def main() -> None:
    app = None
    try:
        app = ProjectLauncher()
        app.mainloop()
    finally:
        if app is not None:
            app.shutdown()


if __name__ == "__main__":
    main()
