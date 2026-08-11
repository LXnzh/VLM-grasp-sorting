import importlib
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import messagebox
import subprocess
import shutil
from collections import OrderedDict

package_name = "pyperclip"

try:
    pyperclip = importlib.import_module(package_name)
except ImportError:
    print(f"{package_name} not found. Installing...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
    print(f"{package_name} installed successfully. Please restart the application.")
    exit(0)

# === CONFIGURATION: Define button labels and corresponding commands ===
# Format: "Button Label": ("command to run", stay_open_after)
# For grouped buttons, use a common prefix with variants in parentheses, e.g.:
# "Launch Basic Robot Setup (Real)", "Launch Basic Robot Setup (Sim)", ...
COMMANDS = {
    "Mujoco Simulation (Start - headless)": ("cd ./src/ifl_air_mujoco_sim/ && ./.venv/bin/python ./ros2_main.py sim.headless=true", False),
    "Mujoco Simulation (Start - with GUI)": ("cd ./src/ifl_air_mujoco_sim/ && ./.venv/bin/python ./ros2_main.py", False),
    "Mujoco Simulation (Reset GUI)": ("ros2 run sim_pick_place sim_reset_gui_node", True),
    "Launch ROS2 Setup (Mujoco)": ("./scripts/launch_ros2_robot_setup_mujoco.bash", True),
    "Launch sim_pick_place (pick_place)": ("ros2 run sim_pick_place sim_pick_place_node", False),
}

# === Check if 'terminator' is installed ===
def check_terminator():
    return shutil.which("terminator") is not None

# === Run the command in a new terminal window ===
def run_in_new_terminal(command, close_when_done):
    if not check_terminator():
        messagebox.showerror(
            "Missing Dependency",
            "The 'terminator' terminal emulator is not installed.\n"
            "Please install it with:\n\nsudo apt install terminator"
        )
        pyperclip.copy("sudo apt install terminator")
        return

    # If terminal should stay open, add 'exec bash' after command
    full_command = f"{command}; exec bash" if not close_when_done else command

    # Launch terminator
    subprocess.Popen(["terminator", "-x", "bash", "-c", full_command])


# === Helper: group commands by common prefix ===
def group_commands(commands_dict):
    """
    Group commands by the part before the final ' (Variant)'.
    Example key:
        'Launch Basic Robot Setup (Real)'
    becomes:
        prefix: 'Launch Basic Robot Setup'
        variant: 'Real'
    Commands without '(...)' stay as singletons.
    """
    grouped = OrderedDict()

    for label, (cmd, close_when_done) in commands_dict.items():
        # Look for pattern: "<prefix> (<variant>)"
        if " (" in label and label.endswith(")"):
            prefix, variant_with_paren = label.rsplit(" (", 1)
            variant = variant_with_paren[:-1]  # strip trailing ')'
        else:
            # No variant -> treat full label as prefix, no variant
            prefix = label
            variant = None

        if prefix not in grouped:
            grouped[prefix] = []

        grouped[prefix].append((variant, cmd, close_when_done))

    return grouped


# === GUI Setup ===
root = tk.Tk()
root.title("GUI Command Launcher")

icon_path = Path(__file__).with_name("gui_icon.png")  # put icon.png next to your .py
icon = tk.PhotoImage(file=str(icon_path))
root.iconphoto(True, icon) 


# Frame that holds the label+buttons grid
button_frame = tk.Frame(root)
button_frame.pack(fill=tk.BOTH, expand=True)

grouped_commands = group_commands(COMMANDS)

# Determine maximum number of buttons in any group (for layout / columnspan)
max_variants = 1
for prefix, items in grouped_commands.items():
    if len(items) == 1 and items[0][0] is None:
        # Single, non-variant command -> 1 button
        max_variants = max(max_variants, 1)
    else:
        # Multiple variant buttons
        max_variants = max(max_variants, len(items))

# Configure grid columns: column 0 for labels, 1..max_variants for buttons
button_frame.grid_columnconfigure(0, weight=0)  # label column fixed size (max label width)
for col in range(1, max_variants + 1):
    button_frame.grid_columnconfigure(col, weight=1)  # buttons expand horizontally

row = 0
for prefix, items in grouped_commands.items():
    # Label in column 0, same column for all rows
    lbl = tk.Label(button_frame, text=prefix, anchor="w")
    lbl.grid(row=row, column=0, padx=(10, 10), pady=5, sticky="w")

    # Case 1: only one item and no variant -> label + 1 big button spanning rest of row
    if len(items) == 1 and items[0][0] is None:
        _, cmd, close_when_done = items[0]
        btn = tk.Button(
            button_frame,
            text=prefix,
            command=lambda c=cmd, close=close_when_done: run_in_new_terminal(c, close)
        )
        btn.grid(
            row=row,
            column=1,
            columnspan=max_variants,  # occupy all button columns
            padx=(0, 10),
            pady=5,
            sticky="ew"
        )

    else:
        # Case 2: multiple variants -> multiple buttons in columns 1..N
        for idx, (variant, cmd, close_when_done) in enumerate(items):
            text = variant if variant is not None else prefix
            btn = tk.Button(
                button_frame,
                text=text,
                command=lambda c=cmd, close=close_when_done: run_in_new_terminal(c, close)
            )
            btn.grid(
                row=row,
                column=1 + idx,
                padx=(0 if idx == 0 else 5, 5),
                pady=5,
                sticky="ew"
            )

    row += 1

root.mainloop()