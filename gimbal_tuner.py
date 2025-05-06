import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import serial
import serial.tools.list_ports
import threading
import time
import queue
from collections import deque
import re
import math
import traceback

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

MAX_DATA_POINTS = 200
PLOT_UPDATE_MS = 100 # Can be slower for balancing (e.g., 100-200ms)
SERIAL_TIMEOUT = 0.1
# ANGLE_VIZ is less relevant for balancing, can be removed or kept for one motor
# ANGLE_VIZ_UPDATE_MS = 100
# ANGLE_VIZ_SIZE = 100

class BalanceTunerApp: # Renamed class
    def __init__(self, root):
        self.root = root
        self.root.title("Self-Balancing Robot Tuner")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.serial_port = None
        self.is_connected = False
        self.serial_thread = None
        self.serial_queue = queue.Queue()
        self.stop_serial_thread = threading.Event()
        
        self.balancing_active_gui = False # To track GUI button state

        self.time_data = deque(maxlen=MAX_DATA_POINTS)
        self.pitch_data = deque(maxlen=MAX_DATA_POINTS)
        self.roll_data = deque(maxlen=MAX_DATA_POINTS)
        self.yaw_data = deque(maxlen=MAX_DATA_POINTS)
        self.pid_out_data = deque(maxlen=MAX_DATA_POINTS)
        self.motor_L_vel_data = deque(maxlen=MAX_DATA_POINTS)
        self.motor_R_vel_data = deque(maxlen=MAX_DATA_POINTS)
        
        self.current_pitch_deg = 0.0 # For text display
        self.current_pid_out = 0.0

        self.start_time = time.time()
        self.plotting_paused = False

        # Plot visibility
        self.plot_pitch_var = tk.BooleanVar(value=True)
        self.plot_roll_var = tk.BooleanVar(value=False)
        self.plot_yaw_var = tk.BooleanVar(value=False)
        self.plot_pid_out_var = tk.BooleanVar(value=True)
        self.plot_motor_L_vel_var = tk.BooleanVar(value=True)
        self.plot_motor_R_vel_var = tk.BooleanVar(value=True)

        self.auto_y_axis_var = tk.BooleanVar(value=True)
        self.y_min_var = tk.StringVar(value=""); self.y_max_var = tk.StringVar(value="")
        self.manual_y_min = None; self.manual_y_max = None

        # Balance PID tuning variables
        self.balance_p_var = tk.DoubleVar(value=0.8) # Match ESP32 initial
        self.balance_i_var = tk.DoubleVar(value=0.2)
        self.balance_d_var = tk.DoubleVar(value=0.5)
        self.balance_setpoint_var = tk.DoubleVar(value=0.0)

        # Live data display
        self.pitch_label_var = tk.StringVar(value="Pitch: ---°")
        self.pid_out_label_var = tk.StringVar(value="PID Out: ---")
        
        self.create_widgets()
        self.update_serial_ports()
        self.update_gui_state()
        self.schedule_updates()

    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1); self.root.rowconfigure(0, weight=1)

        left_panel = ttk.Frame(main_frame); left_panel.grid(row=1, column=0, sticky="nsew", pady=5, padx=(0,5))
        
        conn_frame = ttk.LabelFrame(main_frame, text="Serial Connection", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5)
        conn_frame.columnconfigure(1, weight=1)
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.com_port_var = tk.StringVar()
        self.com_port_combo = ttk.Combobox(conn_frame, textvariable=self.com_port_var, state="readonly", width=15); self.com_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.refresh_button = ttk.Button(conn_frame, text="Refresh", command=self.update_serial_ports, width=8); self.refresh_button.grid(row=0, column=2, padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect_serial, width=10); self.connect_button.grid(row=1, column=0, padx=5, pady=5)
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_serial, width=10); self.disconnect_button.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        self.status_label_var = tk.StringVar(value="Status: Disconnected"); ttk.Label(conn_frame, textvariable=self.status_label_var).grid(row=1, column=2, columnspan=2, padx=5, pady=5, sticky="w")

        # --- Balancing Control Frame ---
        balance_ctrl_frame = ttk.LabelFrame(left_panel, text="Balancing Control", padding="10")
        balance_ctrl_frame.grid(row=0, column=0, sticky="new", pady=5)
        
        self.enable_balance_button = ttk.Button(balance_ctrl_frame, text="ENABLE Balancing", command=self.enable_balancing)
        self.enable_balance_button.pack(pady=2, fill=tk.X)
        self.disable_balance_button = ttk.Button(balance_ctrl_frame, text="DISABLE Balancing", command=self.disable_balancing)
        self.disable_balance_button.pack(pady=2, fill=tk.X)

        ttk.Label(balance_ctrl_frame, text="--- Balance PID ---").pack(pady=(10,2))
        def create_balance_pid_row(parent, label, var, from_, to_):
            row_frame = ttk.Frame(parent); row_frame.pack(fill=tk.X, pady=1)
            ttk.Label(row_frame, text=label, width=10).pack(side=tk.LEFT)
            ttk.Scale(row_frame, from_=from_, to=to_, orient=tk.HORIZONTAL, variable=var).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
            ttk.Entry(row_frame, textvariable=var, width=7).pack(side=tk.LEFT, padx=2)
        
        create_balance_pid_row(balance_ctrl_frame, "Setpoint:", self.balance_setpoint_var, -10.0, 10.0)
        create_balance_pid_row(balance_ctrl_frame, "Kp:", self.balance_p_var, 0.0, 10.0) # Adjust ranges as needed
        create_balance_pid_row(balance_ctrl_frame, "Ki:", self.balance_i_var, 0.0, 5.0)
        create_balance_pid_row(balance_ctrl_frame, "Kd:", self.balance_d_var, 0.0, 5.0)
        
        self.send_balance_pid_button = ttk.Button(balance_ctrl_frame, text="Send Balance PID/Setpoint", command=self.send_balance_params)
        self.send_balance_pid_button.pack(pady=5, fill=tk.X)

        # --- Live Data Display ---
        live_data_frame = ttk.LabelFrame(left_panel, text="Live Data", padding="10")
        live_data_frame.grid(row=1, column=0, sticky="new", pady=5)
        ttk.Label(live_data_frame, textvariable=self.pitch_label_var).pack(anchor="w")
        ttk.Label(live_data_frame, textvariable=self.pid_out_label_var).pack(anchor="w")


        # --- Plot Area ---
        plot_area_frame = ttk.Frame(main_frame); plot_area_frame.grid(row=1, column=1, rowspan=1, sticky="nsew", pady=5, padx=(5,0))
        main_frame.columnconfigure(1, weight=1); main_frame.rowconfigure(1, weight=1)
        plot_frame = ttk.LabelFrame(plot_area_frame, text="Real-time Data", padding="10"); plot_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 5), dpi=100)
        self.ax = self.fig.add_subplot(111); self.ax.set_xlabel("Time (s)"); self.ax.set_ylabel("Value")
        self.line_pitch, = self.ax.plot([], [], 'g-', label='Pitch (°)')
        self.line_pid_out, = self.ax.plot([], [], 'c-', label='PID Out')
        self.line_motor_L_vel, = self.ax.plot([], [], 'b-', label='L Vel (rad/s)')
        self.line_motor_R_vel, = self.ax.plot([], [], 'r--', label='R Vel (rad/s)')
        self.line_roll,  = self.ax.plot([], [], 'm:', label='Roll (°)') # Optional plots
        self.line_yaw,   = self.ax.plot([], [], 'k:', label='Yaw (°)')  # Optional plots
        self.ax_legend = self.ax.legend(loc='upper left', fontsize='small')
        self.ax.grid(True); self.fig.tight_layout(pad=0.5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame); self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame); toolbar.update(); toolbar.pack(side=tk.BOTTOM, fill=tk.X)

        plot_controls_frame = ttk.Frame(plot_frame); plot_controls_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(5,0))
        cb_frame1 = ttk.Frame(plot_controls_frame); cb_frame1.pack(fill=tk.X)
        self.pitch_cb = ttk.Checkbutton(cb_frame1, text="Pitch", variable=self.plot_pitch_var, command=self.update_plot_visibility); self.pitch_cb.pack(side=tk.LEFT, padx=2)
        self.pid_out_cb = ttk.Checkbutton(cb_frame1, text="PID Out", variable=self.plot_pid_out_var, command=self.update_plot_visibility); self.pid_out_cb.pack(side=tk.LEFT, padx=2)
        self.motor_L_vel_cb = ttk.Checkbutton(cb_frame1, text="L Vel", variable=self.plot_motor_L_vel_var, command=self.update_plot_visibility); self.motor_L_vel_cb.pack(side=tk.LEFT, padx=2)
        self.motor_R_vel_cb = ttk.Checkbutton(cb_frame1, text="R Vel", variable=self.plot_motor_R_vel_var, command=self.update_plot_visibility); self.motor_R_vel_cb.pack(side=tk.LEFT, padx=2)
        self.roll_cb = ttk.Checkbutton(cb_frame1, text="Roll", variable=self.plot_roll_var, command=self.update_plot_visibility); self.roll_cb.pack(side=tk.LEFT, padx=2)
        self.yaw_cb = ttk.Checkbutton(cb_frame1, text="Yaw", variable=self.plot_yaw_var, command=self.update_plot_visibility); self.yaw_cb.pack(side=tk.LEFT, padx=2)
        
        btn_frame = ttk.Frame(plot_controls_frame); btn_frame.pack(fill=tk.X, pady=(5,0))
        self.pause_button = ttk.Button(btn_frame, text="Pause", command=self.pause_plot); self.pause_button.pack(side=tk.LEFT, padx=2)
        self.resume_button = ttk.Button(btn_frame, text="Resume", command=self.resume_plot); self.resume_button.pack(side=tk.LEFT, padx=2)
        self.clear_button = ttk.Button(btn_frame, text="Clear", command=self.clear_plot); self.clear_button.pack(side=tk.LEFT, padx=2)

        yaxis_frame = ttk.Frame(plot_controls_frame); yaxis_frame.pack(fill=tk.X, pady=(5,0))
        self.auto_y_check = ttk.Checkbutton(yaxis_frame, text="Auto Y", variable=self.auto_y_axis_var, command=self.toggle_y_axis_mode); self.auto_y_check.pack(side=tk.LEFT, padx=5)
        ttk.Label(yaxis_frame, text="YMin:").pack(side=tk.LEFT, padx=(5,0))
        self.y_min_entry = ttk.Entry(yaxis_frame, textvariable=self.y_min_var, width=6); self.y_min_entry.pack(side=tk.LEFT, padx=(0,2))
        ttk.Label(yaxis_frame, text="YMax:").pack(side=tk.LEFT, padx=(5,0))
        self.y_max_entry = ttk.Entry(yaxis_frame, textvariable=self.y_max_var, width=6); self.y_max_entry.pack(side=tk.LEFT, padx=(0,2))
        self.apply_yrange_button = ttk.Button(yaxis_frame, text="Set Y", command=self.apply_manual_y_range, width=6); self.apply_yrange_button.pack(side=tk.LEFT, padx=2)

        self.update_plot_visibility() # Initial call
        self.toggle_y_axis_mode(initial=True)

    def enable_balancing(self):
        self._send_command("ENABLE")
        # self.balancing_active_gui = True # Update based on ESP confirmation
        # self.update_gui_state()

    def disable_balancing(self):
        self._send_command("DISABLE")
        # self.balancing_active_gui = False # Update based on ESP confirmation
        # self.update_gui_state()

    def send_balance_params(self):
        p = self.balance_p_var.get()
        i = self.balance_i_var.get()
        d = self.balance_d_var.get()
        sp = self.balance_setpoint_var.get()
        # Send P, I, D together
        pid_cmd = f"P={p:.3f} I={i:.3f} D={d:.3f}"
        self._send_command(pid_cmd)
        # Send setpoint separately or combine if ESP code handles it
        time.sleep(0.05) # Small delay if sending separate commands
        sp_cmd = f"SETPOINT={sp:.2f}"
        self._send_command(sp_cmd)

    # ... (validate_and_update, update_serial_ports, connect_serial, disconnect_serial are similar, check states) ...
    def validate_and_update(self, tk_var, entry_widget): # For PID entries
        current_value_str = entry_widget.get()
        try: tk_var.set(float(current_value_str))
        except ValueError:
            entry_widget.delete(0, tk.END); entry_widget.insert(0, f"{tk_var.get():.3f}") #Format
            print(f"Invalid input '{current_value_str}', reverted.")
        self.root.focus()

    def update_serial_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.com_port_combo['values'] = ports
        if ports and self.com_port_var.get() not in ports: self.com_port_var.set(ports[0])
        elif not ports: self.com_port_var.set("")
        self.update_gui_state()

    def connect_serial(self):
        port_name = self.com_port_var.get()
        if not port_name: messagebox.showerror("Error", "No COM port selected."); return
        try:
            if self.serial_port and self.serial_port.is_open: self.disconnect_serial(); time.sleep(0.2)
            print(f"Attempting to connect to {port_name}...")
            self.serial_port = serial.Serial(port_name, baudrate=115200, timeout=SERIAL_TIMEOUT)
            self.is_connected = True; self.balancing_active_gui = False # Reset on connect
            self.status_label_var.set(f"Status: Connected to {port_name}. Balancing Inactive.")
            print(f"Connected successfully.")
            self.start_time = time.time(); self.clear_data()
            self.auto_y_axis_var.set(True); self.manual_y_min = None; self.manual_y_max = None
            self.y_min_var.set(""); self.y_max_var.set("")
            self.stop_serial_thread.clear()
            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True); self.serial_thread.start()
            self.process_serial_queue()
        except serial.SerialException as e:
            self.is_connected = False; self.balancing_active_gui = False
            self.status_label_var.set("Status: Connection Failed")
            messagebox.showerror("Connection Error", f"Failed to connect to {port_name}:\n{e}")
            if self.serial_port: self.serial_port.close(); self.serial_port = None
        finally: self.update_gui_state()

    def disconnect_serial(self):
        print("Disconnecting..."); self.stop_serial_thread.set()
        if self.is_connected and self.serial_port and self.serial_port.is_open:
             self._send_command("DISABLE") # Try to tell ESP to disable balancing
             time.sleep(0.1) # Give it a moment

        if self.serial_port and self.serial_port.is_open:
            try: self.serial_port.close(); print("Serial port closed.")
            except Exception as e: print(f"Error closing serial port: {e}")
        if self.serial_thread and self.serial_thread.is_alive():
            print("Waiting for serial thread to join..."); self.serial_thread.join(timeout=1.0)
            if self.serial_thread.is_alive(): print("Warning: Serial thread did not exit cleanly.")
        self.serial_port = None; self.serial_thread = None
        self.is_connected = False; self.balancing_active_gui = False
        self.status_label_var.set("Status: Disconnected")
        self.auto_y_axis_var.set(True); self.manual_y_min = None; self.manual_y_max = None
        self.y_min_var.set(""); self.y_max_var.set("")
        while not self.serial_queue.empty():
            try: self.serial_queue.get_nowait()
            except queue.Empty: break
            except Exception as e_q: print(f"Error clearing serial queue: {e_q}"); break
        self.update_gui_state(); print("Disconnected state updated.")


    def update_gui_state(self):
        is_conn = self.is_connected
        conn_s = tk.NORMAL if is_conn else tk.DISABLED
        disconn_s = tk.DISABLED if is_conn else tk.NORMAL

        self.connect_button.config(state=disconn_s)
        self.disconnect_button.config(state=conn_s)
        self.com_port_combo.config(state=disconn_s if self.com_port_combo['values'] else tk.DISABLED)
        self.refresh_button.config(state=disconn_s)

        # Balancing controls
        balance_on_s = tk.NORMAL if is_conn and self.balancing_active_gui else tk.DISABLED
        balance_off_s = tk.NORMAL if is_conn and not self.balancing_active_gui else tk.DISABLED
        if not is_conn: balance_on_s = tk.DISABLED; balance_off_s = tk.DISABLED
        
        self.enable_balance_button.config(state=balance_off_s)
        self.disable_balance_button.config(state=balance_on_s)
        self.send_balance_pid_button.config(state=conn_s) # Allow sending PID when connected, ESP handles if active or not

        # Plot controls
        self.pause_button.config(state=conn_s if not self.plotting_paused else tk.DISABLED)
        self.resume_button.config(state=conn_s if self.plotting_paused else tk.DISABLED)
        self.clear_button.config(state=conn_s)
        
        plot_cb_widgets = [ # Check if these widget attributes exist before configuring
            getattr(self, 'pitch_cb', None), getattr(self, 'pid_out_cb', None),
            getattr(self, 'motor_L_vel_cb', None), getattr(self, 'motor_R_vel_cb', None),
            getattr(self, 'roll_cb', None), getattr(self, 'yaw_cb', None)
        ]
        for cb_widget in plot_cb_widgets:
            if cb_widget: cb_widget.config(state=conn_s)

        self.auto_y_check.config(state=conn_s)
        manual_y_s = tk.NORMAL if is_conn and not self.auto_y_axis_var.get() else tk.DISABLED
        self.y_min_entry.config(state=manual_y_s); self.y_max_entry.config(state=manual_y_s)
        self.apply_yrange_button.config(state=manual_y_s)


    def read_serial_data(self):
        print("SRT: Started"); buffer = ""
        while not self.stop_serial_thread.is_set():
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    if self.is_connected: self.serial_queue.put("SERIAL_ERROR"); print("SRT: Port N/A")
                    break
                if self.serial_port.in_waiting > 0:
                    try: buffer += self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='replace')
                    except Exception as e: print(f"SRT: Read/Decode err: {e}"); buffer = ""; continue
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip(): self.serial_queue.put(line.strip())
                if not self.serial_port.in_waiting: time.sleep(0.005)
            except serial.SerialException as e: print(f"SRT: SerialEx: {e}"); self.serial_queue.put("SERIAL_ERROR"); break
            except Exception as e: print(f"SRT: Unexpected: {e}"); traceback.print_exc(); break
        print("SRT: Exiting")

    def process_serial_queue(self):
        if not self.is_connected: return
        try:
            for _ in range(50): # Max lines per cycle
                if self.serial_queue.empty(): break
                line = self.serial_queue.get_nowait()
                if line == "SERIAL_ERROR":
                    if self.is_connected: messagebox.showerror("Serial Error", "Comm err. Disconnecting."); self.disconnect_serial()
                    return
                
                # Check for CMD_RESP from ESP32
                if line.startswith("CMD_RESP:"):
                    response = line.split("CMD_RESP:",1)[1].strip()
                    print(f"ESP Response: {response}")
                    if "Balancing Enabled" in response:
                        self.balancing_active_gui = True
                        self.status_label_var.set("Status: Conn. Balancing ENABLED.")
                    elif "Balancing Disabled" in response or "Max Tilt" in response:
                        self.balancing_active_gui = False
                        self.status_label_var.set("Status: Conn. Balancing DISABLED.")
                    # Add other CMD_RESP handling if needed (e.g. PID updated)
                    self.update_gui_state()
                    continue # Handled this line

                # General Info (not a CMD_RESP and not telemetry)
                info_prefixes = ("CMD:", "Set ", "Applying ", "Parsed:", "Target Angle", "MOT:", "Init", "Error:", "SimpleFOC", "Encoder", "Driver", "Motor Init", "FOC Init", "PID/Speed", "Unknown", "MPU", "Calibrat", "Gyro", "SPI Bus", "Initializing Motor:", "Setup for Motor:", "  Sensor CS Pin:", "  Initializing sensor", "  >", "  !!! WARNING:", "  Initializing driver", "  Linking sensor and driver", "  Initializing motor library", "  Initializing FOC for", "Motor '", "System Ready", "If motor power", "Check 'Initial Raw Angle'", "Enabling balancing...", "Disabling balancing...", "Balance PID Updated:", "Balance Setpoint Updated:")
                is_info = any(line.startswith(p) for p in info_prefixes)
                if is_info: print(f"ESP Info: {line}"); continue

                # Try to parse as telemetry
                parsed = self.parse_serial_line(line)
                if parsed:
                    self.current_pitch_deg = parsed.get('PITCH', self.current_pitch_deg)
                    self.current_pid_out = parsed.get('PID_OUT', self.current_pid_out)
                    self.update_data(parsed)
                elif not is_info: # Only print if not CMD_RESP, not info, and not telemetry
                    print(f"ESP Unhandled: {line}")
        except queue.Empty: pass
        except Exception as e: print(f"Queue Err: {e}"); traceback.print_exc()
        finally:
            if self.is_connected and self.root.winfo_exists(): self.root.after(20, self.process_serial_queue)

    def parse_serial_line(self, line): # For balancing telemetry
        data = {}
        matches = re.findall(r"([A-Z_0-9]+)=(-?\d+(?:\.\d+(?:[eE][-+]?\d+)?)?)", line)
        for key, value_str in matches:
            try: data[key.upper()] = float(value_str)
            except ValueError: print(f"ParseFail k='{key}',v='{value_str}' in '{line}'")
        expected = ['PITCH','ROLL','YAW','PID_OUT','MTR_L_VEL','MTR_R_VEL']
        return data if any(k in data for k in expected) else None

    def update_data(self, parsed): # For balancing telemetry
        if self.plotting_paused: return
        now = time.time() - self.start_time; added = False
        def app(dq, key, df=0.0): nonlocal added; val=parsed.get(key); dq.append(val if val is not None else (dq[-1] if dq else df)); added = added or (val is not None)
        
        app(self.pitch_data, 'PITCH'); app(self.roll_data, 'ROLL'); app(self.yaw_data, 'YAW')
        app(self.pid_out_data, 'PID_OUT')
        app(self.motor_L_vel_data, 'MTR_L_VEL'); app(self.motor_R_vel_data, 'MTR_R_VEL')
        
        if added or self.time_data or any([self.pitch_data, self.pid_out_data]): # Add time if any relevant data was added
             self.time_data.append(now)
        
        self.pitch_label_var.set(f"Pitch: {self.current_pitch_deg:.2f}°")
        self.pid_out_label_var.set(f"PID Out: {self.current_pid_out:.3f}")


    def schedule_updates(self):
        if self.root.winfo_exists():
            self.plot_update_id = self.root.after(PLOT_UPDATE_MS, self.update_plot)
            # No angle viz scheduler for balancing GUI for now
            # self.viz_update_id = self.root.after(ANGLE_VIZ_UPDATE_MS, self.update_angle_visualization_scheduler)

    def update_plot_visibility(self): # For balancing plots
        vis_map = {
            'line_pitch': self.plot_pitch_var.get(), 'line_pid_out': self.plot_pid_out_var.get(),
            'line_motor_L_vel': self.plot_motor_L_vel_var.get(), 'line_motor_R_vel': self.plot_motor_R_vel_var.get(),
            'line_roll': self.plot_roll_var.get(), 'line_yaw': self.plot_yaw_var.get()
        }
        active_handles, active_labels = [], []
        for attr, is_vis in vis_map.items():
            if hasattr(self, attr):
                line = getattr(self, attr)
                if line: line.set_visible(is_vis)
                if line and is_vis and line.get_label() and not line.get_label().startswith('_'):
                    active_handles.append(line); active_labels.append(line.get_label())
        if self.ax_legend: self.ax_legend.remove()
        self.ax_legend = self.ax.legend(active_handles, active_labels, loc='upper left', fontsize='small') if active_handles else None
        if self.canvas: self.canvas.draw_idle()

    def update_plot(self): # For balancing plots
        if self.root.winfo_exists():
            redraw_required = False
            if self.is_connected and not self.plotting_paused and len(self.time_data) > 0:
                deques = [self.time_data, self.pitch_data, self.pid_out_data, 
                          self.motor_L_vel_data, self.motor_R_vel_data,
                          self.roll_data, self.yaw_data]
                lists = [list(d) for d in deques]

                if not lists[0]: # time_data empty
                    if self.root.winfo_exists(): self.plot_update_id = self.root.after(PLOT_UPDATE_MS, self.update_plot)
                    return
                
                min_len = len(lists[0]) # Use time_data length as reference
                for i in range(1, len(lists)): # Pad other lists if shorter
                    while len(lists[i]) < min_len: lists[i].append(lists[i][-1] if lists[i] else 0.0)
                
                final_min_len = min(len(lst) for lst in lists) # Should be min_len now
                if final_min_len < 1: final_min_len = 1

                plot_data_vals = [lst[-final_min_len:] for lst in lists]
                plot_map = dict(zip(['t', 'p', 'pid', 'lv', 'rv', 'rl', 'yw'], plot_data_vals))
                
                try:
                    if self.plot_pitch_var.get() and plot_map['t']: self.line_pitch.set_data(plot_map['t'], plot_map['p']); redraw_required = True
                    if self.plot_pid_out_var.get() and plot_map['t']: self.line_pid_out.set_data(plot_map['t'], plot_map['pid']); redraw_required = True
                    if self.plot_motor_L_vel_var.get() and plot_map['t']: self.line_motor_L_vel.set_data(plot_map['t'], plot_map['lv']); redraw_required = True
                    if self.plot_motor_R_vel_var.get() and plot_map['t']: self.line_motor_R_vel.set_data(plot_map['t'], plot_map['rv']); redraw_required = True
                    if self.plot_roll_var.get() and plot_map['t']: self.line_roll.set_data(plot_map['t'], plot_map['rl']); redraw_required = True
                    if self.plot_yaw_var.get() and plot_map['t']: self.line_yaw.set_data(plot_map['t'], plot_map['yw']); redraw_required = True

                    if redraw_required:
                        if self.auto_y_axis_var.get(): self.ax.relim(); self.ax.autoscale_view(tight=False, scalex=True, scaley=True)
                        elif self.manual_y_min is not None and self.manual_y_max is not None and plot_map['t']:
                            self.ax.set_ylim(self.manual_y_min, self.manual_y_max)
                            self.ax.set_xlim(plot_map['t'][0], plot_map['t'][-1])
                        elif plot_map['t']: self.ax.relim(); self.ax.autoscale_view(tight=False, scalex=True, scaley=True)
                        if plot_map['t'] or not self.auto_y_axis_var.get(): self.canvas.draw_idle()
                except Exception as e: print(f"Plot Update Err: {e}"); traceback.print_exc()
            
            if self.root.winfo_exists(): self.plot_update_id = self.root.after(PLOT_UPDATE_MS, self.update_plot)
        elif self.root.winfo_exists(): self.plot_update_id = self.root.after(PLOT_UPDATE_MS, self.update_plot)


    def pause_plot(self): self.plotting_paused=True; self.update_gui_state(); print("Plot Paused")
    def resume_plot(self): self.plotting_paused=False; self.update_gui_state(); print("Plot Resumed")

    def clear_data(self): # For balancing data
        for attr_name in dir(self):
            if attr_name.endswith("_data") and isinstance(getattr(self, attr_name), deque): getattr(self, attr_name).clear()
        self.current_pitch_deg=0.0; self.current_pid_out=0.0
        self.pitch_label_var.set("Pitch: ---°"); self.pid_out_label_var.set("PID Out: ---")

    def clear_plot(self): # For balancing plots
        self.clear_data(); self.start_time = time.time()
        plot_lines_attrs = ['line_pitch', 'line_pid_out', 'line_motor_L_vel', 'line_motor_R_vel', 'line_roll', 'line_yaw']
        for attr in plot_lines_attrs:
            if hasattr(self, attr) and getattr(self,attr): getattr(self,attr).set_data([],[])
        self.update_plot_visibility()
        if self.canvas: self.ax.relim(); self.ax.autoscale_view(tight=True); self.canvas.draw_idle()
        print("Plot Cleared.")

    def _send_command(self, cmd):
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open: print("Send Fail: Not connected."); return False
        try: self.serial_port.write((cmd if cmd.endswith('\n') else cmd + '\n').encode('utf-8')); print(f"Sent: {cmd.strip()}"); return True
        except serial.SerialException as e: messagebox.showerror("Serial Error", f"Send Fail: {e}\nDisconnecting."); self.disconnect_serial(); return False
        except Exception as e: messagebox.showerror("Error", f"Send Fail: {e}"); traceback.print_exc(); return False

    # Methods for Y-axis and closing are similar to previous motor tuner
    def toggle_y_axis_mode(self, initial=False):
        is_auto = self.auto_y_axis_var.get()
        if not initial: print(f"Y-Axis: {'Auto' if is_auto else 'Manual'}")
        if is_auto: self.manual_y_min = None; self.manual_y_max = None
        else:
            try: ymin, ymax = self.ax.get_ylim(); self.y_min_var.set(f"{ymin:.2f}"); self.y_max_var.set(f"{ymax:.2f}"); self.apply_manual_y_range(False)
            except Exception: self.y_min_var.set(""); self.y_max_var.set(""); self.manual_y_min=None; self.manual_y_max=None
        self.update_gui_state()

    def apply_manual_y_range(self, show_error=True):
        if self.auto_y_axis_var.get(): return
        try:
            ymin, ymax = float(self.y_min_var.get()), float(self.y_max_var.get())
            if ymin >= ymax:
                if show_error: messagebox.showerror("Input Error", "YMin < YMax req.")
                self.manual_y_min=None; self.manual_y_max=None; return
            self.manual_y_min, self.manual_y_max = ymin, ymax; print(f"Manual Y: {ymin:.2f} to {ymax:.2f}")
            if self.ax and self.canvas: self.ax.set_ylim(ymin, ymax); self.ax.autoscale_view(False,True,False); self.canvas.draw_idle()
        except ValueError:
            if show_error: messagebox.showerror("Input Error", "Invalid Y Min/Max.")
            self.manual_y_min=None; self.manual_y_max=None
        except Exception as e:
            if show_error: messagebox.showerror("Error", f"Y Range apply error: {e}"); traceback.print_exc()
            self.manual_y_min=None; self.manual_y_max=None

    def on_closing(self):
        print("Closing application..."); self.plotting_paused = True
        if hasattr(self, 'plot_update_id') and self.plot_update_id:
            try: self.root.after_cancel(self.plot_update_id)
            except Exception: pass
        # No viz_update_id in this version
        if self.is_connected:
            print("Sending disable balancing cmd..."); self.disable_balancing(); time.sleep(0.15)
            self.disconnect_serial()
        print("Destroying window."); self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = BalanceTunerApp(root) # Use new class name
    root.mainloop()