# IboSoft EFIS Display Software

# Libraries
import re                       # Internal
from datetime import datetime   # Internal
import configparser             # Internal
import serial                   # pyserial, external
import logging                  # Internal
import time                     # Internal
import math                     # Internal
import pygame                   # External
import sys                      # Internal
import tkinter as tk            # Internal
import threading                # Internal

# --------------------
# Icons


# --------------------
# Serial Port

ser = None
serialExceptionDelayTime = 2    # s
dataTimeoutThr = 0.1            # s
dataLowRateThr = 100            # ms
dataTimeout = True              # For determining if data is timeout

def start_serial_port():
    try:
        ser = serial.Serial(serialPortNum, serialBaudRate)
        logging.info(f"Seri port {serialBaudRate} başarıyla açıldı.")
        return ser
    except Exception as e:
        logging.error(f"Seri port açılamadı: {e}")
        time.sleep(serialExceptionDelayTime)
        return None
        
# --------------------

# Functions
def take_sign(value):
    if value < 0:
        return -1
    else:
        return 1

def convert_bool(value):
    try:
        return bool(int(value))
    except ValueError:
        return None
def convert_float(value):
    try:
        return float(value)
    except ValueError:
        return None
def convert_int(value):
    try:
        return int(value)
    except ValueError:
        return None

# Dereceyi radyana çevirme fonksiyonu
def degrees_to_radians(degrees):
    return degrees * math.pi / 180

# Arc çizme fonksiyonu
def draw_arc(surface, color, center, radius, start_angle, end_angle, thickness):
    # Convert angles to radians
    start_angle_rad = math.radians(start_angle)
    end_angle_rad = math.radians(end_angle)

    # Calculate the number of segments needed to approximate the thickness
    num_segments = int(thickness)

    # Draw each segment of the arc
    for i in range(num_segments):
        outer_radius = radius + i
        inner_radius = radius - thickness + i
        outer_rect = pygame.Rect(center[0] - outer_radius, center[1] - outer_radius, outer_radius * 2, outer_radius * 2)
        inner_rect = pygame.Rect(center[0] - inner_radius, center[1] - inner_radius, inner_radius * 2, inner_radius * 2)
        pygame.draw.arc(surface, color, outer_rect, start_angle_rad, end_angle_rad, 1)
        pygame.draw.arc(surface, color, inner_rect, start_angle_rad, end_angle_rad, 1)

# İbre çizme fonksiyonu
def draw_hand(surface, color, center, radius, angle, thickness):
    angle_radians = degrees_to_radians(angle)
    end_pos = (center[0] + radius * math.cos(angle_radians), center[1] - radius * math.sin(angle_radians))
    pygame.draw.line(surface, color, center, end_pos, thickness)

# Çentik çizme fonksiyonu
def draw_ticks_in(surface, color, center, radius, start_angle, end_angle, tick_count, tick_length, thickness):
    angle_interval = (end_angle - start_angle) / (tick_count - 1)
    for i in range(tick_count):
        angle = start_angle + i * angle_interval
        angle_radians = degrees_to_radians(angle)
        start_pos = (center[0] + radius * math.cos(angle_radians), center[1] - radius * math.sin(angle_radians))
        end_pos = (center[0] + (radius - tick_length) * math.cos(angle_radians), center[1] - (radius - tick_length) * math.sin(angle_radians))
        pygame.draw.line(surface, color, start_pos, end_pos, thickness)

def draw_ticks_out(surface, color, center, radius, start_angle, end_angle, tick_count, tick_length, thickness):
    angle_interval = (end_angle - start_angle) / (tick_count - 1)
    for i in range(tick_count):
        angle = start_angle + i * angle_interval
        angle_radians = degrees_to_radians(angle)
        
        # Çentiğin başlayacağı nokta (çemberin üzerinde)
        start_pos = (center[0] + radius * math.cos(angle_radians), center[1] - radius * math.sin(angle_radians))
        
        # Çentiğin biteceği nokta (çemberin dışına doğru)
        end_pos = (center[0] + (radius + tick_length) * math.cos(angle_radians), center[1] - (radius + tick_length) * math.sin(angle_radians))
        
        pygame.draw.line(surface, color, start_pos, end_pos, thickness)

# Draw Arrow
def draw_arrow(screen, color, start, end, thickness):
    pygame.draw.line(screen, color, start, end, thickness)
    
    # Ok ucunu çizmek için yön ve açı hesaplamaları
    rotation = math.atan2(start[1] - end[1], end[0] - start[0]) 
    arrow_length = 10
    arrow_angle = math.pi / 6

    # Ok ucunun sağ tarafı
    right_arrow_x = end[0] + arrow_length * math.cos(rotation + arrow_angle)
    right_arrow_y = end[1] + arrow_length * math.sin(rotation + arrow_angle)
    
    # Ok ucunun sol tarafı
    left_arrow_x = end[0] + arrow_length * math.cos(rotation - arrow_angle)
    left_arrow_y = end[1] + arrow_length * math.sin(rotation - arrow_angle)
    
    pygame.draw.line(screen, color, end, (right_arrow_x, right_arrow_y), thickness)
    pygame.draw.line(screen, color, end, (left_arrow_x, left_arrow_y), thickness)

# --------------------

# Config
config = configparser.ConfigParser()
config.read('config.ini')
    # Serial
serialBaudRate = config.getint('SERIAL', 'BAUD_RATE')
serialPortNum = config.get('SERIAL', 'PORT_NUM')
    # PFD
pfdTick = config.getint('PFD', 'TICK')

# Logging
logging.basicConfig(filename='logs.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

logging.info("--------------------")
logging.info("EFIS Started")
logging.info("Serial Port Num: " + serialPortNum)
logging.info("Serial Baud Rate: " + str(serialBaudRate))
logging.info("--------------------")

print("--------------------")
print("EFIS Started")
print("Serial Port Num: " + serialPortNum)
print("Serial Baud Rate: " + str(serialBaudRate))
print("--------------------")

# Constanst
constStdP = 101325.0	# pascal, ref: ICAO Doc 7488/3
constg0 = 9.80665       # m/s^2, ref: ICAO Doc 7488/3
constHpaToInhg = 0.02952998057228486     # 1 hPa = ? inhg
constmToNM = 0.0005399568034557235       # 1 m = ? NM

# Variables
transition_buffer_trl_ta = False   # For determining if airplane at between transition altitude and level
transition_buffer_ta_trl = False   # For determining if airplane at between transition altitude and level
alt_stg_prev_alt_stg = 101300.0
alt_stg_stby_buffer = False

# Incoming Data
# General
messageInterval = 0
# Settings
set_altStd = False 
set_altStg = 101300.0
# A/G Sensing
ag_onGnd1 = True; ag_onGnd2 = True; ag_onGnd3 = True
# AOA 
aoa_angle = 0.0         # (deg)
# Temperature
temp_TATC = 0.0         # (C)
# IMU
imuStatus = False
imu_ax = 0.0; imu_ay = 0.0; imu_az = 0.0    # (g)
imu_gx = 0.0; imu_gy = 0.0; imu_gz = 0.0    # (deg/s)
# Magnetometer
magStatus = False
# Pressure
pressStatus = False
press_pressPa = 0.0     # (Pa)
# Differential Pressure
diffStatus = False
diff_pressPa = 0.0      # (Pa)
# Derived Values
drv_pitch = 0.0         # Pitch (deg)
drv_roll = 0.0          # Roll (deg)
drv_turnRate = 0.0      # Turn Rate (deg/s)
drv_linearAcc = 0.0     # Linear Acceleration (g)
drv_magUncorrHdg = 0.0  # (deg)
drv_magCorrHdg = 0.0    # (deg)
drv_SATC = 0.0          # SAT (C)
drv_pressAltFt = 0.0    # Pressure Alt (ft)
drv_baroVspdFpm = 0.0   # Vertical Speed (fpm)
drv_indAltFt = 0.0      # Indicated Alt (ft)
drv_kias = 0.0          # KIAS (kts)
drv_ktas = 0.0          # KTAS (kts)
drv_mach = 0.0          # Mach (Mach)
drv_kcas = 0.0          # KCAS (kts)
# Internal Variables

# --------------------
class SharedData:
    def __init__(self):
        # Menu
        self.menu_pfd_altStgUnit = True  # True: hPa, False: inHg
        self.menu_pfd_altStgHpa = 1013
        self.menu_pfd_altStgInHg = 29.92
        self.menu_pfd_altStgStd = False  # True: STD
        self.menu_pfd_ta = 10000
        self.menu_pfd_trl = 100
        self.menu_pfd_magTru = False  # True: TRU, False: MAG
        self.menu_pfd_magCorr = True  # True: Corrected, False: Uncorrected
        self.menu_pfd_magVar = 0.0
        self.menu_pfd_resetG = False  # Reset G Peaks

        self.pfdGPeakMin = 99.9
        self.pfdGPeakMax = -99.9

        self.lock = threading.Lock()  # You can use threading.Lock() here if needed
# --------------------

# Menu
class App(tk.Tk):
    

    def __init__(self, shared_data):
        super().__init__()

        self.shared_data = shared_data

        self.menu_pfd_altStgUnit = tk.BooleanVar(value=shared_data.menu_pfd_altStgUnit)
        self.menu_pfd_altStgHpa = tk.IntVar(value=shared_data.menu_pfd_altStgHpa)
        self.menu_pfd_altStgInHg = tk.DoubleVar(value=shared_data.menu_pfd_altStgInHg)
        self.menu_pfd_altStgStd = tk.BooleanVar(value=shared_data.menu_pfd_altStgStd)
        self.menu_pfd_ta = tk.IntVar(value=shared_data.menu_pfd_ta)
        self.menu_pfd_trl = tk.IntVar(value=shared_data.menu_pfd_trl)
        self.menu_pfd_magTru = tk.BooleanVar(value=shared_data.menu_pfd_magTru)
        self.menu_pfd_magCorr = tk.BooleanVar(value=shared_data.menu_pfd_magCorr)
        self.menu_pfd_magVar = tk.DoubleVar(value=shared_data.menu_pfd_magVar)
        self.menu_pfd_resetG = tk.BooleanVar(value=shared_data.menu_pfd_resetG)

        self.title("Control Display Unit")
        self.geometry("400x500")
        self.iconbitmap("images/cdu.ico")

        self.create_widgets()

        self.show_main_menu()

    def create_widgets(self):
        # Ana çerçeve
        self.main_frame = tk.Frame(self)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # Yan menü çerçevesi
        self.side_menu = tk.Frame(self.main_frame, width=100, bg="grey")
        self.side_menu.pack(side=tk.LEFT, fill=tk.Y)

        self.side_menu_buttons()

        # İçerik çerçevesi
        self.content_frame = tk.Frame(self.main_frame, bg="white")
        self.content_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    def side_menu_buttons(self):
        # Ana menü butonları
        self.main_menu_button = tk.Button(self.side_menu, text="CDU Main Menu", command=self.show_main_menu, font=('Arial', 10, 'bold'))
        self.main_menu_button.pack(pady=10)

        self.efis_button = tk.Button(self.side_menu, text="EFIS Controls", command=self.show_efis_menu, font=('Arial', 10, 'bold'))
        self.efis_button.pack(pady=10)

    def show_main_menu(self):
        self.clear_content_frame()
        
        self.label = tk.Label(self.content_frame, text="CDU Main Menu", bg="white", font=('Arial', 10, 'bold'))
        self.label.pack(pady=10)

        self.efis_button_main = tk.Button(self.content_frame, text="EFIS Controls", command=self.show_efis_menu, font=('Arial', 10, 'bold'))
        self.efis_button_main.pack(pady=10)

    def show_efis_menu(self):
        self.clear_content_frame()
        
        self.label = tk.Label(self.content_frame, text="EFIS - Main Menu", bg="white", font=('Arial', 10, 'bold'))
        self.label.pack(pady=10)

        self.altimeter_settings_button = tk.Button(self.content_frame, text="Altimeter Settings", command=self.show_altimeter_menu, font=('Arial', 10, 'bold'))
        self.altimeter_settings_button.pack(pady=10)

        self.minimums_button = tk.Button(self.content_frame, text="Minimums Settings", command=self.show_minimums_menu, font=('Arial', 10, 'bold'))
        self.minimums_button.pack(pady=10)

        self.mag_settings_button = tk.Button(self.content_frame, text="Compass Settings", command=self.show_mag_settings_menu, font=('Arial', 10, 'bold'))
        self.mag_settings_button.pack(pady=10)

        # G Meter Button
        self.g_menu_button = tk.Button(self.content_frame, text="G Meter", command=self.show_g_meter_menu, font=('Arial', 10, 'bold'))
        self.g_menu_button.pack(pady=10)

        # Seperating line
        self.seperator = tk.Frame(self.content_frame, height=2, bg="black")
        self.seperator.pack(fill=tk.X, pady=10)

        # Prev Page Butonu
        self.prev_button = tk.Button(self.content_frame, text="Prev Page", command=self.show_main_menu, font=('Arial', 10, 'bold'))
        self.prev_button.pack(pady=5)

    def show_altimeter_menu(self):
        self.clear_content_frame()

        self.label = tk.Label(self.content_frame, text="EFIS - Altimeter Settings", bg="white", font=('Arial', 10, 'bold'))
        self.label.pack(pady=10)

        # STD seçici
        self.std_button = tk.Button(self.content_frame, text="STD", command=self.toggle_std, font=('Arial', 10, 'bold'), fg="black")
        self.std_button.pack(pady=5)

        # hPa / inHg seçici
        self.unit_button = tk.Button(self.content_frame, text="hPa / inHg", command=self.toggle_unit, font=('Arial', 10, 'bold'), fg="black")
        self.unit_button.pack(pady=5)

        # Altimeter Setting
        self.altimeter_label = tk.Label(self.content_frame, text="Altimeter Setting", bg="white")
        self.altimeter_label.pack(pady=5)
        self.update_altimeter_label()   # Reload yapınca etiket geri hPa'ya dönmesin diye
        
        self.altimeter_entry = tk.Entry(self.content_frame)
        self.altimeter_entry.pack(pady=5)
        self.update_altimeter_entry()

        # Altimeter Settings Send Button
        self.save_button = tk.Button(self.content_frame, text="Send", command=self.save_altimeter_settings, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

        # Seperating line
        self.seperator = tk.Frame(self.content_frame, height=2, bg="black")
        self.seperator.pack(fill=tk.X, pady=10)

        # Transition Altitude
        self.ta_label = tk.Label(self.content_frame, text="Transition Altitude (ft)", bg="white")
        self.ta_label.pack(pady=5)
        
        self.ta_entry = tk.Entry(self.content_frame)
        self.ta_entry.pack(pady=5)
        self.ta_entry.insert(0, self.menu_pfd_ta.get())

        # Transition Level
        self.trl_label = tk.Label(self.content_frame, text="Transition Level (FL)", bg="white")
        self.trl_label.pack(pady=5)

        self.trl_entry = tk.Entry(self.content_frame)
        self.trl_entry.pack(pady=5)
        self.trl_entry.insert(0, self.menu_pfd_trl.get())

        # TA - TRL Enter Button
        self.save_button = tk.Button(self.content_frame, text="Enter", command=self.save_ta_trl_settings, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

        # Seperating line
        self.seperator = tk.Frame(self.content_frame, height=2, bg="black")
        self.seperator.pack(fill=tk.X, pady=10)

        # Reload Butonu
        self.save_button = tk.Button(self.content_frame, text="Reload", command=self.show_altimeter_menu, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

        # Prev Page Butonu
        self.save_button = tk.Button(self.content_frame, text="Prev Page", command=self.show_efis_menu, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

    def show_minimums_menu(self):
        self.clear_content_frame()

        self.label = tk.Label(self.content_frame, text="EFIS - Minimums Settings", bg="white", font=('Arial', 10, 'bold'))
        self.label.pack(pady=10)
        # Minimums settings content can be added here

        # Seperating line
        self.seperator = tk.Frame(self.content_frame, height=2, bg="black")
        self.seperator.pack(fill=tk.X, pady=10)

        # Reload Butonu
        self.save_button = tk.Button(self.content_frame, text="Reload", command=self.show_minimums_menu, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

        # Prev Page Butonu
        self.save_button = tk.Button(self.content_frame, text="Prev Page", command=self.show_efis_menu, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

    def show_mag_settings_menu(self):
        self.clear_content_frame()

        self.label = tk.Label(self.content_frame, text="EFIS - Mag Settings", bg="white", font=('Arial', 10, 'bold'))
        self.label.pack(pady=10)

        # MAG TRU Button
        self.tru_button = tk.Button(self.content_frame, text="MAG / TRU", command=self.toggle_mag_tru, font=('Arial', 10, 'bold'))
        self.tru_button.pack(pady=5)

        # Magnetic Variation Entry
        self.mag_var_label = tk.Label(self.content_frame, text="Magnetic Variation (° W+ E-)", bg="white")
        self.mag_var_label.pack(pady=5)

        self.mag_var_entry = tk.Entry(self.content_frame)
        self.mag_var_entry.pack(pady=5)
        self.mag_var_entry.insert(0, self.menu_pfd_magVar.get())

        # Save Button
        self.save_button = tk.Button(self.content_frame, text="Save", command=self.save_mag_var, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

        # Corr Button
        self.mag_corr_label = tk.Label(self.content_frame, text="Tilt Correction On / Off", bg="white")
        self.mag_corr_label.pack(pady=5)

        self.mag_corr_button = tk.Button(self.content_frame, text="Change", command=self.toggle_mag_corr, font=('Arial', 10, 'bold'))
        self.mag_corr_button.pack(pady=5)

        # Seperating line
        self.seperator = tk.Frame(self.content_frame, height=2, bg="black")
        self.seperator.pack(fill=tk.X, pady=10)

        # Reload Butonu
        self.save_button = tk.Button(self.content_frame, text="Reload", command=self.show_mag_settings_menu, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

        # Prev Page Button
        self.prev_button = tk.Button(self.content_frame, text="Prev Page", command=self.show_efis_menu, font=('Arial', 10, 'bold'))
        self.prev_button.pack(pady=5)

    def save_mag_var(self):
        try:
            mag_var_value = float(self.mag_var_entry.get())
            if -90.0 <= mag_var_value <= 90.0:
                self.shared_data.menu_pfd_magVar = mag_var_value
                self.menu_pfd_magVar.set(mag_var_value)
            else:
                print("Invalid Magnetic Variation value!")
        except ValueError:
            print("Invalid Magnetic Variation value!")

    def show_g_meter_menu(self):
        def update_values():
            # G Peak Max and Min
            self.g_peaks_label.config(text=f"Max: {self.shared_data.pfdGPeakMax:.1f} G - Min: {self.shared_data.pfdGPeakMin:.1f} G")
            self.after(100, update_values)  # ms

        self.clear_content_frame()

        self.label = tk.Label(self.content_frame, text="EFIS - G Meter", bg="white", font=('Arial', 10, 'bold'))
        self.label.pack(pady=10)

        # G Peak Max and Min
        self.g_peaks_label = tk.Label(self.content_frame, text=f"Max: {self.shared_data.pfdGPeakMax:.1f} G - Min: {self.shared_data.pfdGPeakMin:.1f} G", bg="white", font=('Arial', 10, 'bold'))
        self.g_peaks_label.pack(pady=10)

        self.g_reset_button = tk.Button(self.content_frame, text="Reset Peaks", command=self.reset_g, font=('Arial', 10, 'bold'))
        self.g_reset_button.pack(pady=10)

        # Seperating line
        self.seperator = tk.Frame(self.content_frame, height=2, bg="black")
        self.seperator.pack(fill=tk.X, pady=10)

        # Prev Page Button
        self.prev_button = tk.Button(self.content_frame, text="Prev Page", command=self.show_efis_menu, font=('Arial', 10, 'bold'))
        self.prev_button.pack(pady=5)

        # Değerlerin canlı olarak güncellenmesi için döngü
        update_values()

    def reset_g(self):
        self.shared_data.menu_pfd_resetG = True
        self.menu_pfd_resetG.set(True)

    def update_altimeter_label(self):
        current_unit = self.menu_pfd_altStgUnit.get()
        if current_unit:
            self.altimeter_label.config(text="Altimeter Setting (hPa)")
        else:
            self.altimeter_label.config(text="Altimeter Setting (inHg)")

    def toggle_unit(self):
        current_unit = self.menu_pfd_altStgUnit.get()
        self.menu_pfd_altStgUnit.set(not current_unit)
        self.update_altimeter_label()
        self.update_altimeter_entry()
        # Değişikliği paylaşılan veriye de yansıt
        self.shared_data.menu_pfd_altStgUnit = not current_unit

    def toggle_std(self):
        current_std = self.menu_pfd_altStgStd.get()
        self.menu_pfd_altStgStd.set(not current_std)
        self.update_altimeter_entry()
        # Değişikliği paylaşılan veriye de yansıt
        self.shared_data.menu_pfd_altStgStd = not current_std

    def toggle_mag_tru(self):
        current_mag_tru = self.menu_pfd_magTru.get()
        self.menu_pfd_magTru.set(not current_mag_tru)
        # Değişikliği paylaşılan veriye de yansıt
        self.shared_data.menu_pfd_magTru = not current_mag_tru

    def toggle_mag_corr(self):
        current_mag_corr = self.menu_pfd_magCorr.get()
        self.menu_pfd_magCorr.set(not current_mag_corr)
        # Değişikliği paylaşılan veriye de yansıt
        self.shared_data.menu_pfd_magCorr = not current_mag_corr

    def update_altimeter_entry(self):
        current_unit = self.menu_pfd_altStgUnit.get()
        if current_unit:
            self.altimeter_entry.delete(0, tk.END)
            self.altimeter_entry.insert(0, self.menu_pfd_altStgHpa.get())
        else:
            self.altimeter_entry.delete(0, tk.END)
            self.altimeter_entry.insert(0, self.menu_pfd_altStgInHg.get())

    def save_altimeter_settings(self):
        if True:
            if self.menu_pfd_altStgUnit.get():  # hPa
                try:
                    value = int(self.altimeter_entry.get())
                    if 940 <= value <= 1050:
                        self.shared_data.menu_pfd_altStgHpa = round(value)
                        self.shared_data.menu_pfd_altStgInHg = round(value*constHpaToInhg, 2)
                        self.menu_pfd_altStgHpa.set(round(value))
                        self.menu_pfd_altStgInHg.set(round(value*constHpaToInhg, 2))
                    else:
                        print("Invalid hPa value!")
                except ValueError:
                    print("Invalid hPa value!")
            else:  # inHg
                try:
                    value = float(self.altimeter_entry.get())
                    if 27.50 <= value <= 31.50:
                        self.shared_data.menu_pfd_altStgInHg = round(value, 2)
                        self.shared_data.menu_pfd_altStgHpa = round(value/constHpaToInhg)
                        self.menu_pfd_altStgInHg.set(round(value, 2))
                        self.menu_pfd_altStgHpa.set(round(value/constHpaToInhg))
                    else:
                        print("Invalid inHg value!")
                except ValueError:
                    print("Invalid inHg value!")

    def save_ta_trl_settings(self):
        try:
            ta_value = int(self.ta_entry.get())
            if 100 <= ta_value <= 99999:
                self.shared_data.menu_pfd_ta = ta_value
                self.menu_pfd_ta.set(ta_value)
            else:
                print("Invalid TA value!")
        except ValueError:
            print("Invalid TA value!")

        try:
            trl_value = int(self.trl_entry.get())
            if 1 <= trl_value <= 999:
                self.shared_data.menu_pfd_trl = trl_value
                self.menu_pfd_trl.set(trl_value)
            else:
                print("Invalid TRL value!")
        except ValueError:
            print("Invalid TRL value!")

    

    def clear_content_frame(self):
        for widget in self.content_frame.winfo_children():
            widget.destroy()

# --------------------

# Tkinter uygulamasını ayrı bir iş parçacığında çalıştır
shared_data = SharedData()

gui_thread = threading.Thread(target=lambda: App(shared_data).mainloop())
gui_thread.daemon = True
gui_thread.start()

# --------------------
# Display
SCREEN_WIDTH = 858
SCREEN_HEIGHT = 857

pygame.display.set_icon(pygame.image.load("images/pfd.ico"))

pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Primary Flight Display")

clock = pygame.time.Clock()

# PFD Background
pfdBackground = pygame.image.load("pfd_symbology/pfd_background.png")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BOEING_GRAY = (104, 104, 104)
BOEING_MAGENTA = (255, 0, 204)
BOEING_CYAN = (0, 255, 255)
BOEING_GREEN = (0, 255, 0)
BOEING_AMBER = (255, 179, 0)
BOEING_RED = (252, 0, 0)

# Flags
    # Positions related to top left corner
pfd_flag_alt = pygame.image.load("pfd_flags/pfd_flag_alt.png")
pfd_flag_alt_pos = (666, 373)
pfd_flag_att = pygame.image.load("pfd_flags/pfd_flag_att.png")
pfd_flag_att_pos = (361, 350)
pfd_flag_att_border = pygame.image.load("pfd_flags/pfd_flag_att_border.png")
pfd_flag_att_border_pos = (0, 0)
pfd_flag_hdg = pygame.image.load("pfd_flags/pfd_flag_hdg.png")
pfd_flag_hdg_pos = (358, 815)
pfd_flag_no_v_spd = pygame.image.load("pfd_flags/pfd_flag_no_v_spd.png")
pfd_flag_no_v_spd_pos = (141, 265)
pfd_flag_spd = pygame.image.load("pfd_flags/pfd_flag_spd.png")
pfd_flag_spd_pos = (88,373)
pfd_flag_vert = pygame.image.load("pfd_flags/pfd_flag_vert.png")
pfd_flag_vert_pos = (789, 359)
pfd_flag_data_rate = pygame.image.load("pfd_flags/pfd_flag_data_rate.png")
pfd_flag_data_rate_pos = (30, 770)

# Fonts
pfdSpdFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 24)
pfdHdgFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 12)
pfdVspdFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 14)

pfdSpdFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 20)
pfdSpdTapeFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 16)

pfdMachFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 20)

pfdAltFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 18)
pfdAltTapeFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 14)

pfdAltStgFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 14)
pfdAltStgUnitFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 12)
pfdAltStdFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 20)
pfdAltStgStbyFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 12)

pfdCompass_font_small = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 12)
pfdCompass_font_large = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 16)
pfdCompass_status_text_font = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 10)

pfdAoaFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 10)

pfdGFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 14)

pfdDataTimeoutFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 24)
pfdDataLowRateFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 16)

# Attitude Indicator
pfd_att_img = pygame.image.load("pfd_symbology/pfd_att.png")
pfd_att_rect = pfd_att_img.get_rect()

pfd_att_roll_pointer_img = pygame.image.load("pfd_symbology/pfd_att_roll_pointer.png")
pfd_att_roll_pointer_amber_img = pygame.image.load("pfd_symbology/pfd_att_roll_pointer_amber.png")

pfd_att_slipskid_white_img = pygame.image.load("pfd_symbology/pfd_att_slipskid_white.png")
pfd_att_slipskid_white_filled_img = pygame.image.load("pfd_symbology/pfd_att_slipskid_white_filled.png")
pfd_att_slipskid_amber_img = pygame.image.load("pfd_symbology/pfd_att_slipskid_amber.png")
pfd_att_slipskid_amber_filled_img = pygame.image.load("pfd_symbology/pfd_att_slipskid_amber_filled.png")

pfd_att_split_axis_pointer = pygame.image.load("pfd_symbology/pfd_att_split_axis_pointer.png")
pfd_att_roll_scale = pygame.image.load("pfd_symbology/pfd_att_roll_scale.png")

# Vertical Speed Background
pfd_vspd_background = pygame.image.load("pfd_symbology/pfd_vspd_background.png")

# Pointers
pfd_spd_pointer = pygame.image.load("pfd_symbology/pfd_spd_pointer.png")
pfd_alt_pointer = pygame.image.load("pfd_symbology/pfd_alt_pointer.png")
pfd_compass_pointer = pygame.image.load("pfd_symbology/pfd_compass_pointer.png")

# Variables
    # Atitude Indicator
att_ctr_x = 388
att_ctr_y = 427
pitch_offset = 8.8              # Pixels per degree
bank_amber_threshold = 35       # At or more
slipskid_offset = 250           # Pixels per g
slipskid_fill_threshold = 0.2   # g
    # Attitude Border
border_corner1 = (180, 190)     # Top left corner
border_corner2 = (600, 640)     # Bottom right corner
    # Speed
spd_min = 30
spd_max = 220
spd_pointer_y = 427     # px
spd_line_x_left = 128   # px
spd_line_x_right = 152  # px
spd_line_width = 3      # px
spd_div = 50            # px
spd_div_kts = 10        # kts
spd_kts_to_px = 10/50   # 0.2
spd_lapse = 250         # px, full scale 500
spd_text_x = 118        # px, aligned to right
spd_text_y_offset = 1   # -px
pfd_spd_pointer_pos = (39,385)
pfdSpdTextPos = (47, 409)
    # Mach Indicator
mach_transition = 100       # kts
pfdMachTextPos = (71, 770)
    # Altitude
alt_min = -2000
alt_max = 36000
alt_pointer_y = 427     # px
alt_line_x_left = 625   # px
alt_line_x_right = 649  # px
alt_line_width = 3      # px
alt_div = 75            # px
alt_div_ft = 100        # ft
alt_ft_to_px = 100/75   # 1.33...
alt_lapse = 300         # px, full scale 600
alt_text_x = 653        # px
alt_text_y_offset = -12 # -px
pfd_alt_pointer_pos = (647,385)
pfdAltTextPos = (671, 410)
    # Verical Speed Line
vspd_line_ctr_x_pos = 857       # px
vspd_line_ctr_y_pos = 427       # px
vspd_line_tie_x_pos = 778       # px
vspd_line_width = 5             # px
vspd_fpmPerPxTo1000 = 12.19     # fpm per px between 0 - 1000
vspd_fpmPerPxTo2000 = 16.66     # fpm per px between 1000 - 2000
vspd_fpmPerPxTo6000 = 100.0     # fpm per px between 2000 - 6000
    # Compass
pfdCompass_radius = 257             # Pusula yarıçapı      
pfdCompass_center_x = 390           # Pusula merkezinin x konumu
pfdCompass_center_y = 968           # Pusula merkezinin y konumu
pfdCompass_short_tick_length = 15   # Kısa çizgi uzunluğu
pfdCompass_long_tick_length = 25    # Uzun çizgi uzunluğu
pfdCompass_degree_line_thickness = 4  # Pusula derece çizgilerinin kalınlığı
pfdCompass_pointer_pos = (374,690)
pfdCompass_status_text_pos = (446, 810)
    # Rate of Turn Indicator
rot_arc_center_x = 390      # px
rot_arc_center_y = 968      # px
rot_arc_radius = 257        # px
rot_arc_back_thickness = 3  # px
rot_arc_front_thickness = 4 # px
rot_tick_thickness = 2      # px
rot_tick_long_length = 10   # px
rot_tick_short_length = 6   # px
rot_arc_limit = 40          # deg, on screen
rot_scale_factor = 2
    # Speed Trend Arrow
accel_arrowX = 128          # px
accel_arrowCtrY = 427       # px
accel_arrowLimYUp = 318     # px
accel_arrowLimYDown = 319   # px
accel_arrowDeathZone = 10   # px
accel_arrowThickness = 4    # px
accel_factor = 2000
    # Angle of Attack Indicator
pfdAoaTextPos = (513,152)
aoa_indicator_pos = (555, 147)
aoa_arc_radius = 42
aoa_arc_start_angle = -90
aoa_arc_end_angle = 135
aoa_thickness = 3
aoa_needle_thickness = 4
aoa_tick_count = 6
aoa_tick_length = 8
aoa_min = -90
aoa_max = 135
aoa_scale_factor = 4.5
    # Vertical G Indicator
pfdGTextPos = (240,135)
g_indicator_pos_x = 221
g_indicator_pos_y = 147
g_arc_radius = 42
g_arc_start_angle = 45
g_arc_end_angle = 315
g_thickness = 3
g_needle_thickness = 4
g_tick_count = 5
g_tick_length = 8
g_min = -90
g_max = 135
g_scale_factor = 67.5
g_peak_tick_length = 8
pfd_g_peak_max = -99.9
pfd_g_peak_min = +99.9
    # Error Messages
pfdDataTimeoutPos = (SCREEN_WIDTH/2, 20)  # Center referenced

# Performance and Limitations

# --------------------

# Main Loop
while True:
    with shared_data.lock:  # Shared variables between threads

        # Display
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        # Background
        screen.fill(BLACK)

        # Attitude Indicator
        if imuStatus:
                # Attitude Image
                # Center image
            pfd_att_rect = pfd_att_img.get_rect(center=(att_ctr_x, att_ctr_y))
                # Rotate image
            pfd_rotated_img = pygame.transform.rotate(pfd_att_img, drv_roll)
            pfd_rotated_rect = pfd_rotated_img.get_rect(center=pfd_att_rect.center)
                # Displace image according to rotaton
            if -90 <= drv_pitch and drv_pitch < 90:
                pfd_rotated_rect.x += round(math.cos(math.radians(90-drv_roll)) * pitch_offset * drv_pitch)
                pfd_rotated_rect.y += round(math.sin(math.radians(90-drv_roll)) * pitch_offset * drv_pitch)
            elif 90 <= drv_pitch and drv_pitch < 180:
                pfd_rotated_rect.x += round(math.cos(math.radians(90-drv_roll)) * pitch_offset * (drv_pitch-180))
                pfd_rotated_rect.y += round(math.sin(math.radians(90-drv_roll)) * pitch_offset * (drv_pitch-180))
            else:
                pfd_rotated_rect.x += round(math.cos(math.radians(90-drv_roll)) * pitch_offset * (drv_pitch+180))
                pfd_rotated_rect.y += round(math.sin(math.radians(90-drv_roll)) * pitch_offset * (drv_pitch+180))
            # Draw att image
            screen.blit(pfd_rotated_img, pfd_rotated_rect)
                
                # Roll Pointer
            if abs(drv_roll) < bank_amber_threshold:
                # Center pointer image
                pfd_att_roll_pointer_rect = pfd_att_roll_pointer_img.get_rect(center=(att_ctr_x, att_ctr_y))
                # Rotate pointer image
                pfd_att_roll_pointer_rotated_img = pygame.transform.rotate(pfd_att_roll_pointer_img, drv_roll)
                pfd_att_roll_pointer_rotated_rect = pfd_att_roll_pointer_rotated_img.get_rect(center=pfd_att_rect.center)
                # Draw pointer image
                screen.blit(pfd_att_roll_pointer_rotated_img, pfd_att_roll_pointer_rotated_rect)
            else:
                # Center pointer image
                pfd_att_roll_pointer_amber_rect = pfd_att_roll_pointer_amber_img.get_rect(center=(att_ctr_x, att_ctr_y))
                # Rotate pointer image
                pfd_att_roll_pointer_amber_rotated_img = pygame.transform.rotate(pfd_att_roll_pointer_amber_img, drv_roll)
                pfd_att_roll_pointer_amber_rotated_rect = pfd_att_roll_pointer_amber_rotated_img.get_rect(center=pfd_att_rect.center)
                # Draw pointer image
                screen.blit(pfd_att_roll_pointer_amber_rotated_img, pfd_att_roll_pointer_amber_rotated_rect)   
            
            # Slip/Skid Indicator
            if abs(drv_roll) < bank_amber_threshold:
                if abs(imu_ax) <= slipskid_fill_threshold:
                        # Center image
                    pfd_att_slipskid_white_rect = pfd_att_slipskid_white_img.get_rect(center=(att_ctr_x, att_ctr_y))
                        # Rotate image
                    pfd_att_slipskid_white_rotated_img = pygame.transform.rotate(pfd_att_slipskid_white_img, drv_roll)
                    pfd_att_slipskid_white_rotated_rect = pfd_att_slipskid_white_rotated_img.get_rect(center=pfd_att_slipskid_white_rect.center)
                        # Displace image according to rotaton
                    pfd_att_slipskid_white_rotated_rect.x += round((imu_ax*slipskid_offset*math.cos(math.radians(drv_roll))))
                    pfd_att_slipskid_white_rotated_rect.y -= round((imu_ax*slipskid_offset*math.sin(math.radians(drv_roll))))
                        # Draw att image
                    screen.blit(pfd_att_slipskid_white_rotated_img, pfd_att_slipskid_white_rotated_rect)
                else:
                        # Center image
                    pfd_att_slipskid_white_filled_rect = pfd_att_slipskid_white_filled_img.get_rect(center=(att_ctr_x, att_ctr_y))
                        # Rotate image
                    pfd_att_slipskid_white_filled_rotated_img = pygame.transform.rotate(pfd_att_slipskid_white_filled_img, drv_roll)
                    pfd_att_slipskid_white_filled_rotated_rect = pfd_att_slipskid_white_filled_rotated_img.get_rect(center=pfd_att_slipskid_white_filled_rect.center)
                        # Displace image according to rotaton
                    pfd_att_slipskid_white_filled_rotated_rect.x += round((take_sign(imu_ax)*slipskid_fill_threshold*slipskid_offset*math.cos(math.radians(drv_roll))))
                    pfd_att_slipskid_white_filled_rotated_rect.y -= round((take_sign(imu_ax)*slipskid_fill_threshold*slipskid_offset*math.sin(math.radians(drv_roll))))
                        # Draw att image
                    screen.blit(pfd_att_slipskid_white_filled_rotated_img, pfd_att_slipskid_white_filled_rotated_rect)
            else:
                if abs(imu_ax) <= slipskid_fill_threshold:
                        # Center image
                    pfd_att_slipskid_amber_rect = pfd_att_slipskid_amber_img.get_rect(center=(att_ctr_x, att_ctr_y))
                        # Rotate image
                    pfd_att_slipskid_amber_rotated_img = pygame.transform.rotate(pfd_att_slipskid_amber_img, drv_roll)
                    pfd_att_slipskid_amber_rotated_rect = pfd_att_slipskid_amber_rotated_img.get_rect(center=pfd_att_slipskid_amber_rect.center)
                        # Displace image according to rotaton
                    pfd_att_slipskid_amber_rotated_rect.x += round((imu_ax*slipskid_offset*math.cos(math.radians(drv_roll))))
                    pfd_att_slipskid_amber_rotated_rect.y -= round((imu_ax*slipskid_offset*math.sin(math.radians(drv_roll))))
                        # Draw att image
                    screen.blit(pfd_att_slipskid_amber_rotated_img, pfd_att_slipskid_amber_rotated_rect)
                else:
                    # Center image
                    pfd_att_slipskid_amber_filled_rect = pfd_att_slipskid_amber_filled_img.get_rect(center=(att_ctr_x, att_ctr_y))
                        # Rotate image
                    pfd_att_slipskid_amber_filled_rotated_img = pygame.transform.rotate(pfd_att_slipskid_amber_filled_img, drv_roll)
                    pfd_att_slipskid_amber_filled_rotated_rect = pfd_att_slipskid_amber_filled_rotated_img.get_rect(center=pfd_att_slipskid_amber_filled_rect.center)
                        # Displace image according to rotaton
                    pfd_att_slipskid_amber_filled_rotated_rect.x += round((take_sign(imu_ax)*slipskid_fill_threshold*slipskid_offset*math.cos(math.radians(drv_roll))))
                    pfd_att_slipskid_amber_filled_rotated_rect.y -= round((take_sign(imu_ax)*slipskid_fill_threshold*slipskid_offset*math.sin(math.radians(drv_roll))))
                        # Draw att image
                    screen.blit(pfd_att_slipskid_amber_filled_rotated_img, pfd_att_slipskid_amber_filled_rotated_rect)

                # Split Axis Pointer
            screen.blit(pfd_att_split_axis_pointer, (0, 0))

                # Roll Scale
            screen.blit(pfd_att_roll_scale, (0, 0))            
                    
        # Attitude Black Border Mask:
                # Kareyi tanımla
        border_left = min(border_corner1[0], border_corner2[0])
        border_right = max(border_corner1[0], border_corner2[0])
        border_top = min(border_corner1[1], border_corner2[1])
        border_bottom = max(border_corner1[1], border_corner2[1])
            # Kare alanı dışındaki bölgeleri siyaha boya
            # Üst bölge
        pygame.draw.rect(screen, (0, 0, 0), (0, 0, SCREEN_WIDTH, border_top))
            # Alt bölge
        pygame.draw.rect(screen, (0, 0, 0), (0, border_bottom, SCREEN_WIDTH, SCREEN_HEIGHT - border_bottom))
            # Sol bölge
        pygame.draw.rect(screen, (0, 0, 0), (0, border_top, border_left, border_bottom - border_top))
            # Sağ bölge
        pygame.draw.rect(screen, (0, 0, 0), (border_right, border_top, SCREEN_WIDTH - border_right, border_bottom - border_top))

        # Speed Tape
        if diffStatus:
            pygame.draw.rect(screen, BOEING_GRAY, pygame.Rect(45, 105, 110, 645))  # Background; x, y, width, height

            spd_tape_value = drv_kias
            if spd_tape_value < spd_min:
                spd_tape_value = spd_min
            if spd_tape_value > spd_max:
                spd_tape_value = spd_max

            if abs(spd_tape_value/spd_kts_to_px) < spd_lapse:
                spd_line_section = 1
            else:
                spd_line_section = math.ceil((abs(spd_tape_value)/spd_kts_to_px-spd_lapse)/(2*spd_lapse))+1
            
            if spd_tape_value < 0:
                spd_line_section = 1
                spd_tape_value = 0

            spd_line_ref_y = round( spd_pointer_y + (spd_tape_value/spd_kts_to_px) - (spd_line_section-1)*2*spd_lapse)
            spd_ref_spd = (spd_line_section-1)*round(2*spd_lapse*spd_kts_to_px, -1)

            if (spd_min <= int(spd_ref_spd-10*spd_div_kts) <= spd_max):
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*10), (spd_line_x_right, spd_line_ref_y+spd_div*10), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-10*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*10)
                screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd-9*spd_div_kts) <= spd_max):  
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*9), (spd_line_x_right, spd_line_ref_y+spd_div*9), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-9*spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*9)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd-8*spd_div_kts) <= spd_max):      
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*8), (spd_line_x_right, spd_line_ref_y+spd_div*8), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-8*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*8)
                screen.blit(spd_text, spd_text_rect)   
            if (spd_min <= int(spd_ref_spd-7*spd_div_kts) <= spd_max):    
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*7), (spd_line_x_right, spd_line_ref_y+spd_div*7), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-7*spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*7)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd-6*spd_div_kts) <= spd_max):
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*6), (spd_line_x_right, spd_line_ref_y+spd_div*6), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-6*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*6)
                screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd-5*spd_div_kts) <= spd_max):  
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*5), (spd_line_x_right, spd_line_ref_y+spd_div*5), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-5*spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*5)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd-4*spd_div_kts) <= spd_max):
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*4), (spd_line_x_right, spd_line_ref_y+spd_div*4), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-4*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*4)
                screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd-3*spd_div_kts) <= spd_max):
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*3), (spd_line_x_right, spd_line_ref_y+spd_div*3), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-3*spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*3)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd-2*spd_div_kts) <= spd_max): 
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div*2), (spd_line_x_right, spd_line_ref_y+spd_div*2), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-2*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div*2)
                screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd-spd_div_kts) <= spd_max):
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y+spd_div), (spd_line_x_right, spd_line_ref_y+spd_div), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd-spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset+spd_div)
                # screen.blit(spd_text, spd_text_rect)
            
            if (spd_min <= int(spd_ref_spd) <= spd_max):
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y), (spd_line_x_right, spd_line_ref_y), spd_line_width)  # Middle Line
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset)
                screen.blit(spd_text, spd_text_rect)

            if (spd_min <= int(spd_ref_spd+spd_div_kts) <= spd_max):
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div), (spd_line_x_right, spd_line_ref_y-spd_div), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+2*spd_div_kts) <= spd_max):
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*2), (spd_line_x_right, spd_line_ref_y-spd_div*2), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+2*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*2)
                screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+3*spd_div_kts) <= spd_max):    
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*3), (spd_line_x_right, spd_line_ref_y-spd_div*3), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+3*spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*3)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+4*spd_div_kts) <= spd_max):   
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*4), (spd_line_x_right, spd_line_ref_y-spd_div*4), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+4*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*4)
                screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+5*spd_div_kts) <= spd_max):    
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*5), (spd_line_x_right, spd_line_ref_y-spd_div*5), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+5*spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*5)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+6*spd_div_kts) <= spd_max):    
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*6), (spd_line_x_right, spd_line_ref_y-spd_div*6), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+6*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*6)
                screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+7*spd_div_kts) <= spd_max):   
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*7), (spd_line_x_right, spd_line_ref_y-spd_div*7), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+7*spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*7)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+8*spd_div_kts) <= spd_max):    
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*8), (spd_line_x_right, spd_line_ref_y-spd_div*8), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+8*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*8)
                screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+9*spd_div_kts) <= spd_max):   
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*9), (spd_line_x_right, spd_line_ref_y-spd_div*9), spd_line_width)
                # spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+9*spd_div_kts)), True, WHITE)
                # spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*9)
                # screen.blit(spd_text, spd_text_rect)
            if (spd_min <= int(spd_ref_spd+10*spd_div_kts) <= spd_max):    
                pygame.draw.line(screen, WHITE, (spd_line_x_left, spd_line_ref_y-spd_div*10), (spd_line_x_right, spd_line_ref_y-spd_div*10), spd_line_width)
                spd_text = pfdSpdTapeFont.render(format(int(spd_ref_spd+10*spd_div_kts)), True, WHITE)
                spd_text_rect = spd_text.get_rect(right=spd_text_x, centery=spd_line_ref_y+spd_text_y_offset-spd_div*10)
                screen.blit(spd_text, spd_text_rect)      

        # Altitude Tape
        if pressStatus:
            pygame.draw.rect(screen, BOEING_GRAY, pygame.Rect(623, 105, 110, 645))  # Background; x, y, width, height

            alt_tape_value = drv_indAltFt
            if alt_tape_value < alt_min:
                alt_tape_value = alt_min
            if alt_tape_value > alt_max:
                alt_tape_value = alt_max

            if abs(alt_tape_value/alt_ft_to_px) < alt_lapse:
                alt_line_section = 1
            else:
                alt_line_section = math.ceil((abs(alt_tape_value)/alt_ft_to_px-alt_lapse)/(2*alt_lapse))+1
            if alt_tape_value/alt_ft_to_px < 0:
                alt_line_section = -alt_line_section

            if alt_line_section >= 0:
                alt_line_ref_y = round( alt_pointer_y + (alt_tape_value/alt_ft_to_px) - (alt_line_section-1)*2*alt_lapse)
                alt_ref_alt = (alt_line_section-1)*round(2*alt_lapse*alt_ft_to_px, -1)
            else:
                alt_line_ref_y = round( alt_pointer_y + (alt_tape_value/alt_ft_to_px) - (alt_line_section+1)*2*alt_lapse)
                alt_ref_alt = (alt_line_section+1)*round(2*alt_lapse*alt_ft_to_px, -1)

            if (alt_min <= int(alt_ref_alt-8*alt_div_ft) <= alt_max):
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*8), (alt_line_x_right, alt_line_ref_y+alt_div*8), alt_line_width)
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-8*alt_div_ft)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*8))
            if (alt_min <= int(alt_ref_alt-7*alt_div_ft) <= alt_max):
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*7), (alt_line_x_right, alt_line_ref_y+alt_div*7), alt_line_width)
                # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-7*alt_div_ft)), True, WHITE)
                # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*7))
            if (alt_min <= int(alt_ref_alt-6*alt_div_ft) <= alt_max):    
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*6), (alt_line_x_right, alt_line_ref_y+alt_div*6), alt_line_width)
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-6*alt_div_ft)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*6))
            if (alt_min <= int(alt_ref_alt-5*alt_div_ft) <= alt_max):    
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*5), (alt_line_x_right, alt_line_ref_y+alt_div*5), alt_line_width)
                # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-5*alt_div_ft)), True, WHITE)
                # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*5))
            if (alt_min <= int(alt_ref_alt-4*alt_div_ft) <= alt_max):
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*4), (alt_line_x_right, alt_line_ref_y+alt_div*4), alt_line_width)
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-4*alt_div_ft)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*4))
            if (alt_min <= int(alt_ref_alt-3*alt_div_ft) <= alt_max):    
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*3), (alt_line_x_right, alt_line_ref_y+alt_div*3), alt_line_width)
                # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-3*alt_div_ft)), True, WHITE)
                # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*3))
            if (alt_min <= int(alt_ref_alt-2*alt_div_ft) <= alt_max):    
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*2), (alt_line_x_right, alt_line_ref_y+alt_div*2), alt_line_width)
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-2*alt_div_ft)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*2))
            if (alt_min <= int(alt_ref_alt-alt_div_ft) <= alt_max):    
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div), (alt_line_x_right, alt_line_ref_y+alt_div), alt_line_width)
                # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-alt_div_ft)), True, WHITE)
                # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div))
                    
            if (alt_min <= int(alt_ref_alt) <= alt_max):    
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y), (alt_line_x_right, alt_line_ref_y), alt_line_width)  # Middle Line
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset))

            if (alt_min <= int(alt_ref_alt+alt_div_ft) <= alt_max):     
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div), (alt_line_x_right, alt_line_ref_y-alt_div), alt_line_width)
                # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+alt_div_ft)), True, WHITE)
                # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div))
            if (alt_min <= int(alt_ref_alt+2*alt_div_ft) <= alt_max):     
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*2), (alt_line_x_right, alt_line_ref_y-alt_div*2), alt_line_width)
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+2*alt_div_ft)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*2))
            if (alt_min <= int(alt_ref_alt+3*alt_div_ft) <= alt_max):     
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*3), (alt_line_x_right, alt_line_ref_y-alt_div*3), alt_line_width)
                # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+3*alt_div_ft)), True, WHITE)
                # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*3))
            if (alt_min <= int(alt_ref_alt+4*alt_div_ft) <= alt_max):    
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*4), (alt_line_x_right, alt_line_ref_y-alt_div*4), alt_line_width)
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+4*alt_div_ft)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*4))
            if (alt_min <= int(alt_ref_alt+5*alt_div_ft) <= alt_max): 
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*5), (alt_line_x_right, alt_line_ref_y-alt_div*5), alt_line_width)
                # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+5*alt_div_ft)), True, WHITE)
                # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*5))
            if (alt_min <= int(alt_ref_alt+6*alt_div_ft) <= alt_max):     
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*6), (alt_line_x_right, alt_line_ref_y-alt_div*6), alt_line_width)
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+6*alt_div_ft)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*6))
            if (alt_min <= int(alt_ref_alt+7*alt_div_ft) <= alt_max):     
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*7), (alt_line_x_right, alt_line_ref_y-alt_div*7), alt_line_width)
                # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+7*alt_div_ft)), True, WHITE)
                # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*7))
            if (alt_min <= int(alt_ref_alt+8*alt_div_ft) <= alt_max):    
                pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*8), (alt_line_x_right, alt_line_ref_y-alt_div*8), alt_line_width)
                alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+8*alt_div_ft)), True, WHITE)
                screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*8))

        # Vertical Speed Line
        if pressStatus:
            screen.blit(pfd_vspd_background, (0, 0))  # Background

            if drv_baroVspdFpm >= 0:
                if drv_baroVspdFpm <= 1000:
                    vspd_line_tie_y_pos = vspd_line_ctr_y_pos - round(drv_baroVspdFpm / vspd_fpmPerPxTo1000)
                elif drv_baroVspdFpm <= 2000:
                    vspd_line_tie_y_pos = vspd_line_ctr_y_pos - round(1000 / vspd_fpmPerPxTo1000 + (drv_baroVspdFpm-1000) / vspd_fpmPerPxTo2000)
                elif drv_baroVspdFpm <= 6000:
                    vspd_line_tie_y_pos = vspd_line_ctr_y_pos - round(1000 / vspd_fpmPerPxTo1000 + 1000 / vspd_fpmPerPxTo2000 + (drv_baroVspdFpm-2000) / vspd_fpmPerPxTo6000)
                else:
                    vspd_line_tie_y_pos = vspd_line_ctr_y_pos - round(1000 / vspd_fpmPerPxTo1000 + 1000 / vspd_fpmPerPxTo2000 + 4000 / vspd_fpmPerPxTo6000)
            else:
                if drv_baroVspdFpm >= -1000:
                    vspd_line_tie_y_pos = vspd_line_ctr_y_pos + round(-drv_baroVspdFpm / vspd_fpmPerPxTo1000)
                elif drv_baroVspdFpm >= -2000:
                    vspd_line_tie_y_pos = vspd_line_ctr_y_pos + round(1000 / vspd_fpmPerPxTo1000 + (-drv_baroVspdFpm-1000) / vspd_fpmPerPxTo2000)
                elif drv_baroVspdFpm >= -6000:
                    vspd_line_tie_y_pos = vspd_line_ctr_y_pos + round(1000 / vspd_fpmPerPxTo1000 + 1000 / vspd_fpmPerPxTo2000 + (-drv_baroVspdFpm-2000) / vspd_fpmPerPxTo6000)
                else:
                    vspd_line_tie_y_pos = vspd_line_ctr_y_pos + round(1000 / vspd_fpmPerPxTo1000 + 1000 / vspd_fpmPerPxTo2000 + 4000 / vspd_fpmPerPxTo6000)
        
            pygame.draw.line(screen, WHITE, (vspd_line_ctr_x_pos,vspd_line_ctr_y_pos), (vspd_line_tie_x_pos,vspd_line_tie_y_pos), vspd_line_width)

        # Speed Trend Arrow
        if imuStatus:
            accel_ArrowTipY = accel_arrowCtrY + round((-drv_linearAcc)*accel_factor)
            
            if (abs(accel_ArrowTipY) >= accel_arrowCtrY+accel_arrowDeathZone) or (abs(accel_ArrowTipY) <= accel_arrowCtrY-accel_arrowDeathZone):
                if accel_ArrowTipY <= accel_arrowCtrY-accel_arrowLimYUp:
                    accel_ArrowTipY = accel_arrowCtrY-accel_arrowLimYUp
                if accel_ArrowTipY >= accel_arrowCtrY+accel_arrowLimYDown:
                    accel_ArrowTipY = accel_arrowCtrY+accel_arrowLimYDown

                draw_arrow(screen, BOEING_GREEN, (accel_arrowX, accel_arrowCtrY), (accel_arrowX, accel_ArrowTipY), accel_arrowThickness)

        # Compass
        if magStatus:
            if not imuStatus:
                shared_data.menu_pfd_magCorr = False
            if shared_data.menu_pfd_magCorr:
                if shared_data.menu_pfd_magTru:
                    compassValue = drv_magCorrHdg + shared_data.menu_pfd_magVar
                    pfdCompass_status_text = pfdCompass_status_text_font.render("TRU", True, BOEING_GREEN)
                else:
                    compassValue = drv_magCorrHdg
                    pfdCompass_status_text = pfdCompass_status_text_font.render("MAG", True, BOEING_GREEN)
            else:
                if shared_data.menu_pfd_magTru:
                    compassValue = drv_magUncorrHdg + shared_data.menu_pfd_magVar
                    pfdCompass_status_text = pfdCompass_status_text_font.render("TRU UNCORR", True, BOEING_AMBER)
                else:
                    compassValue = drv_magUncorrHdg
                    pfdCompass_status_text = pfdCompass_status_text_font.render("MAG UNCORR", True, BOEING_AMBER)
            
                # Kerteriz çemberini çiz
            pygame.draw.circle(screen, BOEING_GRAY, (pfdCompass_center_x, pfdCompass_center_y), pfdCompass_radius, 0)

            text_angle = compassValue + 10    # Kerteriziz 0 noktası sabit olduğu için üst kısımdaki yazıların düz görünmesini sağlamak için. +10 ise aşağıda -10 yapılacağı için ilk yazının düz olması için.
            for degree in range(0, 360, 10):
                rad = math.radians(degree)
                adjusted_angle = rad + math.radians(-compassValue-90)
                x1 = pfdCompass_center_x + (pfdCompass_radius - pfdCompass_short_tick_length) * math.cos(adjusted_angle)
                y1 = pfdCompass_center_y + (pfdCompass_radius - pfdCompass_short_tick_length) * math.sin(adjusted_angle)
                x2 = pfdCompass_center_x + pfdCompass_radius * math.cos(adjusted_angle)
                y2 = pfdCompass_center_y + pfdCompass_radius * math.sin(adjusted_angle)
                if degree % 30 == 0:
                    x1 = pfdCompass_center_x + (pfdCompass_radius - pfdCompass_long_tick_length) * math.cos(adjusted_angle)
                    y1 = pfdCompass_center_y + (pfdCompass_radius - pfdCompass_long_tick_length) * math.sin(adjusted_angle)
                pygame.draw.line(screen, WHITE, (x1, y1), (x2, y2), pfdCompass_degree_line_thickness)

                if degree % 10 == 0:
                    text = str(degree // 10) if degree % 30 != 0 else str(degree // 10)
                    font = pfdCompass_font_large if degree % 30 == 0 else pfdCompass_font_small
                    text_angle = text_angle - 10
                    text_x = pfdCompass_center_x + (pfdCompass_radius - pfdCompass_long_tick_length - 10) * math.cos(adjusted_angle)
                    text_y = pfdCompass_center_y + (pfdCompass_radius - pfdCompass_long_tick_length - 10) * math.sin(adjusted_angle)
                    text_surface = font.render(text, True, WHITE)
                    text_rect = text_surface.get_rect(center=(text_x, text_y))
                    rotated_surface = pygame.transform.rotate(text_surface, text_angle)
                    rotated_rect = rotated_surface.get_rect(center=(text_x, text_y))
                    screen.blit(rotated_surface, rotated_rect.topleft)
            
            screen.blit(pfdCompass_status_text, pfdCompass_status_text_pos)

        # == PFD BACKGROUND ==
        screen.blit(pfdBackground, (0, 0))

        # Rate of Turn Indicator
        if imuStatus:
            rot_value = -drv_turnRate * rot_scale_factor

            if abs(rot_value) > rot_arc_limit:
                if rot_value > 0:
                    rot_value = rot_arc_limit
                else:
                    rot_value = -rot_arc_limit

            draw_arc(screen, WHITE, (rot_arc_center_x, rot_arc_center_y), rot_arc_radius, (90-rot_arc_limit), (90+rot_arc_limit), (rot_arc_back_thickness-2))
            draw_ticks_out(screen, WHITE, (rot_arc_center_x, rot_arc_center_y), rot_arc_radius, (90-20*rot_scale_factor), (90+20*rot_scale_factor), 5, rot_tick_long_length, rot_tick_thickness)
            draw_ticks_out(screen, WHITE, (rot_arc_center_x, rot_arc_center_y), rot_arc_radius, (90-6*rot_scale_factor), (90+6*rot_scale_factor), 9, rot_tick_short_length, rot_tick_thickness)
            draw_ticks_out(screen, WHITE, (rot_arc_center_x, rot_arc_center_y), rot_arc_radius, (90-6*rot_scale_factor), (90+6*rot_scale_factor), 5, rot_tick_long_length, rot_tick_thickness)
            
            if rot_value > 0:
                draw_arc(screen, BOEING_GREEN, (rot_arc_center_x, rot_arc_center_y), rot_arc_radius, (90), (90+rot_value), (rot_arc_front_thickness-2))
            if rot_value < 0:
                draw_arc(screen, BOEING_GREEN, (rot_arc_center_x, rot_arc_center_y), rot_arc_radius, (90+rot_value), (90), (rot_arc_front_thickness-2))
            
        # Compass Pointer
        if magStatus:
            screen.blit(pfd_compass_pointer, pfdCompass_pointer_pos)  

        # Vertical Speed Indicator
        if pressStatus:
            vspd_ind_min_value = 300    # Threshold absolute value to display

            vspd_ind_value = round(drv_baroVspdFpm / 50) * 50
            if vspd_ind_value >= 9999:
                vspd_ind_value = 9999
            if vspd_ind_value <= -9999:
                vspd_ind_value = -9999

            pfdVspd = pfdVspdFont.render(format(vspd_ind_value), True, WHITE)

            if vspd_ind_value >= vspd_ind_min_value:
                screen.blit(pfdVspd, (752, 200))
            if vspd_ind_value <= -vspd_ind_min_value:
                screen.blit(pfdVspd, (752, 635))

        # Speed Indicator
        if diffStatus:
            screen.blit(pfd_spd_pointer, pfd_spd_pointer_pos)
            pfdSpd = pfdSpdFont.render(format(round(spd_tape_value)), True, WHITE)
            screen.blit(pfdSpd, pfdSpdTextPos)

        # Mach Indicator
        if diffStatus & (int(drv_kias) >= mach_transition):
            pfdMachFormatted = "{:.3f}".format(round(drv_mach, 3))
            if pfdMachFormatted.startswith("0."):
                pfdMachFormatted = pfdMachFormatted[1:]
            pfdMachFormattedText = pfdMachFont.render(pfdMachFormatted, True, WHITE)
            screen.blit(pfdMachFormattedText, pfdMachTextPos)
 
        # Altitude Indicator
        if pressStatus:
            screen.blit(pfd_alt_pointer, pfd_alt_pointer_pos)
            pfdAlt = pfdAltFont.render(format(int(round(alt_tape_value, -1))), True, WHITE)
            screen.blit(pfdAlt, pfdAltTextPos)

        # Altimeter Settings
        if pressStatus:
            if set_altStd == True:
                if (int(drv_indAltFt/100) > shared_data.menu_pfd_trl) or transition_buffer_ta_trl:
                    pfdAltStd = pfdAltStdFont.render("STD", True, BOEING_GREEN)
                else:
                    pfdAltStd = pfdAltStdFont.render("STD", True, BOEING_AMBER)
                    transition_buffer_trl_ta = True
                screen.blit(pfdAltStd, (653, 755))

                if (round(set_altStg, 1) != round(alt_stg_prev_alt_stg, 1)):
                    alt_stg_stby_buffer = True

                if alt_stg_stby_buffer:
                    if shared_data.menu_pfd_altStgUnit == True:
                        pfdAltStgStby = pfdAltStgStbyFont.render(f"{round(set_altStg/100)} HPA", True, WHITE)
                    else:
                        pfdAltStgStby = pfdAltStgStbyFont.render("{:.2f} IN.".format(round(set_altStg/100*constHpaToInhg, 2)), True, WHITE)
                    screen.blit(pfdAltStgStby, (646, 785))        
            else:
                alt_stg_stby_buffer = False
                if (int(drv_indAltFt) < shared_data.menu_pfd_ta) or transition_buffer_trl_ta:
                    if shared_data.menu_pfd_altStgUnit == True:
                        pfdAltStg = pfdAltStgFont.render(format(round(set_altStg/100)), True, BOEING_GREEN)
                        pfdAltStgUnit = pfdAltStgUnitFont.render("HPA", True, BOEING_GREEN)
                    else:
                        pfdAltStg = pfdAltStgFont.render("{:.2f}".format(round(set_altStg/100*constHpaToInhg, 2)), True, BOEING_GREEN)
                        pfdAltStgUnit = pfdAltStgUnitFont.render("IN.", True, BOEING_GREEN)
                else:
                    if shared_data.menu_pfd_altStgUnit == True:
                        pfdAltStg = pfdAltStgFont.render(format(round(set_altStg/100)), True, BOEING_AMBER)
                        pfdAltStgUnit = pfdAltStgUnitFont.render("HPA", True, BOEING_AMBER)
                        transition_buffer_ta_trl = True
                    else:
                        pfdAltStg = pfdAltStgFont.render("{:.2f}".format(round(set_altStg/100*constHpaToInhg, 2)), True, BOEING_AMBER)
                        pfdAltStgUnit = pfdAltStgUnitFont.render("IN.", True, BOEING_AMBER)
                        transition_buffer_ta_trl = True
                screen.blit(pfdAltStgUnit, (720, 762))
                screen.blit(pfdAltStg, (638, 760))
            if  int(drv_indAltFt) < shared_data.menu_pfd_ta or int(drv_indAltFt/100) > shared_data.menu_pfd_trl:
                transition_buffer_trl_ta = False
                transition_buffer_ta_trl = False
            alt_stg_prev_alt_stg = set_altStg   

        # Angle of Attack Indicator
        if True:
            aoa_indicator_value = aoa_angle * aoa_scale_factor
            if aoa_indicator_value < aoa_arc_start_angle:
                aoa_indicator_value = aoa_arc_start_angle
            if aoa_indicator_value > aoa_arc_end_angle:
                aoa_indicator_value = aoa_arc_end_angle

            draw_arc(screen, WHITE, aoa_indicator_pos, aoa_arc_radius, aoa_arc_start_angle, aoa_arc_end_angle, aoa_thickness-2)          
            draw_ticks_in(screen, WHITE, aoa_indicator_pos, aoa_arc_radius, aoa_arc_start_angle, aoa_arc_end_angle, aoa_tick_count, aoa_tick_length, aoa_thickness)
            draw_hand(screen, WHITE, aoa_indicator_pos, aoa_arc_radius, aoa_indicator_value, aoa_needle_thickness)
            pfdAoaText = pfdAoaFont.render(format(round(aoa_indicator_value/aoa_scale_factor, 1), '.1f'), True, WHITE)
            screen.blit(pfdAoaText, pfdAoaTextPos)
 
        # Vertical G Indicator
        if imuStatus:
            if imu_ay > shared_data.pfdGPeakMax:
                shared_data.pfdGPeakMax = imu_ay

            if imu_ay < shared_data.pfdGPeakMin:
                shared_data.pfdGPeakMin = imu_ay

            if shared_data.menu_pfd_resetG:
                shared_data.pfdGPeakMax = imu_ay
                shared_data.pfdGPeakMin = imu_ay
                shared_data.menu_pfd_resetG = False

            pfd_g_peak_max_indicator_value = -(shared_data.pfdGPeakMax-1) * g_scale_factor + 180
            if pfd_g_peak_max_indicator_value < g_arc_start_angle:
                pfd_g_peak_max_indicator_value = g_arc_start_angle
            if pfd_g_peak_max_indicator_value > g_arc_end_angle:
                pfd_g_peak_max_indicator_value = g_arc_end_angle

            pfd_g_peak_min_indicator_value = -(shared_data.pfdGPeakMin-1) * g_scale_factor + 180
            if pfd_g_peak_min_indicator_value < g_arc_start_angle:
                pfd_g_peak_min_indicator_value = g_arc_start_angle
            if pfd_g_peak_min_indicator_value > g_arc_end_angle:
                pfd_g_peak_min_indicator_value = g_arc_end_angle

            g_indicator_value = -(imu_ay-1) * g_scale_factor + 180
            if g_indicator_value < g_arc_start_angle:
                g_indicator_value = g_arc_start_angle
            if g_indicator_value > g_arc_end_angle:
                g_indicator_value = g_arc_end_angle     

            pfdG0Text = pfdGFont.render("0", True, WHITE)
            screen.blit(pfdG0Text, (g_indicator_pos_x-10, g_indicator_pos_y+8))
            pfdG2Text = pfdGFont.render("2", True, WHITE)
            screen.blit(pfdG2Text, (g_indicator_pos_x-10, g_indicator_pos_y-30))                  
            draw_ticks_in(screen, WHITE, (g_indicator_pos_x, g_indicator_pos_y), g_arc_radius, g_arc_start_angle, g_arc_end_angle, g_tick_count, g_tick_length, g_thickness)
            draw_ticks_out(screen, BOEING_GREEN, (g_indicator_pos_x, g_indicator_pos_y), g_arc_radius, pfd_g_peak_max_indicator_value, pfd_g_peak_min_indicator_value, 2, g_peak_tick_length, g_thickness)
            draw_arc(screen, WHITE, (g_indicator_pos_x, g_indicator_pos_y), g_arc_radius, g_arc_start_angle, g_arc_end_angle, g_thickness-2)
            draw_hand(screen, WHITE, (g_indicator_pos_x, g_indicator_pos_y), g_arc_radius, g_indicator_value, g_needle_thickness)
            pfdGText = pfdGFont.render(format(round(imu_ay, 1), '.1f'), True, WHITE)
            screen.blit(pfdGText, pfdGTextPos)
                    
        # # Heading [TEST]
        # pfdHdg = pfdHdgFont.render(format(round(drv_magUncorrHdg)), True, WHITE)
        # screen.blit(pfdHdg, (388, 50))

        # Flags:
        if not imuStatus:
            screen.blit(pfd_flag_att_border, pfd_flag_att_border_pos)
            screen.blit(pfd_flag_att, pfd_flag_att_pos) 
        if not magStatus:
            screen.blit(pfd_flag_hdg, pfd_flag_hdg_pos)
        if not pressStatus:
            screen.blit(pfd_flag_alt, pfd_flag_alt_pos)
            screen.blit(pfd_flag_vert, pfd_flag_vert_pos)
        if not diffStatus:
            screen.blit(pfd_flag_spd, pfd_flag_spd_pos)
        if messageInterval > dataLowRateThr:
            screen.blit(pfd_flag_data_rate, pfd_flag_data_rate_pos)

        # Error Messages
        if dataTimeout:
            pfdDataTimeout = pfdDataTimeoutFont.render("DATA TIMEOUT", True, BOEING_RED)    # Yazıyı render et           
            text_rect = pfdDataTimeout.get_rect()               # Yazının boyutlarını al
            text_rect.center = pfdDataTimeoutPos                # Pozisyonu yazının merkezine göre ayarla     
            background_rect = text_rect.inflate(10, 5)          # Arka planın boyutunu yazıya göre biraz daha büyük yap        
            pygame.draw.rect(screen, BLACK, background_rect)    # Arka planı çiz       
            screen.blit(pfdDataTimeout, text_rect)              # Yazıyı çiz

        pygame.display.flip()
        clock.tick(pfdTick)
        # --------------------

        dataTimeout = True
        try:
            # Seri port açık değilse veya port None ise yeniden başlatmayı dene
            if ser is None or not ser.is_open:
                logging.warning("Seri port kapalı. Yeniden açmaya çalışılıyor...")
                ser = start_serial_port()

            if ser and ser.is_open:
                # Seri port üzerinden veri gönderme veya alma işlemleri

                # Process incoming data
                while True:      
                    start_time = time.time()  # Döngü başlangıç zamanını al
                    while True:
                        try:
                            if (time.time() - start_time) > dataTimeoutThr:  # 200 ms'yi kontrol et
                                dataTimeout = True
                                break  # İç döngüden çık
                            incoming_data = ser.readline().decode('ascii').strip()
                            dataTimeout = False
                            break  # Başarılı bir okuma yapıldığında iç döngüden çık
                        except UnicodeDecodeError:
                            continue  # UnicodeDecodeError oluşursa devam et
                    
                    # Veri işleme işaretlerine göre veriyi parçala
                    if incoming_data.startswith('/'):
                        key, value = incoming_data[1:].split('=')
                        if key == 'i':
                            messageInterval = convert_int(value)        
                    elif incoming_data.startswith('!'):
                        key, value = incoming_data[1:].split('=')
                        if key == 'asd':
                            set_altStd = convert_bool(value)
                        elif key == 'atg':
                            set_altStg = convert_float(value)
                    elif incoming_data.startswith('%'):
                        key, value = incoming_data[1:].split('=')
                        if key == 'imu':
                            imuStatus = convert_bool(value)
                        elif key == 'mag':
                            magStatus = convert_bool(value)
                        elif key == 'prs':
                            pressStatus = convert_bool(value)
                        elif key == 'dif':
                            diffStatus = convert_bool(value)
                    elif incoming_data.startswith('$'):
                        key, value = incoming_data[1:].split('=')
                        if key == 'gn1':
                            ag_onGnd1 = convert_bool(value)
                        elif key == 'gn2':
                            ag_onGnd2 = convert_bool(value)
                        elif key == 'gn3':
                            ag_onGnd3 = convert_bool(value)
                        elif key == 'aoa':
                            aoa_angle = convert_float(value)
                        elif key == 'tat':
                            temp_TATC = convert_float(value)
                        elif key == 'ax':
                            imu_ax = convert_float(value)
                        elif key == 'ay':
                            imu_ay = convert_float(value)
                        elif key == 'az':
                            imu_az = convert_float(value)
                        elif key == 'gx':
                            imu_gx = convert_float(value)
                        elif key == 'gy':
                            imu_gy = convert_float(value)
                        elif key == 'gz':
                            imu_gz = convert_float(value)
                        elif key == 'prs':
                            press_pressPa = convert_float(value)
                        elif key == 'dif':
                            diff_pressPa = convert_float(value)
                    elif incoming_data.startswith('&'):
                        key, value = incoming_data[1:].split('=')
                        if key == 'pit':
                            drv_pitch = convert_float(value)
                        elif key == 'rol':
                            drv_roll = convert_float(value)
                        elif key == 'trn':
                            drv_turnRate = convert_float(value)
                        elif key == 'lac':
                            drv_linearAcc = convert_float(value)
                        elif key == 'umh':
                            drv_magUncorrHdg = convert_float(value)
                        elif key == 'cmh':
                            drv_magCorrHdg = convert_float(value)
                        elif key == 'plt':
                            drv_pressAltFt = convert_float(value)
                        elif key == 'ilt':
                            drv_indAltFt = convert_float(value)
                        elif key == 'vsp':
                            drv_baroVspdFpm = convert_float(value)
                        elif key == 'ias':
                            drv_kias = convert_float(value)
                        elif key == 'cas':
                            drv_kcas = convert_float(value)
                        elif key == 'tas':
                            drv_ktas = convert_float(value)
                        elif key == 'mac':
                            drv_mach = convert_float(value)
                        elif key == 'sat':
                            drv_SATC = convert_float(value)

                    # İşlemi bitir
                    elif incoming_data == '+':
                        break

                # Print Incoming Data
                if(False):  # Currently Disabled
                    print("Incoming Data:")
                    print("messageInterval:", messageInterval)
                    print("set_altStd:", set_altStd)
                    print("set_altStg:", set_altStg)
                    print("ag_onGnd1:", ag_onGnd1)
                    print("ag_onGnd2:", ag_onGnd2)
                    print("ag_onGnd3:", ag_onGnd3)
                    print("aoa_angle:", aoa_angle)
                    print("temp_TATC:", temp_TATC)
                    print("imuStatus:", imuStatus)
                    print("imu_ax:", imu_ax)
                    print("imu_ay:", imu_ay)
                    print("imu_az:", imu_az)
                    print("imu_gx:", imu_gx)
                    print("imu_gy:", imu_gy)
                    print("imu_gz:", imu_gz)
                    print("magStatus:", magStatus)
                    print("pressStatus:", pressStatus)
                    print("press_pressPa:", press_pressPa)
                    print("diffStatus:", diffStatus)
                    print("diff_pressPa:", diff_pressPa)
                    print("drv_pitch:", drv_pitch)
                    print("drv_roll:", drv_roll)
                    print("drv_turnRate:", drv_turnRate)
                    print("drv_linearAcc:", drv_linearAcc)
                    print("drv_magUncorrHdg:", drv_magUncorrHdg)
                    print("drv_magCorrHdg:", drv_magCorrHdg)
                    print("drv_pressAltFt:", drv_pressAltFt)
                    print("drv_baroVspdFpm:", drv_baroVspdFpm)
                    print("drv_indAltFt:", drv_indAltFt)
                    print("drv_kias:", drv_kias)
                    print("drv_ktas:", drv_ktas)
                    print("drv_mach:", drv_mach)
                    print("drv_kcas:", drv_kcas)
                    print("drv_SATC:", drv_SATC)

                # --------------------
                # Send data to Serial Port
                ser.write("...\r\n#\r\n".encode('ascii'))

                if set_altStd != shared_data.menu_pfd_altStgStd:
                    ser.write(f"!asd={int(shared_data.menu_pfd_altStgStd)}\r\n".encode('ascii'))

                if shared_data.menu_pfd_altStgUnit == True:
                    if round(set_altStg/100) != shared_data.menu_pfd_altStgHpa:
                        ser.write(f"!atg={float(int(shared_data.menu_pfd_altStgHpa*100))}\r\n".encode('ascii'))
                else:
                    if round(set_altStg/100*constHpaToInhg) != shared_data.menu_pfd_altStgInHg:
                        ser.write(f"!atg={float(int(shared_data.menu_pfd_altStgInHg/constHpaToInhg*100))}\r\n".encode('ascii'))

                ser.write("+\r\n+\r\n".encode('ascii'))
                # --------------------    
                pass

        except Exception as e:
            logging.error(f"Seri port hatası: {e}")
            if ser:
                ser.close()
                ser = None
            time.sleep(serialExceptionDelayTime)


        # --------------------