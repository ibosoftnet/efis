import re
from datetime import datetime
import configparser
import serial
import logging
import time
import math
import pygame
import sys
import tkinter as tk
import threading


# Config
config = configparser.ConfigParser()
config.read('config.ini')
    # Serial
serialBaudRate = config.getint('SERIAL', 'BAUD_RATE')
serialPortNum = config.get('SERIAL', 'PORT_NUM')
    # PFD
pfdTick = config.getint('PFD', 'TICK')



# Logging
logging.basicConfig(filename='status.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

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
hpaToInhg = 0.02952998057228486     # 1 hPa = ? inhg

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
mag_hdg = 0.0           # (deg)
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
drv_SATC = 0.0          # SAT (C)
drv_pressAltFt = 0.0    # Pressure Alt (ft)
drv_baroVspdFpm = 0.0   # Vertical Speed (fpm)
drv_indAltFt = 0.0      # Indicated Alt (ft)
drv_kias = 0.0          # KIAS (kts)
drv_ktas = 0.0          # KTAS (kts)
drv_mach = 0.0          # Mach (Mach)
drv_kcas = 0.0          # KCAS (kts)

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
    
# Serial Port
ser = serial.Serial(serialPortNum, serialBaudRate)
def serialOpen():
    try:
        ser = serial.Serial(serialPortNum, serialBaudRate)
        if not ser.is_open:
            ser.open()
    except serial.SerialException as e:
        logging.error("Serial port error:", str(e))
        print("Serial port error:", str(e))
    
# --------------------
# Shared Data
class SharedData:
    def __init__(self):
        self.menu_pfd_altStgUnit = True  # True: hPa, False: inHg
        self.menu_pfd_altStgHpa = 1013
        self.menu_pfd_altStgInHg = 29.92
        self.menu_pfd_altStgStd = False  # True: STD
        self.menu_pfd_ta = 10000
        self.menu_pfd_trl = 100
        self.lock = threading.Lock()  # You can use threading.Lock() here if needed

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

        self.title("Control Display Unit")
        self.geometry("400x500")

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

        # Prev Page Butonu
        self.save_button = tk.Button(self.content_frame, text="Prev Page", command=self.show_main_menu, font=('Arial', 10, 'bold'))
        self.save_button.pack(pady=5)

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
                        self.shared_data.menu_pfd_altStgInHg = round(value*hpaToInhg, 2)
                        self.menu_pfd_altStgHpa.set(round(value))
                        self.menu_pfd_altStgInHg.set(round(value*hpaToInhg, 2))
                    else:
                        print("Invalid hPa value!")
                except ValueError:
                    print("Invalid hPa value!")
            else:  # inHg
                try:
                    value = float(self.altimeter_entry.get())
                    if 27.50 <= value <= 31.50:
                        self.shared_data.menu_pfd_altStgInHg = round(value, 2)
                        self.shared_data.menu_pfd_altStgHpa = round(value/hpaToInhg)
                        self.menu_pfd_altStgInHg.set(round(value, 2))
                        self.menu_pfd_altStgHpa.set(round(value/hpaToInhg))
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

# Fonts
pfdSpdFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 24)
pfdHdgFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 12)
pfdVspdFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 14)

pfdAltFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 18)
pfdAltTapeFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 14)

pfdAltStgFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 14)
pfdAltStgUnitFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 12)
pfdAltStdFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 20)
pfdAltStgStbyFont = pygame.font.Font('fonts/OCR-B/OCR-B.ttf', 12)

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

# Speed Tape
pfd_spd_tape = pygame.image.load("pfd_symbology/pfd_spd_tape.png")
pfd_spd_tape_rect = pfd_spd_tape.get_rect()

# Vertical Speed Background
pfd_vspd_background = pygame.image.load("pfd_symbology/pfd_vspd_background.png")
    
# --------------------

# Main Loop
while True:
    with shared_data.lock:  # Shared variables between threads
    
        # Process incoming data
        while True:       
            while True:
                try:
                    incoming_data = ser.readline().decode('ascii').strip()
                    break
                except serial.SerialException as e:
                    logging.error("Serial port error:", str(e))
                    print("Serial port error:", str(e))
                    serialOpen()
                except UnicodeDecodeError:
                    continue
                time.sleep(1)


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
                elif key == 'mhd':
                    mag_hdg = convert_float(value)
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
                elif key == 'sat':
                    drv_SATC = convert_float(value)
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

            # İşlemi bitir
            elif incoming_data == '+':
                break

        # Print incoming data
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
        print("mag_hdg:", mag_hdg)
        print("pressStatus:", pressStatus)
        print("press_pressPa:", press_pressPa)
        print("diffStatus:", diffStatus)
        print("diff_pressPa:", diff_pressPa)
        print("drv_pitch:", drv_pitch)
        print("drv_roll:", drv_roll)
        print("drv_turnRate:", drv_turnRate)
        print("drv_SATC:", drv_SATC)
        print("drv_pressAltFt:", drv_pressAltFt)
        print("drv_baroVspdFpm:", drv_baroVspdFpm)
        print("drv_indAltFt:", drv_indAltFt)
        print("drv_kias:", drv_kias)
        print("drv_ktas:", drv_ktas)
        print("drv_mach:", drv_mach)
        print("drv_kcas:", drv_kcas)

        # --------------------
        # Send data to Serial Port
        ser.write("...\r\n#\r\n".encode('ascii'))

        if set_altStd != shared_data.menu_pfd_altStgStd:
            ser.write(f"!asd={int(shared_data.menu_pfd_altStgStd)}\r\n".encode('ascii'))

        if shared_data.menu_pfd_altStgUnit == True:
            if round(set_altStg/100) != shared_data.menu_pfd_altStgHpa:
                ser.write(f"!atg={float(int(shared_data.menu_pfd_altStgHpa*100))}\r\n".encode('ascii'))
        else:
            if round(set_altStg/100*hpaToInhg) != shared_data.menu_pfd_altStgInHg:
                ser.write(f"!atg={float(int(shared_data.menu_pfd_altStgInHg/hpaToInhg*100))}\r\n".encode('ascii'))

        ser.write("+\r\n+\r\n".encode('ascii'))
        # --------------------

        # Display
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        # Background
        screen.fill(BOEING_GRAY)

        # Atitude Indicator
        att_ctr_x = 388
        att_ctr_y = 427
        pitch_offset = 8.8              # Pixels per degree
        bank_amber_threshold = 35       # At or more
        slipskid_offset = 250           # Pixels per g
        slipskid_fill_threshold = 0.2   # g

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
        
        # Speed Tape
        spd_tape_x_ctr = 100    # Location of center of wide on x axis.
        spd_tape_pixelsPerKts = 5
        spd_tape_ctrSpd = 134.2   # Speed of center of tape.
        spd_tape_min = 30
        spd_tape_max = 220

        spd_tape_value = drv_kcas
        if spd_tape_value < spd_tape_min:
            spd_tape_value = spd_tape_min
        if spd_tape_value > spd_tape_max:
            spd_tape_value = spd_tape_max
            
        pfd_spd_tape_rect.center = (spd_tape_x_ctr, round((spd_tape_value*spd_tape_pixelsPerKts)-(spd_tape_ctrSpd*spd_tape_pixelsPerKts)))
        screen.blit(pfd_spd_tape, pfd_spd_tape_rect)

        # Altitude Tape

        pygame.draw.rect(screen, BOEING_GRAY, pygame.Rect(623, 105, 110, 645))  # Background

        alt_min = -2000
        alt_max = 36000

        alt_tape_value = drv_indAltFt
        if alt_tape_value < alt_min:
            alt_tape_value = alt_min
        if alt_tape_value > alt_max:
            alt_tape_value = alt_max


        alt_pointer_y = 427     # Px
        alt_line_x_left = 625   # Px
        alt_line_x_right = 649  # Px
        alt_line_width = 3      # Px
        alt_div = 75            # Px
        alt_div_ft = 100        # ft
        alt_ft_to_px = 100/75   # 1.33...
        alt_lapse = 300         # px, full scale 600
        alt_text_x = 653        # px
        alt_text_y_offset = -12 # -px

        if abs(alt_tape_value/alt_ft_to_px) < alt_lapse:
            alt_line_section = 1
        else:
            alt_line_section = math.ceil((abs(alt_tape_value)/alt_ft_to_px-alt_lapse)/(2*alt_lapse))+1
        if alt_tape_value/alt_ft_to_px < 0:
            alt_line_section = -alt_line_section

        alt_line_section_text = pfdAltFont.render(format(alt_line_section), True, WHITE)
        screen.blit(alt_line_section_text, (700, 100))

        if alt_line_section >= 0:
            alt_line_ref_y = round( alt_pointer_y + (alt_tape_value/alt_ft_to_px) - (alt_line_section-1)*2*alt_lapse)
            alt_ref_alt = (alt_line_section-1)*round(2*alt_lapse*alt_ft_to_px, -1)
        else:
            alt_line_ref_y = round( alt_pointer_y + (alt_tape_value/alt_ft_to_px) - (alt_line_section+1)*2*alt_lapse)
            alt_ref_alt = (alt_line_section+1)*round(2*alt_lapse*alt_ft_to_px, -1)

        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*8), (alt_line_x_right, alt_line_ref_y+alt_div*8), alt_line_width)
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-8*alt_div_ft)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*8))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*7), (alt_line_x_right, alt_line_ref_y+alt_div*7), alt_line_width)
        # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-7*alt_div_ft)), True, WHITE)
        # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*7))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*6), (alt_line_x_right, alt_line_ref_y+alt_div*6), alt_line_width)
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-6*alt_div_ft)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*6))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*5), (alt_line_x_right, alt_line_ref_y+alt_div*5), alt_line_width)
        # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-5*alt_div_ft)), True, WHITE)
        # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*5))

        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*4), (alt_line_x_right, alt_line_ref_y+alt_div*4), alt_line_width)
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-4*alt_div_ft)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*4))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*3), (alt_line_x_right, alt_line_ref_y+alt_div*3), alt_line_width)
        # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-3*alt_div_ft)), True, WHITE)
        # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*3))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div*2), (alt_line_x_right, alt_line_ref_y+alt_div*2), alt_line_width)
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-2*alt_div_ft)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div*2))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y+alt_div), (alt_line_x_right, alt_line_ref_y+alt_div), alt_line_width)
        # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt-alt_div_ft)), True, WHITE)
        # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset+alt_div))
            
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y), (alt_line_x_right, alt_line_ref_y), alt_line_width)  # Middle Line
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset))

        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div), (alt_line_x_right, alt_line_ref_y-alt_div), alt_line_width)
        # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+alt_div_ft)), True, WHITE)
        # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*2), (alt_line_x_right, alt_line_ref_y-alt_div*2), alt_line_width)
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+2*alt_div_ft)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*2))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*3), (alt_line_x_right, alt_line_ref_y-alt_div*3), alt_line_width)
        # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+3*alt_div_ft)), True, WHITE)
        # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*3))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*4), (alt_line_x_right, alt_line_ref_y-alt_div*4), alt_line_width)
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+4*alt_div_ft)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*4))

        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*5), (alt_line_x_right, alt_line_ref_y-alt_div*5), alt_line_width)
        # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+5*alt_div_ft)), True, WHITE)
        # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*5))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*6), (alt_line_x_right, alt_line_ref_y-alt_div*6), alt_line_width)
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+6*alt_div_ft)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*6))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*7), (alt_line_x_right, alt_line_ref_y-alt_div*7), alt_line_width)
        # alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+7*alt_div_ft)), True, WHITE)
        # screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*7))
        pygame.draw.line(screen, WHITE, (alt_line_x_left, alt_line_ref_y-alt_div*8), (alt_line_x_right, alt_line_ref_y-alt_div*8), alt_line_width)
        alt_text = pfdAltTapeFont.render(format(int(alt_ref_alt+8*alt_div_ft)), True, WHITE)
        screen.blit(alt_text, (alt_text_x, alt_line_ref_y+alt_text_y_offset-alt_div*8))
        
        
        # Vertical Speed Line
        vspd_line_ctr_x_pos = 857
        vspd_line_ctr_y_pos = 427
        vspd_line_tie_x_pos = 778
        vspd_line_width = 5
        vspd_fpmPerPxTo1000 = 12.19
        vspd_fpmPerPxTo2000 = 16.66
        vspd_fpmPerPxTo6000 = 100.0

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

        # == PFD Background ==
        screen.blit(pfdBackground, (0, 0))

        # Vertical Speed Indicator
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
        pfdSpd = pfdSpdFont.render(format(round(spd_tape_value)), True, WHITE)
        screen.blit(pfdSpd, (47, 406))

        # Altitude Indicator
        pfdAlt = pfdAltFont.render(format(int(round(alt_tape_value, -1))), True, WHITE)
        screen.blit(pfdAlt, (671, 410))


        # Altimeter Settings

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
                    pfdAltStgStby = pfdAltStgStbyFont.render("{:.2f} IN.".format(round(set_altStg/100*hpaToInhg, 2)), True, WHITE)
                screen.blit(pfdAltStgStby, (646, 785))
            
        else:
            alt_stg_stby_buffer = False
            if (int(drv_indAltFt) < shared_data.menu_pfd_ta) or transition_buffer_trl_ta:
                if shared_data.menu_pfd_altStgUnit == True:
                    pfdAltStg = pfdAltStgFont.render(format(round(set_altStg/100)), True, BOEING_GREEN)
                    pfdAltStgUnit = pfdAltStgUnitFont.render("HPA", True, BOEING_GREEN)
                else:
                    pfdAltStg = pfdAltStgFont.render("{:.2f}".format(round(set_altStg/100*hpaToInhg, 2)), True, BOEING_GREEN)
                    pfdAltStgUnit = pfdAltStgUnitFont.render("IN.", True, BOEING_GREEN)
            else:
                if shared_data.menu_pfd_altStgUnit == True:
                    pfdAltStg = pfdAltStgFont.render(format(round(set_altStg/100)), True, BOEING_AMBER)
                    pfdAltStgUnit = pfdAltStgUnitFont.render("HPA", True, BOEING_AMBER)
                    transition_buffer_ta_trl = True
                else:
                    pfdAltStg = pfdAltStgFont.render("{:.2f}".format(round(set_altStg/100*hpaToInhg, 2)), True, BOEING_AMBER)
                    pfdAltStgUnit = pfdAltStgUnitFont.render("IN.", True, BOEING_AMBER)
                    transition_buffer_ta_trl = True
            screen.blit(pfdAltStgUnit, (720, 762))
            screen.blit(pfdAltStg, (638, 760))
        if  int(drv_indAltFt) < shared_data.menu_pfd_ta or int(drv_indAltFt/100) > shared_data.menu_pfd_trl:
            transition_buffer_trl_ta = False
            transition_buffer_ta_trl = False
        alt_stg_prev_alt_stg = set_altStg

        # Heading
        pfdHdg = pfdHdgFont.render(format(round(mag_hdg)), True, WHITE)
        screen.blit(pfdHdg, (388, 50))


        pygame.display.flip()
        clock.tick(pfdTick)
        # --------------------