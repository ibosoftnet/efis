import re
from datetime import datetime
import configparser
import serial

# Config
config = configparser.ConfigParser()
config.read('config.ini')
serialBaudRate = config.getint('SERIAL', 'BAUD_RATE')
serialPortNum = config.get('SERIAL', 'PORT_NUM')

# Serial Port
ser = serial.Serial(serialPortNum, serialBaudRate)
if not ser.is_open:
    ser.open()

# Constanst
constStdP = 101325.0	# pascal, ref: ICAO Doc 7488/3

# Incoming Data
# General
messageInterval = False  
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

while True:
    # Process incoming data
    while True:       
        # Seri porttan gelen veriyi oku
        try:
            incoming_data = ser.readline().decode('ascii').strip()
        except UnicodeDecodeError:
            continue

        # Veri işleme işaretlerine göre veriyi parçala
        if incoming_data.startswith('!'):
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