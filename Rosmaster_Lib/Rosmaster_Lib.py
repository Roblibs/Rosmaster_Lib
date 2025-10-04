#!/usr/bin/env python3
# coding: utf-8
"""Rosmaster robot control library.

This module exposes the `Rosmaster` class which wraps the serial protocol for
multi‑wheel and Ackermann (R2) robotic platforms. It provides APIs for:

* Background reception & parsing of asynchronous sensor reports
* High‑level motion (discrete state or continuous vx, vy, wz) control
* Direct motor PWM, RGB effects, buzzer
* PWM & bus (UART) servo control, including 6‑axis robotic arm helpers
* PID parameter tuning & querying
* IMU (gyro, accel, mag) / attitude / encoder / battery telemetry
* Flash configuration persistence & factory reset

Only comments were refactored into PEP 257 style docstrings; executable code
remains unchanged. When `debug=True`, additional diagnostic prints appear.
"""

import struct
import time
import serial
import threading


# V3.3.9
class Rosmaster(object):
    """Controller for Rosmaster robotic platforms.

    Parameters
    ----------
    car_type : int, optional
        Numeric platform type (see CARTYPE_* constants). Default 1.
    com : str, optional
        Serial port (e.g. "COM5" on Windows or "/dev/ttyUSB0").
    delay : float, optional
        Inter‑command delay seconds (default 0.002) to avoid MCU overwhelm.
    debug : bool, optional
        If True, prints verbose protocol debug output.
    """
    __uart_state = 0

    def __init__(self, car_type=1, com="/dev/myserial", delay=.002, debug=False):
        # com = "COM30"
        # com="/dev/ttyTHS1"
        # com="/dev/ttyUSB0"
        # com="/dev/ttyAMA0"

        self.ser = serial.Serial(com, 115200)

        self.__delay_time = delay
        self.__debug = debug

        self.__HEAD = 0xFF
        self.__DEVICE_ID = 0xFC
        self.__COMPLEMENT = 257 - self.__DEVICE_ID
        self.__CAR_TYPE = car_type
        self.__CAR_ADJUST = 0x80

        self.FUNC_AUTO_REPORT = 0x01
        self.FUNC_BEEP = 0x02
        self.FUNC_PWM_SERVO = 0x03
        self.FUNC_PWM_SERVO_ALL = 0x04
        self.FUNC_RGB = 0x05
        self.FUNC_RGB_EFFECT = 0x06

        self.FUNC_REPORT_SPEED = 0x0A
        self.FUNC_REPORT_MPU_RAW = 0x0B
        self.FUNC_REPORT_IMU_ATT = 0x0C
        self.FUNC_REPORT_ENCODER = 0x0D
        self.FUNC_REPORT_ICM_RAW = 0x0E
        
        self.FUNC_RESET_STATE = 0x0F

        self.FUNC_MOTOR = 0x10
        self.FUNC_CAR_RUN = 0x11
        self.FUNC_MOTION = 0x12
        self.FUNC_SET_MOTOR_PID = 0x13
        self.FUNC_SET_YAW_PID = 0x14
        self.FUNC_SET_CAR_TYPE = 0x15

        self.FUNC_UART_SERVO = 0x20
        self.FUNC_UART_SERVO_ID = 0x21
        self.FUNC_UART_SERVO_TORQUE = 0x22
        self.FUNC_ARM_CTRL = 0x23
        self.FUNC_ARM_OFFSET = 0x24

        self.FUNC_AKM_DEF_ANGLE = 0x30
        self.FUNC_AKM_STEER_ANGLE = 0x31


        self.FUNC_REQUEST_DATA = 0x50
        self.FUNC_VERSION = 0x51

        self.FUNC_RESET_FLASH = 0xA0

        self.CARTYPE_X3 = 0x01
        self.CARTYPE_X3_PLUS = 0x02
        self.CARTYPE_X1 = 0x04
        self.CARTYPE_R2 = 0x05

        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__gx = 0.0
        self.__gy = 0.0
        self.__gz = 0.0
        self.__mx = 0.0
        self.__my = 0.0
        self.__mz = 0.0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0

        self.__yaw = 0.0
        self.__roll = 0.0
        self.__pitch = 0.0

        self.__encoder_m1 = 0
        self.__encoder_m2 = 0
        self.__encoder_m3 = 0
        self.__encoder_m4 = 0

        self.__read_id = 0
        self.__read_val = 0

        self.__read_arm_ok = 0
        self.__read_arm = [-1, -1, -1, -1, -1, -1]

        self.__version_H = 0
        self.__version_L = 0
        self.__version = 0

        self.__pid_index = 0
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0

        self.__arm_offset_state = 0
        self.__arm_offset_id = 0
        self.__arm_ctrl_enable = True

        self.__battery_voltage = 0

        self.__akm_def_angle = 100
        self.__akm_readed_angle = False
        self.__AKM_SERVO_ID = 0x01

        self.__read_car_type = 0

        if self.__debug:
            print("cmd_delay=" + str(self.__delay_time) + "s")

        if self.ser.isOpen():
            print("Rosmaster Serial Opened! Baudrate=115200")
        else:
            print("Serial Open Failed!")
        # 打开机械臂的扭矩力，避免6号舵机首次插上去读不到角度。
        # Enable the robotic arm torque so that servo #6 can report its angle the first time it is plugged in.
        self.set_uart_servo_torque(1)
        time.sleep(.002)

    def __del__(self):
        """Close the serial port on destruction (best‑effort)."""
        self.ser.close()
        self.__uart_state = 0
        print("serial Close!")

    def __parse_data(self, ext_type, ext_data):
        """Parse one received frame payload by function code.

        Parameters
        ----------
        ext_type : int
            Function/report identifier.
        ext_data : list[int]
            Raw payload bytes (header/checksum removed).
        """
        # print("parse_data:", ext_data, ext_type)
        if ext_type == self.FUNC_REPORT_SPEED:
            # print(ext_data)
            self.__vx = int(struct.unpack('h', bytearray(ext_data[0:2]))[0]) / 1000.0
            self.__vy = int(struct.unpack('h', bytearray(ext_data[2:4]))[0]) / 1000.0
            self.__vz = int(struct.unpack('h', bytearray(ext_data[4:6]))[0]) / 1000.0
            self.__battery_voltage = struct.unpack('B', bytearray(ext_data[6:7]))[0]
    # 解析MPU9250原始陀螺仪、加速度计、磁力计数据
    # Parse raw MPU9250 gyroscope, accelerometer and magnetometer data
        elif ext_type == self.FUNC_REPORT_MPU_RAW:
            # 陀螺仪传感器:±500dps=±500°/s ±32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
            # Gyro sensor range: ±500 dps => raw ±32768. Conversion: (raw/32768*500)*PI/180(rad/s)=raw/3754.9
            gyro_ratio = 1 / 3754.9 # ±500dps
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0]*gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0]*-gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0]*-gyro_ratio
            # 加速度传感器:±2g=±2*9.8m/s^2 ±32768 accel/32768*19.6=accel/1671.84
            # Accelerometer: ±2g => raw ±32768. Conversion: raw/1671.84 (m/s^2)
            accel_ratio = 1 / 1671.84
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0]*accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0]*accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0]*accel_ratio
            # 磁力计传感器
            # Magnetometer raw values (scaling placeholder = 1.0)
            mag_ratio = 1.0
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0]*mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0]*mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0]*mag_ratio
    # 解析ICM20948原始陀螺仪、加速度计、磁力计数据
    # Parse raw ICM20948 gyroscope, accelerometer and magnetometer data
        elif ext_type == self.FUNC_REPORT_ICM_RAW:
            gyro_ratio = 1 / 1000.0
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0]*gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0]*gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0]*gyro_ratio

            accel_ratio = 1 / 1000.0
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0]*accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0]*accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0]*accel_ratio

            mag_ratio = 1 / 1000.0
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0]*mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0]*mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0]*mag_ratio
    # 解析板子的姿态角
    # Parse the attitude angles (roll, pitch, yaw)
        elif ext_type == self.FUNC_REPORT_IMU_ATT:
            self.__roll = struct.unpack('h', bytearray(ext_data[0:2]))[0] / 10000.0
            self.__pitch = struct.unpack('h', bytearray(ext_data[2:4]))[0] / 10000.0
            self.__yaw = struct.unpack('h', bytearray(ext_data[4:6]))[0] / 10000.0
    # 解析四个轮子的编码器数据
    # Parse encoder counts for all four wheels
        elif ext_type == self.FUNC_REPORT_ENCODER:
            self.__encoder_m1 = struct.unpack('i', bytearray(ext_data[0:4]))[0]
            self.__encoder_m2 = struct.unpack('i', bytearray(ext_data[4:8]))[0]
            self.__encoder_m3 = struct.unpack('i', bytearray(ext_data[8:12]))[0]
            self.__encoder_m4 = struct.unpack('i', bytearray(ext_data[12:16]))[0]

        else:
            if ext_type == self.FUNC_UART_SERVO:
                self.__read_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__read_val = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                if self.__debug:
                    print("FUNC_UART_SERVO:", self.__read_id, self.__read_val)

            elif ext_type == self.FUNC_ARM_CTRL:
                self.__read_arm[0] = struct.unpack('h', bytearray(ext_data[0:2]))[0]
                self.__read_arm[1] = struct.unpack('h', bytearray(ext_data[2:4]))[0]
                self.__read_arm[2] = struct.unpack('h', bytearray(ext_data[4:6]))[0]
                self.__read_arm[3] = struct.unpack('h', bytearray(ext_data[6:8]))[0]
                self.__read_arm[4] = struct.unpack('h', bytearray(ext_data[8:10]))[0]
                self.__read_arm[5] = struct.unpack('h', bytearray(ext_data[10:12]))[0]
                self.__read_arm_ok = 1
                if self.__debug:
                    print("FUNC_ARM_CTRL:", self.__read_arm)

            elif ext_type == self.FUNC_VERSION:
                self.__version_H = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__version_L = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                if self.__debug:
                    print("FUNC_VERSION:", self.__version_H, self.__version_L)

            elif ext_type == self.FUNC_SET_MOTOR_PID:
                self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
                self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]
                if self.__debug:
                    print("FUNC_SET_MOTOR_PID:", self.__pid_index, [self.__kp1, self.__ki1, self.__kd1])

            elif ext_type == self.FUNC_SET_YAW_PID:
                self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
                self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
                self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]
                if self.__debug:
                    print("FUNC_SET_YAW_PID:", self.__pid_index, [self.__kp1, self.__ki1, self.__kd1])

            elif ext_type == self.FUNC_ARM_OFFSET:
                self.__arm_offset_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__arm_offset_state = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                if self.__debug:
                    print("FUNC_ARM_OFFSET:", self.__arm_offset_id, self.__arm_offset_state)

            elif ext_type == self.FUNC_AKM_DEF_ANGLE:
                id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__akm_def_angle = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                self.__akm_readed_angle = True
                if self.__debug:
                    print("FUNC_AKM_DEF_ANGLE:", id, self.__akm_def_angle)
            
            elif ext_type == self.FUNC_SET_CAR_TYPE:
                car_type = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__read_car_type = car_type
            
    def __receive_data(self):
        """Continuously read, assemble, verify and parse incoming frames.

        Runs as a daemon thread created by :meth:`create_receive_threading`.
        Flushes the input buffer once, then blocks on serial reads, validating
        simple additive checksums before dispatching payloads to
        :meth:`__parse_data`.
        """
        self.ser.flushInput()
        while True:
            head1 = bytearray(self.ser.read())[0]
            if head1 == self.__HEAD:
                head2 = bytearray(self.ser.read())[0]
                check_sum = 0
                rx_check_num = 0
                if head2 == self.__DEVICE_ID - 1:
                    ext_len = bytearray(self.ser.read())[0]
                    ext_type = bytearray(self.ser.read())[0]
                    ext_data = []
                    check_sum = ext_len + ext_type
                    data_len = ext_len - 2
                    while len(ext_data) < data_len:
                        value = bytearray(self.ser.read())[0]
                        ext_data.append(value)
                        if len(ext_data) == data_len:
                            rx_check_num = value
                        else:
                            check_sum = check_sum + value
                    if check_sum % 256 == rx_check_num:
                        self.__parse_data(ext_type, ext_data)
                    else:
                        if self.__debug:
                            print("check sum error:", ext_len, ext_type, ext_data)

    def __request_data(self, function, param=0):
        """Internal: send a request frame to MCU.

        Parameters
        ----------
        function : int
            Function code whose data is requested.
        param : int, optional
            Optional argument byte (default 0).
        """
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_REQUEST_DATA, int(function) & 0xff, int(param) & 0xff]
        checksum = sum(cmd, self.__COMPLEMENT) & 0xff
        cmd.append(checksum)
        self.ser.write(cmd)
        if self.__debug:
            print("request:", cmd)
        time.sleep(0.002)

    def __arm_convert_value(self, s_id, s_angle):
        """Convert joint angle (deg) to raw pulse value for a given servo id."""
        value = -1
        if s_id == 1:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 2:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 3:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 4:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 5:
            value = int((3700 - 380) * (s_angle - 0) / (270 - 0) + 380)
        elif s_id == 6:
            value = int((3100 - 900) * (s_angle - 0) / (180 - 0) + 900)
        return value

    def __arm_convert_angle(self, s_id, s_value):
        """Inverse of :meth:`__arm_convert_value` returning angle degrees."""
        s_angle = -1
        if s_id == 1:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 2:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 3:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 4:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 5:
            s_angle = int((270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5)
        elif s_id == 6:
            s_angle = int((180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5)
        return s_angle

    def __limit_motor_value(self, value):
        """Clamp raw PWM to [-100,100]; 127 sentinel preserves prior speed."""
        if value == 127:
            return 127
        elif value > 100:
           return 100
        elif value < -100:
            return -100
        else:
            return int(value)

    def create_receive_threading(self):
        """Start the UART receive & parse daemon thread (no-op if started)."""
        try:
            if self.__uart_state == 0:
                name1 = "task_serial_receive"
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                task_receive.setDaemon(True)
                task_receive.start()
                print("----------------create receive threading--------------")
                self.__uart_state = 1
                time.sleep(.05)
        except:
            print('---create_receive_threading error!---')
            pass
    
    def set_auto_report_state(self, enable, forever=False):
        """Enable/disable periodic sensor auto-reporting.

        When enabled the MCU emits four cyclic packets every 10 ms (each
        packet updated every 40 ms). If disabled, some read methods relying
        on cached values may not function. If ``forever`` is True the setting
        is persisted to flash.
        """
        try:
            state1 = 0
            state2 = 0
            if enable:
                state1 = 1
            if forever:
                state2 = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_AUTO_REPORT, state1, state2]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("report:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_auto_report_state error!---')
            pass

    def set_beep(self, on_time):
        """Control buzzer: 0=off, 1=continuous, >=10 duration ms (multiple of 10)."""
        try:
            if on_time < 0:
                print("beep input error!")
                return
            value = bytearray(struct.pack('h', int(on_time)))

            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_BEEP, value[0], value[1]]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("beep:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_beep error!---')
            pass

    def set_pwm_servo(self, servo_id, angle):
        """Set one PWM servo (id 1..4) to angle 0..180 degrees."""
        try:
            if servo_id < 1 or servo_id > 4:
                if self.__debug:
                    print("set_pwm_servo input invalid")
                return
            if angle > 180:
                angle = 180
            elif angle < 0:
                angle = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO, int(servo_id), int(angle)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pwmServo:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_pwm_servo error!---')
            pass

    def set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4):
        """Set up to four PWM servos at once angle_sX=[0, 180] (out-of-range -> 255 ignored)."""
        try:
            if angle_s1 < 0 or angle_s1 > 180:
                angle_s1 = 255
            if angle_s2 < 0 or angle_s2 > 180:
                angle_s2 = 255
            if angle_s3 < 0 or angle_s3 > 180:
                angle_s3 = 255
            if angle_s4 < 0 or angle_s4 > 180:
                angle_s4 = 255
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO_ALL, \
                   int(angle_s1), int(angle_s2), int(angle_s3), int(angle_s4)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("all Servo:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_pwm_servo_all error!---')
            pass
    
    def set_colorful_lamps(self, led_id, red, green, blue):
        """RGB programmable light belt control, can be controlled individually or collectively, before control need to stop THE RGB light effect.
            Led_id =[0, 13], control the CORRESPONDING numbered RGB lights;  Led_id =0xFF, controls all lights.
            Red,green,blue=[0, 255], indicating the RGB value of the color.
        """
        try:
            id = int(led_id) & 0xff
            r = int(red) & 0xff
            g = int(green) & 0xff
            b = int(blue) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB, id, r, g, b]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_colorful_lamps error!---')
            pass

    def set_colorful_effect(self, effect, speed=255, parm=255):
        """
        RGB可编程灯带特效展示。
        Run predefined RGB strip animation effect
        effect=[0, 6]，0：停止灯效，1：流水灯，2：跑马灯，3：呼吸灯，4：渐变灯，5：星光点点，6：电量显示
        speed=[1, 10]，数值越小速度变化越快。
        parm，可不填，作为附加参数。用法1：呼吸灯效果传入[0, 6]可修改呼吸灯颜色。
        RGB programmable light band special effects display.
        Effect =[0, 6], 0: stop light effect, 1: running light, 2: running horse light, 3: breathing light, 4: gradient light, 5: starlight, 6: power display 
        Speed =[1, 10], the smaller the value, the faster the speed changes
        Parm, left blank, as an additional argument.  Usage 1: The color of breathing lamp can be modified by the effect of breathing lamp [0, 6]
        """
        try:
            eff = int(effect) & 0xff
            spe = int(speed) & 0xff
            par = int(parm) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB_EFFECT, eff, spe, par]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb_effect:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_colorful_effect error!---')
            pass


    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        """Raw open-loop PWM control for four motors (speeds -100..100)."""
        try:
            t_speed_a = bytearray(struct.pack('b', self.__limit_motor_value(speed_1)))
            t_speed_b = bytearray(struct.pack('b', self.__limit_motor_value(speed_2)))
            t_speed_c = bytearray(struct.pack('b', self.__limit_motor_value(speed_3)))
            t_speed_d = bytearray(struct.pack('b', self.__limit_motor_value(speed_4)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTOR,
                   t_speed_a[0], t_speed_b[0], t_speed_c[0], t_speed_d[0]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motor:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_motor error!---')
            pass


    def set_car_run(self, state, speed, adjust=False):
        """Discrete movement
        High-level directional movement helper.
        Controls the car to move: forward, backward, left, right, rotate in place, or stop.
        state = 0~7:
          0 = stop (immediate stop)
          1 = forward
          2 = backward
          3 = move left (strafe)
          4 = move right (strafe)
          5 = rotate left (counter‑clockwise)
          6 = rotate right (clockwise)
          7 = park (stop and hold)
        speed = -100 ~ 100 (0 = stop). Positive magnitude increases speed.
        adjust = True enables gyro-assisted heading correction (currently not implemented).
        """
        try:
            car_type = self.__CAR_TYPE
            if adjust:
                car_type = car_type | self.__CAR_ADJUST
            t_speed = bytearray(struct.pack('h', int(speed)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_CAR_RUN, \
                car_type, int(state&0xff), t_speed[0], t_speed[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_run:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_car_run error!---')
            pass

    def set_car_motion(self, v_x, v_y, v_z):
        """Car movement control : Continuous Omnidirectional velocity command (vx, vy m/s, vz rad/s).
        输入范围 input range: 
        X3: v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5, 5]
        X3PLUS: v_x=[-0.7, 0.7], v_y=[-0.7, 0.7], v_z=[-3.2, 3.2]
        R2/R2L: v_x=[-1.8, 1.8], v_y=[-0.045, 0.045], v_z=[-3, 3]
        """
        try:
            vx_parms = bytearray(struct.pack('h', int(v_x*1000)))
            vy_parms = bytearray(struct.pack('h', int(v_y*1000)))
            vz_parms = bytearray(struct.pack('h', int(v_z*1000)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTION, self.__CAR_TYPE, \
                vx_parms[0], vx_parms[1], vy_parms[0], vy_parms[1], vz_parms[0], vz_parms[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motion:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_car_motion error!---')
            pass


    def set_pid_param(self, kp, ki, kd, forever=False):
        """Set motion PID gains (kp, ki, kd in [0,10]); persist if forever=True.
        PID 参数控制，会影响set_car_motion函数控制小车的运动速度变化情况。默认情况下可不调整。
        Tune motion PID loop (normally not required)
        kp ki kd = [0, 10.00], 可输入小数。
        forever=True永久保存，=False临时作用。
        由于永久保存需要写入芯片flash中，操作时间较长，所以加入delay延迟时间，避免导致单片机丢包的问题。
        临时作用反应快，单次有效，重启单片后数据不再保持。
        PID parameter control will affect the set_CAR_motion function to control the speed change of the car.  This parameter is optional by default.  
        KP ki kd = [0, 10.00]  
        forever=True for permanent, =False for temporary.  
        Since permanent storage needs to be written into the chip flash, which takes a long time to operate, delay is added to avoid packet loss caused by MCU.  
        Temporary effect fast response, single effective, data will not be maintained after restarting the single chip
        """
        try:
            state = 0
            if forever:
                state = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_MOTOR_PID]
            if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
                print("PID value must be:[0, 10.00]")
                return
            kp_params = bytearray(struct.pack('h', int(kp * 1000)))
            ki_params = bytearray(struct.pack('h', int(ki * 1000)))
            kd_params = bytearray(struct.pack('h', int(kd * 1000)))
            cmd.append(kp_params[0])  # low
            cmd.append(kp_params[1])  # high
            cmd.append(ki_params[0])  # low
            cmd.append(ki_params[1])  # high
            cmd.append(kd_params[0])  # low
            cmd.append(kd_params[1])  # high
            cmd.append(state)
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pid:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.1)
        except:
            print('---set_pid_param error!---')
            pass
    
    # 设置偏航角调节的PID
    # Set the PID for yaw Angle adjustment
    # def set_yaw_pid_param(self, kp, ki, kd, forever=False):
    #     try:
    #         state = 0
    #         if forever:
    #             state = 0x5F
    #         cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_YAW_PID]
    #         if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
    #             print("YAW PID value must be:[0, 10.00]")
    #             return
    #         kp_params = bytearray(struct.pack('h', int(kp * 1000)))
    #         ki_params = bytearray(struct.pack('h', int(ki * 1000)))
    #         kd_params = bytearray(struct.pack('h', int(kd * 1000)))
    #         cmd.append(kp_params[0])  # low
    #         cmd.append(kp_params[1])  # high
    #         cmd.append(ki_params[0])  # low
    #         cmd.append(ki_params[1])  # high
    #         cmd.append(kd_params[0])  # low
    #         cmd.append(kd_params[1])  # high
    #         cmd.append(state)
    #         checksum = sum(cmd, self.__COMPLEMENT) & 0xff
    #         cmd.append(checksum)
    #         self.ser.write(cmd)
    #         if self.__debug:
    #             print("pid:", cmd)
    #         time.sleep(self.__delay_time)
    #         if forever:
    #             time.sleep(.1)
    #     except:
    #         print('---set_pid_param error!---')
    #         pass

    # 设置小车类型
    # Persistently set car base type (and store to flash)
    # Set car Type
    def set_car_type(self, car_type):
        """Persist platform type to flash (numeric string or int)."""
        if str(car_type).isdigit():
            self.__CAR_TYPE = int(car_type) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_SET_CAR_TYPE, self.__CAR_TYPE, 0x5F]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_type:", cmd)
            time.sleep(.1)
        else:
            print("set_car_type input invalid")

    # 控制总线舵机。servo_id:[1-255],表示要控制的舵机的ID号, id=254时, 控制所有已连接舵机。
    # Control a bus (UART) servo; 254 broadcasts to all connected servos
    # pulse_value=[96,4000]表示舵机要运行到的位置。
    # run_time表示运行的时间(ms),时间越短,舵机转动越快。最小为0，最大为2000
    # Control bus steering gear.  Servo_id :[1-255], indicating the ID of the steering gear to be controlled. If ID =254, control all connected steering gear.  
    # pulse_value=[96,4000] indicates the position to which the steering gear will run.  
    # run_time indicates the running time (ms). The shorter the time, the faster the steering gear rotates.  The minimum value is 0 and the maximum value is 2000
    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        """Control a bus servo by pulse value (96..4000) over optional time (ms)."""
        try:
            if not self.__arm_ctrl_enable:
                return
            if servo_id < 1 or pulse_value < 96 or pulse_value > 4000 or run_time < 0:
                print("set uart servo input error")
                return
            if run_time > 2000:
                run_time = 2000
            if run_time < 0:
                run_time = 0
            s_id = int(servo_id) & 0xff
            value = bytearray(struct.pack('h', int(pulse_value)))
            r_time = bytearray(struct.pack('h', int(run_time)))

            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_UART_SERVO, \
                s_id, value[0], value[1], r_time[0], r_time[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo:", servo_id, int(pulse_value), cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo error!---')
            pass

    # 设置总线舵机角度接口：s_id:[1,6], s_angle: 1-4:[0, 180], 5:[0, 270], 6:[0, 180], 设置舵机要运动到的角度。
    # Set articulated arm servo by angle (converted internally)
    # run_time表示运行的时间(ms),时间越短,舵机转动越快。最小为0，最大为2000
    # Set bus steering gear Angle interface: s_id:[1,6], s_angle: 1-4:[0, 180], 5:[0, 270], 6:[0, 180], set steering gear to move to the Angle.  
    # run_time indicates the running time (ms). The shorter the time, the faster the steering gear rotates.  The minimum value is 0 and the maximum value is 2000
    def set_uart_servo_angle(self, s_id, s_angle, run_time=500):
        """Convenience: set arm joint by angle (maps to pulse)."""
        try:
            if s_id == 1:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_1 set error!")
            elif s_id == 2:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_2 set error!")
            elif s_id == 3:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_3 set error!")
            elif s_id == 4:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_4 set error!")
            elif s_id == 5:
                if 0 <= s_angle <= 270:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_5 set error!")
            elif s_id == 6:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_6 set error!")
        except:
            print('---set_uart_servo_angle error! ID=%d---' % s_id)
            pass

    # 设置总线舵机的ID号(谨慎使用)，servo_id=[1-250]。
    # Change bus servo ID (ensure only one servo connected!)
    # 运行此函数前请确认只连接一个总线舵机，否则会把所有已连接的总线舵机都设置成同一个ID，造成控制混乱。
    # Set the bus servo ID(Use with caution), servo_id=[1-250].  
    # Before running this function, please confirm that only one bus actuator is connected. Otherwise, all connected bus actuators will be set to the same ID, resulting in confusion of control
    def set_uart_servo_id(self, servo_id):
        """Assign new ID to a single connected bus servo (1..250)."""
        try:
            if servo_id < 1 or servo_id > 250:
                print("servo id input error!")
                return
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_ID, int(servo_id)]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_id:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo_id error!---')
            pass

    # 关闭/打开总线舵机扭矩力, enable=[0, 1]。
    # Enable/disable torque (power) output of all bus servos
    # enable=0:关闭舵机扭矩力，可以用手转动舵机，但命令无法控制转动；
    # enable=1：打开扭矩力，命令可以控制转动，不可以用手转动舵机。
    # Turn off/on the bus steering gear torque force, enable=[0, 1].  
    # enable=0: Turn off the torque force of the steering gear, the steering gear can be turned by hand, but the command cannot control the rotation;  
    # enable=1: Turn on torque force, command can control rotation, can not turn steering gear by hand
    def set_uart_servo_torque(self, enable):
        """Enable (1) or disable (0) torque on all connected bus servos."""
        try:
            if enable > 0:
                on = 1
            else:
                on = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_TORQUE, on]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_torque:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo_torque error!---')
            pass

    # 设置机械臂控制开关，enable=True正常发送控制协议，=False不发送控制协议
    # Enable/disable sending arm control commands (freeze outputs)
    # Set the control switch of the manipulator. Enable =True Indicates that the control protocol is normally sent; False indicates that the control protocol is not sent
    def set_uart_servo_ctrl_enable(self, enable):
        """Globally enable/disable sending bus servo control commands."""
        if enable:
            self.__arm_ctrl_enable = True
        else:
            self.__arm_ctrl_enable = False



    # 同时控制机械臂所有舵机的角度。
    # Atomically set all 6 arm joint target angles
    # Meanwhile, the Angle of all steering gear of the manipulator is controlled
    def set_uart_servo_angle_array(self, angle_s=[90, 90, 90, 90, 90, 180], run_time=500):
        """Set all six arm joint angles simultaneously (list length 6)."""
        try:
            if not self.__arm_ctrl_enable:
                return
            if 0 <= angle_s[0] <= 180 and 0 <= angle_s[1] <= 180 and 0 <= angle_s[2] <= 180 and \
                0 <= angle_s[3] <= 180 and 0 <= angle_s[4] <= 270 and 0 <= angle_s[5] <= 180:
                if run_time > 2000:
                    run_time = 2000
                if run_time < 0:
                    run_time = 0
                temp_val = [0, 0, 0, 0, 0, 0]
                for i in range(6):
                    temp_val[i] = self.__arm_convert_value(i+1, angle_s[i])
                    
                value_s1 = bytearray(struct.pack('h', int(temp_val[0])))
                value_s2 = bytearray(struct.pack('h', int(temp_val[1])))
                value_s3 = bytearray(struct.pack('h', int(temp_val[2])))
                value_s4 = bytearray(struct.pack('h', int(temp_val[3])))
                value_s5 = bytearray(struct.pack('h', int(temp_val[4])))
                value_s6 = bytearray(struct.pack('h', int(temp_val[5])))

                r_time = bytearray(struct.pack('h', int(run_time)))
                cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_CTRL, \
                       value_s1[0], value_s1[1], value_s2[0], value_s2[1], value_s3[0], value_s3[1], \
                       value_s4[0], value_s4[1], value_s5[0], value_s5[1], value_s6[0], value_s6[1], \
                       r_time[0], r_time[1]]
                cmd[2] = len(cmd) - 1
                checksum = sum(cmd, self.__COMPLEMENT) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
                if self.__debug:
                    print("arm:", cmd)
                    print("value:", temp_val)
                time.sleep(self.__delay_time)
            else:
                print("angle_s input error!")
        except:
            print('---set_uart_servo_angle_array error!---')
            pass


    # 设置机械臂的中位偏差，servo_id=0~6， =0全部恢复出厂默认值
    # Calibrate (offset) arm joint mid-position; 0 resets all to defaults
    # Run the following command to set the mid-bit deviation of the manipulator: servo_id=0 to 6, =0 Restore the factory default values
    def set_uart_servo_offset(self, servo_id):
        """Calibrate midpoint offset (0 resets all, 1..6 single joint)."""
        try:
            self.__arm_offset_id = 0xff
            self.__arm_offset_state = 0
            s_id = int(servo_id) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_OFFSET, s_id]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_offset:", cmd)
            time.sleep(self.__delay_time)
            for i in range(200):
                if self.__arm_offset_id == servo_id:
                    if self.__debug:
                        if self.__arm_offset_id == 0:
                            print("Arm Reset Offset Value")
                        else:
                            print("Arm Offset State:", self.__arm_offset_id, self.__arm_offset_state, i)
                    return self.__arm_offset_state
                time.sleep(.001)
            return self.__arm_offset_state
        except:
            print('---set_uart_servo_offset error!---')
            pass

    # 设置阿克曼类型(R2)小车前轮的默认角度，angle=[60, 120]
    # Set default (center) steering angle for Ackermann front wheels
    # forever=True永久保存，=False临时作用。
    # 由于永久保存需要写入芯片flash中，操作时间较长，所以加入delay延迟时间，避免导致单片机丢包的问题。
    # 临时作用反应快，单次有效，重启单片后数据不再保持。
    # Set the default Angle of akerman type (R2) car front wheel, Angle =[60, 120]
    # forever=True for permanent, =False for temporary.
    # Since permanent storage needs to be written into the chip flash, which takes a long time to operate, delay is added to avoid packet loss caused by MCU.  
    # Temporary effect fast response, single effective, data will not be maintained after restarting the single chip
    def set_akm_default_angle(self, angle, forever=False):
        """Set default (center) steering angle 60..120; persist if forever=True."""
        try:
            if int(angle) > 120 or int(angle) < 60:
                return
            id = self.__AKM_SERVO_ID
            state = 0
            if forever:
                state = 0x5F
                self.__akm_def_angle = angle
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_AKM_DEF_ANGLE, id, int(angle), state]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("akm set def angle:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.1)
        except:
            print('---set_akm_default_angle error!---')
            pass

    # 控制阿克曼类型(R2)小车相对于默认角度的转向角，向左为负数，向右为正数，angle=[-45, 45]
    # Command steering offset relative to stored default
    # ctrl_car=False，只控制舵机角度，=True，控制舵机角度同时修改左右电机的速度。
    # Control the steering Angle of ackman type (R2) car relative to the default Angle, negative for left and positive for right, Angle =[-45, 45]
    # ctrl_car=False, only control the steering gear Angle, =True, control the steering gear Angle and modify the speed of the left and right motors.
    def set_akm_steering_angle(self, angle, ctrl_car=False):
        """Set steering offset -45..45 (negative left); optionally adjust drive."""
        try:
            if int(angle) > 45 or int(angle) < -45:
                return
            id = self.__AKM_SERVO_ID
            if ctrl_car:
                id = self.__AKM_SERVO_ID + 0x80
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_AKM_STEER_ANGLE, id, int(angle)&0xFF]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("akm_steering_angle:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_akm_steering_angle error!---')
            pass


    # 重置小车flash保存的数据，恢复出厂默认值。
    # Factory reset (erase stored parameters in flash)
    # Reset the car flash saved data, restore the factory default value
    def reset_flash_value(self):
        """Erase stored flash parameters (factory reset)."""
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_FLASH, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("flash:", cmd)
            time.sleep(self.__delay_time)
            time.sleep(.1)
        except:
            print('---reset_flash_value error!---')
            pass
    
    # 重置小车状态，包括停车，关灯，关蜂鸣器
    # Stop motion, turn off LEDs and buzzer
    # Reset car status, including parking, lights off, buzzer off
    def reset_car_state(self):
        """Stop car, disable LEDs/effects and buzzer (safe idle)."""
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_STATE, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("reset_car_state:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---reset_car_state error!---')
            pass

    # 清除单片机自动发送过来的缓存数据
    # Zero out last cached auto-report sensor values
    # Clear the cache data automatically sent by the MCU
    def clear_auto_report_data(self):
        """Clear cached auto-report sensor values (local only)."""
        self.__battery_voltage = 0
        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__gx = 0.0
        self.__gy = 0.0
        self.__gz = 0.0
        self.__mx = 0.0
        self.__my = 0.0
        self.__mz = 0.0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0
        self.__yaw = 0.0
        self.__roll = 0.0
        self.__pitch = 0.0

    def get_akm_default_angle(self):
        """Read stored default front steering angle (R2 Ackermann) or -1 timeout."""
        if not self.__akm_readed_angle:
            self.__request_data(self.FUNC_AKM_DEF_ANGLE, self.__AKM_SERVO_ID)
            akm_count = 0
            while True:
                if self.__akm_readed_angle:
                    break
                akm_count = akm_count + 1
                if akm_count > 100:
                    return -1
                time.sleep(.01)
        return self.__akm_def_angle



    def get_uart_servo_value(self, servo_id):
        """Return (id, pulse_value) for servo_id or (-1,-1) timeout, (-2,-2) error.
        servo_id=[1-250]
        """
        try:
            if servo_id < 1 or servo_id > 250:
                print("get servo id input error!")
                return
            self.__read_id = 0
            self.__read_val = 0
            self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xff)
            timeout = 30
            while timeout > 0:
                if self.__read_id > 0:
                    return self.__read_id, self.__read_val
                timeout = timeout - 1
                time.sleep(.001)
            return -1, -1
        except:
            print('---get_uart_servo_value error!---')
            return -2, -2

    def get_uart_servo_angle(self, s_id):
        """Return joint angle degrees or -1 out-of-range, -2 comms error.
        Read the Angle of the bus steering gear, s_id indicates the ID number of the steering gear to be read, s_id=[1-6]
        """
        try:
            angle = -1
            read_id, value = self.get_uart_servo_value(s_id)
            if s_id == 1 and read_id == 1:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 2 and read_id == 2:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 3 and read_id == 3:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 4 and read_id == 4:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 5 and read_id == 5:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 270 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 6 and read_id == 6:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            else:
                if self.__debug:
                    print("read servo:%d error!" % s_id)
            if self.__debug:
                print("request angle %d: %d, %d" % (s_id, read_id, value))
            return angle
        except:
            print('---get_uart_servo_angle error!---')
            return -2

    def get_uart_servo_angle_array(self):
        """
        一次性读取六个舵机的角度[xx, xx, xx, xx, xx, xx]，如果某个舵机错误则那一位为-1
        Bulk read all 6 joint angles (invalid -> -1)
        Read the angles of three steering gear [xx, xx, xx, xx, xx, xx] at one time. If one steering gear is wrong, that one is -1
        """
        try:
            # angle = [-1, -1, -1, -1, -1, -1]
            # for i in range(6):
            #     temp1 = self.get_uart_servo_angle(i + 1)
            #     if temp1 >= 0:
            #         angle[i] = temp1
            #     else:
            #         break
            # return angle

            angle = [-1, -1, -1, -1, -1, -1]
            self.__read_arm = [-1, -1, -1, -1, -1, -1]
            self.__read_arm_ok = 0
            self.__request_data(self.FUNC_ARM_CTRL, 1)
            timeout = 30
            while timeout > 0:
                if self.__read_arm_ok == 1:
                    for i in range(6):
                        if self.__read_arm[i] > 0:
                            angle[i] = self.__arm_convert_angle(i+1, self.__read_arm[i])
                    if self.__debug:
                        print("angle_array:", 30-timeout, angle)
                    break
                timeout = timeout - 1
                time.sleep(.001)
            return angle
        except:
            print('---get_uart_servo_angle_array error!---')
            return [-2, -2, -2, -2, -2, -2]

    def get_accelerometer_data(self):
        """Return (ax, ay, az) m/s^2 from last auto-report."""
        a_x, a_y, a_z = self.__ax, self.__ay, self.__az
        # self.__ax, self.__ay, self.__az = 0.0, 0.0, 0.0
        return a_x, a_y, a_z

    def get_gyroscope_data(self):
        """Return (gx, gy, gz) rad/s from last auto-report."""
        g_x, g_y, g_z = self.__gx, self.__gy, self.__gz
        # self.__gx, self.__gy, self.__gz = 0.0, 0.0, 0.0
        return g_x, g_y, g_z

    def get_magnetometer_data(self):
        """Return (mx, my, mz) raw magnetometer values."""
        m_x, m_y, m_z = self.__mx, self.__my, self.__mz
        # self.__mx, self.__my, self.__mz = 0.0, 0.0, 0.0
        return m_x, m_y, m_z

    def get_imu_attitude_data(self, ToAngle=True):
        """Return (roll, pitch, yaw) in degrees (default) or radians."""
        if ToAngle:
            RtA = 57.2957795
            roll = self.__roll * RtA
            pitch = self.__pitch * RtA
            yaw = self.__yaw * RtA
        else:
            roll, pitch, yaw = self.__roll, self.__pitch, self.__yaw
        # self.__roll, self.__pitch, self.__yaw = 0.0, 0.0, 0.0
        return roll, pitch, yaw

    def get_motion_data(self):
        """Return (vx, vy, vz) velocities from last report."""
        val_vx = self.__vx
        val_vy = self.__vy
        val_vz = self.__vz
        # self.__vx, self.__vy, self.__vz = 0.0, 0.0, 0.0
        return val_vx, val_vy, val_vz

    def get_battery_voltage(self):
        """Return battery voltage (float volts)."""
        vol = self.__battery_voltage / 10.0
        # self.__battery_voltage = 0
        return vol

    def get_motor_encoder(self):
        """Return wheel encoder counts tuple (m1,m2,m3,m4)."""
        m1, m2, m3, m4 = self.__encoder_m1, self.__encoder_m2, self.__encoder_m3, self.__encoder_m4
        # self.__encoder_m1, self.__encoder_m2, self.__encoder_m3, self.__encoder_m4 = 0, 0, 0, 0
        return m1, m2, m3, m4

    def get_motion_pid(self):
        """Return PID parameters [kp, ki, kd] or [-1, -1, -1] on timeout."""
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0
        self.__pid_index = 0
        self.__request_data(self.FUNC_SET_MOTOR_PID, int(1))
        for i in range(20):
            if self.__pid_index > 0:
                kp = float(self.__kp1 / 1000.0)
                ki = float(self.__ki1 / 1000.0)
                kd = float(self.__kd1 / 1000.0)
                if self.__debug:
                    print("get_motion_pid: {0}, {1}, {2}".format(self.__pid_index, [kp, ki, kd], i))
                return [kp, ki, kd]
            time.sleep(.001)
        return [-1, -1, -1]

    # 获取小车偏航角PID参数
    # PID parameters of trolley yaw Angle were obtained
    # def get_yaw_pid(self):
    #     self.__kp1 = 0
    #     self.__ki1 = 0
    #     self.__kd1 = 0
    #     self.__pid_index = 0
    #     self.__request_data(self.FUNC_SET_YAW_PID, int(5))
    #     for i in range(20):
    #         if self.__pid_index > 0:
    #             kp = float(self.__kp1 / 1000.0)
    #             ki = float(self.__ki1 / 1000.0)
    #             kd = float(self.__kd1 / 1000.0)
    #             if self.__debug:
    #                 print("get_yaw_pid: {0}, {1}, {2}".format(self.__pid_index, [kp, ki, kd], i))
    #             return [kp, ki, kd]
    #         time.sleep(.001)
    #     return [-1, -1, -1]

    def get_car_type_from_machine(self):
        """Query MCU for stored car type (flash) or -1 on timeout."""
        self.__request_data(self.FUNC_SET_CAR_TYPE)
        for i in range(0, 20):
            if self.__read_car_type != 0:
                car_type = self.__read_car_type
                self.__read_car_type = 0
                return car_type
            time.sleep(.001)
        return -1


    def get_version(self):
        """Return firmware version (float like 1.1) or -1 if unavailable."""
        if self.__version_H == 0:
            self.__request_data(self.FUNC_VERSION)
            for i in range(0, 20):
                if self.__version_H != 0:
                    val = self.__version_H * 1.0
                    self.__version = val + self.__version_L / 10.0
                    if self.__debug:
                        print("get_version:V{0}, i:{1}".format(self.__version, i))
                    return self.__version
                time.sleep(.001)
        else:
            return self.__version
        return -1


if __name__ == '__main__':
    # 小车底层处理库
    # Underlying vehicle (car) processing library demo
    import platform
    device = platform.system()
    print("Read device:", device)
    if device == 'Windows':
        com_index = 1
        while True:
            com_index = com_index + 1
            try:
                print("try COM%d" % com_index)
                com = 'COM%d' % com_index
                bot = Rosmaster(1, com, debug=True)
                break
            except:
                if com_index > 256:
                    print("-----------------------No COM Open--------------------------")
                    exit(0)
                continue
        print("--------------------Open %s---------------------" % com)
    else:
        bot = Rosmaster(com="/dev/ttyUSB0", debug=True)
    
    bot.create_receive_threading()
    time.sleep(.1)
    bot.set_beep(50)
    time.sleep(.1)

    version = bot.get_version()
    print("version=", version)

    # bot.set_car_type(4)
    # time.sleep(.1)
    # car_type = bot.get_car_type_from_machine()
    # print("car_type:", car_type)
    # bot.set_uart_servo_angle(1, 100)

    # s_id, value = bot.get_uart_servo_value(1)
    # print("value:", s_id, value)
    
    # bot.set_uart_servo_torque(1)
    # time.sleep(.1)
    # state = bot.set_uart_servo_offset(6)
    # print("state=", state)

    # bot.set_pid_param(0.5, 0.1, 0.3, 8, True)
    # bot.set_yaw_pid_param(0.4, 2, 0.2, False)
    # bot.reset_flash_value()

    # pid = bot.get_motion_pid(5)
    # pid = bot.get_yaw_pid()
    # print("pid:", pid)

    # angle= bot.get_uart_servo_angle(1)
    # print("angle:", angle)

    # angle_array = bot.get_uart_servo_angle_array()
    # print("angle_array:", angle_array)

    # bot.set_car_motion(0.5, 0, 0)

    # bot.send_ip_addr("192.168.1.2")

    # bot.set_uart_servo_angle(6, 150)

    # bot.set_auto_report_state(0, False)

    # bot.set_pwm_servo_all(50, 50, 50, 50)
    # time.sleep(1)
    # bot.set_car_motion(0, 0, -3.5)
    # bot.set_car_run(6, 50)
    try:
        while True:
            ax, ay, az = bot.get_accelerometer_data()
            gx, gy, gz = bot.get_gyroscope_data()
            mx, my, mz = bot.get_magnetometer_data()
            # print(ax, ay, az)
            print(ax, ay, az, gx, gy, gz, mx, my, mz)
            # print("%3.3f, %3.3f, %3.3f,      %3.3f, %3.3f, %3.3f" % 
            # (ax, ay, az, gx, gy, gz))
            # print("%3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f" % 
            # (ax, ay, az, gx, gy, gz, mx, my, mz))
            # roll, pitch, yaw = bot.get_imu_attitude_data()
            # print("roll:%f, pitch:%f, yaw:%f" % (roll, pitch, yaw))
            # m1, m2, m3, m4 = bot.get_motor_encoder()
            # print("encoder:", m1, m2, m3, m4)

            # v = bot.get_motion_data()
            # print("v:", v)

            # pid = bot.get_motion_pid()
            # print(pid)
            # version = bot.get_version()
            # print("version=", version)
            # vx, vy, vz = bot.get_motion_data()
            # print("V:", vx, vy, vz)
            time.sleep(.1)
    except KeyboardInterrupt:
        bot.set_car_motion(0, 0, 0)
        pass
    exit(0)
