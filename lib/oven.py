import threading # only use threading.Thread
import time # only use time.sleep
import random # unused
import datetime # only use datetime.datetime
import logging # only use logging.getLogger
import json # only use json.loads

import config

log = logging.getLogger(__name__)

try:
    if config.max31855 + config.max6675 + config.max31855spi > 1:
        log.error("choose (only) one converter IC")
        exit()
    if config.max31855:
        from max31855 import MAX31855, MAX31855Error
        log.info("import MAX31855")
    if config.max31855spi:
        import Adafruit_GPIO.SPI as SPI
        from max31855spi import MAX31855SPI, MAX31855SPIError
        log.info("import MAX31855SPI")
        spi_reserved_gpio = [7, 8, 9, 10, 11]
        if config.gpio_air in spi_reserved_gpio:
            raise Exception(f"gpio_air pin {config.gpio_air:s} collides with SPI pins {spi_reserved_gpio:s}")
        if config.gpio_cool in spi_reserved_gpio:
            raise Exception(f"gpio_cool pin {config.gpio_cool:s} collides with SPI pins {spi_reserved_gpio:s}")
        if config.gpio_door in spi_reserved_gpio:
            raise Exception(f"gpio_door pin {config.gpio_door:s} collides with SPI pins {spi_reserved_gpio:s}")
        if config.gpio_heat in spi_reserved_gpio:
            raise Exception(f"gpio_heat pin {config.gpio_heat:s} collides with SPI pins {spi_reserved_gpio:s}")
    if config.max6675:
        from max6675 import MAX6675, MAX6675Error
        log.info("import MAX6675")
    sensor_available = True
except ImportError:
    log.exception("Could not initialize temperature sensor, using dummy values!")
    sensor_available = False

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(config.gpio_heat, GPIO.OUT)
    GPIO.setup(config.gpio_cool, GPIO.OUT)
    GPIO.setup(config.gpio_air, GPIO.OUT)
    GPIO.setup(config.gpio_door, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    gpio_available = True
except ImportError:
    msg = "Could not initialize GPIOs, oven operation will only be simulated!"
    log.warning(msg)
    gpio_available = False


class Oven (threading.Thread):
    STATE_IDLE = "IDLE"
    STATE_RUNNING = "RUNNING"

    def __init__(self, simulate=False, time_step=config.sensor_time_wait):
        threading.Thread.__init__(self)
        self.daemon = True
        self.simulate = simulate
        self.time_step = time_step
        self.reset()
        if simulate:
            self.temp_sensor = TempSensorSimulate(self, 0.5, self.time_step)
        if sensor_available:
            self.temp_sensor = TempSensorReal(self.time_step)
        else:
            self.temp_sensor = TempSensorSimulate(self,
                                                  self.time_step,
                                                  self.time_step)
        self.temp_sensor.start()
        self.start()

    def reset(self):
        self.profile = None
        self.start_time = 0
        self.runtime = 0
        self.totaltime = 0
        self.target = 0
        self.door = self.get_door_state()
        self.state = Oven.STATE_IDLE
        self.set_heat(False)
        self.set_cool(False)
        self.set_air(False)
        self.pid = PID(ki=config.pid_ki, kd=config.pid_kd, kp=config.pid_kp)

    def run_profile(self, profile):
        log.info(f"Running profile {profile.name}")
        self.profile = profile
        self.totaltime = profile.get_duration()
        self.state = Oven.STATE_RUNNING
        self.start_time = datetime.datetime.now()
        log.info("Starting")

    def abort_run(self):
        self.reset()

    def run(self):
        temperature_time = 0
        last_temp = 0
        pid = 0
        while True:
            self.door = self.get_door_state()

            if self.state == Oven.STATE_RUNNING:
                if self.simulate:
                    self.runtime += 0.5
                else:
                    runtime_delta = datetime.datetime.now() - self.start_time
                    self.runtime = runtime_delta.total_seconds()
                log.info(f"running at {self.temp_sensor.temperature:.1f} deg C (Target: {self.target:.1f}) , heat {self.heat:.2f}, cool {self.cool:.2f}, air {self.air:.2f}, door {self.door:s} ({self.runtime:.1f}s/{self.totaltime:.0f})")
                self.target = self.profile.get_target_temperature(self.runtime)
                pid = self.pid.compute(self.target, self.temp_sensor.temperature, self.simulate)

                log.info(f"pid: {pid:.3f}")

                self.set_cool(pid <= -1)

                if (pid >= 1):
                    # The temp should be changing with the heat on, therefore
                    # monitor duration heat has been on full power.
                    # If the heat has been on full for [30] seconds and nothing
                    # is changing, reset.
                    # This prevents runaway in the event of sensor read failure.
                    # Note: the PID parameter will first have to reach 1 before
                    # a sensor failure will be identified.
                    if last_temp == self.temp_sensor.temperature:
                        if (self.runtime - temperature_time) >= 30:
                            log.info("Error reading sensor, oven temp not responding to heat.")
                            self.reset()
                    else:
                        temperature_time = self.runtime

                #Capture the last temperature value.  This must be done before set_heat, since there is a sleep in there now.
                last_temp = self.temp_sensor.temperature

                self.set_heat(pid)

                #if self.profile.is_rising(self.runtime):
                #    self.set_cool(False)
                #    self.set_heat(self.temp_sensor.temperature < self.target)
                #else:
                #    self.set_heat(False)
                #    self.set_cool(self.temp_sensor.temperature > self.target)

                if self.temp_sensor.temperature > 200:
                    self.set_air(False)
                elif self.temp_sensor.temperature < 180:
                    self.set_air(True)

                if self.runtime >= self.totaltime:
                    log.info('Run complete')
                    self.reset()


            if pid > 0:
                time.sleep(self.time_step * (1 - pid))
            else:
                time.sleep(self.time_step)

    def set_heat(self, value):
        if value > 0:
            self.heat = 1.0
            if gpio_available and not self.simulate:
                if config.heater_invert:
                    GPIO.output(config.gpio_heat, GPIO.LOW)
                    time.sleep(self.time_step * value)
                    GPIO.output(config.gpio_heat, GPIO.HIGH)
                else:
                    GPIO.output(config.gpio_heat, GPIO.HIGH)
                    time.sleep(self.time_step * value)
                    GPIO.output(config.gpio_heat, GPIO.LOW)
            else:
                time.sleep(self.time_step * value)
        else:
            self.heat = 0.0
            if gpio_available:
               if config.heater_invert:
                 GPIO.output(config.gpio_heat, GPIO.HIGH)
               else:
                 GPIO.output(config.gpio_heat, GPIO.LOW)

    def set_cool(self, value):
        if value:
            self.cool = 1.0
            if gpio_available:
                GPIO.output(config.gpio_cool, GPIO.LOW)
        else:
            self.cool = 0.0
            if gpio_available:
                GPIO.output(config.gpio_cool, GPIO.HIGH)

    def set_air(self, value):
        if value:
            self.air = 1.0
            if gpio_available:
                GPIO.output(config.gpio_air, GPIO.LOW)
        else:
            self.air = 0.0
            if gpio_available:
                GPIO.output(config.gpio_air, GPIO.HIGH)

    def get_state(self):
        state = {
            'runtime': self.runtime,
            'temperature': self.temp_sensor.temperature,
            'target': self.target,
            'state': self.state,
            'heat': self.heat,
            'cool': self.cool,
            'air': self.air,
            'totaltime': self.totaltime,
            'door': self.door
        }
        return state

    def get_door_state(self):
        if gpio_available:
            return "OPEN" if GPIO.input(config.gpio_door) else "CLOSED"
        else:
            return "UNKNOWN"


class TempSensor(threading.Thread):
    def __init__(self, time_step):
        threading.Thread.__init__(self)
        self.daemon = True
        self.temperature = 0
        self.time_step = time_step


class TempSensorReal(TempSensor):
    def __init__(self, time_step):
        TempSensor.__init__(self, time_step)
        if config.max6675:
            log.info("init MAX6675")
            self.thermocouple = MAX6675(config.gpio_sensor_cs,
                                     config.gpio_sensor_clock,
                                     config.gpio_sensor_data,
                                     config.temp_scale)

        if config.max31855:
            log.info("init MAX31855")
            self.thermocouple = MAX31855(config.gpio_sensor_cs,
                                     config.gpio_sensor_clock,
                                     config.gpio_sensor_data,
                                     config.temp_scale)

        if config.max31855spi:
            log.info("init MAX31855-spi")
            self.thermocouple = MAX31855SPI(spi_dev=SPI.SpiDev(port=0, device=config.spi_sensor_chip_id))

    def run(self):
        # init previous temp
        from collections import deque
        temps = deque([])
        for i in range(3):
            temps.append(self.thermocouple.get())
        good_reading = (temps.count(temps[0]) != len(temps))
        while (not good_reading) and (i < 10):
            # checks if all list elements equal
            log.info("Suspect thermocouple reading...re-reading.")
            temps.append(self.thermocouple.get())
            temps.popleft()
            i += 1
            good_reading = (temps.count(temps[0]) != len(temps))
            time.sleep(0.01)
        temp_prev = temps[-1]
        log.info(f"Starting temp: {temp_prev:.1f} deg C")

        while True:
            try:
                temp_new = self.thermocouple.get()
                good_reading = (abs(temp_new - temp_prev) <= 5)
                while (not good_reading) and (i < 3):
                    log.info("Suspect thermocouple reading...re-reading.")
                    time.sleep(0.01)
                    temp_new = self.thermocouple.get()
                    log.info(f"New temp: {temp_new:.1f} deg C, Previous temp: {temp_prev:.1f} deg C")
                    good_reading = (abs(temp_new - temp_prev) <= 2)
                if good_reading:
                    self.temperature = temp_new
                else:
                    log.info("No good reading found, keeping previous reading.")

            except Exception:
                log.exception("Problem reading temp")
            time.sleep(self.time_step)


class TempSensorSimulate(TempSensor):
    def __init__(self, oven, time_step, sleep_time):
        TempSensor.__init__(self, time_step)
        self.oven = oven
        self.sleep_time = sleep_time

    def run(self):
        t_env      = config.sim_t_env
        c_heat     = config.sim_c_heat
        c_oven     = config.sim_c_oven
        p_heat     = config.sim_p_heat
        R_o_nocool = config.sim_R_o_nocool
        R_o_cool   = config.sim_R_o_cool
        R_ho_noair = config.sim_R_ho_noair
        R_ho_air   = config.sim_R_ho_air

        t = t_env  # deg C  temp in oven
        t_h = t    # deg C temp of heat element
        while True:
            #heating energy
            Q_h = p_heat * self.time_step * self.oven.heat

            #temperature change of heat element by heating
            t_h += Q_h / c_heat

            if self.oven.air:
                R_ho = R_ho_air
            else:
                R_ho = R_ho_noair

            #energy flux heat_el -> oven
            p_ho = (t_h - t) / R_ho

            #temperature change of oven and heat el
            t   += p_ho * self.time_step / c_oven
            t_h -= p_ho * self.time_step / c_heat

            #energy flux oven -> env
            if self.oven.cool:
                p_env = (t - t_env) / R_o_cool
            else:
                p_env = (t - t_env) / R_o_nocool

            #temperature change of oven by cooling to env
            t -= p_env * self.time_step / c_oven
            log.debug(f"energy sim: -> {(p_heat * self.oven.heat):.0f}W heater: {t_h:.0f} -> {p_ho:.0f}W oven: {t:.0f} -> {p_env:.0f}W env")
            self.temperature = t

            time.sleep(self.sleep_time)


class Profile():
    def __init__(self, json_data):
        obj = json.loads(json_data)
        self.name = obj["name"]
        self.data = sorted(obj["data"])

    def get_duration(self):
        return max([t for (t, x) in self.data])

    def get_surrounding_points(self, time):
        if time > self.get_duration():
            return (None, None)

        prev_point = None
        next_point = None

        for i in range(len(self.data)):
            if time <= self.data[i][0]:
                prev_point = self.data[i-1]
                next_point = self.data[i]
                break

        return (prev_point, next_point)

    def is_rising(self, time):
        (prev_point, next_point) = self.get_surrounding_points(time)
        if prev_point and next_point:
            return prev_point[1] < next_point[1]
        else:
            return False

    def get_target_temperature(self, time):
        if time > self.get_duration():
            return 0

        (prev_point, next_point) = self.get_surrounding_points(time)

        incl = float(next_point[1] - prev_point[1]) / float(next_point[0] - prev_point[0])
        temp = prev_point[1] + (time - prev_point[0]) * incl
        return temp


class PID():
    def __init__(self, ki=1, kp=1, kd=1):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.lastNow = datetime.datetime.now()
        self.iterm = 0
        self.lastErr = 0

    def compute(self, setpoint, ispoint, simulate=False):
        if simulate:
            now = self.lastNow + datetime.timedelta(seconds=0.5)
        else:
            now = datetime.datetime.now()
        timeDelta = (now - self.lastNow).total_seconds()

        error = float(setpoint - ispoint)
        self.iterm += (error * timeDelta * self.ki)
        self.iterm = sorted([-1, self.iterm, 1])[1]
        dErr = (error - self.lastErr) / timeDelta

        output = self.kp * error + self.iterm + self.kd * dErr
        output = sorted([-1, output, 1])[1]
        self.lastErr = error
        self.lastNow = now

        return output
