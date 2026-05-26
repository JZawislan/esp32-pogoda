from machine import Pin, I2C
import time
import ustruct
import network
from umqtt.simple import MQTTClient

class BME280:
    def __init__(self, i2c, addr=0x76):
        self.i2c = i2c
        self.addr = addr

        # Odczyt kalibracji temperatury i ciśnienia (0x88 do 0x9F)
        calib = self.i2c.readfrom_mem(self.addr, 0x88, 24)
        (self.dig_T1, self.dig_T2, self.dig_T3,
         self.dig_P1, self.dig_P2, self.dig_P3, self.dig_P4,
         self.dig_P5, self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9) = ustruct.unpack(
            "<HhhHhhhhhhhh", calib
        )

        # Odczyt kalibracji wilgotności (0xA1 oraz 0xE1-0xE7)
        self.dig_H1 = self.i2c.readfrom_mem(self.addr, 0xA1, 1)[0]
        calib_h = self.i2c.readfrom_mem(self.addr, 0xE1, 7)
        self.dig_H2 = ustruct.unpack("<h", calib_h[0:2])[0]
        self.dig_H3 = calib_h[2]
        
        # Specyficzne bitowe upakowanie rejestrów H4 i H5
        h4 = (calib_h[3] << 4) | (calib_h[4] & 0x0F)
        if h4 > 2047: h4 -= 4096
        self.dig_H4 = h4
        
        h5 = (calib_h[5] << 4) | (calib_h[4] >> 4)
        if h5 > 2047: h5 -= 4096
        self.dig_H5 = h5
        
        self.dig_H6 = ustruct.unpack("<b", calib_h[6:7])[0]

        # Inicjalizacja BME280: Rejestr ctrl_hum MUSI być zapisany przed ctrl_meas
        self.i2c.writeto_mem(self.addr, 0xF2, b"\x01") # osrs_h = 1
        self.i2c.writeto_mem(self.addr, 0xF4, b"\x27") # osrs_t=1, osrs_p=1, tryb normalny
        self.i2c.writeto_mem(self.addr, 0xF5, b"\xA0") # standby 1000ms, filter off

    def read_data(self):
        # Pobieramy 8 bajtów danych pomiarowych naraz (Ciśnienie, Temperatura, Wilgotność)
        data = self.i2c.readfrom_mem(self.addr, 0xF7, 8)
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_H = (data[6] << 8) | data[7]

        # --- Obliczenia zmiennoprzecinkowe (Float) dla BME280 ---
        
        # Temperatura
        var1 = (adc_T / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_T / 131072.0 - self.dig_T1 / 8192.0) * (adc_T / 131072.0 - self.dig_T1 / 8192.0)) * self.dig_T3
        t_fine = var1 + var2
        temp_c = t_fine / 5120.0

        # Ciśnienie
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (self.dig_P6 / 32768.0)
        var2 = var2 + var1 * (self.dig_P5 * 2.0)
        var2 = (var2 / 4.0) + (self.dig_P4 * 65536.0)
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        
        if var1 == 0:
            press_hpa = 0.0
        else:
            p = 1048576.0 - adc_P
            p = ((p - (var2 / 4096.0)) * 6250.0) / var1
            var1 = self.dig_P9 * p * p / 2147483648.0
            var2 = p * self.dig_P8 / 32768.0
            press_hpa = (p + (var1 + var2 + self.dig_P7) / 16.0) / 100.0

        # Wilgotność
        h = t_fine - 76800.0
        if h != 0:
            h = (adc_H - (self.dig_H4 * 64.0 + self.dig_H5 / 16384.0 * h)) * (self.dig_H2 / 65536.0 * (1.0 + self.dig_H6 / 67108864.0 * h * (1.0 + self.dig_H3 / 67108864.0 * h)))
            h = h * (1.0 - self.dig_H1 * h / 524288.0)
            if h > 100.0:
                h = 100.0
            elif h < 0.0:
                h = 0.0
        hum_perc = h

        return temp_c, press_hpa, hum_perc


WIFI_SSID = "WiFiNet"
WIFI_PASSWORD = "Admin2003"

MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "esp32-bme280"
MQTT_TOPIC = "PK/JZiKR"


def connect_wifi(timeout_ms=15000, retries=3):
    wlan = network.WLAN(network.STA_IF)
    for _ in range(retries):
        try:
            wlan.active(False)
            time.sleep(0.2)
            wlan.active(True)
            if not wlan.isconnected():
                wlan.connect(WIFI_SSID, WIFI_PASSWORD)
                start = time.ticks_ms()
                while not wlan.isconnected():
                    if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                        raise OSError("WiFi connect timeout")
                    time.sleep(0.2)
            return wlan
        except OSError:
            time.sleep(1)
    raise OSError("WiFi failed after retries")


def mqtt_connect():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
    client.connect()
    return client

# Inicjalizacja I2C
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)

print("I2C scan:", i2c.scan())  # Upewnij się, że używasz właściwego adresu z tego wyniku

# Inicjalizacja naszej nowej klasy BME280
bme = BME280(i2c, addr=0x76)    # Zmień na 0x77 jesli scan wskazał ten adres

connect_wifi()
mqtt = mqtt_connect()

while True:
    # Odczyt wszystkich danych w jednym kroku
    temp_c, press_hpa, hum_perc = bme.read_data()
    
    # Budowanie JSON-a z uwzględnieniem wilgotności
    payload = "{{\"temp_c\":{:.2f},\"press_hpa\":{:.2f},\"hum_perc\":{:.2f}}}".format(temp_c, press_hpa, hum_perc)
    
    print("Temp: {:.2f} C  Press: {:.2f} hPa  Hum: {:.2f} %".format(temp_c, press_hpa, hum_perc))
    
    try:
        mqtt.publish(MQTT_TOPIC, payload)
    except OSError:
        mqtt = mqtt_connect()
        mqtt.publish(MQTT_TOPIC, payload)
        
    time.sleep(1)