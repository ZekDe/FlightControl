# FlightControl - ESP32 Quadcopter Flight Controller

ESP32-C3 tabanlı, profesyonel kalitede quadcopter uçuş kontrolcüsü.

## 📋 Özellikler

- **Attitude Estimation**: Quaternion tabanlı EKF (Extended Kalman Filter)
- **Kontrol**: Cascaded PID (Attitude + Rate loops)
- **Motor Mixer**: X-konfigürasyon quadcopter
- **Sensörler**: MPU9250 (Gyro + Accel), AK8963 (Magnetometer)
- **Haberleşme**: BLE (Nordic UART Service), WiFi
- **OTA Update**: Web tabanlı firmware güncelleme
- **RC Input**: SBUS receiver desteği
- **Web Dashboard**: Gerçek zamanlı telemetri ve PID tuning

## 🔧 Donanım

| Bileşen | Model |
|---------|-------|
| MCU | ESP32-C3 Super Mini |
| IMU | MPU9250 / MPU9255 |
| Motor Driver | 4x PWM ESC |

### Bağlantı Şeması

```
ESP32-C3           MPU9250
--------           -------
GPIO6 (SDA) ───── SDA
GPIO7 (SCL) ───── SCL
3V3         ───── VCC
GND         ───── GND

ESP32-C3           ESC (PWM)
--------           --------
GPIO2       ───── Motor 1 (Front-Right)
GPIO3       ───── Motor 2 (Back-Right)
GPIO4       ───── Motor 3 (Back-Left)
GPIO5       ───── Motor 4 (Front-Left)
```

## 🚀 Kurulum

### Gereksinimler
- ESP-IDF v5.x
- Python 3.x

### Derleme
```bash
cd "esp32 c3 Super Mini"
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

## 📱 BLE Komutları

"QUAD-FC" cihazına bağlanarak aşağıdaki komutları gönderebilirsiniz:

| Komut | Açıklama |
|-------|----------|
| `ARM` | Motorları aktif et |
| `DISARM` | Motorları kapat |
| `THR=xx` | Throttle ayarla (0-100) |
| `ROLL=xx` | Roll açısı (-45 to +45) |
| `PITCH=xx` | Pitch açısı (-45 to +45) |
| `YAW=xx` | Yaw rate (-180 to +180) |
| `TEL=1/0` | Telemetri aç/kapat |
| `GET` | PID değerlerini al |
| `RATE_RP=p,i,d` | Rate PID ayarla |

## 🌐 WiFi & OTA

### AP Mode (Varsayılan)
- SSID: `QUAD-FC-SETUP`
- Password: `12345678`

### OTA Update
1. WiFi'ye bağlan
2. Tarayıcıda `http://192.168.4.1/update` adresine git
3. Firmware (.bin) dosyasını yükle

### Web Dashboard
- `http://192.168.4.1/` - Gerçek zamanlı telemetri ve kontrol

## ⚠️ Güvenlik Uyarıları

> **DİKKAT**: Bu bir gerçek uçuş kontrolcüsüdür!

- ARM komutundan önce pervaneleri çıkarın
- İlk testleri masada, motorlar bağlı değilken yapın
- PID değerleri her drone için farklıdır

## 📁 Dosya Yapısı

```
main/
├── main.c              # Ana uygulama
├── ekf_quat.c/h        # Extended Kalman Filter
├── attitude_controller.c/h  # Outer loop (angle)
├── rate_controller.c/h      # Inner loop (rate)
├── pid_controller.c/h       # PID algoritması
├── motor_mixer.c/h          # Motor karıştırıcı
├── mpu9250.c/h              # IMU driver
├── ak8963.c/h               # Magnetometer driver
├── ble_tuner.c/h            # BLE arayüzü
├── wifi_manager.c/h         # WiFi yönetimi
├── ota_update.c/h           # OTA güncelleme
├── sbus_receiver.c/h        # SBUS RC input
├── web_server.c/h           # Web dashboard
└── fc_error.h               # Hata kodları
```

## 📄 Lisans

MIT License

## 👤 Yazar

Emrah Duatepe ([@ZekDe](https://github.com/ZekDe))

*Bu proje yapay zeka yardımıyla geliştirilmiştir.*
