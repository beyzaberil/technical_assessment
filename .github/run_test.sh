#!/bin/bash

# Herhangi bir komut başarısız olursa, betiğin tamamını durdur ve hata koduyla çık.
# Bu, GitHub Actions'ın yeşil tik göstermesini engeller.
set -e

# Betik sonlandığında (başarılı veya başarısız) arka plandaki tüm işleri sonlandıracak bir kapan kur.
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# --- Kurulum ve Derleme ---
echo "Sourcing ROS 2 and building the workspace..."
source /opt/ros/humble/setup.bash
colcon build

# Derlenmiş yeni çalışma alanını kaynak göster
source install/setup.bash

# --- Simülasyonu Başlatma ---
echo "Starting PX4 SITL simulation for standard_plane..."
# HATA 2 ÇÖZÜMÜ: Hangi uçağın yükleneceğini çevre değişkeni ile belirtiyoruz.
# PX4 ana dizinine giderek komutu çalıştırıyoruz.
cd /root/PX4-Autopilot
PX4_GZ_MODEL=standard_plane make px4_sitl_default gazebo &
# Arka planda çalışması için '&' simgesini kullanıyoruz.
PX4_PID=$!
cd /root/ # Proje kök dizinine geri dön

echo "Waiting for simulation to initialize..."
sleep 20 # Simülasyonun başlaması için bekleme süresi

# --- MAVROS Köprüsünü Başlatma ---
echo "Starting MAVROS..."
# HATA 1 ÇÖZÜMÜ: Köprüyü doğrudan değil, launch dosyasıyla güvenli bir şekilde başlatıyoruz.
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557" &
MAVROS_PID=$!

echo "Waiting for MAVROS to connect..."
sleep 15 # MAVROS'un PX4'e bağlanması için bekle

# --- Testi Çalıştırma ---
echo "Running the main test script..."
# Bu komut ön planda çalışır. Başarısı veya başarısızlığı betiğin sonucunu belirler.
# Burası, adayın kodunu ve test doğrulama mantığını çalıştıracağımız yer olacak.
# Şimdilik sadece "arm" görevini test eden bir betik varsayalım.
ros2 run test_package test_arming # Bu betiği bir sonraki adımda dolduracağız.

# Eğer buraya kadar geldiyse, test başarılıdır. `set -e` sayesinde
# önceki adımlardan birinde hata olsaydı betik zaten dururdu.
echo "✅ Test script finished successfully!"
