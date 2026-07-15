# Gazebo Harmonic — Tilt-Rotor VTOL (gz_tiltrotor)

PX4 **v1.15.4** üzerinde Gazebo Harmonic (gz-sim 8) ile çalışan Tilt-Rotor VTOL
simülasyonu. Gazebo Classic'e ihtiyaç duymaz.

Ön iki rotor (`motor_0`, `motor_2`) multicopter modunda dikey durur ve fixed-wing
geçişinde fiziksel olarak **0° → 90°** tilt eder.

---

## 1. Gereksinimler

Gazebo Harmonic **dev kütüphaneleriyle birlikte** kurulu olmalıdır. Yalnızca
`gz-transport13-cli` kuruluysa PX4 `gz_bridge` modülünü **sessizce atlar** ve
hiçbir `gz_*` hedefi üretilmez (bkz. Sorun Giderme).

```bash
sudo apt install -y gz-harmonic
```

Doğrulama:

```bash
gz sim --version      # Gazebo Sim, version 8.x
```

## 2. Başlatma

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_tiltrotor
```

Gazebo GUI açılır, model spawn olur ve aynı terminalde PX4'ün `pxh>` konsolu gelir.

GUI olmadan (sunucu / uzak makine):

```bash
HEADLESS=1 make px4_sitl gz_tiltrotor
```

İlk çağrı derleme gerektirirse birkaç dakika sürer; sonrakiler saniyeler içinde açılır.

### Farklı world'ler

CMake her world için ayrı hedef üretir:

```bash
make px4_sitl gz_tiltrotor_baylands
make px4_sitl gz_tiltrotor_windy
make px4_sitl gz_tiltrotor_lawn
```

## 3. Uçuş — `pxh>` konsolunda

```sh
commander takeoff      # MC modunda kalkış
commander transition   # MC -> FW  (rotorlar 0° -> 90° tilt eder)
commander transition   # FW -> MC  (geri geçiş)
commander land
```

Geçişin oturması ~6 saniye sürer.

### QGroundControl

Açıksa UDP 14550 üzerinden otomatik bağlanır; uçuş modları ve VTOL geçişi oradan da
tetiklenebilir.

## 4. Doğrulama komutları

VTOL durumu (`3` = MC, `4` = FW, `1`/`2` = geçiş anı):

```bash
./build/px4_sitl_default/bin/px4-listener vtol_vehicle_status | grep vehicle_vtol_state
```

PX4'ün ürettiği servo komutları — `control[3]` ve `control[4]` tilt servolarıdır
(`-1` = MC/dikey, `+1` = FW/ileri):

```bash
./build/px4_sitl_default/bin/px4-listener actuator_servos
```

Rotorun **fiziksel** açısı (RPY'nin pitch bileşeni; `0` = dikey, `1.57` = ileri):

```bash
gz model -m tiltrotor_0 | awk '/- Name: motor_0$/,/Link \[25\]/' | tail -2
```

> `gz topic -e` bu ortamda güvenilmezdir (gz-transport docker0 arayüzüne bağlanıyor).
> Pub/sub yerine yukarıdaki **servis çağrısı** tabanlı komutları kullanın.

## 5. Sorun Giderme

### `No rule to make target 'gz_tiltrotor'`

Gazebo Harmonic dev kütüphaneleri eksiktir. `gz_bridge/CMakeLists.txt` içindeki
`find_package(gz-transport NAMES gz-transport13)` başarısız olur ve tüm
`if(gz-transport_FOUND)` bloğu — modül **ve** `gz_*` hedeflerini üreten döngü —
atlanır. Teşhis: `make px4_sitl gz_x500` de aynı hatayı veriyorsa sorun tiltrotor'da
değildir.

Çözüm: `sudo apt install -y gz-harmonic`, ardından CMake cache'i sıfırlanmalıdır:

```bash
rm -rf build/px4_sitl_default
make px4_sitl_default
```

### `apt update` → `NO_PUBKEY 67170598AF249743`

`sources.list`, OSRF anahtarının ASCII-armored kopyasını gösteriyor olabilir; apt
`signed-by=` ile verilen `.gpg` yolunda **binary** format bekler ve anahtar mevcut
olsa bile okuyamaz. Aynı anahtarın binary kopyasına yönlendirin:

```bash
sudo sed -i 's|gazebo-archive-keyring.gpg|pkgs-osrf-archive-keyring.gpg|' \
  /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
```

İmza doğrulamasını baypas etmeyin (`[trusted=yes]`, `--allow-unauthenticated`); gerek yoktur.

### Airframe parametreleri uygulanmıyor

`param set-default` yalnızca parametre kayıtlı değilse etkilidir; kaydedilmiş paramlar
onu gölgeler. `4020_gz_tiltrotor` içinde değişiklik yaptıysanız:

```bash
rm -rf build/px4_sitl_default/rootfs/eeprom
```

### Sim açılmıyor / model spawn olmuyor

Önceki oturumdan artakalan `gz sim` sunucusu olabilir:

```bash
pkill -9 -f 'gz sim'; pkill -9 -f 'px4_sitl_default/bin/px4'
```

### `gz_frame_id ... not defined in SDF` uyarıları

Zararsızdır. Model `<sdf version='1.5'>` beyan eder, bu şemada `gz_frame_id` tanımlı
değildir; sdformat onu kopyalar ve sensörler çalışır. **Sürümü yükseltmeyin** — SDF
1.7 pose frame semantiğini değiştirdi ve link pozlarını sessizce bozabilir.

---

## 6. Mimari — dosyalar ve rolleri

### Model (submodule: `Tools/simulation/gz`)

`models/tiltrotor/model.sdf` — Classic pluginlerinin gz-sim karşılıkları:

| Classic | Gazebo Harmonic |
|---|---|
| `libLiftDragPlugin.so` | `gz-sim-lift-drag-system` |
| `libgazebo_motor_model.so` | `gz-sim-multicopter-motor-model-system` |
| `libgazebo_mavlink_interface.so` (servo kanalları) | `gz-sim-joint-position-controller-system` + PX4 `gz_bridge` |

> Bu klasör bir **git submodule**'dür (`PX4-gazebo-models`). Commit `px4-v1.15.4-tiltrotor`
> dalındadır. Kalıcı olması için kendi fork'unuza push edip `.gitmodules` URL'ini
> güncellemeniz gerekir; aksi halde `git submodule update` değişikliği siler.

### PX4 deposu

| Dosya | Rol |
|---|---|
| `ROMFS/.../airframes/4020_gz_tiltrotor` | Airframe; `gz_tiltrotor` hedefi **bundan otomatik üretilir** |
| `ROMFS/.../airframes/CMakeLists.txt` | Airframe kaydı |
| `src/modules/simulation/gz_bridge/module.yaml` | `SIM_GZ_SV_MINA/MAXA${i}` servo açı parametreleri |
| `.../GZMixingInterfaceServo.{hpp,cpp}` | Servo komutunu MINA/MAXA ile radyana ölçekler |

### Servo eşlemesi

| gz topic | Joint | Param | Fonksiyon |
|---|---|---|---|
| `servo_0` | `left_elevon_joint` | `SIM_GZ_SV_FUNC1` | 201 |
| `servo_1` | `right_elevon_joint` | `SIM_GZ_SV_FUNC2` | 202 |
| `servo_2` | `elevator_joint` | `SIM_GZ_SV_FUNC3` | 203 |
| `servo_3` | `motor_0_joint` (**tilt**) | `SIM_GZ_SV_FUNC4` | 204 (Tilt 1) |
| `servo_4` | `motor_2_joint` (**tilt**) | `SIM_GZ_SV_FUNC5` | 205 (Tilt 2) |

Topic'ler 0-indeksli, parametreler 1-indekslidir (`servo_3` ↔ `SIM_GZ_SV_*4`).

### Neden bridge değişikliği gerekti

Stok v1.15.4 servo komutunu sabit `(output - 500) / 500` ile yayınlıyordu, yani
**sabit ±1 rad**. Tilt 90° = 1.5708 rad gerektirir → rotorlar **57.3°'de tıkanır** ve
geçiş tamamlanamaz. `SIM_GZ_SV_MAX`'ı yükseltmek çözmez (mixer 1000'de sınırlı).
v1.16'nın açı parametresi yaklaşımı geri-port edildi.

Varsayılanlar upstream'den **bilinçli olarak** saptırılmıştır: upstream ±45° kullanır,
burada **±57.29578° (= ±1 rad)** kullanılır; böylece mevcut `standard_vtol`,
`advanced_plane`, `rc_cessna` airframe'lerinin davranışı birebir korunur.

### Classic'ten sapmalar

- **Airspeed**: upstream model `model://airspeed` içerir, ancak o model bu submodule
  sürümünde yoktur. Kaldırıldı; airspeed `SENS_EN_ARSPDSIM=1` ile PX4 tarafından
  simüle edilir (bu sürümdeki diğer VTOL airframe'leriyle aynı yöntem).
- **Disarm'da tilt**: `SIM_GZ_SV_DIS4/5 = 0` eklendi; rotorlar dikey park eder.
  Classic'in `zero_position_disarmed=0` davranışıyla aynıdır. (Upstream v1.16 bunu
  yapmaz ve 45°'de durur.)

---

## 7. Doğrulanmış davranış

| Test | Sonuç |
|---|---|
| MC kalkış | 9.3 m irtifa |
| FW geçişi | `vtol_state: 4`, 16.35 m/s airspeed |
| Geri geçiş | `vtol_state: 2 → 3`, irtifa korunuyor |
| Fiziksel tilt (`motor_0` RPY pitch) | disarm `0.000` → FW `1.570` rad (**89.95°**) |
| Geriye uyumluluk (mevcut modeller) | sapma **8.5e-9 rad** |

Tilt ölçümü komut topic'i değil, fizik motorundaki gerçek rijit-cisim durumudur.
Joint'in `<upper>` limiti 1.57 olduğundan 1.5708 yerine 1.570'te durur (0.045° fark,
upstream'in kendi SDF limiti).
