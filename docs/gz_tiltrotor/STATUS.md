# Proje Durumu — Gazebo Harmonic VTOL Portu

> Bu dosya bir **devir/devam noktası**dır. Çalışmaya buradan devam edilir.
> Kullanım talimatları için [README.md](README.md).
>
> Son güncelleme: 2026-07-15 · Dal: `gz-tiltrotor-port` · Son commit: `f29b93020e`

---

## 1. Nerede kaldık

PX4 v1.15.4 üzerinde Gazebo Harmonic ile çalışan **iki bağımsız VTOL modeli** var.
İkisi de SITL'de uçuruldu ve doğrulandı. Açık iş kalemleri için → [§6](#6-açık-konular).

| Model | Hedef | Konfigürasyon | Durum |
|---|---|---|---|
| **Model-1** | `gz_tiltrotor` | 4 rotor; ön 2'si tilt, arka 2'si sabit dikey | ✅ Uçuyor |
| **Model-2** | `gz_tiltrotor_2plus1` | 3 rotor; kanatta 2 tilt + kuyrukta 1 tilt (cruise'da pusher) | ✅ Uçuyor |

```bash
make px4_sitl gz_tiltrotor          # Model-1
make px4_sitl gz_tiltrotor_2plus1   # Model-2
```

---

## 2. Yapılan eklemeler

### 2.1 `b87255f406` — Bridge backport + Model-1

**Asıl mühendislik işi buydu.** Stok v1.15.4 servo komutunu sabit
`(output - 500) / 500` ile yayınlıyordu → **sabit ±1 rad**. Tilt 90° = 1.5708 rad
gerektirir, yani rotorlar **57.3°'de tıkanıyor** ve geçiş asla tamamlanamıyordu.
`SIM_GZ_SV_MAX`'ı yükseltmek çözmez (mixer 1000'de sınırlı). v1.16'nın açı
parametresi yaklaşımı geri-port edildi.

| Dosya | Ne yapıldı |
|---|---|
| `src/modules/simulation/gz_bridge/module.yaml` | `SIM_GZ_SV_MINA/MAXA${i}` (8 kanal) |
| `.../GZMixingInterfaceServo.hpp` | `DEFINE_PARAMETERS` + `get_servo_angle_min/max()` |
| `.../GZMixingInterfaceServo.cpp` | Ham çıkışı radyan aralığına ölçekler |
| `ROMFS/.../airframes/4020_gz_tiltrotor` | Model-1 airframe |
| `ROMFS/.../airframes/CMakeLists.txt` | Airframe kaydı |
| `docs/gz_tiltrotor/README.md` | Kullanım dokümanı |

**Bilinçli sapma:** upstream v1.16 varsayılanı ±45°; burada **±57.29578° (= ±1 rad)**
kullanıldı. Böylece eski sabit eşleme birebir üretilir ve mevcut `standard_vtol`,
`advanced_plane`, `rc_cessna` airframe'lerinin kontrol yüzeyi kazançları **değişmez**
(ölçülen azami sapma: **8.5e-9 rad**). Yalnızca tiltrotor airframe'leri override eder.

### 2.2 `1941058d64` — Submodule fork'a yönlendirildi

`b87255f406` submodule'ü yalnızca yerelde var olan bir commit'e bump etmişti; dalı
klonlayan başkası için model erişilemezdi. `.gitmodules` fork'a çevrildi (HTTPS —
CI ve taze klonlar anahtarsız çekebilsin).

### 2.3 `f29b93020e` — Model-2 (tri-tiltrotor)

| Dosya | Ne yapıldı |
|---|---|
| `Tools/simulation/gz/models/tiltrotor_2plus1/model.sdf` | Yeni model (submodule `bee034b`) |
| `.../tiltrotor_2plus1/model.config` | Model kaydı |
| `ROMFS/.../airframes/4021_gz_tiltrotor_2plus1` | Model-2 airframe |
| `ROMFS/.../airframes/CMakeLists.txt` | Airframe kaydı (+1 satır) |

Model-1'e **hiç dokunulmadı**; kanat, meshler, kütle ve LiftDrag katsayıları ondan
yeniden kullanıldı. Yalnızca tahrik düzeni ve `iyy`/`izz` (kuyruk kolu için
0.146/0.148 → 0.20/0.20) değişti.

---

## 3. Model-2 neden "2+1" değil de tri-tiltrotor?

**Orijinal istek:** kanatta 2 tilt + arkada **sabit** pusher, hover yalnızca 2 tilt
motorla. **Bu konfigürasyon PX4'te hover edemez** — modelleme değil, kontrol tahsisi
sınırı:

- İki kanat rotoru da `x≈0`'da → itki farkı **pitch momenti üretmez**
- Geriye tek yol kolektif tilt — ama `ActuatorEffectivenessTiltrotorVTOL.cpp:96`
  bunu **koşulsuz kapatıyor**:
  ```cpp
  _tilts.updateTorqueSign(_mc_rotors.geometry(), true /* disable pitch to avoid configuration errors */);
  ```
  Yani `CA_SV_TL0_CT = 3` ('Yaw and Pitch') ayarlansa bile tilt pitch için kullanılmaz.
- Sabit pusher hover'da kapalı → pitch'e katkısı yok
- Kanat kontrol yüzeyleri hover'da (airspeed ≈ 0) işe yaramaz
- PX4'te bicopter airframe'i yok (`CA_AIRFRAME` 0–12 arasında yok)

**Çözüm:** arka motor da tilt eder. Hover'da üçü de yukarı bakar; arka rotor
`x = -0.65`'te olduğu için pitch **sıradan itki farkından** gelir
(`2 · T_ön · 0.25 = T_arka · 0.65`) → PX4 kod değişikliği gerekmez. Cruise'da üçü de
90°'ye döner, arka rotor pusher görevini yapar.

### Geometri (Model-2)

SDF **FLU** (X ileri, Y **sol**, Z yukarı) ↔ PX4 **FRD** (Y **sağ**, Z aşağı):
**`PY = -y_sdf`, `PZ = -z_sdf`**.

| Rotor | SDF konum | Yön | PX4 | Rol |
|---|---|---|---|---|
| `rotor_0` | `(0.25, -0.35, 0.07)` | ccw | `PX 0.25, PY 0.35, KM 0.05, TILT 1` | Kanat sağ |
| `rotor_1` | `(0.25, +0.35, 0.07)` | cw | `PX 0.25, PY -0.35, KM -0.05, TILT 2` | Kanat sol |
| `rotor_2` | `(-0.65, 0, 0.07)` | ccw | `PX -0.65, PY 0, KM 0.05, TILT 3` | Kuyruk / pusher |

Yaw: kanat rotorlarının diferansiyel tilt'i. Arka rotor merkez hattında olduğundan
tilt'i yaw momenti üretmez → `CA_SV_TL2_CT 0`.

### Servo eşlemesi (Model-2)

Topic'ler 0-indeksli, parametreler 1-indekslidir (`servo_3` ↔ `SIM_GZ_SV_*4`).

| gz topic | Joint | Param | Fonksiyon |
|---|---|---|---|
| `servo_0/1/2` | elevon'lar + elevator | `SIM_GZ_SV_FUNC1..3` | 201–203 |
| `servo_3` | `motor_0_joint` | `SIM_GZ_SV_FUNC4` | 204 (Tilt 1) |
| `servo_4` | `motor_1_joint` | `SIM_GZ_SV_FUNC5` | 205 (Tilt 2) |
| `servo_5` | `motor_2_joint` | `SIM_GZ_SV_FUNC6` | 206 (Tilt 3) |

---

## 4. Doğrulanmış davranış

Hepsi SITL'de ölçüldü; tilt değerleri **fizik motorundaki gerçek link pozundan**
okundu (komut topic'inden değil).

### Model-1 — `gz_tiltrotor`
| Test | Sonuç |
|---|---|
| MC kalkış | 9.3 m |
| FW geçişi | `vtol_state: 4`, **16.35 m/s** |
| Geri geçiş | `vtol_state: 2 → 3`, irtifa korunuyor |
| Fiziksel tilt (`motor_0` RPY pitch) | disarm `0.000` → FW `1.570` rad (**89.95°**) |
| Regresyon (Model-2 eklendikten sonra) | MC 9.8 m → FW `state 4` → **16.67 m/s** ✅ |

### Model-2 — `gz_tiltrotor_2plus1`
| Test | Sonuç |
|---|---|
| Hover | 12.2 m'de sabit (`vx=0.08`, `vy=0.05` m/s) |
| Motor dağılımı | `[0.634, 0.636, 0.553]` — ön ikisi eşit, arka düşük: geometrinin öngördüğü oran |
| FW geçişi | `vtol_state: 4`, **16.48 m/s** |
| Cruise itkisi | üç motor da eşit `0.243` → arka rotor gerçekten pusher |
| Fiziksel tilt | arka `motor_2`: **89.95°**, ön `motor_0`: **89.78°** |
| Geri geçiş | `vtol_state: 3`, hover'a dönüş |

### Bridge backport
| Kontrol | Sonuç |
|---|---|
| Geriye uyumluluk (eski↔yeni formül) | azami sapma **8.5e-9 rad** |
| Uçtan uca ölçekleme | `control[3] = -0.98850` → MixingOutput `≈6` → `1.5708 × 6/1000 = 0.00942` → gözlenen **`0.00942477822303772`** |

---

## 5. Depo durumu

```
PX4-Autopilot          gz-tiltrotor-port      f29b93020e  → fork ✓
└─ Tools/simulation/gz   px4-v1.15.4-tiltrotor  bee034b    → fork ✓
~/px4-tiltrotor-backup/  3 patch (68K)                     → yerel (Model-2 YOK, bkz. §6)
```

- **Fork'lar:** `umranmeryemkarabakal/PX4-Autopilot`, `umranmeryemkarabakal/PX4-gazebo-models`
- **Push:** her iki depoda `fork` remote'u hazır → `git push fork <dal>`
- **Taban:** `origin/release/1.15` ucu (v1.15.4). Fork'ta yalnızca bizim commit'lerimiz özgün.
- Taze klon + `git submodule update --init` testi geçti: model gerçekten iniyor.

---

## 6. Açık konular

### 6.1 ⚠️ Model-2: hover'da yaw yetkisi tek yönlü kısıtlı — **incelenecek**

**Gözlem:** hover'da tiltler `[-0.831, -1.0, -1.0]` → ön-sağ rotor **7.6° eğik**
duruyor. Bu, arka rotorun sürükleme torkunun diferansiyel tilt ile trimlenmesi.

**Sorun:** tiltler MC'de 0°'de ve joint limiti (`0 .. 1.57`) negatife izin vermiyor.
`CA_SV_TL*_MINA = 0` olduğu için hover'daki nötr tilt = 0°. Diferansiyel trim
yalnızca **pozitif yöne** açılabiliyor → yaw yetkisi asimetrik. Testlerde sorun
çıkmadı ama agresif yaw manevralarında zayıf kalabilir.

**Olası çözüm:** tilt aralığını `-15°..90°` yapmak:
- `model.sdf`: `motor_{0,1,2}_joint` → `<lower>-0.26</lower>`
- airframe: `CA_SV_TL*_MINA -15`, `SIM_GZ_SV_MINA4/5/6 -15`
- `SIM_GZ_SV_DIS4/5/6` yeniden hesaplanmalı (0° dikey için artık 500 değil)

**Dikkat:** bu, MC'deki nötr tilt pozisyonunu kaydırır (`MINA` hover pozisyonudur),
yani ayrı bir test turu ister. Ölçülecek: hover'da tilt komutu gerçekten 0°'ye mi
oturuyor, yaw step yanıtı simetrik mi.

### 6.2 Arka rotorun sürükleme torku trimlenmiyor, dengelenmiyor

Model-2'de `rotor_0` (ccw) ve `rotor_1` (cw) birbirini götürüyor, ama `rotor_2` (ccw)
tek başına net yaw torku üretiyor ve bu sürekli trim gerektiriyor (§6.1'in kaynağı).
Alternatif: arka rotoru koaksiyel karşı-dönüşlü yapmak veya `momentConstant`'ını
düşürmek. Şu an gerçekçi bir davranış (gerçek trikopterler de trimler), acil değil.

### 6.3 Yedek Model-2'yi kapsamıyor

`~/px4-tiltrotor-backup/` yamaları `1941058d64` zamanında üretildi; `f29b93020e` ve
submodule `bee034b` içinde **yok**. Fork'lar güncel olduğu için kritik değil, ama
yamaları tazelemek isterseniz:

```bash
cd ~/PX4-Autopilot && rm -f ~/px4-tiltrotor-backup/*.patch
git format-patch -o ~/px4-tiltrotor-backup origin/release/1.15..gz-tiltrotor-port
cd Tools/simulation/gz && rm -f ~/px4-tiltrotor-backup/submodule/*.patch
git format-patch -o ~/px4-tiltrotor-backup/submodule d754381..px4-v1.15.4-tiltrotor
```

### 6.4 CA parametreleri ile SDF geometrisi — Model-1'de tutarsız

Model-1'in airframe'i `CA_ROTOR0_PX 0.1515, PY 0.245` derken `model.sdf` rotorları
`(0.35, ±0.35)`'te. Bu tutarsızlık upstream'den (classic modelden) miras; uçuşu
bozmuyor çünkü tahsis yaklaşıktır. **Model-2'de bilerek düzeltildi** — CA
parametreleri SDF geometrisiyle birebir eşleşiyor. Model-1'e dokunulmadı.

### 6.5 Upstream'e katkı

Bridge backport'u v1.16'da zaten var → oraya gitmez. Gönderilebilecek özgün parça:
`SIM_GZ_SV_DIS4/5 = 0` düzeltmesi — upstream'in tiltrotor'u disarm'dayken 45°'de
duruyor, bizimki Classic'in `zero_position_disarmed=0` davranışı gibi dikey park
ediyor. Ayrıca `docs/` klasörü PX4'ün yapısında yok; PR'da ayrılmalı.

---

## 7. Öğrenilen tuzaklar

| Belirti | Gerçek sebep |
|---|---|
| `No rule to make target 'gz_tiltrotor'` | Harmonic dev kütüphaneleri eksik → `gz_bridge` sessizce atlanıyor. Teşhis: `gz_x500` de yoksa sorun tiltrotor'da değil. |
| `apt` → `NO_PUBKEY 67170598AF249743` | Anahtar mevcut ama **ASCII-armored**; apt `.gpg` yolunda binary bekler. Aynı anahtarın binary kopyasına yönlendir. |
| `PX4 server already running for instance 0` | Artakalan `/tmp/px4-sock-0`. → `rm -f /tmp/px4-sock-0` |
| `param set-default` etkisiz | Kaydedilmiş paramlar gölgeler → `rm -rf build/px4_sitl_default/rootfs/eeprom` |
| `gz topic -e` sessizce boş dönüyor | gz-transport docker0'a (`172.17.0.1`) bağlanıyor; CLI ulaşamıyor. Veri akışı sağlam — doğrulamayı `gz model` (servis çağrısı) ile yap. |
| `gz_frame_id ... not defined in SDF` | Zararsız; model `<sdf version='1.5'>` beyan ediyor. **Sürümü yükseltme** — SDF 1.7 pose frame semantiğini değiştirdi, link pozlarını bozabilir. |
| Sim yeniden başlatmada takılıyor | Artakalan `gz sim` sunucusu → `pkill -9 -f 'gz sim'` |

---

## 8. Sonraki adım önerisi

1. **§6.1'i incele** — yaw asimetrisi. Ölçüm önce: hover'da yaw step yanıtını iki
   yönde karşılaştır, gerçekten sorun mu teyit et. Sorunsa tilt aralığını `-15°..90°`
   yap ve hover nötr pozisyonunu yeniden doğrula.
2. §6.3 — yedek yamalarını tazele (ucuz).
3. İsteğe bağlı: Model-2 için mission/otonom uçuş testi (şu ana kadar yalnızca
   `commander takeoff` + `transition` ile manuel test edildi).
