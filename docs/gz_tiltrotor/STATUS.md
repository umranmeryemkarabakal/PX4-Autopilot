# Proje Durumu — Gazebo Harmonic VTOL Portu

> Bu dosya bir **devir/devam noktası**dır. Çalışmaya buradan devam edilir.
> Kullanım talimatları için [README.md](README.md).
>
> Son güncelleme: 2026-07-16 · Dal: `gz-tiltrotor-port` · Son commit: `d15969691c`

---

## 1. Nerede kaldık

PX4 v1.15.4 üzerinde Gazebo Harmonic ile çalışan **üç bağımsız VTOL modeli** var.
Üçü de SITL'de uçuruldu ve doğrulandı. Açık iş kalemleri için → [§6](#6-açık-konular).

| Model | Hedef | Konfigürasyon | Durum |
|---|---|---|---|
| **Model-1** | `gz_tiltrotor` | 4 rotor; ön 2'si tilt, arka 2'si sabit dikey | ✅ Uçuyor |
| **Model-2** | `gz_tiltrotor_2plus1` | 3 rotor; kanatta 2 tilt + kuyrukta 1 tilt (cruise'da pusher) | ✅ Uçuyor |
| **Model-3** | `gz_tiltrotor_tailplane` | Model-2'nin tahrik düzeni + klasik kuyruk (2 elevator + rudder) | ✅ Uçuyor |

```bash
make px4_sitl gz_tiltrotor            # Model-1
make px4_sitl gz_tiltrotor_2plus1     # Model-2
make px4_sitl gz_tiltrotor_tailplane  # Model-3
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

### 2.4 `d15969691c` — Model-3 (kuyruklu tri-tiltrotor)

| Dosya | Ne yapıldı |
|---|---|
| `Tools/simulation/gz/models/tiltrotor_tailplane/model.sdf` | Yeni model (submodule `debcdee`) |
| `.../tiltrotor_tailplane/model.config` | Model kaydı |
| `ROMFS/.../airframes/4022_gz_tiltrotor_tailplane` | Model-3 airframe |
| `ROMFS/.../airframes/CMakeLists.txt` | Airframe kaydı (+1 satır) |

Model-1 ve Model-2'ye **hiç dokunulmadı**; ikisi de uçmaya devam ediyor. Paylaşılan
hiçbir kod değişmedi — CMakeLists'teki tek satırlık kayıt dışında Model-3 tamamen
kendi dosyalarında.

**Neden var:** Model-2 roll ve pitch'i kanat elevon'larına karıştırıyor ve **hiç yaw
yüzeyi yok** — cruise'da yaw rotorlara kalıyor. Model-3 ileri uçuş kontrolünü
eksenlere ayırıyor: elevon'lar yalnız roll, iki elevator pitch, rudder yaw. Yani
klasik bir uçağın yaptığı şey; Model-3'ü diğer ikisinin yanında anlamlı kılan da bu.

Hover ilkesi Model-2 ile **aynı**: pitch hâlâ kanat ikilisi ile arka rotor arasındaki
itki farkından geliyor → `ActuatorEffectivenessTiltrotorVTOL`'ün kapalı tilt-pitch'ine
hiç ihtiyaç yok, kontrol tahsisi değişmiyor.

**Kanat rotorları neden `y = ±0.25`?** (Model-2'de `±0.35`.) Yeniden kullanılan
`x8_wing` meshi `x = 0.22`'ye kadar ancak bu dış istasyonda uzanıyor. Yani rotorları
pitch kolundan (`PX = 0.22`) ödün vermeden kanadın **altında** tutan konum burası.

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

### Geometri (Model-3)

Aynı FLU↔FRD dönüşümü (`PY = -y_sdf`, `PZ = -z_sdf`). CA parametreleri SDF ile
**birebir** eşleşiyor (Model-2'deki gibi, Model-1'in aksine — bkz. §6.4).

| Rotor | SDF konum | Yön | PX4 | Rol |
|---|---|---|---|---|
| `rotor_0` | `(0.22, -0.25, -0.06)` | ccw | `PX 0.22, PY 0.25, PZ 0.06, KM 0.05, TILT 1` | Kanat sağ |
| `rotor_1` | `(0.22, +0.25, -0.06)` | cw | `PX 0.22, PY -0.25, PZ 0.06, KM -0.05, TILT 2` | Kanat sol |
| `rotor_2` | `(-0.65, 0, 0.07)` | ccw | `PX -0.65, PY 0, PZ -0.07, KM 0.05, TILT 3` | Kuyruk / pusher |

Hover pitch dengesi: `2 · T_ön · 0.22 = T_arka · 0.65` → kabaca ağırlığın %37'si her
kanat rotorunda, %25'i kuyrukta. `PZ` bu dengeye girmez (yalnız `PX`'e bağlı).

Yaw yine kanat rotorlarının diferansiyel tilt'i; arka rotor merkez hattında →
`CA_SV_TL2_CT 0`. Pitch **bilerek** tilt'lere tahsis edilmedi (`CT 2/3` yok): PX4
tilt-pitch'i zaten kapatıyor ve bu airframe'in ihtiyacı yok.

Aero yüzeyleri (5 LiftDrag, hepsi `base_link`'e bağlı, `cp`): kanat `(-0.05, ±0.3, 0.05)`,
elevator'lar `(-0.70, ±0.15, -0.04)`, fin `(-0.74, 0, 0.12)`.

### Servo eşlemesi (Model-3)

Model-2'den farklı olarak **8 servo** var (5 yüzey + 3 tilt). Tilt'ler `SIM_GZ_SV_*6/7/8`.

| gz topic | Joint | Param | Fonksiyon |
|---|---|---|---|
| `servo_0` | `left_elevon_joint` | `SIM_GZ_SV_FUNC1` | 201 (Aileron, `TRQ_R -0.5`) |
| `servo_1` | `right_elevon_joint` | `SIM_GZ_SV_FUNC2` | 202 (Aileron, `TRQ_R 0.5`) |
| `servo_2` | `left_elevator_joint` | `SIM_GZ_SV_FUNC3` | 203 (Elevator, `TRQ_P 0.5`) |
| `servo_3` | `right_elevator_joint` | `SIM_GZ_SV_FUNC4` | 204 (Elevator, `TRQ_P 0.5`) |
| `servo_4` | `rudder_joint` | `SIM_GZ_SV_FUNC5` | 205 (Rudder, `TRQ_Y 1`) |
| `servo_5` | `motor_0_joint` | `SIM_GZ_SV_FUNC6` | 206 (Tilt 1) |
| `servo_6` | `motor_1_joint` | `SIM_GZ_SV_FUNC7` | 207 (Tilt 2) |
| `servo_7` | `motor_2_joint` | `SIM_GZ_SV_FUNC8` | 208 (Tilt 3) |

İki elevator'ın her biri pitch torkunun **yarısını** alıyor → ikisi toplamda
Model-2'nin tek elevator'ının yetkisini veriyor. Tilt servolarında `MINA/MAXA = 0/90`
ve `DIS = 0` (disarm'da dikey park) override'ları şart — yoksa bridge ±57.29578°
varsayılanına düşer ve geçiş tamamlanmaz (bkz. §2.1).

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

### Model-3 — `gz_tiltrotor_tailplane`
| Test | Sonuç |
|---|---|
| Hover | 10.6 m'de sabit (`vx=-0.07`, `vy=-0.04` m/s) |
| Motor dağılımı | `[0.640, 0.645, 0.538]` — kanat ikilisi eşit, kuyruk düşük: geometrinin öngördüğü oran |
| FW geçişi | `vtol_state: 4`, **16.98 m/s** |
| Cruise itkisi | üç motor da eşit `0.268` → arka rotor gerçekten pusher |
| Fiziksel tilt (FW) | `motor_0` **89.81°**, `motor_1` **89.67°**, `motor_2` **89.95°** |
| Eksen ayrımı (cruise) | elevon `±0.0008` (roll≈0), iki elevator **eşit** `-0.055` (pitch), rudder `0.155` (yaw) |
| Geri geçiş | `vtol_state: 3`, hover'a dönüş, tiltler 0°'ye |
| İniş | 1.06 m'de disarm |

Cruise satırı Model-3'ün varlık sebebini doğruluyor: yüzeyler eksen başına ayrışıyor,
yaw'ı rudder taşıyor (Model-2'de bu yüzey yok).

Hover'da `motor_0` **9.6°** tilt trimi taşırken diğer ikisi 0° limitinde — Model-2'nin
7.8°'lik trimiyle **aynı olgu** (§6.1/§6.2), beklenen davranış.

**Regresyon:** Model-1/2 yeniden test edilmedi; gerekmedi. Model-3 paylaşılan hiçbir
dosyaya dokunmuyor (tek istisna CMakeLists'e eklenen kayıt satırı).

### Bridge backport
| Kontrol | Sonuç |
|---|---|
| Geriye uyumluluk (eski↔yeni formül) | azami sapma **8.5e-9 rad** |
| Uçtan uca ölçekleme | `control[3] = -0.98850` → MixingOutput `≈6` → `1.5708 × 6/1000 = 0.00942` → gözlenen **`0.00942477822303772`** |

---

## 5. Depo durumu

```
PX4-Autopilot          gz-tiltrotor-port      d15969691c  → yerel (push edilmedi)
└─ Tools/simulation/gz   px4-v1.15.4-tiltrotor  debcdee    → yerel (push edilmedi)
~/px4-tiltrotor-backup/  3 patch (68K)                     → yerel (Model-2/3 YOK, bkz. §6.3)
```

> ⚠️ **Model-3 commit'leri (`d15969691c` + submodule `debcdee`) henüz push edilmedi.**
> Submodule'ü **önce** push edin, yoksa ana depodaki bump erişilemez bir commit'i
> gösterir (bkz. §2.2 — aynı hata bir kez yapıldı):
> ```bash
> cd Tools/simulation/gz && git push fork px4-v1.15.4-tiltrotor
> cd ~/PX4-Autopilot     && git push fork gz-tiltrotor-port
> ```

- **Fork'lar:** `umranmeryemkarabakal/PX4-Autopilot`, `umranmeryemkarabakal/PX4-gazebo-models`
- **Push:** her iki depoda `fork` remote'u hazır → `git push fork <dal>`
- **Taban:** `origin/release/1.15` ucu (v1.15.4). Fork'ta yalnızca bizim commit'lerimiz özgün.
- Taze klon + `git submodule update --init` testi geçti: model gerçekten iniyor.

---

## 6. Açık konular

### 6.1 ✅ Model-2: hover'da yaw asimetrisi — **ölçüldü, zararsız çıktı; tilt aralığı değiştirilmedi**

Önceki not, tiltlerin 0° tabanına dayanmasının yaw yetkisini tek yönlü kısıtladığından
şüpheleniyor ve tilt aralığını `-15°..90°` yapmayı öneriyordu. **Ölçüm bunu
doğrulamadı** — asimetri gerçek ama yalnızca geçici rejimde; kapalı çevrim takibi
her iki yönde eşit. Aralık **olduğu gibi bırakıldı**.

**Nasıl ölçüldü:** hover'da offboard pozisyon setpoint'i ile ±30° ve ±120° yaw adımı;
yanıt `ATTITUDE`'dan (jiroskop `yawspeed`, türev değil), tilt komutları uçuş
ULog'undaki `actuator_servos`'tan okundu.

**Mekanizma doğrulandı.** Hover trimi:

| Tilt | Komut | Açı |
|---|---|---|
| Tilt1 (sağ kanat) | `-0.827` | **7.8°** |
| Tilt2 (sol kanat) | `-1.000` | **0.0°** (limitte) |
| Tilt3 (kuyruk) | `-1.000` | **0.0°** (limitte) |

Yaw daima **bir kanat rotoru kaldırılarak** üretiliyor; diğeri adımın ~%97'sinde
0°'ye çakılı kalıyor. Yani 0° tabanı gerçekten bağlayıcı — ama her iki yön de kendi
rotorunu kaldırarak tork üretebildiği için **yetki kaybı yok**. Üst limit (90°) hiç
görülmedi: 120°'lik adımda bile azami tilt **45°**, yani aralığın yarısı kullanılmıyor.

**Sonuçlar** (her iki adım da hedefe oturdu):

| Adım | Tepe yaw hızı | t90% | Aşım | Kalıcı hata | İrtifa sapması |
|---|---|---|---|---|---|
| +30° | 0.88 rad/s (50.6°/s) | 0.94 s | %2.6 | +0.04° | 0.08 m |
| −30° | 1.11 rad/s (63.6°/s) | 0.96 s | %3.5 | −0.05° | 0.13 m |
| +120° | 2.37 rad/s (135.8°/s) | 1.22 s | %0.7 | +0.23° | 0.12 m |
| −120° | 3.30 rad/s (189.0°/s) | 1.20 s | %0.4 | +0.02° | 0.12 m |

**Bulgu:** tepe yaw hızı yönler arasında tutarlı biçimde **%20–28** farklı (negatif yön
hep daha hızlı; iki bağımsız ±30° koşusunda 0.88/1.11 ve 0.84/1.17 rad/s). Sebep,
7.8°'lik trim yanlılığı + yaw torkunun tilt açısında doğrusal olmaması
(`τ ∝ sin(a)`, ama tahsis doğrusal varsayıyor).

Buna karşılık **t90% farkı %2'nin altında**, aşım her iki yönde de küçük, kalıcı hata
derece-altı, irtifa 0.13 m içinde korunuyor. Yani fark yalnızca geçici tepe hızda;
takip performansı simetrik. `-15°..90°` değişikliği nötr tilt pozisyonunu kaydırıp yeni
bir test turu gerektirirdi ve ölçülen bir eksiği kapatmıyor → **yapılmadı**.

**Ne zaman geri dönülmeli:** yaw hızı doyuma ulaşan bir senaryo çıkarsa (azami tilt
90°'ye dayanırsa) veya agresif yaw'da pozitif yön yetersiz kalırsa. Ölçüm scriptleri
tekrar üretilebilir: hover'da offboard yaw adımı + `actuator_servos`'u ULog'dan oku.

### 6.2 Arka rotorun sürükleme torku trimlenmiyor, dengelenmiyor

Model-2'de `rotor_0` (ccw) ve `rotor_1` (cw) birbirini götürüyor, ama `rotor_2` (ccw)
tek başına net yaw torku üretiyor ve bu sürekli trim gerektiriyor (§6.1'deki 7.8°'lik
trim yanlılığının kaynağı).
Alternatif: arka rotoru koaksiyel karşı-dönüşlü yapmak veya `momentConstant`'ını
düşürmek. Şu an gerçekçi bir davranış (gerçek trikopterler de trimler), acil değil.

**Model-3 aynı rotor düzenini miras aldı → aynı olgu var** (hover trimi 9.6°, §4). Fark:
Model-3'ün rudder'ı var, yani cruise'da tork rudder'a gidiyor (ölçülen `0.155`) ve
tiltler serbest kalıyor. Hover'da rudder işe yaramaz (airspeed ≈ 0), orada durum
Model-2 ile birebir aynı. §6.1'in ölçümü Model-3'te **tekrarlanmadı**; mekanizma ortak
olduğu için sonucun taşınması bekleniyor, ama doğrulanmadı.

### 6.3 Yedek Model-2 ve Model-3'ü kapsamıyor

`~/px4-tiltrotor-backup/` yamaları `1941058d64` zamanında üretildi; `f29b93020e`,
`d15969691c` ve submodule `bee034b`/`debcdee` içinde **yok**. Model-3 commit'leri
**henüz push de edilmedi** (§5), yani şu an yalnızca bu makinede duruyor — Model-2'nin
aksine fork yedeği yok. Yamaları tazelemek için:

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
| `make px4_sitl` logu dakikada yüzlerce MB | stdout tty değilken `pxh>` promptu sonsuz yeniden çiziliyor. Betikle sürerken doğrudan `px4 -d` çalıştır (daemon, interaktif shell yok): `cd build/px4_sitl_default/src/modules/simulation/gz_bridge && PX4_SIM_MODEL=gz_tiltrotor_2plus1 ../../../../bin/px4 -d`. Log ikili veri içerdiği için `grep -a` gerekir. |
| Adım yanıtı ölçümü yönler arası asimetrik görünüyor | pymavlink alım tamponu boşaltılmazsa bekleme fazında biriken `ATTITUDE`'lar sonraki adımın **ilk örnekleri** olarak okunuyor → bir önceki manevra yeni adıma karışıyor. Her bekleme döngüsünde `recv_match(blocking=False)` ile tamponu boşalt; adımın gerçekten hedef açıdan başladığını doğrula. |
| `px4-listener <topic> -n N` takılıyor | Araç disarm'dayken konu yeniden yayınlanmıyor; `-n` yeni yayın bekler. Tek anlık değer için argümansız çağır — sürekli kayıt için `listener` yerine uçuş **ULog**'unu (`rootfs/log/…​.ulg`, `pyulog`) kullan. |

---

## 8. Sonraki adım önerisi

1. ~~§6.1'i incele~~ — **yapıldı**, asimetri ölçüldü ve zararsız çıktı; tilt aralığı
   değişmedi. Ayrıntı → [§6.1](#61--model-2-hoverda-yaw-asimetrisi--ölçüldü-zararsız-çıktı-tilt-aralığı-değiştirilmedi).
2. **Model-3'ü push et** (§5) — şu an yalnızca yerelde, yedeği yok. Önce submodule.
3. §6.3 — yedek yamalarını tazele (ucuz).
4. İsteğe bağlı: Model-2/3 için mission/otonom uçuş testi (şu ana kadar yalnızca
   `commander takeoff` + `transition` + `land` ile manuel test edildi, artı §6.1'in
   offboard yaw adımları).
5. İsteğe bağlı: Model-3'ün rudder'ının cruise'da gerçekten yaw yetkisi verdiğini
   ölçülü doğrula (şu an yalnızca trim değeri `0.155` gözlendi; adım yanıtı alınmadı).
