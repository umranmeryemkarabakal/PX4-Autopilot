# Çalıştırma Talimatları — Tilt-Rotor VTOL Modelleri

> Üç modelin **başlatma ve uçurma adımları**. Kurulum ve sorun giderme için →
> [README.md](README.md) · Tasarım gerekçeleri ve açık işler için → [STATUS.md](STATUS.md)

| Model | Hedef | gz model adı | Tilt eden rotorlar |
|---|---|---|---|
| **Model-1** | `gz_tiltrotor` | `tiltrotor_0` | `motor_0`, `motor_2` (ön ikisi) |
| **Model-2** | `gz_tiltrotor_2plus1` | `tiltrotor_2plus1_0` | `motor_0`, `motor_1`, `motor_2` |
| **Model-3** | `gz_tiltrotor_tailplane` | `tiltrotor_tailplane_0` | `motor_0`, `motor_1`, `motor_2` |

---

## 1. Başlatma — iki yol

### Yol A: `make` hedefi (elle uçurmak için)

En basiti. Gazebo GUI açılır, model spawn olur, aynı terminalde `pxh>` konsolu gelir.

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_tiltrotor            # Model-1
make px4_sitl gz_tiltrotor_2plus1     # Model-2
make px4_sitl gz_tiltrotor_tailplane  # Model-3
```

GUI'siz (sunucu / uzak makine):

```bash
HEADLESS=1 make px4_sitl gz_tiltrotor_tailplane
```

### Yol B: `px4 -d` daemon (betikle sürmek için)

**Betikten sürüyorsanız Yol A'yı kullanmayın.** stdout tty değilken `pxh>` promptu
sonsuz yeniden çizilir ve log **dakikada yüzlerce MB** şişer. `-d` daemon modudur,
interaktif shell açmaz:

```bash
cd ~/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
setsid nohup env PX4_SIM_MODEL=gz_tiltrotor_tailplane ../../../../bin/px4 -d \
  > /tmp/px4.log 2>&1 < /dev/null & disown
```

`PX4_SIM_MODEL` airframe adıdır (`gz_` önekiyle). Hazır olduğunu doğrula:

```bash
grep -a 'Ready for takeoff' /tmp/px4.log
```

> Log ikili veri içerir → `grep -a` şart, yoksa "binary file matches" der.

Bu yol GUI **açmaz** (bridge yalnız sunucuyu başlatır). Arayüz için:

```bash
setsid nohup env DISPLAY=:0 gz sim -g > /tmp/gz_gui.log 2>&1 < /dev/null & disown
```

### Başlatmadan önce temizlik

Artakalan soket ve sunucu, simin sessizce açılmamasının en sık sebebi:

```bash
pkill -9 -f 'px4_sitl_default/bin/px4'; pkill -9 -f 'gz sim'; rm -f /tmp/px4-sock-0
```

> ⚠️ `pkill -f` kendi kabuğunuzun komut satırını da eşleştirip **oturumu öldürebilir**
> (deseni içeren komutu siz yazdığınız için). Betik içinde PID ile öldürün:
> `pgrep -f 'gz.sim.*default.sdf'` → `kill -9 <pid>`.

Airframe dosyasını değiştirdiyseniz kaydedilmiş paramlar `param set-default`'u gölgeler:

```bash
rm -rf build/px4_sitl_default/rootfs/eeprom
```

---

## 2. Uçurma

### `pxh>` konsolundan (Yol A)

```sh
commander takeoff      # MC kalkış (~10 m)
commander transition   # MC -> FW: tiltler 0° -> 90°, ~6 s
commander transition   # FW -> MC: geri geçiş
commander land
```

### Betikten / ayrı terminalden (Yol B)

Daemon'da `pxh>` yok, client binary'leri kullanılır:

```bash
export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH

px4-commander takeoff
px4-commander transition   # aynı komut hem ileri hem geri geçişi tetikler
px4-commander land
```

`transition` **toggle**'dır: MC'deyken FW'ye, FW'deyken MC'ye geçirir.

---

## 3. Ne görmeli — modele göre beklenen değerler

Hepsi SITL'de ölçüldü. Sapma varsa bir şey bozulmuş demektir.

### Ortak: fiziksel tilt açısı

Komut topic'ini değil, **fizik motorundaki gerçek link pozunu** okur (asıl doğrulama budur):

```bash
gz model -m tiltrotor_tailplane_0 -l motor_0 | grep -A2 '^  - Pose' | tail -1 \
  | awk '{gsub(/[\[\]]/,""); printf "%.2f derece\n", $2*57.29578}'
```

`$2` = RPY'nin **pitch** bileşeni. `0` = dikey (MC), `1.57 rad` = 90° (FW).

> Joint'in `<upper>` limiti 1.57 olduğu için 1.5708 yerine **89.95°**'te durur.
> 0.045°'lik bu fark upstream'in kendi SDF limiti, hata değil.

### Model-1 — `gz_tiltrotor`

| Aşama | Beklenen |
|---|---|
| MC kalkış | ~9.3 m |
| FW geçişi | `vtol_state: 4`, **16.35 m/s** |
| Fiziksel tilt (FW) | `motor_0`: **89.95°** |
| Geri geçiş | `vtol_state: 2 → 3`, irtifa korunur |

Tilt servoları `actuator_servos`'ta **`control[3]`, `control[4]`** (`-1`=dikey, `+1`=90°).

### Model-2 — `gz_tiltrotor_2plus1`

| Aşama | Beklenen |
|---|---|
| Hover | 12.2 m'de sabit (`vx≈0.08`, `vy≈0.05`) |
| Motor dağılımı (hover) | `[0.634, 0.636, 0.553]` — kanat ikilisi eşit, kuyruk düşük |
| FW geçişi | `vtol_state: 4`, **16.48 m/s** |
| Cruise itkisi | üç motor da eşit `0.243` → arka rotor pusher |
| Fiziksel tilt (FW) | `motor_2`: **89.95°**, `motor_0`: **89.78°** |

Tilt servoları **`control[3]`, `control[4]`, `control[5]`**.

### Model-3 — `gz_tiltrotor_tailplane`

| Aşama | Beklenen |
|---|---|
| Hover | 10.6–11.0 m'de sabit (`vx`, `vy` < 0.1) |
| Motor dağılımı (hover) | `[0.640, 0.645, 0.538]` — kanat ikilisi eşit, kuyruk düşük |
| Hover tiltleri | `[~9°, 0°, 0°]` — **bir kanat rotoru eğik durur, bkz. aşağıdaki not** |
| FW geçişi | `vtol_state: 4`, **~17 m/s** |
| Cruise itkisi | `[0.267, 0.267, 0.267]` — üçü **birebir eşit** |
| Fiziksel tilt (FW) | üçü de **89.95°** |
| Eksen ayrımı (cruise) | elevon `±0.0004`, iki elevator **eşit** `-0.053`, rudder `0.176` |

Tilt servoları **`control[5]`, `control[6]`, `control[7]`** (Model-3'te 8 servo var:
`[0-4]` yüzeyler, `[5-7]` tiltler).

> **Hover'da bir rotorun eğik durması normaldir.** Arka rotor tek ccw olduğu için
> dengelenmemiş sürükleme torku üretir; araç bunu bir kanat rotorunu eğerek trimler
> (Model-3'te ~9°, Model-2'de 7.8° ölçüldü). Ölçüldü ve zararsız bulundu →
> STATUS.md §6.1/§6.2. Cruise'da bu tork rudder'a geçer ve tiltler serbest kalır —
> Model-3'ün üç tilti de tam 90°'de durmasının sebebi budur.

---

## 4. İzleme komutları

```bash
export PATH=~/PX4-Autopilot/build/px4_sitl_default/bin:$PATH

px4-listener vtol_vehicle_status   # vehicle_vtol_state: 3=MC, 4=FW, 1/2=geçiş anı
px4-listener actuator_servos       # yüzeyler + tiltler
px4-listener actuator_motors       # itki dağılımı
px4-listener vehicle_local_position
px4-listener airspeed_validated
```

İki tuzak:

- **`px4-listener <topic> -n N` takılır** — araç disarm'dayken konu yeniden yayınlanmaz
  ve `-n` yeni yayın bekler. Anlık değer için **argümansız** çağırın. Sürekli kayıt
  gerekiyorsa `listener` yerine uçuş ULog'unu kullanın
  (`build/px4_sitl_default/rootfs/log/…/*.ulg`, `pyulog`).
- **`gz topic -e` sessizce boş döner** — gz-transport docker0'a (`172.17.0.1`) bağlanır,
  CLI ulaşamaz. Veri akışı sağlamdır; doğrulamayı `gz model` / `gz service` gibi
  **servis çağrılarıyla** yapın.

---

## 5. Kamera — aracı takip etme

İleri uçuşta araç görüş alanından çıkar. GUI'den: Entity Tree → modele sağ tıkla →
**Follow**. Komutla:

```bash
# aracı takip et
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: "tiltrotor_tailplane_0"'

# arkadan-üstten
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'x: -6, y: 0, z: 2'

# yandan — tilt açısını izlemek için en iyisi
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'x: 0, y: -4, z: 0.5'

# takibi bırak
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: ""'
```

---

## 6. Tilt süpürmesini yavaşlatarak izleme

Geçiş ~6 saniyede biter; tiltlerin dönüşünü gözle takip etmek için sim yavaşlatılabilir.
PX4 **sim saatini** kullandığı için uçuş davranışı değişmez, yalnızca duvar saatinde
yayılır:

```bash
# 0.15x — geçiş ~6 s yerine ~40 s sürer
gz service -s /world/default/set_physics --reqtype gz.msgs.Physics \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'real_time_factor: 0.15'

# normale dön
gz service -s /world/default/set_physics --reqtype gz.msgs.Physics \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'real_time_factor: 1.0'
```

`data: true` dönmesi ayarın **uygulandığını kanıtlamaz**. Gerçekten ölçmek için sim
saatiyle duvar saatini karşılaştırın:

```bash
t1=$(px4-listener vehicle_local_position | grep -m1 timestamp: | awk '{print $2}'); w1=$(date +%s.%N)
sleep 5
t2=$(px4-listener vehicle_local_position | grep -m1 timestamp: | awk '{print $2}'); w2=$(date +%s.%N)
echo "RTF = $(echo "scale=3; (($t2-$t1)/1000000)/($w2-$w1)" | bc)"
```

### Yavaşlatınca ne görünür

0.15x'te ölçülen ileri geçiş (Model-3), tilt açısı ve airspeed:

| tilt | airspeed |
|---|---|
| 11.5° | 0.7 |
| 22.0° | 1.4 |
| 34.2° | 5.8 |
| 44.8° | 9.7 |
| 63.9° | 13.8 |
| **90.0°** | 15.8 → `state=4` |

**İleri geçiş monoton bir rampadır** — araç hızlanırken tiltler programlı şekilde yatar.
63.9°'den 90°'ye sıçrama `VT_TILT_TRANS 0.6` parametresidir: geçiş rampası ~54°'de
tutulur, araç geçiş hızına ulaşınca kalan yol tek seferde tamamlanır. Bug değil.

**Geri geçiş salınımlı görünür** ve bu da normaldir: kapalı döngü bir manevradır
(17 m/s'den fren + dengede kalma), tiltler yaw için aktif kullanılır. Ayrıca yavaşlatılmış
simi sık örneklerseniz her örnek arası yalnızca ~0.6 sim saniyesidir, yani manevranın
en hareketli anına mikroskopla bakarsınız. Bu sırada **airspeed negatif okunabilir** —
araç yavaşlamak için burnunu kaldırır ve kısa süre havaya göre geriye gider; pitot bunu
negatif okur, arıza değildir.

---

## 7. Kapatma

```bash
pkill -f 'bin/px4 -d'; pkill -f 'gz sim'; rm -f /tmp/px4-sock-0
```

Soketi silmezseniz bir sonraki başlatma `PX4 server already running for instance 0`
der ve açılmaz.
