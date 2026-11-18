# Örnek Senaryo: Yavaş Giden Aracın Arkasına Takılmak

Bu senaryo, [mimari dokümanındaki](../architecture.md) veri akışının uçtan uca nasıl işlediğini gösterir.
Sayılar örnektir; `behavior_planner_node.py` içindeki `decide()` formülleriyle tutarlıdır.
Akış CARLA ile çalıştırılarak doğrulanmamıştır.

**Durum:** Ego araç `Town03` haritasında 8 m/s ile gidiyor. Önünde daha yavaş bir araç var. Kullanıcı RViz'de bir hedef işaretliyor.

## Başlangıç: Hedef verilir

| # | Kim | Ne yapıyor | Topic |
|---|---|---|---|
| 1 | Kullanıcı (RViz) | "2D Goal Pose" ile hedef işaretler | `/goal_pose` |
| 2 | `path_planner_node` | Nav2'den rota ister (action) | `compute_path_to_pose` |
| 3 | Nav2 | Costmap'e bakıp rotayı hesaplar (costmap'i LiDAR besler) | `/perception/lidar/nonground` |
| 4 | `path_planner_node` | Rotayı yayınlar | `/planning/global_path` |

## Tur 1: Önde yavaş bir araç var (30 m)

**A. Köprü.** `carla_bridge_node` simülatörü bir adım ilerletir ve sensör verisini yayınlar:
`/carla/ego/camera/front/image_raw`, `/carla/ego/lidar/points`, `/carla/ego/radar/front`, `/carla/ego/odometry`.

**B. Algılama.** Üç sensör aynı nesne için farklı şeyler söyler:

| Sensör | Ne söylüyor |
|---|---|
| `yolo_detector_node` | "Önde `car` var, güven %91. Mesafe yok, yalnızca açı (~0°)" |
| `lidar_processor_node` | "x = 30 m, y = 0.1 m'de bir küme var, sınıfı `unknown`" |
| `radar_processor_node` | "Aynı yönde nesne, radyal hız −2 m/s (yaklaşıyoruz)" |

**C. Füzyon.** `kalman_fusion_node` LiDAR kümesinden iz oluşturur, radarla hızı düzeltir, kameranın etiketini açıya göre eşler:

```
/fusion/tracked_objects → { id: 7, class: "car", x: 30, y: 0.1, vx: -2 }
```

**D. Davranış kararı.** `decide()` ego hızı 8 m/s, mesafe 30 m, bağıl hız −2 m/s için:

```
1. Koridorda mı?        |y| = 0.1 < 1.8                       → evet
2. Çarpma süresi        30 / 2 = 15 sn  (> 1.5 sn)             → acil durum yok
3. Yaya / dur tabelası? car                                    → hayır
4. İstenen mesafe       5 + 8 × 2 = 21 m
   30 m < 2 × 21 m                                            → araca takıl
5. Öndeki aracın hızı   8 - 2 = 6 m/s
   hedef hız            min(8, 6 × min(1, 30/21)) = 6 m/s
```

```
/planning/behavior_state → FOLLOW_VEHICLE, target_speed = 6.0, lead_object_id = 7
```

**E. Kontrol.** `vehicle_controller_node`:
- Pure Pursuit: yolda ilerideki bir noktaya bakarak direksiyonu hesaplar. Bakış mesafesi `max(4, 0.8 × 8)` = 6.4 m.
- PID: hedef 6, gerçek 8 m/s. Hata negatif olduğu için gazı keser, gerekirse fren verir.

```
/control/vehicle_cmd → { throttle: 0, steer: 0.03, brake: 0.2 }
```

**F. Döngü kapanır.** `carla_bridge_node` komutu CARLA'daki araca uygular, simülasyon bir adım ilerler ve akış A adımından yeniden başlar.

## Tur N: Öndeki araç aniden duruyor

Ego hızı 6 m/s, öndeki araç duruyor, mesafe 8 m. Radar şimdi −6 m/s ölçer, füzyon `vx = -6` yayınlar.

```
çarpma süresi = 8 / 6 = 1.33 sn  <  1.5 sn   → EMERGENCY_BRAKE, target_speed = 0
```

`decide()` önce çarpma süresini kontrol eder. Bu yüzden mesafe hâlâ "takip" aralığında olsa da acil fren, takip kararının önüne geçer.

## Akış özeti

```
Tıklama ─► /goal_pose ─► path_planner ◄─► Nav2 ─► /planning/global_path
                                                        │
 CARLA ─► sensörler ─► perception (3 düğüm)             │
                           │                             │
                           ▼                             │
                    /fusion/tracked_objects              │
                           │                             │
                           ▼                             ▼
                    behavior_planner ──► /planning/behavior_state
                                                  │
                                                  ▼
                                     vehicle_controller (PP + PID)
                                                  │
                    CARLA ◄── carla_bridge ◄── /control/vehicle_cmd
```

## Kapsam dışı

Bu senaryoda şerit değiştirme yoktur. `LANE_CHANGE_*` durumları `BehaviorState` mesajında tanımlıdır, ancak davranış planlayıcıda henüz kullanılmaz.
