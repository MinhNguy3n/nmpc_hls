# Reely NEW1 (brushed, 1:10, 4WD): vehicle parameters

**Status (2026-11-21):** collected from the manufacturer documentation. Nothing has been measured yet. Every value below is either **spec, unverified** or `PLACEHOLDER_`. None comes from the Husky or from F1TENTH cars.

**Vehicle:**
- Reely NEW1 1:10 electric monster truck, 4WD, RtR;
- Conrad item **1559975** ("Super Combo-Set": the vehicle plus a 6-cell NiMH drive battery and a NiMH charger);
- OEM name FS Racing FS-53692, which no public source found confirms.

## Sources

| Id | Source | What was read | Notes |
|---|---|---|---|
| M | Reely "1:10 Elektro-Monstertruck" operating manual. The title page names **item no. 1551069** and **item no. 1559975 (Super Combo-Set)**. | Technical data, DE p. 27 and EN p. 53; scope of delivery, p. 11; troubleshooting, EN p. 52. Read through the ManualsLib copy (manualslib.de, manual 146780), because conrad.com refuses automated fetches (HTTP 403). | The user's manual link (`asset.conrad.com/…/001551069ML02/…`) carries the same item number 1551069. It is probably this manual, but whether the ManualsLib copy is the same edition (ML02) is **not verified**. The user should compare it with the PDF they have. |
| C | Conrad product listing 1559975 | Only as quoted by a search engine; the page itself was not readable (403) | Use only where M gives nothing, and flag it |
| — | Reely NEW1 **brushless** manual and **kit** manual (ManualsLib 190839, 439378) | Technical data | **Other variants. Not used for values.** They give the same dimensions, but other weights (2310 g brushless, 1798 g kit) and other ESCs. |

The manual says: "Small deviations in dimensions and weight are due to production technical reasons."

## A. Manufacturer data (spec, unverified)

| Item | Value | Source | Status |
|---|---|---|---|
| Scale | 1:10 | M | spec, unverified |
| Dimensions L × W × H | 440 × 340 × 210 mm | M | spec, unverified |
| **Wheelbase** | **275 mm** | M (also C) | spec, unverified. The "~285 mm" in the user's pasted advice disagrees and is unverified; measure (sheet item 1) |
| Ground clearance | 50 mm | M | spec, unverified |
| Tyres W × Ø | 70 × 125 mm (nominal, unloaded) | M | spec, unverified. Nominal radius 62.5 mm; the loaded rolling radius is smaller (sheet item 3) |
| Mass | 2215 g **without** drive battery | M | spec, unverified. The battery adds `PLACEHOLDER_` g (weigh it) |
| Drivetrain | 4WD via cardan shaft; differentials front and rear | M | spec, unverified |
| Suspension | independent, double wishbones; oil shock absorbers with coil springs, adjustable; front toe and front/rear camber adjustable | M | spec. The toe and camber settings affect the effective steering and must be recorded as set (sheet item 9) |
| Motor | electric motor "type 550" (brushed, per the product name and C) | M, C | spec, unverified |
| ESC | continuous current 40 A forward, 20 A reverse; 180 A / 90 A for 1 s; BEC 5 V DC, 2 A; overtemperature protection about +100 °C | M | spec, unverified. Brake, drag-brake and reverse behaviour are **not given** in M |
| Drive batteries allowed | 2S LiPo (7.4 V nominal), 6-cell NiMH (7.2 V), 7-cell NiMH (8.4 V) | M | spec. Item 1559975 ships with a 6-cell NiMH pack (M p. 11) |
| Radio | 2.4 GHz; the receiver must be bound to the transmitter | M | spec. Channel count and fail-safe behaviour are not given (sheet item 11) |
| Top speed | 35 km/h | C only (search-engine quote) | **unverified, low confidence**: not in M |
| Steering servo | 3-wire; 50 Hz, 1000–2000 µs pulses | the RC-car prompt (user's description), **not in M** | `PLACEHOLDER_` until measured. The model, torque, speed and travel are not given anywhere |
| Gear ratio (pinion / spur / diff) | not given | — | `PLACEHOLDER_` (count the teeth, sheet item 10) |

## B. Model parameters (Ackermann note §13 rows)

| Parameter | Symbol | Unit | Value now | Where it will come from |
|---|---|---|---|---|
| Wheelbase | `L` | m | 0.275 (spec, unverified) | sheet item 1 |
| Front track (between steering pivots) | `W_f` | m | `PLACEHOLDER_`. Not given; overall width 340 mm minus one tyre width 70 mm suggests about 0.27 m between tyre centres, **an estimate, not a pivot distance** | sheet item 2 |
| Rear track | `W_r` | m | `PLACEHOLDER_` (same remark) | sheet item 2 |
| Steering travel left / right (per wheel and bicycle-equivalent) | `delta_max,L`, `delta_max,R` | rad | `PLACEHOLDER_`. Not given | sheet item 4 |
| Minimum turning radius | `R_min` | m | `PLACEHOLDER_`. Not given | sheet item 5 |
| Steering-rate limit | `delta_rate_max` | rad/s | `PLACEHOLDER_` | sheet item 7 (rough); step identification after 27 Nov |
| Servo pulse → wheel angle map, deadband, backlash | — | — | `PLACEHOLDER_` | sheet item 6 |
| Forward speed limit | `v_max` | m/s | `PLACEHOLDER_` (a safety-plan cap). The 35 km/h top speed is not a usable limit | safety plan |
| Reverse, dwell | `v_min` | m/s, s | `PLACEHOLDER_`. The stock ESC's reverse behaviour is not given | VESC configuration later; forward-only by default |
| Acceleration / deceleration | `accel_max`, `decel_max` | m/s² | `PLACEHOLDER_` | identification with the VESC (after 27 Nov) |
| Lateral-acceleration limit or μ | `a_y,max` | m/s² | `PLACEHOLDER_` | surface tests; speed sweep in phase 4 (simulation only) |
| Speed and steering lag, dead time | `tau_v`, `tau_delta`, `T_d` | s | `PLACEHOLDER_` | step responses (Ackermann note §14 S3) |
| Minimum controllable speed, command resolution | — | m/s, rad | `PLACEHOLDER_` | VESC tests |
| Command interface | — | — | Simulation: `AckermannDriveStamped` (speed, steering angle). Vehicle: VESC ERPM + servo; `PLACEHOLDER_` | hardware decision (the user) |
| Command watchdog | `T_wd` | s | `PLACEHOLDER_` (the prompt suggests 0.25–0.5 s for the simulation mux) | safety plan; verify by test |
| Pose reference offset (rigid-body pivot → rear-axle midpoint) | `d` | m | `PLACEHOLDER_` | sheet item 8 and the OptiTrack rigid body |
| Pose, speed and steering rates and latencies | — | Hz, s | `PLACEHOLDER_` | OptiTrack and VESC setup |
| Control period | `Ts` | s | 0.05 (system design, as for the Husky; not a vehicle value) | design |
| Wheel radius, encoder resolution | `r_w` | m | 0.0625 nominal, unloaded (spec, unverified); encoder `PLACEHOLDER_` | sheet item 3; sensored motor (later) |

---

## C. Measurement sheet for the user

**Safety:**
- Agents never command the car.
- For items 4, 6 and 7, put the car on a stand with all four wheels off the ground, and **unplug the motor from the ESC**, so only the steering servo is powered.
- For item 5, push the car by hand at full lock with the motor unplugged. Driving the circle is optional, at the lowest speed, in a clear area, with you holding the transmitter.

**Setup:**
- Record the date, the drive battery used (type and voltage when measured) and the car's configuration (body on or off, toe and camber as delivered).
- Take a photo of each setup.

| # | Item | How | Result |
|---|---|---|---|
| 1 | **Wheelbase `L`** | Front axle centre to rear axle centre, left and right sides, wheels straight. Two readings each | `PLACEHOLDER_` mm (L / R) |
| 2 | **Front and rear track** | Tyre centre to tyre centre at the contact patch, front and rear. Front also: distance between the steering pivots (kingpin axes), if accessible | front `PLACEHOLDER_` mm, front pivots `PLACEHOLDER_` mm, rear `PLACEHOLDER_` mm |
| 3 | **Wheel radius, loaded** | Battery in, car on the floor. Mark a tyre and the floor, push the car straight for 5 full turns of that wheel, measure the distance: `r = d / (10π)`. Repeat 3 times | `PLACEHOLDER_` mm |
| 4 | **Full-lock wheel angles** | On the stand, with the transmitter trim centred: full left, then full right. Measure each front wheel's angle to the straight-ahead line (a protractor on a straight edge held against the wheel, or a photo from straight above with a grid sheet underneath) | left lock: left wheel `PLACEHOLDER_`°, right wheel `PLACEHOLDER_`°; right lock: left `PLACEHOLDER_`°, right `PLACEHOLDER_`° |
| 5 | **Minimum turning radius** | Full left lock, push by hand in a full circle on paper or tape; mark the **rear-axle midpoint** path (or the outer front tyre's track) and measure the diameter. Repeat at full right lock | left `PLACEHOLDER_` m, right `PLACEHOLDER_` m (state which point was traced) |
| 6 | **Servo pulse → wheel angle** | With a servo tester or a pulse reading (when available): pulses 1000, 1250, 1500, 1750, 2000 µs (and the transmitter's real endpoints), the angles of both front wheels. Approach 1500 µs from both sides to see deadband and backlash | table `PLACEHOLDER_` |
| 7 | **Steering speed, rough** | Phone slow-motion video of lock-to-lock with a full stick step; count frames | `PLACEHOLDER_` s lock-to-lock |
| 8 | **Mass and centre of mass** | Kitchen scale: whole car with battery; then front wheels only on the scale (rear wheels on a block of the same height) and rear wheels only. `x_CoM from the rear axle = L · m_front / m_total` | total `PLACEHOLDER_` g; front `PLACEHOLDER_` g; rear `PLACEHOLDER_` g; battery `PLACEHOLDER_` g |
| 9 | **Toe and camber as set** | Photo; note whether they were changed from delivery | `PLACEHOLDER_` |
| 10 | **Gear ratio** | Count pinion and spur teeth (visible after removing the gear cover); the diff ratio from the manual's parts list if printed | pinion `PLACEHOLDER_`, spur `PLACEHOLDER_`, diff `PLACEHOLDER_` |
| 11 | **Radio fail-safe** | **Only with the car on the stand and the motor unplugged:** what does the servo do when the transmitter is switched off? (The motor reaction is checked later, under the safety plan) | `PLACEHOLDER_` |
| 12 | **Servo identification** | Brand and model printed on the servo (photo) | `PLACEHOLDER_` |

When it comes back:
- the table in §B is updated row by row, citing this sheet (date and item);
- the Gazebo model's `PLACEHOLDER_` defaults are replaced;
- the manufacturer values stay as cross-checks.
