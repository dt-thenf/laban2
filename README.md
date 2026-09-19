# laban2 — La bàn GY-85 cho thiết bị đeo (ESP32)

Bộ firmware này dùng module **GY-85** gồm:

- **ADXL345** — accelerometer 3 trục.
- **ITG3205 / ITG3200** — gyroscope 3 trục.
- **HMC5883L** — magnetometer 3 trục.

Mục tiêu là tạo một la bàn cho thiết bị đeo có hành vi gần với la bàn điện thoại hơn cách tính đơn giản \`atan2(my, mx)\`: khi thiết bị nghiêng, ngửa hoặc cúi, hướng vẫn được bù nghiêng; hướng vật lý của "mũi tên" trên thiết bị chỉ cần xác lập một lần rồi được lưu trong flash/NVS của ESP32.

> Firmware hiện tại tối ưu cho **ESP32 + Arduino framework**. Không cần cài thư viện cảm biến bên thứ ba vì driver I²C cho cả ba IC đã nằm trong sketch.

---

## 1. File chính

- \`firmware/GY85_Wearable_Compass/GY85_Wearable_Compass.ino\`  
  Firmware hoàn chỉnh.
- \`tools/calibrate_mag.py\`  
  Hiệu chỉnh magnetometer nâng cao bằng ellipsoid fitting để xử lý hard-iron và full 3x3 soft-iron.
- \`platformio.ini\`  
  Cấu hình build nhanh bằng PlatformIO.

---

## 2. Vì sao không dùng công thức la bàn phẳng thông thường?

Công thức:

\`\`\`cpp
heading = atan2(my, mx);
\`\`\`

chỉ đúng tốt khi cảm biến gần như nằm ngang.

Khi đeo trên người, bo mạch liên tục nghiêng theo tay/ngực/lưng. Nếu không bù nghiêng, thành phần từ trường theo trục X/Y thay đổi và heading có thể sai hàng chục độ dù người dùng không đổi hướng.

Firmware này làm theo hướng eCompass 3D:

1. ADXL345 đo hướng trọng lực.
2. ITG3205 hỗ trợ ước lượng trọng lực khi thiết bị đang chuyển động, tránh chỉ dựa vào accelerometer.
3. HMC5883L được hiệu chỉnh hard-iron / soft-iron.
4. Từ trường được chiếu lên mặt phẳng ngang vuông góc với vector trọng lực.
5. Vector mũi tên vật lý của thiết bị cũng được chiếu lên mặt phẳng ngang.
6. Góc có dấu giữa "Bắc từ" và "mũi tên" được tính trong 3D.
7. Kết quả được lọc theo miền góc tròn 0–360° để không giật khi qua 359° → 0°.

Cách biểu diễn bằng vector tránh việc lấy trực tiếp yaw từ Euler angles nên không bị phụ thuộc vào singularity của biểu diễn roll/pitch/yaw thông thường.

### Giới hạn vật lý cần hiểu

Nếu **mũi tên vật lý của thiết bị đang chỉ gần như thẳng đứng lên trời hoặc xuống đất**, hình chiếu của nó lên mặt phẳng ngang gần bằng 0. Khi đó "hướng Bắc/Đông/Nam/Tây của mũi tên" về mặt toán học không còn xác định duy nhất.

Firmware không đoán bừa trong trường hợp này mà giữ heading hợp lệ gần nhất và báo:

\`\`\`
quality=VERTICAL_HOLD
\`\`\`

Đây là hành vi cố ý để tránh nhảy ngẫu nhiên 180°.

---

## 3. Đấu dây ESP32 ↔ GY-85

Mặc định firmware:

| GY-85 | ESP32 |
|---|---|
| 3V3 | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

I²C chạy ở 400 kHz.

Địa chỉ dự kiến:

| Sensor | I²C |
|---|---:|
| ADXL345 | \`0x53\` |
| ITG3205 | \`0x68\` |
| HMC5883L | \`0x1E\` |

Sau khi nạp code, mở Serial Monitor ở **115200 baud** và chạy:

\`\`\`
SCAN
\`\`\`

Kết quả mong đợi có ba địa chỉ trên.

Nếu không thấy \`0x1E\`, cần kiểm tra xem module có thực sự dùng HMC5883L hay là clone QMC5883L. Firmware này hiện không coi QMC5883L là tương đương HMC5883L.

---

## 4. Build và nạp code

### Arduino IDE

Mở:

\`\`\`
firmware/GY85_Wearable_Compass/GY85_Wearable_Compass.ino
\`\`\`

Chọn đúng board ESP32, cổng COM và Upload.

### PlatformIO

Tại thư mục repo:

\`\`\`bash
pio run
pio run -t upload
pio device monitor
\`\`\`

---

## 5. Quy trình setup chuẩn

Nên lắp GY-85 vào **đúng vị trí cuối cùng trên thiết bị đeo** trước khi hiệu chỉnh magnetometer. Pin, khung kim loại, vít thép, loa, nam châm, dây nguồn có dòng lớn đều có thể làm thay đổi từ trường.

### Bước 1 — Gyroscope

Lần đầu boot, nếu chưa có gyro bias trong NVS, firmware tự yêu cầu giữ thiết bị đứng yên và lấy bias khoảng 2 giây.

Có thể chạy lại thủ công:

\`\`\`
GYRO_CAL
\`\`\`

Trong lúc đó không chạm, không xoay và không rung thiết bị.

Bias được lưu trong NVS.

Firmware còn tự điều chỉnh bias rất chậm trong RAM khi phát hiện thiết bị đứng yên, giúp giảm drift do nhiệt độ mà không ghi flash liên tục.

---

## 6. Hiệu chỉnh HMC5883L — mức cơ bản

Chạy:

\`\`\`
MAG_CAL_START
\`\`\`

Trong khoảng 30–60 giây:

- xoay số 8;
- lật đủ các mặt;
- quay quanh X;
- quay quanh Y;
- quay quanh Z;
- cố gắng phủ càng nhiều hướng 3D càng tốt.

Sau đó:

\`\`\`
MAG_CAL_STOP
\`\`\`

Firmware sẽ tính:

- hard-iron bias X/Y/Z;
- scale X/Y/Z;
- reference magnetic norm;

và lưu vào NVS.

Đây là hiệu chỉnh nhanh, phù hợp để test.

---

## 6.1 Cách dễ nhất: thu magnetometer bằng wizard Python

Repo có sẵn:

\`\`\`
tools/capture_mag_guided.py
\`\`\`

Script này tự:

- tìm và cho chọn cổng COM của ESP32;
- mở Serial ở 115200 baud;
- gửi \`MAG_STREAM 1\`;
- hướng dẫn từng tư thế xoay/lật GY-85;
- đếm ngược từng bước;
- chỉ ghi các dòng \`MAGCSV\` hợp lệ;
- bỏ mẫu trùng liên tiếp;
- gửi \`MAG_STREAM 0\` khi kết thúc;
- tạo \`tools/mag_log.txt\`;
- in số mẫu và range X/Y/Z;
- nhắc lệnh chạy \`calibrate_mag.py\` tiếp theo.

### Chuẩn bị một lần

Cài pySerial:

\`\`\`powershell
pip install pyserial
\`\`\`

Đóng Arduino Serial Monitor hoặc PlatformIO Monitor trước khi chạy để tránh cổng COM bị chiếm.

### Chạy wizard

Từ thư mục \`tools\`:

\`\`\`powershell
python .\capture_mag_guided.py
\`\`\`

Nếu máy có nhiều cổng COM, script sẽ hiện danh sách và hỏi bạn chọn số tương ứng với ESP32.

Sau đó chỉ cần đọc hướng dẫn trên màn hình và nhấn Enter ở từng bước. Các bước gồm:

1. mặt bo ngửa lên, xoay ngang 360°;
2. mặt bo úp xuống, xoay ngang 360°;
3. dựng cạnh trái và xoay;
4. dựng cạnh phải và xoay;
5. đầu mũi tên hướng lên;
6. đầu mũi tên hướng xuống;
7. vẽ hình số 8 trong không gian;
8. quay tự do quanh cả ba trục để bổ sung độ phủ.

Mặc định mỗi bước ghi 7 giây. Có thể tăng:

\`\`\`powershell
python .\capture_mag_guided.py --seconds 10
\`\`\`

Hoặc chỉ định thẳng COM:

\`\`\`powershell
python .\capture_mag_guided.py --port COM5
\`\`\`

Kết thúc, file được tạo tại:

\`\`\`
tools/mag_log.txt
\`\`\`

Script sẽ in đường dẫn tuyệt đối để dễ kiểm tra.

Sau đó chạy:

\`\`\`powershell
python .\calibrate_mag.py .\mag_log.txt
\`\`\`

## 7. Hiệu chỉnh nâng cao — full 3x3 soft-iron

Đây là phương án nên dùng cho bản thiết bị chính thức.

### 7.1 Thu dữ liệu

Serial:

\`\`\`
MAG_STREAM 1
\`\`\`

Ghi log Serial ra file trong khi xoay thiết bị qua toàn bộ không gian 3D. Nên có khoảng **1000–3000 mẫu**.

Dừng:

\`\`\`
MAG_STREAM 0
\`\`\`

Các dòng log có dạng:

\`\`\`
MAGCSV,123,-456,78
\`\`\`

### 7.2 Chạy Python

Cài NumPy:

\`\`\`bash
pip install numpy
\`\`\`

Chạy:

\`\`\`bash
python tools/calibrate_mag.py mag_log.txt
\`\`\`

Tool sẽ:

- fit ellipsoid;
- tìm tâm ellipsoid = hard-iron bias;
- tính ma trận 3×3 biến ellipsoid về gần hình cầu;
- kiểm tra coverage;
- in độ lệch chuẩn bán kính trước/sau;
- sinh sẵn một lệnh Serial.

Ví dụ đầu ra:

\`\`\`
MAG_MATRIX bX bY bZ m00 m01 m02 m10 m11 m12 m20 m21 m22 ref
\`\`\`

Copy nguyên dòng này vào Serial Monitor.

Firmware sẽ lưu ma trận vào NVS.

### Vì sao full 3x3 tốt hơn min/max?

Min/max chỉ sửa offset và scale từng trục. Nó không sửa tốt trường hợp soft-iron làm ellipsoid bị xoay / có coupling giữa X-Y-Z.

Ma trận 3×3 xử lý được các thành phần chéo nên phù hợp hơn với thiết bị đeo có pin, PCB, dây dẫn và kết cấu cơ khí xung quanh cảm biến.

---

## 8. Magnetic declination

HMC5883L chỉ cho hướng theo **Bắc từ**.

Nếu muốn Bắc địa lý / True North, cần nhập magnetic declination của vị trí triển khai:

\`\`\`
DECL 0.50
\`\`\`

Quy ước:

- Đông: dương.
- Tây: âm.

Ví dụ:

\`\`\`
DECL -2.30
\`\`\`

Giá trị được lưu trong NVS.

Không nên hard-code một declination dùng cho mọi vị trí.

---

## 9. Cấu hình đeo cố định của dự án

Firmware được cấu hình theo đúng cách đeo thực tế:

- mũi tên **+X** in trên PCB hướng lên đầu;
- mũi tên **+Y** hướng sang tay trái;
- mặt PCB giống ảnh tham chiếu hướng ra phía trước;
- mặt chứa các IC hướng vào cơ thể.

Với hệ trục tay phải này:

```
+X = HEAD
+Y = LEFT
+Z = BODY / inward
FORWARD = -Z
```

Do đó firmware dùng trực tiếp vector:

```cpp
Vec3(0.0f, 0.0f, -1.0f)
```

làm hướng phía trước của người đeo.

**Không cần `ARROW_SET`.** Thiết bị được sử dụng khi PCB nằm dọc sát người.

### Kiểm tra tư thế đeo

Đeo thiết bị đúng vị trí, đứng thẳng và nhập:

```
MOUNT_CHECK
```

Kết quả mong muốn:

```
KIỂM_TRA_ĐEO,kết_quả=ĐÚNG,...
```

Khi đứng thẳng, vector `UP` phải gần `+X`, vì +X đang hướng lên đầu.

Nếu nhận `X_REVERSED_OR_DEVICE_UPSIDE_DOWN`, hướng X đang ngược hoặc thiết bị bị đeo ngược. Nếu nhận `CHECK_AXIS_MAP`, cần kiểm tra lại `ACCEL_MAP / GYRO_MAP / MAG_MAP`.

---

## 10. Sau khi đã căn chỉnh

Luồng sử dụng bình thường:

1. Bật nguồn.
2. ESP32 đọc calibration từ NVS.
3. Firmware dùng cố định hướng người đeo = -Z của PCB.
4. Heading xuất liên tục.

Ví dụ:

\`\`\`
LA_BÀN,θ=83.42°,θ_thô=84.10°,hướng=Đông,trạng_thái=Tốt,|B|=...,F_ngang=...,U=(...)
\`\`\`

Các hướng 8 phương được hiển thị bằng tiếng Việt:

- Bắc
- Đông Bắc
- Đông
- Đông Nam
- Nam
- Tây Nam
- Tây
- Tây Bắc

Ký hiệu chính trong dòng la bàn:

- `θ`: góc hướng đã lọc, đơn vị độ `°`;
- `θ_thô`: góc hướng chưa lọc;
- `|B|`: độ lớn vector từ trường sau hiệu chỉnh;
- `F_ngang`: độ lớn hình chiếu ngang của vector hướng tiến;
- `U=(Ux,Uy,Uz)`: vector hướng lên ước lượng từ accelerometer + gyro.

---

## 11. Các lệnh Serial

| Lệnh | Chức năng |
|---|---|
| \`HELP\` | In danh sách lệnh |
| \`STATUS\` | Xem sensor, calibration, bias, matrix |
| \`SCAN\` | Quét I²C |
| \`GYRO_CAL\` | Calibrate gyro khi đứng yên |
| \`MAG_CAL_START\` | Bắt đầu thu min/max magnetometer |
| \`MAG_CAL_STOP\` | Tạo basic mag calibration và lưu |
| \`MAG_STREAM 1\` | Xuất \`MAGCSV,x,y,z\` |
| \`MAG_STREAM 0\` | Dừng stream magnetometer |
| \`RAW 1\` | Stream accel + gyro + mag |
| \`RAW 0\` | Dừng raw stream |
| \`MAG_MATRIX ...\` | Nạp full 3x3 calibration |
| \`MAG_RESET\` | Xóa calibration magnetometer |
| \`DECL x\` | Set declination |
| \`ARROW_SET x\` | Set mũi tên vật lý một lần theo bearing x |
| \`ARROW_RESET\` | Xóa offset mũi tên |
| \`RESET_ALL\` | Reset toàn bộ config |

---

## 12. Quality flags

### \`OK\`

Heading hợp lệ, magnetometer đã calibration.

### \`NO_MAG_CAL\`

Vẫn tính heading nhưng chưa có calibration đáng tin cậy.

Không nên dùng kết quả này làm dữ liệu chính thức.

### \`MAG_DISTURBANCE\`

Độ lớn từ trường khác nhiều so với reference đã calibration.

Nguyên nhân thường gặp:

- điện thoại để sát module;
- nam châm;
- động cơ;
- dòng điện lớn;
- pin / dây nguồn chuyển vị trí;
- đặt gần kết cấu thép.

Firmware giữ heading trước đó thay vì tin một measurement bất thường.

### \`VERTICAL_HOLD\`

Hướng phía trước `-Z` đang gần thẳng đứng, horizontal projection quá nhỏ.

Heading cũ được giữ lại để tránh flip.

### \`SENSOR_ERROR\`

Không có dữ liệu hợp lệ.

---

## 13. Axis alignment — cực kỳ quan trọng

Một eCompass chỉ bù nghiêng đúng khi:

- accelerometer;
- gyro;
- magnetometer

đều được biểu diễn trong **cùng một hệ trục**.

Đầu sketch có:

\`\`\`cpp
static constexpr AxisMap ACCEL_MAP(0, 1, 2, +1, +1, +1);
static constexpr AxisMap GYRO_MAP (0, 1, 2, +1, +1, +1);
static constexpr AxisMap MAG_MAP  (0, 1, 2, +1, +1, +1);
\`\`\`

Mặc định là identity.

Nếu clone GY-85 của bạn bố trí chip khác, phải chỉnh map.

Ví dụ:

\`\`\`cpp
{{1,0,2}, {+1,-1,+1}}
\`\`\`

nghĩa là:

- body X = native Y;
- body Y = -native X;
- body Z = native Z.

### Dấu hiệu axis map sai

Đặt thiết bị tại một heading cố định rồi chỉ nghiêng lên/xuống hoặc trái/phải.

Nếu heading thay đổi lớn theo tilt dù magnetometer đã calibration tốt thì ưu tiên kiểm tra:

1. MAG axis map;
2. ACCEL axis map;
3. GYRO axis map;
4. hard/soft-iron calibration.

Không nên cố "chữa" lỗi axis mapping bằng \`ARROW_SET\`. \`ARROW_SET\` chỉ sửa hướng mũi tên trong mặt phẳng thiết bị, không sửa sai hệ trục 3D giữa các sensor.

---

## 14. Quy trình calibration nên dùng cho wearable thật

Thứ tự khuyến nghị:

1. Gắn ESP32 + GY-85 + pin + dây + vỏ đúng cấu hình sử dụng cuối.
2. Đặt HMC5883L càng xa:
   - nguồn DC/DC;
   - loa;
   - nam châm;
   - dây dòng lớn;
   - vật liệu sắt.
3. \`GYRO_CAL\`.
4. Full 3D mag capture.
5. Chạy \`calibrate_mag.py\`.
6. Nạp \`MAG_MATRIX ...\`.
7. Set \`DECL\` nếu cần True North.
8. Set \`ARROW_SET ...\` đúng một lần.
9. Tắt/bật nguồn.
10. Test tilt ở nhiều heading khác nhau.

---

## 15. Test nhanh chất lượng bù nghiêng

Chọn một hướng cố định, ví dụ ~90°.

Giữ vị trí, chỉ thay đổi:

- roll +45° / -45°;
- pitch +45° / -45°;
- lật nhẹ mặt thiết bị;
- đeo lên vị trí sử dụng thật.

Heading không nên chạy hàng chục độ chỉ vì tilt.

Nếu sai số thay đổi theo góc nghiêng, lỗi thường đến từ calibration hoặc axis alignment.

---

## 16. Các tối ưu đã dùng

### Gyro-assisted gravity

Không low-pass accelerometer đơn thuần.

Firmware tích phân gyro để dự đoán vector trọng lực trong body frame, sau đó correction dần bằng accelerometer khi độ lớn gia tốc gần 1g.

Khi người dùng chạy hoặc rung mạnh, confidence của accelerometer giảm.

### Vector projection thay vì yaw Euler

North vector:

\`\`\`
north_horizontal = mag - up * dot(mag, up)
\`\`\`

Arrow vector:

\`\`\`
arrow_horizontal = arrow - up * dot(arrow, up)
\`\`\`

Heading là signed angle giữa hai vector trên.

### Circular low-pass

Sai số góc dùng \`wrap180()\`, do đó:

\`\`\`
359° -> 1°
\`\`\`

được hiểu là thay đổi 2°, không phải 358°.

### Magnetic disturbance rejection

Sau calibration, độ lớn từ trường hiện tại được so với reference. Measurement quá bất thường sẽ không cập nhật heading.

### NVS persistence

Các giá trị sau tồn tại qua reboot:

- gyro calibration;
- magnetometer bias;
- magnetometer matrix;
- magnetic norm reference;
- declination;

---

## 17. Nguồn kỹ thuật tham khảo

Các tài liệu chính được dùng để xây dựng hướng tiếp cận:

- NXP / Freescale AN4248 — *Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors*  
  https://www.nxp.com/docs/en/application-note/AN4248.pdf
- NXP / Freescale AN4246 — *Calibrating an eCompass in the Presence of Hard- and Soft-Iron Interference*  
  https://www.nxp.com/docs/en/application-note/AN4246.pdf
- Analog Devices ADXL345 datasheet  
  https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
- Arduino MadgwickAHRS implementation  
  https://github.com/arduino-libraries/MadgwickAHRS
- GY-85 technical notes / reference implementation  
  https://github.com/madc/GY-85
- HMC5883L datasheet mirror  
  https://www.elecrow.com/download/HMC5883L-datasheet.pdf

ITG-3200/3205 có scale factor điển hình **14.375 LSB/(°/s)** ở ±2000°/s; firmware sử dụng trực tiếp thông số này.

---

## 18. Điều nên cải tiến tiếp nếu cần độ chính xác cao hơn

Bản hiện tại ưu tiên dễ kiểm soát và ít phụ thuộc thư viện.

Các bước nâng cấp tiếp theo có thể gồm:

- calibration accelerometer 6 mặt;
- tự xác định sensor-to-body rotation;
- nhiệt độ-compensated gyro bias;
- adaptive magnetic anomaly model;
- Mahony/Madgwick quaternion 9-DoF đầy đủ;
- lưu nhiều profile calibration theo từng vị trí đeo;
- kiểm thử sai số theo bàn xoay chuẩn;
- tự đánh giá calibration quality trên ESP32;
- hỗ trợ QMC5883L clone.

Đối với wearable dùng trong nhà, magnetic disturbance từ kết cấu thép và thiết bị điện thường là giới hạn lớn hơn độ phân giải danh nghĩa của HMC5883L. Vì vậy nên đánh giá hệ thống trong đúng môi trường sử dụng cuối thay vì chỉ test trên bàn.


## 9. Cấu hình đeo cố định của dự án

Firmware hiện được cấu hình đúng theo cách lắp thực tế:

- mũi tên **+X** in trên PCB hướng lên đầu;
- mũi tên **+Y** hướng sang tay trái;
- mặt PCB giống hình tham chiếu hướng ra phía trước;
- mặt có các IC hướng vào cơ thể.

Với hệ trục tay phải này, **+Z hướng vào cơ thể**, do đó hướng tiến của người đeo là:

```
FORWARD = -Z
```

Firmware dùng trực tiếp vector `(0, 0, -1)` làm hướng phía trước. Vì vậy:

- không cần `ARROW_SET`;
- PCB không cần đặt nằm ngang khi sử dụng;
- thiết bị được đeo dọc sát người đúng như thiết kế;
- tilt compensation vẫn chiếu vector `-Z` xuống mặt phẳng ngang để tính azimuth.

Sau khi đeo đúng tư thế và người đứng thẳng, chạy:

```
MOUNT_CHECK
```

Kết quả tốt:

```
MOUNT_CHECK,result=OK,...
```

Khi đó vector trọng lực phải gần `+X` vì +X đang hướng lên đầu.

Nếu nhận:

```
KIỂM_TRA_ĐEO,kết_quả=X_BỊ_NGƯỢC_HOẶC_ĐEO_NGƯỢC
```

thì dấu trục X hoặc cách đeo đang ngược.

Nếu nhận:

```
KIỂM_TRA_ĐEO,kết_quả=CẦN_KIỂM_TRA_AXIS_MAP
```

thì cần kiểm tra `ACCEL_MAP / GYRO_MAP / MAG_MAP` trên đúng board GY-85 thực tế.

Các lệnh `ARROW_SET` và `ARROW_RESET` chỉ còn được giữ để tương thích; firmware sẽ bỏ qua chúng.
