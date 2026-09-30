# Lecture 1：用 IMU 做輪式平衡控制

`test.py` 讓雙輪足機器人（`robot/pineapple_v0`）落地後靠輪子維持直立：

- **腿**：用 PD 控制維持固定姿態
- **輪子**：讀 IMU 的 pitch 角與 pitch 角速度，輸出平衡力矩（輪式倒單擺）

## 感測器與致動器順序

致動器（`data.ctrl`）與 `jointpos` / `jointvel` 感測器的順序相同：

| index | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| 關節 | L_thigh | L_calf | **L_wheel** | R_thigh | R_calf | **R_wheel** |

- `data.sensordata[0:6]`：關節角度
- `data.sensordata[6:12]`：關節角速度
- 輪子的 index 是 `WHEEL_IDX = [2, 5]`

IMU 是掛在 `base_link` 上的 site `imu`，程式裡依名稱讀取，不用自己算 sensordata 的 index：

```python
imu_quat = data.sensor('imu_quat').data   # [w, x, y, z]
imu_gyro = data.sensor('imu_gyro').data   # 機身座標系的角速度 [wx, wy, wz]
```

## 控制架構

6 個馬達分成兩組，各自用不同的訊號來算力矩：

| | 腿（4 個：L/R_thigh、L/R_calf） | 輪子（2 個：L/R_wheel） |
|---|---|---|
| 目的 | 維持固定姿態 | 讓機身不倒 |
| 看什麼 | 關節角度、關節角速度 | IMU 的 pitch 角、pitch 角速度 |
| 目標 | `target_dof_pos` | `target_pitch` |
| 控制方式 | PD | PD |

每個模擬步驟做的事：

1. 讀感測器：關節角度與角速度，以及 IMU 的四元數與角速度
2. 腿：用「目標角度 − 目前角度」做 PD，算出 4 個腿部力矩
3. 輪子：用「目前 pitch − 目標 pitch」做 PD，算出 2 個輪子力矩
4. 把 6 個力矩寫進 `data.ctrl`，執行 `mj_step`

兩組是互相獨立的：腿不管機身有沒有在倒，輪子也不管腿的角度。

### 1. 從四元數算 pitch

四元數 $(w, x, y, z)$：

$$
\text{pitch} = \arcsin\big(2(wy - zx)\big)
$$

pitch 角速度取 gyro（機身座標系角速度 $(\omega_x, \omega_y, \omega_z)$）的 y 分量：

$$
\dot{\text{pitch}} = \omega_y
$$

座標軸：$x$ 朝前、$y$ 朝左、$z$ 朝上。依右手定則繞 $+y$ 轉正角度時，$x$ 軸會往 $-z$ 轉，所以：

- $\text{pitch} > 0$：機身前傾
- $\text{pitch} < 0$：機身後仰

### 2. 腿：PD 維持姿態

沿用原本的 `pd_control()`，目標角度為 `target_dof_pos = [1.27, -2.127, 0, 1.27, -2.127, 0]`（輪子那兩項會被下一步覆蓋）。

### 3. 輪子：PD 平衡

左右輪輸出相同的力矩：

$$
\tau_{\text{wheel}} = K_{p,\text{pitch}}\,(\text{pitch} - \text{pitch}^*) + K_{d,\text{pitch}}\,\dot{\text{pitch}}
$$

- $\text{pitch}^*$：目標 pitch（`target_pitch`）
- $K_{p,\text{pitch}}$、$K_{d,\text{pitch}}$：PD 增益（`kp_pitch`、`kd_pitch`）

**符號推導**：輪子轉軸是 $+y$，$\tau_{\text{wheel}} > 0$ 會讓輪子往 $+x$（前方）滾。

- 機身往前倒（$\text{pitch} > \text{pitch}^*$）→ $\tau_{\text{wheel}} > 0$ → 輪子往前追到重心下方
- 機身往後倒（$\text{pitch} < \text{pitch}^*$）→ $\tau_{\text{wheel}} < 0$ → 輪子往後追

另外，輪子的反作用力矩也會把機身往回推，方向同樣是對的。

## 參數

| 參數 | 值 | 說明 |
|---|---|---|
| `kps`（腿） | 40 | 原本 10 撐不住機身，腿會被壓垮 |
| `kds`（腿） | 1 | 原本 0.1 |
| `kp_pitch` | 50 | |
| `kd_pitch` | 2 | |
| `target_pitch` | 0 rad | 目前這組腿部姿態下會倒，見下方說明 |

輪子的實際輸出力矩會被 XML 裡的 `actuatorfrcrange="-3.69 3.69"` 限制在 ±3.69 Nm。

### `target_pitch = 0` 的問題

在 `target_dof_pos` 這組腿部姿態下，機身 pitch = 0 時，整體重心在輪軸後方約 2.3 cm。控制器把 0 當成平衡點，會一直把機身拉回 0，但重力會一直把它往後拉，所以機器人最後會倒。

把腿設定在目標姿態，掃描不同的 pitch 並計算「重心 x − 輪軸 x」：

| pitch (rad) | 0.00 | 0.10 | 0.15 | 0.20 | 0.25 |
|---|---|---|---|---|---|
| 重心 − 輪軸 (m) | −0.0226 | −0.0102 | −0.0039 | +0.0024 | +0.0087 |

重心要在 pitch ≈ 0.18～0.2 rad 時才會對準輪軸。想用 `target_pitch = 0` 站穩，就要調整 `target_dof_pos`，讓 pitch = 0 時重心剛好落在輪軸正上方。
