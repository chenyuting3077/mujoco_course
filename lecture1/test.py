import mujoco
import mujoco.viewer
import time
import numpy as np

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

def quat_to_pitch(quat):
    """Pitch angle (rotation about y) from a [w, x, y, z] quaternion"""
    w, x, y, z = quat
    return np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))

NUM_MOTOR = 6
WHEEL_IDX = [2, 5]  # L_wheel, R_wheel
# Load a sample model
model = mujoco.MjModel.from_xml_path('../robot/pineapple_v0/scene.xml')
data = mujoco.MjData(model)
target_dof_pos = np.array([1.27, -2.127, 0, 1.27, -2.127, 0])

simulation_dt = 0.005
kps = np.array([40, 40, 40, 40, 40, 40])
kds = np.array([1, 1, 1, 1, 1, 1])

# Wheel balance gains (IMU pitch -> wheel torque)
target_pitch = 0.0  # target body pitch (rad); 0 = upright
kp_pitch = 50
kd_pitch = 2
# Run a simple simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # Read IMU
        imu_quat = data.sensor('imu_quat').data
        imu_gyro = data.sensor('imu_gyro').data
        pitch = quat_to_pitch(imu_quat)
        pitch_rate = imu_gyro[1]

        # Legs: hold joint positions
        tau = pd_control(target_dof_pos, data.sensordata[:NUM_MOTOR], kps, np.zeros(6), data.sensordata[NUM_MOTOR:NUM_MOTOR + NUM_MOTOR], kds)
        # Wheels: balance from IMU
        tau[WHEEL_IDX] = -pd_control(target_pitch, pitch, kp_pitch, 0, pitch_rate, kd_pitch)
        data.ctrl[:] = tau
        model.opt.timestep = simulation_dt
        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

