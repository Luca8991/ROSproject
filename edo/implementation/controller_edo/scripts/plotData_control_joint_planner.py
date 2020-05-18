import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

#Parameters
start_delay = 0.5

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

plan_t = np.array([])

plan_pos_link1 = np.array([])
plan_vel_link1 = np.array([])
plan_acc_link1 = np.array([])

plan_pos_link2 = np.array([])
plan_vel_link2 = np.array([])
plan_acc_link2 = np.array([])

plan_pos_link3 = np.array([])
plan_vel_link3 = np.array([])
plan_acc_link3 = np.array([])

torque_t = np.array([])

torque_link1 = np.array([])
torque_link2 = np.array([])
torque_link3 = np.array([])

jointstate_t = np.array([])

jointstate_pos_link1 = np.array([])
jointstate_vel_link1 = np.array([])

jointstate_pos_link2 = np.array([])
jointstate_vel_link2 = np.array([])

jointstate_pos_link3 = np.array([])
jointstate_vel_link3 = np.array([])

for topic, msg, t in bag.read_messages():
    if topic == "/joint_trajectory":
        plan_t = np.append(plan_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_pos_link1 = np.append(plan_pos_link1, msg.positions[0])
        plan_pos_link2 = np.append(plan_pos_link2, msg.positions[1])
        plan_pos_link3 = np.append(plan_pos_link3, msg.positions[2])

        plan_vel_link1 = np.append(plan_vel_link1, msg.velocities[0])
        plan_vel_link2 = np.append(plan_vel_link2, msg.velocities[1])
        plan_vel_link3 = np.append(plan_vel_link3, msg.velocities[2])

        plan_acc_link1 = np.append(plan_acc_link1, msg.accelerations[0])
        plan_acc_link2 = np.append(plan_acc_link2, msg.accelerations[1])
        plan_acc_link3 = np.append(plan_acc_link3, msg.accelerations[2])

    if topic == "/joint_torque":
        torque_t = np.append(torque_t, msg.data[0])
        torque_link1 = np.append(torque_link1, msg.data[1])
        torque_link2 = np.append(torque_link2, msg.data[2])
        torque_link3 = np.append(torque_link3, msg.data[3])

    if topic == "/joint_states":
        jointstate_t = np.append(jointstate_t, float(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0e-9))
        jointstate_pos_link1 = np.append(jointstate_pos_link1, msg.position[0])
        jointstate_pos_link2 = np.append(jointstate_pos_link2, msg.position[1])
        jointstate_pos_link3 = np.append(jointstate_pos_link3, msg.position[2])

        jointstate_vel_link1 = np.append(jointstate_vel_link1, msg.velocity[0])
        jointstate_vel_link2 = np.append(jointstate_vel_link2, msg.velocity[1])
        jointstate_vel_link3 = np.append(jointstate_vel_link3, msg.velocity[2])

bag.close()

# Plot data
# Planned trajectory link1
plt.figure(1)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_link1,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_link1,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (link1)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_link1,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_link1,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Planned trajectory link2
plt.figure(2)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_link2,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_link2,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (link2)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_link2,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_link2,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Planned trajectory link3
plt.figure(3)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_link3,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_link3,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (link3)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_link3,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_link3,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Joint torques
plt.figure(4)

plt.subplot(3, 1, 1)
plt.plot(torque_t,torque_link1)
plt.ylabel('Torque link1 [Nm]')
plt.title('Joint torques')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(torque_t,torque_link2)
plt.ylabel('Torque link2 [Nm]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(torque_t,torque_link3)
plt.ylabel('Torque link3 [Nm]')
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()
