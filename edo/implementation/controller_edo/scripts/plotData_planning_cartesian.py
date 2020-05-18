import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

plan_t = np.array([])

plan_pos_link1 = np.array([])
plan_vel_link1 = np.array([])

plan_pos_link2 = np.array([])
plan_vel_link2 = np.array([])

plan_pos_link3 = np.array([])
plan_vel_link3 = np.array([])

plan_cart_t = np.array([])
plan_cart_states_t = np.array([])


plan_cart_pos = np.array([])
plan_cart_vel = np.array([])
plan_cart_acc = np.array([])

plan_cart_states_pos_x = np.array([])
plan_cart_states_vel_x = np.array([])
plan_cart_states_pos_y = np.array([])
plan_cart_states_vel_y = np.array([])
plan_cart_states_pos_z = np.array([])
plan_cart_states_vel_z = np.array([])


for topic, msg, t in bag.read_messages():
    if topic == "/joint_trajectory":
        plan_t = np.append(plan_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_pos_link1 = np.append(plan_pos_link1, msg.positions[0])
        plan_pos_link2 = np.append(plan_pos_link2, msg.positions[1])
        plan_pos_link3 = np.append(plan_pos_link3, msg.positions[2])

        plan_vel_link1 = np.append(plan_vel_link1, msg.velocities[0])
        plan_vel_link2 = np.append(plan_vel_link2, msg.velocities[1])
        plan_vel_link3 = np.append(plan_vel_link3, msg.velocities[2])
    if topic == "/cartesian_trajectory":
        #print(msg)
        plan_cart_t = np.append(plan_cart_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_cart_pos = np.append(plan_cart_pos, msg.positions[0])
        plan_cart_vel = np.append(plan_cart_vel, msg.velocities[0])
        plan_cart_acc = np.append(plan_cart_acc, msg.accelerations[0])
    if topic == "/cartesian_states":
        #print(msg)
        plan_cart_states_t = np.append(plan_cart_states_t, float(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0e-9))
        plan_cart_states_pos_x = np.append(plan_cart_states_pos_x, msg.position[0])
        plan_cart_states_vel_x = np.append(plan_cart_states_vel_x, msg.velocity[0])

        plan_cart_states_pos_y = np.append(plan_cart_states_pos_y, msg.position[1])
        plan_cart_states_vel_y = np.append(plan_cart_states_vel_y, msg.velocity[1])

        plan_cart_states_pos_z = np.append(plan_cart_states_pos_z, msg.position[2])
        plan_cart_states_vel_z = np.append(plan_cart_states_vel_z, msg.velocity[2])

bag.close()

# Plot data
# Planned trajectory link1
plt.figure(1)

plt.subplot(2, 1, 1)
plt.plot(plan_t,plan_pos_link1)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (link1)')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(plan_t,plan_vel_link1)
plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

# Planned trajectory link2
plt.figure(2)

plt.subplot(2, 1, 1)
plt.plot(plan_t,plan_pos_link2)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (link2)')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(plan_t,plan_vel_link2)
plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

# Planned trajectory link3
plt.figure(3)

plt.subplot(2, 1, 1)
plt.plot(plan_t,plan_pos_link3)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (link3)')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(plan_t,plan_vel_link3)
plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

# Planned Cartesian trajectory
plt.figure(4)

plt.subplot(3, 1, 1)
plt.plot(plan_cart_t,plan_cart_pos)
plt.ylabel('Position [m]')
plt.title('Planned end-effector Cartesian trajectory')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_cart_t,plan_cart_vel)
plt.ylabel('Velocity [m/s]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(plan_cart_t,plan_cart_acc)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s^2]')
plt.grid(True)

# Planned Cartesian states
plt.figure(5)

plt.subplot(3, 1, 1)
plt.plot(plan_cart_states_t,plan_cart_states_pos_x)
plt.ylabel('Position [m]')
plt.title('Planned end-effector Cartesian states X')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_cart_states_t,plan_cart_states_vel_x)
plt.ylabel('Velocity [m/s]')
plt.grid(True)

plt.figure(6)

plt.subplot(3, 1, 1)
plt.plot(plan_cart_states_t,plan_cart_states_pos_y)
plt.ylabel('Position [m]')
plt.title('Planned end-effector Cartesian states Y')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_cart_states_t,plan_cart_states_vel_y)
plt.ylabel('Velocity [m/s]')
plt.grid(True)

plt.figure(7)

plt.subplot(3, 1, 1)
plt.plot(plan_cart_states_t,plan_cart_states_pos_z)
plt.ylabel('Position [m]')
plt.title('Planned end-effector Cartesian states Z')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_cart_states_t,plan_cart_states_vel_z)
plt.ylabel('Velocity [m/s]')
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()
