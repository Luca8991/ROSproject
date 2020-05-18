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

plan_pos_link2 = np.array([])
plan_vel_link2 = np.array([])

plan_pos_link3 = np.array([])
plan_vel_link3 = np.array([])

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

plan_cart_t = np.array([])
plan_cart_pos_x = np.array([])
plan_cart_pos_y = np.array([])
plan_cart_pos_z = np.array([])

plan_cart_vel_x = np.array([])
plan_cart_vel_y = np.array([])
plan_cart_vel_z = np.array([])

plan_cart_acc_x = np.array([])
plan_cart_acc_y = np.array([])
plan_cart_acc_z = np.array([])

cartstate_t = np.array([])
cartstate_pos_x = np.array([])
cartstate_pos_y = np.array([])
cartstate_pos_z = np.array([])

cartstate_vel_x = np.array([])
cartstate_vel_y = np.array([])
cartstate_vel_z = np.array([])

for topic, msg, t in bag.read_messages():
    if topic == "/joint_trajectory":
        plan_t = np.append(plan_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_pos_link1 = np.append(plan_pos_link1, msg.positions[0])
        plan_pos_link2 = np.append(plan_pos_link2, msg.positions[1])
        plan_pos_link3 = np.append(plan_pos_link3, msg.positions[2])

        plan_vel_link1 = np.append(plan_vel_link1, msg.velocities[0])
        plan_vel_link2 = np.append(plan_vel_link2, msg.velocities[1])
        plan_vel_link3 = np.append(plan_vel_link3, msg.velocities[2])

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

    if topic == "/cartesian_trajectory":
        plan_cart_t = np.append(plan_cart_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_cart_pos_x = np.append(plan_cart_pos_x, msg.positions[0])
        plan_cart_pos_y = np.append(plan_cart_pos_y, msg.positions[1])
        plan_cart_pos_z = np.append(plan_cart_pos_z, msg.positions[2])

        plan_cart_vel_x = np.append(plan_cart_vel_x, msg.velocities[0])
        plan_cart_vel_y = np.append(plan_cart_vel_y, msg.velocities[1])
        plan_cart_vel_z = np.append(plan_cart_vel_z, msg.velocities[2])

        plan_cart_acc_x = np.append(plan_cart_acc_x, msg.accelerations[0])
        plan_cart_acc_y = np.append(plan_cart_acc_y, msg.accelerations[1])
        plan_cart_acc_z = np.append(plan_cart_acc_z, msg.accelerations[2])

    if topic == "/cartesian_states":
        cartstate_t = np.append(cartstate_t, float(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0e-9))
        cartstate_pos_x = np.append(cartstate_pos_x, msg.position[0])
        cartstate_pos_y = np.append(cartstate_pos_y, msg.position[1])
        cartstate_pos_z = np.append(cartstate_pos_z, msg.position[2])

        cartstate_vel_x = np.append(cartstate_vel_x, msg.velocity[0])
        cartstate_vel_y = np.append(cartstate_vel_y, msg.velocity[1])
        cartstate_vel_z = np.append(cartstate_vel_z, msg.velocity[2])
bag.close()

# Plot data
# Planned trajectory link1
plt.figure(1)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_link1,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_link1,label='actual')
plt.ylabel('Position [rad]')
plt.title('Joint trajectory (link1)')
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
plt.title('Joint trajectory (link2)')
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
plt.title('Joint trajectory (link3)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_link3,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_link3,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Cartesian trajectory (time)
plt.figure(4)

plt.subplot(3, 1, 1)
ref, = plt.plot(plan_cart_t,plan_cart_pos_x,label='reference')
act, = plt.plot(cartstate_t-start_delay,cartstate_pos_x,label='actual')
plt.ylabel('x position [m]')
plt.title('Cartesian trajectory')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(3, 1, 2)
ref, = plt.plot(plan_cart_t,plan_cart_pos_y,label='reference')
act, = plt.plot(cartstate_t-start_delay,cartstate_pos_y,label='actual')
plt.ylabel('y position [m]')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(3, 1, 3)
ref, = plt.plot(plan_cart_t,plan_cart_pos_z,label='reference')
act, = plt.plot(cartstate_t-start_delay,cartstate_pos_z,label='actual')
plt.ylabel('z position [m]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Cartesian trajectory (xy)
plt.figure(5)

ref, = plt.plot(plan_cart_pos_x,plan_cart_pos_y,label='reference')
act, = plt.plot(cartstate_pos_x,cartstate_pos_y,label='actual')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Planned Cartesian trajectory')
plt.legend(handles=[ref, act])
plt.grid(True)

# Cartesian trajectory (xz)
plt.figure(6)

ref, = plt.plot(plan_cart_pos_x,plan_cart_pos_z,label='reference')
act, = plt.plot(cartstate_pos_x,cartstate_pos_z,label='actual')
plt.xlabel('X [m]')
plt.ylabel('Z [m]')
plt.title('Planned Cartesian trajectory')
plt.legend(handles=[ref, act])
plt.grid(True)

# Cartesian trajectory (yz)
plt.figure(7)

ref, = plt.plot(plan_cart_pos_y,plan_cart_pos_z,label='reference')
act, = plt.plot(cartstate_pos_y,cartstate_pos_z,label='actual')
plt.xlabel('Y [m]')
plt.ylabel('Z [m]')
plt.title('Planned Cartesian trajectory')
plt.legend(handles=[ref, act])
plt.grid(True)

# Joint torques
plt.figure(8)

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
