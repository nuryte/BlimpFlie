import matplotlib.pyplot as plt
import numpy as np
import time
import pygame

def angle_to_coordinates(radians: float, radius: float = 1.0) -> tuple:
    x = radius * np.cos(radians)
    y = radius * np.sin(radians)
    return x, y

plt.ion()
fig, ax = plt.subplots()
fig.patch.set_facecolor('black')
ax.set_facecolor('black')

ax.set_xlim(-1.1, 2.1)
ax.set_ylim(-1.1, 1.5)
ax.set_aspect('equal', 'box')
circle = plt.Circle((0, 0), 1, fill=False, color='white', linewidth=3)
ax.add_artist(circle)
ax.set_xticks([])
ax.set_yticks([])

bar_width = 0.4
bar_bottom = -1

cur_height_bar = ax.bar(1.8, 0, bar_width, color='r', bottom=bar_bottom)
des_height_bar = ax.bar(1.4, 0, bar_width, color='g', bottom=bar_bottom)

current_yaw = ax.arrow(0, 0, 0, 0, head_width=0.1, head_length=0.1, fc='r', ec='r', linewidth=3)
desired_yaw = ax.arrow(0, 0, 0, 0, head_width=0.1, head_length=0.1, fc='g', ec='g', linewidth=3)

current_yaw_value = ax.text(-0.4, 1.1, '', fontsize=12, color='white')
desired_yaw_value = ax.text(-0.4, 1.3, '', fontsize=12, color='white')

current_height_value = ax.text(1.6, -1.1, '', fontsize=12, color='white')
desired_height_value = ax.text(1.2, -1.1, '', fontsize=12, color='white')

hight_label = ax.text(1.4, -1.2, 'Height', fontsize=12, color='white')
yaw_label = ax.text(-0.1, -1.2, 'Yaw', fontsize=12, color='white')

pygame.init()
display = pygame.display.set_mode((100, 100))

cur_yaw = 0
des_yaw = 0
cur_height = 0
des_height = 0
delay = 0.01

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        keys = pygame.key.get_pressed()

        if keys[pygame.K_UP]:
            des_height += 0.1
            time.sleep(0.01)
        if keys[pygame.K_DOWN]:
            des_height -= 0.1
            time.sleep(0.01)
        if keys[pygame.K_LEFT]:
            des_yaw += 0.07
            time.sleep(0.01)
        if keys[pygame.K_RIGHT]:
            des_yaw -= 0.07            
            time.sleep(0.01)
        if keys[pygame.K_ESCAPE]:
            break

        if not any(keys):
            time.sleep(delay)

        cur_yaw += 0.03 * (des_yaw - cur_yaw)
        cur_x, cur_y = angle_to_coordinates(cur_yaw)
        des_x, des_y = angle_to_coordinates(des_yaw)

        cur_height += 0.02 * (des_height - cur_height)

        current_yaw.remove()
        desired_yaw.remove()

        current_yaw = ax.arrow(0, 0, cur_x, cur_y, head_width=0.1, head_length=0.1, fc='r', ec='r', linewidth=3)
        desired_yaw = ax.arrow(0, 0, des_x, des_y, head_width=0.1, head_length=0.1, fc='g', ec='g', linewidth=3)

        cur_height_bar[0].set_height(cur_height / 10 if cur_height > 0 else 0)
        des_height_bar[0].set_height(des_height / 10 if des_height > 0 else 0)

        current_yaw_value.set_text(f"Current: {(cur_yaw % (2*np.pi)) / np.pi * 180:.2f}˚")
        current_yaw_value.set_color('r')  # Setting the text color to red

        desired_yaw_value.set_text(f"Desired: {(des_yaw % (2*np.pi)) / np.pi * 180:.2f}˚")
        desired_yaw_value.set_color('g')  # Setting the text color to green
        
        desired_height_value.set_text(f"D{des_height if des_height > 0 else 0:.2f}")
        desired_height_value.set_position((1.2, max(des_height / 10 - 0.9, -0.9)))
        desired_height_value.set_color('g')  # Setting the text color to green
        
        current_height_value.set_text(f"C{cur_height if cur_height > 0 else 0:.2f}")
        current_height_value.set_position((1.6, max(cur_height / 10 - 0.9, -0.9)))
        current_height_value.set_color('r')

        plt.draw()

except KeyboardInterrupt:
    print("Loop terminated by user.")
