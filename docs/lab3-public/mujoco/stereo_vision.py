import mujoco
from PIL import Image
import cv2
import numpy as np

# distance between two cameras
baseline = 0.1

sphere_size = 0.01
box1_size = 0.05
box2_size = 0.02

resolution = (1280, 1280)

img1 = cv2.imread('left.png')
img2 = cv2.imread('right.png')

#TODO: find the positions of the objects and reconstruct the images

# red sphere
pos_sphere = (...)

# green box
pos_box1 = (...)

# blue box
pos_box2 = (...)

xml_string =f"""\
<mujoco model="simple_scene">
      <visual>
     <global offwidth="{resolution[0]}" offheight="{resolution[1]}"/>
  </visual>
    <asset>
        <texture name="plane_texture" type="2d" builtin="checker" rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3" width="512" height="512"/>
        <material name="plane_material" texture="plane_texture" texrepeat="5 5" reflectance="0."/>
    </asset>

    <worldbody>
        <geom type="plane" size="1 1 0.1" material="plane_material"/>

        <body pos="{pos_sphere[0]} {pos_sphere[1]} {pos_sphere[2]}">
            <geom type="sphere" size="{sphere_size}" rgba="1 0 0 1"/>
        </body>

        <body pos="{pos_box1[0]} {pos_box1[1]} {pos_box1[2]}">
            <geom type="box" size="0.05 0.05 0.05" rgba="0 1 0 1"/>
        </body>

        <body pos="{pos_box2[0]} {pos_box2[1]} {pos_box2[2]} ">
            <geom type="box" size="0.02 0.02 0.02" rgba="0 0 1 1"/>
        </body>

        <!-- Cameras -->
        <camera name="camera1" pos="0 0 1" fovy="90"/>
        <camera name="camera2" pos="{baseline} 0 1" fovy="90"/>
    </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml_string)
renderer = mujoco.Renderer(model, resolution[0], resolution[1])

data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

renderer.update_scene(data, camera="camera1")
img1_r = Image.fromarray(renderer.render())
img1_r.save("reconstruct_left.png")

renderer.update_scene(data, camera="camera2")
img2_r = Image.fromarray(renderer.render())
img2_r.save("reconstruct_right.png")
