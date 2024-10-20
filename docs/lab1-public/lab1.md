Today's lab consists of two parts: working with a Colab notebook and using the MuJoCo simulator.

# Colab

[lab1-colab](https://colab.research.google.com/github/mim-ml-teaching/public-rc-2024-25/blob/refs/heads/main/docs/lab1-public/lab1-colab-student.ipynb)

# MuJoCo

A significant portion of robotics work involves using simulators like MuJoCo. This lab introduces you to the basics of MuJoCo.

1. Clone this repo (it contains `world1.xml` and `4x4_1000-0.png` files)
2. Install MuJoCo (either just exe, or python library).
   - GitHub repo is available at: <https://github.com/google-deepmind/mujoco>
   - Current release <https://github.com/google-deepmind/mujoco/releases/tag/3.2.3>
   - Instead of installing MuJoCo exe, you can use python like this:

   ```bash
   python3 -m venv venv
   ./venv/bin/activate
   pip install mujoco
   python -m mujoco.viewer 
   ```

3. Run the simulator:
- Run the MuJoCo simulator either by running exe or using the command: `python -m mujoco.viewer`
- You should see a window similar to this: ![MuJoCo Simulator Window](mujoco_1.png)
4. Load the World:
- Drag and drop `world1.xml` into the simulator window. Make sure `4x4_1000-0.png` is in the same directory as `world1.xml`.
- You should now see the world loaded in the simulator:
![MuJoCo World Loaded](mujoco_2.png)
- Explore the MuJoCo interface. Learn how to move the camera, zoom in and out, and rotate the view.
5. Detect Aruco Codes:
- Take a screenshot of the simulator window.
- Use OpenCV to detect the Aruco code in the screenshot.
- *Hint:* you should use `cv2.aruco.DICT_4X4_1000` dictionary
6. Add another Aruco code to the world.
- Create a new Aruco code using a generator like this one: <https://chev.me/arucogen/>
- Convert the SVG to PNG (ImageMagick does not work for this purpose)
- Adjust the XML file accordingly.
7. Detect Multiple Aruco Codes:
- Use OpenCV to detect both Aruco codes in the scene.
- Draw bounding boxes around the detected Aruco codes.
8. Modify the World:
- Change some of the box shapes in the world to other shapes like cylinders or spheres by modifying the geom type in the XML file. Explore the MuJoCo documentation for available geom types.

## Description

The `world1.xml` file describes the simulated environment. In this lab, we are working with static objects.  You might notice some objects appear to be floating. This is because MuJoCo is designed for dynamic simulations, but we are focusing on the visual aspects in this lab.

## Key XML tags

- `asset`: Contains definitions of materials, textures, and other resources.
- `texture`: Defines images used for visual appearance.
- `material`: Specifies the properties of materials.
- `worldbody`: The root element defining the world.
- `body`: Represents an object in the world.
- `geom`: Defines the shape of an object (e.g., box, sphere, cylinder).

## Reference

MuJoCo XML Reference: <https://mujoco.readthedocs.io/en/stable/XMLreference.html>
