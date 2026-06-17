# SunoBot 

<!-- ![SunoBotLogo](./assets/SunoBotLogo.png) -->

<p align="left">
<img src="./assets/SunoBotLogo.png" alt="SunoBotLogo" width="25%">

SunoBot is an open-source autonomous mobile robot platform meant to explore computer vision, motion planning, and robotics.

## Capabilities:

<!-- <p align="center">
<img src="assets\cropped-SLAM_Graphic.svg" alt="SunoBotLogo" width="50%" style="border-radius: 15px;">

<p align="center">
<img src="assets\cropped-motion_planning_graphic.svg" alt="SunoBotLogo" width="50%" style="border-radius: 15px;">

<p align="center">
<img src="assets\cropped-semantic_navigation.svg" alt="SunoBotLogo" width="50%" style="border-radius: 15px;"> -->


<p float="center">
  <img src="assets/cropped-SLAM_Graphic.svg" width="32%" bindmedia="" style="border-radius: 15px;"/>
  <img src="assets/cropped-semantic_navigation.svg" width="32%" style="border-radius: 15px;"/>
  <img src="assets/cropped-motion_planning_graphic.svg" width="32%" style="border-radius: 15px;"/>
</p>


SLAM
Motion Planning
Semantic Navigation
Semantic Slam


## Code Installation

Requires Python >=3.12.9

```
pip install suno-bot
```

For systems with an Nvidia GPU, you can use the following command to ensure you install torch with cuda to accelerate inference when running local models.

```
pip install suno-bot[cuda]
```

# Build List
| Part | Quantity | Unit Price (USD) | Link |
| --- | --- | --- | --- |
| **Compute** | 
| Nvidia Jetson Nano Orin | 1 | $250 | [Amazon](https://a.co/d/08duLBSw) |
| 128 GB Micro SD Card | 1 | $34 | [Amazon](https://a.co/d/0giFn1uy) |
| **Sensors** |
| RPLidar C1 | 1 | $69 | [Amazon](https://a.co/d/01sqCkox) |
| Luxonis OAK-D Camera | 1 | $170 | [Luxonis](https://shop.luxonis.com/products/oak-d-lite-1?srsltid=AfmBOorUc5neeXpmWIGfSTZnxkuYcXCcGmICYdHt1S2bHc7bLUCkAe_w) |
| **Motors & Accessories** |
| Dynamixel XL-250 | 2 | $27.50 | [Robotis](https://www.robotis.us/dynamixel-xl430-w250-t/?srsltid=AfmBOorzzLew-BFB_joZx9VtZZxa4Z9oEqbe-iFhbOnDY0QIEfWiF8ji) |
| U2D2 | 1 | $36.92 | [Robotis](https://www.robotis.us/u2d2/?srsltid=AfmBOooSLo45Nv1YYB132Rkxfk1DpJbxzMnHwRaMWvsoDXhAL2AbefDP) |
| U2D2 Powerboard | 1 | $21.85 | [Robotis](https://www.robotis.us/u2d2-power-hub-board-set/?srsltid=AfmBOor3gywk-VzFmWBfLSQ8p4dGw5MIHY-m2bNRO97dZFuF4yPFUO9C) | 
| **Power & Accessories** |
| KBT 12V Battery | 1 | $30 | [Amazon](https://a.co/d/07AhwwMU) |
| 8x AA Battery Holder | 1 | $9 | [Amazon](https://a.co/d/021ZlgrT) |
| 20x AA Batteries | 1 | $10 | [Amazon](https://a.co/d/07lIH3iI) |
| USB C to USB A Cable | 1 | $7 | [Amazon](https://a.co/d/0gSawx84) |
| **Assembly Hardware** |
| Turtle Bot 3 Wheel and Tire Kit (2 Pack) | 1 | $10.81 | [Robotis](https://www.robotis.us/tb3-wheel-tire-set-isw-01-2ea/?srsltid=AfmBOoqbfCWowxFdXaFuWtPJexun5_ZJ9yr--zFMI7q9_nNrtQws9ymt) |
| Turtle Bot 3 Casters | 2 | $4.03 | [Robotis](https://www.robotis.us/tb3-ball-caster-a01-1ea/?srsltid=AfmBOoqPqs-xQAHIdZojc0kwzRB7mlPQLGsexGX9tZXpylSCpcbwvvi0) |
| Screw Pack | 1 | $9 | [Amazon](https://a.co/d/002Dz5YJ) |
| Standoffs | 1 | $10 | [Amazon](https://a.co/d/06RelOfa) |
| *Total* | --- | $730.64 | --- |


# Chassis & Brackets - 3D Printed Parts
| Part | Quantity | CAD File |
| --- | --- | --- |
| Base Board | 2 | Link |
| Sensor Mounting Bracket Riser | 1 | Link |
| Jetson Mount | 1 | Link |
| Jetson Securing Bracket | 1 | Link |
| Camera Mounting Bracket | 1 | Link |
| Camera Mounting Bracket Spacer | 1 | Link |
| Lidar Mounting Bracket | 1 | Link |
| Battery Holder | 1 | Link |
| AA Battery Holder | 1 | Link |

Any 3D printer large enough should work to print all of these parts. For reference, I printed these parts using a Prusa MK4S printer using PLA and got fairly decent results.

# Assembly Instructions
Please follow this tutorial for instructions on how to assemble the SunoBot.

# Technology Stack

<p float="center">
  <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" width="32%" style="border-radius: 15px;"/>
  <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/numpy/numpy-original.svg" width="32%" style="border-radius: 15px;"/>
  <img src="https://upload.wikimedia.org/wikipedia/commons/b/b2/SCIPY_2.svg" width="32%" style="border-radius: 15px;"/>
  <img src="    https://www.sympy.org/static/images/logo.png" width="32%" style="border-radius: 15px;"/>
  <img src="https://upload.wikimedia.org/wikipedia/commons/0/05/Scikit_learn_logo_small.svg" width="32%" style="border-radius: 15px;"/>
  <img src="https://huggingface.co/front/assets/huggingface_logo-noborder.svg" width="32%" style="border-radius: 15px;"/>
  <img src="https://www.luxonis.com/assets/marketing/brand/luxonis_logo_symbol.png" width="32%" style="border-radius: 15px;"/>
  <img src="https://www.gstatic.com/lamda/images/gemini_sparkle_v002_d4735304ff6292a690345.svg" width="32%" style="border-radius: 15px;"/>
  <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/opencv/opencv-original.svg" width="32%" style="border-radius: 15px;"/>

  <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/pytorch/pytorch-original.svg" width="32%" style="border-radius: 15px;"/>
  <img src="https://plugins.jetbrains.com/files/14004/1074279/icon/default.png" width="32%" style="border-radius: 15px;"/>
  <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/matplotlib/matplotlib-original.svg" width="32%" style="border-radius: 15px;"/>
  <img src="https://images.icon-icons.com/2415/PNG/512/redis_original_wordmark_logo_icon_146369.png" width="32%" style="border-radius: 15px;"/>
</p>


<!-- <div align="left"> -->

<div style="display: inline-block; width: 22%; margin: 10px; text-align: center; vertical-align: top;">
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" style="width: 100%; border-radius: 15px;"/>
<br><strong style="font-size: 16px;">Python</strong>
</div>
<div style="display: inline-block; width: 22%; margin: 10px; text-align: center; vertical-align: top;">
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/numpy/numpy-original.svg" style="width: 100%; border-radius: 15px;"/>
<br><strong style="font-size: 16px;">NumPy</strong>
</div>
<div style="display: inline-block; width: 22%; margin: 10px; text-align: center; vertical-align: top;">
<img src="https://upload.wikimedia.org/wikipedia/commons/b/b2/SCIPY_2.svg" style="width: 100%; border-radius: 15px;"/>
<br><strong style="font-size: 16px;">SciPy</strong>
</div>

<div align="center">
  <div style="display: inline-block; width: 22%; margin: 10px; text-align: center; vertical-align: top;"><img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" style="width: 100%; border-radius: 15px;"/><br><strong style="font-size: 16px; font-family: sans-serif;">Python</strong></div><div style="display: inline-block; width: 22%; margin: 10px; text-align: center; vertical-align: top;"><img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/numpy/numpy-original.svg" style="width: 100%; border-radius: 15px;"/><br><strong style="font-size: 16px; font-family: sans-serif;">NumPy</strong></div><div style="display: inline-block; width: 22%; margin: 10px; text-align: center; vertical-align: top;"><img src="https://upload.wikimedia.org/wikipedia/commons/b/b2/SCIPY_2.svg" style="width: 100%; border-radius: 15px;"/><br><strong style="font-size: 16px; font-family: sans-serif;">SciPy</strong></div>
</div>

  <!-- </div> -->
