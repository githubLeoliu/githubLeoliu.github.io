# Gimbal Lock.

This program is a demonstration of gimbal lock. A gimbal has 3 Degrees of Freedom. If the first term ring o the third term ring becomes parallel to the middle one, the gimbal lock and it loses a DoF. This can be avoided by knowing when gimbal lock will occur and take it to account when you do the rotations. Also by adding a fourth ring but it isn't convinient in robot simulation. 

First we import the libraries. Then we set some variables that are going to be useful later in the code. After that we initialize the physics server as a GUI, and define the variable useFixedBase.


```python
import pybullet as pb
import pybullet_data
import ipympl
import matplotlib.pyplot
%matplotlib ipympl

#GUI
physicsClient = pb.connect(pb.DIRECT)
#physicsClient = pb.connect(pb.GUI)

#Plane 
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
plane= pb.loadURDF('plane.urdf')

useFixedBase=True
```

    pybullet build time: Aug 29 2022 20:09:30



```python
from dataclasses import dataclass
from pandas import array
import transforms3d as tf 
import numpy as np
from numpy import ndarray, uint8
from matplotlib.image import AxesImage

#Functions and Variables

#Corrects the pose of the gimbal in pybullet 
def Pose(Ring1,Ring2,Ring3,PosR1,PosR2,PosR3):
  pb.resetBasePositionAndOrientation(Ring1,[0,0,1], PosR1) 
  pb.resetBasePositionAndOrientation(Ring2,[0,0,1], PosR2)        
  pb.resetBasePositionAndOrientation(Ring3,[0,0,1], PosR3)

@dataclass
class Configs:
    roll_Camera  = np.pi/4
    pitch_Camera = -10.0
    yaw_Camera=np.pi/4
    fov=60
    pixelWidth = 1500
    pixelHeight = 700
    upAxisIndex = 2
    camDistance = 4
    Near = 0.01
    Far = 100
    R=0.0 # Roll_Slider
    P=0.0 # Pitch_Slider
    Y=0.0 # Yaw_Slider
    i=0.0
    w:int=0
    h:int=0
    rgb:tuple=()
    cameraPos = [1, 1, 1]
    camTargetPos = [0, 0, 0]
    np_img_arr: np.ndarray = ()
    img_data: AxesImage = ()


configs = Configs()


```

Figure 1 shows that when we run the program we can se that we spawn the environment. 



In this section, we load the three rings that are going to be used to make a gimbal. Notice that when we load the Urdf files, I put useFixedBase, that is because we need that the rings are not able to move.

Then we set some sliders, there we define the name of the slider, the range and set the startup number.


```python
#Ring 1 
Ring1=pb.loadURDF("3rd ring.urdf", useFixedBase= useFixedBase) # 3rd term / Outer ring

#Ring 2
Ring2=pb.loadURDF("1st ring.urdf", useFixedBase= useFixedBase) # Middle ring

#Ring 3
Ring3=pb.loadURDF("2nd ring.urdf", useFixedBase= useFixedBase) # 1st term/ Inner ring

```

***Simulation steps***

In this part of the code, first we make the program read the keyboard so we can code some keyboard events, and define which keys are set to do an specific action.Then we make the system read the parameters we define outside the loop so they can appear in the simulation constantly. This parameters define zyx (Roll, Pitch, Yaw) of the gimbal given in Euler Angles. After that, we define the actions of the keys, this actions trigger gimbal lock.

Finally, using the library Transforms3d, we convert the Euler angles to quarternions to define, and then we define the pose of the gimbal.

The rotation sequence I used is: Rzyx

$R = R_z(\psi) R_y(\theta) R_x(\phi)$

$R=\begin{bmatrix}cos(\psi) & -sin(\psi) & 0\\ sin(\psi) & cos(\psi) & 0 \\ 0 & 0 & 1 \end{bmatrix} * \begin{bmatrix}cos(\theta) & 0 & sin(\theta)\\ 0 & 1 & 0 \\ -sin(\theta) & 0 & cos(\theta) \end{bmatrix} * \begin{bmatrix} 1 & 0 & 0\\ 0 & cos(\phi) & -sin(\phi) \\0 & sin(\phi) & cos(\phi) \end{bmatrix}$ 

We use the camera to get the image of the giroscope so we can plot it with matplotlib.


```python
from IPython.display import clear_output
import matplotlib.pyplot as plt
import ipywidgets as widgets
import time

# Create the sliders and a button 
Slider_1 = widgets.FloatSlider(value=configs.R, description= 'Roll' ,min=-np.pi, max=np.pi, step=0.05) #Roll
Slider_2 = widgets.FloatSlider(value=configs.P, description= 'Pitch' ,min=-np.pi, max=np.pi, step=0.05) #Pitch
Slider_3 = widgets.FloatSlider(value=configs.Y, description= 'Yaw' ,min=-np.pi, max=np.pi, step=0.05) #Yaw

Updt=widgets.Button(description='Show Graph') # This button is used to update the image of the gimbal

# Outputs 
output2 = widgets.Output() #Image output
output_Sliders=widgets.Output() #Sliders output


# Display Widgets, and Outputs
display(Slider_1,Slider_2,Slider_3,Updt,output2,output_Sliders)

# The following three functions are used to change the value of the rotations of the gimbal

# Roll funtion 
def value_slider(change):
    with output_Sliders:
        clear_output()
        configs.R=change['new']

# Pitch function
def value_slider1(change):
    with output_Sliders:
        clear_output()
        configs.P=change['new']

# Yaw function
def value_slider2(change):
    with output_Sliders:
        clear_output()
        configs.Y=change['new']

# These callback functions are activated when you move the sliders.        

# Main Function
def Update_Value(b):
    with output2:
        clear_output()
        PosR1=tf.euler.euler2quat(0,np.pi/2,configs.Y, 'rzyx') # 3rd term / Outer ring
        PosR2=tf.euler.euler2quat(0, configs.P+np.pi,configs.Y, 'rzyx') # Middle ring
        PosR3=tf.euler.euler2quat(np.pi/2+configs.R,np.pi+configs.P,configs.Y, 'rzyx') # 1st term/ Inner ring
 
        # Calls function for pose
        Pose(Ring1,Ring2,Ring3,PosR1,PosR2,PosR3)

        # Renders the image
        viewMatrix = pb.computeViewMatrixFromYawPitchRoll(configs.camTargetPos, configs.camDistance, configs.yaw_Camera, configs.pitch_Camera,
                                                                configs.roll_Camera, configs.upAxisIndex)
        aspect = configs.pixelWidth / configs.pixelHeight
        projectionMatrix = pb.computeProjectionMatrixFOV(configs.fov, aspect, configs.Near, configs.Far)

        img_arr = pb.getCameraImage(configs.pixelWidth,
                                        configs.pixelHeight,
                                        viewMatrix,
                                        projectionMatrix,
                                        shadow=1,
                                        lightDirection=[1, 1, 1],
                                        renderer=pb.ER_TINY_RENDERER)

        configs.w = img_arr[0]  #width of the image, in pixels
        configs.h = img_arr[1]  #height of the image, in pixels
        configs.rgb = img_arr[2]  #color data RGB

     
        configs.np_img_arr = np.reshape(configs.rgb, (configs.h, configs.w, 4)).astype(np.uint8)

        if configs.i==0:
            configs.img_data = plt.imshow(configs.np_img_arr) 
            configs.i=1
        else:
            configs.img_data.set_data(configs.np_img_arr)
        

        plt.title('Gimbal')
        plt.axis('off')
        plt.pause(.00001)

 
# Links the button with the main function 
Updt.on_click(Update_Value)

# Checks the state of the value of each slider

Slider_1.observe(value_slider, names='value')
Slider_2.observe(value_slider1, names='value')
Slider_3.observe(value_slider2, names='value')
```


    FloatSlider(value=0.0, description='Roll', max=3.141592653589793, min=-3.141592653589793, step=0.05)



    FloatSlider(value=0.0, description='Pitch', max=3.141592653589793, min=-3.141592653589793, step=0.05)



    FloatSlider(value=0.0, description='Yaw', max=3.141592653589793, min=-3.141592653589793, step=0.05)



    Button(description='Show Graph', style=ButtonStyle())



    Output()



    Output()



```python

```
