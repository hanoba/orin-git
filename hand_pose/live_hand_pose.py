#!/usr/bin/env python
# coding: utf-8

# In[1]:


import json
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg 
import trt_pose.coco
import math
import os
import numpy as np
import traitlets


# In[5]:


with open('preprocess/hand_pose.json', 'r') as f:
    hand_pose = json.load(f)

topology = trt_pose.coco.coco_category_to_topology(hand_pose)
import trt_pose.models

num_parts = len(hand_pose['keypoints'])
num_links = len(hand_pose['skeleton'])

model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
import torch


WIDTH = 224
HEIGHT = 224
data = torch.zeros((1, 3, HEIGHT, WIDTH)).cuda()

if not os.path.exists('model/hand_pose_resnet18_att_244_244_trt.pth'):
    MODEL_WEIGHTS = 'model/hand_pose_resnet18_att_244_244.pth'
    model.load_state_dict(torch.load(MODEL_WEIGHTS, weights_only=False))
    import torch2trt
    model_trt = torch2trt.torch2trt(model, [data], fp16_mode=False, max_workspace_size=1<<25)
    OPTIMIZED_MODEL = 'model/hand_pose_resnet18_att_244_244_trt.pth'
    torch.save(model_trt.state_dict(), OPTIMIZED_MODEL)


OPTIMIZED_MODEL = 'model/hand_pose_resnet18_att_244_244_trt.pth'
from torch2trt import TRTModule

model_trt = TRTModule()
model_trt.load_state_dict(torch.load(OPTIMIZED_MODEL))


# In[2]:


from trt_pose.draw_objects import DrawObjects
from trt_pose.parse_objects import ParseObjects

parse_objects = ParseObjects(topology,cmap_threshold=0.15, link_threshold=0.15)
draw_objects = DrawObjects(topology)


# In[4]:


import torchvision.transforms as transforms
import PIL.Image

mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
device = torch.device('cuda')

def preprocess(image):
    global device
    device = torch.device('cuda')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device)
    image.sub_(mean[:, None, None]).div_(std[:, None, None])
    return image[None, ...]


# In[5]:


from preprocessdata import preprocessdata
preprocessdata = preprocessdata(topology, num_parts)
from gesture_classifier import gesture_classifier
gesture_classifier = gesture_classifier()


# Next, let's define a function that will preprocess the image, which is originally in BGR8 / HWC format.

# In[6]:


def draw_joints(image, joints):
    count = 0
    for i in joints:
        if i==[0,0]:
            count+=1
    if count>= 3:
        return 
    for i in joints:
        cv2.circle(image, (i[0],i[1]), 2, (0,0,255), 1)
    cv2.circle(image, (joints[0][0],joints[0][1]), 2, (255,0,255), 1)
    for i in hand_pose['skeleton']:
        if joints[i[0]-1][0]==0 or joints[i[1]-1][0] == 0:
            break
        cv2.line(image, (joints[i[0]-1][0],joints[i[0]-1][1]), (joints[i[1]-1][0],joints[i[1]-1][1]), (0,255,0), 1)


# In[7]:


from jetcam.usb_camera import USBCamera
from jetcam.csi_camera import CSICamera
from jetcam.utils import bgr8_to_jpeg

camera = USBCamera(width=WIDTH, height=HEIGHT, capture_fps=30, capture_device=1)
#camera = CSICamera(width=WIDTH, height=HEIGHT, capture_fps=30)

camera.running = True


# In[8]:


import ipywidgets
from IPython.display import display


image_w = ipywidgets.Image(format='jpeg', width=224, height=224)
display(image_w)


# In[9]:


def execute(change):
    image = change['new']
    data = preprocess(image)
    cmap, paf = model_trt(data)
    cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
    counts, objects, peaks = parse_objects(cmap, paf)
    joints = preprocessdata.joints_inference(image, counts, objects, peaks)
    draw_joints(image, joints)
    #draw_objects(image, counts, objects, peaks)# try this for multiple hand pose prediction 
    image_w.value = bgr8_to_jpeg(image[:, ::-1, :])


# In[10]:


execute({'new': camera.value})


# In[11]:


camera.observe(execute, names='value')


# In[16]:


camera.unobserve_all()


# In[ ]:


#camera.running = False


# In[ ]:





# In[ ]:





# In[ ]:




