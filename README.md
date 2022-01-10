# HK - Volvo CE Software
![gif](https://github.com/JeremyKTH/Plastic-Bale-Image-Recognition/blob/main/Aladdin.gif)

# Table of contents

<!--ts-->
   * [Plastic Bale Recognition](#Plastic_Bale_Recognition)
   * [Mapping](#Mapping)
   * [Navigation](#Navigation)
   * [Requirements](#Requirements)
   * [Contacts](#Contacts)
   

<!--te-->


## Plastic_Bale_Recognition

<!--ts-->
   * [Introduction](#Introduction)
   * [Datasets](#Datasets)
   * [Experiments](#Experiments)
   * [Results](#Results)
<!--te-->


### Introduction
In order to recognize plastic bale, a model of yolov3 is used. The model is trained on a custom made dataset and recognizes one class, plastic bales. When running yolo, the software creates a bounding box surrounding the plastic bale.

### Datasets

The datasets that is being used is created by the team. It consists of images taken from the internet of the plastic bale as well as images of our own plastic bale model. In order to get more data, we have augumented the images with 90 degree flips to get more images. 



### Results



### Introduction_Mapping

A 2D map is created through SLAM. The camera being used is a ZED2, which has an IMU. Therefore, it is translated into laserscan in order to use gmapping. 





<!-- Navigation -->

## Navigation

<!--ts-->
   * [Introduction Navigation](#Introduction_Navigation)
   * [Results Navigation](#Results_Navigation)
<!--te-->





## Requirements
- Python 3.5+
- PyTorch 3.8.1
- Ubuntu 18.04 or 20.04
- ROS Melodic or Neotic
- CUDA >= 10.2
- cuDNN >= 8.0.2
- OpenCV >= 2.4
- CMake >= 3.18


<!-- CONTACT -->
## Contacts
- Chieh-Ju Wu (Jeremy) - jeremy.cjwukth@gmail.com
- Fredrik Mazur - fredrik@mazur.se
- Daniel Grönås - daniel.gronas@hotmail.com
- Joachim Ottosson - joa.ottosson@gmail.com
- Tess Antonsson - tantonsson@gmail.com
- Jan Siwek - jantesiw@gmail.com 



