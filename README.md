# HK - Volvo CE Software
![gif](https://github.com/JeremyKTH/Plastic-Bale-Image-Recognition/blob/main/Aladdin.gif)

# Table of contents

<!--ts-->
   * [Plastic Bale Recognition](#Plastic_Bale_Recognition)
   * [Mapping](#Mapping)
   * [Navigation](#Navigation)
   * [Contacts](#Contacts)
   

<!--te-->


## Plastic_Bale_Recognition

<!--ts-->
   * [Introduction](#Introduction)
   * [Datasets](#Datasets)
   * [Requirements](#Requirements)
   * [Experiments](#Experiments)
   * [Results](#Results)
<!--te-->


### Introduction
In order to recognize plastic bale, a model of yolov3 is used. The model is trained on a custom made dataset and recognizes one class, plastic bales. When running yolo, the software creates a bounding box surrounding the plastic bale.

### Datasets
<img src = "https://github.com/JeremyKTH/Plastic-Bale-Image-Recognition/blob/main/Images/plasticbale.png" width="250" height="220"> 
The datasets that is being used is created by the team. It consists of images taken from the internet of the plastic bale as well as images of our own plastic bale model. In order to get more data, we have augumented the images with 90 degree flips to get more images. 

### Requirements
- Python  3.9.1
- PyTorch 1.8.1
- CUDA toolkit 10.2


### Results
<img src = "https://github.com/JeremyKTH/Plastic-Bale-Image-Recognition/blob/main/tony.jpg" width="150" height="250"> 



<!-- Mapping -->

## Mapping

<!--ts-->
   * [Introduction Mapping](#Introduction Mapping)
   * [Results Mapping](#Results Mapping)
<!--te-->


### Introduction Mapping

A 2D map is created through SLAM. The camera being used is a ZED2, which has an IMU. Therefore, it is translated into laserscan in order to use gmapping. 


### Results Mapping
<img src = "https://github.com/JeremyKTH/Plastic-Bale-Image-Recognition/blob/main/tony.jpg" width="150" height="250"> 



<!-- Navigation -->

## Navigation

<!--ts-->
   * [Introduction Navigation](#Introduction Navigation)
   * [Results Navigation](#Results Navigation)
<!--te-->



### Introduction Navigation

A 2D map is created through SLAM. The camera being used is a ZED2, which has an IMU. Therefore, it is translated into laserscan in order to use gmapping. 


### Results Navigation
<img src = "https://github.com/JeremyKTH/Plastic-Bale-Image-Recognition/blob/main/tony.jpg" width="150" height="250"> 




<!-- CONTACT -->
## Contact
- Chieh-Ju Wu (Jeremy) - jeremy.cjwukth@gmail.com
- Fredrik Mazur - fredrik@mazur.se
- Daniel Grönås - daniel.gronas@hotmail.com
- Joachim Ottosson - joa.ottosson@gmail.com
- Tess Antonsson - tantonsson@gmail.com
- Jan Siwek - jantesiw@gmail.com 



