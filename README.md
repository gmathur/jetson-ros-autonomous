# How to build your own self-driving robot

Given all the efforts in this space, I wanted to spend some time playing around with the state of the art in this space. The first step was to build a robot platform that would let me do that so I decided to have some fun building out a platform from scratch and have fun doing that.

## Parts list

1. The basic metal tank crawler [here](https://www.banggood.com/DIY-T300-Metal-Tracked-Crawler-Smart-Robot-Car-Robotic-Chassis-Platform-Track-Tank-Kit-For-Arduino-p-1132988.html?utm_source=Youtube&utm_medium=thonain&utm_campaign=BG28172553&utm_content=luoyun&cur_warehouse=CN) for $132. There are a lot of other crawlers there - buy one to your liking
2. Ydlidar X4 lidar for $99 [here](https://www.superdroidrobots.com/shop/item.aspx/ydlidar-x4-360-degree-10-meter-2d-lidar/2510/)
3. [Raspberry Pi 3](https://www.amazon.com/Raspberry-Pi-RASPBERRYPI3-MODB-1GB-Model-Motherboard/dp/B01CD5VC92/ref=sr_1_1?s=electronics&ie=UTF8&qid=1528144609&sr=1-1&keywords=raspberry+pi+3) for $35 (The quad core is *really* nice)
4. Dimension Engineering dual 12Ax2 regenerative motor driver [here](https://www.dimensionengineering.com/products/sabertooth2x12) for $80. Provides a nice 5v output for the raspberry pi and the rest of the system to consume too.
5. [Raspberry pi v2 camera](https://www.amazon.com/Raspberry-Pi-Camera-Module-Megapixel/dp/B01ER2SKFS/ref=sr_1_cc_1?s=aps&ie=UTF8&qid=1528144583&sr=1-1-catcorr&keywords=raspberry+pi+v2+camera) for $25
6. MPU 9255 IMU that you can get [here](https://www.amazon.com/WINGONEER-MPU-9255-GY-9255-Attitude-Accelerator/dp/B06XHK8BK6/ref=sr_1_1?ie=UTF8&qid=1528144440&sr=8-1&keywords=mpu9255) for $10-15
7. A small OLED screen like [this](https://www.amazon.com/gp/product/B01HHPOD44/ref=oh_aui_search_detailpage?ie=UTF8&psc=1) for $25... because why not :)
8. I decided to go with LiPo batteries. [These](https://www.amazon.com/gp/product/B00FE0OHKK/ref=oh_aui_search_detailpage?ie=UTF8&psc=1) ones output 11.1v and have proven to be a good workhorse. Also got [this alarm](https://www.amazon.com/gp/product/B00XQ91ECA/ref=oh_aui_search_detailpage?ie=UTF8&psc=1) to make sure I dont over discarge my batteries. ($38 + $9)
9. A basic voltage regulator to keep it steady at 12v is probably a good idea like [this](https://www.amazon.com/gp/product/B075DFBTRM/ref=oh_aui_search_detailpage?ie=UTF8&psc=1) - almost certainly needed if you are getting a LiPo battery ($10)

Total BOM: ~$463
