# Smart-bracelet-based-on-stm32
本项目顺应健康管理趋势，设计了一款腕带类产品——智能手环。用户的功能性需求 主要包括健康管理、便携佩戴、价格、续航等方面，故运动检测和健康管理等功能基本是标 配。通过这款手环，用户可以记录日常生活中的锻炼、环境、时间等实时数据，并将这些数 据与手机、平板同步，起到通过数据指导健康生活的作用
该项目名称是基于 STM32 的智能手环，智能手环的硬件系统即为 STM32 单片机开发板和外设传感系统，软件开发使用 Keil5 软件，用 C 语言进行开发。  
本项目可以实现使用 DHT11 温湿度传感器测量环境温湿度并显示；当温度达到报警值时的报警系统；  RTC 实时时间显示以及使用 MPU6050 传感器检测加速度和角速度进行输出。  另外，还可使用 WIFI 模块将传感器数据上传到主机。该设计应用人群和范围广泛，可以为人们提供日常生活和运动时的相关数据，并提供一定的身体状况相
关数据，对使用人的身体健康状况提供一定的参考。  
功能如下：  
（1）实时时间显示  
使用 RTC 输出实时时间，包括年、月、日、时、分、秒等等，在 LCD 屏上显示。  
（2）环境温湿度检测  
使用 DHT11 传感器进行环境温湿度检测，并显示在 LCD 屏上。  
（3）温度报警  
当 DHT11 感应到的温度超过设定的温度值后，则蜂鸣器报警，屏幕字体变红，LED 灯变红。  
（4）运动数据检测  
使用 MPU6050 传感器对运动产生的加速度，角速度等进行检测，并通过 I2C 总线获取数据。  
（5）WIFI 数据传送  
使用 ESP8266 进行 WIFI 连接，实现监测数据上传
