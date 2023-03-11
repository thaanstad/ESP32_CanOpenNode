# ESP32_CanOpenNode
CanOpenNode implementation for ESP32 Wrover

CanOpenNode implementation for ESP32

ESP32 Core: 2.0.7
CANOpenNode: ca5f3578f0e1b9d032165a358720bd7c42447476

Utilize 2 CAN Wrover ESP32 Modules and designate one as node#9 and the other as node#10

Demonstrates the use of PDO, SDO and HB producer (still working on HB consumer)
-PDOs sent at rate of 1500ms (RPDO on Node 10 for this data)
-HB sent at rate of 1000ms
-SDO requested at rate of 100ms

To view Serial print data set baudrate to 1000000 and have verbose ESP32 logging output

![image](https://user-images.githubusercontent.com/6760950/224513531-c77e4c42-9a40-46f8-9a6a-752097a94344.png)
![image](https://user-images.githubusercontent.com/6760950/224513539-2eb36db6-d9aa-40d8-a574-2be36564c704.png)
