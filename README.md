# ESP32_CanOpenNode
CanOpenNode implementation for ESP32 Wrover

CanOpenNode implementation for ESP32


Arduino: 1.8.13
ESP32 Arduino Core: 2.0.7
https://github.com/espressif/arduino-esp32
CANOpenNode: ca5f3578f0e1b9d032165a358720bd7c42447476
https://github.com/CANopenNode/CANopenNode
CANOpenEditor: 76baf61845d0dfeb26f36e7db8a07f2cc0f3d30b
https://github.com/CANopenNode/CANopenEditor

Thanks to Hamed for your very helpful youtube video!
https://www.youtube.com/watch?v=R-r5qIOTjOo

Utilize 2 CAN Wrover ESP32 Modules and designate one as node#9 and the other as node#10

Demonstrates the use of PDO, SDO and HB producer (still working on HB consumer)
-PDOs sent at rate of 1500ms (RPDO on Node 10 for this data)
-HB sent at rate of 1000ms
-SDO requested at rate of 100ms

To view Serial print data set baudrate to 1000000 and have verbose ESP32 logging output

![image](https://user-images.githubusercontent.com/6760950/224513531-c77e4c42-9a40-46f8-9a6a-752097a94344.png)
![image](https://user-images.githubusercontent.com/6760950/224513539-2eb36db6-d9aa-40d8-a574-2be36564c704.png)
![image](https://user-images.githubusercontent.com/6760950/224513744-9d829d40-3960-4a08-ab8b-d9c822de9602.png)
![image](https://user-images.githubusercontent.com/6760950/224513753-8edea17d-c565-4c63-8589-d58f663a9fd0.png)
![image](https://user-images.githubusercontent.com/6760950/224513760-053a4452-d18b-4cd2-8297-61fdc03d68f9.png)
![image](https://user-images.githubusercontent.com/6760950/224513774-f4b71cee-b7d9-45fe-8071-f8cf4c36303a.png)
![image](https://user-images.githubusercontent.com/6760950/224513789-32a8042e-0a8f-473d-9583-c7e22aedee7a.png)
