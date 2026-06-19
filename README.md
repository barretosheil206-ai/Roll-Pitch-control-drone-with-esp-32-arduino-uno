# Roll-Pitch-control-drone-with-esp-32-arduino-uno
Applying the theory behind the complimentary filter and PID tuning in real time to see its effects.

## What It Does
The setup aims to control the roll/pitch axis angle.

## Hardware Required
- esp-32 Wroom E (x2) (Can use one arduino uno as shown in one of the images titled "Arduino Setup" with some cav)
- MPU 6050 (x1)
- 820 Coreless DC motor (x2) (Or any other suitable dc motor of your choice)
- L298N H-Bridge (x1)
- Lego Technic (Or make your own physical mechanism similar to mine using material of your choice :) )

## How to Upload
1. Download the `.ino` files from this reposotery.
2. Open it in the Arduino IDE.
3. Select the board of you choice (esp/uno) and port.
4. Make sure to have suitable adafruit libraries for the MPU6050 (Watch the linked YouTube videos in the refrences by Paul McWhorter)
5. "ESP_TX" is for the transmiter code and "PID_Roll_esp_now" for the reciever code, or you can just use the reciever code on its own by removing the wireless related code and losing the wireless capability of real time PID tuning.
6. If using esp-now, make sure to use your own MAC address instead of mine when transmiting and recieving data wirelessly.

## References
1. https://www.youtube.com/watch?v=Krl_6N71uro&list=PLGs0VKk2DiYyn0wN335MXpbi3PRJTMmex&index=77, Credit: Paul McWhorter, Watch lessons 77 to 85 to fully understand how to use MPU6050 as its the most crucial part of the system in my opinion.
2. ![Wiring Diagram for arduino]()
3. ![Wiring Diagram for esp32]()

## License
This project is free to be used by anyone.
