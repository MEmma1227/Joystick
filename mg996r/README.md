## Raspberry pi 控制mg996r伺服馬達

### 硬體規格 與 code功能
  - 硬體規格: mg996r 必須是 180 規格的

  - 執行電腦系統: Ubuntu

  - code功能: 使馬達從0°每次遞增+10°直到180°

  - 伺服馬達套件使用: PIGPIO vs. RPi.GPIO

  - **效果表現:  PIGPIO較佳**


### raspberry pi 硬體接線
##### 伺服馬達1：
  - 信號線: GPIO18（物理引腳12）

  - 電源線（紅色）: 5V（物理引腳2或4）

  - 地線（黑色或棕色）: GND（物理引腳6）
##### (伺服馬達2)(code還沒)：
  - 信號線: GPIO12（物理引腳32）

  - 電源線（紅色）: 5V（物理引腳2或4）

  - 地線（黑色或棕色）: GND（物理引腳6）

### PIGPIO
1. 建立檔案
    ```
    touch mg996r_test_pigpio.py
    ```
2. 編輯檔案 (https://github.com/MEmma1227/Joystick/blob/main/mg996r/mg996r_test_pigpio.py)
    ```
    nano mg996r_test_pigpio.py
    ```
    Save / Exit
    ```
    Ctrl + O / Ctrl + X
    ```
3. 使檔案具有執行權限
    ```
    chmod +x mg996r_test_pigpio.py
    ```
4. 開啟 PIGPIO Server
    ```
    sudo pigpiod
    ```
5. 執行 mg996r_test_pigpio.py
    ```
    python mg996r_test_pigpio.py
    ```
6. 若不再執行PIGPIO,關掉Server
    ```
    sudo killall pigpiod
    ```
    #### 安裝 PIGPIO 套件
    ```
    sudo apt-get install pigpio python-pigpio python3-pigpio
    ```

### RPi.GPIO
1. 建立檔案
    ```
    touch mg996r_test_gpio.py
    ```
2. 編輯檔案 (https://github.com/MEmma1227/Joystick/blob/main/mg996r/mg996r_test_GPIO.py)
    ```
    nano mg996r_test_gpio.py
    ```
    Save / Exit
    ```
    Ctrl + O / Ctrl + X
    ```
3. 使檔案具有執行權限
    ```
    chmod +x mg996r_test_gpio.py
    ```
4. 執行 mg996r_test_gpio.py
    ```
    python mg996r_test_gpio.py
    ```

