# Pulse-Oximeter-with-MAX3010X
This project is MAX3010x library for STM32F4, currently supported MAX30100、MAX30102. </br>
The Pulse Oximeter Monitor is a simple PC-Host tool for PPG signal used in pulse oximetry for the computation of oxygen saturation(SpO2).  

![MAX30102](https://github.com/GCY/Pulse-Oximeter-with-MAX3010X/blob/master/res/max30102.jpg)

## USE
### 1. Pulse Oximeter Monitor Compile：

 - g++ -o2 -o main.app main.cpp mathplot.cpp connectargsdlg.cpp serialport.cpp \`wx-config --cxxflags --libs\` --std=c++11 -m64 

### 2. Build stm32f4 project : 
First, select MAX3010x type.
</br>

```cpp
 #define _MAX30102_
//#define _MAX30100_
```

 - Make "./MAX3010x/src/stm32f4 project max3010x" or include "./Library/MAX3010x.h"  into your project.
 - Load main.elf file to STM32F4

### Demo：

![MAX30102 DEMO](https://github.com/GCY/Pulse-Oximeter-with-MAX3010X/blob/master/res/MAX30102%20Demo.gif)

</br>

[![Audi R8](http://img.youtube.com/vi/26pw-d6lBSQ/0.jpg)](https://youtu.be/26pw-d6lBSQ)

</br>

## Near-Infrared Rpectroscopy (NIRS) measure oxygenation in a whole blood

The aim of pulse oximetry is to measure the percentage of oxygenated hemoglobin (HbO2) to the total hemoglobin (Hb) (oxygenated plus deoxygenated) in the arterial blood – this is referred to as SpO2. Oxygenated hemoglobin in the blood is distinctively red, whereas deoxygenated hemoglobin in the blood has a characteristic dark blue coloration. measures light absorbance at one wavelength (or more wavelengths) where there is a large difference between Hb and HbO2 and at another wavelength (or more wavelengths) to quantify oxygen saturation

Absorption spectroscopy – oxygen saturation
![Biomedical Photonics Handbook, Figure 29.2](https://github.com/GCY/Pulse-Oximeter-with-MAX3010X/blob/master/res/Biomedical%20Photonics%20Handbook%2C%20Figure%2029.2.png)

For SpO2 measure the ratios between our two readings(IR and RED), on base level their DC levels should be nearly identical.

MAX30100 adjustment strategy (step 0 to 15 = 0mA to 51mA)：
 - If RED_DC > IR_DC then decrease RED current (-1)
 - If RED_DC < IR_DC then increase RED current (+1)
 
MAX30102 adjustment strategy (step 0 to 255 = 0mA to 51mA)：
decay_coeff = adjustment decay rate.
decay_counter = count(ordinal) of adjustment.
decay_factor = exp(decay_coeff * decay_counter);
 - If RED_DC > IR_DC then decrease RED step of decay_factor value, decay_counter += 1
 - If RED_DC < IR_DC then increase RED step of decay_factor value, decay_counter += 1
 - If (RED_DC - IR_DC) < Threshold(MAGIC_ACCEPTABLE_INTENSITY_DIFF) then decay_counter = 0, done

If decay_coeff or RED_LED_CURRENT_ADJUSTMENT_NS too small, it will result in a lot of oscillation.

![Adjustment Strategy](https://github.com/GCY/Pulse-Oximeter-with-MAX3010X/blob/master/res/DC-Level%20adjustment%20strategies.png)

## Reference：
 - https://morf.lv/implementing-pulse-oximeter-using-max30100 </br>
 - http://www.ti.com/lit/an/slaa274b/slaa274b.pdf </br>
 - http://www.ti.com/lit/ug/tidu542/tidu542.pdf </br>
 - [Biomedical Photonics Handbook](https://books.google.com.tw/books?id=Pl4wsXCiZdQC&pg=SA29-PA2&lpg=SA29-PA2&dq=Biomedical+Photonics+Handbook,+Figure+29.2&source=bl&ots=S9G1e1hksl&sig=ACfU3U1ysW_Pg7jeCagiaYxPfjGsRmO1gg&hl=zh-TW&sa=X&ved=2ahUKEwiktcDUpIXqAhUMH3AKHaWfANUQ6AEwAXoECAsQAQ#v=onepage&q=Biomedical%20Photonics%20Handbook%2C%20Figure%2029.2&f=false)
 
 LICENSE
-------

MIT License

Copyright (c) 2018 GCY

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
