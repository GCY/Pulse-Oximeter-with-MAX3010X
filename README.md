# Pulse-Oximeter-with-MAX3010X
Pulse Oximeter Signal Process Demo, MAC OS X Monitor and STM32F4

1.Pulse Oximeter Monitor Compile：
</br>
g++ -o2 -o main.app main.cpp mathplot.cpp connectargsdlg.cpp serialport.cpp \`wx-config --cxxflags --libs\` --std=c++11 -m64 </br>
2. Build stm32f4 project : Make
</br>
3. Connect STM32F4 Device to Monitor with VCP
</br>
</br>
Demo video：
</br>
</br>

[![Audi R8](http://img.youtube.com/vi/26pw-d6lBSQ/0.jpg)](https://youtu.be/26pw-d6lBSQ)

</br>
</br>

## Reference：
 -1. https://morf.lv/implementing-pulse-oximeter-using-max30100 </br>
 -2. http://www.ti.com/lit/an/slaa274b/slaa274b.pdf </br>
 -3. http://www.ti.com/lit/ug/tidu542/tidu542.pdf </br>
 
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
