# ILI9486 Driver for RPi Type displays.
I was using a RPi Type display (http://www.lcdwiki.com/3.5inch_RPi_Display), and noticed that none of the current libraries in Rust worked with this spesific type of display.
Alot of the initializing code is very much inspired from the TFT_eSPI library by Bodmer (https://github.com/Bodmer/TFT_eSPI), because that was the only library that i actually got working with the screen beforehand.
Currently, the "library" is not yet formatted as a library, and is provided very much as is. If i find the motivation, i will set it up so that the naming conventions in the code is more consistent, and that it is compiled as a proper library.
There is a very very simple implementation of the Embedded_graphics included. It enables the library to draw graphics pixel-by-pixel, so i would not expect insane refresh rates, but it does work.

A little TODO list, for when i find the time and motivation:
* Convert to a proper library.
* Include more Embedded_Graphics implementations.
* Clean up Interface.rs
