# router_lift
Router table with electronic router lift using ESP32 and 3.5" touch screen

In 2020 I designed and built this router table with an electronic router lift. Since then i upgraded the electronics and rewrote the software, i made a YT video for it and published the design.

Main features:

- Z height calibration using a sense block
- absolute positioning
- adjustable step size and plunge rate
- 25 presets
- 5 x 5 step sequencer
- dial a position of calculate a position

The electronics has a provision for a second stepper motor in case i later want to redesign it with a electronic fence.

Since all parts are made by hand there are no precise mechanical drawings but I included some freecad concept drawings.

Development in the Arduino 1 IDE in portable mode, uses FastAccelStepper 1.2.7 stepper driver library, LovyanGFX 1.2.9 graphics driver library and LVGL 9.5.0 GUI library.


