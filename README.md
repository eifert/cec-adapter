CEC-Adapter (Work-in-Progress)
==============================

CEC-Adapter is an implementation of the HDMI CEC protocol built on the [Embassy](https://embassy.dev/) framework for embedded applications.

Requirements
============

- Embassy supported MCU with a open-drain capable GPIO (should be pretty much all Embassy supported MCUs)CE
- Access to the physical CEC line of a HDMI connection (for example by using a HDMI breakout board)

What works
==========

- Building for RP2040/Rasperry Pi Pico
- Receiving and decoding HDMI CEC frames, with automatic acknowledgment of a configured local CEC physical address
- Sending CEC frames, including proper line-free detection

What's TODO
=================

- Support some form of external interface to actually make use of CEC data, like USB CDC/serial command interface
- Support building for other MCUs
- Nicer API
