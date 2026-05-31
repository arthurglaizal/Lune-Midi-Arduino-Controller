# Lune MIDI Arduino Controller

**Lune MIDI Arduino Controller** is a DIY MIDI controller built with Arduino for DJs, musicians, and electronic music experimentation.

This repository contains the Arduino source code for the project.  
The full build guide, images, explanations, and hardware details are available on Instructables:

[View the full project on Instructables](https://www.instructables.com/id/MIDI-Controller-With-Arduino-for-DJ-or-Musician/)

## Demo video

Watch the Lune MIDI Controller in action:

[![Lune MIDI Controller demo](https://img.youtube.com/vi/z67HJUsKVNc/maxresdefault.jpg)](https://www.youtube.com/watch?v=z67HJUsKVNc)

## Overview

Lune was my first Arduino project. The goal was to build a custom MIDI controller to control parameters in music software using different types of physical inputs.

The controller includes:

- **9 axial potentiometers**
  - 6 for EQ
  - 3 for effects
- **1 encoder**
- **4 linear potentiometers** for volume
- **4 push buttons**
  - 2 for play controls
  - 2 for cue controls
- **2 ultrasonic sensors** for gesture-based effect control
- **2 RGB LED pads**

## How it works

The controller reads values from physical sensors and sends them to the computer.

Basic flow:

```text
Sensor input → Arduino → MIDI data → Music software
