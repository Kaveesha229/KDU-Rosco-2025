<h1 align="center">ROSCO 2025 Competition Robot</h1>
<p align="center"><strong>ROBOTICS | NAVIGATION | CONTROL</strong></p>
<p align="center"><a href="https://github.com/Kaveesha229">Kaveesha Gunawardena | Engineering portfolio</a></p>

## Overview

Competition robot work by Team KETHAKARU for ROSCO 2025 at KDU. The team achieved second place. The repository contains navigation firmware variants and a team image.

## Repository guide

<table>
<tr><th>File or folder</th><th>Purpose</th></tr>
<tr><td><a href="final_to_left_pidm.ino">final_to_left_pidm.ino</a></td><td>Root navigation sketch</td></tr>
<tr><td><a href="Rosco/multiplexer/multiplexer.ino">Rosco/multiplexer/multiplexer.ino</a></td><td>Sensor acquisition experiment</td></tr>
<tr><td><a href="Rosco/pid/white/final_to_left_pidm/final_to_left_pidm.ino">Rosco/pid/white/final_to_left_pidm/final_to_left_pidm.ino</a></td><td>Nested navigation variant</td></tr>
<tr><td><a href="Rosco/pid/black/pidm_black_bg/pidm_black_bg.ino">Rosco/pid/black/pidm_black_bg/pidm_black_bg.ino</a></td><td>Alternative line background variant</td></tr>
</table>

## System overview

The navigation code combines a 16 channel infrared array through a multiplexer, differential motor control and two ultrasonic sensors. Separate routines handle line following on different backgrounds, turning and wall following.

## Hardware and software

The source uses Arduino C++, `Wire` and `NewPing`. Multiplexer selection uses A0 through A3 with its signal on A4. Motor PWM outputs are pins 5 and 6. The two ultrasonic trigger and echo pairs use 9 with 10 and 11 with 12. Check the complete selected sketch for the remaining inputs.

## Getting started

1. Inspect the firmware variants and choose the one that matches the robot and track.
2. Install `NewPing` and select a board compatible with the analogue and digital pin assignments.
3. Validate the multiplexer channel order, motor polarity and ultrasonic orientation.
4. Calibrate the sensor threshold and tune control gains on the actual robot.
5. Compile the selected sketch before uploading. The current root sketch begins with a stray `w` before the first include directive; remove that character in your working copy.

## Implementation notes

The multiplexer experiment currently prints `sensorValues[8]` repeatedly inside its output loop. It therefore does not display all 16 readings as written. Correct the index to the loop variable in your working copy before using that output for calibration. No new hardware validation or timing benchmark is claimed here.

## Team contribution

The award belongs to Team KETHAKARU. Individual responsibilities are not assigned in this documentation because the repository does not establish who completed each part.

## Project media

Photos and demonstration videos will be added here as the project presentation is updated.

<p align="center"><sub>Practical engineering through building, testing and refinement.</sub></p>
