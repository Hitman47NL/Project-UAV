#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 22:09:07 2024

@author: iwan
"""
from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
#picam2.start_preview(Preview.QTGL)
picam2.start()
time.sleep(5)
picam2.stop()
#picam2.stop_preview()
time.sleep(2)
print("-------done-------")