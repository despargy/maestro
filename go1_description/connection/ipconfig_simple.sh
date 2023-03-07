#!/bin/bash


sudo ifconfig enp59s0 down
sudo ifconfig enp59s0 up 192.168.123.162 netmask 255.255.255.0

