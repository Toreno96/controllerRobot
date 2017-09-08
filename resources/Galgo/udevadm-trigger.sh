#!/bin/sh -e

udevadm trigger --subsystem-match="usb" --attr-match="idVendor=0403" --attr-match="idProduct=6014"

exit $?
