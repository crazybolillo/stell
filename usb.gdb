# Automate common breakpoint and data extraction for USB debugging
set width 0
set height 0
set verbose off

b usbReset
commands 1
  continue
end

b usbAfterReset
commands 2
  continue
end

b usbRead
commands 3
  printf "GRXSTSR: %x\n", OTG->GRXSTSR
  continue
end

b usbWrite
commands 4
  continue
end

b usb.c:173
commands 5
  continue
end

target extended-remote localhost:3333
run
