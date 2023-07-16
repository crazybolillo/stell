# Automate common breakpoint and data extraction for USB debugging
set width 0
set height 0
set verbose off

b usbReset
commands 1
  continue
end

b usbRead
commands 2
  printf "GRXSTSR: %x\n", OTG->GRXSTSR
  continue
end

b usbWrite
commands 3
  continue
end

b usb.c:164
commands 4
  printf "EP0.DOEPINT: %x\n:", ep->DOEPINT
  continue
end

b usb.c:184
commands 5
  printf "EP0.DOEPINT: %x\n:", ep->DIEPINT
  continue
end

target extended-remote localhost:3333
run
