#ifndef STELL_USB_H
#define STELL_USB_H

#include <stm32f4xx.h>

#define USB_FIFO_EP0_SZ 0x20

#define USB_GET_STATUS 0x00
#define USB_SET_ADDRESS 0x05
#define USB_GET_DESCRIPTOR 0x06
#define USB_GET_CONFIGURATION 0x08
#define USB_SET_CONFIGURATION 0x09

#define USB_DESCRIPTOR_DEVICE 1
#define USB_DESCRIPTOR_CONFIGURATION 2
#define USB_DESCRIPTOR_STRING 3
#define USB_DESCRIPTOR_INTERFACE 4
#define USB_DESCRIPTOR_ENDPOINT 5

#define PKTSTS_OUT_RX 0x02
#define PKTSTS_OUT_COMPLETE 0x03
#define PKTSTS_SETUP_COMPLETE 0x04
#define PKTSTS_SETUP_RX 0x06

#define MPSIZ_64B 0x00

#define USB_BCD_2 0x0200
#define USB_CDC_CLASS 0x02
#define USB_CDC_ACM_SUBCLASS 0x02
#define USB_NO_SPECIFIC_PROTOCOL 0x00

struct UsbDeviceDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
};

struct UsbControlRequest {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint8_t wValueL;
    uint8_t wValueH;
    uint16_t wIndex;
    uint16_t wLength;
};

extern struct UsbDeviceDescriptor stellDeviceDescriptor;
extern uint16_t stellStatusResponse;

/**
 * Return a pointer to the starting address for the registers which configure the
 * specified endpoint.
 * @param ep Endpoint number
 * @return Pointer to struct to configure the specified OUT endpoint
 */
USB_OTG_OUTEndpointTypeDef *usbEpout(uint8_t ep);

/**
 * Return a pointer to the starting address for the registers which configure the
 * specified endpoint.
 * @param ep Endpoint number
 * @return Pointer to struct to configure the specified IN endpoint
 */
USB_OTG_INEndpointTypeDef *usbEpin(uint8_t ep);

/**
 * Return the start address for an endpoint's FIFO. The start addresses are generated based on
 * section 22.16.1 "CSR memory map".
 * @param ep Endpoint number
 * @return Pointer to the FIFO's start address
 */
uint32_t *usbEpFifo(uint8_t ep);

/**
 * Write data to a USB FIFO with disregard to the core status
 * @param fifo FIFO where data will be written to
 * @param data Data to be written
 * @param len Amount of bytes to be written
 */
void usbRawWrite(volatile uint32_t *fifo, void *data, uint8_t len);

/**
 * Read a SETUP packet from the specified USB FIFO.
 * @param size
 * @param ep
 * @param fifo
 */
void usbReadSetup(const volatile uint32_t *fifo, struct UsbControlRequest *request);

/**
 *
 * @param request
 */
void usbProcessSetup(struct UsbControlRequest *request);

/**
 * Initialize the USB OTG FS core according to the procedure outlined in section 22.17.1/3.
 * Device is setup with no support for SRP or HNP. VBUS and SOF sensing is also
 * disabled. Core is forced to device mode.
 */
void usbdevInit(void);

/**
 * Power-up and enable the USB transceiver and its interrupts. Start USB communication in practical
 * terms.
 */
void usbdevStart(void);

/**
 * Task to handle USB events. Called by the interrupt handler.
 */
void usbdevProcess(void);

#endif  // STELL_USB_H
