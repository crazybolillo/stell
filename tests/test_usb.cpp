#include <gtest/gtest.h>

extern "C" {
#include "usb.h"
}

class TestUsb : public testing::Test {};

TEST_F(TestUsb, TestEPFIFO) {
    ASSERT_EQ((uint32_t *)(USB_OTG_FS_PERIPH_BASE + 0x1000), usbEpFifo(0));
    ASSERT_EQ((uint32_t *)(USB_OTG_FS_PERIPH_BASE + 0x2000), usbEpFifo(1));
    ASSERT_EQ((uint32_t *)(USB_OTG_FS_PERIPH_BASE + 0x3000), usbEpFifo(2));
}

TEST_F(TestUsb, TestEPINOUT) {
    ASSERT_EQ((void *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE), usbEpin(0));
    ASSERT_EQ((void *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + 0x20), usbEpin(1));
    ASSERT_EQ((void *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + 0x40), usbEpin(2));

    ASSERT_EQ((void *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE), usbEpout(0));
    ASSERT_EQ((void *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + 0x20), usbEpout(1));
    ASSERT_EQ((void *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + 0x40), usbEpout(2));
}

TEST_F(TestUsb, TestReadSetup) {
    UsbControlRequest request = {0};
    uint32_t rxFifo[2] = {0x01000680, 0x120000};

    usbReadSetup(rxFifo, &request);
    ASSERT_EQ(0x80, request.bmRequestType);
    ASSERT_EQ(USB_GET_DESCRIPTOR, request.bRequest);
    ASSERT_EQ(USB_DESCRIPTOR_DEVICE, request.wValueH);
    ASSERT_EQ(18, request.wLength);

    rxFifo[0] = 0x80;
    rxFifo[1] = 0x20000;
    usbReadSetup(rxFifo, &request);
    ASSERT_EQ(0x80, request.bmRequestType);
    ASSERT_EQ(USB_GET_STATUS, request.bRequest);
    ASSERT_EQ(2, request.wLength);
}

TEST_F(TestUsb, TestUsbWrite) {
    UsbDeviceDescriptor descriptor = {0};
    usbRawWrite((uint32_t *)&descriptor, &stellDeviceDescriptor, sizeof(stellDeviceDescriptor));

    ASSERT_EQ(stellDeviceDescriptor.bLength, descriptor.bLength);
    ASSERT_EQ(stellDeviceDescriptor.bDescriptorType, descriptor.bDescriptorType);
    ASSERT_EQ(stellDeviceDescriptor.bcdUSB, descriptor.bcdUSB);
    ASSERT_EQ(stellDeviceDescriptor.bDeviceClass, descriptor.bDeviceClass);
    ASSERT_EQ(stellDeviceDescriptor.bDeviceSubClass, descriptor.bDeviceSubClass);
    ASSERT_EQ(stellDeviceDescriptor.bDeviceProtocol, descriptor.bDeviceProtocol);
    ASSERT_EQ(stellDeviceDescriptor.bMaxPacketSize0, descriptor.bMaxPacketSize0);
    ASSERT_EQ(stellDeviceDescriptor.idVendor, descriptor.idVendor);
    ASSERT_EQ(stellDeviceDescriptor.idProduct, descriptor.idProduct);
    ASSERT_EQ(stellDeviceDescriptor.bcdDevice, descriptor.bcdDevice);
    ASSERT_EQ(stellDeviceDescriptor.iManufacturer, descriptor.iManufacturer);
    ASSERT_EQ(stellDeviceDescriptor.iProduct, descriptor.iProduct);
    ASSERT_EQ(stellDeviceDescriptor.iSerialNumber, descriptor.iSerialNumber);
    ASSERT_EQ(stellDeviceDescriptor.bNumConfigurations, descriptor.bNumConfigurations);

    uint32_t fifo[3] = {0};
    uint8_t data[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    usbRawWrite(fifo, data, 5);

    ASSERT_EQ(fifo[0], 0x04030201);
    ASSERT_EQ(fifo[1], 0x05);
    ASSERT_EQ(fifo[2], 0x00);
}
