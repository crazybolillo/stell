#include "usb.h"

struct UsbDeviceDescriptor stellDeviceDescriptor = {
    .bLength = 18,
    .bDescriptorType = USB_DESCRIPTOR_DEVICE,
    .bcdUSB = USB_BCD_2,
    .bDeviceClass = USB_CDC_CLASS,
    .bDeviceSubClass = USB_CDC_ACM_SUBCLASS,
    .bDeviceProtocol = USB_NO_SPECIFIC_PROTOCOL,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0100,
    .iManufacturer = 0,
    .iProduct = 0,
    .iSerialNumber = 0,
    .bNumConfigurations = 1,
};
uint16_t stellStatusResponse = 0;

static struct UsbControlRequest controlRequest;

static USB_OTG_GlobalTypeDef* const OTG = USB_OTG_FS;
static USB_OTG_DeviceTypeDef* const OTGD = (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
static volatile uint32_t* const OTG_CLK = (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE);
const uint32_t USB_INTERRUPTS = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |
                                USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_OEPINT |
                                USB_OTG_GINTMSK_IEPINT;

static void enable_otg_clk() { *OTG_CLK = 0; }
static void enable_soft_disconnect() { OTGD->DCTL |= USB_OTG_DCTL_SDIS; }
static void disable_soft_disconnect() { OTGD->DCTL &= ~USB_OTG_DCTL_SDIS; }
static inline void flushTX() {
    OTG->GRSTCTL |= (0x10 << USB_OTG_DIEPCTL_TXFNUM_Pos) | USB_OTG_GRSTCTL_TXFFLSH;
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) continue;
}

USB_OTG_INEndpointTypeDef* usbEpin(uint8_t ep) {
    return (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep << 5));
}

USB_OTG_OUTEndpointTypeDef* usbEpout(uint8_t ep) {
    return (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep << 5));
}

uint32_t* usbEpFifo(uint8_t ep) {
    return (uint32_t*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep << 12));
}

void usbWrite(uint8_t ep, void* data, uint8_t len) {
    USB_OTG_INEndpointTypeDef* endpoint = usbEpin(ep);
    volatile uint32_t* fifo = usbEpFifo(ep);

    uint16_t wordLen = (len + 3) >> 2;
    if (wordLen > endpoint->DTXFSTS) {
        return;
    }
    if ((ep != 0) && (endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA)) {
        return;
    }

    endpoint->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
    endpoint->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    usbRawWrite(fifo, data, len);
}

void usbRawWrite(volatile uint32_t* fifo, void* data, uint8_t len) {
    uint32_t fifoWord;
    uint32_t* buffer = (uint32_t*)data;
    uint8_t remains = len;
    for (uint8_t idx = 0; idx < len; idx += 4, remains -= 4, buffer++) {
        switch (remains) {
            case 0:
                break;
            case 1:
                fifoWord = *buffer & 0xFF;
                *fifo = fifoWord;
                break;
            case 2:
                fifoWord = *buffer & 0xFFFF;
                *fifo = fifoWord;
                break;
            case 3:
                fifoWord = *buffer & 0xFFFFFF;
                *fifo = fifoWord;
                break;
            default:
                *fifo = *buffer;
                break;
        }
#ifdef BUILD_MACHINE
        fifo++;
#endif
    }
}

static void processDeviceToHostSetup(struct UsbControlRequest* request) {
    switch (request->bRequest) {
        case USB_GET_STATUS:
            usbWrite(0x00, &stellStatusResponse, 2);
            break;
        case USB_GET_DESCRIPTOR:
            usbWrite(0x00, &stellDeviceDescriptor, 18);
            break;
        default:
            break;
    }
}

static void processHostToDeviceSetup(struct UsbControlRequest* request) {
    uint8_t address;
    switch (request->bRequest) {
        case USB_SET_ADDRESS:
            address = request->wValueL & 0xEF;
            OTGD->DCFG |= address << 4;
            usbWrite(0, 0, 0);
            break;
    }
}

void usbProcessSetup(struct UsbControlRequest* request) {
    if (request->bmRequestType & 0x80) {
        processDeviceToHostSetup(request);
    } else {
        processHostToDeviceSetup(request);
    }
}

void usbReadSetup(const volatile uint32_t* fifo, struct UsbControlRequest* request) {
    uint32_t* buffer = (uint32_t*)request;

    for (uint16_t idx = 0; idx < 2; idx++, buffer++) {
        *buffer = *fifo;
#ifdef BUILD_MACHINE
        fifo++;
#endif
    }
}

/**
 * Called whenever there is a USB packet to read, this is signaled by RXFLVL being set on GINTSTS.
 */
static void usbRead() {
    uint32_t readStatus = OTG->GRXSTSP;
    uint8_t epNum = readStatus & 0xF;
    uint16_t byteCount = (readStatus & USB_OTG_GRXSTSP_BCNT) >> 4;

    switch ((readStatus & USB_OTG_GRXSTSP_PKTSTS) >> 17) {
        case PKTSTS_OUT_RX:
            break;
        case PKTSTS_SETUP_RX:
            if ((epNum != 0) || (byteCount != 8)) {
                return;
            }
            usbReadSetup(usbEpFifo(0), &controlRequest);
            break;
        case PKTSTS_SETUP_COMPLETE:
            break;
    }
}

static void usbEpInterruptOut() {
    USB_OTG_OUTEndpointTypeDef* ep = usbEpout(0);
    if (ep->DOEPINT & USB_OTG_DOEPINT_STUP) {
        usbProcessSetup(&controlRequest);
    } else if (ep->DOEPINT & USB_OTG_DOEPINT_OTEPDIS) {
        (void)ep->DOEPINT;
    } else if (ep->DOEPINT & USB_OTG_DOEPINT_OTEPSPR) {
        ep->DOEPTSIZ |= (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos);
        ep->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
    } else if (ep->DOEPINT & USB_OTG_DOEPINT_XFRC) {
        ep->DOEPTSIZ |= (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos);
        ep->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
    }
    ep->DOEPINT = ep->DOEPINT;
}

static void usbEpInterruptIn() {
    USB_OTG_INEndpointTypeDef* ep = usbEpin(0);
    USB_OTG_OUTEndpointTypeDef* epOut = usbEpout(0);
    if (ep->DIEPINT & USB_OTG_DIEPINT_XFRC) {
        epOut->DOEPTSIZ |= (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos);
        epOut->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
    }
    ep->DIEPINT = ep->DIEPINT;
}

static void usbInitPeriph() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (0x02 << GPIO_MODER_MODER11_Pos) | (0x02 << GPIO_MODER_MODER12_Pos);
    GPIOA->OSPEEDR |= (0x02 << GPIO_OSPEEDR_OSPEED11_Pos) | (0x02 << GPIO_OSPEEDR_OSPEED12_Pos);
    GPIOA->AFR[1] |= (0x0A << GPIO_AFRH_AFSEL11_Pos) | (0x0A << GPIO_AFRH_AFSEL12_Pos);

    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    while ((OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) continue;
}

static void usbdevInitDev(void) {
    OTG->GUSBCFG &= ~(USB_OTG_GUSBCFG_SRPCAP | USB_OTG_GUSBCFG_TRDT);
    OTG->GUSBCFG |= (USB_OTG_GUSBCFG_FDMOD | (0x06 << USB_OTG_GUSBCFG_TRDT_Pos));
    OTG->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;

    enable_otg_clk();
    enable_soft_disconnect();

    OTGD->DCFG &= ~USB_OTG_DCFG_DAD;
    OTGD->DCFG |= (0x03 << USB_OTG_DCFG_DSPD_Pos) | USB_OTG_DCFG_NZLSOHSK;

    OTG->GINTMSK = USB_INTERRUPTS;
    OTG->GINTSTS = 0xFFFFFFFF;
    OTG->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
}

void usbdevInit(void) {
    usbInitPeriph();
    usbdevInitDev();
}

void usbdevStart(void) {
    OTG->GCCFG |= USB_OTG_GCCFG_PWRDWN;
    disable_soft_disconnect();
}

static void usbReset(void) {
    OTGD->DCFG &= ~USB_OTG_DCFG_DAD;
    OTG->GRXFSIZ = 0x30;
    OTG->DIEPTXF0_HNPTXFSIZ = (16 << USB_OTG_TX0FD_Pos) | 0x30;
    OTGD->DCTL |= USB_OTG_DCTL_CGONAK;
}

static void usbAfterReset(void) {
    OTGD->DAINTMSK = ((1 << 16) | 1);
    OTGD->DOEPMSK = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_OTEPDM |
                    USB_OTG_DOEPMSK_OTEPSPRM;
    OTGD->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;

    USB_OTG_INEndpointTypeDef* in0 = usbEpin(0);
    in0->DIEPCTL &= (0x03 << USB_OTG_DIEPCTL_MPSIZ_Pos);

    USB_OTG_OUTEndpointTypeDef* out0 = usbEpout(0);
    out0->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos);
    out0->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
}

void usbdevProcess(void) {
    uint32_t interrupt = OTG->GINTSTS;
    if (interrupt & USB_OTG_GINTSTS_USBRST) {
        OTG->GINTSTS |= USB_OTG_GINTSTS_USBRST;
        usbReset();
    } else if (interrupt & USB_OTG_GINTSTS_ENUMDNE) {
        OTG->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
        usbAfterReset();
    } else if (interrupt & USB_OTG_GINTSTS_RXFLVL) {
        OTG->GINTSTS |= USB_OTG_GINTSTS_RXFLVL;
        usbRead();
    } else if (interrupt & USB_OTG_GINTSTS_OEPINT) {
        OTG->GINTSTS |= USB_OTG_GINTSTS_OEPINT;
        usbEpInterruptOut();
    } else if (interrupt & USB_OTG_GINTSTS_IEPINT) {
        OTG->GINTSTS |= USB_OTG_GINTMSK_OEPINT;
        usbEpInterruptIn();
    }
}
