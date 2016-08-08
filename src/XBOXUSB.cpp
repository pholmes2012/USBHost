/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "XBOXUSB.h"

#define LOCAL_DEBUG

#ifdef LOCAL_DEBUG
#define DEBUGOUT debugOut
#else
#define DEBUGOUT
#endif

#ifndef NULL
#define NULL 0
#endif

void debugOut(const char* text, ...)
{
	if (Serial)
	{
		char str[256];
		va_list ap;
		va_start(ap, text);
		vsprintf (str, text, ap);
		va_end(ap);
		Serial.println(str);
	}
}

//#define PRINTREPORT // Uncomment to print the report send by the Xbox 360 Controller

XBOXUSB::XBOXUSB() :
							m_pUsb(NULL), // pointer to USB class instance - mandatory
							bAddress(0), // device address - mandatory
							bPollEnable(false)
{
}

XBOXUSB::XBOXUSB(USBHost* pUsb) :
							bAddress(0), // device address - mandatory
							bPollEnable(false)
{
	Begin(pUsb);
}

void XBOXUSB::Begin(USBHost* pUsb)
{
	m_pUsb = pUsb;

	DEBUGOUT("XBOXUSB Begin");

	for (uint8_t i = 0; i < XBOX_MAX_ENDPOINTS; i++)
	{
			epInfo[i].deviceEpNum = 0;
			epInfo[i].maxPktSize = (i) ? 0 : 8;
			epInfo[i].bmSndToggle = 0;
			epInfo[i].bmRcvToggle = 0;
			epInfo[i].bmNakPower = (i) ? USB_NAK_NOWAIT : USB_NAK_MAX_POWER;
	}

	if(pUsb) // register in USB subsystem
		pUsb->RegisterDeviceClass(this); //set devConfig[] entry
}

uint32_t XBOXUSB::Init(uint32_t parent, uint32_t port, uint32_t lowspeed)
{
	uint8_t buf[sizeof (USB_DEVICE_DESCRIPTOR)];
	USB_DEVICE_DESCRIPTOR* udd = reinterpret_cast<USB_DEVICE_DESCRIPTOR*>(buf);
	uint8_t rcode;
	UsbDevice *p = NULL;
	EpInfo *oldep_ptr = NULL;

	// get memory address of USB device address pool
	AddressPool &addrPool = m_pUsb->GetAddressPool();

	memset(buf, 0xaa, sizeof(buf));

	epInfo[XBOX_CONTROL_PIPE].deviceEpNum = 0; // XBOX 360 control endpoint
	epInfo[XBOX_INPUT_PIPE].ID = 0x8700;
	epInfo[XBOX_INPUT_PIPE].deviceEpNum = 1; // XBOX 360 report endpoint
	epInfo[XBOX_INPUT_PIPE].ID = 0x8701;
	epInfo[XBOX_OUTPUT_PIPE].deviceEpNum = 2; // XBOX 360 output endpoint
	epInfo[XBOX_OUTPUT_PIPE].ID = 0x8702;

	// check if address has already been assigned to an instance
	if(bAddress)
	{
		debugOut("Address in use");
		return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
	}

	// Get pointer to pseudo device with address 0 assigned
	p = addrPool.GetUsbDevicePtr(0);
	if (!p)
	{
		debugOut("Address not found");
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
	}
	else if (!p->epinfo)
	{
		debugOut("epinfo is null");
		return USB_ERROR_EPINFO_IS_NULL;
	}

	// Save old pointer to EP_RECORD of address 0
	oldep_ptr = p->epinfo;

	// Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
	p->epinfo = epInfo;

	p->lowspeed = lowspeed;

	// Restore p->epinfo
	p->epinfo = oldep_ptr;

	// Get device descriptor
	rcode = m_pUsb->getDevDescr(0, 0, sizeof (USB_DEVICE_DESCRIPTOR), (uint8_t*)buf); // Get device descriptor - addr, ep, nbytes, data
	if(rcode)
		goto FailGetDevDescr;

	DEBUGOUT("XBOXUSB::Init: VID %04x, PID %04x", udd->idVendor, udd->idProduct);
	if (VIDPIDOK(udd->idVendor, udd->idProduct)==false)
	{
		if (udd->idProduct == XBOX_WIRELESS_PID)
			debugOut("You have plugged in a wireless Xbox 360 controller - it doesn't support USB communication");
		else
			debugOut("This library only supports Xbox 360 controllers via USB");
		goto FailUnknownDevice;
	}

	// Allocate new address according to device class
	bAddress = addrPool.AllocAddress(parent, false, port);
	if(!bAddress)
		return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;

	// Extract Max Packet Size from device descriptor
	epInfo[0].maxPktSize = udd->bMaxPacketSize0;

	// Assign new address to the device
	rcode = m_pUsb->setAddr(0, 0, bAddress);
	if(rcode)
	{
		p->lowspeed = false;
		addrPool.FreeAddress(bAddress);
		bAddress = 0;
		return rcode;
	}
	DEBUGOUT("XBOXUSB::Init: Addr 0x%x", bAddress);
	//delay(300); // Spec says you should wait at least 200ms

	p->lowspeed = false;

	//get pointer to assigned address record
	p = addrPool.GetUsbDevicePtr(bAddress);
	if(!p)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

	p->lowspeed = lowspeed;

	// Assign epInfo to epinfo pointer - only EP0 is known
	rcode = m_pUsb->setEpInfoEntry(bAddress, 1, epInfo);
	if(rcode)
		goto FailSetDevTblEntry;

	/* The application will work in reduced host mode, so we can save program and data
	   memory space. After verifying the VID we will use known values for the
	   configuration values for device, interface, endpoints and HID for the XBOX360 Controllers */

	/* Initialize data structures for endpoints of device */
	epInfo[XBOX_INPUT_PIPE].epAttribs = USB_TRANSFER_TYPE_INTERRUPT;
	epInfo[XBOX_INPUT_PIPE].bmNakPower = USB_NAK_NOWAIT; // Only poll once for interrupt endpoints
	epInfo[XBOX_INPUT_PIPE].maxPktSize = EP_MAXPKTSIZE;
	epInfo[XBOX_INPUT_PIPE].bmSndToggle = 0;
	epInfo[XBOX_INPUT_PIPE].bmRcvToggle = 0;
	epInfo[XBOX_INPUT_PIPE].hostPipeNum = UHD_Pipe_Alloc(bAddress,
														 epInfo[XBOX_INPUT_PIPE].deviceEpNum,
														 UOTGHS_HSTPIPCFG_PTYPE_BLK,
														 UOTGHS_HSTPIPCFG_PTOKEN_IN,
														 epInfo[XBOX_INPUT_PIPE].maxPktSize,
														 0,
														 UOTGHS_HSTPIPCFG_PBK_1_BANK);

	epInfo[XBOX_OUTPUT_PIPE].epAttribs = USB_TRANSFER_TYPE_INTERRUPT;
	epInfo[XBOX_OUTPUT_PIPE].bmNakPower = USB_NAK_NOWAIT; // Only poll once for interrupt endpoints
	epInfo[XBOX_OUTPUT_PIPE].maxPktSize = EP_MAXPKTSIZE;
	epInfo[XBOX_OUTPUT_PIPE].bmSndToggle = 0;
	epInfo[XBOX_OUTPUT_PIPE].bmRcvToggle = 0;
	epInfo[XBOX_OUTPUT_PIPE].hostPipeNum = UHD_Pipe_Alloc(bAddress,
														  epInfo[XBOX_OUTPUT_PIPE].deviceEpNum,
														  UOTGHS_HSTPIPCFG_PTYPE_BLK,
														  UOTGHS_HSTPIPCFG_PTOKEN_OUT,
														  epInfo[XBOX_OUTPUT_PIPE].maxPktSize,
														  0,
														  UOTGHS_HSTPIPCFG_PBK_1_BANK);

	rcode = m_pUsb->setEpInfoEntry(bAddress, 3, epInfo);
	if(rcode)
		goto FailSetDevTblEntry;

	delay(200); // Give time for address change

	rcode = m_pUsb->setConf(bAddress, epInfo[XBOX_CONTROL_PIPE].deviceEpNum, 1);
	if(rcode)
		goto FailSetConfDescr;

	onInit();

	Xbox360Connected = true;
	bPollEnable = true;

	debugOut("Xbox 360 Controller Connected");

	return 0; // Successful configuration

	/* Diagnostic messages */
FailGetDevDescr:
	goto Fail;
FailSetDevTblEntry:
	goto Fail;
FailSetConfDescr:
	goto Fail;
FailUnknownDevice:
	rcode = USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;
Fail:
	Release();

	return rcode;
}

/* Performs a cleanup after failed Init() attempt */
uint32_t XBOXUSB::Release()
{
	Xbox360Connected = false;
	m_pUsb->GetAddressPool().FreeAddress(bAddress);
	bAddress = 0;
	bPollEnable = false;
	return 0;
}

uint32_t XBOXUSB::Poll()
{
	if(!bPollEnable)
		return 0;
	uint32_t BUFFER_SIZE = EP_MAXPKTSIZE;
	m_pUsb->inTransfer(bAddress, epInfo[XBOX_INPUT_PIPE].deviceEpNum, &BUFFER_SIZE, m_readBuffer); // input on endpoint 1
	readReport();
#ifdef PRINTREPORT
	printReport(); // Uncomment "#define PRINTREPORT" to print the report send by the Xbox 360 Controller
#endif
	return 0;
}

void XBOXUSB::readReport()
{
	if (m_readBuffer[0] == 0x00 && m_readBuffer[1] == 0x14) // Check if it's the correct report - the controller also sends different status reports
	{
		ButtonState = (uint32_t)(m_readBuffer[5] | ((uint16_t)m_readBuffer[4] << 8) | ((uint32_t)m_readBuffer[3] << 16) | ((uint32_t)m_readBuffer[2] << 24));

		hatValue[LeftHatX] = (int16_t)(((uint16_t)m_readBuffer[7] << 8) | m_readBuffer[6]);
		hatValue[LeftHatY] = (int16_t)(((uint16_t)m_readBuffer[9] << 8) | m_readBuffer[8]);
		hatValue[RightHatX] = (int16_t)(((uint16_t)m_readBuffer[11] << 8) | m_readBuffer[10]);
		hatValue[RightHatY] = (int16_t)(((uint16_t)m_readBuffer[13] << 8) | m_readBuffer[12]);

		if (ButtonState != OldButtonState)
		{
			ButtonClickState = (ButtonState >> 16) & ((~OldButtonState) >> 16); // Update click state variable, but don't include the two trigger buttons L2 and R2
			if(((uint8_t)OldButtonState) == 0 && ((uint8_t)ButtonState) != 0) // The L2 and R2 buttons are special as they are analog buttons
					R2Clicked = true;
			if((uint8_t)(OldButtonState >> 8) == 0 && (uint8_t)(ButtonState >> 8) != 0)
					L2Clicked = true;
			OldButtonState = ButtonState;
		}
	}
}

//Uncomment "#define PRINTREPORT" to print the report send by the Xbox 360 Controller
void XBOXUSB::printReport()
{
#ifdef PRINTREPORT
	if(m_readBuffer == NULL)
			return;
	for(uint8_t i = 0; i < XBOX_REPORT_BUFFER_SIZE; i++) {
			D_PrintHex<uint8_t > (m_readBuffer[i], 0x80);
			Notify(PSTR(" "), 0x80);
	}
	Notify(PSTR("\r\n"), 0x80);
#endif
}

uint8_t XBOXUSB::getButtonPress(ButtonEnum b)
{
	if(b == L2) // These are analog buttons
		return (uint8_t)(ButtonState >> 8);
	else if(b == R2)
		return (uint8_t)ButtonState;
	return (bool)(ButtonState & ((uint32_t)pgm_read_word(&XBOX_BUTTONS[(uint8_t)b]) << 16));
}

bool XBOXUSB::getButtonClick(ButtonEnum b)
{
	if(b == L2)
	{
		if(L2Clicked)
		{
			L2Clicked = false;
			return true;
		}
		return false;
	}
	else if(b == R2)
	{
		if(R2Clicked)
		{
			R2Clicked = false;
			return true;
		}
		return false;
	}
	uint16_t button = pgm_read_word(&XBOX_BUTTONS[(uint8_t)b]);
	bool click = (ButtonClickState & button);
	ButtonClickState &= ~button; // clear "click" event

	return click;
}

int16_t XBOXUSB::getAnalogHat(AnalogHatEnum a)
{
	return hatValue[a];
}


/* Xbox Controller commands */
void XBOXUSB::XboxCommand(uint8_t* data, uint32_t nbytes)
{
	//bmRequest = Host to device (0x00) | Class (0x20) | Interface (0x01) = 0x21, bRequest = Set Report (0x09), Report ID (0x00), Report Type (Output 0x02), interface (0x00), datalength, datalength, data)
	m_pUsb->ctrlReq(bAddress,
					epInfo[XBOX_CONTROL_PIPE].hostPipeNum,
					bmREQ_HIDOUT,
					HID_REQUEST_SET_REPORT,
					0x00, 0x02,
					0x00,
					(uint16_t)nbytes,
					nbytes,
					data,
					NULL);
}

void XBOXUSB::setLedRaw(uint8_t value)
{
	writeBuf[0] = 0x01;
	writeBuf[1] = 0x03;
	writeBuf[2] = value;

	XboxCommand(writeBuf, 3);
}

void XBOXUSB::setLedOn(LEDEnum led)
{
	if(led == OFF)
		setLedRaw(0);
	else if(led != ALL) // All LEDs can't be on a the same time
		setLedRaw(pgm_read_byte(&XBOX_LEDS[(uint8_t)led]) + 4);
}

void XBOXUSB::setLedBlink(LEDEnum led)
{
	setLedRaw(pgm_read_byte(&XBOX_LEDS[(uint8_t)led]));
}

void XBOXUSB::setLedMode(LEDModeEnum ledMode)
{ // This function is used to do some special LED stuff the controller supports
	setLedRaw((uint8_t)ledMode);
}

void XBOXUSB::setRumbleOn(uint8_t lValue, uint8_t rValue)
{
	writeBuf[0] = 0x00;
	writeBuf[1] = 0x08;
	writeBuf[2] = 0x00;
	writeBuf[3] = lValue; // big weight
	writeBuf[4] = rValue; // small weight
	writeBuf[5] = 0x00;
	writeBuf[6] = 0x00;
	writeBuf[7] = 0x00;

	XboxCommand(writeBuf, 8);
}

void XBOXUSB::onInit()
{
	if(pFuncOnInit)
		pFuncOnInit(); // Call the user function
	else
		setLedOn(static_cast<LEDEnum>(LED1));
}
