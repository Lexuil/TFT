import machine
import time
from registers import *


class ILI9341(object):
    # Pinout, change within or outside class for your use case
    PINS = [32,33,25,26,27,14,12,13,16,17,4,19,22]
    # Pin names, don't change
    PIN_NAMES = ['RST','CS','RS','WR','RD','D0','D1','D2','D3','D4','D5','D6','D7']

    # Dict of pins
    pins = {}

    # Pin mode, push-pull control
    PIN_MODE = machine.Pin.OUT

    TFTWIDTH  = 240
    TFTHEIGHT = 320

    # Designation of T/F for command data and command modes
    LCD_CDT = True
    LCD_CMD = False

    # Timing constants
    E_PULSE = 50
    E_DELAY = 50

    def init(self):
        # Initialise pins
        for pin, pin_name in zip(self.PINS, self.PIN_NAMES):
            # setattr(self, 'LCD_'+pin_name,   # Unsupported
            #     machine.Pin(pin, self.PIN_MODE))
            self.pins['LCD_'+pin_name] = machine.Pin(pin, self.PIN_MODE)

        self.pin_action('LCD_CS', False)

        # Initialise display
        self.WriteRegister8(ILI9341_SOFTRESET, 0)
        self.udelay(50)
        self.WriteRegister8(ILI9341_DISPLAYOFF, 0)

        self.WriteRegister8(ILI9341_POWERCONTROL1, 0x23)
        self.WriteRegister8(ILI9341_POWERCONTROL2, 0x10)
        self.WriteRegister16(ILI9341_VCOMCONTROL1, 0x2B2B)
        self.WriteRegister8(ILI9341_VCOMCONTROL2, 0xC0)
        self.WriteRegister8(ILI9341_MEMCONTROL, ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR)
        self.WriteRegister8(ILI9341_PIXELFORMAT, 0x55)
        self.WriteRegister16(ILI9341_FRAMECONTROL, 0x001B)

        self.WriteRegister8(ILI9341_ENTRYMODE, 0x07)

        self.WriteRegister8(ILI9341_SLEEPOUT, 0)
        self.udelay(150)
        self.WriteRegister8(ILI9341_DISPLAYON, 0)
        self.udelay(500)

        self.setAddrWindow(0, 0, self.TFTWIDTH-1, self.TFTHEIGHT-1)

        self.pin_action('LCD_CS', True)

    def setAddrWindow(self, x1, x2, y1, y2):

        self.pin_action('LCD_CS', False)

        t = x1;
        t <<= 16;
        t |= x2;
        self.WriteRegister32(ILI9341_COLADDRSET, t)
        t = y1;
        t <<= 16;
        t |= y2;
        self.WriteRegister32(ILI9341_PAGEADDRSET, t)

        self.pin_action('LCD_CS', True)

    def drawPixel(self, x, y, color):

        self.pin_action('LCD_CS', False)

        self.setAddrWindow(x, y, self.TFTWIDTH-1, self.TFTHEIGHT-1)

        self.lcd_byte(0x2C,self.LCD_CMD)
        self.lcd_byte(color >> 8,self.LCD_CDT)
        self.lcd_byte(color,self.LCD_CDT)

        self.pin_action('LCD_CS', True)

    def flood(self, color, lent):

        self.pin_action('LCD_CS', False)

        self.lcd_byte(0x2C,self.LCD_CMD)
        self.lcd_byte(color >> 8,self.LCD_CDT)
        self.lcd_byte(color,self.LCD_CDT)

        blocks = lent/64

        while blocks:
            blocks -= 1
            i = 17
            while i:
                self.lcd_byte(color >> 8,self.LCD_CDT)
                self.lcd_byte(color,self.LCD_CDT)
                self.lcd_byte(color >> 8,self.LCD_CDT)
                self.lcd_byte(color,self.LCD_CDT)
                self.lcd_byte(color >> 8,self.LCD_CDT)
                self.lcd_byte(color,self.LCD_CDT)
                self.lcd_byte(color >> 8,self.LCD_CDT)
                self.lcd_byte(color,self.LCD_CDT)
                i = i - 1

        self.pin_action('LCD_CS', True)

    def fillScreen(self,color):

        self.setAddrWindow(0, 0, self.TFTWIDTH-1, self.TFTHEIGHT-1)
        self.flood(color, self.TFTHEIGHT * self.TFTWIDTH)

    def lcd_byte(self, bits, mode):
        # Send byte to data pins
        # bits = data
        # mode = True  for character
        #        False for command

        self.pin_action('LCD_RS', mode) # RS

        # High bits
        self.pin_action('LCD_D0', False)
        self.pin_action('LCD_D1', False)
        self.pin_action('LCD_D2', False)
        self.pin_action('LCD_D3', False)
        self.pin_action('LCD_D4', False)
        self.pin_action('LCD_D5', False)
        self.pin_action('LCD_D6', False)
        self.pin_action('LCD_D7', False)
        if bits&0x01==0x01:
            self.pin_action('LCD_D0', True)
        if bits&0x02==0x02:
            self.pin_action('LCD_D1', True)
        if bits&0x04==0x04:
            self.pin_action('LCD_D2', True)
        if bits&0x08==0x08:
            self.pin_action('LCD_D3', True)
        if bits&0x10==0x10:
            self.pin_action('LCD_D4', True)
        if bits&0x20==0x20:
            self.pin_action('LCD_D5', True)
        if bits&0x40==0x40:
            self.pin_action('LCD_D6', True)
        if bits&0x80==0x80:
            self.pin_action('LCD_D7', True)

        # Toggle 'Enable' pin
        self.udelay(self.E_DELAY)
        self.pin_action('LCD_WR', False)
        self.udelay(self.E_PULSE)
        self.pin_action('LCD_WR', True)
        self.udelay(self.E_DELAY)

    def WriteRegister8(self, register, cdata):
        self.lcd_byte(register,self.LCD_CMD)
        self.lcd_byte(cdata,self.LCD_CDT)

    def WriteRegister16(self, register, cdata):
        self.lcd_byte(register,self.LCD_CMD)
        self.lcd_byte(cdata & 0xFF,self.LCD_CDT)
        self.lcd_byte((cdata & 0xFF00) >> 8,self.LCD_CDT)

    def WriteRegister32(self, register, cdata):
        self.lcd_byte(register,self.LCD_CMD)
        self.lcd_byte(cdata & 0xFF,self.LCD_CDT)
        self.lcd_byte((cdata & 0xFF00) >> 8,self.LCD_CDT)
        self.lcd_byte((cdata & 0xFF0000) >> 16,self.LCD_CDT)

    def udelay(self, us):
        # Delay by us microseconds, set as function for portability
        time.sleep_us(us)

    def pin_action(self, pin, high):
        # Pin high/low functions, set as function for portability
        if high:
            self.pins[pin].value(1)
        else:
            self.pins[pin].value(0)