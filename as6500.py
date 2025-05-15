import RPi.GPIO as gpio
import spidev
import time


class AS6500:

    # ch1 and ch2 enabled, normal measurement
    # 1 ps lsb, common fifo read on, block wise read off
    conf_regs = [0b0000_0011, 0b0000_0011, 0b0100_0000,
                 0b0100_1000, 0b1110_1000, 0b0000_0001,     # ref clock div 125000 ps
                 0b1100_0000, 0b1010_0011,                  # internal 8 MHz reference clock 
                0xA1, 0x13, 0x00, 0x0A, 0xCC, 0x05, 0xF1, 0x7D, 0x04]

    spiopc_power = 0x30
    spiopc_init = 0x18
    spiopc_write_config = 0x80
    spiopc_read_config = 0x40
    spiopc_read_results = 0x60

    REF_CLK_INT = 1
    REF_CLK_EXT = 0

    CHANNEL_COMBINE_NO = 0
    CHANNEL_COMBINE_PULSE_DISTANCE = 1
    CHANNEL_COMBINE_PULSE_WIDTH = 2

    RESOLUTION_STANDARD = 0
    RESOLUTION_2X = 1
    RESOLUTION_4X = 1

    LSB_01 = 0.1    # 0.1 ps
    LSB_1 = 1       #   1 ps
    LSB_5 = 5       #   5 ps
    LSB_10 = 10
    
    X_OFF = 0
    X_ON = 1
    INT_CLK_FREQ_HZ = 8_000_000

    _lsb = LSB_1    # resolution of 1 ps
    print_channels_separator = '\n'

    def __init__(self, cs_pin, int_pin, rir_pin):
        self.cs = cs_pin
        self.int = int_pin
        self.rir = rir_pin
        self.refclk_period_ns = 0
        try:
            gpio.setwarnings(False)
            gpio.setmode(gpio.BCM)

            gpio.setup(self.int, gpio.IN)
            gpio.setup(self.cs, gpio.OUT)
            gpio.setup(self.rir, gpio.OUT)
            gpio.output(self.cs, gpio.LOW)
            gpio.output(self.rir, gpio.LOW)
            gpio.setup(int_pin, gpio.IN, pull_up_down=gpio.PUD_DOWN)

            self.spi = spidev.SpiDev()
            self.spi.open(0, 1)
            self.spi.max_speed_hz = 10_000_000
            self.spi.mode = 1
            gpio.output(self.cs, gpio.LOW)

            self.power_on_reset()
        except Exception as e:
            print(f"Exception {e}")
            self.spi.close()

    def _setbit(self, value, bit):
        return value | 1<<bit
    
    def _clearbit(self, value, bit):
        return value & ~(1<<bit)

    def _assignbit(self, value, bit, bitvalue):
        return self._setbit(value, bit) if bitvalue else self._clearbit(value, bit)
    
    def _getbit(self, value, bit):
         return (value & (1<<bit))>>bit
    
    def _comstart(self):
        gpio.output(self.cs, gpio.HIGH)
        gpio.output(self.cs, gpio.LOW)


    ########### CFG0 Register (Address 0)\
    @property
    def pin_ena_stop1(self):
        return self._getbit(self.conf_regs[0], 0)
    
    @pin_ena_stop1.setter
    def pin_ena_stop1(self, value):
        self.conf_regs[0] = self._assignbit(self.conf_regs[0], 0, value)

    @property
    def pin_ena_stop2(self):
        return self._getbit(self.conf_regs[0], 1)
    
    @pin_ena_stop2.setter
    def pin_ena_stop2(self, value):
        self.conf_regs[0] = self._assignbit(self.conf_regs[0], 1, value)

    @property
    def pin_ena_stop3(self):
        return self._getbit(self.conf_regs[0], 2)
    
    @pin_ena_stop3.setter
    def pin_ena_stop3(self, value):
        self.conf_regs[0] = self._assignbit(self.conf_regs[0], 2, value)

    @property
    def pin_ena_stop4(self):
        return self._getbit(self.conf_regs[0], 3)
    
    @pin_ena_stop4.setter
    def pin_ena_stop4(self, value):
        self.conf_regs[0] = self._assignbit(self.conf_regs[0], 3, value)

    @property
    def pin_ena_refclk(self):
        return self._getbit(self.conf_regs[0], 4)
    
    @pin_ena_refclk.setter
    def pin_ena_refclk(self, value):
        self.conf_regs[0] = self._assignbit(self.conf_regs[0], 4, value)

    @property
    def pin_ena_disable(self):
        return self._getbit(self.conf_regs[0], 6)
    
    @pin_ena_disable.setter
    def pin_ena_disable(self, value):
        self.conf_regs[0] = self._assignbit(self.conf_regs[0], 6, value)

    @property
    def pin_ena_rstidx(self):
        return self._getbit(self.conf_regs[0], 7)
    
    @pin_ena_rstidx.setter
    def pin_ena_rstidx(self, value):
        self.conf_regs[0] = self._assignbit(self.conf_regs[0], 7, value)

    ########### CFG1 Register (Address 1)
    @property
    def hit_ena_stop1(self):
        return self._getbit(self.conf_regs[1], 0)
    
    @hit_ena_stop1.setter
    def hit_ena_stop1(self, value):
        self.conf_regs[1] = self._assignbit(self.conf_regs[1], 0, value)

    @property
    def hit_ena_stop2(self):
        return self._getbit(self.conf_regs[1], 1)
    
    @hit_ena_stop2.setter
    def hit_ena_stop2(self, value):
        self.conf_regs[1] = self._assignbit(self.conf_regs[1], 1, value)

    @property
    def hit_ena_stop3(self):
        return self._getbit(self.conf_regs[1], 2)
    
    @hit_ena_stop3.setter
    def hit_ena_stop3(self, value):
        self.conf_regs[1] = self._assignbit(self.conf_regs[1], 2, value)

    @property
    def hit_ena_stop4(self):
        return self._getbit(self.conf_regs[1], 3)
    
    @hit_ena_stop4.setter
    def hit_ena_stop4(self, value):
        self.conf_regs[1] = self._assignbit(self.conf_regs[1], 3, value)

    @property
    def channel_combine(self):
        return (self._getbit(self.conf_regs[1], 5)<<1) + self._getbit(self.conf_regs[1], 4)
    
    @channel_combine.setter
    def channel_combine(self, value):
        if value not in [0, 1, 2]:
            raise ValueError("Channel combine must be 0 | 1 | 2")
        self.conf_regs[1] = self._assignbit(self.conf_regs[1], 5, value>>1)
        self.conf_regs[1] = self._assignbit(self.conf_regs[1], 4, value&0x01)

    @property
    def resolution(self):
        return (self._getbit(self.conf_regs[1], 7)<<1) + self._getbit(self.conf_regs[1], 6)
    
    @resolution.setter
    def resolution(self, value):
        if value not in [0, 1, 2]:
            raise ValueError("High resolution must be 0 | 1 | 2")
        self.conf_regs[1] = self._assignbit(self.conf_regs[1], 7, value>>1)
        self.conf_regs[1] = self._assignbit(self.conf_regs[1], 6, value&0x01)


    ########### CFG2 Register (Address 2)

    @property
    def common_fifo_read(self):
        return self._getbit(self.conf_regs[2], 6)

    @common_fifo_read.setter
    def common_fifo_read(self, value):
        self.conf_regs[2] = self._assignbit(self.conf_regs[2], 6, value)

    @property
    def blockwise_fifo_read(self):
        return self._getbit(self.conf_regs[2], 7)

    @blockwise_fifo_read.setter
    def blockwise_fifo_read(self, value):
        self.conf_regs[2] = self._assignbit(self.conf_regs[2], 7, value)


    ########### CFG3-7 Register (Address 3-7)

    @property
    def refclk(self):
        return self.REF_CLK_INT if (self.conf_regs[7] & 0x80) else self.REF_CLK_EXT
    
    @refclk.setter
    def refclk(self, value):
        if value == self.REF_CLK_EXT:
            if DEBUG:
                print("Setting external reference clock")
            self.conf_regs[0] = self._assignbit(self.conf_regs[0], 4, 1)    # enable PIN_ENA_REFCLK
            self.conf_regs[7] = self._assignbit(self.conf_regs[7], 7, 0)    # enable external clock signal
        else:
            if DEBUG:
                print("Setting internal reference @ 8MHz")
            self.conf_regs[7] = self._assignbit(self.conf_regs[7], 7, 1)    # enable onboard oscillator
            self.conf_regs[0] = self._assignbit(self.conf_regs[0], 4, 0)    # disable PIN_ENA_REFCLK
            self.reffreq_hz = self.INT_CLK_FREQ_HZ

    @property
    def _ref_clock_divisions(self):
        return ((self.conf_regs[5] & 0x0F) << 16) + (self.conf_regs[4] << 8) + self.conf_regs[3]

    @_ref_clock_divisions.setter
    def _ref_clock_divisions(self, value):
        if (0 > value) or (value > (2**20)):
            raise ValueError("Ref clock division must be max 20bits")
        self.conf_regs[3] = value & 0xFF
        self.conf_regs[4] = (value >> 8) & 0xFF
        self.conf_regs[5] = (value >> 16) & 0x0F
        
    @property
    def reffreq_hz(self):
        return int(1.0e12/(self._ref_clock_divisions*self._lsb))

    @reffreq_hz.setter
    def reffreq_hz(self, value):
        div = int(1e12/(value*self._lsb))
        if div >= (2**20):
            raise ValueError("Invalid combination of ref clock frequency and LSB. (ref_clock_division overflow)")
        self._ref_clock_divisions = int(1.0e12/(value*self._lsb))
        self.refclk_period_ns = (1.0e9/value)

    @property
    def lsb(self):
        return self._lsb

    @lsb.setter
    def lsb(self, value):
        freq_hz = self.reffreq_hz
        self._lsb = value
        self.reffreq_hz = freq_hz


    def power_on_reset(self):
        self._comstart()
        self.spi.xfer(bytearray([self.spiopc_power]))

    def meas_init(self):
        self._comstart()
        self.spi.xfer(bytearray([self.spiopc_init]))

    def reset_clkidx(self, pulse_len_s=0.001):
        gpio.output(self.rir, gpio.HIGH)
        if pulse_len_s >= 0:
            time.sleep(pulse_len_s)
        gpio.output(self.rir, gpio.LOW)

    def write_config_reg(self, address, data:list):
        self._comstart()
        adr = self.spiopc_write_config | (address & 0x1F)
        if (DEBUG):
            print(f"writing config to address {address}: {data}")
        self.spi.xfer(bytearray([adr] + data))

    def read_config_reg(self, address, nbytes):
        self._comstart()
        adr = self.spiopc_read_config | (address & 0x1F)
        if (DEBUG):
            print(f"reading config address {address}")
        rb = self.spi.xfer(bytearray([adr] + ([0x00]*nbytes)))
        return rb[1:]

    def read_results_reg(self, address, nbytes):
        self._comstart()
        adr = self.spiopc_read_results | (address & 0x1F)
        if (DEBUG):
            print(f"reading results address {address}")
        rb = self.spi.xfer(bytearray([adr] + ([0x00]*nbytes)))
        return rb[1:]

    def measure(self, nchannels=1):
        while (gpio.input(self.int) ==  1):
            pass
        self._comstart()
        res = self.read_results_reg(8, nchannels*6)
        ret = {}
        for i in range(nchannels):
            ref_idx = (res[i*6+0] << 16) + (res[i*6+1] << 8) + res[i*6+2]
            stop = (res[i*6+3] << 16) + (res[i*6+4] << 8) + res[i*6+5]
            ret[i+1] = (ref_idx, stop)
        if (DEBUG):
            print(f"measurements: {ret}")
        return ret

    def _tmark(self, refid, tstop):
        return (refid + tstop/self._ref_clock_divisions) * self.refclk_period_ns  

    def _tdiff(self, refid1, tstop1, refid2, tstop2):
        """ returns (1,)-(2,) """
        if (refid1 - refid2) < -2**12:
            refid1 += 2**24
        return (refid1-refid2 + (tstop1-tstop2)/self._ref_clock_divisions) * self.refclk_period_ns

    def tm(self, measure, channel):
        return self._tmark(measure[channel][0], measure[channel][1])
    
    def ti(self, measure, ch1, ch2):
        return self._tdiff(measure[ch2][0], measure[ch2][1], measure[ch1][0], measure[ch1][1])

    def tr(self, measure, channel):
        return measure[channel][1]/self._ref_clock_divisions * self.refclk_period_ns
        
    def add_24bit_signed(a, b):
        result = (a + b) & 0xFFFFFF  # keep only lowest 24 bits
        if result & 0x800000:        # check if MSB (bit 23) is 1 → negative
            return result - 0x1000000  # apply two's complement
        return result
    
    def close(self):
        self.spi.close()

    def print_results(self, measurements):
        hesx = [self.hit_ena_stop1, self.hit_ena_stop2, self.hit_ena_stop3, self.hit_ena_stop4]
        for i in range(1, 5):
            if hesx[i-1]:
                print(f"{self.t(measurements, i):.3f} Ch{i}", end=self.print_channels_separator)
        if self.print_channels_separator != '\n':
            print('\n')
