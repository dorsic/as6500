
# AD6500 sample program for RPi

import RPi.GPIO as gpio
import spidev
import time
from datetime import datetime, timedelta
from as6500 import AS6500


GPIO_CS = 12
GPIO_INT = 25     # low on the pin indicates to the controller that new data is available
GPIO_RIR = 19	  # clock index reset
DEBUG = None

def main():
    tdc = AS6500(GPIO_CS, GPIO_INT, GPIO_RIR)
    tdc.refclk = tdc.REF_CLK_EXT
    tdc.reffreq_hz = 10_000_000
    tdc.lsb = tdc.LSB_1
    tdc.pin_ena_rstidx = 1
    tdc.pin_ena_stop1 = 0
    tdc.hit_ena_stop1 = 0
    tdc.pin_ena_stop2 = 1
    tdc.hit_ena_stop2 = 1
    tdc.pin_ena_stop3 = 1
    tdc.hit_ena_stop3 = 1
#    tdc.hit_ena_stop4 = 1
    tdc.channel_combine = tdc.CHANNEL_COMBINE_NO
    tdc.resolution = tdc.RESOLUTION_STANDARD
    try:
        tdc.power_on_reset()
        tdc.write_config_reg(0, tdc.conf_regs)
        if (DEBUG):
            print(f"Ref clock period: {tdc.refclk_period_ns} ns")
            print(f"Ref clock freq: {tdc.reffreq_hz} Hz")
            print(f"LSB: {tdc.lsb} ps")
            print(f"Ref clock divisions: {tdc._ref_clock_divisions}")

        tdc.meas_init()
        tdc.reset_clkidx()
        cnt = 0
        st = datetime.now()
        while True:
            m = tdc.measure(3)
            #tdc.reset_clkidx(0)
#            print(f"{tdc.tm(m, 1):.3f} Ch1")
#            print(f"{tdc.tm(m, 3):.3f} Ch3")
#            print(f"{tdc.td(m, 1, 3):.3f}")
#            print(f"{tdc.td(m, 3, 1):.3f}")
#            print(f"{tdc.tm(m, 1):.3f} Ch1\t{tdc.tm(m, 3):.3f} Ch3\t{tdc.td(m, 1, 3):.3f} Ch1-3\t {tdc.td(m, 3, 1):.3f} Ch3-1")
            print(f"{m[2][0]}\t{m[2][1]}\t{m[3][0]}\t{m[3][1]}\t{tdc.ti(m, 3, 2):.3f}")
            #print(f"{tdc.tr(m, 1):.3f}")
            cnt +=1
            if cnt == 1000000:
                raise KeyboardInterrupt("CNT done")
    except KeyboardInterrupt:
        print(f"done in {(datetime.now()-st).total_seconds()}")
        pass        
    except Exception as e:
        print(f"{e}")
    finally:
        tdc.close()
        print("DONE")

if __name__ == '__main__':
    main()
