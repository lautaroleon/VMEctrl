from DACVME_ctrl import VMECTRL
import time

DAC0 = VMECTRL("192.168.0.108", 8880)
i=0.0
iprev = -1
#DAC0.setChVol(0,0,-4)
while True:
    
    if iprev != i:
        if DAC0.setChVol(0,0,i) == 0:
            print("ok")
            iprev = i
            i = i+0.1
        if i>1:
            i=0
        time.sleep(0.001)