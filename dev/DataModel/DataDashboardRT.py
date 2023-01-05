
from DataModel import DataModel
from time import sleep 
GunnerusData = DataModel()

try:
    GunnerusData.start()
    # wait around, catch keyboard interrupt  
    while True: 
        sleep(0.1)

except KeyboardInterrupt:
    # terminate main thread 
    GunnerusData.stop()
    print('Exiting...')