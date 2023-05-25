
from DataModel import DataModel
from time import sleep 
import argparse 

parser = argparse.ArgumentParser(description='File opening script')
parser.add_argument('-f', '--file', help='Path to the log file to open')
args = parser.parse_args()

if args.file:
        GunnerusData = DataModel(args.file)
else:
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