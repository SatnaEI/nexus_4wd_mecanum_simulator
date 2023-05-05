from huskylib import HuskyLensLibrary
import time

# replace the address value with your I2C address from before in 0x00 form
huskyLens= HuskyLensLibrary("I2C","",address=0x32)
#print(hl.knock())

#huskyLens.algorthim("ALGORITHM_TAG_RECOGNITION")
while(True):
    try:
        data=huskyLens.blocks()
        print(data.ID)
        #print(len(data))
    except Exception as error:
        print(error)
        #print("Nothing detected")
    time.sleep(0.5)
    '''x=0
    for i in data:
        x=x+1
        print("Face {} data: {}".format(x,i)'''
