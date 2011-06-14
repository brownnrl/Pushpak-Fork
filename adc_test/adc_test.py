#Sending commands from PC and checking status
import struct
import time
import math
import serial
    

# x     pad byte        no value    
# c     char            string of length 1  
# b     signed char     integer     
# B     unsigned char   integer     
# h     short           integer     
# H     unsigned short  integer     
# i     int             integer     
# I     unsigned int        long    
# l     long                integer     
# L     unsigned long       long    
# q     long long           long    (1)
# Q     unsigned long long  long    (1)
# f     float   float   
# d     double  float   
# s     char[]  string  
# p     char[]  string  
# P     void *  integer

# struct msg_type1{
#   uint16_t    header;
#   uint8_t     id;         
#   uint8_t     length;
#   uint32_t    sys_time;
#   int16_t acc_x;
#   int16_t acc_y;
#   int16_t acc_z;
#   int16_t gyro_x;
#   int16_t gyro_y;
#   int16_t gyro_z;
# };


data_format = '=HBBIhhhhhh'
msg_size = struct.calcsize(data_format)
#############################################################################################

def get_next_msg():
    header_found = False;
    
    while(header_found == False):
        x = ser.read(1)
        print("read byte", x)
        if len(x) == 0 :
            print("Serial port time out")
            return 0
        #data comes in little endian format so check for CD first
        if ord(x) == 0xCD :
            x = ser.read(1)
            print("read byte", x)
            if len(x) == 0 :
                print("Serial port time out")
                return 0
            if ord(x) == 0xAB :
                header_found = True;
                break;
            
    #note this will lockup, if no timeout defined
    x = chr(0xAB) + chr(0xCD) + ser.read(msg_size - 2)
#    print("printing the complete stream")
#    for i in x :
#        print(ord(i))
    
    if len(x) == msg_size :
        y = struct.unpack(data_format,x)
        print y 
    else :
        print "Serial port timeout,bytes recieved = ", len(x)

##############################################################################################
#Main program starts from here      
                            
print ("Pushpak Quadrotor ADC test Program")
print ("Using com port : COM4")

#####################################
#open serial port

ser = serial.Serial(2,115200) #to open COM4, use value 3.
result = ser.isOpen()
ser.setTimeout(1)
print "Serial port opened:", result
print "Size of message in bytes = ", msg_size


while(True):      
    get_next_msg()
        
ser.close()
print "closing"




