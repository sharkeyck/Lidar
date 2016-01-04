#Display Data from Neato LIDAR
#based on code from Nicolas "Xevel" Saugnier
#requires vpython and pyserial


import thread, time, sys, traceback, math

COM_PORT = "COM4" # example: 5 == "COM6" == "/dev/tty5"
BAUD_RATE = 115200
FPS = 60
OFFSET = 140
init_level = 0
index = 0

from visual import *
point = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0 , 1, 0)) #green, good point
lidar_representation = ring(pos=(0,0,0), axis=(0,1,0), radius=OFFSET-1, thickness=1, color = color.yellow)
lidar_representation.visible = True

label_speed = label(pos = (0,-500,0), xoffset=1, box=False, opacity=0.1)
label_errors = label(pos = (0,-1000,0), xoffset=1, text="errors: 0", visible = False, box=False)


def reset_display( angle ):
    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)
    
    #reset the point display    
    point.pos[angle] = vector( 0, 0, 0 )         
         
def parse_point_data( angle, data ):
    dist_mm = data[0] | (( data[1] & 0x3f) << 8) # distance is coded on 14 bits
    quality = data[2] | (data[3] << 8) # quality is on 16 bits
    is_bad_data = data[1] & 0x80
    is_low_quality = data[1] & 0x40
    parsed_data = (dist_mm, is_bad_data, is_low_quality)
    return parsed_data
    
def update_point( angle, data ):
    """Updates the view of a sample.

Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
"""
    
    (dist_mm, is_bad_data, is_low_quality) = parse_point_data(angle, data)
    
    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)
    
    dist_x = dist_mm*c
    dist_y = dist_mm*s
           
    reset_display(angle)
    # display the sample
    if is_bad_data:
        # FIGURE OUT EQUAL WITH POINTS TODO
        return
    else:
        point.pos[angle] = vector( dist_x,0, dist_y)
        if is_low_quality:
            point.color[angle] =(0.4, 0, 0) #red for low quality
        else:
            point.color[angle] =(0, 1, 0) #green for good quality


def check_sum(data):
    """Compute and return the check_sum as an int.

data -- list of 20 bytes (as ints), in the order they arrived in.
"""
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        #TODO make less confusing
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the check_sum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    check_sum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    check_sum = check_sum & 0x7FFF # truncate to 15 bits
    return int( check_sum )

def read_lidar():
    global init_level, index
    #TODO variable for each index in full_data?
    #data string: <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
    #INDEX_BYTE = 1
    #SPEED_BYTES = [2,3]
    #DATA_BYTES = [[4,5,6,7],......], ect
    
    SPEED_PACKETS = 2
    DATA_PACKETS = 4
    DATA_BYTES = 4
    PACKET_BYTES = 22
    check_sum_errors = 0
    
    while True:
        try:
            #time.sleep(0.00001) # do not hog the processor power
            b = ord(ser.read(1))
            # start byte
            if b != 0xFA :
                continue
                
            # position index
            b = ord(ser.read(1))
            if b >= 0xA0 and b <= 0xF9 :
                index = b - 0xA0
            else:
                continue
                
            # speed
            b_speed = [ ord(b) for b in ser.read(2)]
            
            # data
            b_data = [[ord(b) for b in ser.read(DATA_BYTES)] for inc in range(DATA_PACKETS)]

            # for the checksum, we need all the data of the packet...
            # this could be collected in a more elegent fashion...
            all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data[0] + b_data[1] + b_data[2] + b_data[3]

            # checksum
            b_check_sum = [ ord(b) for b in ser.read(2) ]
            incoming_check_sum = int(b_check_sum[0]) + (int(b_check_sum[1]) << 8)

            # verify that the received checksum is equal to the one computed from the data
            if check_sum(all_data) == incoming_check_sum:
                speed_rpm = float( b_speed[0] | (b_speed[1] << 8) ) / 64.0
                label_speed.text = "RPM : " + str(speed_rpm) 
            else:
                # the checksum does not match, something went wrong...
                check_sum_errors +=1
                label_errors.text = "errors: "+str(check_sum_errors)
                # give the samples an error state
                b_data = [[0, 0x80, 0, 0] for inc in range(DATA_PACKETS)]
                
            for inc in range(DATA_PACKETS):
                update_point(index * 4 + inc, b_data[inc])    

        except :
            # TODO remove try/except and handle exceptions locally
            traceback.print_exc(file=sys.stdout)

                      
            """if init_level == 0 : #TODO rename to be more packet-y
                b = ord(ser.read(1)) #TODO look for a read bytes function (not chars)
                # start byte
                if b == 0xFA :
                    #found data packet, compare data check_sum to given check_sum
                    full_data = [b] + [ord(b) for b in ser.read(PACKET_BYTES-1)]
                    incoming_check_sum = (int(full_data.pop()) <<8 ) + int(full_data.pop())
                    if check_sum(full_data) == incoming_check_sum:
                        #TODO if statements needed? how to except if bad?
                        if b >= 0xA0 and b <= 0xF9 :
                            index = full_data[1] - 0xA0
                        elif b != 0xFA:
                            init_level = 0
                        #TODO this is dumb and ugly.
                        b_speed = [ full_data[2], full_data[3]]
                        b_data = [full_data[4:8],full_data[8:12],full_data[12:16],full_data[16:20]]
                        speed_rpm = float( b_speed[0] | (b_speed[1] << 8) ) / 64.0
                        label_speed.text = "RPM : " + str(speed_rpm)
                        
                    else:
                        # the check_sum does not match, something went wrong...
                        check_sum_errs +=1
                        label_errors.text = "errors: "+str(check_sum_errs)
                        #display the samples in an error state
                        b_data = [[0, 0x80, 0, 0] for inc in range(DATA_PACKETS)]
                        
                    for inc in range(DATA_PACKETS) :
                        update_point(index * 4 + inc, b_data[inc])
                    init_level = 0
                else:
                    init_level = (b == 0xFA)"""  
            
def check_keys():
    if scene.kb.keys: # event waiting to be processed?
        s = scene.kb.getkey() # get keyboard info

        if s=="j": # Toggle rpm
            label_speed.visible = not label_speed.visible
        elif s=="k": # Toggle errors
            label_errors.visible = not label_errors.visible


import serial
ser = serial.Serial(COM_PORT, BAUD_RATE)
th = thread.start_new_thread(read_lidar, ())

while True:
    rate(FPS) # synchonous repaint at 60fps
    check_keys()
    
