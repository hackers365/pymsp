#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import serial
import struct
import binascii
import random
import commands
import time
import threading
import web
import types



CMD2CODE = {
    'MSP_IDENT'           :     100,
    'MSP_STATUS'          :     101,
    'MSP_RAW_IMU'         :     102,
    'MSP_SERVO'           :     103,
    'MSP_MOTOR'           :     104,
    'MSP_RC'              :     105,
    'MSP_RAW_GPS'         :     106,
    'MSP_COMP_GPS'        :     107,
    'MSP_ATTITUDE'        :     108,
    'MSP_ALTITUDE'        :     109,
    'MSP_ANALOG'          :     110,
    'MSP_RC_TUNING'       :     111,
    'MSP_PID'             :     112,
    'MSP_BOX'             :     113,
    'MSP_MISC'            :     114,
    'MSP_MOTOR_PINS'      :     115,
    'MSP_BOXNAMES'        :     116,
    'MSP_PIDNAMES'        :     117,
    'MSP_WP'              :     118,
    'MSP_BOXIDS'          :     119,

    'MSP_SET_RAW_RC'      :     200,
    'MSP_SET_RAW_GPS'     :     201,
    'MSP_SET_PID'         :     202,
    'MSP_SET_BOX'         :     203,
    'MSP_SET_RC_TUNING'   :     204,
    'MSP_ACC_CALIBRATION' :     205,
    'MSP_MAG_CALIBRATION' :     206,
    'MSP_SET_MISC'        :     207,
    'MSP_RESET_CONF'      :     208,
    'MSP_SET_WP'          :     209,
   'MSP_SWITCH_RC_SERIAL' :     210,
   'MSP_IS_SERIAL'        :     211,
    'MSP_DEBUG'           :     254,
}


#serial_port = '/dev/ttyUSB0'
band_rate = 115200
ihead_flag = '$M>'
ser = None
data_length = 0
multi_info = {
    'rcdata': [1400, 300, 12998, 2499, 2344,1234,1234,1234],
    'rcdataname': ['roll', 'pitch', 'yaw', 'thr', 'aux1', 'aux2', 'aux3', 'aux4'],
    'debug': None,
    'control_type':None,
    'is_valid_serial': True,
}

CODE2INFO = {
    100: {'type': '', 'data': None},
    105: {'type': '8h', 'data': multi_info['rcdata']},
    211: {'type': 'B', 'data': multi_info['control_type']},
    254: {'type': '4h', 'data': multi_info['debug']},


    210: {'type': '', 'data': None},
}

def sendData(data_length, code, data):
    checksum = 0
    total_data = ['$', 'M', '<', data_length, code] + data
    for i in struct.pack('<2B%dh' % len(data), *total_data[3:len(total_data)]):
        checksum = checksum ^ ord(i)

    total_data.append(checksum)

    #print ser
    try:
        b = None
        b = ser.write(struct.pack('<3c2B%dhB' % len(data), *total_data))
        ser.flush()
    except Exception, ex:
        print 'send data is_valid_serial fail'
        multi_info['is_valid_serial'] = False;
        connect()
    return b

class index:
    def GET(self, name):
        global multi_info
        if name == 'set':
            global ser
            userdata = web.input(roll=1500,pitch=1500,yaw=1500,thr=1500,aux1=1500,aux2=1500,aux3=1500,aux4=1500)
            rcdata = []
            rcdata.append(True and int(userdata.roll) or 1500)
            rcdata.append(True and int(userdata.pitch) or 1500)
            rcdata.append(True and int(userdata.yaw) or 1500)
            rcdata.append(True and int(userdata.thr) or 1500)
            rcdata.append(True and int(userdata.aux1) or 1500)
            rcdata.append(True and int(userdata.aux2) or 1500)
            rcdata.append(True and int(userdata.aux3) or 1500)
            rcdata.append(True and int(userdata.aux4) or 1500)

            sendData(16, CMD2CODE['MSP_SET_RAW_RC'], rcdata)
            sendData(0, CMD2CODE['MSP_RC'], [])
            return 'set ok'
        elif name == 'switch':
            userdata = web.input(i=1)
            sendData(2, CMD2CODE['MSP_SWITCH_RC_SERIAL'], [int(userdata.i)])
            return 'switch ok'
        elif name == 'get':
            sendData(0, CMD2CODE['MSP_RC'], [])
        elif name == 'is_serial':
            sendData(0, CMD2CODE['MSP_IS_SERIAL'], [])
        elif name == 'debug':
            sendData(0, CMD2CODE['MSP_DEBUG'], [])
        elif name == 'info':
            render = web.template.render('')
            return render.multi_info(multi_info)
        else:
            render = web.template.render('')
            return render.hello()
        return 'not set'

class WebApp(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def run(self):
        self.create_server()
    def create_server(self):
        urls = (
          '/(.*)', 'index'
        )

        app = web.application(urls, globals())
        app.run()

class receiveData(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        self.receive_proc()

    def unpack_data(self, data):
        code = ord(data[1:2])
        dl = ord(data[0:1])

        if CODE2INFO[code]['data'] != None:
            if type(CODE2INFO[code]['data']) == types.ListType:
                del CODE2INFO[code]['data'][:]
                CODE2INFO[code]['data'].extend(struct.unpack('<' + CODE2INFO[code]['type'], data[2:2+dl]))
        return

    def receive_proc(self):
        global ser
        rbuffer = ''
        need_read = True
        #search head flag
        pos = 0
        i = 0
        while 1:
            data_length = 0
            try:
                ser.flush()
                data_length = ser.inWaiting()
            except Exception, ex:
                print 'inWaiting is_valid_serial fail'
                connect()

            if data_length > 0:
                rbuffer += ser.read(data_length)
                need_read = 0
            else:
                time.sleep(0.1)
                continue

            while not need_read:
                try:
                    pos = rbuffer.find(ihead_flag)
                except Exception, ex:
                    need_read = True
                    break

                ##print 'pos:' + str(pos)
                if pos >=0:
                    ##print len(rbuffer)
                    try:
                        dl = ord(rbuffer[pos+len(ihead_flag):pos+len(ihead_flag)+1])
                    except Exception, ex:
                        need_read = True
                        break
                    data = rbuffer[pos+len(ihead_flag):pos+len(ihead_flag) + dl + 3]
                    ##print 'data_len:' + str(len(data))
                    if (dl + 3) == len(data) and len(data) > 3:
                        checksum = 0
                        orig_checksum = data[-1:]
                        #sign checksum
                        for i in data[:-1]:
                            checksum = checksum ^ ord(i)
                        ##print "checksum:" + str(checksum)
                        ##print "orig_checksum:" + str(ord(orig_checksum))
                        if ord(orig_checksum) == checksum:
                            self.unpack_data(data)
                    #not complete data
                    elif (dl + 3) > len(data):
                        need_read = True
                        break
                    rbuffer = rbuffer[pos + len(ihead_flag) + len(data):]
                else:
                    rbuffer = ''
                    need_read = True
                    break

def connect():
    try:
        global ser
        while True:
            serial_port = commands.getoutput('ls /dev/ttyUSB*')
            print serial_port
            if serial_port:
                ser = serial.Serial(serial_port, band_rate)
                print serial_port
                if ser:
                    break
            time.sleep(1)
    except Exception, ex:
        print 'open serial port fail\n'
        sys.exit()
    multi_info['is_valid_serial'] = True;
if __name__ == "__main__":
    connect()
    #start read thread
    rd = receiveData()
    rd.start()
    wa = WebApp()
    wa.start()
    while 1:
        #~ rc_data = [1401, 1300, 1598, 1389, 1289, 1487, 1278, 1698]  #roll,pitch,yaw,thr,aux1,aux2,aux3,aux4
        #~ i = 7
        #~ while i >= 0:
            #~ rc_data[i] = random.randint(1100, 1900)
            #~ i = i -1
        #~ sendData(16, CMD2CODE['MSP_SET_RAW_RC'], rc_data)
        sendData(0, CMD2CODE['MSP_RC'], [])
        time.sleep(0.1)


    #print sendData(16, CMD2CODE['MSP_SET_RAW_RC'], rc_data)
    #print ser.write(struct.pack('<3c3B', '$', 'M', '<', 0, 105, 105))

    sendData(0, CMD2CODE['MSP_RC'], [])

    rd.join()
    wa.join()
