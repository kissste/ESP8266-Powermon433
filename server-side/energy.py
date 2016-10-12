import socket
import datetime
import pytz
import rrdtool
import base64
import binascii
import re
import MySQLdb

if(0):
    rrdtool.create('energy.rrd', '--start', 'now', '--step', '30'
    , 'RRA:AVERAGE:0.5:1:20160' #30s for 1w
    , 'RRA:AVERAGE:0.5:2:20160' #1m for 2w
    , 'RRA:AVERAGE:0.5:10:8928' #5m for 1m
    , 'RRA:AVERAGE:0.5:20:26640' #10m for 6m
    , 'RRA:AVERAGE:0.5:120:43800' #1h for 5y
    , 'DS:energy:GAUGE:60:0:20000'
    , 'DS:temp:GAUGE:60:-73:100')
if(0):
    rrdtool.create('temp1.rrd', '--start', 'now', '--step', '60'
    , 'RRA:AVERAGE:0.5:1:20160' #30s for 1w
    , 'RRA:AVERAGE:0.5:2:20160' #1m for 2w
    , 'RRA:AVERAGE:0.5:10:8928' #5m for 1m
    , 'RRA:AVERAGE:0.5:20:26640' #10m for 6m
    , 'RRA:AVERAGE:0.5:120:43800' #1h for 5y
    , 'DS:temp:GAUGE:120:-73:100'
    , 'DS:hum:GAUGE:120:0:100'
    )
if(0):
    rrdtool.create('boil.rrd', '--start', 'now', '--step', '60'
    , 'RRA:AVERAGE:0.5:1:20160' #30s for 1w
    , 'RRA:AVERAGE:0.5:2:20160' #1m for 2w
    , 'RRA:AVERAGE:0.5:10:8928' #5m for 1m
    , 'RRA:AVERAGE:0.5:20:26640' #10m for 6m
    , 'RRA:AVERAGE:0.5:120:43800' #1h for 5y
    , 'DS:temp:GAUGE:120:-73:100'
    , 'DS:hum:GAUGE:120:0:100'
    , 'DS:t0:GAUGE:120:0:100'
    , 'DS:t1:GAUGE:120:0:100'
    , 'DS:t2:GAUGE:120:0:100'
    , 'DS:t3:GAUGE:120:0:100'
    , 'DS:t4:GAUGE:120:0:100'
    )

db = MySQLdb.connect(user="youruser", passwd="yourpassword", db="yourdatabase")
c = db.cursor()

UDP_IP = "yourserver.com"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Internet, UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    u = datetime.datetime.utcnow()
    u = u.replace(tzinfo=pytz.utc)
    msg = data.decode("utf-8").strip()
    mylist = msg.split(',')
    print(u.astimezone(pytz.timezone("America/New_York")), ':', msg)
    if len(mylist)<3:
        continue
    elif mylist[3] == 'Start':
        #print('Start,', msg)
        pass
    elif mylist[3] == 'Wifi':
        #print('Wifi,', msg) 
        data = ''
        try:
            data = base64.b64decode(mylist[4])
        except:
            pass
        #print(data)
        for i in range(0,len(data),16):
            mac1 = binascii.hexlify(data[i:i+6])
            snap = data[i+6]
            rssi = data[i+7]
            channel = data[i+8]
            mac2 = binascii.hexlify(data[i+9:i+15])
            print(mac1, snap, rssi, channel, mac2)
            x = c.execute("""INSERT INTO b_wifi (src, mac1, snap, rssi, channel, mac2) VALUES (%s,%s,%s,%s,%s,%s) ON DUPLICATE KEY UPDATE snap = VALUES(snap)"""
                ,(0, mac1, round(u.timestamp())-snap, rssi, channel, mac2))
    elif mylist[3] == 'Read':
        rrdtool.update('energy.rrd','N:'+mylist[7]+':'+mylist[8])
        #print('Energy,','N:'+mylist[7]+':'+mylist[8],round(u.timestamp()))
    elif mylist[3] == 'PIR':
        Temp = Hum = 'nan'
        for i in range(4,len(mylist)): 
            a = mylist[i].split('=')
            if a[0] == 'Temp': Temp = a[1]
            elif a[0] == 'Hum': Hum = a[1]
        rrdtool.update('temp1.rrd','N:'+Temp+':'+Hum)
        #print('Temp1,','N:'+Temp+':'+Hum,round(u.timestamp()))
    elif mylist[3] == 'Boil':
        Temp = Hum = T0 = T1 = T2 = T3 = T4 = 'nan'
        for i in range(4,len(mylist)): 
            a = mylist[i].split('=')
            if a[0] == 'Temp': Temp = a[1]
            elif a[0] == 'Hum': Hum = a[1]
            elif a[0] == 'fc43d1': T0 = a[1]
            elif a[0] == '0ef668': T1 = a[1]
            elif a[0] == '0ebf67': T2 = a[1]
            elif a[0] == 'ddaa68': T3 = a[1]
            elif a[0] == 'fbc6d0': T4 = a[1]
        rrdtool.update('boil.rrd','N:'+Temp+':'+Hum+':'+T0+':'+T1+':'+T2+':'+T3+':'+T4)
        #print('Boil,','N:'+Temp+':'+Hum+':'+T0+':'+T1+':'+T2+':'+T3+':'+T4,round(u.timestamp()))
    with open("energy.log","a") as myfile:
        print(u.astimezone(pytz.timezone("America/New_York")),end=',',file=myfile)
        print(round(u.timestamp()),end=',',file=myfile)
        myfile.write(msg)
        myfile.write("\r\n")
