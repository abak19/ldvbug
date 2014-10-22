import serial
import time
import glob
import math

PI = 3.1415926535897932384626433832795
DEBUG = 0

def debug(*args):
    if DEBUG:
        print args
        pass

threshold1 = 100
threshold2 = 235



class Robot():
    def __init__(self,sim=0):
        self.sim = sim
        if sim == 0:
            ttydevs = glob.glob('/dev/ttyACM*')
            if len(ttydevs) == 0:
                debug ("error: no tty Port")
                self.s = Simulator()
                #exit()
            else:
                self.s = serial.Serial(ttydevs[0], 38400, 8, 'N', 1, 0.001)
        else:
            self.s = Simulator()
        self.groundSensor = [0,0,0,0] 
        self.wallSensor = 0
        self.contactSwitch = [0,0,0,0,0] #KONTAKT_LINKS,KONTAKT_VORNE,KONTAKT_RECHTS,BUMPER_LINKS,BUMPER_RECHTS
        self.batteryVoltage = 0
        self.status = 0
        self.isMoving = 0
        self.state = 0
    def command(self,ctype,*args):
        startbyte = chr(0xEA)
        if ctype == 1:
            speed = 0
            distance = 0
            phi = 0
            for count, arg in enumerate(args):
                if count == 0:
                    speed = arg
                elif count == 1:
                    distance = arg
                elif count == 2:
                    phi = arg
            if speed<0:
                speed=0
                distance = -distance
            if speed>255:
                speed=255
            if speed<45:
                speed=45
            speed=chr(speed)
            if distance < -32767:
                distance = -32767
            if distance > 32767:
                distance = 32767
            if distance < 0:
                distance += 65536
            distanceH = distance / 256
            distanceH = chr(distanceH)
            distanceL = distance % 256
            distanceL = chr(distanceL)
            
            if phi < -32767:
                phi = -32767
            if phi > 32767:
                phi = 32767
            if phi < 0:
                phi += 65536
            phiH = phi / 256
            phiH = chr(phiH)
            phiL = phi % 256
            phiL = chr(phiL)
            self.s.write(startbyte+"\x08\x01"+speed+distanceH+distanceL+phiH+phiL)
            self.isMoving = 1
            self.getStatus()
        elif  ctype == 2:
            pass
        elif  ctype == 3:
            self.s.write("\xEA\x04\x03\x00")
            pass
        elif  ctype == 4:
            self.s.write("\xEA\x04\x04\x00")
            pass
        elif  ctype == 5:
            self.s.write("\xEA\x04\x05\x00")
            pass
        elif  ctype == 200:
            interval = 0
            for count, arg in enumerate(args):
                if count == 0:
                    interval = arg
            intervalH = chr(interval / 256)
            intervalL = chr(interval % 256)
            self.s.write("\xEA\x05\xC8"+intervalH+intervalL)
            pass
        elif  ctype == 203:
            t=0
            for count, arg in enumerate(args):
                if count == 0:
                    t = arg
            t = chr(t)
            self.s.write("\xEA\x04\xCB"+t)
            pass
            
    def drive(self,speed,d,p):
        self.command(1,speed,d,p)
    def drive80mm(self):
        self.command(1,80,80,0)
    def turn20deg(self):
        self.command(1,80,0,20)
    def turnm20deg(self):
        self.command(1,80,0,-20)
    def reqSensor(self):
        self.command(3)
    def getStatus(self):
        self.command(4)
    def stop(self):
        self.command(5)
    def setAutoUpdate(self,interval):
        self.command(200,interval)
    def setMovementOverwrite(self,s):
        self.command(203,s)
    
    def read(self):
        global threshold1, threshold2
        while (self.s.inWaiting() > 2):
            res = ord(self.s.read())
            #print "res = "+str(res)
            if res == 234:
                ##
                l = ord(self.s.read())
                #print "l = "+str(l)
                if (self.s.inWaiting() > l-3 and l>2):
                    t = ord(self.s.read())
                    #print "t = "+str(t)
                    if t==101:
                        self.groundSensor[0] = ord(self.s.read())
                        self.groundSensor[1] = ord(self.s.read())
                        self.groundSensor[2] = ord(self.s.read())
                        self.groundSensor[3] = ord(self.s.read())
                        self.state = 0
                        #if self.groundSensor[0]> threshold1:
                        if self.groundSensor[0] < threshold2:
                            self.state += 1
                        if self.groundSensor[1] < threshold2:
                            self.state += 2
                        if self.groundSensor[2] < threshold2:
                            self.state += 4
                        if self.groundSensor[3] < threshold2:
                            self.state += 8
                        
                    if t==100:
                        self.status = ord(self.s.read())
                        self.isMoving = ord(self.s.read())

                    if t==102:#distancesensors 102
                        self.groundSensor[0] = ord(self.s.read())
                        self.groundSensor[1] = ord(self.s.read())
                        self.groundSensor[2] = ord(self.s.read())
                        self.groundSensor[3] = ord(self.s.read())
                        self.wallSensor = ord(self.s.read())
                        self.state = 0
                        if self.groundSensor[0] < threshold2:
                            self.state += 1
                        if self.groundSensor[1] < threshold2:
                            self.state += 2
                        if self.groundSensor[2] < threshold2:
                            self.state += 4
                        if self.groundSensor[3] < threshold2:
                            self.state += 8

                    if t==104:#contactswitches 104
                        t = ord(self.s.read())
                        if (t>15):
                            contactSwitch[4]=1;
                            t=t-16
                        else:
                            contactSwitch[4]=0;
                        if (t>7):
                            contactSwitch[3]=1;
                            t=t-8
                        else:
                            contactSwitch[3]=0;
                        if (t>3):
                            contactSwitch[2]=1;
                            t=t-4
                        else:
                            contactSwitch[2]=0;
                        if (t>1):
                            contactSwitch[1]=1;
                            t=t-2
                        else:
                            contactSwitch[1]=0;
                        if (t>0):
                            contactSwitch[0]=1;
                        else:
                            contactSwitch[0]=0;
                        
                        
                    if t==105:
                        self.batteryVoltage = ord(self.s.read()) / 10.0
        
    def updateState(self):
        self.reqSensor()
        pass
    def getState(self):
        self.updateState()
        return self.state



"""For Testing purpose"""
class Simulator():
    def __init__(self):
        self.groundSensor = [0,0,0,0] 
        self.status = 0
        self.isMoving = 0
        self.state = 0
        self.x = 0
        self.y = 0
        self.phi = 0
        self.slist = ["\xEA\x04\x04\x00"]
        
    def write(self,string):
        #print string, " of len ", len(string), " starting with ", ord(string[0])
        if ord(string[0]) == 234:
            if ord(string[2]) == 0:
                pass
            elif ord(string[2]) == 1:
                s = ord(string[3])
                dH = ord(string[4])
                dL = ord(string[5])
                d = dH * 256 + dL
                pH = ord(string[6])
                pL = ord(string[7])
                p = pH * 256 + pL
                
                if p  > 32767:
                    p -= 65536
                if d  > 32767:
                    d -= 65536
                
                
                
                
                print "move: ",s,"\t",d,"\t",p
                self.phi += p
                if self.phi > 360:
                    self.phi -= 360
                if self.phi < -360:
                    self.phi += 360
                self.x += d*math.sin((self.phi)/180.0*PI)
                self.y += d*math.cos((self.phi)/180.0*PI)
                print "moved to: ",self.x,"\t",self.y,"\t",self.phi
                pass
            elif ord(string[2]) == 2:
                pass
            elif ord(string[2]) == 3:
                self.sensors()
                self.slist.append("\xEA\x07\x65"+chr(self.groundSensor[0])+chr(self.groundSensor[1])+chr(self.groundSensor[2])+chr(self.groundSensor[3]))
                pass
            elif ord(string[2]) == 4:
                self.slist.append("\xEA\x05\x64\x00\x00")
                pass
        
        pass
    def inWaiting(self):
        if len(self.slist):
            return 10
        return 0
        pass
    def read(self):
        if len(self.slist):
            if len(self.slist[0]) == 0:
                pass
            a,b = self.slist[0][0:1], self.slist[0][1:]
            if len(b):
                self.slist[0] = b
            else:
                self.slist.pop(0)
            return a
        return 0
        pass
    def sensors(self):
        global PI
        s1x = self.x + math.cos((self.phi-50)/180.0*PI)*150
        s1y = self.y + math.sin((self.phi-50)/180.0*PI)*150
        s2x = self.x + math.cos((self.phi-15)/180.0*PI)*150
        s2y = self.y + math.sin((self.phi-15)/180.0*PI)*150
        s3x = self.x + math.cos((self.phi+15)/180.0*PI)*150
        s3y = self.y + math.sin((self.phi+15)/180.0*PI)*150
        s4x = self.x + math.cos((self.phi+50)/180.0*PI)*150
        s4y = self.y + math.sin((self.phi+50)/180.0*PI)*150
        
        print "GSP:\t",s1x," ",s1y,"\t",s2x," ",s2y,"\t",s3x," ",s3y,"\t",s4x," ",s4y
        
        self.groundSensor[0] = self.getColor(s1x,s1y)
        self.groundSensor[1] = self.getColor(s2x,s2y)
        self.groundSensor[2] = self.getColor(s3x,s3y)
        self.groundSensor[3] = self.getColor(s4x,s4y)
        pass
    def getColor(self,x,y):
        if max(abs(x),abs(y)) > 1500:
            return 250
        return 0
