import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math


CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        self.speedmotor_left=0.1
        self.speedmotor_rigth=0.1
        counter=0
        I=0
        self.sensor_centro = 0  
        #self.prev_loc = None
        #self.cur_loc = None
        self.previous=[]
        self.prev_loc =(0,0)  # Inicialize prev_loc com uma tupla
        self.cur_loc  =(0,0)# Inicialize cur_loc com uma tupl 
        self.prev_predicted_loc = (0, 0)   

        self.out_l=0
        self.out_r=0
        self.theta=0
        self.rotating=None

        self.compass_buffer = []

        """self.out_r = 0
        self.out_l = 0
        self.X = 0
        self.Y = 0
        self.theta = 0"""
        
        h = 0.050 # tempo de amostragem
        Td=1*h
        Ti=1/h
        pos_over_line = 0
        n_active_sensors = 0
        controller=0.0
        erro_ant=0.0
        erro_curr=0.0
        deltah = 0.05 
        integral=0
        #mapping_dic={}
        self.X=(0,0)
        self.Y=(0,0)

        while True:
            #k=0.01
            #self.line_dir()
        
            self.readSensors()
            if self.measures.start:
           # self.print_complete_ma
            #self.movement_model()
                if self.measures.endLed:
                        print(self.robName + " exiting")
                        quit()
                if state == 'stop' and self.measures.start:
                        state = stopped_state

                if state != 'stop' and self.measures.stop:
                        stopped_state = state
                        state = 'stop'

                if state == 'run':
                        if self.measures.visitingLed==True:
                            state='wait'
                        if self.measures.ground==0:
                            self.setVisitingLed(True)
                        self.wander()
                    
                elif state=='wait':
                        self.setReturningLed(True)
                        if self.measures.visitingLed==True:
                            self.setVisitingLed(False)
                        if self.measures.returningLed==True:
                            state='return'
                        self.driveMotors(0.0,0.0)
                elif state=='return':
                        if self.measures.visitingLed==True:
                            self.setVisitingLed(False)
                        if self.measures.returningLed==True:
                            self.setReturningLed(False)  
                line = self.measures.lineSensor  # Assume que line \u00e9 uma lista de 0s e 1s

                if self.rotating == None:

                    if line:
                    
                        n = 0
                        _sum = 0

                        for i in range(len(line)):
                            if line[i] == '1':
                                _sum += i + 1
                                n += 1

                                print('sum: ',_sum)
                                print('n: ', n)

                        if n != 0:
                            average = _sum/n
                        else:
                            average = 4.0

                        print('average: ', average)

                        self.sensor_centro = average
                    if line == ['0','0','0','0','0','0','0']:

                        sum_left = sum(1 for i in self.previous if i == 'left')
                        sum_right = sum(1 for i in self.previous if i == 'right')  

                        #print(self.previous)
                        #print('sum_left: ', sum_left)
                        #print('sum_right: ', sum_right)

                        if sum_left > sum_right:
                            self.rotating = 'left'
                        else:
                            self.rotating = 'right'
                        #self.rotating = self.previous
                    else:
                        #Initialize the PID controller
                        Kp = 0.1
                        Kd = 0.009
                        Ki = 0.0
                        #Kp = 1
                        #Kd = 0.1
                        #Ki = 0.01

                        if counter == 0:
                            # Coordenadas iniciais reais
                            initial_x_real = self.measures.x
                            initial_y_real = self.measures.y
                            counter = counter + 1

                        # Coordenadas medidas
                        coord_X = self.measures.x
                        coord_Y = self.measures.y

                        # Coordenadas ajustadas
                        adjusted_coord_X = coord_X - initial_x_real
                        adjusted_coord_Y = coord_Y - initial_y_real

                        # Arredonde as coordenadas ajustadas para duas casas decimais (ou qualquer quantidade desejada)
                        adjusted_coord_X_arredondado = round(adjusted_coord_X)
                        adjusted_coord_Y_arredondado = round(adjusted_coord_Y)

                        self.prev_loc = self.cur_loc
                        self.cur_loc = (adjusted_coord_X_arredondado, adjusted_coord_Y_arredondado)

                        dt = 0.1
                        #Calculate the derivative of error
                        # derivate = (erro - self.previous_erro) / self.dt
                        erro = self.sensor_centro - 4
                        #Calculate the integral of the error
                        integral = integral + erro * dt

                        #Calculate the control signal
                        controller = Kp * erro + Ki * integral

                        self.previous_erro = erro
                        self.updatedrivemotor(self.speedmotor_left+controller,self.speedmotor_rigth-controller)       
                        print("gps_X: ",self.cur_loc[0])
                        print("gps_y: ",self.cur_loc[1])
                        """print("Err: ", erro)
                        print("sensor_centro: ", self.sensor_centro)"""
                        
                    print(line[0:3])
                    sum_left = sum(1 for i in line[0:3] if i == '1')
                    sum_right = sum(1 for i in line[-3:] if i == '1')   
                    # 

                    if sum_left < sum_right:
                        line[0:2] = ['0','0']
                    elif sum_left > sum_right:
                        line[-2:] = ['0','0']
                    

                    if line[0] == line[6] == '1':
                        pass
                    elif line[0] == '1':
                        self.previous.append('left')
                        #self.previous = 'left'
                    elif line[6] == '1':
                        self.previous.append('right')
                        #self.previous = 'right'


        

                    if len(self.previous) == 4:
                        self.previous.pop(0)

                elif self.rotating == 'left':

                    self. updatedrivemotor(-self.speedmotor_left,+ self.speedmotor_rigth)

                elif self.rotating == 'right':

                    self. updatedrivemotor(+self.speedmotor_left,-self.speedmotor_rigth)


                if line[3] == '1' and (line[2] == '1' or line[4] == '1'):
                    self.rotating = None

                print(line)
    def movement_model(self,in_r,in_l):
            # Movement Model
            #self.theta = self.measures.compass
            #self.driveMotors(in_l,in_r)
            self.out_l=(in_l+ self.out_l)*1/2
            self.out_r=(in_r+ self.out_r)*1/2
        
            lin = (self.out_l + self.out_r) /2
            Diametro=1
            rot = (self.out_r - self.out_l) / Diametro

            #self.previous_compass_angle = self.current_compass_angle
                # Obt\u00e9m o \u00e2ngulo atual da b\u00fassola
            
            self.X=self.prev_predicted_loc[0]+ lin*(math.cos(self.theta))
            self.Y=self.prev_predicted_loc[1]+ lin*(math.sin(self.theta)) 
            self.theta=self.theta-rot

            self.predicted_loc = (self.X,self.Y)
            self.prev_predicted_loc=self.predicted_loc
            
            print('self.out_l: ',self.out_l)
            print('self.out_r: ',self.out_r) 
            print('self.theta: ',self.theta)
            print('X: ',self.X)
            print('Y: ',self.Y)
            #print('self.Y: ',self.Y)

            if len(self.compass_buffer) == 10:
                self.compass_buffer.pop(0)
            
            self.compass_buffer.append(self.theta)
            

    def updatedrivemotor(self,in_l,in_r):
        self.driveMotors(in_l,in_r)
        self.movement_model(in_l,in_r)

    def updateGps(self):

        print("######")

    
    def updateCompass(self):

        average = 0

        for x in self.compass_buffer:
            average += x
        
        average = average / 10

        return average



    



    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            #print('Rotate left')
            self.updatedrivemotor(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
           # print('Rotate slowly right')
            self. updatedrivemotor(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            #print('Rotate slowly left')
            self.updatedrivemotor(0.0,0.1)
        else:
            #print('Go')
            self.updatedrivemotor(0.1,0.1)
            
class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    #sys.stdout = open("debug.txt", "w")
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()