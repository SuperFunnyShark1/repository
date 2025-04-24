#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time


class dead_reckoning_nav( Node ):

    def __init__( self ):
        super().__init__("dead_reckoning_nav")
        #self.lista_charcha_de_prueba = [(1,0,0),(0,1,0),(-1,0,0),(0,-1,4)] #lista de poses
        #self.lista_charcha_de_prueba = [()]  #lista de vels
        self.v_speed = 0.2 # [m/s] #velocidad lineal
        self.w_speed = 1.0 # [rad/s] #velocidad rotacional!
        self.cmd_vel_mux_pub = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 ) #envia al sim
        #TODO: Agregar acá mandar el mensaje no solo al sim, sino q tambien al tb
        #timer_period = 1  # [s]
        #self.timer = self.create_timer( timer_period, self.move )
        #TODO: hacer que mediante otro nodo partamos nuestro script
        #self.recibir_txt = self.create_subscription("geometry_msgs/PoseArray",'goal_list',self.accion_mover_cb,10)
        #la siguiente linea se meteria dentro de self.accion mover de mas arriba, y se le pone un cancel
        self.timer = self.create_timer(0.05,self.temp)
        self.timer = 0 #se actualizara a cada rato
        self.dt = 0
        self.lista_vtiempos = [3,7,4,7,4,4,4,4]

        self.x = 0.2
        self.y = 1.0

    def temp( self ): # es temporal waos
        
        speed = Twist()
        if self.dt == 0:
            self.timer = time.time() #estamos partiendo de nuevo
    
        self.dt = time.time() - self.timer
        if  self.dt > self.lista_vtiempos[0]:            
            self.x += 0.1 #ojala funcuione la vaina alkshjdalksdjaskl
            self.y =  -self.y
            if self.lista_vtiempos == []:
                #cancelar y cerrar el loop
                self.get_logger().info("se han acabado los elementos")
                self.create.timer.cancel()
            else:
                self.lista_vtiempos.pop(0) # le quitamos el primero
            self.dt = 0

        speed.linear.x += self.x
        speed.angular.z += self.y

        #self.velocidad_tb.publish( speed )
        self.cmd_vel_mux_pub.publish( speed )
    #self.get_logger().info() #esta está recibiendo, el % no se q es lo q hace la vdd


    #incorporar despues
    def accion_mover_cb(self,msg): # el msg es lo que recibimos desde el otro nodo
        freq = 0.05 #en sec #este callback nos llamará otra funcion vaaarias veces UwU, itera
        #aqui hay quee pasar el msg de .txt a lista que nodamos leer de panita amiwo
        
        self.timer = self.create.timer(freq,self.mover_robot_a_destino)

    def mover_robot_a_destino(speed_command_lsit): #recibe lista de posiciones objetivo tipo (x,y,theta)
        pass #TODO

    def traducir_pose(self,xytheta):
        #self.v_speed = 0.2 # [m/s] #velocidad lineal
        #self.w_speed = 1.0 # [rad/s] #velocidad rotacional!
        v = self.v_speed
        w = self.w_speed
        out = [0,1,2,3,4]
        x = xytheta[0]
        y = xytheta[1]
        th = xytheta[2]
        turn_t = np.pi/ (2 *w) #tiempo de rotacion, ojito
        dir = int((y / abs(y)))

        out[0] = tuple(v,0,abs(x) / self.v)
        out[1] = tuple(0, dir * w,turn_t  )
        out[2] = tuple(v,0,abs(y) / self.v)
        out[3] = tuple(0,-dir*w , turn_t)
        out[4] = tuple(0, (th*abs(th) ) * w, abs(th) / w )
        #ahora, podriamos poner un map rapido o algo de numpy, para pasar todos
        #estos a la version float corto

        return out
        

def main(args=None):
    rclpy.init(args=args)
    mic = dead_reckoning_nav()
    rclpy.spin( mic )
    mic.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


