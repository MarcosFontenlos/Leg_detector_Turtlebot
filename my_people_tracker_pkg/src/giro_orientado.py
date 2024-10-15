#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from people_msgs.msg import PositionMeasurementArray
from sensor_msgs.msg import LaserScan

class PersonaSeguir(object):
    def __init__(self):
        self.personas = []  # Lista para almacenar las personas detectadas
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)  # Publicador para mover el robot
        self.sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, self.callback)
        self.selected_person = None  # Persona seleccionada para seguir
        self.UMBRAL_RELIABILITY = 0.6  # Umbral para considerar una persona como valida

    def callback(self, msg):
        self.personas = msg.people
        if len(self.personas) > 0:
            for persona in self.personas:
                print("ID: ",persona.object_id, "Posicion: ",persona.pos.x, persona.pos.y, persona.pos.z, "Confiabilidad: ",persona.reliability)
            
            # Seleccionar a la persona con mayor probabilidad
            self.selected_person = max(self.personas, key=lambda p: p.reliability)

    def calcular_angulo_persona(self, pos_persona):
        # Calcular el angulo entre el robot y la persona (usando solo x e y)
        angulo = math.atan2(pos_persona.y, pos_persona.x)
        return angulo

    def girar_hacia_persona(self):
        if self.selected_person:
            if self.selected_person.reliability >= self.UMBRAL_RELIABILITY:
                print("Persona seleccionada con confiabilidad alta: ", self.selected_person.reliability)
                
                # Calcular el angulo hacia la persona
                angulo_persona = self.calcular_angulo_persona(self.selected_person.pos)

                # Crear mensaje Twist para girar el robot
                twist = Twist()
                if abs(angulo_persona) > 0.15:  # Si el angulo es mayor que un umbral pequeno, girar
                    twist.angular.z = 0.7 * (1 if angulo_persona > 0 else -1)  # Velocidad de giro
                else:
                    twist.angular.z = 0  # Detener el giro si ya esta alineado

                self.cmd_vel_pub.publish(twist)
            else:
                print("Confiabilidad de la persona seleccionada es baja, no se girara.")
    
    def spin(self):
        rate = rospy.Rate(10)  # Frecuencia de 10 Hz
        while not rospy.is_shutdown():
            self.girar_hacia_persona()
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('seguimiento_persona')
        seguir_persona = PersonaSeguir()
        seguir_persona.spin()
    except rospy.ROSInterruptException:
        pass