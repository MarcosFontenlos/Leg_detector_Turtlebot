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
        self.UMBRAL_DISTANCIA = 0.5  # Distancia minima a la persona antes de detener el avance

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

    def calcular_distancia_persona(self, pos_persona):
        # Calcular la distancia entre el robot y la persona (usando solo x e y)
        distancia = math.sqrt(pos_persona.x**2 + pos_persona.y**2)
        return distancia

    def seguir_persona(self):
        if self.selected_person:
            # Verificar si la persona seleccionada tiene una confiabilidad alta
            if self.selected_person.reliability >= self.UMBRAL_RELIABILITY:
                print("Persona seleccionada con confiabilidad alta: ", self.selected_person.reliability)
              # Calcular el angulo y la distancia hacia la persona
                angulo_persona = self.calcular_angulo_persona(self.selected_person.pos)
                distancia_persona = self.calcular_distancia_persona(self.selected_person.pos)

                # Crear mensaje Twist para mover el robot
                twist = Twist()
                #Ajustar la velocidad angular (giro)
                if abs(angulo_persona) > 0.1:  # Si el angulo es mayor que un umbral pequeno, girar
                    twist.angular.z = 0.6 * (1 if angulo_persona > 0 else -1)  # Velocidad de giro
                else:
                    twist.angular.z = 0  # Detener el giro si ya esta alineado
                # Ajustar la velocidad lineal (avance) si la persona no esta demasiado cerca
                if distancia_persona > self.UMBRAL_DISTANCIA:
                    twist.linear.x = max(-0.45, distancia_persona * -0.15)  # Velocidad proporcional a la distancia
                    #twist.linear.x = -0.3
                else:
                    twist.linear.x = 0  # Detener avance si la persona esta muy cerca

                # Publicar los comandos de movimiento
                self.cmd_vel_pub.publish(twist)
            else:
                print("Confiabilidad de la persona seleccionada es baja, no se seguira ni girara.")

    def spin(self):
        rate = rospy.Rate(4)  # Frecuencia de 10 Hz
        while not rospy.is_shutdown():
            self.seguir_persona()
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('seguimiento_persona')
        seguir_persona = PersonaSeguir()
        seguir_persona.spin()
    except rospy.ROSInterruptException:
        pass