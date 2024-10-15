#!/usr/bin/python

import rospy
from geometry_msgs.msg import Point, Twist
from people_msgs.msg import PositionMeasurementArray
import math

def distance(point1, point2):
    """Calcula la distancia entre dos puntos 3D."""
    return math.sqrt(math.pow(point1.x - point2.x, 2) +
                     math.pow(point1.y - point2.y, 2))


class SimplePersonSubscriber(object):
    def __init__(self):
        # Se suscribe al mismo topic
        self.sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, self.callback)
        self.sensor_position = Point(0.0, 0.0, 0.0)  
	# Variables para almacenar la primera persona detectada
        self.tracked_person_id = None
        self.tracked_person_position = Point(0.0, 0.0, 0.0)
        self.last_detection_time = rospy.Time.now()  # Tiempo de la ltima deteccin

        # Tiempo mximo permitido para perder a la persona antes de dejar de seguirla (5 segundos)
        self.detection_timeout = rospy.Duration(5.0)

        # Publisher para controlar el movimiento del TurtleBot
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detected_people = {}  # {id: (position, reliability)}


    def callback(self, msg):
        current_time = rospy.Time.now()
        if not msg.people:  # Si no se detectan personas
            if self.tracked_person_id is not None:
                # Si la persona ha estado desaparecida ms de 5 segundos, dejar de seguirla
                if current_time - self.last_detection_time > self.detection_timeout:
                    rospy.loginfo("Lost person ID: %s after 5 seconds, stopping tracking.", self.tracked_person_id)
                    self.tracked_person_id = None  # Dejar de seguir a la persona
                else:
                    # Seguir la ltima posicin conocida
                    rospy.loginfo("Following last known position of person ID: %s, Position: (%.2f, %.2f)", 
                                  self.tracked_person_id, self.tracked_person_position.x, self.tracked_person_position.y)
            return  # Salir de la funcin si no hay detecciones

        if self.tracked_person_id is None:
            max_reliability = -1
            best_person = None

            for person_id, (pos, rel) in self.detected_people.items():
                rospy.loginfo("Person ID: %s, Reliability: %.2f, Position: (%.2f, %.2f)", 
                              person_id, rel, pos.x, pos.y)

                if rel > max_reliability:
                    max_reliability = rel
                    best_person = (person_id, pos)

            
            if best_person is not None:
                self.tracked_person_id, self.tracked_person_position = best_person
                self.last_detection_time = current_time  # Actualizar tiempo de deteccion
                rospy.loginfo("Started tracking person with highest reliability: ID: %s, Reliability: %.2f, Position: (%.2f, %.2f)", 
                              self.tracked_person_id, max_reliability, self.tracked_person_position.x, self.tracked_person_position.y)
            return
        for pm in msg.people:
            person_position = pm.pos
            person_id = pm.object_id
            reliability = pm.reliability
            dist = distance(self.sensor_position, person_position)  # Calcular distancia desde el sensor

            self.detected_people[person_id] = (person_position, reliability)

            # Imprimir informacion de todas las personas detectadas
            rospy.loginfo("Detected person ID: %s, Reliability: %.2f, Position: (%.2f, %.2f)", 
                          person_id, reliability, person_position.x, person_position.y)

            
        if self.tracked_person_id is not None:
            sperson_position = self.detected_people[self.tracked_person_id][0]
            dist = distance(self.sensor_position, person_position)  # Calcular distancia desde el sensor
            self.tracked_person_position = person_position
            self.last_detection_time = current_time
            rospy.loginfo("-> Currently following person ID: %s, Position: (%.2f, %.2f), Distance: %.2f", 
                          self.tracked_person_id, person_position.x, person_position.y, dist)
            self.turn_towards_person(person_position)

    def turn_towards_person(self, person_pos):
        move_cmd = Twist()

        # Calcular la diferencia en la posicin
        delta_x = person_pos.x - self.sensor_position.x
        delta_y = person_pos.y - self.sensor_position.y
        angle_to_person = math.atan2(delta_y, delta_x)

        # Establecer la velocidad angular para girar hacia la persona
        move_cmd.angular.z = angle_to_person  # Girar hacia el ngulo de la persona

        # Publicar el comando de movimiento
        self.cmd_pub.publish(move_cmd)


if __name__ == '__main__':
    rospy.init_node('simple_person_subscriber')
    sps = SimplePersonSubscriber()
    rospy.spin()
