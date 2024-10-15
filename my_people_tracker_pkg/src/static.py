#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import LaserScan  # Para el scanner
from geometry_msgs.msg import Twist    # Para la velocidad

from people_msgs.msg import People, Person, PositionMeasurementArray


class Persona(object):
    def __init__(self): 
        self.people = {}  
        self.TIMEOUT = rospy.Duration(rospy.get_param('~timeout', 1.0))     
        self.sub = rospy.Subscriber('/people_tracker_measurements',PositionMeasurementArray,self.callback)
        self.ppub = rospy.Publisher('/people', People, queue_size=10)  # Publicador para las personas

    def callback(self, msg):
        for pm in msg.people:
            if pm.object_id in self.people:
                self.people[pm.object_id].update(pm)  # Actualizar si ya esta en el diccionario
            else:
                # Crear una nueva persona si no esta en el diccionario
                p = {
                    "id": pm.object_id,
                    "position": pm.pos,
                    "reliability": pm.reliability,
                    "last_seen": rospy.Time.now()  # Guardar el tiempo actual
                }
                self.people[pm.object_id] = p
    def spin(self):
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            # Remove People Older Than timeout param
            now = rospy.Time.now()
            for p in self.people.values():
                if now - p.age() > self.TIMEOUT:
                    del self.people[p.id()]
            self.publish()
            rate.sleep()


    def publish(self):
        pl = People()
        pl.header.frame_id = "base_link"  # Establecer un frame_id valido
        pl.header.stamp = rospy.Time.now()

        for p in self.people.values():
            person = Person()
            person.name = str(p["id"])
            person.position.x = p["position"].x
            person.position.y = p["position"].y
            person.position.z = p["position"].z
            person.reliability = p["reliability"]
            pl.people.append(person)

        self.ppub.publish(pl)



def distance(leg1, leg2):
    return math.sqrt(math.pow(leg1.x - leg2.x, 2) +
                     math.pow(leg1.y - leg2.y, 2) +
                     math.pow(leg1.z - leg2.z, 2))

if __name__ == '__main__':
    try:
        rospy.init_node('filtro')
        print("s")
        vt = Persona()
        vt.spin()
    except rospy.ROSInterruptException:
        pass

