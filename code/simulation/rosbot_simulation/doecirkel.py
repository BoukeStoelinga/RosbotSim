#!/usr/bin/env python
# Houd die eerste regel er in, fixt soms dingen
import rospy
from geometry_msgs.msg import Twist #zoek het msg type dat je nodig hebt
# soms zit msg in iets anders zoals std_msgs voor strings, floats enzo

# from std_msgs.msg import String
def setzero(set_vel):
    set_vel.linear.x = 0
    set_vel.linear.y = 0
    set_vel.linear.z = 0
    set_vel.angular.x = 0
    set_vel.angular.y = 0
    set_vel.angular.z = 0
def circle():
    vel = Twist()
    setzero(vel)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  #waar hij moet publishen, wat voor message en hoe lang hij berichten blijft opslaan
    rospy.init_node('ikbenpython', anonymous=True) #zeggen wat voor node hij is en anonymous erachter zodat 2x opstarten unieke nodes geeft
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown(): #blijven doen totdat ros hem wilt stoppen, denk bv aan ctrl x
        vel.linear.x = 0.3
        vel.angular.z = 1
        rospy.loginfo(vel) #zet in de comand line wat de snelheid is
        pub.publish(vel) #publish de snelheid
        rate.sleep() #wacht zodat de frequntie van erboven gevolgd wordt

"""
Stukje hieronder: voer de functie circle() alleen uit als deze python file
als '__main__' wordt gezien. Dit is alleen als je deze file runt. Zo kun je
in een andere file
import doecirkel.py
gebruiken zonder dat het hele script wordt uitgevoerd

try:
    doe iets
except <welke error krijgt doe iets, leeg is alle errors>:
    als doe iets een error geeft doet negeert hij de error en doet
    hij dit
"""
if __name__ == '__main__': #
    try:
        circle()
    except rospy.ROSInterruptException:
        pass
