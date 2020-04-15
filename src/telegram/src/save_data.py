#!/usr/bin/env python
import rospy

from std_msgs.msg import Int32
from random import randint

import mysql.connector

#// display received data
def callback(data):
    rospy.loginfo("I received %s", data.data)

#//define the subscriber
def random_subscriber():
    rospy.init_node('random_subscriber')
    rospy.Subscriber('data_jarak',Int32, callback)
    rospy.spin()

if __name__=='__main__':
    #try:
    #    random_number_publisher()
    #except rospy.ROSInterruptException:
    #    pass
    try:
        random_subscriber()
        mydb = mysql.connector.connect(
            host="localhost",
            user="phpmyadmin",
            passwd="1234",
            database="trash"
        )
        mycursor = mydb.cursor()
        sql = "INSERT INTO data_sensor (id_robot,v_motor, v_pc, t_est_motor, t_est_pc, trash_level, status) VALUES (%s,%s, %s, %s, %s, %s, %s)"
        val = ("1","12.5", "11.2","30.5","40.7","80.7","1")

        mycursor.execute(sql, val)

        mydb.commit()

        print(mycursor.rowcount, "record inserted.")
    except rospy.ROSInterruptException:
        print("Error reading data from MySQL table", e)
