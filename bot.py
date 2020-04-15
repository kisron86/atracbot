import sys
import time
import random
import datetime
import telepot
import os
import random
from telepot.namedtuple import ReplyKeyboardMarkup, KeyboardButton
from telepot.loop import MessageLoop
from telepot.namedtuple import InlineKeyboardMarkup, InlineKeyboardButton

import mysql.connector
from mysql.connector import Error

try:
    connection = mysql.connector.connect(host='localhost',
                                         database='trash',
                                         user='phpmyadmin',
                                         password='Qwerty123#')
    sql_select_Query = "select * from data_sensor ORDER BY id DESC LIMIT 1"
    cursor = connection.cursor()
    cursor.execute(sql_select_Query)
    records = cursor.fetchall()

except Error as e:
    print("Error reading data from MySQL table", e)
finally:
    if (connection.is_connected()):
        connection.close()
        cursor.close()
        print("MySQL connection is closed")

def handle(msg):
    global command
    chat_id = msg['chat']['id']
    command = msg['text']   
    if command == '/start' or command == '/Kembali' or command == '/Start':
        bot.sendMessage(chat_id, 'Halo Selamat datang Ada yang bisa dibantu?',
                       reply_markup=ReplyKeyboardMarkup(
                       keyboard=
                       [[KeyboardButton(text="/Voltage_Motor"), 
                       KeyboardButton(text="/Voltage_PC")],
                       [KeyboardButton(text="/Estimation_Motor"), 
                       KeyboardButton(text="/Estimation_PC")],
                       [KeyboardButton(text="/Trash_Level"), 
                       KeyboardButton(text="/Power_Off")]]))
        
    elif command == '/Voltage_Motor':
        for row in records:
                v_motor = str(row[2])
        bot.sendMessage(chat_id, "Voltage of battery motor is " + v_motor + " Volt")
    elif command == '/Voltage_PC':
        for row in records:
                v_pc = str(row[3])
        bot.sendMessage(chat_id, "Voltage of battery PC is " + v_pc + " Volt")
    elif command == '/Estimation_Motor':
        for row in records:
                t_est_motor = str(row[4])
        bot.sendMessage(chat_id, "Estimation working time of battery motor is " + t_est_motor + " Minutes")
    elif command == '/Estimation_PC':
        for row in records:
                t_est_pc = str(row[5])
        bot.sendMessage(chat_id, "Estimation working time of battery PC is " + t_est_pc + " Minutes")
    elif command == '/Trash_Level':
        for row in records:
                trash_level = str(row[6])
        bot.sendMessage(chat_id, "The level of trash is " + trash_level + " %")
    elif command == '/Power_Off':
        for row in records:
                status = row[7]
                if status == 1:
                    bot.sendMessage(chat_id, "Are you sure to shutdown the robot? Y/T")
    elif command == 'Y':
        bot.sendMessage(chat_id, "The robot is stopped working");
                    
bot = telepot.Bot('465769589:AAFg10QLqp7XKf_2DpGgTfCWPzCtiOe0Yus')
bot.message_loop(handle)
print ('Menunggu Perintah ...')

while 1:
	time.sleep(10)
