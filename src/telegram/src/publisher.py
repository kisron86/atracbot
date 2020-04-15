#!/usr/bin/env python
import rospy

import mysql.connector

import sys
import time
import random
import datetime
import telepot
import os
import random

from mysql.connector import Error
from telepot.namedtuple import ReplyKeyboardMarkup, KeyboardButton
from telepot.loop import MessageLoop
from telepot.namedtuple import InlineKeyboardMarkup, InlineKeyboardButton


if __name__=='__main__':
    connection = mysql.connector.connect(host='localhost',
                                            database='trash',
                                            user='phpmyadmin',
                                            password='1234')
    sql_select_Query = "select * from data_sensor"
    cursor = connection.cursor()
    cursor.execute(sql_select_Query)
    records = cursor.fetchall()
    if (connection.is_connected()):
        connection.close()
        cursor.close()
        print("MySQL connection is closed")

def handle(msg):
    global command
    chat_id = msg['chat']['id']
    command = msg['text']
    print("Mudun")
    if command == '/start' or command == '/Kembali' or command == '/Start':
        bot.sendMessage(chat_id, 'Halo Selamat datang Ada yang bisa dibantu?',
                        reply_markup=ReplyKeyboardMarkup(
                        keyboard=[[KeyboardButton(text="/Voltage"), KeyboardButton(text="/Volume")],[KeyboardButton(text="/a"), KeyboardButton(text="/b")]]))
    elif command == '/Voltage':
        for row in records:
            id = row[0]
            nama = row[1]
            alamat = row[3]

    bot.sendMessage(chat_id, id)
    bot = telepot.Bot('576188757:AAESFfKu1T7jvZbASG6wpHf0m1WG0-xlOmQ')
    bot.message_loop(handle)
    print ('Menunggu Perintah ...')

while 1:
    time.sleep(10)
pass
