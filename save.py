import mysql.connector

mydb = mysql.connector.connect(
  host="localhost",
  user="phpmyadmin",
  passwd="1234",
  database="trash"
)

mycursor = mydb.cursor()

sql = "INSERT INTO data_sensor (v_motor, v_pc, t_est_motor, t_est_pc, trash_level, status) VALUES (%s, %s, %s, %s, %s, %s)"
val = ("12.5", "11.2","30.5","40.7","90.1","1")

mycursor.execute(sql, val)

mydb.commit()

print(mycursor.rowcount, "record inserted.")
