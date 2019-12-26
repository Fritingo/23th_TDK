import serial
import time
import threading
import RPi.GPIO as GPIO

mpu6050 = serial.Serial("COM6")  # open COM4 port
mpu6050.baudrate = 115200

ks103 = serial.Serial("COM8")  # open COM4 port
ks103.baudrate = 115200


step = 0
yaw = 0   

ena = 13
in1 = 6
in2 = 5
enb = 18
in3 = 23
in4 = 24
PWM_FREQ = 50
 
GPIO.setmode(GPIO.BCM)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
 
ena_pwm = GPIO.PWM(ena, PWM_FREQ)
enb_pwm = GPIO.PWM(enb, PWM_FREQ)
ena_pwm.start(0)
enb_pwm.start(0)
 
def angle_to_duty_cycle(angle=0):
    duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * angle / 180)
    return duty_cycle

def motor1_test(speed = 100):
    GPIO.output(in1,1)
    GPIO.output(in2,0)
    ena_pwm.ChangeDutyCycle(100)


def Arduino_mpu6050():  
    global yaw

    while True:
    
        serial_error_counter = 0
        # ser.flushInput()
        x = mpu6050.readline()
        # print(x)
        # print(len(x))
    
        if(len(x)<=8 and len(x)>3):
            x = x.decode('utf-8', errors='ignore')
            x = filter(lambda ch: ch in '0123456789.',x)
            x = ''.join((x))
        
        for i in range(len(x)):
            if (x[i]=='.'):
                serial_error_counter = serial_error_counter + 1

        if(serial_error_counter < 2 and serial_error_counter > 0 and len(x)>=3):
            yaw = float(x)
            # print("mpu6050:")
            # print(yaw)
        # print("ss")
        
        # t0 = time.clock()
        #ser.write(yaw.encode())
        # print('time:'+str(time.clock()-t0))
        # ser.flush()
        # print("write")

def Arduino_ks103():
    global step

    while True:
        serial_error_counter = 0
        step_raw = ks103.readline()
    
        if(len(step_raw)<=8 and len(step_raw)>0):
            step_raw = step_raw.decode('utf-8', errors='ignore')
            step_raw = filter(lambda ch: ch in '0123456789.',step_raw)
            step_raw = ''.join((step_raw))
        
        for i in range(len(step_raw)):
            if (step_raw[i]=='.'):
                serial_error_counter = serial_error_counter + 1

        if(serial_error_counter < 2 and len(step_raw)>0):
            step = int(step_raw)
            # print("ks103:")
            # print(step)
   
        
def motor():
    global yaw
    global step
    while True:
        motor1_test()

        # print(yaw)
        # yaw = str(yaw)
        # ks103.write(yaw.encode())
        # print("sended")
        # # ks103.flush()
        # time.sleep(1)




if __name__ == "__main__":
    
    print("Begin")
    mpu6050_thread = threading.Thread(target = Arduino_mpu6050)
    # ks103_thread = threading.Thread(target = Arduino_ks103)
    motor_thread = threading.Thread(target = motor)

    print("thread1")
    mpu6050_thread.start()
    print("thread2")
    motor_thread.start()
    
    
    
    # while True:
    #     print("ks103:")
    #     print(step)
    #     print("mpu6050:")
    #     print(yaw)