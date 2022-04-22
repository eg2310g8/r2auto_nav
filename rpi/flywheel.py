import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

motor_A = 13
motor_B = 19

coil_A_1_pin = 23 # pink
coil_A_2_pin = 24 # orange
coil_B_1_pin = 25 # blue
coil_B_2_pin = 8 # yellow

StepCount=8
Seq = [
    [0,0,0,1],
    [0,0,1,1],
    [0,0,1,0],
    [0,1,1,0],
    [0,1,0,0],
    [1,1,0,0],
    [1,0,0,0],
    [1,0,0,1]
    ]

GPIO.setup(motor_A, GPIO.OUT)
GPIO.setup(motor_B, GPIO.OUT)

GPIO.setup(coil_A_1_pin, GPIO.OUT)
GPIO.setup(coil_A_2_pin, GPIO.OUT)
GPIO.setup(coil_B_1_pin, GPIO.OUT)
GPIO.setup(coil_B_2_pin, GPIO.OUT)

pwm_A = GPIO.PWM(motor_A, 50)
pwm_B = GPIO.PWM(motor_B, 50)

def setStep(w1, w2, w3, w4):
    GPIO.output(coil_A_1_pin, w1)
    GPIO.output(coil_A_2_pin, w2)
    GPIO.output(coil_B_1_pin, w3)
    GPIO.output(coil_B_2_pin, w4)

def backwards(delay, steps):
    for i in range(steps):
        for j in range(StepCount):
            setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3])
            time.sleep(delay/1000)

def forwards(delay, steps):
    for i in range(steps):
        for j in reversed(range(StepCount)):
            setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3])
            time.sleep(delay/1000)

def fire(num, speed):
    pwm_A.start(speed)
    pwm_B.start(0)

    #load
    backwards(0.8, 150)
    
    time.sleep(0.5)

    for i in range(num):
        forwards(0.8, 330)
        time.sleep(0.5)
        if i+1 == num:
            pwm_A.stop()
            pwm_B.stop()
            backwards(0.8, 180)
        else:
            backwards(0.8, 330)
        time.sleep(0.1)

    #forwards(1, 180)

    pwm_A.stop()
    pwm_B.stop()

if __name__ == '__main__':
    try:
        fire(2, 30)
    except KeyboardInterrupt:
        pwm_A.stop()
        pwm_B.stop()
        GPIO.cleanup



#stepper fire 260
#move to default 
