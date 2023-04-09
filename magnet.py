import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

channel = 18

GPIO.setup(channel, GPIO.OUT, initial=GPIO.LOW)

try:
    while True:
        # Turn the GPIO pin on
        GPIO.output(channel, GPIO.HIGH)
        time.sleep(30)

        # Turn the GPIO pin off
        GPIO.output(channel, GPIO.LOW)
        time.sleep(15)
except KeyboardInterrupt:
    GPIO.output(channel, GPIO.LOW)
    GPIO.cleanup(channel)
