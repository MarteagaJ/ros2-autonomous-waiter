import RPi.GPIO as GPIO
# import time

# def magnet_test():
#     GPIO.setmode(GPIO.BCM)
#     GPIO.setwarnings(False)

#     channel = 18

#     GPIO.setup(channel, GPIO.OUT, initial=GPIO.LOW)

#     try:
#         # Turn the GPIO pin on
#         GPIO.output(channel, GPIO.HIGH)
#         time.sleep(30)

#         # Turn the GPIO pin off
#         GPIO.output(channel, GPIO.LOW)
#         time.sleep(15)
#     except KeyboardInterrupt:
#         GPIO.output(channel, GPIO.LOW)
#         GPIO.cleanup(channel)

class Magnet:
    pin = None

    def _init_(self, pin):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    def magnet_on(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def magnet_off(self):
        GPIO.output(self.pin, GPIO.LOW)
