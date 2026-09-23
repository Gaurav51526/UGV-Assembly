# UGV-Assembly
UGV assemble of the hardware  and sensors intigration



import serial
import time
import statistics
import RPi.GPIO as GPIO


# ==========================================================
# TFmini Plus
# ==========================================================

TFMINI_PORT = "/dev/ttyS0"
BAUDRATE = 115200

ser = serial.Serial(
    port=TFMINI_PORT,
    baudrate=BAUDRATE,
    timeout=0.05
)

ser.reset_input_buffer()


# ==========================================================
# ULTRASONIC SENSORS
# ==========================================================

LEFT_US = 17
FRONT_US = 27
RIGHT_US = 22


# ==========================================================
# L298N MOTOR DRIVER
# ==========================================================

IN1 = 23
IN2 = 24
IN3 = 25
IN4 = 26


# ==========================================================
# GPIO SETUP
# ==========================================================

GPIO.setmode(GPIO.BCM)

GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(LEFT_US, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(FRONT_US, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(RIGHT_US, GPIO.OUT, initial=GPIO.LOW)


# ==========================================================
# MOTOR FUNCTIONS
# ==========================================================

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


def forward():
    # Left motor
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

    # Right motor
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)


def reverse():
    # Left motor
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

    # Right motor
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)


def turn_left():
    # Left motor reverse
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

    # Right motor forward
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)


def turn_right():
    # Left motor forward
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

    # Right motor reverse
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)


# ==========================================================
# GROVE ULTRASONIC V2.0
# Single SIG pin = Trigger + Echo
# ==========================================================

def read_ultrasonic(pin):

    # -----------------------------
    # Trigger
    # -----------------------------

    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

    time.sleep(0.000002)

    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.000010)
    GPIO.output(pin, GPIO.LOW)

    # -----------------------------
    # Change SIG to input
    # -----------------------------

    GPIO.setup(pin, GPIO.IN)

    # -----------------------------
    # Wait for echo HIGH
    # -----------------------------

    timeout = time.monotonic() + 0.03

    while GPIO.input(pin) == GPIO.LOW:

        if time.monotonic() > timeout:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            return None

    start = time.monotonic()

    # -----------------------------
    # Wait for echo LOW
    # -----------------------------

    timeout = time.monotonic() + 0.03

    while GPIO.input(pin) == GPIO.HIGH:

        if time.monotonic() > timeout:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            return None

    end = time.monotonic()

    # Return pin to safe state
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    # -----------------------------
    # Calculate distance
    # -----------------------------

    duration = end - start

    distance = (duration * 34300) / 2

    if distance < 2 or distance > 350:
        return None

    return distance


# ==========================================================
# ACCURATE ULTRASONIC READING
# ==========================================================

def accurate_ultrasonic(pin):

    readings = []

    # Take 5 measurements
    for _ in range(5):

        distance = read_ultrasonic(pin)

        if distance is not None:
            readings.append(distance)

        # Avoid interference between ultrasonic measurements
        time.sleep(0.05)

    if not readings:
        return None

    # Median removes occasional bad reading
    return statistics.median(readings)


# ==========================================================
# TFMINI PLUS READING
# ==========================================================

def read_tfmini():

    while True:

        b1 = ser.read(1)

        if not b1:
            return None

        if b1[0] != 0x59:
            continue

        b2 = ser.read(1)

        if not b2:
            return None

        if b2[0] != 0x59:
            continue

        data = ser.read(7)

        if len(data) != 7:
            return None

        frame = bytes([0x59, 0x59]) + data

        # Check checksum
        checksum = sum(frame[:8]) & 0xFF

        if checksum != frame[8]:
            continue

        distance = frame[2] | (frame[3] << 8)

        strength = frame[4] | (frame[5] << 8)

        if distance <= 0:
            continue

        return distance, strength


# ==========================================================
# GET ALL ULTRASONIC DISTANCES
# ==========================================================

def get_distances():

    left = accurate_ultrasonic(LEFT_US)

    front = accurate_ultrasonic(FRONT_US)

    right = accurate_ultrasonic(RIGHT_US)

    return left, front, right


# ==========================================================
# MOTOR DECISION
# ==========================================================

LIMIT = 40


def obstacle_control(left, front, right):

    # ------------------------------------------------------
    # If front sensor has no reading
    # Stop for safety
    # ------------------------------------------------------

    if front is None:

        stop()

        print("FRONT SENSOR ERROR -> STOP")

        return


    # ------------------------------------------------------
    # FRONT CLEAR
    # ------------------------------------------------------

    if front >= LIMIT:

        forward()

        print("FORWARD")

        return


    # ------------------------------------------------------
    # FRONT BLOCKED
    # ------------------------------------------------------

    print("FRONT BLOCKED")

    # If side sensor has no reading,
    # treat it as blocked for safety.

    if left is None:
        left = 0

    if right is None:
        right = 0


    # ------------------------------------------------------
    # BOTH SIDES BLOCKED
    # ------------------------------------------------------

    if left < LIMIT and right < LIMIT:

        print("LEFT + RIGHT BLOCKED")
        print("REVERSING FOR 10 SECONDS")

        reverse()

        # Reverse for 10 seconds
        time.sleep(10)

        stop()

        print("10 SECOND REVERSE COMPLETE")

        # Re-check left and right
        left = accurate_ultrasonic(LEFT_US)
        right = accurate_ultrasonic(RIGHT_US)

        if left is None:
            left = 0

        if right is None:
            right = 0

        print(
            "AFTER REVERSE -> LEFT: {:.1f} cm | RIGHT: {:.1f} cm"
            .format(left, right)
        )

        # --------------------------------------------------
        # Choose LEFT
        # --------------------------------------------------

        if left >= LIMIT and left > right:

            print("TURNING LEFT")

            turn_left()

            time.sleep(0.7)

            stop()

        # --------------------------------------------------
        # Choose RIGHT
        # --------------------------------------------------

        elif right >= LIMIT and right > left:

            print("TURNING RIGHT")

            turn_right()

            time.sleep(0.7)

            stop()

        # --------------------------------------------------
        # Both available and equal
        # --------------------------------------------------

        elif left >= LIMIT and right >= LIMIT:

            print("BOTH SIDES CLEAR - TURN LEFT")

            turn_left()

            time.sleep(0.7)

            stop()

        # --------------------------------------------------
        # Still blocked
        # --------------------------------------------------

        else:

            print("ALL SIDES BLOCKED -> STOP")

            stop()

        return


    # ------------------------------------------------------
    # RIGHT CLEAR, LEFT BLOCKED
    # ------------------------------------------------------

    if right >= LIMIT and left < LIMIT:

        print("RIGHT CLEAR -> TURN RIGHT")

        turn_right()

        time.sleep(0.7)

        stop()

        return


    # ------------------------------------------------------
    # LEFT CLEAR, RIGHT BLOCKED
    # ------------------------------------------------------

    if left >= LIMIT and right < LIMIT:

        print("LEFT CLEAR -> TURN LEFT")

        turn_left()

        time.sleep(0.7)

        stop()

        return


    # ------------------------------------------------------
    # BOTH SIDES CLEAR
    # Choose the side with greater distance
    # ------------------------------------------------------

    if left >= LIMIT and right >= LIMIT:

        if left > right:

            print("BOTH CLEAR -> LEFT HAS MORE SPACE")

            turn_left()

        else:

            print("BOTH CLEAR -> RIGHT HAS MORE SPACE")

            turn_right()

        time.sleep(0.7)

        stop()

        return


    # ------------------------------------------------------
    # Safety
    # ------------------------------------------------------

    stop()


# ==========================================================
# MAIN PROGRAM
# ==========================================================

print()
print("==========================================")
print(" RASPBERRY PI ROBOT")
print(" TFmini Plus + 3 Grove Ultrasonic")
print(" L298N Motor Driver")
print("==========================================")
print()
print("LEFT   = GPIO 17")
print("FRONT  = GPIO 27")
print("RIGHT  = GPIO 22")
print()
print("L298N:")
print("IN1 = GPIO 23")
print("IN2 = GPIO 24")
print("IN3 = GPIO 25")
print("IN4 = GPIO 26")
print()
print("Obstacle limit = 40 cm")
print("Reverse time   = 10 seconds")
print("==========================================")


last_lidar_print = time.monotonic()


try:

    while True:

        # ==============================================
        # Read ultrasonic sensors
        # ==============================================

        left, front, right = get_distances()


        # ==============================================
        # Display ultrasonic readings
        # ==============================================

        print()
        print("---------- ULTRASONIC ----------")

        if left is not None:
            print("LEFT  : {:.2f} cm".format(left))
        else:
            print("LEFT  : ERROR")

        if front is not None:
            print("FRONT : {:.2f} cm".format(front))
        else:
            print("FRONT : ERROR")

        if right is not None:
            print("RIGHT : {:.2f} cm".format(right))
        else:
            print("RIGHT : ERROR")

        print("--------------------------------")


        # ==============================================
        # Motor decision
        # ==============================================

        obstacle_control(left, front, right)


        # ==============================================
        # TFmini Plus
        # Print approximately every 2 seconds
        # ==============================================

        if time.monotonic() - last_lidar_print >= 2:

            lidar = read_tfmini()

            if lidar is not None:

                lidar_distance, strength = lidar

                print()
                print(
                    "TFmini Plus: {:.2f} cm | {:.2f} m | Strength: {}".format(
                        lidar_distance,
                        lidar_distance / 100,
                        strength
                    )
                )

            else:

                print("TFmini Plus: No reading")

            last_lidar_print = time.monotonic()


        # Small delay
        time.sleep(0.05)


except KeyboardInterrupt:

    print()
    print("Program stopped by user")


finally:

    stop()

    ser.close()

    GPIO.cleanup()

    print("Motors stopped")
    print("UART closed")
    print("GPIO cleaned up")
