#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
Left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
Left_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
Left = MotorGroup(Left_motor_a, Left_motor_b)
Right_motor_a = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
Right_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
Right = MotorGroup(Right_motor_a, Right_motor_b)
controller_1 = Controller(PRIMARY)
Conveyor = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
Intake = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)
plateOpt = Optical(Ports.PORT4)
LDistance = Distance(Ports.PORT5)
RDistance = Distance(Ports.PORT6)
Solenoid = DigitalOut(brain.three_wire_port.a)
doinker = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
conveyorOpt = Optical(Ports.PORT21)
encoder_g = Encoder(brain.three_wire_port.g)
led1 = Led(brain.three_wire_port.b)
led2 = Led(brain.three_wire_port.c)
led3 = Led(brain.three_wire_port.d)
led4 = Led(brain.three_wire_port.e)
led5 = Led(brain.three_wire_port.f)
inertial_12 = Inertial(Ports.PORT12)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      Downum 2 (python version)
#	Author:       Zach Rushworth
#	Created:      January 27, 2025
#   Last modified: March 8, 2025
#	Description:  Code for Downum 2, 5525A's High Stakes Robot
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code
piston = 0
side = 0
doink = 0
stop = 0

#PID variables
WheelCircumInch = 10.25
Gearing = 2.333
senseWheelCircmInch = 8.5
forward1 = 0

def drivePID(driveDistanceinch, driveRotation, percentPwr, percentPwrTrn, band = 1):

    #How far the robot should move
    driveDistance = driveDistanceinch * 40.2
    angleTurn = driveRotation
    
    #PID Constants
    #Tuning done with Randy Gross
    kP = 0.002
    kI = 0.00015
    kD = 0.015

    #PID variables
    error = 0                           #how far the robot is from the target rotation
    integral = 0                        #area under the error vs time graph
    derivative = 0                      #slope of error vs time graph
    prevError = 0                       #error for previous iteration of loop
    currentDistance = 0

    #motor power variables
    maxPower = percentPwr / 100
    motorPower = 0                      #how much power to apply to the motors
    prevMotorPower = 0                  #power in previous iteration
    slewRate = 0.05                     #starting scaler for smooth launch
    scaledMotorPower = 0                #scalinf for PROS motor_move_voltage to mV, using 11 volt int for PROS
    PiLimit = 100                       #limit error before PIntegration is calculated
    errorBand = 20 * band               #error amount used for deadband

    currentTime = 0

    #PID turn constants
    kPTrn = 0.02
    kITrn = 0.002
    kDTrn = 0.033

    #Turn PID variables
    errorTrn = 0
    integralTrn = 0
    derivativeTrn = 0
    prevErrorTrn = 0

    #Motor turn power variables
    maxPowerTrn = percentPwrTrn / 100
    motorpowerTrn = 0
    prevMotorPowerTrn = 0
    slewRateTrn = 0.02
    PiLimitTrn = 10
    errorBandTrn = 0.75
    currentRotation = 0

    #reset motor encoders
    Left.set_position(0, DEGREES)
    Right.set_position(0, DEGREES)
    encoder_g.set_position(0, RotationUnits.DEG)

    #store a previous error to keep derivative from spiking at first call
    prevError = driveDistance - encoder_g.position(RotationUnits.DEG)
    prevErrorTrn = angleTurn - inertial_12.rotation(RotationUnits.DEG)

    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1,1)
    controller_1.screen.print("Moving", driveDistanceinch, "inches", sep=" ")
    controller_1.screen.set_cursor(2,1)
    controller_1.screen.print("and", driveRotation, "degrees", sep=" ")

    #PID Loop
    while True:
        #Forward is negative
        currentDistance = 0 - encoder_g.position(RotationUnits.DEG)
        error = driveDistance - currentDistance

        currentRotation = inertial_12.rotation(RotationUnits.DEG)
        errorTrn = angleTurn - currentRotation

        if abs(error) < PiLimit:
            #update the integral term
            integral += error

        if abs(errorTrn) < PiLimitTrn:
            integralTrn += errorTrn
        
        derivative = error - prevError
        motorPower = (kP * error) + (kI * integral) + (kD * derivative)

        derivativeTrn = errorTrn - prevErrorTrn
        motorPowerTrn = (kPTrn * errorTrn) + (kITrn * integralTrn) + (kDTrn * derivativeTrn)

        #caps motor power at 1
        if motorPower > 1:
            motorPower = 1
        if motorPower < -1:
            motorPower = -1

        if motorPowerTrn > 1:
            motorPowerTrn = 1
        if motorPowerTrn < -1:
            motorPowerTrn = -1

        #slew rate limiter
        #this slowly spools up the motors to prevent it jerking
        if motorPower > prevMotorPower + slewRate:
            motorPower = prevMotorPower + slewRate
        if motorPower < prevMotorPower - slewRate:
            motorPower = prevMotorPower - slewRate

        #finally get to spin the motors
        Left.spin(FORWARD, 11 * (motorPower + motorPowerTrn), VOLT) 
        Right.spin(FORWARD, 11 * (motorPower - motorPowerTrn), VOLT)

        #Exit the PID loop if the robot is within errorBand degrees of the target
        if error > 0 - errorBand and error < errorBand and error - prevError > 0 - errorBand and error - prevError < errorBand and errorTrn > 0 - errorBandTrn and errorTrn < errorBandTrn and errorTrn - prevErrorTrn > -errorBandTrn and errorTrn - prevErrorTrn < errorBandTrn:
            break

        #update "previous" variables
        prevMotorPower = motorPower
        prevError = error

        prevMotorPowerTrn = motorPowerTrn
        prevErrorTrn = errorTrn

        controller_1.screen.set_cursor(3,1)
        controller_1.screen.print(encoder_g.position(RotationUnits.DEG) / 40.2, "inches,", inertial_12.rotation(RotationUnits.DEG), "deg", sep=" ")
        wait(20, MSEC)
        currentTime += 20

    Left.stop()
    Right.stop()
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1,1)
    controller_1.screen.print("Moved", driveDistanceinch, "inches", sep=" ")
    controller_1.screen.set_cursor(2,1)
    controller_1.screen.print("and", driveRotation, "degrees", sep=" ")
    controller_1.screen.set_cursor(3,1)
    controller_1.screen.print("in", currentTime / 1000, "seconds", sep=" ")
  
def sensors():
    doinker.set_velocity(30, PERCENT)
    doinker.spin_for(FORWARD, 130, DEGREES)
    global side
    global alliance
    plateOpt.set_light(LedStateType.ON)
    plateOpt.set_light_power(100, PERCENT)
    wait(1, SECONDS)
    plateOpt.set_light(LedStateType.ON)

    #reads plate color
    #since the sensor reads the team we aren't on, it's flipped
    if plateOpt.color() == Color.RED:
        brain.screen.set_pen_color(Color.BLUE)
        brain.screen.set_fill_color(Color.BLUE)
        brain.screen.draw_rectangle(0, 220, 20, 20)
        alliance = 1
    elif plateOpt.color() == Color.BLUE:
        brain.screen.set_pen_color(Color.RED)
        brain.screen.set_fill_color(Color.RED)
        brain.screen.draw_rectangle(0, 220, 20, 20)
        alliance = 2
    else:
        brain.screen.set_pen_color(Color.GREEN)
        brain.screen.set_fill_color(Color.GREEN)
        brain.screen.draw_rectangle(0, 220, 20, 20)
        alliance = -1
    

    leftD = abs(LDistance.object_distance(INCHES) - 18)
    rightD = abs(RDistance.object_distance(INCHES) - 18)
    #1 for positive, -1 for negative
    if leftD < rightD:
        if alliance == 1:
         side = 1
        elif alliance == 2:
         side = -1
    elif rightD < leftD:
        if alliance == 1:
         side = -1
        elif alliance == 2:
         side = 1
    
    brain.screen.set_pen_color(Color.WHITE)
    brain.screen.set_fill_color(Color.WHITE)
    if side == 1:
        #draws plus
        brain.screen.draw_rectangle(8, 223, 4, 14)
        brain.screen.draw_rectangle(3, 228, 14, 4)
    elif side == -1:
        #draws minus
        brain.screen.draw_rectangle(3, 228, 14, 4)
    else:
        #draws dot
        brain.screen.draw_rectangle(8, 228, 4, 4)

    doinker.set_velocity(30, PERCENT)
    doinker.spin_for(FORWARD, -130, DEGREES)
    if not doinker.is_spinning():
        doinker.stop()

def driver_control():
    #robot won't move until inertial sensor calibrated
    while not calibrate:
        wait(50, MSEC)
    while True:
        #drive control
        Left.set_velocity(controller_1.axis3.position() + controller_1.axis1.position(), PERCENT)
        Right.set_velocity(controller_1.axis3.position() - controller_1.axis1.position(), PERCENT)
        Left.spin(FORWARD)
        Right.spin(FORWARD)

        #controller deadzone
        if controller_1.axis3.position() == 0 and controller_1.axis1.position() == 0:
            Left.set_stopping(BRAKE)
            Right.set_stopping(BRAKE)

        #debug lights for the team
        #no blue LEDs so I used green
        if plateOpt.color() == Color.BLUE:
            led1.on()
            led2.off()
        elif plateOpt.color() == Color.RED:
            led1.off()
            led2.on()
        else:
            led1.off()
            led2.off()

        #debug lights for the ring
        #still no blue LEDs
        if conveyorOpt.color() == Color.RED:
            led3.on()
            led4.off()
        elif conveyorOpt.color() == Color.BLUE:
            led3.off()
            led4.on()
        else:
            led3.off()
            led4.off()

        #this turns on a light if one of the drive motors gets too hot
        temps = [Left_motor_a.temperature(), Left_motor_b.temperature(), Right_motor_a.temperature(), Right_motor_b.temperature()]
        if max(temps) >= 55:
            led5.on()
        else:
            led5.off()

def autonomous():
    while not calibrate:
        wait(50, MSEC)
    sensors()
    Intake.set_velocity(100, PERCENT)
    Intake.spin(FORWARD)

    if side == -1:
        drivePID(36, 0, 50, 40)
        wait(1, SECONDS)
        Intake.stop()
    elif side == 1:
        drivePID(6, 0, 100, 40)
        Intake.stop()
        wait(0.5, SECONDS)
        drivePID(0, -90, 30, 50)
        Intake.spin(REVERSE)
        wait(0.5, SECONDS)
        Intake.spin(FORWARD)
        drivePID(0, 0, 30, 50)
        wait(0.5, SECONDS)
        drivePID(32, 0, 50, 40)
        wait(1, SECONDS)
    
    Solenoid.set(True)
    if (alliance == 2 and   side == -1) or (alliance == 1 and  side == 1):
        turn = -90
    elif (alliance == 2 and side == 1) or (alliance == 1 and side == -1):
        turn = 90    
    
    drivePID(0, turn, 50, 50)
    drivePID(-20, turn, 35, 50, 10)
    Solenoid.set(False)
    wait(1, SECONDS)
    Conveyor.spin(FORWARD, 70, PERCENT)
    Intake.spin(FORWARD, 100, PERCENT)

def R2press():
    global stop
    stop = 0
    while controller_1.buttonR2.pressing():
        controller_1.screen.clear_screen()
        controller_1.screen.set_cursor(1,1)
        controller_1.screen.print(stop)
        if conveyorOpt.color() != plateOpt.color():
            Conveyor.set_velocity(70, PERCENT)
            Intake.set_velocity(100, PERCENT)
            Conveyor.spin(FORWARD)
            Intake.spin(FORWARD)
              
        if conveyorOpt.color() == plateOpt.color():
            if stop == 0:
                eject()
                stop = 1
            

    while not controller_1.buttonR2.pressing():
        Intake.stop()
        Conveyor.stop() 
        break

def eject():
    Conveyor.reset_position()
    Conveyor.spin(FORWARD, 70, PERCENT)
    while not Conveyor.position(RotationUnits.DEG) > 420:
        wait(1, MSEC)
    Conveyor.stop()    
    wait(500, MSEC) 
    R2press()

def R1press():
    while controller_1.buttonR1.pressing():
        Intake.set_velocity(-100, PERCENT)
        Conveyor.set_velocity(-70, PERCENT)
        Intake.spin(FORWARD)
        Conveyor.spin(FORWARD)
    while not controller_1.buttonR1.pressing():
        Intake.stop()
        Conveyor.stop()  
        break

def L2press():
    global piston
    if piston == 0:
        Solenoid.set(True)
        piston = 1
    elif piston == 1:
        Solenoid.set(False)
        piston = 0

def upPress():
    while controller_1.buttonUp.pressing():
        doinker.set_velocity(30, PERCENT)
        doinker.spin(FORWARD)
    while not controller_1.buttonUp.pressing():
        doinker.stop()
        break

def downPress():
    while controller_1.buttonDown.pressing():
        doinker.set_velocity(-30, PERCENT)
        doinker.spin(FORWARD)
    while not controller_1.buttonDown.pressing():
        doinker.stop()
        break

def preAuton():
    conveyorOpt.set_light(LedStateType.ON)
    conveyorOpt.set_light_power(100, PERCENT)
    plateOpt.set_light(LedStateType.ON)
    plateOpt.set_light_power(100, PERCENT)
    conveyorOpt.integration_time(20)
    brain.screen.set_cursor(1,1)
    brain.screen.clear_line(1)
    inertial_12.calibrate()
    while inertial_12.is_calibrating():
        brain.screen.print(".")
        wait(500, MSEC)
    calibrateTest1 = inertial_12.rotation(DEGREES)
    wait(100, MSEC)
    conveyorOpt.set_light(LedStateType.ON)
    calibrateTest2 = inertial_12.rotation(DEGREES)
    if abs(calibrateTest1 - calibrateTest2) > 0.3:
        brain.screen.set_cursor(2, 1)
        brain.screen.clear_line(2)
        brain.screen.set_pen_color(Color.RED)
        brain.screen.print("CALIBRATE FAILED")
        wait(500, MSEC)
        preAuton()
    else:
        brain.screen.set_cursor(2, 1)
        brain.screen.clear_line(2)
        brain.screen.set_pen_color(Color.GREEN)
        brain.screen.print("CALIBRATE PASSED")
        return True

#run the calibration function on program start
#nothing else works until it's complete
calibrate = preAuton()

competition = Competition(driver_control, autonomous)

controller_1.buttonA.pressed(sensors)

controller_1.buttonR2.pressed(R2press)
    
controller_1.buttonR1.pressed(R1press)
    
controller_1.buttonL2.pressed(L2press)

controller_1.buttonUp.pressed(upPress)

controller_1.buttonDown.pressed(downPress)
