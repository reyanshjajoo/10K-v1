#include "main.h"
#include "lemlib/api.hpp"
//
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue);

pros::Motor firstStageIntake(-17, pros::MotorGearset::blue);
pros::Motor basketRoller(2, pros::MotorGearset::blue);
pros::Motor hood(10, pros::MotorGearset::blue);

pros::Imu imu(14);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.2, // track width
                              lemlib::Omniwheel::NEW_275,
                              450,
                              2 // horizontal drift
);

lemlib::ControllerSettings linearController(5.05, // kP
                                            0,    // kI
                                            2,    // kD
                                            3,    // anti windup
                                            1,    // small error range, in inches
                                            100,  // small error range timeout, in milliseconds
                                            3,    // large error range, in inches
                                            500,  // large error range timeout, in milliseconds
                                            20    // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(1.09, // kP
                                             0,    // kI
                                             6,    // kD
                                             0,    // anti windup
                                             1,    // small error range, in degrees
                                             100,  // small error range timeout, in milliseconds
                                             3,    // large error range, in degrees
                                             500,  // large error range timeout, in milliseconds
                                             0     // maximum acceleration (slew)
);


lemlib::ControllerSettings angularControllerAWP(1.19, // kP
                                             0,    // kI
                                             6,    // kD
                                             0,    // anti windup
                                             1,    // small error range, in degrees
                                             100,  // small error range timeout, in milliseconds
                                             3,    // large error range, in degrees
                                             500,  // large error range timeout, in milliseconds
                                             0     // maximum acceleration (slew)
);

lemlib::ControllerSettings linearControllerAWP(5.2, // kP
                                            0,    // kI
                                            2,    // kD
                                            3,    // anti windup
                                            1,    // small error range, in inches
                                            100,  // small error range timeout, in milliseconds
                                            3,    // large error range, in inches
                                            500,  // large error range timeout, in milliseconds
                                            20    // maximum acceleration (slew)
);
pros::Rotation vertical_encoder(-6);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2, 0.25);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);
//lemlib::Chassis chassis(drivetrain, linearControllerAWP, angularControllerAWP, sensors);
lemlib::Chassis chassisAWP(drivetrain, linearControllerAWP, angularControllerAWP, sensors);
pros::Optical optical(7);
pros::Optical optical2(9);

pros::ADIDigitalOut basket('A');
bool basketExtended = false;

pros::ADIDigitalOut matchload('B');
bool matchloadOut = false;

pros::ADIDigitalOut horn('H');
bool hornDescore = true;

enum class DriveMode
{
    Tank,
    Arcade
};

enum class ColorSortMode
{
    Blue,
    Red,
    Off
};

enum class BallColor
{
    Red,
    Blue,
    Unknown
};

enum class Mode
{
    Idle,
    IntakeToBasket,
    ScoreTop,
    CycleAuto,
    ScoreMid,
    ScoreMidAuton,
    ScoreLow,
    Unjam,
    BottomLoad,
    EjectBall
};

enum class Auton
{
    AWP,
    Left,
    LeftElim,
    Right,
    RightElim,
    Skills,
};

std::unordered_map<int, Auton> createAutonMap()
{
    return {
        {1, Auton::AWP},
        {2, Auton::Left},
        {3, Auton::LeftElim},
        {4, Auton::Right},
        {5, Auton::RightElim},
        {6, Auton::Skills},
    };
}

const char *autonToString(Auton auton)
{
    switch (auton)
    {
    case Auton::AWP:
        return "AWP";
    case Auton::Left:
        return "Left";
    case Auton::LeftElim:
        return "LeftElim";
    case Auton::Right:
        return "Right";
    case Auton::RightElim:
        return "RightElim";
    case Auton::Skills:
        return "Skills";
    default:
        return "Unknown";
    }
}

std::unordered_map<int, Auton> autonMap = createAutonMap();

struct HueRange
{
    double min;
    double max;
    bool contains(double hue) const
    {
        if (min < max)
        {
            return hue >= min && hue <= max;
        }
        else
        {
            return hue >= min || hue <= max;
        }
    }
};

DriveMode driveMode = DriveMode::Arcade;
ColorSortMode colorSortMode = ColorSortMode::Red;
const HueRange RED_RANGE{0.0, 26.0};
const HueRange BLUE_RANGE{200.0, 250.0};
BallColor ballColor = BallColor::Unknown;
Mode currentMode = Mode::Idle;

int autonCount = 1;
bool cycle = false;

BallColor identifyColor()
{
    const int PROX_THRESHOLD = 60;

    int prox1 = optical.get_proximity();
    int prox2 = optical2.get_proximity();

    if (prox1 < PROX_THRESHOLD && prox2 < PROX_THRESHOLD || colorSortMode == ColorSortMode::Off)
    {
        return BallColor::Unknown;
    }

    double hue1 = optical.get_hue();
    double hue2 = optical2.get_hue();

    if (RED_RANGE.contains(hue1) || RED_RANGE.contains(hue2))
    {
        return BallColor::Red;
    }
    if (BLUE_RANGE.contains(hue1) || BLUE_RANGE.contains(hue2))
    {
        return BallColor::Blue;
    }
    return BallColor::Unknown;
}

void moveChassis(int speed)
{
    leftMotors.move_velocity(speed);
    rightMotors.move_velocity(speed);
}

void handleL1Press()
{
    if ((colorSortMode == ColorSortMode::Blue && ballColor == BallColor::Red) ||
        (colorSortMode == ColorSortMode::Red && ballColor == BallColor::Blue))
    {
        currentMode = Mode::EjectBall;
    }
    else
    {
        if (currentMode == Mode::IntakeToBasket)
        {
            currentMode = Mode::Idle;
        }
        else
        {
            currentMode = Mode::IntakeToBasket;
        }
    }
}

void handleR1Press()
{
    currentMode = (currentMode == Mode::ScoreTop) ? Mode::Idle : Mode::ScoreTop;
}

void handleR2Press()
{
    currentMode = (currentMode == Mode::ScoreMid) ? Mode::Idle : Mode::ScoreMid;
}

void handleYPress()
{
    currentMode = (currentMode == Mode::ScoreLow) ? Mode::Idle : Mode::ScoreLow;
}

void handleRightPress()
{
    currentMode = (currentMode == Mode::BottomLoad) ? Mode::Idle : Mode::BottomLoad;
}

void handleUpPress()
{
    cycle = !cycle;
    controller.rumble(".");
}

void handleAPress()
{
    driveMode = (driveMode == DriveMode::Tank) ? DriveMode::Arcade : DriveMode::Tank;
    controller.rumble(".");
}

void handleLeftPress()
{
    switch (colorSortMode)
    {
    case ColorSortMode::Blue:
        colorSortMode = ColorSortMode::Red;
        break;
    case ColorSortMode::Red:
        colorSortMode = ColorSortMode::Off;
        break;
    case ColorSortMode::Off:
        colorSortMode = ColorSortMode::Blue;
        break;
    }
    controller.rumble(".");
}

void handleL2Held(bool pressed)
{
    if (pressed)
        currentMode = Mode::Unjam;
    else if (currentMode == Mode::Unjam)
        currentMode = Mode::Idle;
}

void on_center_button()
{
    autonCount += 1;
    if (autonCount == 7)
    {
        autonCount = 1;
    }
}

void on_right_button()
{
    autonCount -= 1;
    if (autonCount == 0)
    {
        autonCount = 6;
    }
}

void on_left_button()
{
    handleLeftPress();
}

void checkButtons()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        handleL1Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        handleR1Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        handleR2Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        handleYPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        handleRightPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        handleLeftPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        handleAPress();
    handleL2Held(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
}

void colorSortTask()
{
    while (true)
    {
        ballColor = identifyColor();
        pros::delay(20);
    }
}

void intakeControl()
{
    while (true)
    {
        switch (currentMode)
        {
        case Mode::Idle:
            firstStageIntake.move_velocity(0);
            hood.move_velocity(0);
            basketRoller.brake();
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::IntakeToBasket:
            if ((colorSortMode == ColorSortMode::Blue && ballColor == BallColor::Red) ||
                (colorSortMode == ColorSortMode::Red && ballColor == BallColor::Blue))
            {
                currentMode = Mode::EjectBall;
            }

            firstStageIntake.move_velocity(600);
            hood.move_velocity(600);
            if (cycle)
            {
                basketRoller.move_velocity(200);
            }
            else
            {
                basketRoller.move_velocity(150 * sin(pros::millis() / 50));
            }
            basketExtended = false;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::ScoreTop:
            firstStageIntake.move_velocity(600);
            basketRoller.move_velocity(600);
            hood.move_velocity(600);
            pros::delay(250);
            basketExtended = true;
            basket.set_value(basketExtended);
            break;

        case Mode::CycleAuto:
            firstStageIntake.move_velocity(300);
            basketRoller.move_velocity(100);
            hood.move_velocity(300);
            pros::delay(250);
            basketExtended = false;
            basket.set_value(basketExtended);
            break;

        case Mode::ScoreMid:
            basketRoller.move_velocity(600);
            firstStageIntake.move_velocity(200);
            hood.move_velocity(-600);
            basketExtended = true;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::ScoreMidAuton:
            basketRoller.move_velocity(600);
            firstStageIntake.move_velocity(100);
            hood.move_velocity(-600);
            basketExtended = true;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::ScoreLow:
            basketRoller.move_velocity(300);
            firstStageIntake.move_velocity(-200);
            hood.move_velocity(0);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::Unjam:
            basketRoller.move_velocity(-600);
            firstStageIntake.move_velocity(-600);
            hood.move_velocity(-600);
            basketExtended = false;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::BottomLoad:
            firstStageIntake.move_velocity(600);
            hood.move_velocity(0);
            basketRoller.move_velocity(-600);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::EjectBall:
            firstStageIntake.move_velocity(600);
            hood.move_velocity(-600);
            basketRoller.move_velocity(0);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            pros::delay(100);
            currentMode = Mode::IntakeToBasket;
            break;
        }

        pros::delay(20);
    }
}

void toggleTask()
{
    while (true)
    {
        checkButtons();
        pros::delay(20);
    }
}

void pneumaticControl()
{
    while (true)
    {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            matchloadOut = !matchloadOut;
            matchload.set_value(matchloadOut);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            hornDescore = !hornDescore;
            horn.set_value(hornDescore);
        }
        pros::delay(20);
    }
}

void displayStatusTask()
{
    while (true)
    {
        const char *sortText;
        switch (colorSortMode)
        {
        case ColorSortMode::Blue:
            sortText = "Color Sort: KEEP BLUE";
            break;
        case ColorSortMode::Red:
            sortText = "Color Sort: KEEP RED ";
            break;
        case ColorSortMode::Off:
            sortText = "Color Sort: OFF            ";
            break;
        }

        controller.set_text(0, 0, sortText);
        pros::delay(50);
        controller.set_text(1, 0, (driveMode == DriveMode::Tank) ? "Drive: TANK   " : "Drive: ARCADE ");
        pros::delay(50);
    }
}

void AWP()
{
    // left, 2 mid goal + 5 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    horn.set_value(true);
    chassisAWP.setPose(-48, 16, 0);
    // move to high goal
    chassisAWP.moveToPoint(-48, 45, 800, {}, false);
    matchload.set_value(true);
    chassisAWP.turnToHeading(270, 700, {}, false);
    currentMode = Mode::IntakeToBasket;
    chassisAWP.moveToPoint(-58, 45, 600, {.minSpeed = 30}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    // stop matchload
    pros::delay(200);
    chassisAWP.moveToPoint(-48, 47, 600, {.forwards = false, .minSpeed = 40}, false);
    matchload.set_value(false);
    chassisAWP.turnToPoint(-22, 47, 800);
    chassisAWP.moveToPoint(-22, 47, 800, {.minSpeed = 60}, false);
    currentMode = Mode::ScoreTop;
    moveChassis(40);
    pros::delay(700);
    currentMode = Mode::IntakeToBasket;
    // back up
    chassisAWP.moveToPoint(-40, 46, 600, {.forwards = false, .earlyExitRange = 2}, false);
    // go to three ball
    chassisAWP.turnToPoint(-23, 24, 600, {}, false);
    chassisAWP.moveToPoint(-23, 24, 800, {}, false);
    matchload.set_value(true);
    // middle goal
    pros::delay(100);
    chassisAWP.moveToPose(-2.5, 2.5, 135, 1500, {}, false);
    currentMode = Mode::ScoreMid;
    pros::delay(400);
    currentMode = Mode::ScoreMidAuton;
    pros::delay(500);
    // stop scoring and back out
    currentMode = Mode::Idle;
    // move back to last position
    chassisAWP.moveToPoint(-22, 24, 800, {.forwards = false, .minSpeed = 40}, false);
    matchload.set_value(false);
    // next three ball
    currentMode = Mode::IntakeToBasket;
    chassisAWP.turnToPoint(-24, -30, 800, {}, false);
    chassisAWP.moveToPoint(-24, -30, 1300, {.minSpeed = 20}, false);
    chassisAWP.moveToPoint(-46, -48, 1200, {.forwards = false}, false);
    //chassisAWP.turnToHeading(90, 800);
    // scores
    chassisAWP.turnToPoint(-19, -48, 800, {}, false);
    chassisAWP.moveToPoint(-19, -48, 1000, {}, false);
    chassisAWP.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    currentMode = Mode::ScoreTop;
    moveChassis(40);
}


void left()
{
    // left, 2 mid goal + 5 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    chassis.setPose(-48, 13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, 20, 1000, {.maxSpeed = 70, .earlyExitRange = 4}, false);
    matchload.set_value(true);
    pros::delay(800);
    chassis.moveToPose(-9, 5, 130, 1000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 65, .minSpeed = 15}, false);

    currentMode = Mode::ScoreMid;
    pros::delay(500);
    currentMode = Mode::ScoreMidAuton;
    pros::delay(1200);//added 300 

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, 26, 800, {.forwards = false}, false);

    // line up with matchload
    chassis.moveToPose(-47, 42.5, 270, 1400, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, 42.5, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(600);
    chassis.moveToPoint(-42, 42.5, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-22, 40.5, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    /*
    pros::delay(3500);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-300);
    rightMotors.move_velocity(-300);
    pros::delay(500);
    chassis.turnToHeading(270, 1000);
    matchloadOut = true;
    matchload.set_value(true);
    */
    
}

void leftElim() { ///calling this left for now shoudl be elim 
    // left, 2 mid goal + 5 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    chassis.setPose(-48, 13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, 20, 1000, {.maxSpeed = 70, .earlyExitRange = 4}, false);
    matchload.set_value(true);
    pros::delay(800);
    //chassis.moveToPose(-9, 5, 130, 1000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 65, .minSpeed = 15}, false);

    //currentMode = Mode::ScoreMid;
    //pros::delay(500);
    //currentMode = Mode::ScoreMidAuton;
    //pros::delay(1200);//added 300 

    // stop scoring and back out
    //currentMode = Mode::Idle;
    //pros::delay(500);
    //chassis.moveToPoint(-33, 26, 800, {.forwards = false}, false);

    // line up with matchload
    chassis.turnToPoint(-47,43,1000, {}, false);
    chassis.moveToPose(-47, 42.5, 270, 1400, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, 42.5, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(600);
    chassis.moveToPoint(-42, 42.5, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-22, 43, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    /*
    pros::delay(3500);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-300);
    rightMotors.move_velocity(-300);
    pros::delay(500);
    chassis.turnToHeading(270, 1000);
    matchloadOut = true;
    matchload.set_value(true);
    */
}

void right()
{
    // rightTallGoal, 7 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    horn.set_value(true);
    chassis.setPose(-48, -13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, -21, 1000, {.minSpeed = 20});
    // disrupt
    matchload.set_value(true);
    chassis.moveToPoint(-6, -36, 1300, {.maxSpeed = 70}, false);//inch back on x
    // back to 3 ball pose; should be same
    chassis.moveToPoint(-24, -20, 1000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 2}, false);
    matchload.set_value(true);
    chassis.turnToPoint(-50, -46, 1000, {.minSpeed = 50, .earlyExitRange = 3}, false);
    chassis.moveToPose(-50, -46, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, -46, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    // stop matchload
    pros::delay(340);
    chassis.moveToPoint(-48, -44.5, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);
    // scores
    chassis.moveToPoint(-20, -44.5, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(2700);
    currentMode = Mode::Idle;

    chassis.moveToPoint(-44, -44.5, 1000, {.forwards = false, .minSpeed = 40}, false);
    chassis.turnToHeading(270, 800, {}, false);
    matchloadOut = true;
    matchload.set_value(true);
}

void rightElim() {
    // rightTallGoal, 7 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    horn.set_value(true);
    chassis.setPose(-48, -13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, -21, 1000, {.minSpeed = 20});
    // disrupt
    matchload.set_value(true);
    chassis.moveToPoint(-6, -36, 1300, {.maxSpeed = 70}, false);//inch back on x
    // back to 3 ball pose; should be same
    chassis.moveToPoint(-24, -20, 1000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 2}, false);
    matchload.set_value(true);
    chassis.turnToPoint(-50, -46, 1000, {.minSpeed = 50, .earlyExitRange = 3}, false);
    chassis.moveToPose(-50, -46, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, -46, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    // stop matchload
    pros::delay(340);
    chassis.moveToPoint(-48, -44.5, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);
    // scores
    chassis.moveToPoint(-20, -44.5, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
}

void skills()
{
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    colorSortMode = ColorSortMode::Off;
    horn.set_value(true);
    // matchload 1
    chassis.setPose(-48, -18, 180);
    matchload.set_value(true);
    chassis.moveToPoint(-48, -46, 1000, {}, false);
    chassis.turnToPoint(-58, -47, 1000, {}, false);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(-59, -47, 2400, {.minSpeed=300}, false);
    moveChassis(300);
    pros::delay(600);
    // go back and score
    chassis.moveToPoint(-48, -48.5, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToPoint(-22, -47, 800, {}, false);
    chassis.moveToPoint(-22, -47, 1000, {}, false);
    currentMode = Mode::ScoreTop;
    moveChassis(40);
    pros::delay(3000);
    moveChassis(0);
    currentMode = Mode::IntakeToBasket;
    // go back and then to matchload 2
    chassis.moveToPoint(-40, -47, 1000, {.forwards = false}, false);
    chassis.turnToPoint(-40, -60.5, 800, {}, false);
    chassis.moveToPoint(-40, -60.5, 1000, {}, false);
    chassis.turnToPoint(42, -58.5, 800, {}, false);//changed alley half inch closer to origin
    chassis.moveToPoint(42, -58.5, 2000, {}, false);

    chassis.moveToPose(48,-46.8,90, 2000, {}, false);
    //chassis.turnToPoint(48, -46, 800, {}, false);
    //chassis.moveToPoint(48, -46, 1000, {}, false);

    // matchload 2
    matchload.set_value(true);
    /*
    chassis.turnToPoint(58, -46.8, 800, {}, false);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(64, -46.8, 2000, {.minSpeed=300}, false);
    */
    chassis.turnToPoint(58, -46, 800, {}, false);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(64, -46, 2000, {.minSpeed=300}, false);
    moveChassis(100);
    // go back and score
    chassis.moveToPoint(48, -46.5, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToPoint(22, -48, 800, {}, false);
    chassis.moveToPoint(22, -48, 1000, {}, false);
    currentMode = Mode::ScoreTop;
    moveChassis(40);
    pros::delay(3000);
    moveChassis(0);
    currentMode = Mode::IntakeToBasket;
    // go to both 4 stacks
    chassis.moveToPoint(48, -55, 1000, {.forwards = false}, false);
    //chassis.turnToPoint(24, -22, 800, {}, false);
    chassis.moveToPoint(24, -22, 1500, {}, false);
    chassis.turnToPoint(24, 22, 800, {}, false);
    chassis.moveToPoint(24, 12, 1000, {.earlyExitRange=10}, false);
    chassis.moveToPoint(24, 22, 1000, {.maxSpeed=80}, false);
    // go and score
    chassis.turnToPoint(45, 49, 800, {}, false);
    chassis.moveToPoint(45, 49, 2000, {}, false);
    chassis.turnToPoint(22, 49, 800, {}, false);
    chassis.moveToPoint(22, 49, 1000, {}, false);
    currentMode = Mode::ScoreTop;
    moveChassis(40);
    pros::delay(3000);
    moveChassis(0);
    // go back and to matchload 3
    chassis.moveToPoint(40, 49, 1000, {.forwards = false}, false);
    matchload.set_value(true);
    chassis.turnToPoint(64, 49, 800, {}, false);
    pros::delay(200);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(64, 49, 2000, {.minSpeed=300}, false);
    moveChassis(100);
    // go back and score
    chassis.moveToPoint(48, 48, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToPoint(22, 47, 800, {}, false);
    chassis.moveToPoint(22, 47, 1000, {}, false);
    currentMode = Mode::ScoreTop;
    moveChassis(40);
    pros::delay(3000);
    moveChassis(0);
    // go to 4 stack and mid goal
    chassis.moveToPoint(40, 47, 1000, {.forwards = false}, false);
    chassis.turnToPoint(40, 22, 800, {}, false);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(40, 22, 1000, {}, false);
    // chassis.turnToPoint(-22, 22, 800, {}, false);
    // chassis.moveToPoint(-22, 22, 2500, {}, false);
    // // chassis.turnToPoint(-7, 7, 800, {}, false);
    // // // score mid goal
   // matchload.set_value(true);
    // // pros::delay(800);
    // // chassis.moveToPose(-7, 7, 130, 1000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 65, .minSpeed = 15}, false);
    // // currentMode = Mode::ScoreMid;
    // // pros::delay(500);
    // // currentMode = Mode::ScoreMidAuton;
    // // pros::delay(1200);
    // // currentMode = Mode::Idle;
    // // pros::delay(500);
    // // go to matchload 4
    // chassis.moveToPoint(-30, 30, 800, {.forwards = false}, false);
    chassis.turnToPoint(-47, 22, 800, {}, false);
    chassis.moveToPoint(-47, 22, 2500, {}, false);
    chassis.moveToPoint(-47, 45, 1000, {.forwards = false}, false);
    // chassis.moveToPose(-47, 43.5, 270, 1400, {.horizontalDrift = 8, .lead = 0.3}, false);
    currentMode = Mode::IntakeToBasket;
    matchload.set_value(true);
    chassis.moveToPoint(-64, 45, 2300, {}, false);
    moveChassis(300);
    chassis.moveToPoint(-48, 45, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    pros::delay(600);
    // TODO: Park
    chassis.turnToPoint(-61, 29, 800, {.forwards=false}, false);
    chassis.moveToPoint(-61, 29, 1300, {.forwards=false}, false);
    chassis.turnToHeading(0, 800, {}, false);
    moveChassis(-600);
    pros::delay(1500);
    moveChassis(0);
}

void initialize()
{
    pros::lcd::initialize();
    pros::lcd::register_btn1_cb(on_center_button);
    pros::lcd::register_btn2_cb(on_left_button);
    pros::lcd::register_btn0_cb(on_right_button);
    controller.clear();
    optical.set_led_pwm(100);
    optical2.set_led_pwm(100);
    optical.set_integration_time(50);
    optical2.set_integration_time(50);
    chassis.calibrate();
    basketRoller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task screenTask([&]()
                          {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Hue1: %.2f  Hue2: %.2f", optical.get_hue(), optical2.get_hue());
            pros::lcd::print(4, "Prox1: %d  Prox2: %d", optical.get_proximity(), optical2.get_proximity());
            pros::lcd::print(5, "Auton: %s", autonToString(autonMap[autonCount]));
            pros::lcd::print(6, "Alliance Color: %s", 
                colorSortMode == ColorSortMode::Blue ? "Blue" :
                colorSortMode == ColorSortMode::Red ? "Red" : "Off"
            );
            pros::lcd::print(7, "Ball Color: %s", ballColor == BallColor::Red ? "Red" : (ballColor == BallColor::Blue ? "Blue" : "Unknown"));
			lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        } });
}

void disabled() {}
void competition_initialize() {}
void autonomous()
{
    auto it = autonMap.find(autonCount);
    Auton selected = (it != autonMap.end()) ? it->second : Auton::AWP;

    switch (selected)
    {
    case Auton::AWP:
        AWP();
        break;
    case Auton::Left:
        left();
        break;
    case Auton::LeftElim:
        leftElim();
        break;
    case Auton::Right:
        right();
        break;
    case Auton::RightElim:
        rightElim();
        break;
    case Auton::Skills:
        skills();
        break;
    default:
        AWP();
        break;
    }
}

void opcontrol()
{
    hornDescore = true;
    horn.set_value(hornDescore);
    currentMode = Mode::IntakeToBasket;
    pros::Task intake_task(intakeControl);
    pros::Task toggle_task(toggleTask);
    pros::Task pneumatic_task(pneumaticControl);
    pros::Task color_task(colorSortTask);
    pros::Task displayTask(displayStatusTask);

    while (true)
    {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (driveMode == DriveMode::Tank)
        {
            chassis.tank(leftY, rightY);
        }
        else
        {
            chassis.arcade(leftY, rightX);
        }

        pros::delay(10);
    }
}
