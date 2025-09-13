#include "main.h"
#include "lemlib/api.hpp"

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

pros::Rotation vertical_encoder(-6);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2, 0.25);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

pros::Optical optical(7);
pros::Optical optical2(9);

pros::ADIDigitalOut basket('A');
bool basketExtended = false;

pros::ADIDigitalOut matchload('B');
bool matchloadOut = false;

pros::ADIDigitalOut scooper('H');
bool scooperDescore = true;


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
    Right,
    LeftTallGoal,
    RightTallGoal,
    LeftMatchload,
    RightMatchload,
    Skills
};

std::unordered_map<int, Auton> createAutonMap()
{
    return {
        {1, Auton::AWP},
        {2, Auton::Left},
        {3, Auton::Right},
        {4, Auton::LeftTallGoal},
        {5, Auton::RightTallGoal},
        {6, Auton::LeftMatchload},
        {7, Auton::RightMatchload},
        {8, Auton::Skills}};
}

const char *autonToString(Auton auton)
{
    switch (auton)
    {
    case Auton::AWP:
        return "AWP";
    case Auton::Left:
        return "Left";
    case Auton::Right:
        return "Right";
    case Auton::LeftTallGoal:
        return "Left Tall Goal";
    case Auton::RightTallGoal:
        return "Right Tall Goal";
    case Auton::LeftMatchload:
        return "Left Matchload";
    case Auton::RightMatchload:
        return "Right Matchload";
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

DriveMode driveMode = DriveMode::Tank;
ColorSortMode colorSortMode = ColorSortMode::Red;
const HueRange RED_RANGE{0.0, 26.0};
const HueRange BLUE_RANGE{200.0, 250.0};
BallColor ballColor = BallColor::Unknown;
Mode currentMode = Mode::Idle;

int autonCount = 2;
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

void handleBPress()
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
    if (autonCount == 9)
    {
        autonCount = 1;
    }
}

void on_right_button()
{
    autonCount -= 1;
    if (autonCount == 0)
    {
        autonCount = 8;
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
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        handleBPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        handleRightPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        handleLeftPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        handleAPress();
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    //     handleUpPress();
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
            if (cycle){
                basketRoller.move_velocity(200);
            } else {
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
            hood.move_velocity(-300);
            basketExtended = true;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::ScoreMidAuton:
            basketRoller.move_velocity(600);
            firstStageIntake.move_velocity(100);
            hood.move_velocity(-250);
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

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            scooperDescore = !scooperDescore;
            scooper.set_value(scooperDescore);
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
            sortText = "Color Sort: KEEP RED";
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
    
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    
    chassis.setPose(-48, -13, 90);
    // start intake
    currentMode = Mode::BottomLoad;
    // move to 3 block stack
    chassis.moveToPoint(-24, -20, 1000, {.minSpeed = 20, .earlyExitRange = 4}, false);
    //chassis.turnToPoint(-13, -9.5,500);
    //chassis.moveToPose(-11, -8.5, 45, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    chassis.moveToPose(-13, -10.5, 46, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);// was 12
    currentMode = Mode::ScoreLow;
    pros::delay(1000); //was 1650
    // stop scoring and back out
    //currentMode = Mode::BottomLoad
    currentMode = Mode::BottomLoad;
    chassis.moveToPoint(-23,-22,1000, {.forwards = false}, false);
    //currentMode = Mode::IntakeToBasket;
    chassis.turnToPoint(-22,28.5,1000, {.minSpeed = 20});
    chassis.moveToPoint(-22,28.5 ,1400, {.minSpeed = 20}, false); //go to other 3 ball
    pros::delay(150);
    matchload.set_value(true);
    chassis.moveToPose(-3, 12,135,1000, {.horizontalDrift = 8, .lead = 0.3, .minSpeed = 20}, false);//was 10m 
    //score middle
    currentMode = Mode::ScoreMidAuton;
    pros::delay(1300);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(-42,50.5,1000, {.forwards = false});
    chassis.turnToPoint(-72,50.5,1000);
    //set up matchload
    chassis.moveToPoint(-63.5,50.5, 1000);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    pros::delay(1000);
    //do scoring
    chassis.moveToPoint(-42,50.5,1000,{.forwards = false});
    pros::delay(100);
    matchload.set_value(false);
    chassis.turnToHeading(90,700);
    chassis.moveToPoint(-20,50.5,600, {.maxSpeed = 90, .minSpeed=40}, false);
    currentMode = Mode::ScoreTop;
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    
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
    chassis.moveToPose(-9, 7.5, 135, 1000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 65, .minSpeed = 15}, false);

    currentMode = Mode::ScoreMid;
    pros::delay(500);
    currentMode = Mode::ScoreMidAuton;
    pros::delay(1000);

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, 26, 800, {.forwards = false}, false);

    // line up with matchload
    chassis.moveToPose(-47, 42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, 42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(600);
    // currentMode = Mode::Idle;
    chassis.moveToPoint(-50, 43, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-19, 43, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(2700);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-200);
    rightMotors.move_velocity(-200);
    pros::delay(500);
    leftMotors.move_velocity(600);
    rightMotors.move_velocity(600);
}

void right()
{
    // right, 2 mid goal + 5 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    chassis.setPose(-48, -13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, -20, 1000, {.maxSpeed = 70}, false);
    chassis.moveToPose(-12, -9.5, 46, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);//NEW

    currentMode = Mode::ScoreLow;
    pros::delay(1320);

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, -26, 800, {.forwards = false}, false);

    // line up with matchload
    matchload.set_value(true);
    chassis.moveToPose(-50, -42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, -42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(650);
    chassis.moveToPoint(-50, -42, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-20, -42, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(3500);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-200);
    rightMotors.move_velocity(-200);
    pros::delay(500);
    leftMotors.move_velocity(600);
    rightMotors.move_velocity(600);
}

void leftTallGoal()
{
    // leftTallGoal, 7 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    chassis.setPose(-48, 13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, 20, 1000, {.maxSpeed = 70, .earlyExitRange = 4}, false);
    matchload.set_value(true);
    pros::delay(800);
    chassis.moveToPose(-9, 7.5, 135, 1000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 65, .minSpeed = 15}, false);

    pros::delay(500);

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, 26, 800, {.forwards = false}, false);

    // line up with matchload
    chassis.moveToPose(-47, 42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, 42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(600);
    // currentMode = Mode::Idle;
    chassis.moveToPoint(-50, 43, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-19, 43, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(2700);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-200);
    rightMotors.move_velocity(-200);
    pros::delay(500);
    leftMotors.move_velocity(600);
    rightMotors.move_velocity(600);
}

void rightTallGoal()
{
    // rightTallGoal, 7 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    chassis.setPose(-48, -13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, -20, 1000, {.maxSpeed = 70}, false);
    chassis.moveToPose(-12, -9.5, 46, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);//NEW

    pros::delay(300);

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, -26, 800, {.forwards = false}, false);

    // line up with matchload
    matchload.set_value(true);
    chassis.moveToPose(-50, -42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, -42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(650);
    chassis.moveToPoint(-50, -42, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-20, -42, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(3500);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-200);
    rightMotors.move_velocity(-200);
    pros::delay(500);
    leftMotors.move_velocity(600);
    rightMotors.move_velocity(600);
}


void rightMatchload()
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
    chassis.moveToPose(-9, 7.5, 135, 1000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 65, .minSpeed = 15}, false);

    currentMode = Mode::ScoreMid;
    pros::delay(500);
    currentMode = Mode::ScoreMidAuton;
    pros::delay(1000);

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, 26, 800, {.forwards = false}, false);

    // line up with matchload
    chassis.moveToPose(-47, 42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, 42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(600);
    // currentMode = Mode::Idle;
    chassis.moveToPoint(-50, 43, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-19, 43, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(2700);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-200);
    rightMotors.move_velocity(-200);
    pros::delay(500);
    chassis.turnToHeading(270, 1000);
    matchloadOut = true;
    matchload.set_value(true);
}

void leftMatchload()
{
    // right, 2 mid goal + 5 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    chassis.setPose(-48, -13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, -20, 1000, {.maxSpeed = 70}, false);
    chassis.moveToPose(-12, -9.5, 46, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);//NEW

    currentMode = Mode::ScoreLow;
    pros::delay(1320);

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, -26, 800, {.forwards = false}, false);

    // line up with matchload
    matchload.set_value(true);
    chassis.moveToPose(-50, -42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, -42, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(650);
    chassis.moveToPoint(-50, -42, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-20, -42, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(3500);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-200);
    rightMotors.move_velocity(-200);
    pros::delay(500);
    chassis.turnToHeading(270, 1000);
    matchloadOut = true;
    matchload.set_value(true);
}

void skills()
{
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    colorSortMode = ColorSortMode::Off;
    scooper.set_value(true);   

    // chassis.setPose(-49,-15,90);
    // chassis.moveToPoint(-49,-45,1000, {.maxSpeed = 70}, false);
    // chassis.turnToPoint(-60,-45,1000);//or turn to heading 270 if not accurate
    // //aligned with matchload
    // currentMode = Mode::IntakeToBasket;
    // matchload.set_value(true);
    // chassis.moveToPoint(-62,-47,600, {.maxSpeed = 70}, false); 
    // leftMotors.move_velocity(300);
    // rightMotors.move_velocity(300);
    // pros::delay(1800);
    // chassis.moveToPoint(-47,-46,1000, {.forwards=false, .maxSpeed=70}, false);
    // pros::delay(800);
    // matchload.set_value(false);
    // chassis.turnToPoint(-16 ,-19.5,1000);
    // chassis.moveToPose(-16 ,-19.5, 75, 2000, {.maxSpeed=30}, false);

    chassis.setPose(-48, -13, 90);
    // start intake
    currentMode = Mode::IntakeToBasket;
    // move to 3 block stack
    chassis.moveToPose(-16 ,-24, 115, 2000, {.maxSpeed=80}, false);
    pros::delay(500);
    // line up with matchload
    matchload.set_value(true);
    chassis.moveToPose(-47, -45, 270, 1200, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=80}, false);
    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, -45, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    // stop matchload
    pros::delay(1250);
    chassis.moveToPoint(-50, -44, 1000, {.forwards = false, .maxSpeed=80}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);
    // scores
    chassis.moveToPoint(-24, -44, 1000, {.maxSpeed=80}, false);
    leftMotors.move_velocity(20);
    rightMotors.move_velocity(20);
    currentMode = Mode::ScoreTop;
    pros::delay(4500);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(-52, -44, 1000, {.forwards=false, .maxSpeed=80}, false);
    chassis.turnToPoint(-52,-57,500);
    chassis.moveToPoint(-52, -57, 3000, {.maxSpeed=80}, false);
    chassis.moveToPose(40, -57, 90, 3000, {.maxSpeed=80}, false);
    
    matchload.set_value(true);
    chassis.moveToPose(47, -42, 90, 1200, {.horizontalDrift = 8, .lead = 0.3}, false);
    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(64.5, -42, 90, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    // stop matchload
    pros::delay(1250);
    chassis.moveToPoint(50, -43, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(20, -43, 1000, {.maxSpeed=80}, false);
    leftMotors.move_velocity(20);
    rightMotors.move_velocity(20);
    currentMode = Mode::ScoreTop;
    pros::delay(2500);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(46,-43, 1000, {.forwards = false, .maxSpeed=80}, false);
    //chassis.moveToPose(20, -14,315,1000, {.maxSpeed=80}, false);
    chassis.turnToPoint(20,-20,1000, {}, false);
    chassis.moveToPoint(20,-20,2000, {.maxSpeed=80, .earlyExitRange=6}, false);
    chassis.turnToPoint(-20 ,-24, 1000);
    chassis.moveToPoint(-20,-24,2000, {.maxSpeed=80, .earlyExitRange=6}, false);
    //chassis.moveToPose(-16 ,24, 0, 2000, {.maxSpeed=80}, false);
    chassis.turnToPoint(-20,24,1000, {}, false);
    chassis.moveToPoint(-20,24,2000, {.maxSpeed=80, .earlyExitRange=6}, false);
    pros::delay(1000);
    chassis.turnToPoint(-48,52,1000);
    chassis.moveToPoint(-48,52, 2000, {.maxSpeed=80}, false);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-20,52, 2000, {.maxSpeed=80}, false);

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
    case Auton::Right:
        right();
        break;
    case Auton::LeftTallGoal:
        leftTallGoal();
        break;
    case Auton::RightTallGoal:
        rightTallGoal();
        break;
    case Auton::LeftMatchload:
        leftMatchload();
        break;
    case Auton::RightMatchload:
        rightMatchload();
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
    scooperDescore = true;
    scooper.set_value(scooperDescore);
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
