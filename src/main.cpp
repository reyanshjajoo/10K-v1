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
                            &imu
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

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
    Right,
    Skills,
};

std::unordered_map<int, Auton> createAutonMap()
{
    return {
        {1, Auton::AWP},
        {2, Auton::Left},
        {3, Auton::Right},
        {4, Auton::Skills},
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
    case Auton::Right:
        return "Right";
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
ColorSortMode colorSortMode = ColorSortMode::Blue; 
const HueRange RED_RANGE{0.0, 26.0};
const HueRange BLUE_RANGE{200.0, 250.0};
BallColor ballColor = BallColor::Unknown;
Mode currentMode = Mode::Idle;

int autonCount = 4;
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
    if (autonCount == 5)
    {
        autonCount = 1;
    }
}

void on_right_button()
{
    autonCount -= 1;
    if (autonCount == 0)
    {
        autonCount = 4;
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
    // left, 2 mid goal + 5 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    chassis.setPose(-48, 16, 0);
    // move to high goal
    chassis.moveToPoint(-48, 47, 1000, {.minSpeed = 20}, false);
    chassis.turnToPoint(-23, 47, 700);
    chassis.moveToPoint(-23, 47, 1000, {.minSpeed = 40}, false);
    currentMode = Mode::ScoreTop;
    pros::delay(1000);
    currentMode = Mode::IntakeToBasket;
    // back up
    chassis.moveToPoint(-40, 46, 1000, {.forwards = false}, false);
    // go to three ball
    chassis.turnToPoint(-20, 22, 700, {}, false);
    chassis.moveToPoint(-20, 22, 1900, {}, false);
    pros::delay(300);
    // middle goal
    matchload.set_value(true);
    chassis.moveToPose(-6.5, 5.5, 135, 1500, {}, false);
    currentMode = Mode::ScoreMid;
    pros::delay(500);
    currentMode = Mode::ScoreMidAuton;
    pros::delay(600);
    // stop scoring and back out
    currentMode = Mode::Idle;
    // move back to last position
    chassis.moveToPoint(-22, 22, 2000, {.forwards = false, .minSpeed = 40}, false);
    matchload.set_value(false);
    // next three ball
    currentMode = Mode::IntakeToBasket;
    chassis.turnToPoint(-19, -23, 800, {}, false);
    chassis.moveToPoint(-19, -23, 2000, {.minSpeed = 10, .earlyExitRange = 4}, false);
    // pros::delay(500);
    // matchload.set_value(true);
    // //matchload
    // chassis.moveToPose(-46,-50, 270, 2000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=100, .earlyExitRange=6}, false);
    // chassis.moveToPoint(-68,-50, 800, {.minSpeed=70}, false);
    // pros::delay(380);
    chassis.moveToPoint(-40, -51, 1000, {.forwards = false, .minSpeed = 40}, false);
    // matchload.set_value(false);
    chassis.turnToHeading(90, 700);
    // scores
    chassis.moveToPoint(-19, -51, 750, {.minSpeed = 40}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    currentMode = Mode::ScoreTop;
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
    pros::delay(1200);

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, 26, 800, {.forwards = false}, false);

    // line up with matchload
    chassis.moveToPose(-47, 42.5, 270, 1400, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    // chassis.moveToPose(-64.5, 42.5, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    chassis.moveToPose(-64.5, 42.5, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(600);
    // currentMode = Mode::Idle;
    chassis.moveToPoint(-42, 42.5, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);

    // scores
    chassis.moveToPoint(-22, 40.5, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(2000);
    currentMode = Mode::Idle;
    leftMotors.move_velocity(-300);
    rightMotors.move_velocity(-300);
    pros::delay(500);
    chassis.turnToHeading(270, 1000);
    matchloadOut = true;
    matchload.set_value(true);
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
    chassis.moveToPoint(-24, -20, 1000, {.minSpeed = 20}, false);
    // disrupt
    matchload.set_value(true);
    chassis.moveToPoint(-5, -36, 1300, {.maxSpeed = 70}, false);
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
    //pros::delay(500);
    chassis.turnToHeading(90, 1000);
    // scores
    chassis.moveToPoint(-20, -44.5, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.move_velocity(40);
    rightMotors.move_velocity(40);
    currentMode = Mode::ScoreTop;
    pros::delay(2000);
    currentMode = Mode::Idle;
    // done scoring use horn
    //  chassis.moveToPoint(-24, -55,1000,{.maxSpeed=70,.minSpeed=20, .earlyExitRange = 1}, false);

    /*
        chassis.moveToPoint(-44, -44.5, 1000, {.forwards = false, .minSpeed = 20}, false);
    chassis.moveToPose(-18, -33.6, 97, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    
    horn.set_value(false);
    //pros::delay(300);
    chassis.moveToPose(-7, -34, 90, 1000, {.horizontalDrift = 8, .lead = 0.3, .minSpeed = 20}, false);
    
    */
    chassis.moveToPoint(-44, -44.5, 1000, {.forwards = false, .minSpeed = 40}, false);
    chassis.moveToPose(-18, -34, 95, 1750, {.horizontalDrift = 8, .lead = 0.3}, false);
    
    horn.set_value(false);
    //pros::delay(300);
    chassis.moveToPose(-4, -34, 90, 3000, {.horizontalDrift = 8, .lead = 0.3, .minSpeed = 40}, false);
    
}

void skills()
{
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    colorSortMode = ColorSortMode::Off;
    horn.set_value(true);

    chassis.setPose(-48, -13, 90);
    // start intake
    currentMode = Mode::IntakeToBasket;
    // move to 3 block stack
    chassis.moveToPose(-16, -23, 100, 2000, {.maxSpeed = 80}, false);
    pros::delay(500);
    // line up with matchload
    matchload.set_value(true);
    chassis.moveToPose(-47, -45, 270, 1200, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 80}, false);
    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, -45, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    // stop matchload
    pros::delay(1250);
    chassis.moveToPoint(-50, -44, 1000, {.forwards = false, .maxSpeed = 80}, false);
    matchload.set_value(false);
    chassis.turnToHeading(90, 1000);
    // scores
    chassis.moveToPoint(-24, -44, 1000, {.maxSpeed = 80}, false);
    leftMotors.move_velocity(20);
    rightMotors.move_velocity(20);
    currentMode = Mode::ScoreTop;
    pros::delay(4500);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(-52, -44, 1000, {.forwards = false, .maxSpeed = 80}, false);
    chassis.turnToPoint(-52, -57, 500);
    chassis.moveToPoint(-52, -57, 3000, {.maxSpeed = 80}, false);
    chassis.moveToPose(40, -57, 90, 3000, {.maxSpeed = 80}, false);

    matchload.set_value(true);
    chassis.moveToPose(47, -41, 90, 1200, {.horizontalDrift = 8, .lead = 0.3}, false);
    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(64.5, -41, 90, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    // stop matchload
    pros::delay(1250);
    chassis.moveToPoint(50, -43.5, 1000, {.forwards = false}, false);
    matchload.set_value(false);
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(20, -43.5, 1000, {.maxSpeed = 80}, false);
    leftMotors.move_velocity(20);
    rightMotors.move_velocity(20);
    //Score
    currentMode = Mode::ScoreTop;
    pros::delay(2500);
    currentMode = Mode::IntakeToBasket;

    chassis.moveToPoint(46, -43, 1000, {.forwards = false, .maxSpeed = 80}, false);
    // chassis.moveToPose(20, -14,315,1000, {.maxSpeed=80}, false);
    chassis.turnToPoint(21, -19, 800, {}, false);
    chassis.moveToPoint(20, -19, 2000, {.maxSpeed = 80}, false);
    chassis.turnToHeading(270, 800);
    chassis.moveToPoint(-20, -22, 2000, {.maxSpeed = 80}, false);
    chassis.turnToPoint(-20, 22, 800);
    // chassis.moveToPose(-16 ,24, 0, 2000, {.maxSpeed=80}, false);
    chassis.moveToPoint(-20, 22, 2000, {.maxSpeed = 80}, false);
    chassis.turnToPoint(-47, 53, 800);
    chassis.moveToPose(-47, 53, 270, 3000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 80}, true);
    pros::delay(500);
    matchload.set_value(true);
    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, 53, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);
    // stop matchload
    pros::delay(1250);
    chassis.moveToPoint(-50, 53, 1000, {.forwards = false, .maxSpeed = 80}, false);
    matchload.set_value(false);
    pros::delay(1000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-20, 53, 2000, {.maxSpeed = 80}, false);
    currentMode = Mode::ScoreTop;
    pros::delay(4500);
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPoint(-40, 53, 1000, {.forwards = false, .maxSpeed = 80}, false);
    chassis.moveToPoint(-40, 0, 1000, {.forwards = false, .maxSpeed = 80}, false);
    chassis.turnToPoint(-58, 20, 1000);
    chassis.moveToPoint(-58, 20, 3000, {.forwards = false, .minSpeed = 280}, false);
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
    // horn.set_value(hornDescore);
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
