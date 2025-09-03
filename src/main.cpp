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

pros::Rotation vertical_encoder(8);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2, -0.25);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

enum class DriveMode { Tank, Arcade };
DriveMode driveMode = DriveMode::Tank;

enum class BallColor
{
    Red,
    Blue,
    Unknown
};

bool colorSort = true;

pros::Optical optical(7);
pros::Optical optical2(9);
BallColor targetColor = BallColor::Blue;

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

const HueRange RED_RANGE{0.0, 26.0};
const HueRange BLUE_RANGE{200.0, 250.0};
BallColor ballColor = BallColor::Unknown; // initially

BallColor identifyColor()
{
    const int PROX_THRESHOLD = 60;

    int prox1 = optical.get_proximity();
    int prox2 = optical2.get_proximity();

    if (prox1 < PROX_THRESHOLD && prox2 < PROX_THRESHOLD) 
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

void colorSortTask()
{
    while (true)
    {
        ballColor = identifyColor();
        pros::delay(20);
    }
}

pros::ADIDigitalOut basket('A');
bool basketExtended = true;

pros::ADIDigitalOut matchload('B');
bool matchloadOut = false;

enum class Mode
{
    Idle,
    IntakeToBasket,
    ScoreTop,
    ScoreMid,
    ScoreMidAuton,
    ScoreLow,
    Unjam,
    BottomLoad,
    ejectBall
};

enum class Auton
{
    AWP,
    Left,
    Right,
    LeftElim,
    RightElim,
    Skills
};

std::unordered_map<int, Auton> createAutonMap()
{
    return {
        {1, Auton::AWP},
        {2, Auton::Left},
        {3, Auton::Right},
        {4, Auton::LeftElim},
        {5, Auton::RightElim},
        {6, Auton::Skills}};
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
    case Auton::LeftElim:
        return "LeftElim";
    case Auton::RightElim:
        return "RightElim";
    case Auton::Skills:
        return "Skills";
    default:
        return "Unknown";
    }
}

std::unordered_map<int, Auton> autonMap = createAutonMap();

Mode currentMode = Mode::Idle;

void handleL1Press()
{
    if (ballColor == targetColor)
    {
        currentMode = Mode::ejectBall;
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

void handleYPress() {                             
    driveMode = (driveMode == DriveMode::Tank) ? DriveMode::Arcade : DriveMode::Tank;
    controller.rumble(".");
}


void handleLeftPress()
{
    colorSort = !colorSort;
    controller.rumble(".");
}

void handleL2Held(bool pressed)
{
    if (pressed)
        currentMode = Mode::Unjam;
    else if (currentMode == Mode::Unjam)
        currentMode = Mode::Idle;
}

int autonCount = 2;

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
    if (targetColor == BallColor::Red)
    {
        targetColor = BallColor::Blue;
    }
    else
    {
        targetColor = BallColor::Red;
    }
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
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        handleYPress();
    handleL2Held(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
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
            if (ballColor == targetColor)
            {
                currentMode = Mode::ejectBall;
                break;
            }
            firstStageIntake.move_velocity(600);
            hood.move_velocity(600);
            basketRoller.move_velocity(150 * sin(pros::millis() / 50));
            basketExtended = true;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::ScoreTop:
            firstStageIntake.move_velocity(600);
            basketRoller.move_velocity(600);
            hood.move_velocity(600);
            pros::delay(250);
            basketExtended = false;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::ScoreMid:
            basketRoller.move_velocity(600);
            firstStageIntake.move_velocity(200);
            hood.move_velocity(-300);
            basketExtended = false;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::ScoreMidAuton:
            basketRoller.brake();
            firstStageIntake.move_velocity(600);
            hood.move_velocity(-600);
            basketExtended = false;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::ScoreLow:
            basketRoller.move_velocity(600);
            firstStageIntake.move_velocity(-400);
            hood.move_velocity(0);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::Unjam:
            basketRoller.move_velocity(-600);
            firstStageIntake.move_velocity(-600);
            hood.move_velocity(-600);
            basketExtended = true;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::BottomLoad:
            firstStageIntake.move_velocity(600);
            hood.move_velocity(0);
            basketRoller.move_velocity(-600);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::ejectBall:
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
        pros::delay(20);
    }
}

void displayStatusTask()
{
    while (true)
    {
        controller.set_text(0, 0, colorSort ? "Color Sort: ON " : "Color Sort: OFF");
        pros::delay(50);
        controller.set_text(1, 0, (driveMode == DriveMode::Tank) ? "Drive: TANK   " : "Drive: ARCADE "); 
        pros::delay(50);
    }
}

void AWP()
{
    // AWP code
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000);
}

void left()
{
    // leftSafe, 3 mid goal + 4 top goal
    pros::Task intake_task(intakeControl);
    pros::Task color_task(colorSortTask);
    chassis.setPose(-48, 13, 90);

    // start intake
    currentMode = Mode::BottomLoad;

    // move to 3 block stack
    chassis.moveToPoint(-24, 20, 1000, {.maxSpeed = 70, .earlyExitRange = 4}, false);
    // // move to 2 ball stack
    // chassis.moveToPose(-8, 38, 0, 2000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=90, .earlyExitRange=2}, false);
    // currentMode = Mode::Idle;
    // chassis.moveToPoint(-8, 44, 800, {});
    // chassis.moveToPoint(-24, 20, 1000, {.forwards=false});
    // currentMode = Mode::IntakeToBasket;
    // move towards goal
    matchload.set_value(true);
    chassis.moveToPose(-9, 7.5, 135, 1000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 65, .minSpeed = 15}, false);

    currentMode = Mode::ScoreMidAuton;
    pros::delay(900);

    // stop scoring and back out
    currentMode = Mode::Idle;
    pros::delay(500);
    chassis.moveToPoint(-33, 26, 1000, {.forwards = false, .earlyExitRange = 15}, false);

    // line up with matchload
    chassis.moveToPose(-47, 41.2, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);

    // start matchload and drive into matchload
    currentMode = Mode::IntakeToBasket;
    chassis.moveToPose(-64.5, 43, 270, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    leftMotors.move_velocity(300);
    rightMotors.move_velocity(300);

    // stop matchload
    pros::delay(3000);
    matchload.set_value(false);
    chassis.moveToPoint(-50, 45, 1000, {.forwards = false, .earlyExitRange = 2}, false);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-20, 44, 1000, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    currentMode = Mode::ScoreTop;
}

void right(){
    chassis.setPose({0, 0, 0});
    chassis.moveToPoint(24, 24, 3000);
}

void leftElim()
{
    chassis.setPose({0, 0, 0});
    chassis.moveToPoint(24, 24, 3000);
}

void rightElim()
{
    chassis.setPose({0, 0, 0});
    chassis.moveToPoint(-24, -24, 3000);
}

void skills()
{
    chassis.setPose({0, 0, 0});
    chassis.moveToPoint(0, -24, 3000);
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
            pros::lcd::print(6, targetColor == BallColor::Red ? "blue color" : "red color");
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
    case Auton::LeftElim:
        leftElim();
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
    pros::Task intake_task(intakeControl);
    pros::Task toggle_task(toggleTask);
    pros::Task pneumatic_task(pneumaticControl);
    pros::Task color_task(colorSortTask);
    pros::Task displayTask(displayStatusTask);

    while (true)
    {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int rightX  = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

         if (driveMode == DriveMode::Tank) {
            chassis.tank(leftY, rightY);
        } else {
            chassis.arcade(leftY, rightX);
        }

        pros::delay(10);
    }
}
