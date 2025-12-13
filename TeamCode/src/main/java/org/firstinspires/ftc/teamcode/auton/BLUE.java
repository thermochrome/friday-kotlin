package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name = "BLUE")
@SuppressWarnings("unused")
public class BLUE extends OpMode {

    private final Motor right = new Motor(hardwareMap, "outtake_r");
    private final Motor left = new Motor(hardwareMap, "outtake_l");
    private final CRServo feeder = hardwareMap.get(CRServo.class, "upper_intake");
    private final Servo pusher = hardwareMap.get(Servo.class, "feeder");

    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState{
        START_SHOOT,
        CURRYONTHETHREE,
        SHOOT_LEAVE,
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.34629404617254, 125.62818955042528, Math.toRadians(147));
    private final Pose shootPose = new Pose(61.939246658566226, 82.06075334143378, Math.toRadians(135));
    private final Pose ByeBye = new Pose(61.76427703523694, 48.46658566221142, Math.toRadians(135));
    private PathChain driveStarttoCurry, driveCurrytoBye;
    public void buildPaths() {
        driveStarttoCurry = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveCurrytoBye = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,ByeBye))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ByeBye.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch(pathState){
            case START_SHOOT:
                follower.followPath(driveStarttoCurry, true);
                pathState = PathState.CURRYONTHETHREE;
                break;
            case CURRYONTHETHREE:
                right.set(-0.75);
                left.set(0.75);
                feeder.setPower(1);
                pusher.setPosition(45.0 / 57.2958);
                if (!follower.isBusy() && pathTimer.getElapsedTime()>5) {
                    pusher.setPosition(22.0 / 57.2958);
                    right.set(0);
                    left.set(0);
                    pathState = PathState.SHOOT_LEAVE;
                    break;
                }
            case SHOOT_LEAVE:
                follower.followPath(driveCurrytoBye, true);
                break;
        }
    }

    public void SetPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.START_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower= Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        SetPathState(pathState);
    }

    public void loop() {
        follower.update();
        statePathUpdate();

    }
}