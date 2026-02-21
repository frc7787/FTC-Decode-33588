package org.firstinspires.ftc.teamcode.Opmodes;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Mechanisms.AutoConstantsRed.*;



@Autonomous(name = "AutoRedGoal", group = "opmodes")
public class AutoRedGoal extends  OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Shooter shooter;




    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1StartPose.getHeading())
                .addPath(new BezierLine(pickup1StartPose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1StartPose.getHeading(), pickup1EndPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, scorePoseFake))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePoseFake.getHeading())
                .setGlobalDeceleration(4)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        grabPickup2Pre = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFake, pickup2StartPrePose))
                .setLinearHeadingInterpolation(scorePoseFake.getHeading(), pickup2StartPrePose.getHeading())
                .addPath(new BezierLine(pickup2StartPrePose, pickup2StartPose))
                .setLinearHeadingInterpolation(pickup2StartPrePose.getHeading(), pickup2StartPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                //.addPath(new BezierLine(scorePose, pickup2StartPose))
                //.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2StartPose.getHeading())
                .addPath(new BezierLine(pickup2StartPose,pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(),pickup2EndPose.getHeading())
                .setGlobalDeceleration(3)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, pickup2StartPose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), pickup2StartPose.getHeading())
                .addPath(new BezierLine(pickup2StartPose, scorePoseFake2))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), scorePoseFake2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3StartPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        leaveGoal = follower.pathBuilder()
                .addPath(new BezierLine(startPose,leavePoseGoal))
                .setLinearHeadingInterpolation(startPose.getHeading(),leavePoseGoal.getHeading())
                .build();

        // AUDIENCE SIDE PATHS

        scorePreloadAudience = new Path(new BezierLine(startPoseAudience, scorePoseAudience));
        scorePreloadAudience.setLinearHeadingInterpolation(startPoseAudience.getHeading(), scorePoseAudience.getHeading());

        grabPickup3Audience = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseAudience, pickup3StartPose))
                .setLinearHeadingInterpolation(scorePoseAudience.getHeading(), pickup3StartPose.getHeading())
                .addPath(new BezierLine(pickup3StartPose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3StartPose.getHeading(), pickup3EndPose.getHeading())
                .setGlobalDeceleration(4)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3Audience= follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, scorePoseAudience))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePoseAudience.getHeading())
                .setGlobalDeceleration(4)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        grabPickup2PreAudience = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseAudience, pickup2StartPrePose))
                .setLinearHeadingInterpolation(scorePoseAudience.getHeading(), pickup2StartPrePose.getHeading())
                .addPath(new BezierLine(pickup2StartPrePose, pickup2StartPose))
                .setLinearHeadingInterpolation(pickup2StartPrePose.getHeading(), pickup2StartPose.getHeading())
                .build();

        grabPickup2Audience = follower.pathBuilder()
                .addPath(new BezierLine(pickup2StartPose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), pickup2EndPose.getHeading())
                .setGlobalDeceleration(3)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        scorePickup2Audience= follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose  , scorePoseAudience))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePoseAudience.getHeading())
                .setGlobalDeceleration(4)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        leaveAudience = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseAudience,leavePoseAudience))
                .setLinearHeadingInterpolation(scorePoseAudience.getHeading(),leavePoseAudience.getHeading())
                .build();


    } // end of BuildPaths


    // MANAGING PATH STATES
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {  // FOLLOW PATH TO SCORING - preloaded
                if (shooter.launch(true)) {
                    //follower.followPath(scorePreload);
                    setPathState(1);
                }
                //shooter.spin(2000);
                break;
            }
            case 1: {  // FOLLOW PATH TO SCORING - preloaded
                follower.followPath(leaveGoal);
                setPathState(2);
                //shooter.spin(2000);
                break;
            }
            case 2: { // DONE PATH

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    // turn off mechanisms
                    setPathState(-1);
                }
                break;
            }
            case 3: { // JUST SCORING - preloaded
                if (true //shooter.score(false, 3,telemetry)
                ) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, 0.6, true);
                    // setPathState(2); OK, let's just test the first two paths.
                    setPathState(3);
                }
                break;
            }
            case 4: { // FOLLOW GRAB PATH - pickup 1
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, 0.7,true);
                    setPathState(4);
                }
                break;
            }
            case 5: { // FOLLOW PATH TO SCORING - pickup 3
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            }
            case 6: { // JUST SCORING - pickup 3
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (true //shooter.score(false, 3, telemetry)
                ) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2Pre, true);
                    setPathState(6);
                }
                break;
            }
            case 7: {
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2,0.6,true);
                    setPathState(61);
                }
                break;
            }


            case 8: { // FOLLOW GRAB PATH - pickup 2
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;
            }
            case 9: { // FOLLOW PATH TO SCORING - pickup 2
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            }
            case 10: { // JUST SCORING - pickup 2
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (true //shooter.score(false, 3, telemetry)
                ) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(grabPickup3,true);
                    follower.followPath(leaveGoal, true);
                    setPathState(9);
                }
                break;
            }
            case 11: { // FOLLOW PATH LEAVEAUDIENCE
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            }
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    } // end autonomousPathUpdate

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    } // end setPathState

    // INIT AND LOOP METHODS
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    } // end loop

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter = new Shooter(hardwareMap);




        //shooter = new Shooter(hardwareMap);

        //shooter.setShooterVelocity(shooter.RPM_GOAL); // NEAR for Goal

    } // end init

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}




} // end of AutoByExampleDec

