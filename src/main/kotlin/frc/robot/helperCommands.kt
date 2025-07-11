package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.toRotation2d
import frc.robot.lib.extensions.volts
import frc.robot.subsystems.drive.alignToPose

fun resetLocation(): Command = runOnce({ swerveDrive.resetOdometry(Pose2d()) })

fun goTo(x: Distance, y: Distance, angle: Angle): Command =
    alignToPose(Pose2d(x, y, angle.toRotation2d()))

fun stopGripper(): Command = TODO("They need to fill this")

fun stopElevator(): Command = TODO("They need to fill this")

fun lockDrive(): Command = runOnce({ swerveDrive.stopWithX() })

fun resetRobotRotation(): Command =
    runOnce(
            { swerveDrive.resetGyroBasedOnAlliance(Rotation2d.kZero) },
            swerveDrive
        )
        .ignoringDisable(true)

