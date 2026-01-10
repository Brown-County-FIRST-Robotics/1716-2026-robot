# IOLayer & WhoAmI Subsystems Guide

## Table of Contents

1. [Overview](#overview)
    1. [AdvantageKit](#advantagekit)
    2. [WhoAmI](#whoami)
2. [Interfaces In Java](#interfaces-in-java)
3. [Adding an `IOLayer` to a Subsystem](#adding-an-iolayer-to-a-subsystem)
4. [Misc. Logging](#misc-logging)

> NOTE: This guide is designed to be read after the [Command Based Document](./command_based.md), as that document explains important concepts referenced throughout this file.

## Overview

The [Command Based Document](./command_based.md) explains the basic layout and construction of a default command based project. In that document, we discuss three distinct layers (or abstractions). Our codebase, on the other hand, has significantly modified subsystems to allow for additional functionality. This functionality is essentially described by two systems we are using:

### AdvantageKit

AdvantageKit is a tool and library combination developed by team 6328 that allows for complete replay of testing and matches entirely through onboard robot logging data. To accomplish this, we add an additional layer beneath subsystems called the `IOLayer`. This layer is used to log motor data including setpoint, encoder position, and temperature. For more information on what this system is and why we use it, please refer to [this video](https://www.youtube.com/watch?v=mmNJjKJG8mw).

### WhoAmI

This is a custom system developed by Colin Finn to allow for more modular code. The essential basis of this system is to allow one variable change to determing the complete configuration of the robot, such as whether it has an arm, is using swerve or meccanum, or is in demo mode. To accomplish this, we split the `IOLayer` of each subsystem into multiple files, each of which can have a different configuration. The most common use of this involves the real motors used on the robot and a simulated version.

## Interfaces in Java

An important concept to understand before we begin is [interfaces](https://www.w3schools.com/java/java_interface.asp) (similar to [traits](https://doc.rust-lang.org/book/ch10-02-traits.html) in Rust). These are constructs in Java used for inheritance where the parent is not (and cannot be) used directly. They are useful for cases where we have various types of something, but we want the same code to apply to all of them. This is done by declaring several methods and variables in the interface that are available to all classes implementing it. These methods must provide a definition, providing the functionality for when the method is called. Each class implementing the interface can have different definitions and functionality for the methods while still being able to be called by the same code, provided it is written to take the interface. Additionally, interfaces can provide default functionality for methods, which means that classes implementing it are no longer required to provide their own definition.

This construct is used heavily in the [WhoAmI](#whoami) system. In this case, each subsystem's IOLayer is an interface, allowing for multiple different versions of that subsystem such as one with REV hardware, one with CTRe hardware, and a simulated version. This is what allows the modularity of this system.

## Adding an `IOLayer` to a Subsystem

This guide will build off of the Command Based Document [subsystem guide](./command_based.md#subsystem).

To start, we will create an `ExampleSubsystemIO.java` file (replacing `ExampleSubsystem` with your subsystem name). For this example, let's use a simple subsystem with a single `SparkMax`. Create a new folder with your subsystem's name, and move your subsystem into it. Then create `ExampleSubsystemIO.java` and add the following to it:

```java
package frc.robot.subsystems.examplesubsystem;

import org.littletonrobotics.junction.AutoLog; //Used for the @AutoLog annotation

public interface ExampleSubsystemIO {
  @AutoLog //Used for the AdvantageKit logging, required for your inputs class
  class ExampleSubsystemIOInputs { //Include every variable that needs logging in IOInputs
    //including position, velocity, appliedOutput, temperature, and current for each motor
    double position = 0.0;
    double velocity = 0.0;

    double appliedOutput = 0.0;
    double temperature = 0.0;
    double current = 0.0;
  }

  default void updateInputs(ExampleSubsystemIOInputs inputs) {} //This will be called to update IOInputs

  default void setSpeed(double speed) {} //Any commands to the motors must be provided in this interface
}
```

Next, we need to implement the `IOLayer` in a separate file. This one will be named `ExampleSubsystemIOSparkMax.java` based on our motor configuration:

```java
package frc.robot.subsystems.examplesubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ExampleSubsystemIOSparkMax implements ExampleSubsystemIO {
  private final SparkMax sparkMax; //The motor is declared here
  private final RelativeEncoder encoder; //The encoder is primarily used for logging data

  public ExampleSubsystemIOSparkMax(int id) { //Initialize the motor and encoder in the constructor
    sparkMax = new SparkMax(id, MotorType.kBrushless);
    encoder = sparkMax.getEncoder();
  }

  public void updateInputs(ExampleSubsystemIOInputs inputs) { //Update the inputs for logging
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();

    inputs.appliedOutput = sparkMax.getAppliedOutput();
    inputs.temperature = sparkMax.getMotorTemperature();
    inputs.current = sparkMax.getOutputCurrent();
  }

  public void setSpeed(double speed) { //Implement the speed method to allow for actual motor control
    sparkMax.set(speed);
  }
}
```

Finally, we need to update the subsystem accordingly (`ExampleSubsystem.java`):

```java
package frc.robot.subsystems.examplesubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ExampleSubsystem extends SubsystemBase { //Extends `SubsystemBase` as usual
  private final ExampleSubsystemIO exampleSubsystem; //access the components such as motors through the IOLayer
  private final ExampleSubsystemIOInputsAutoLogged exampleSubsystemInputs = new ExampleSubsystemIOInputsAutoLogged();
    //ExampleSubsystemIOInputsAutoLogged is automatically generated by the @AutoLog annotation in the interface
    //Before this is built for the first time, this will show as an error
  public Manipulator(ExampleSubsystemIO exampleSubsystem) {
    //initialize the IOLayer from `RobotContainer.java`, allowing for all configurations to be selected in one place
    this.exampleSubsystem = exampleSubsystem;
  }

  @Override
  public void periodic() {
    //Be sure to update the inputs each "frame"
    exampleSubsystem.updateInputs(exampleSubsystemInputs);
    Logger.processInputs("ExampleSubsystem", exampleSubsystemInputs);
  }

  public void setSpeed(double speed) {
    //speed can be set by calling the ExampleSubsystemIO.setSpeed() method defined in the interface
    exampleSubsystem.setSpeed(speed);
    Logger.recordOutput("ExampleSubsystem/speedReference", speed);
      //Record the setpoint for the speed so we know what the robot was trying to do
  }
}
```

## RobotContainer?

Cover the modifications to `RobotContainer.java`?

## Misc. Logging

In order to maintain complete simulation capability using AdvantageScope, we must log a variety of things. These include... This can be done by...
