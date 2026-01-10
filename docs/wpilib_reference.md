# WPILib Reference Document

## Table of Contents:

1. [Component Class Reference](#component-class-reference)
2. [3rd Party Vendor Libraries](#3rd-party-vendor-libraries)
3. [Posting Dashboard Values](#posting-dashboard-values) 
4. [Posting NetworkTables Values](#posting-networktables-values)
5. [Solenoids](#solenoids)

## Component Class Reference:

|Class|Import Path|Notes|
|-|-|-|
|`TalonFX`|`com.ctre.phoenix6.hardware.TalonFX`|-|
|`CANSparkMax`|`com.revrobotics.CANSparkMax`|-|
|`WPI_TalonSRX`|`com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX`|-|
|`DoubleSolenoid`|`edu.wpi.first.wpilibj.DoubleSolenoid`|Positions are accessible at `DoubleSolenoid.Value`; more info available [here](#solenoids)|

## 3rd Party Vendor Libraries:

*3rd Party Vendor Libraries* are libraries written and distributed by component manufacturers for use in interfacing with their components (usually motors). To install REVLib or the CTRE Phoenix library:

1. Install their respective software. 
	- For REVLib, download the [Java/C++ API](https://docs.revrobotics.com/brushless/spark-flex/revlib#c-and-java-installation) (blue text with "REVLib C++/Java Download-Version 2024.2.0" on it) and unzip it into the C:\Users\Public\wpilib\2024 directory on Windows or ~/wpilib/2024 directory on Linux, as described on the website under [offline installation](https://docs.revrobotics.com/brushless/spark-flex/revlib#offline-installation). 
	- For the CTRE Phoenix library, download and install the [Phoenix Framework](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/download/v24.1.0/Phoenix-Offline_v24.1.0.exe).
2. Next, in VSCode, press the wpilib icon in the top right in your project and go to `WPILib: Manage Vendor Libraries` then `Install new libraries (offline)` and select `REVLib`, `Phoenix (v5)`, or `Phoenix (v6)`.

To uninstall a library, go to `WPILib: Manage Vendor Libraries` again and select `Manage current libraries`, then select any libraries you wish to remove and press enter. Note that you'll need both Phoenix v5 and v6 in most cases. This menu can also be used to check what libraries you currently have installed.

> ***IMPORTANT:*** **Do not** install the `Phoenix (Pro)` library, as it is locked behind a paywall and will prevent your code from building while installed.

## Posting Dashboard Values:

FILL THIS IN COLIN!

## Posting NetworkTables Values:

Another feature of WPILib is NetworkTables, which can be used to communicate values between the driverstation computer, the RoboRIO, and any coprocessors (such as Raspberry Pi's) that the robot my have on it. All NetworkTable values are copied to all devices connected to NetworkTables, and values are organized similarly to a filesystem, where the folders are "subtables" and the files are "topics" (the NetworkTables term for a value). You can reference a topic/value by accessing its table, similarly to opening a folder, then subscribing (read topics/values) or publishing (write topics/values) to your topic.

1. Import `edu.wpi.first.networktables.NetworkTable`, `edu.wpi.first.networktables.NetworkTableInstance`, and the topic for the data type you are working with, for example a float: `edu.wpi.first.networktables.FloatTopic`
2. Create a `NetworkTableInstance`, a `NetworkTable`, and a subscriber or publisher for the data type:
	```java
	NetworkTableInstance networkTableInst = NetworkTableInstance.getDefault(); //the default networktables network, which is the one the RoboRIO and DriverStation are connected to

	NetworkTable table = networkTableInst.getTable("YourTable"); //the table to be used, if it does not exist this will create it

	FloatSubscriber subscriber = table.getFloatTopic("YourTopic").subscribe(0.0f); //get the topic within the table to be read, then subscribe to it; the `subscribe` parameter sets the default value in case the topic is empty (read)

	FloatPublisher publisher = table.getFloatTopic("YourTopic").publish(); //publish to the same topic as above (write)

	//if the topic does not exist calling `getFloatTopic` will create it
	```
3. Read or write values from or to your topic:
	```java
	topic.get(); //returns the float value the topic is currently set to

	publisher.set(123.4f); //takes a float to set the topic value to
	```

 [**DOCS**](https://docs.wpilib.org/en/stable/docs/software/networktables/tables-and-topics.html)

> ***NOTE:*** If you need to both subscribe and publish to a topic, you can use `getEntry()`, which acts as both in one variable.


## Solenoids

Solenoids are the controllers for air-powered devices, generally pistons. We use double solenoids, which can both push a piston out and suck it back in. These have three settings: Forward, Reverse, and Off. A double solenoid should never be in Forward or Reverse for longer than a fraction of a second (about 3 ms), just long enough to move the piston. It must then be set to Off again in order to avoid damaging the piston. There is a sample subsystem and command available for them [here](command_based#example-solenoid-subsystem-and-command).

[**DOCS**](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html)
