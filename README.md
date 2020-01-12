# 2020 FIRST Robot

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

In order to get started with this project you will need the required software provided by the WPILib Installer for Java: https://github.com/wpilibsuite/allwpilib/releases

The individual downloads can also be found here:
1. Java SE Development Kit 11 (http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)
2. Text editor or IDE. The officially supported text editor is Visual Studio Code with the WPILib plugin found here:
https://marketplace.visualstudio.com/items?itemName=wpilibsuite.vscode-wpilib An alternative is to use [Intellij IDEA] (https://www.jetbrains.com/idea/)
2. FRC 2019 Update Suite (http://www.ni.com/download/first-robotics-software-2017/7904/en/)
4. CTRE Framework (http://www.ctr-electronics.com/hro.html#product_tabs_technical_resources)

### Installing

A step by step installation guide to run this program on the Team 61 FRC Robot

1. Get the FRC 2019 Update Suite by visiting [their website](http://www.ni.com/download/first-robotics-software-2017/7904/en/) and downloading the Update Suite. A encryption key may be necessary, if so, for 2019 it is: '$Robots&in#SPACE!!'. Be sure to uninstall any previous version of National Instruments software before starting. Extract the package that you have downloaded, and run the installer. For testing purposes, it is not necessary to activate the software if asked.
2. Download the installer for the Java SE Development Kit 8 (any other version will not function properly with the 2018/2019 code) from [this website.](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html) Make sure that the version downloaded fit the operating system of the computer you are testing on. Run the installer.
3. Next, download the IDE you plan on using and run the installer if there is any.
4. You might have to now set your JAVA_HOME variable, and if you do not know how to do this, you may find out [here](https://docs.oracle.com/cd/E19182-01/820-7851/inst_cli_jdk_javahome_t/).
5. Next, download the installer for the CTRE Toolsuite Legacy v4 from [this website](http://www.ctr-electronics.com/hro.html#product_tabs_technical_resources).

## Setting up Intellij IDEA (Windows users only)

If you are not familiar with any IDE, and are having trouble using this program with an IDE, you can follow this step-by-step guide on setting up a workspace with Intellij IDEA. Although Visual Studio Code is the recommended IDE for FRC development, we primarily used IDEA and can therefore best write a guide on setting up IDEA. If you are interested in Visual Studio Code, there are some setup tutorials [here](https://wpilib.screenstepslive.com/s/currentCS/m/java/c/57246).

1. Download and install Intellij IDEA. The download link can be found [here](https://www.jetbrains.com/idea/). Installation should be pretty easy if the installer is used.
2. Open the 'build.gradle' file within the project and add the line: 'id "idea"' to the plugins section if it is not already there.
3. Shift + right click on the directory you have pulled from this Github page, and open a Powershell window at the directory. Type the commands that follow to set up the IDEA environment:

```
./gradlew build
./gradlew idea
```

1. The last step is to open IDEA, and click open on the Welcome screen or File | Open on the main menu, and select the project directory. You should now be able to work on the project within IDEA. If you are struggling with using IDEA, there is a good tutorial [here](https://www.jetbrains.com/help/idea/using-code-editor.html).

## Deployment

To run the robot code, connect your computer to the RoboRIO, either by USB or Ethernet/radio. Next, open a powershell window at the project directory, and type the command:

```
./gradlew deploy
```

The console in Driver Station is very helpful in debugging issues in a scenario where this does not properly work. Many useful messages can be found to help with problems that might be run into.

## Built With

- [Java JDK](http://www.oracle.com/technetwork/java/javase/overview/index.html) - The language we used
- [WPILib](http://first.wpi.edu/FRC/roborio/release/docs/java/) - Libraries to program the FRC Robot

## Contributing

Since the season is close to over, we will most likely archive this project from further contributions.

## Authors

- **Kevin Downing** - *Head Programmer* - [KevinDowning](https://github.com/KevinDowning)
- **Thomas Buckley** - *Head Programmer* - [thomasbuckley](https://github.com/thomasabuckley)
- **Daniel Cardone** - *Programmer* - [daniel-cardone]( https://github.com/orgs/Team61/people/daniel-cardone )
- **Baraka  Consuegra** - *Shadowed Programmer* - [barakac]( https://github.com/orgs/Team61/people/barakac )
- **Jacob Gray** - *Shadowed Programmer* - [JGray2020](https://github.com/JGray2020)
- **Adam Dickey** - *Shadowed Programmer* - [adamdickey](https://github.com/adamdickey)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/Team61/2019-robot-v1/blob/master/LICENSE.md) file for details

## Acknowledgments

- Many thanks to FRC Team 5188 for their guide on [setting up the PixyCam with the RoboRio](https://github.com/FRC5188/ArduinoPixyAndRoboRIO).
- Thanks to the [previous 2015 Team 61 Programming Team](https://github.com/BVT-Team-61) for their helpful documentation and source code
