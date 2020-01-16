# 2020 FIRST Robot

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

## Setup Instructions

1. Clone this repository.
2. Run `./gradlew` to download Gradle along with the required WPILib and Vender libraries.
3. Run `./gradlew tasks` to view of available tasks.

### Setting up text editor or IDE

#### Visual Studio Code (Official IDE)

1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater



#### Setting up Intellij IDEA

*Note: Intellij IDEA isn't directly supported by FRC and may run into issues. Visual Studio Code is the support text editor for FRC.*

1. Open the `build.gradle` file within the project and add the line: `id "idea"` to the plugins section if not already there.
2. Type the following commands to set up the IDEA environment:

```
./gradlew build
./gradlew idea
```



### Deployment

Here are a set of instructions for uploading code to your robot: 

1. connect your computer to the RoboRIO, either by USB or Ethernet/radio. 
2. Then run the command `./gradlew deploy` .

If you run into any issues use the console in Driver Station to help with debugging.

## Code Highlights

* Tracking position on the field

  Through kinematics, odometry and absolute measures such as with vision and check points, the robot is able to accurately track it's position on the field.

* Turret locked onto power port

  Based on the coordinate system the robot has on the field, the turret is able to constantly face in the direction of any of the power ports as long as the turret is within range (200° view).  When not even view the turret heading is based on how close a side is to being in view. 

* Cubic splined paths

  Rather than making a move-stop-turn approach, we're using cubic splined paths which created smooth curves on the points we need to travel. This significantly increases the speed at which we travel.

## Built With

- [Java JDK  11](http://www.oracle.com/technetwork/java/javase/overview/index.html) - The language we used
- [WPILib](http://first.wpi.edu/FRC/roborio/release/docs/java/) - Libraries to program the FRC Robot
- [Shuffle Board](https://github.com/wpilibsuite/PathWeaver) - A board to display robot data in real time
- [Path Weaver](https://github.com/wpilibsuite/shuffleboard) - A program to illustrate trajectories and paths on the field

## Contributing

Since the season is close to over, we will most likely archive this project from further contributions.

## Authors

- **Kevin Downing** - *Head Programmer and Author of Documentation* - [KevinDowning](https://github.com/KevinDowning)
- **Thomas Buckley** - *Head Programmer* - [thomasbuckley](https://github.com/thomasabuckley)
- **Daniel Cardone** - *Programmer* - [daniel-cardone]( https://github.com/orgs/Team61/people/daniel-cardone )
- **Baraka  Consuegra** - *Shadowed Programmer* - [barakac]( https://github.com/orgs/Team61/people/barakac )
- **Jacob Gray** - *Shadowed Programmer* - [JGray2020](https://github.com/JGray2020)
- **Adam Dickey** - *Shadowed Programmer* - [adamdickey](https://github.com/adamdickey)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/Team61/2019-robot-v1/blob/master/LICENSE.md) file for details

## Acknowledgments

- Thanks Team 254 on their countless tutorials on path planning, trajectories and vision cam.
- Thanks Team 2102 for helping us create code for the LIDARLite sensor
