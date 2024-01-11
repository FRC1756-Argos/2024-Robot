# 2023-Robot

[![CI](https://github.com/FRC1756-Argos/2024-Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/FRC1756-Argos/2024-Robot/actions/workflows/ci.yml) [![Format](https://github.com/FRC1756-Argos/2024-Robot/actions/workflows/format.yml/badge.svg)](https://github.com/FRC1756-Argos/2024-Robot/actions/workflows/format.yml) [![Documentation](https://github.com/FRC1756-Argos/2024-Robot/actions/workflows/doxygen.yml/badge.svg)](https://github.com/FRC1756-Argos/2024-Robot/actions/workflows/doxygen.yml)

Robot code for 2023 FRC season

## Key Features

### LED Color Codes

| Color | Pattern | Strips | Meaning | Trigger |
| ----- | ------- | ------ | ------- | ------- |
| Red/Blue | Solid | All | Robot disabled; indicates alliance received from DS | Robot disabled |
| Cat Yellow | Solid | All | Robot disabled; no connection to FMS or driver station since startup | Robot startup |
| Red/Blue | Uniform fade in/out | All | Teleop enabled; indicates alliance received from DS | Robot enabled in teleop mode |
| Red/Blue | Fade sweeping from front to back | All | Autonomous enabled; indicates alliance received from DS | Robot enabled in autonomous mode |

### Controller Vibration Feedback

| Pattern | Controller | Meaning |
| ------- | ---------- | ------- |
| Continuous 100% | Both | Swapping controllers activated.  Swap occurs after vibration ends |

## FTP

* ftp://172.22.11.2 -USB only
* ftp://roborio-1756-frc.local -USB, Wifi, or Ethernet
* ftp://10.17.56.2 -Wifi or Ethernet

## Commissioning A Robot

When commissioning a new robot, you should set the instance type to either "Competition" or "Practice" by creating a text file using FTP readable by `lvuser` on the RoboRIO at path `/home/lvuser/robotInstance`.  The content of this file should be just the text `Competition` or `Practice` with no whitespace preceding.  If no valid instance is found at runtime, competition instance will be used and an error will be generated.

### Homing Swerve Drive

1. When homing swerve modules figure out what is the front of the robot. The intake is the front.
2. Rotate each swerve module to where each bevel gear is to the left side of the robot.
3. Use something flat that is the length of the robot and line the swerve wheels up.
4. Power on the robot and connect your computer. You will need one XBox controller. Make sure the controller says its controller 0(driver) in drivers station. Once connected, enable the robot and press your swerve homing buttons. We are using the buttons <kbd>A</kbd>, <kbd>B</kbd>, and <kbd>X</kbd> on the XBox controller. You have to hold the three buttons for 1.5 seconds.
5. Once the homes have been set the homes will be put in a document. You will be able to open up the document.
6. First open file explorer.
7. Once you are in click in the white box at the top where you can type.
8. Use one of the above FTP addresses.
9. Then double click on the home folder.
10. Then double click on the lvuser folder.
11. Then double click on the homes folder.
12. Then you can click on the swerveHomes file to see the homes of the swerve modules.


### Vision

See [vision readme](vision/README.md) for information on which pipelines to use and which indices to install these pipelines on.

### Playing With Fusion Addressing

Go to http://10.17.56.2:5812 while connected to the robot to update firmware or change addresses.

## Project Setup

### Pre-Commit

This project uses [pre-commit](https://pre-commit.com/) to check code formatting before accepting commits.

First install the prerequisites:

* python3 (with pip) - [instructions](https://realpython.com/installing-python/)
  * Python 3.12.x from the [Python website](https://www.python.org/downloads/) works well.  Make sure to check the add to path option in the installer.
* pip packages:
  * You may need to add the pip install path to your shell's path if you're using Git Bash.  In git bash:
    1. Open (or create) new file `~/.bashrc` by running `vim ~/.bashrc`
    2. Add this to the end: `PATH=$PATH:$LOCALAPPDATA/Programs/Python/Python39/Scripts/` (change `Python39` to match your python version)
       * **Note**: The actual path you need to add (`$LOCALAPPDATA/Programs/Python/Python39/Scripts/` in the above example) depends on your Python installation.  If y ou do the `pip install` steps first, pip will print the path you need to add.
       * To type in Vim, type <kbd>i</kbd> and you should see `INSERT` at the bottom of the window to indicate you're editing in insert mode
    3. Exit by pressing <kbd>Esc</kbd> then type `:wq` and press <kbd>Enter</kbd>
    4. Run `source ~/.bashrc` to update your session
  * wpiformat - `pip install wpiformat`
  * clang-format - `pip install clang-format`
  * pre-commit - `pip install pre-commit`

  Make sure to run `pip install <package>` commands in an administrator terminal if installing in windows

Then initialize:

```
pre-commit install
pre-commit run
```

The first run may take a moment, but subsequent automatic runs are very fast.

You'll now have the linter run before each commit!  For compatibility with Windows, we recommend the pip version of clang-format, but wpi-format will find any installed `clang-format` binary in the system path.

## Controls

**Driver:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | Drive |
| Left JS Y       | Drive |
| Right JS X      | Turn |
| Right JS Y      | Unused |
| DPad Up         | Unused |
| DPad Right      | Unused |
| DPad Down       | Unused |
| DPad Left       | Unused |
| A               | Home Swerve (hold with <kbd>B</kbd> and <kbd>X</kbd>) |
| B               | Home Swerve (hold with <kbd>A</kbd> and <kbd>X</kbd>) |
| X               | Home Swerve (hold with <kbd>A</kbd> and <kbd>B</kbd>) |
| Y               | Field Home (hold) |
| LB              | Unused |
| RB              | Unused |
| LT              | Unused |
| RT              | Unused |
| Back            | Swap (hold with <kbd>Start</kbd>) |
| Start           | Swap (hold with <kbd>Back</kbd>) |
| Left JS Button  | Unused |
| Right JS Button | Unused |

**Operator:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | Unused |
| Left JS Y       | Unused |
| Right JS X      | Unused |
| Right JS Y      | Unused |
| A               | Unused |
| B               | Unused |
| X               | Unused |
| Y               | Unused |
| DPad Up         | Unused |
| DPad Right      | Unused |
| DPad Down       | Unused |
| DPad Left       | Unused |
| LB              | Unused |
| RB              | Unused |
| LT              | Unused |
| RT              | Unused |
| Back            | Swap (hold with <kbd>Start</kbd>) |
| Start           | Swap (hold with <kbd>Back</kbd>) |
| Left JS Button  | Unused |
| Right JS Button | Unused |

## Software Checkout

## Software Versions

We're using the following dependencies:

 * [CTRE Phoenix 24.1.0](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v24.1.0)
 * [Playing With Fusion 2024.01.10](https://www.playingwithfusion.com/docview.php?docid=1205)
 * [REVLib 2024.2.0](https://docs.revrobotics.com/brushless/spark-max/revlib)
 * [WPILib 2024.1.1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.1.1)

## Special Thanks

 * Our sponsors for the 2022 season.  Thank you for your continued support
   * [Caterpillar](https://www.caterpillar.com/)
   * [Gene Haas Foundation](https://ghaasfoundation.org/)
   * [Boeing](https://www.boeing.com/)
   * [Limestone Community High School](https://www.limestone310.org/)
   * [J. H. Benedict Co](https://www.jhbenedict.com/)
   * [LCHS Booster Club](https://www.facebook.com/LCHSBoosterClub/)
   * [Playing With Fusion](https://www.playingwithfusion.com/)
   * [Bean's Best LLC](https://beansbestllc.com/)
   * Caterpillar employees & [The Caterpillar Foundation](https://www.caterpillar.com/en/company/caterpillar-foundation.html)
 * [Doxygen Awesome](https://jothepro.github.io/doxygen-awesome-css/) - for making our [documentation](https://frc1756-argos.github.io/2022-Robot/) look great

## License
This software is licensed under the [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause). If you would like to use this software under the terms of a different license agreement, please [contact us](mailto:1756argos1756@limestone310.org).
