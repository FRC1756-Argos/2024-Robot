# Vision Pipelines

This directory contains vision pipelines to detect the 2024 vision targets (April Tags) in order to determine target distances and direction.

We explored both [Limelight](https://limelightvision.io/) and [PhotonVision](https://photonvision.org/) options, but ultimately used Limelights as we had various issues with PhotonVision.

The pipelines are detailed below.

## Limelight

The main limelight is on the back of the robot and it has a static IP of `10.17.56.122`.
The secondary limelight is on the front of the robot near the intake and it has a static IP of `10.17.56.123`.

In the `limelight3` directory, we have five pipelines:
* `PrimaryPipeline` - This pipeline is the only pipeline we use for the back limelight. It filters april tags to detect tags 4, 17, 11, 12, 13, 14, 15, and 16. It is used for shooting notes into the speaker and alignment to the trap.

In the `limelight3_front` directory, we have one pipeline:
* `SecondaryPipeline` - This pipeline is the only pipeline we use for the front limelight. It filters april tags to only detect tags 4 and 7. It is only used for shooting notes into the speaker.
