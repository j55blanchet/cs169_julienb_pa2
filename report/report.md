# Programming Assignment 2
Julien Blanchet - CS169 - 2/2/2020

**KEY LINKS**
* ROSBag & Results csv file https://1drv.ms/u/s!AiFiPYRO3Kyph49sCXQj4euRLGRaRA?e=ATnOvU
* Github repository with code https://github.com/j55blanchet/cs169_julienb_pa2
<hr>

## Description

One of my key design decisions was to refer to the starting point of the robot as position x=0 for the purposes of the pose estimation and kalman filter. I chose this because it seemed to allow for the quickest bootstrapping of a valid localization - if we refer to the starting position as 0, we can start off with a high confidence / low uncertainty in our position. Then, incorporating this choice into a larger map is as simple as publishing a tf transform when we discover elements that allow us to locate / identify the desired map frame.

I opted to seperate the logic for the Kalman Filter into its own class, for reusability purposes. I started off by following the math described in the class slides very closely, however I found this didn't work in context of my decision to use the starting position at 0, because then the "predicted reading" of the scan sensors couldn't be computed by multiplication - it was an addition problem. So, I decided to refactor the filter to take the "expected reading" as an input to the update step, which allows for this flexibility. It's also worth noting that my filter imlementation omitted some transformations that were unnecessary for the current application (such as using the sensor model to transform the Kalman Gain, sensor covariance, and uncertainty measurments). This means that the logic of the filter would need to be modified to support different sensor models.

## Evaluation 

## Allocation of Effort
As this was an individual programming assignment, all work was done by myself. I did discuss with Mingi how to incorporate rosbag into a launch file.
