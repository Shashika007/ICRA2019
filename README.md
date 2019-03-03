
# RoboGrinder 2019 IRCA Competition Code

## Introduction
RoboRTS is an open source software stack for Real-Time Strategy research on mobile robots, developed by RoboMaster and motivated by [RoboMaster AI Challenge](#competition)

The framework of RoboRTS consists two parts: autonomous mobile robot layer and intelligent decision-making layer.

<img src="images/system.png" style="zoom:80%;display: inline-block; float:middle"/>

The autonomous mobile robot layer alone can let a robot, offically supported for RoboMaster AI Robot platform, to demonstrate a certain level of intelligence. On its on-board computer runs perception, motion planning, decision modules. It is fully supported in ROS with community driven as well as platform-customized codes and examples. On its MCU, an RT low-level robot controller is implemented to govern the robot driving system.  

**TODO:** Intelligent decision-making layer includes a multi-agent decision-making framework and a game simulator, it will be released soon in the future.

### Development process

#### 1. Create a branch from master on computer
>\>>git checkout -b Myfeature

#### 2. Create same branch on github
>\>>git push -u origin Myfeature

#### 3. Do stuff on branch
>\>>vim main.cpp

> int main(argc, argv\*\*)\
>{\
>cout << "Robogrinder Win"!\
>}

#### 4. COMMIT AND PUSH OFTEN
>\>>git commit -a\
\>>git push

#### 5. Go to Manifold and test your changes
#### 6. If everthing works and you are done, go to step 7. If you broke something or you are not done go back to step 3.
#### 7. Initiate a pull-request
After you have finished and have tested your work, you will merge your branch with the branch called "integration".
follow these steps:
1. Got to this reposotory in GitHub 
2. In the "Branch" menu, choose your branch.
3. To the right of the Branch menu, click New pull request.
4. **\*IMPORTANT\*** Change the base branch to **"integration"** NOT "master."
5. Change the compare branch to your branch.
6. Type a title and description for your pull request.
7.  Click "Create pull request".

You can find more information on creating a pull-request here: https://help.github.com/articles/creating-a-pull-request/

Once you create a pull request the team lead will review it and test what you have done. The team lead will tell you if you need to make changes. \
Once the team lead says evertyhing is okay, you will close the pull-request using the following steps:
1. Under your repository name, click  "Pull requests".
2. In the "Pull Requests" list, click the pull request you'd like to merge.
3. Select "Merge pull request".
4. Type a commit message or accept the default message. Under the commit message box, click "Confirm merge".
5. If you are done with the branch, then you should delete it.

You can find more information on merging a pull-request here: https://help.github.com/articles/merging-a-pull-request/

Remember, **DO NOT** merge anything into master. Only the team lead will be allowed to do so. If you try it will not work anyway.

If you have any questions you can post them on the group chat.

## Tutorial

For more information about RoboMaster AI Robot platform and RoboRTS framework, please refer to [RoboRTS Tutorial](https://robomaster.github.io/RoboRTS-Tutorial/#/)

## Competition

RoboMaster holds AI Challenge since 2017. In the challenge, multiple robots should fight with each other on a game field automatically under different rules.

For more information, please refer to

- [DJI RoboMaster 2019 ICRA AI Challenge](https://icra2019.org/competitions/dji-robomaster-ai-challenge)

- [DJI RoboMaster 2018 ICRA AI Challenge](https://icra2018.org/dji-robomaster-ai-challenge/)

## Copyright and License

RoboRTS is provided under the [GPL-v3](COPYING).
