# Welcome to Solaris!

Thank you very much for visiting our repository! The goal of Solaris is to develop and provide reusable code for amateur rocketry.
Through the different modules and the software architecture, our objective is to enable the rapid development of software for amateur rocketry teams that lack the time or resources to build it themselves.

## Installation
To use and compile the code, we have developed a Docker container with the latest ESPRESSIF image — the manufacturer of our boards.
However, this configuration can easily be adapted (by changing the Docker image) to any other board or architecture.
We use Docker as a common base so that all our developers share the same dependencies and can build code in a consistent environment.

We recommend reading our [installation guide for Linux users](https://github.com/secureshadow/solaris-software/wiki/Installation-guide-for-Linux-users) and also for the [Windows users](https://github.com/secureshadow/solaris-software/wiki/Installation-guide-for-Windows-users)

## Versions
We have currently one version of our code: [solaris-v0](https://github.com/Software-Solaris/solaris-software/tree/solaris-v0) where we configure the barometer BMP390 and the IMU ICM20948.

## Next steps
We are currently working on an improved version of the Solaris software. We will introduce in nexts versions the use of a new Solaris Packet Protocol(SPP) for efficient communication between modules in amateur rocketry. We are also developing all of our code creating our own Hardware Abstraction Layer (HAL) and Operating System Abstraction Layer (OSAL) to allow everyone to run our code independently of their OS or board!
If you wish to colaborate on this layers, you can explore all our branches or contact the repository administrators via email.

## Collaboration
We are open to contributions, ideas or improvements — it is the only way to progress and be able to develop faster.
To collaborate, you will need to create an account in GitHub. To particpate, users can create forks of our repository and propose their modifications, they will then need to associate a PR to our repository against your branch in your forked repo.

## License
This project is licensed under **GPLv3 with Additional Terms for Amateur Rocketry**.  
Commercial use is allowed, but all modifications must remain open source and properly attributed.  
See [LICENSE](https://softwaresolaris.com/solaris/solaris-software/src/branch/main/LICENSE.md) for full terms


 
