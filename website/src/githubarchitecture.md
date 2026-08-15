## GitHub repository

Our GitHub repository is organized in three repositories as of right now:

- solaris-software: our platform dependent code.
- solaris-packet-protocol: Our own home-made protocol for amateur rocketry.
- .github: this is just for the welcome page of our GitHub.

Let's explain a small brief about each other.


## Solaris Software

In this repository, we have all the dependencies that are strictly tied to our board. Currently, we are using the ESP32S3 board, from Espressif. The directory is organized as follows:

- .devcontainer:
    - This contains all the utilities needed to run our docker container using the VSCode devcontainer extension.
    - It also contains the Dockerfile that is used to build the container. In this Dockerfile, we are installing all the dependencies needed to compile our code.
    - The .devcontainer/devcontainer.json file is used to configure the devcontainer, this inlcudes importing USB ports, and installing the required extensions for working together.

    We decided to use this approach, because it is more comfortable to have all the devs working wih the same dependencies. We don't want a: "In my computer works!" moments. This way, all the code works in every computer, either Windows or Linux.

- .github:
    - This is where we have our GitHub Actions and pipelines.
    - Currently we have just two pipelines. One that build and publish this website to our servers and another that compiles our code. The latest is useful to check if the code we have just pushed to GitHub is wrong or have some compilation problems.
    - We also have some cppcheck to implement MISRA C:2012 rules and we plan on implementing an open source alternative for SonarQube, to check our code quality, but this will be impose more strictly once we have a good code base.

