FROM ubuntu:latest

USER root

RUN apt-get update && apt-get install -y \
    sudo \
    curl \
    gnupg2 \
    nano \
    bash-completion \
    build-essential \
    coreutils \
    grep \
    clang-format \
    cppcheck \
    cmake

RUN mkdir solaris 

WORKDIR /solaris

RUN curl -o /etc/bash_completion.d/git-completion.bash \
    https://raw.githubusercontent.com/git/git/master/contrib/completion/git-completion.bash

COPY setup-terminal.sh /solaris/setup-terminal.sh
RUN chmod +x /solaris/setup-terminal.sh

RUN echo "source /etc/bash_completion.d/git-completion.bash" >> /root/.bashrc \
    && echo "source /solaris/setup-terminal.sh" >> /root/.bashrc

RUN useradd -ms /bin/bash solaris
USER solaris