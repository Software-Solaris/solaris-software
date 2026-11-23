FROM ubuntu:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    curl \
    git \
    gnupg2 \
    nano \
    bash-completion \
    build-essential \
    coreutils \
    grep \
    clang-format \
    cppcheck \
    cmake 

RUN useradd -m -s /bin/bash solaris

WORKDIR /solaris

COPY setup-terminal.sh /solaris/setup-terminal.sh
RUN chmod +x /solaris/setup-terminal.sh \
    && echo "source /etc/bash_completion.d/git-completion.bash" >> /home/solaris/.bashrc \
    && echo "source /solaris/setup-terminal.sh" >> /home/solaris/.bashrc \
    && chown -R solaris:solaris /solaris

USER solaris
RUN echo 'source /solaris/setup-terminal.sh' >> /home/solaris/.bashrc
