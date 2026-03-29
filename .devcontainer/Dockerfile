FROM espressif/idf:v6.0-dev

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
    sed \
    util-linux \
    locales \
    doxygen \
    graphviz \
    texlive-latex-extra \
    texlive-fonts-recommended \
    texlive-plain-generic \
    latexmk \
    clang \
    clangd \
    clang-format \
    clang-tidy \
    clang-tools \
    cppcheck \
    flawfinder \
    python3-pip \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN git config --global core.editor "nano"

WORKDIR /root

RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

RUN curl -o /etc/bash_completion.d/git-completion.bash \
    https://raw.githubusercontent.com/git/git/master/contrib/completion/git-completion.bash

COPY .devcontainer/setup-terminal.sh /root/setup-terminal.sh
RUN chmod +x /root/setup-terminal.sh

RUN echo "source /etc/bash_completion.d/git-completion.bash" >> /root/.bashrc \
    && echo "source /root/setup-terminal.sh" >> /root/.bashrc \
    && echo "source /opt/esp/idf/export.sh >/dev/null 2>&1" >> /root/.bashrc

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

USER root