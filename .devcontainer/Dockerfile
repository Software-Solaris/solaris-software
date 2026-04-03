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
    clang-format \
    cppcheck \
    flawfinder \
    python3-pip \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Install SonarScanner CLI (SonarQube)
ARG SONAR_SCANNER_VERSION=7.1.0.4889

RUN curl -fsSL -o /tmp/sonar-scanner.zip \
    https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-${SONAR_SCANNER_VERSION}-linux-x64.zip \
    && apt-get update && apt-get install -y unzip \
    && unzip /tmp/sonar-scanner.zip -d /opt \
    && ln -s /opt/sonar-scanner-${SONAR_SCANNER_VERSION}-linux-x64/bin/sonar-scanner /usr/local/bin/sonar-scanner \
    && rm /tmp/sonar-scanner.zip \
    && rm -rf /var/lib/apt/lists/*

RUN git config --system core.editor "nano" \
    && git config --system --add safe.directory /home/user/Documents/solaris-software

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