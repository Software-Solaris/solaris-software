# Use the base image for ESP-IDF development
FROM espressif/idf

# Install necessary packages
USER root

# Install sudo and other essential packages
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
    && rm -rf /var/lib/apt/lists/*

# Set nano as the default Git editor
RUN git config --global core.editor "nano"

# Set the working directory
WORKDIR /root

# Install arduino-cli as the root user
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install git completion
RUN curl -o /etc/bash_completion.d/git-completion.bash \
    https://raw.githubusercontent.com/git/git/master/contrib/completion/git-completion.bash

# Copy and set up terminal scripts
COPY .devcontainer/setup-terminal.sh /root/setup-terminal.sh
RUN chmod +x /root/setup-terminal.sh

# Update .bashrc for git completion and custom scripts
RUN echo "source /etc/bash_completion.d/git-completion.bash" >> /root/.bashrc \
    && echo "source /root/setup-terminal.sh" >> /root/.bashrc

# Ensure the final user is root
USER root
