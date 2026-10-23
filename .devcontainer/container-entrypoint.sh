#!/usr/bin/env bash
# Container entrypoint wrapper.
#
# The host ~/.ssh and ~/.gitconfig are bind-mounted read-only under /tmp so
# they can be copied into root-owned locations. Mounting ~/.ssh directly at
# /root/.ssh leaves the files owned by the host UID (1000), which makes ssh
# reject them with "Bad owner or permissions". Mounting ~/.gitconfig as a
# single file at /root/.gitconfig also breaks `git config --global`, because
# git writes a lock file and renames it over the mount point (EBUSY).
set -e

if [ -d /tmp/host-ssh ]; then
    mkdir -p /root/.ssh
    cp -a /tmp/host-ssh/. /root/.ssh/
    chown -R root:root /root/.ssh
    chmod 700 /root/.ssh
    find /root/.ssh -type f -name '*.pub' -exec chmod 644 {} +
    find /root/.ssh -type f ! -name '*.pub' -exec chmod 600 {} +
fi

if [ -f /tmp/host-gitconfig ]; then
    cp /tmp/host-gitconfig /root/.gitconfig
    chown root:root /root/.gitconfig
    chmod 600 /root/.gitconfig
fi

exec /opt/esp/entrypoint.sh "$@"
