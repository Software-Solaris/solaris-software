#!/bin/bash

# Solaris Team Welcome Message with Date and Time
clear
echo -e "\n\033[1;36m#################################################################\033[0m"
echo -e "\033[1;36m#  🚀 Welcome to the Solaris Software Development Terminal 🚀  #\033[0m"
echo -e "\033[1;33m       Current Date & Time: $(date)                             \033[0m"
echo -e "\033[1;36m#################################################################\033[0m\n"


# Custom terminal prompt with current Git branch and color enhancement
parse_git_branch() {
    git branch 2>/dev/null | grep '^*' | sed 's/^* //'
}
export PS1="\[\033[1;35m\]╭─[\[\033[1;36m\]Solaris Dev\[\033[1;35m\]]-[\[\033[1;34m\]\w\[\033[1;35m\]]\n\[\033[1;35m\]╰─> \[\033[1;32m\]\$(parse_git_branch)\[\033[0m\]$ "

# Arduino alias for common commands (with descriptive echo for clarity)
alias build="arduino-cli compile --fqbn arduino:avr:uno"
alias upload="arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno"
alias clean="arduino-cli cache clean && echo 'Arduino cache cleaned.'"

# Shortcuts and additional aliases
alias gs="git status"
alias gc="git commit"
alias gp="git push"
alias gl="git pull"

