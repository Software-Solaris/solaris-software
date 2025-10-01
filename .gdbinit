local gdbinit = vim.fn.tempname()
local f = io.open(gdbinit, "w")
f:write([[
set confirm off
set pagination off
set breakpoint pending on
set remotetimeout 10
set print pretty on
set mem inaccessible-by-default off
cd /home/user/Documents/software-solaris/solaris-v1
target extended-remote raspi.local:3333
monitor reset halt
# thb app_main   <-- lo pones a mano luego
]])
f:close()

vim.cmd("GdbStart " .. gdb .. " -q -x " .. gdbinit .. " " .. elf)

