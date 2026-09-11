# Why we built Solaris Software

When starting the design of Solaris Software, we knew for sure, that we wanted something we could give to any student team. We see that good teams, often build their software (if they do and does not an external company), at closed gates. 

We could've followed the same approach. To develop everything to ourselfs. We think this does not provide any benefit. The worst thing a team can do is not to share knowledge. Because with sharing, a new person can learn and a more experienced person, can collaborate. This is a win-win because this new person can acquire knowledge and propose solutions in the future and the more experience person, can benefit of the work already done and make it better. This finally ends in a better software that is used in more projects, making it robust and easy to use.

Also, we built Solaris Software, because we see a tendency of people developing with the wrong tools, or in the wrong way. They promise they achieve some milestones in their project, but never provide the code evidence of how they have achieved it. For them 2 + 2 could mean 5 if you know where to put the if statement correctly. Sharing our code we expose ourselfs and our flaws. We like our work to be critizised and improved and so we hope you do it like so. We don't have ego, and if you think something is done better in some way, we will listen to you and make the appropiate changes to improve it.


# How is the software designed

The major challenge is to make a software that is reusable in any platform, almost for any project. That is why we build it with what is called abstractions. Abstractions allow us to write code without knowing in which board or computer will be used, this way, we are not limiting teams on choosing their preferred microcontroller.

Everything in the code is tought to be atomic and isolated. "Atomic" in some sense. You will see later.

If we absract the hardware and all the "services", then the software could be tought as an "app store" where you can install or uninstall apps. It does not matter which device you have. 

In the next sections, we will explain the main architecture and all of its parts one by one. I hope it is clear and that you enjoy reading it.

