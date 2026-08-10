# Installing Stretch on your Own Computer

## Windows

Windows has not historically been very ROS-friendly. But fear not, Windows ships with a handy tool called _Windows Subsystem for Linux_, Version 2, or WSL2 for short. It is very important that you use the second version, which is able to access your computer's GPU. The first version did not do that. WSL2 works best on Windows 11.

WSL2 is _not_ a virtual machine, which makes it even better. Effectively, Windows users have full access to Linux. Follow the steps below to set it up:

1. Open Powershell as administrator
2. Ensure that you're using WSL2, not WSL1. Run `wsl --set-default-version 2`
3. Open a regular powershell session
4. Run `wsl --install -d Ubuntu-22.04` to instal. Ubuntu 22.04
5. Restart your PC if prompted by the installer. Sometimes powershell gets stuck on "Launching Ubuntu 22.04". If this happens and it's been a while, exit out of Powershell, start a new Powershell instance, and run `wsl --shutdown`
6. If Ubuntu 22.04 didn't start automatically from the install process, it should be available from your Start menu. Launch it.
7. You'll be guided through username and password creation for your Ubuntu distro.

You can open as many Ubuntu 22.04 terminals that you need, just like if you had a native Linux machine.

Now that you have Ubuntu 22.04 installed on your Windows computer, the same Linux steps apply for setting up ROS2. Follow the steps here: [GettingStartedLinux.md](GettingStartedLinux.md).