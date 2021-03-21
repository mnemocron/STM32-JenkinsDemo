# JenkinsDemo_STM32f303_Voltmeter
STM32f303 Firmware for a simple Voltmeter Demo using the internal 12-Bit ADC and a SSD1306 Display

[![Build Status](https://jenkins.kaon.ch/buildStatus/icon?job=JenkinsDemo_STM32f303_Voltmeter)](https://jenkins.kaon.ch/job/JenkinsDemo_STM32f303_Voltmeter/)

---

- [how-to-integrate-your-github-repository-to-your-jenkins-project](https://www.blazemeter.com/blog/how-to-integrate-your-github-repository-to-your-jenkins-project)

### Jenkins Installation

The server that Jenkins runs on must have the `arm-none-eabi-gcc` compiler installed.

```bash
sudo apt install gcc-arm-none-eabi
```

### Jenkins Config

#### Build

Execute shell

```bash
echo "Tool versions"
python --version
arm-none-eabi-gcc --version
echo "Starting build using arm-none-eabi-gcc ..."
# sh $WORKSPACE/toolchain.sh
# sh $WORKSPACE/jenkins-build.sh
python makefile-fix.py
cd $WORKSPACE/Debug
pwd
make all
echo "Done!"
```
