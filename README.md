# 🤖 RoboCapture
A photogrammetry robot designed from scratch to autonomously scan indoor spaces.

## 🛠 Development

- **Cross-Platform**: This repository is designed to be cross-platform, working on Windows, MacOS, and Linux. We leverage Docker and the devcontainer extension in VSCode to achieve this.
  
- **Testing**: Most of the development and testing have been done on Linux. However, it might require slight adjustments when setting up on other operating systems.

### 🔧 Pre-requisites

1. `$(hostname).local` should be broadcasted on the network using mDNS. This applies to both the robot and the development computer.
2. Docker must be running on a Linux environment.

### 🍎 MacOS Specifics

If you're on a Mac, you might need to adjust the Docker permissions:
```bash
sudo chown -R $(whoami) ~/.docker
```
This ensures Docker can run without requiring sudo every time.

### 🖥️ X11 Authentication

For enabling X11 forwarding, execute the following command on the host computer:
```bash
xhost +local:docker
```

### 📝 Notes
Please double-check the file permissions before making a commit to ensure they are appropriate, due to docker created files being owned by `root`. Fix shall be issued soon.
