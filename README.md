# Development
## MacOS
On Mac, you may need to run `sudo chown -R $(whoami) ~/.docker` to allow docker to run without sudo.
## X11 auth
To enable X11 forwarding: `xhost +local:docker`