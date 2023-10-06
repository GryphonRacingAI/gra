# Development
## MacOS
On Mac, you may need to run `sudo chown -R $(whoami) ~/.docker` to allow docker to run without sudo.
## X11 auth
To enable X11 forwarding, run `xhost +local:docker` on host computer
## Pre-requisites
- $(hostname).local is broadcasted on the network via mDNS, for both bot and dev computer
- Docker needs to be run on linux
## Notes
- Please make sure the permissions are correct before committing