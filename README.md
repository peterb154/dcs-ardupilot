# dcs-ardupilot
DCS Integration with Ardupilot

## requirements
- git
- virtualbox
- vagrant
- DCS Installed

## installation
1. Clone this repo into ~/Saved Games/DCS.xxx/Scripts/
   ```
   cd ~/Saved Games/DCS.xxx/Scripts/
   git clone git@github.com:ArduPilot/ardupilot.git
   ```
2. Update ~/Saved Games/DCS.xxx/Scripts/Export.lua
    ```
    dofile(lfs.writedir()..[[Scripts\dcs-ardupilot\dcs-ardupilot.lua]])
    ```
3. Clone ardupilot project into `~/Projects`
4. run vagrant up from ardupilot project 
