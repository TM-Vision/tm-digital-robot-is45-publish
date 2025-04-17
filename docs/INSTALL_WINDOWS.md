# Isaac Sim Installation guide for Windows

## Download the TM Digital Robot Extension

```bash
mkdir %USERPROFILE%\projects
cd %USERPROFILE%\projects
git clone https://github.com/tm-vision/tm-digital-robot-is45-publish
cd %USERPROFILE%\projects\tm-digital-robot-is-45-publish
```

-   Please checkout the latest or specific version and then create new branch for your custom development
-   Checkout the latest version, you can use the command below

```bash
git checkout v2.23.1
git branch v2.23.1_custom
git checkout v2.23.1_custom
```

## Install the Isaac Sim

-   Follow the official instruction

## Link Isaac Sim SDK

```bash
cd %USERPROFILE%\projects\tm-digital-robot-is45-publish
mkdir sdk_isaacsim
cd sdk_isaacsim
mklink /D exts %USERPROFILE%\isaac-sim-4.5\exts"
mklink /D extscache %USERPROFILE%\isaac-sim-4.5\extscache"
mklink /D kit %USERPROFILE%\isaac-sim-4.5\kit"
```

-   Install python modules required

```bash
cd %USERPROFILE%\projects\tm-digital-robot-is45-publish
%USERPROFILE%\isaac-sim-4.5\kit\python\python.exe -m pip install --upgrade pip
%USERPROFILE%\isaac-sim-4.5\kit\python\python.exe -m pip install --isolated --no-cache-dir --no-deps -r requirements.txt
```

**HINT**: If you encounter issues of **this system does not have Windows Long Path support enabled**, please refer to [Enable long paths in Windows 10, version 1607, and later](https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=powershell#enable-long-paths-in-windows-10-version-1607-and-later) to solve the issue.

### Install other required packages

```bash
pip install -r requirements.txt
```

### Start Isaac Sim

-   Make sure you are in the virtual environment env_isaacsim

```bash
cd %USERPROFILE%\projects\tm-digital-robot-publish
env_isaacsim\Scripts\activate
```

-   Start Isaac Sim by command below

```bash
isaacsim omni.isaac.sim
```

-   You will see the License Agreement at the first time you start Isaac Sim, key in `y` if you agree with the License Agreement

    ![](images/20241211160639.png)

-   After while, you will see the Isaac Sim window

    ![](images/20241211113858.png)

-   If you can see the Isaac Sim window, it means you have successfully installed Isaac Sim.

-   It's good idea to create a shortcut named isaacsim.bat for the Isaac Sim on your desktop for easy access, please copy the script below and save it as isaacsim.bat

    ![](images/20241231140640.png)

    ```bash
    @echo off
    cd %USERPROFILE%\projects\tm-digital-robot-publish
    call env_isaacsim\Scripts\activate
    isaacsim omni.isaac.sim
    pause
    ```

## Open the source code by Visual Studio Code

-   Open your source code by command below

```bash
cd %USERPROFILE%\projects\tm-digital-robot-publish
code .
```

-   You should see the Visual Studio Code window with the source code, the extension.py that allows you customize as you need

    ![](images/20241231165335.png)

## Next step

-   Next, please go to [Installation of TM-Digital Robot Extension](INSTALL_EXTENSION.md) for the following step
