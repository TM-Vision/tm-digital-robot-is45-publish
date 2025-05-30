# Isaac Sim Installation guide for Windows

## Download the TM Digital Robot Extension

```bash
mkdir %USERPROFILE%\projects
cd %USERPROFILE%\projects
git clone https://github.com/tm-vision/tm-digital-robot-is45-publish

```

-   Please checkout the latest or specific version and then create new branch for your custom development
-   Checkout the latest version, you can use the command below

```bash
cd %USERPROFILE%\projects\tm-digital-robot-is45-publish
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
mklink /J exts %USERPROFILE%\isaac-sim-4.5\exts
mklink /J extscache %USERPROFILE%\isaac-sim-4.5\extscache
mklink /J kit %USERPROFILE%\isaac-sim-4.5\kit
```

### Install other required packages

```bash
cd %USERPROFILE%\projects\tm-digital-robot-is45-publish
%USERPROFILE%\isaac-sim-4.5\kit\python\python.exe -m pip install --upgrade pip
%USERPROFILE%\isaac-sim-4.5\kit\python\python.exe -m pip install --isolated --no-cache-dir --no-deps -r requirements.txt
```

### Start Isaac Sim

```bash
cd %USERPROFILE%\isaac-sim-4.5
isaac-sim.bat
```

-   If everything alright, you should see the Isaac Sim window, it means you have successfully installed Isaac Sim

    ![](images/20250417152028.png)

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
