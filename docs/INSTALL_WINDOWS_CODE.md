# Extension Code Setup for Window

## 1. Clone the TM Digital Robot Extension Code

-   Create a directory for your projects and clone the repository:

```bash
mkdir %USERPROFILE%\projects
cd %USERPROFILE%\projects
git clone https://github.com/tm-vision/tm-digital-robot-is45-publish
```

-   Check out the latest or a specific version, then create a new branch for custom development:

```bash
cd %USERPROFILE%\projects\tm-digital-robot-is45-publish
git checkout v2.23.1
git branch v2.23.1_custom
git checkout v2.23.1_custom
```

## 2. Link Isaac Sim SDK

-   For easier development, link the Isaac Sim SDK to the project directory:

```bash
cd %USERPROFILE%\projects\tm-digital-robot-is45-publish
mkdir sdk_isaacsim
cd sdk_isaacsim
mklink /J exts %USERPROFILE%\isaac-sim-4.5\exts
mklink /J extscache %USERPROFILE%\isaac-sim-4.5\extscache
mklink /J kit %USERPROFILE%\isaac-sim-4.5\kit
```

## 3. Install Required Python Modules

-   Install the necessary Python modules by running the following commands:

```bash
cd %USERPROFILE%\projects\tm-digital-robot-is45-publish
%USERPROFILE%\isaac-sim-4.5\kit\python\python.exe -m pip install --no-warn-script-location --upgrade pip
%USERPROFILE%\isaac-sim-4.5\kit\python\python.exe -m pip install --no-warn-script-location --isolated --no-cache-dir --no-deps -r requirements.txt
```

## Open the source code by Visual Studio Code

-   Open your source code by command below

```bash
cd %USERPROFILE%\projects\tm-digital-robot-is45-publish
code .
```

-   You should see the Visual Studio Code window with the source code, the extension.py that allows you customize as you need

    ![](images/20250418102703.png)

## 4. Next Step

-   Continue to [Install Extension](INSTALL_EXTENSION.md) for further steps.
