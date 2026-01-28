# zephyr-spi-target-demo
Application testing for Zephyr ESP32 SPI Target API contributions.

## Running the example apps
Ensure that the virtual environment is active and you are in the base directory `zephyr-spi-target-demo`.
`source .venv/bin/activate`

For the SPI master app, run:
`just run-esp32-spi-example`

For the SPI slave app, run:
`just run-esp32-spi-module`

For the SPI bit-bang slave app, run:
`just run-esp32-spi-bitbang`

## Setup

_Note: These instructions are specific to macOS_

1. Install prerequisites

    - [Python3](https://www.geeksforgeeks.org/python/download-and-install-python-3-latest-version/)

    - Complete **only** the [Install Dependencies](https://docs.zephyrproject.org/latest/develop/getting_started/index.html#install-dependencies) section of the [Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html#getting-started-guide).

    - [Just command runner](https://github.com/casey/just?tab=readme-ov-file#installation) - This is used like a makefile as a hotkey for all of our project-specific commands
        - If you are using Windows, installing chocolatey package manager is recommended. After installing chocolatey, you can install just by running `choco install just`.
        - If you are on Mac, use the [Homebrew](https://brew.sh/) command `brew install just`.

    - Windows + WSL2: [usbipd-win](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) - This is used to expose the USB ports to WSL2 so that flashing works properly

    - [Protobuf compiler](https://protobuf.dev/installation/) 
        - If using WSL/Ubuntu: `sudo apt install -y protobuf-compiler`
        - If using mac: `brew install protobuf`

2. Clone this repository

3. Create and activate a Python virtual environment:
    ```bash
    python -m venv .venv
    
    source .venv/bin/activate     # Linux/macOS
    .venv\Scripts\activate.bat    # PC, Command Prompt
    .venv\Scripts\Activate.ps1    # PC, Powershell
    source .venv/Scripts/activate # PC. Git bash
    ```

4. Install west
    ```bash
    pip install west
    ```

5. Update the workspace
    ```bash
    west update
    west packages pip --install
    ```

6. Install the Zephyr SDK
    1. navigate to `zephyr/`
        ```bash
        cd zephyr
        ```
    2.  Perform installation 
        ```bash
        west sdk install
        ```
        
7. Test flash
    1. navigate to `zephyr/`
    2. Plug in board and run:
       1. ```bash
          ls /dev/ | grep tty
          ```
       2. You should see a list of connected devices. We are looking for something similar to either of the following: 
          ```bash
          tty.usbserial-210
          tty.usbmodem2101
          ```
          usbserial will appear if you're connected to the expressif's UART path, while usbmodem will appear if you're connected to its USB path
       3. Copy the device name (eg. `tty.usbserial-210`)
       4. Navigate to `justfile` and replace the value in the `ESP_PORT` line with your device name. It will now be able to communicate to the board using this name.
          ```
          ESP_PORT         := "/dev/tty.usbserial-210"
          ```
8. Test Debugger
     1. Ensure you are plugged into the UART connection (the debugger)
     2. `just run-esp32`
     3. Once the board is done flashing, press the `reset` button
