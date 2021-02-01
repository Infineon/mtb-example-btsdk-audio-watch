# Watch app

## Overview
This app demonstrates Bluetooth A2DP source, AVRCP Controller/Target, Apple Media Service (AMS), Apple Notification Center Service (ANCS), and HFP Audio Gateway/Hands-free Unit.

Features demonstrated:

 - WICED BT A2DP Source APIs
 - WICED BT AVRCP (Controller/Target) APIs
 - WICED BT GATT APIs
 - Apple Media Service and Apple Notification Center Services (AMS and ANCS)
 - Handling of the UART/SPI WICED protocol
 - SDP and GATT descriptor/attribute configuration
 - WICED BT SCO/RFCOMM initiator APIs and HFP Audio Gateway role
 - HFP Hands-free Unit role

## Instructions
To demonstrate the app, follow these steps:

1. Build and download the application to the WICED board.
2. Open the BT/BLE Profile ClientControl application
     - [UART] Open the port for WICED HCI for the device.(Default baud rate configured in the application is defined by the BSP HCI\_UART\_DEAULT\_BAUD #define, usually either 3M or 115200 depending on the board UART capabilities.)
     - [SPI] Open the port for WICED PUART for the SPI master device (using 115200 baudrate and without flow control). Refer the instructions below for SPI setup (supported on 20719B2/20721B2 only)
3. Use the ClientControl application to send various commands as mentioned below.
4. Run the BTSpy program to view protocol and application traces.

## SPI as transport instead of UART

1. Download uart_spi_bridge application on a 20706A2 board.
2. Connect SPI pins as indicatd below,

    Master (20706A2):

    - SPI CLK     - P36(J19.6)
    - SPI MOSI    - P0(J22.6)
    - SPI MISO    - P25(J19.4)
    - SPI CS      - P26(J22.4)
    - SLAVE READY - P12(J22.7)
    - GND         - J19.8

    Slave:

    - SPI CLK     - P38(D13)
    - SPI MOSI    - P28(D11)
    - SPI MISO    - P01(D12)
    - SPI CS      - P07(D10)
    - SLAVE READY - P06(D8)
    - GND         - J3.4

3. Ensure that pin config array has the following pins configured correctly,

    [PLATFORM_GPIO_1] = {WICED_P01, spi_0_miso_0_TRIGGER_IN},
    [PLATFORM_GPIO_4] = {WICED_P06, WICED_SPI_1_SLAVE_READY},
    [PLATFORM_GPIO_5] = {WICED_P07, spi_0_cs_0_TRIGGER_IN},
    [PLATFORM_GPIO_11] = {WICED_P28, spi_0_mosi_0_TRIGGER_IN},
    [PLATFORM_GPIO_15] = {WICED_P38, spi_0_clk_0_TRIGGER_IN},

BR/EDR Audio Source and AVRC Target:

- The Watch app can demonstrate how use to BR/EDR Audio Source and AVRC TG profiles.
- Audio Source can use I2S interrupt or SW timer to decide the timing to read PCM.
  For media type as 'I2S input', it will use I2S interrupt, and you need to configure 4 gpios as WICED I2S PINs.
  For media type as 'Wav file' or 'Sine wave', it will use SW timer by calling wiced\_audio\_use\_sw\_timing(1).
  In general, if using WICED HCI UART to transmit audio, it must either allocate I2S pins on unused pins for
  I2S interrupt OR use wiced\_audio\_use\_sw\_timing(1) to enable SW timer.
- Use the buttons in the ClientControl AV Source tab.
- To play a sine wave sample, set the audio frequency to the desired value (48kHz, 44.1kHz, etc.)
  and select the Media type as 'Sine Wave' in the UI. In this case, built-in sine wave audio is played.
- To play music from a .wav file, select the Media type as File, browse and select a .wav file,
  and set the audio frequency to the desired value (48kHz, 44.1kHz, etc.)
  In this case, audio for the .wav file is routed over WICED HCI UART to the WICED board.<br>
  sinc\_44100\_16\_L440\_R1000\_50s\_stereo.wav in app folder can be used as the input of 44.1KHz 16bits stereo samples.
- To play music from a .mp3 file, select the Media type as File, browse and select a .mp3 file,
  and set the audio frequency to the desired value (48kHz, 44.1kHz, etc.)
  In this case, audio for the .mp3 file is routed over WICED HCI UART to the WICED board.
  sinc\_44100\_mono.mp3, sinc\_44100\_stereo.mp3, sinc\_48000\_mono.mp3 and sinc\_48000\_stereo.mp3 in MP3\_sample folder
  can be used as the input of 44.1KHz/48kHz Mono/stereo samples.
  Use only the .mp3 files provided with the application in the 'MP3_samples' folder.
- To play music from the Line-In jack, select the Media type as 'I2S input' and set the
  audio frequency to the desired value (48kHz, 44.1kHz, etc.)
  In this case, audio from Line-In is encoded into I2S signals and routed to the WICED board.
- Put an audio sink device such as BT headphone/speaker in pairable mode.
- Click on the "Start" button from the "BR/EDR Discovery" combo box in ClientControl to find the audio sink device.
- Select the peer device in the BR/EDR Discovery combo box.
- Click the "Connect" button under the AV Source tab.
- Click the "Start Streaming" button. Music will start playing on the peer device.
- The WatchAma app uses the AVRCP Target role. Once connected to a headset/speaker,
  the app can send notifications for play status changes (Play, Pause, Stop) and
  settings changes (Repeat, Shuffle) to the peer AVRCP controller (such as a headset/speaker).<br/>
  Note: the songs shown in the AVRC TG UI and some settings such Repeat/Shuffle are for testing
  AVRC commands only, they do not indicate the actual media played and will not change the media played.

BR/EDR AVRCP Controller:

- The Watch app can demonstrate how to use the AVRC CT profile.
- Disconnect all devices if any are connected.
- Make an audio source device such as an iPhone discoverable/pairable from the Bluetooth Settings UI on the phone.
- Using the "BR/EDR Discovery" "Start" button, search and select the device.
- Use the buttons in the ClientControl AVRC CT tab to Connect and accept pairing.
- Play music on the audio source device and control the music via buttons in the AVRC CT tab.
- In Controller mode, pass-thru commands are executed via Play, Pause, Stop, etc. buttons.
- Absolute volume change can be done via the drop down Volume or Vol Up/Down buttons.
- Note that iPhone does does not support Vol buttons.
- Note that music will continue to play on the audio source device.

iOS ANCS and AMS GATT Services:

- The Watch app can demonstrate how to use AMS and ANCS iOS services as shown below.
- Disconnect all devices if any are connected.
- Select Pairable if it is not checked.
- Click the "Start Adverts" button in the GATT tab.
- From an iPhone app such as 'LightBlue', find and connect to the 'WatchAmaLE' app.
- Allow pairing with the iPhone.
- AMS:
  - Play media on the iPhone.
  - Use buttons in the ClientControl AVRC CT tab to control the music.
  - Note that music will continue to play on iPhone.
- ANCS:
  - Incoming calls and messages to the iPhone will be displayed on the ANCS buttons.
  - Make an incoming call to your iPhone. See a call notification displayed on the UI to accept or reject the call. Similarly, missed call notifications are seen.
  - Send an SMS message to your iPhone to see a message notification.

BLE Client:

- The Watch app can demonstrate BLE Client functionality as shown below.
- Make sure there is a BT device with GATT services that is advertising. For example use an app
  such as 'LightBlue' on your phone and create a 'Virtual Peripheral' such as 'Blood Pressure'.
- To find GATT devices:
  - Click on the "Start" button for the "BLE Discovery" combo box.
  - Click on the "Stop" button to end discovery.
- To connect a BLE device:
  - Choose a device from the "BLE Discovery" drop down combo box and click the "Connect" button.
- To discover services: Click on the "Discover Services" button
- To discover characteristics: Enter the handles in the edit box and click
  on "Discover Characteristics"
- To discover descriptors: Enter the handles in the edit box and click on
  "Discover Descriptors"
- Enter the Handle and Hex Value to write to the remote device using buttons:
  - "Write" : Write a hex value to the remote handle
  - "Write no rsp" : Write a hex value without a response to the remote handle
  - "Value Notify" : Write a notification value to the remote handle
  - "Value Indicate" : Write an indication value to the remote handle

HFP Audio Gateway:

- These targets support HFP Audio Gateway:
  CYW920721B2EVK-02, CYW920721M2EVK-01, CYW920721M2EVK-02, CYW9M2BASE-43012BT and CYW943012BTEVK-01
- Build with "HFP\_AG\_INCLUDED=1" to enable AG. (disables Hands-free Unit simultaneously)
- The Watch app can demonstrate how to use HFP AG as shown below.
- Make a HFP Headset (headphone or earbuds) discoverable and pairable by its specific behavior.
- In ClientControl, click on the "Start" button from the "BR/EDR Discovery" combo box to find the Headset device.
- Select the peer device in the BR/EDR Discovery combo box.
- Click the "Connect" button under the ClientControl AG tab.
- Click the "Audio Connect" button. The AG will create a SCO connection to the Headset, wide-band speech is supported.
- Click the "Audio Disconnect" button to remove the SCO connection.

HFP Hands-free Unit:

- These targets support HFP Hands-free Unit by default:
  CYW920721B2EVK-02, CYW920721M2EVK-01, CYW920721M2EVK-02, CYW9M2BASE-43012BT and CYW943012BTEVK-01
- To create a hands-free connection with a remote Audio Gateway (AG) device (such as a mobile phone), use ClientControl and choose the Bluetooth address of the remote AG device from the BR/EDR combo box.<br/>
  Click the "Connect" button under HF tab.
- OR Put the device in discoverable and connectable mode and search for the device from the AG device and connect.
- The following HF operations can be performed using the ClientControl HF tab
   -  Connect / Disconnect the HF or SCO connection
   -  Answer / Hang-up the call
   -  Dial / Redial the number
   -  Control Held calls ( Only support "Release all held", "Release active accept other", "Place active on hold and accept other", "Add held to conversation( It’s functionality depends on the telecom network operator, if the telecom network side support the feature, the function will work. AG always supports this feature and responses OK)")
   -  Mic / Speaker gain control

## Application Settings
Application specific settings are as shown below:

- SLEEP\_SUPPORTED
    - This option allows the device to enter low power mode. By default the option is off. When sleep is enabled, ClientControl will not be able to communicate with the embedded app unless a GPIO is asserted.
    - 43012C0-related target (CYW9M2BASE-43012BT and CYW943012BTEVK-01) does not support this functionality.

- COEX\_SUPPORTED
    - This option enables BT and Wi-Fi Coexistence. By default the option is off.

- OTA\_FW\_UPGRADE
    - Use this option for OTA firmware upgrade
    - 43012C0-related target (CYW9M2BASE-43012BT and CYW943012BTEVK-01) does not support this functionality.

- OTA\_SEC\_FW\_UPGRADE
    - Use this option for secure OTA firmware upgrade
    - 43012C0-related target (CYW9M2BASE-43012BT and CYW943012BTEVK-01) does not support this functionality.

## Common application settings

Application settings below are common for all BTSDK applications and can be configured via the makefile of the application or passed in via the command line.

- BT\_DEVICE\_ADDRESS<br/>
    - Set the BDA (Bluetooth Device Address) for your device. The address is 6 bytes, for example, 20819A10FFEE. By default, the SDK will set a BDA for your device by combining the 7 hex digit device ID with the last 5 hex digits of the host PC MAC address.

- UART<br/>
    - Set to the UART port you want to use to download the application. For example 'COM6' on Windows or '/dev/ttyWICED\_HCI\_UART0' on Linux or '/dev/tty.usbserial-000154' on macOS. By default, the SDK will auto-detect the port.

- ENABLE_DEBUG<br/>
    - For HW debugging, select the option '1'. See the document [WICED-Hardware-Debugging](https://github.com/cypresssemiconductorco/btsdk-docs/blob/master/docs/BT-SDK/WICED-Hardware-Debugging.pdf) for more information. This setting configures GPIO for SWD.
      - CYW920819EVB-02/CYW920820EVB-02: SWD signals are shared with D4 and D5, see SW9 in schematics.
      - CYBT-213043-MESH/CYBT-213043-EVAL : SWD signals are routed to P02=SWDCK and P03=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
	  - CYBT-223058-EVAL : SWD signals are routed to P02=SWDCK and P03=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
	  - CYBT-243053-EVAL: SWD signals are routed to P12=SWDCK and P13=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
	  - CYBT-253059-EVAL: SWD signals are routed to P12=SWDCK and P13=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
      - CYW989820EVB-01: SWDCK (P02) is routed to the J13 DEBUG connector, but not SWDIO. Add a wire from J10 pin 3 (PUART CTS) to J13 pin 2 to connect GPIO P10 to SWDIO.
      - CYW920719B2Q40EVB-01: PUART RX/TX signals are shared with SWDCK and SWDIO. Remove RX and TX jumpers on J10 when using SWD. PUART and SWD cannot be used simultaneously on this board unless these pins are changed from the default configuration.
      - CYW920721B2EVK-02: SWD hardware debugging supported. SWD signals are shared with D4 and D5, see SW9 in schematics.
      - CYW920721B2EVK-03, CYW920721M2EVK-01: SWD hardware debugging is not supported.
      - CYW920721M2EVK-02: SWD hardware debugging is supported. The default setup uses P03 for SWDIO and P05 for SWDCK.
      - CYW920706WCDEVAL: SWD debugging requires fly-wire connections. The default setup uses P15 (J22 pin 3) for SWDIO and P30 (J19 pin 2) for SWDCK. P30 is shared with BTN1.
      - CYW920735Q60EVB-01: SWD hardware debugging supported. The default setup uses the J13 debug header, P3 (J13 pin 2) for SWDIO and P2 (J13 pin 4) for SWDCK.  They can be optionally routed to D4 and D4 on the Arduino header J4, see SW9 in schematics.
      - CYW920835REF-RCU-01: SWD hardware debugging is not supported.
      - CYW9M2BASE-43012BT: SWD hardware debugging is not supported.
      - CYW943012BTEVK-01: SWD hardware debugging is not supported.

## Building code examples

**Using the ModusToolbox IDE**

1. Install ModusToolbox 2.2.
2. In the ModusToolbox IDE, click the **New Application** link in the Quick Panel (or, use **File > New > ModusToolbox IDE Application**).
3. Pick your board for BTSDK under WICED Bluetooth BSPs.
4. Select the application in the IDE.
5. In the Quick Panel, select **Build** to build the application.
6. To program the board (download the application), select **Program** in the Launches section of the Quick Panel.


**Using command line**

1. Install ModusToolbox 2.2
2. On Windows, use Cygwin from \ModusToolbox\tools_2.2\modus-shell\Cygwin.bat to build apps.
3. Use the tool 'project-creator-cli' under \ModusToolbox\tools_2.2\project-creator\ to create your application.<br/>
   > project-creator-cli --board-id (BSP) --app-id (appid) -d (dir) <br/>
   See 'project-creator-cli --help' for useful options to list all available BSPs, and all available apps per BSP.<br/>
   For example:<br/>
   > project-creator-cli --app-id mtb-example-btsdk-empty --board-id CYW920706WCDEVAL -d .
4. To build the app call make build. For example:<br/>
   > cd mtb-examples-btsdk-empty<br/>
   > make build<br/>
6. To program (download to) the board, call:<br/>
   > make qprogram<br/>
7. To build and program (download to) the board, call:<br/>
   > make program<br/>

   Note: make program = make build + make qprogram


## Downloading an application to a board

If you have issues downloading to the board, follow the steps below:

- Press and hold the 'Recover' button on the board.
- Press and hold the 'Reset' button on the board.
- Release the 'Reset' button.
- After one second, release the 'Recover' button.

Note: this is only applicable to boards that download application images to FLASH storage. Boards that only support RAM download (DIRECT_LOAD) such as CYW9M2BASE-43012BT can be power cycled to boot from ROM.

## SDK software features

- Dual-mode Bluetooth stack included in the ROM (BR/EDR and LE)
- Bluetooth stack and profile level APIs for embedded Bluetooth application development
- WICED HCI protocol to simplify host/MCU application development
- APIs and drivers to access on-board peripherals
- Bluetooth protocols include GAP, GATT, SMP, RFCOMM, SDP, AVDT/AVCT, LE Mesh
- LE and BR/EDR profile APIs, libraries, and sample apps
- Support for Over-The-Air (OTA) upgrade
- Device Configurator for creating custom pin mapping
- Bluetooth Configurator for creating LE GATT Database
- Peer apps based on Android, iOS, Windows, etc. for testing and reference
- Utilities for protocol tracing, manufacturing testing, etc.
- Documentation for APIs, datasheets, profiles, and features
- BR/EDR profiles: A2DP, AVRCP, HFP, HSP, HID, SPP, MAP, PBAP, OPP
- LE profiles: Mesh profiles, HOGP, ANP, BAP, HRP, FMP, IAS, ESP, LE COC
- Apple support: Apple Media Service (AMS), Apple Notification Center Service (ANCS), iBeacon, Homekit, iAP2
- Google support: Google Fast Pair Service (GFPS), Eddystone
- Amazon support: Alexa Mobile Accessories (AMA)

Note: this is a list of all features and profiles supported in BTSDK, but some WICED devices may only support a subset of this list.

## List of boards available for use with BTSDK

- [CYW20819A1 chip](https://github.com/cypresssemiconductorco/20819A1)
    - [CYW920819EVB-02](https://github.com/cypresssemiconductorco/TARGET_CYW920819EVB-02), [CYBT-213043-MESH](https://github.com/cypresssemiconductorco/TARGET_CYBT-213043-MESH), [CYBT-213043-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-213043-EVAL), [CYW920819REF-KB-01](https://github.com/cypresssemiconductorco/TARGET_CYW920819REF-KB-01)
- [CYW20820A1 chip](https://github.com/cypresssemiconductorco/20820A1)
    - [CYW920820EVB-02](https://github.com/cypresssemiconductorco/TARGET_CYW920820EVB-02), [CYW989820EVB-01](https://github.com/cypresssemiconductorco/TARGET_CYW989820EVB-01), [CYBT-243053-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-243053-EVAL)
- [CYW20721B2 chip](https://github.com/cypresssemiconductorco/20721B2)
    - [CYW920721B2EVK-02](https://github.com/cypresssemiconductorco/TARGET_CYW920721B2EVK-02), [CYW920721B2EVK-03](https://github.com/cypresssemiconductorco/TARGET_CYW920721B2EVK-03), [CYW920721M2EVK-01](https://github.com/cypresssemiconductorco/TARGET_CYW920721M2EVK-01), [CYW920721M2EVK-02](https://github.com/cypresssemiconductorco/TARGET_CYW920721M2EVK-02), [CYBT-423060-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-423060-EVAL), [CYBT-483062-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-483062-EVAL), [CYBT-413061-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-413061-EVAL)
- [CYW20719B2 chip](https://github.com/cypresssemiconductorco/20719B2)
    - [CYW920719B2Q40EVB-01](https://github.com/cypresssemiconductorco/TARGET_CYW920719B2Q40EVB-01), [CYBT-423054-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-423054-EVAL), [CYBT-413055-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-413055-EVAL), [CYBT-483056-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-483056-EVAL)
- [CYW20706A2 chip](https://github.com/cypresssemiconductorco/20706A2)
    - [CYW920706WCDEVAL](https://github.com/cypresssemiconductorco/TARGET_CYW920706WCDEVAL), [CYBT-353027-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-353027-EVAL), [CYBT-343026-EVAL](https://github.com/cypresssemiconductorco/TARGET_CYBT-343026-EVAL)
- [CYW20735B1 chip](https://github.com/cypresssemiconductorco/20735B1)
    - [CYW920735Q60EVB-01](https://github.com/cypresssemiconductorco/TARGET_CYW920735Q60EVB-01)
- [CYW20835B1 chip](https://github.com/cypresssemiconductorco/20835B1)
    - [CYW920835REF-RCU-01](https://github.com/cypresssemiconductorco/TARGET_CYW920835REF-RCU-01)
- [CYW43012C0 chip](https://github.com/cypresssemiconductorco/43012C0)
    - [CYW9M2BASE-43012BT](https://github.com/cypresssemiconductorco/TARGET_CYW9M2BASE-43012BT), [CYW943012BTEVK-01](https://github.com/cypresssemiconductorco/TARGET_CYW943012BTEVK-01)

## Folder structure

All BTSDK code examples need the 'mtb\_shared\wiced\_btsdk' folder to build and test the apps. 'wiced\_btsdk' includes the 'dev-kit' and 'tools' folders. The contents of the 'wiced\_btsdk' folder will be automatically populated incrementally as needed by the application being used.

**dev-kit**

This folder contains the files that are needed to build the embedded Bluetooth apps.

* baselib: Files for chips supported by BTSDK. For example CYW20819, CYW20719, CYW20706, etc.

* bsp: Files for BSPs (platforms) supported by BTSDK. For example CYW920819EVB-02, CYW920721B2EVK-02, CYW920706WCDEVAL etc.

* btsdk-include: Common header files needed by all apps and libraries.

* btsdk-tools: Build tools needed by BTSDK.

* libraries: Profile libraries used by BTSDK apps such as audio, LE, HID, etc.

**tools**

This folder contains tools and utilities need to test the embedded Bluetooth apps.

* btsdk-host-apps-bt-ble: Host apps (Client Control) for LE and BR/EDR embedded apps, demonstrates the use of WICED HCI protocol to control embedded apps.

* btsdk-host-peer-apps-mesh: Host apps (Client Control) and Peer apps for embedded Mesh apps, demonstrates the use of WICED HCI protocol to control embedded apps, and configuration and provisioning from peer devices.

* btsdk-peer-apps-ble: Peer apps for embedded LE apps.

* btsdk-peer-apps-ota: Peer apps for embedded apps that support Over The Air Firmware Upgrade.

* btsdk-utils: Utilities used in BTSDK such as BTSpy, wmbt, and ecdsa256.

See README.md in the sub-folders for more information.

## ModusToolbox Tools

Source code generation tools installed by ModusToolbox installer:

- Device Configurator:
      A GUI tool to create custom pin mappings for your device.
- Bluetooth Configurator:
      A GUI tool to create and configure the LE GATT Database and BR/EDR SDP records for your application.

## Using BSPs (platforms)

BTSDK BSPs are located in the \mtb\_shared\wiced\_btsdk\dev-kit\bsp\ folder by default.

#### a. Selecting an alternative BSP

The application makefile has a default BSP. See "TARGET". The makefile also has a list of other BSPs supported by the application. See "SUPPORTED_TARGETS". To select an alternative BSP, use Library Manager from the Quick Panel to deselect the current BSP and select an alternate BSP. Then right-click the newly selected BSP and choose 'Set Active'.  This will automatically update TARGET in the application makefile.

#### b. Custom BSP

**Complete BSP**

To create and use a complete custom BSP that you want to use in applications, perform the following steps:

1. Select an existing BSP you wish to use as a template from the list of supported BSPs in the mtb\_shared\wiced\_btsdk\dev-kit\bsp\ folder.
2. Make a copy in the same folder and rename it. For example mtb\shared\wiced\_btsdk\dev-kit\bsp\TARGET\_mybsp.<br/>
   **Note:** This can be done in the system File Explorer and then refresh the workspace in Eclipse to see the new project.  Delete the .git sub-folder from the newly copied folder before refreshing in Eclipse.
   If done in the IDE, an error dialog may appear complaining about items in the .git folder being out of sync.  This can be resolved by deleting the .git sub-folder in the newly copied folder.

3. In the new mtb\_shared\wiced\_btsdk\dev-kit\bsp\TARGET\_mybsp folder, rename the existing/original (BSP).mk file to mybsp.mk.
4. In the application makefile, set TARGET=mybsp and add it to SUPPORTED\_TARGETS as well as TARGET\_DEVICE\_MAP.  For example: mybsp/20819A1
5. Update design.modus for your custom BSP if needed using the **Device Configurator** link under **Configurators** in the Quick Panel.
6. Update the application makefile as needed for other custom BSP specific attributes and build the application.

**Custom Pin Configuration Only - Multiple Apps**

To create a custom pin configuration to be used by multiple applications using an existing BSP that supports Device Configurator, perform the following steps:

1. Create a folder COMPONENT\_(CUSTOM)\_design\_modus in the existing BSP folder. For example mtb\_shared\wiced\_btsdk\dev-kit\bsp\TARGET\_CYW920819EVB-02\COMPONENT\_my\_design\_modus
2. Copy the file design.modus from the reference BSP COMPONENT\_bsp\_design\_modus folder under mtb\_shared\wiced\_btsdk\dev-kit\bsp\ and place the file in the newly created COMPONENT\_(CUSTOM)\_design\_modus folder.
3. In the application makefile, add the following two lines<br/>
   DISABLE\_COMPONENTS+=bsp\_design\_modus<br/>
   COMPONENTS+=(CUSTOM)\_design\_modus<br/>
   (for example COMPONENTS+=my\_design\_modus)
4. Update design.modus for your custom pin configuration if needed using the **Device Configurator** link under **Configurators** in the Quick Panel.
5. Building of the application will generate pin configuration source code under a GeneratedSource folder in the new COMPONENT\_(CUSTOM)\_design\_modus folder.

**Custom Pin Configuration Only - Per App**

To create a custom configuration to be used by a single application from an existing BSP that supports Device Configurator, perform the following steps:

1. Create a folder COMPONENT\_(BSP)\_design\_modus in your application. For example COMPONENT\_CYW920721B2EVK-03\_design\_modus
2. Copy the file design.modus from the reference BSP under mtb\_shared\wiced\_btsdk\dev-kit\bsp\ and place the file in this folder.
3. In the application makefile, add the following two lines<br/>
   DISABLE\_COMPONENTS+=bsp\_design\_modus<br/>
   COMPONENTS+=(BSP)\_design\_modus<br/>
   (for example COMPONENTS+=CYW920721B2EVK-03\_design\_modus)
4. Update design.modus for your custom pin configuration if needed using the **Device Configurator** link under **Configurators** in the Quick Panel.
5. Building of the application will generate pin configuration source code under the GeneratedSource folder in your application.


## Using libraries

The libraries needed by the app can be found in in the mtb\_shared\wiced\_btsdk\dev-kit\libraries folder. To add an additional library to your application, launch the Library Manager from the Quick Panel to add a library. Then update the makefile variable "COMPONENTS" of your application to include the library. For example:<br/>
   COMPONENTS += fw\_upgrade\_lib


## Documentation

BTSDK API documentation is available [online](https://cypresssemiconductorco.github.io/btsdk-docs/BT-SDK/index.html)

Note: For offline viewing, git clone the [documentation repo](https://github.com/cypresssemiconductorco/btsdk-docs)

BTSDK Technical Brief and Release Notes are available [online](https://community.cypress.com/community/software-forums/modustoolbox-bt-sdk)
