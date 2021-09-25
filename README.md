# BSICS-SVR


BSICS-SVR a C++ firmware project implementing an ethernet-connnected controller for custom Test&Measurement Equipment. 


### Basic characteristics
* uses a STM32 Nucleo-F767ZI board (optionally Nucleo-F746ZG) 
* 100Base-TX ethernet interface with LAN8742A PHY
* uses FreeRTOS (CMSIS-RTOS v1 API), LwIP
* CLion (CMake) project
* SCPI server (default port 5025) via [libscpi](https://github.com/j123b567/scpi-parser/tree/master/libscpi)
* todo: webserver for setup, control and graphing (websockets, flot charts)

See "UM1974 STM32 Nucleo-144 boards (MB1137) 8.0" (available at  https://www.st.com/en/evaluation-tools/nucleo-f767zi.html#documentation) for "Hardware Layout and Configuration".

### Warnings

This project uses several rudimentary starting points which come with their own challenges. It is created as a new project for the Nucleo-F767ZI board, generated and configured using STM32CubeMX and appended by creating and copying files to a tidied-up project structure.

Changes and additions are made in an attempt to clean up the project and address known memory- and RTOS-specific issues. 

* the [scpi-parser](https://github.com/j123b567/scpi-parser) project kindly provides an SCPI server example which does not claim to be authoritative, just a convenient starting point. libscpi is not thread-safe - it requires a dedicated thread, the communication with which is queue-based.
* The ST-provided LwIP\_HTTP\_Server\_Netconn\_RTOS code example is outdated and contains fundamental flaws. It's unclear whether these are fixed at the time of this writing. 
* STM32CubeMX middleware can be assumed to contain bugs, thread-safety issues and memory protection / memory layout issues.

* ST-related ethernet issues ([memory layout and MPU configuration](https://community.st.com/s/question/0D50X0000C4Nk4GSQS/bug-missing-compiler-and-cpu-memory-barriers), caching, LWPI API, ...) are tracked at [https://community.st.com/s/question/0D50X0000BOtfhnSQB/how-to-make-ethernet-and-lwip-working-on-stm32](https://community.st.com/s/question/0D50X0000BOtfhnSQB/how-to-make-ethernet-and-lwip-working-on-stm32) , which is an excellent place to start (read: being thrown in at the deep end).


In the text below, project creation and composition are documented, along with the necessary attempts to resolve issues identified. The LwIP\_HTTP\_Server\_Netconn\_RTOS example was opened side-by-side and STM32CubeMX was used to create an editable project file which wasn't provided with the server example. ST notes the example is currently not compatible with (= outdated) CubeMX.


The author does not claim to be an expert and code is provided as-is. Corrections and suggestions are always welcome.

## The T&M Device Server Project

### Project Structure

* Applications/
	- SCPI_Server	
* Drivers/ 
	- BSP, CMSIS, HAL_Driver
* Middlewares/Third_Party/
	- FreeRTOS, libscpi (libscpi.a + headers), LwIP
* Src/
* Inc/
* startup/


## In-Depth Topics

* [Basic project creation with CLion and STM32CubeMX](doc/readme_project_creation.md)
	- [Introduction to the Cortex-M7 MPU](doc/readme_mpu.md)
	- [Modifications for Thread Safety (newlib)](doc/readme_newlib.md)
* debug ouput via USART and virtual COM port
* establishing and testing ethernet functionality
* adding libscpi
* I2C device support