# Telescope Power Monitor

## Features
* Monitor battery voltage
* Monitor battery load (amperage)


## Components
* GUI
* Firmware
* Hardware


## Recent changes
List of changes can be found in the [changelog](changelog.md).


## To-Do List
* Add emergency battery shutoff voltage (power down PC, turn off dehumidifier and telescope) [`Firmware`, `GUI`] 
* Add JSON API webserver [`GUI`]
* Add an "imaging" mode that disables dehumidifier and emergency shutoff [`Firmware`, `GUI`]
* Add config QC checks [`Firmware`]


### Ideas that may be implemented in the future
* Switch to ethernet board
* Add voltage sensor for solar input


### Ideas unlikely to be implemented:
* Blink LED on battery low voltage (configurable) [`Firmware`, `Hardware`]
* Blink LED on battery high current draw (configurable) [`Firmware`, `Hardware`]


## License
Telescope Power Monitor is licensed under the GNU General Public License (GPL) v2 or later

Copyright © 2016-2021 Steve Cross

>  This program is free software; you can redistribute it and/or modify
>  it under the terms of the GNU General Public License as published by
>  the Free Software Foundation; either version 2 of the License, or
>  (at your option) any later version.
>  
>  This program is distributed in the hope that it will be useful,
>  but WITHOUT ANY WARRANTY; without even the implied warranty of
>  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
>  GNU General Public License for more details.
>  
>  You should have received a copy of the GNU General Public License along
>  with this program; if not, write to the Free Software Foundation, Inc.,
>  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
