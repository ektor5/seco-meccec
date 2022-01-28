# CEC Linux Driver for SECO MEC-based Boards

Sources for SECO MEC-CEC Driver. Based on Linux CEC Framework and legacy seco-cec driver.

Compatible with:

* SECO SBC-D61
* UDOO BOLT (not tested)
* UDOO Vision (not tested)

## Requirements

* Linux Kernel >= 5.4 with `CEC_NOTIFIER` support

In order to use the driver, there are several methods:
* Use DKMS (Dynamic Kernel Module System)
* Compile and mount the module manually

## DKMS

DKMS is an automatic system for mounting and managing Linux external modules.
It is really useful in case of a kernel update or for automounting the module
at boot.

```bash
# install dependencies (make, git, ecc..)
sudo apt install git build-essential linux-headers-`uname -r` dkms

# clone repo
git clone <repo>

# go to dir
cd seco_meccec

# create dkms directory
sudo mkdir '/usr/src/seco_meccec-1.0'

# copy files
sudo cp seco-cec.? dkms.conf Makefile '/usr/src/seco_meccec-1.0'

# install module
sudo dkms install seco_meccec/1.0 -k `uname -r`
```

Now at reboot the module will be mounted automatically.

## Compile

```bash
# install dependencies (make, git, ecc..)
sudo apt install git build-essential linux-headers-`uname -r`

# clone repo
git clone <repo>

# go to dir
cd seco_meccec

# compile
make
```

The module should appear as `seco-meccec.ko`.

## Mount

This module depends on the Linux CEC Framework Module, and it needs to be loaded first.

```bash
#often built-in in kernel
modprobe cec
```

Load the module:

```bash
insmod seco-meccec.ko
```

Done!

## Use

To test the device from userspace (`/dev/cec0`, `/dev/cec1`, ... ), there are
several programs ready to use in [v4l-utils][v4l-utils] tools package
(available to install via apt):

* `cec-ctl`: An application to control cec devices
* `cec-compliance`: An application to verify remote CEC devices
* `cec-follower`: An application to emulate CEC followers

Check the corresponding man pages for info.

[v4l-utils]: https://git.linuxtv.org/v4l-utils.git/

## Debug

To load the module with debug messages:

``` bash
insmod seco-meccec.ko dyndbg
```

or

```bash
modprobe seco-meccec dyndbg
```

Feel free to open issues or mail me directly. Make sure to include *dmesg*,
*lsmod*, etc... in the bug report.

## Credits

Author: Ettore Chimenti  
Thanks to: Hans Verkuil, Andrea Petrini

Copyright (C) 2022, Seco Spa
