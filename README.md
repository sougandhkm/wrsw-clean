
This package is meant to include all white-rabbit software for the
switch.  Part of this comes from the original white-rabbit svn
repository on ohwr.org, part comes from separate repositories that I
merged here.

All documentation lives in doc/ , where the *.in files are the sources
(one only, currently, other ones have been removed when obsoleted).

The wrs-build documentation, thus, is what you actually need to build
all the stuff that goes into the switch, and program to internal flash
memory.


Modifications by Rodrigo and Sougandh

This branch contains software enhancements to support **low-jitter White Rabbit switches**.

### Monitoring port

A new port type is added to the supported ports types.
You can enable it by setting the following in the `dot-config` file for the relevant port:

` DESIRED_STATE = "TIMESCALE_SLAVE" `

This port performs **PTP exchanges** but does **not** participate in the soft-PLL locking scheme.  
It is intended for comparing external clocks connected to the White Rabbit device against the local time.

To monitor values, use:

` $ wr-mon -e `

This prints monitored values for every servo running.  
Our main parameter of interest is **`cko`** (clock offset), which is derived from both **PTP exchanges** and **DDMTD measurements**.
### Applying correction

User-defined corrections can be supplied to the PTP servo through the *ppsi-conf*  in the **ppsi/ tools**.

Run:

`$ ppsi-conf $offset_in_Pico_Seconds

- The **first call** to `ppsi-conf` stops phase tracking of the master by the switch.
    
- Subsequent calls provide custom phase corrections in picoseconds.

### DDMTD Phase tags
#### Enabling Tracking

We provide a custom utility to facilitate printing of phase tracking values.  
Enable it by running:
```
$ /wr/bin/wr_phytool {port} ts set_navg 3800 
$ /wr/bin/wr_phytool {port} ts track
```
- `<port>` = the port number you are connected to.
    
- Average sample rate is calculated as:
`sample_rate = 3814 / n_avg`


#### Printing values

Once tracking is enabled, run:
```
$ /wr/bin/lm32-vuart
```

This prints phase tracking values directly in the shell.  
**Note:** Port numbers are displayed as `(port_number – 1)`.

