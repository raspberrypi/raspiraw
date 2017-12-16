Raspiraw: An example app that receives data directly from CSI sensors
on the Raspberry Pi.

The register sets for OV5647 and IMX219 are often under NDA, which means
that support can not be offered over their contents without breaking
those NDAs. Anything added here by RPF/RPT has to be demonstrable as
already being in the public domain, or from 3rd parties having
reverse engineered how the firmware is working (eg by listening to the
I2C communications).


The raw Bayer format frames captured by **raspiraw** can be converted to .ppm images by modified **dcraw** image processing app: [https://github.com/6by9/dcraw](https://github.com/6by9/dcraw)

Supported sensors:

	adv7282m
	imx219
	ov5647

## raspiraw command line options 

Table of contents:

	$ raspiraw
	
	raspiraw Camera App 0.0.1
	
	-?, --help	: This help information
	-md, --mode	: Set sensor mode <mode>
	-hf, --hflip	: Set horizontal flip
	-vf, --vflip	: Set vertical flip
	-e, --ss	: Set the sensor exposure time (not calibrated units)
	-g, --gain	: Set the sensor gain code (not calibrated units)
	-o, --output	: Set the output filename
	-hd, --header	: Write the BRCM header to the output file
	-t, --timeout	: Time (in ms) before shutting down (if not specified, set to 5s)
	-sr, --saverate	: Save every Nth frame
	-b, --bitdepth	: Set output raw bit depth (8, 10, 12 or 16, if not specified, set to sensor native)
	-c, --cameranum	: Set camera number to use (0=CAM0, 1=CAM1).
	-eus, --expus	: Set the sensor exposure time in micro seconds.
	-y, --i2c	: Set the I2C bus to use.
	-r, --regs	: Change (current mode) regs
	-hi, --hinc	: Set horizontal odd/even inc reg
	-vi, --vinc	: Set vertical odd/even inc reg
	-f, --fps	: Set framerate regs
	-w, --width	: Set current mode width
	-h, --height	: Set current mode height
	-hd0, --header0	: Sets filename to write the BRCM header to
	-ts, --tstamps	: Sets filename to write timestamps to
	-emp, --empty	: Write empty output files
	$

## base options

...

	-md, --mode	: Set sensor mode <mode>

Range is 1-7.

...

	-o, --output	: Set the output filename

For recording more than one frame specify "C printf integer format strings" as part of the filename.
Example: ... -o /dev/shm/out.%04d.raw ...

	-hd, --header	: Write the BRCM header to the output file

If selected, this prepeneds each stored frame with a 32KB header (needed for **dcraw** being able to process the raw frame). An alternative (needed for high framerate capturing) is option -hd0 (see below section), which allows **dcraw** to process the captured frames later as well.

...

	-sr, --saverate	: Save every Nth frame

Per default this is 20, allowing to capture frames to slow SD cards. In high framerate section, storing on (fast) ramdisk allows to use "-sr 1" (store all frames).

...

	-y, --i2c	: Set the I2C bus to use.

Range is 0-2.


## high frame rate options

Lookup ov5647, imx219 or adv7282m data sheets for register details.

#### I2C register setting options

	-r, --regs	: Change (current mode) regs

Allows to change sensor regs in selected sensor mode. Format is a semicolon separated string of assignments.
An assignment consists of a 4 hex digits register address, followed by a comma and one or more 2 hex digit byte values.
Restriction: Only registers present in selected sensor mode can be modified. Example argument: "380A,003C;3802,78;3806,05FB".
In case more than one byte appears after the comma, the byte values get written to next addresses.

	-hi, --hinc	: Set horizontal odd/even inc reg

Sets the horizontal odd and even increment numbers. Argument is a 2 hex digits byte. "-hi xy" is convenience shortcut for "3814,xy" in --regs for ov5647 sensor. Lookup the sensor mode registers for your sensor header file for default values. TODO: Needs to be extended to deal with the other sensors as well.

	-vi, --vinc	: Set vertical odd/even inc reg

Sets the vertical odd and even increment numbers. Argument is a 2 hex digits byte. "-vi xy" is convenience shortcut for "3815,xy" in --regs for ov5647 sensor. TODO: Needs to be extended to deal with the other sensors as well.

	-f, --fps	: Set frame rate regs

Sets the requested frame rate; argument is a floating point number. All sensors but adv7282m sensor are supported. "WARNING ..." gets logged if frame rate is above sensor specified maximal value.

#### Sensor mode setting options

The following options allow to overwrite some sensor mode settings for current sensor mode.

	-w, --width	: Set current mode width

Sets the width value of current mode.

	-h, --height	: Set current mode height

Sets the height value of current mode.


#### File output settings

	-hd0, --header0	: Write the BRCM header to output file

For high frame rate modes writing BRCM header to each file is a bottleneck.
So this option is a replacement for "--header"/"-hd" option.
Instead of writing header to each frame file, it is written to specified output file only.
For decoding ith frame with **dcraw** later, you need to concatenate specified output file and frame i and store somewhere, then **dcraw** that file.

	-ts, --tstamps	: Write timestamps to output file

With this option timestamps for the captured frames get written to specified output file.
This happens after video has been captured, so does not negatively affect capture frame rate.
Format: "delta,index,timestamp\n"

Timestamp distribution analysis can be easily done this way:

	$ cut -f1 -d, tstamps.csv | sort -n | uniq -c
	      1 
	     13 1499
	     17 1500
	     31 1501
	    147 1502
	    376 1503
	     22 1504
	     33 1505
	     14 1506
	      3 1507
	      1 3005
	      2 3006
	$ 


This shows that frame deltas are 1503&micro;s &plusmn; 4&micro;s, which corresponds to 1,000,000/1503=665.3fps.
Three frame skips happened during recording, and their indices can be easily determined by:

	$ grep "^3" tstamps.csv 
	3006,2,6027843627
	3005,85,6027969857
	3006,554,6028676146
	$ 

So we know that frames 0085-0553 have no frame skips.


	-emp, --empty	: Write empty output files

This option allows to determine the maximal frame rate **raspiraw** callback will be triggered. Only empty files will be written for the frames, but the filenames allow to count how many. This would be an example use:

	raspiraw -md 7 -t 3000 -emp {some options from this section} -sr 1 -o /dev/shm/out.%04d.raw 2>/dev/null

Using **/dev/shm** ramdisk for storage is essential for high frame rates. You precede this command by "rm /dev/shm/out.&ast;.raw" and do "ls -l /dev/shm/out.&ast;.raw | wc --lines" afterwards to determine the number of frames written ("-sr 1" means saverate 1 or writing all frames received from camera). "--empty" option allows to determine upper bounds for the absolute maximal frame rate achievable for a given set of high frame rate options.



#### Examples:

This is an example making use of most high frame rate command line options:

	$ rm /dev/shm/out.*.raw
	$ raspiraw -md 7 -t 1000 -ts tstamps.csv -hd0 hd0.raw -h 64 --vinc 1F --fps 660 -r "380A,0040;3802,78;3806,0603" -sr 1 -o /dev/shm/out.%04d.raw 2>/dev/null
	Using i2C device /dev/i2c-0
	$ ls -l /dev/shm/out.*.raw | wc --lines
	660
	$

This command captures video from ov5647 camera on CSI-2 interface:
* based on 640x480 mode (-md 7)
* captures for 1s (-t 1000)
* stores &micro;s timestamps in file tstamps.csv (-ts)
* stores BCRM header needed for **dcraw** only once in file hd0.raw  (-hd0)
* sets frame capture height to 64 (-h 64)
* increases line skipping to 1 and 15 instead of 3 and 5. Results in doubling vertical covered area (--vinc 1F, sum 8 vs 16). 1F shows colors (see below), 3D result is pale
* asks for 660 fps (--fps 660)
* sets some ov5647 registers (380A,0040;3802,78;3806,0603)
* sets saverate to 1 (save all frames)
* outputs in "/dev/shm" ramdisk files starting with "out.0001.raw"
* redirects standard error output (lots of mmal messages) to /dev/null (2>/dev/null)

For being able to convert frame 123 captured frame with **dcraw** these steps are necessary (because of -hd0):

	cat hd0.raw /dev/shm/out.0123.raw > out.0123.raw
	dcraw out.0123.raw

Since line scanning speed was doubled, the captured 128x64 frames need to be stretched by factor 2.
You can use this small C code and know exactly what happens, or any other stretching program (gimp, netpbm tools, ...), the result is still a .ppm format file:
[double.c](tools/double.c)

	double out.0123.ppm > out.0123.ppm.d

This frame was captured with 665fps. It is surprisingly colorful despite only 1.5ms shutter time:
![600fps sample frame just described](res/out.0123.ppm.d.png)


Now some remarks on  -r "380A,0040;3802,78;3806,0603"  register changes.

In above command we changed sensor mode height to 64 via "-h 64". This is reflected by "0040" stored in in DVP output vertical height registers 380A+380B.

Looking up register difference between mode4 (1296x960) and mode5 (1296x720) it can be seen that y_addr_end register (3806+3807) difference (0x07A3 versus 0x06B3) is 0xF0=240. This is the reduction in vertical resulution between both modes. For being based on mode7 the higher value is 0x07A3=1955. Keeping 64 lines from 480 means reduction by 480-64=0x01A0 from 0x07A3, which is 0x0603.

For this to work 78 is needed in register 3802 according diff between mode4 and mode5. 3802 top 4 bits are debug mode (7), while  lower 4 bits are bits [11:8] of y_addr_start. Register 3803 value for bits [7:0] of y_addr_start is 0x00. Not sure what 0x0800=2048 means since that is above vertical sensor row size. But that setting makes it work.


#### Creation of .ogg video from **dcraw** processed and stretched .ppm frames

First you need to convert the .ppm frames you are interested in into .png format, eg. with netpbm tools:

	pnmtopng out.0123.ppm.d > out.0123.ppm.d.png 

This gstreamer pipeline creates .ogg video. You can choose frame rate the video should play with (eg. 1fps for very slow motion), and start index of the frames to be taken. 

	gst-launch-1.0 multifilesrc location="out.%04d.ppm.d.png" index=300 caps="image/png,framerate=\(fraction\)1/1" ! pngdec ! videorate ! videoconvert ! videorate ! theoraenc ! oggmux ! filesink location="$1.ogg"

#### Creation of animated .gif from .ogg video

You can create high quality animated .gif from .ogg video with ffmpeg based [gifenc.sh](tools/gifenc.sh). You only need to adjust **fps** and **scale** in **filters** variable of that script to match what you want.

	gifenc.sh $1.ogg $1.anim.gif

Sample: 360fps 640x120 (rescaled to 640x240) video taken with v1 camera, played 25x slowed down:
![360fps sample video](res/out.360fps.25xSlower.2.anim.gif)

[tools directory](tools/)

## Where is the limit?

Above sample did capture 640x128 frames at 665fps.
It is possible to capture 640x64 stretched frames with 900fps!
[https://www.raspberrypi.org/forums/viewtopic.php?f=63&t=109523&p=1246776#p1246776](https://www.raspberrypi.org/forums/viewtopic.php?f=63&t=109523&p=1246776#p1246776)
![900fps sample frame just described](res/out.3000.ppm.d.png)

Sharp and well lighted video can be taken with NoIR camera with lense and 3W IR LED. This is 640x128 frame from video taken with 665fps:
![665fps NoIR camera with lense sample frame](resout.1000.ppm.d.png)
