video:
“video:
“px4 rover hardware mavros cmd 31010.mp4”

rosrun mavros mavcmd int 31010 4 5 4 5 0 0 0
the rover version runs rover_commander, which
receive both rc input and mavlink cmd. and set pwm
output at auxpins.

the mavlink cmd (MAV_CMD_USER_1) override rc input.
roll: < 1301 left turn, > 1699 right turn [1302 1698] forward or backword.
throttle must be great >1720 or < 900 to see it moving.
the speed of the turn is determined by throttle. roll only decide left
or right or stright.
                                                          thr  roll
rosrun mavros mavcmd int 31010 1800 1200 0 0 0 0 0
        left
rosrun mavros mavcmd int 31010 1800 1700 0 0 0 0 0
        right
rosrun mavros mavcmd int 31010 1900 1400 0 0 0 0 0
        straight, forward fast
rosrun mavros mavcmd int 31010 1720 1400 0 0 0 0 0
        straight, forward slow
rosrun mavros mavcmd int 31010 900 1400 0 0 0 0 0
        straight, backward fast
rosrun mavros mavcmd int 31010 1180 1400 0 0 0 0 0
        straight, backward slow



both has a time out count so if no input the motor
output will be zero.

we will test how mavlink cmd, send over via mavlink, will
be received at the px4.

the px4 is connected to the test computer (this one) by usb and
an additional usb-serial cable. the usb-serial cable connect to
px4's telem 4/5 port, which provide nsh console.

the test sequence is: (1) hookup px4. run nsh5 to open nsh console
to px4. (2) run roscore, launch mavros, (3) send a random mavlink
cmd, check the nsh console to see it pop up. (4) at px4 console
turn on "rover_commander dbgtoggle" to see current auxpin output,
(5) send mav_cmd_user_1 cmd (31010), check nsh console to see new output

See note 1/26/17 for mavros 
---------------9/10/18 rover 1.2.2 starting script-----------
Git tag rover1.2.2-frozen, new develop on rover1.2.3 branch

ROMFS/px4fmu_common/init.d$rcS is the start script, which run rover_commander
This file also specify to run ekf2 or lpe...
The cmake/configs/… file is the make configure file to build these modules.
Using “git checkout origin/master”, you will get back to original setup, and the rcS file there does not have rover_commander.
Use “git commit -a” to commit all the change you made before switch to another branch. The newly created files will still be there.

----------------9/6/18 rover 1.2.2 retest ------------------------------
follow 3/10/17 note still work, except that the usb mavlink not work.
--- cause: incompatible params in the produced .px4 file
--- solution (temporary): in qgc upload firmware 1.5.1 local, then reset params to default, then in command line use “make … upload”, that will result in working usb mavlink.
--- note nsh5 still work in either case. 
--- solution #2: reset parameter SYS_AUTOSTART to 0, close qgc, using nsh5
	param set SYS_AUTOSTART 0
	Don’t know why, SYS_AUTOSTART seems to be airframe number, which has a script assoicated with it. Set to 50001 also work.

--- qgc mavlink established but partial, upload parameters 
    binary-img-px4/1.5.1-dev-local/parameters/board_p3dx_rover.params
						board6_p3dx_rover_calib.params
Firmware to use:
    binary-img-px4/1.5.1-dev-local/rover-1.2.2.px4 ---- for rover
					Nuttx-px4fmu-v2-default.px4--- original 1.5.1 local bld

--- in normal case, mavlink should be avialable at usb and telem #2
--- the original modified p3dx shows 1.2.0, built 2/23/2017. with SYS_AYTOSTART=50001
--- if mavlink is not up, also check the sdcard which is required.

------------------------3/10/17 px4 firmware 1.5.1 rover1.2 src build -----------------
/media/student/code2/px4-1.5/Firmware   1.5.5
/home/student/Downloads/Firmware  1.5.0 rover1.2
cd Firmware, git checkout rover1.2
make px4fmu-v2_default upload
When switch between rover 1.2 and normal copter, the cmake/configs/nuttx_px4fmu-v2_default.cmake should be copied from either nuttx_px4fmu-v2_default.cmake.rover or nuttx_px4fmu-v2_default.cmake.iris . this file will include proper src/modules subdirectory for building
------------------------------- rover_commander version/change ----------------------------
2/9/2017 v1.2   control order: unexpired mavlink over rc input if both present. 
                mavlink timeout timeout 5 secs, rc timeout 3 sec, 
                for advanced mode, set mavlink timeout to -1 so rc will be
                practially disabled. to set timeout value, use mavlink cmd
                or subcommand mavtimeout and rctimeout

                to change the rc transmitter, use rcmodel command, 1 for futuba
                2 for iris. default is futuba

2/8/2017 v1.1   add pwm mode, subcommand: auxpwm, showstatic, resetstatic
                default now at pwm mode, the output is pwm from 0% to 100%
                based on rc input. rc throttle dead zone 1200-1700
                subcommand be careful to reset static variable fd, pwmfd 
                to make sure restart of thread will be fine.a
                pwm mode with mavlink command to be tested.
2/7/2017: v1.0  fixed speed, throttle 100%, -100%, or 0
---------------------------- 2/5/17 -testing aux in pwm mode ----------------------
fmu mode_pwm
pwm arm -d /dev/pwm_output1
pwm rate -d /dev/pwm_output1 -a -r 500
pwm info -d /dev/pwm_output1 
pwm test -d /dev/pwm_output1 -c 123456  -p 100
	100 us pulse width, -p 2000, 2000 us pulse, at 500 HZ, this is about 100% duty cycle

Adding pwm code to rover_commander: linked fmu.cpp for fmu class and functions.
--------------------------2/5/17 rover_commander update --------------------------------------
usage for testing aux output: rover_commander {setaux 7|resume|dbgtoggle} \n"
                "setaux 7: pause thread, set aux output to 000111\n"
                "resume: resume paused thread\n"
                "dbgtoggle: turn on/off dbg info\n"
                "notice that fd must be set to -1 in these code to reopen the"
                "io port\n");
-------------2/3/17 rc input failsafe setup --------------------------------------
Rc receiver can be setup a preset failsafe setting so if transmitter is off, it will output
A preset value to pixhawk.

-------------2/2/17 turn on/off sdlog2 and debug info channel 5/6--------------------------------------
sdlog2.c handle command topic:
                struct vehicle_command_s cmd;
                cmd.command = VEHICLE_CMD_PREFLIGHT_STORAGE;
                cmd.param1 = -1;
                cmd.param2 = -1;
                cmd.param3 = 1;
                orb_advertise(ORB_ID(vehicle_command), &cmd);
Let rover_commander check rc input channel 5, if channel 5 is between 1400- 1500 , then send the log on topic, if between 1100- 1200 send off topic, otherwise don’t send topic. which will make sdlog2 to log again. Param3 =1 log on, param3=2 log off. This no send zone is to necessary to prevend flooding the topic. The rc receiver by default latch the most recent data from transmitter, so the rc input still exist even the transmitter is turned off, until a complete power cycle.  
 
Channel 5 mapping:
	--- iris transmitter: STD position, RTL =off channel5 = 1150 (off)
				AUTO position, RTL = off, channel5=1450 (on)
				LTR, RTL = off, channel5=1320 (no send)
------------- 2/1/17 aux gpio rover control daemon rcS -------------------
Create rover_commander, run as daemon, like sdlog2, started from rcS, 
 src/modules/rover_commander.cpp
Status:  1/31/17 coding starting, 2/1/17 code from topic_listener.cpp and test_gpio imported. topic_listener.cpp backed up with modification on rc_and_vehiclecmd test case. The file will be overwrited by auto code generation, so be careful

----------------------------- 1/26/17 topic_listener pixhawk mavcmd mavlink message--------------------
The companion computer use mavcmd utility to send mavlink message to pixhawk, the mavlink daemon at px4 will publish topics in uorb space (such as vehicle_command topic). The topic_listener program in px4 subscribe to the topic and print out the topic data. 

Send a mavlink message to pixhawk:
rosrun mavros mavcmd int 300 4 5 4 5 0 0 0
		command id: 300, 4 5 4 5 param 1-4i
	rosrun mavros Iris_cmdtest.sh 
		Call mavcmd 25 times.
rosrun mavros mavcmd long 203 0 0 0 0 1 0 0
		Trigger camera one time, must be long, not int
Add topic_listener at the px4 configure file, rebuild, 
nutsh>listener vehicle_command 10
	This will wait for upto 10 uorb topic vehicle_command, 2 second timeout
The  topic send from  Iris_cmdtest.sh will go through mavros and pop up in nsh by listener.


------------------------------ 1/25/17 px4 test gpio functions flow chart ------------------
In nutsh> fmu mode_gpio,
Then nutsh>tests gpio, this will set aux to gpio mode, and toggle all aux pins in one sweep. This is enough to driver an external led, or an relay (powered from battery).
nsh will report that the ioctl is called. The fmu.cpp is modified to print a message when gpio_ioctl is called for debugging purpose. Test result: in test file, put aux0 pin to high, measure the actual volatage, see hardware setup image, a line driver is used btw pixhawk and IBT-2, 74ls367
        px4_ioctl(fd, GPIO_RESET, ~0);
        for (int i =0; i< 20; i++){
                PX4_INFO("test io");
                px4_ioctl(fd, GPIO_SET_OUTPUT_LOW, 0x5555);
                usleep(1000000);
                px4_ioctl(fd, GPIO_SET_OUTPUT_HIGH, 0x5555);
                usleep(1000000);
        }
       
	Pin aux 2,4,6 high, 3.3 vol

src/systemcmds/tests/test_gpio.c, here we open px4fmu device, so ioctl will be routed to px4fmu/fmu.cpp. The original PX4IO_DEVICE_PATH, the ioctl will go to px4io/px4io.cpp 
 int fd = px4_open(PX4FMU_DEVICE_PATH, 0);
src/drivers$ vi drv_gpio.h 
# define GPIO_SERVO_1           (1<<0)          /**< servo 1 output */
# define GPIO_SERVO_2           (1<<1)          /**< servo 2 output */
# define GPIO_SERVO_3           (1<<2)          /**< servo 3 output */
# define GPIO_SERVO_4           (1<<3)          /**< servo 4 output */
# define GPIO_SERVO_5           (1<<4)          /**< servo 5 output */
# define GPIO_SERVO_6           (1<<5)          /**< servo 6 output */

/**
 * Device paths for things that support the GPIO ioctl protocol.
 */
# define PX4FMU_DEVICE_PATH     "/dev/px4fmu"
# define PX4IO_DEVICE_PATH      "/dev/px4io"

src/drivers/px4fmu$ vi fmu.cpp
PX4FMU::ioctl(file *filp, int cmd, unsigned long arg)
{
        int ret;

        /* try it as a GPIO ioctl first */
        ret = gpio_ioctl(filp, cmd, arg);

PX4FMU::gpio_ioctl(struct file *filp, int cmd, unsigned long arg)
{
        switch (cmd) {

        case GPIO_RESET:
                gpio_reset();
                break;
        case GPIO_SET_OUTPUT:
        case GPIO_SET_OUTPUT_LOW:
        case GPIO_SET_OUTPUT_HIGH:
        case GPIO_SET_INPUT:
        case GPIO_SET_ALT_1:
                gpio_set_function(arg, cmd);
        case GPIO_SET:
        case GPIO_CLEAR:
                gpio_write(arg, cmd);
PX4FMU::gpio_write(uint32_t gpios, int function)
{
        int value = (function == GPIO_SET) ? 1 : 0;

        for (unsigned i = 0; i < _ngpio; i++)
                if (gpios & (1 << i)) {
                        px4_arch_gpiowrite(_gpio_tab[i].output, value);
                }
PX4FMU::gpio_set_function(uint32_t gpios, int function)
{
#if defined(BOARD_GPIO_SHARED_BUFFERED_BITS) && defined(GPIO_GPIO_DIR)

        /*
         * GPIOs 0 and 1 must have the same direction as they are buffered
         * by a shared 2-port driver.  Any attempt to set either sets both.
         */
        if (gpios & BOARD_GPIO_SHARED_BUFFERED_BITS) {
                gpios |= BOARD_GPIO_SHARED_BUFFERED_BITS;

                /* flip the buffer to output mode if required */
                if (GPIO_SET_OUTPUT == function ||
                    GPIO_SET_OUTPUT_LOW == function ||
                    GPIO_SET_OUTPUT_HIGH == function) {
                        px4_arch_gpiowrite(GPIO_GPIO_DIR, 1);
                }
        }

------------------------- 1/25/17 test rc input topic listener--------------------------
nsh> tests rc
INFO  [tests] Reading PPM values - press any key to abort
INFO  [tests] This test guarantees: 10 Hz update rates, no glitches (channel values), no channel count changes.


src/systemcmds/tests/test_rc.c
int _rc_sub = orb_subscribe(ORB_ID(input_rc))

nsh> listener input_rc 2

------------------------------ 1/25/17 px4 led functions flow chart ------------------
In nutsh> tests led, the led on left upper will flash quickly.
																																																																																																																																	
src/systemcmds/tests/test_led.c, 
	This only test the B/E led on left up side.  
fd = px4_open(LED0_DEVICE_PATH, 0);:	
px4_ioctl(fd, LED_OFF, LED_AMBER);

src/drivers$ vi drv_led.h 
#define LED_BASE_DEVICE_PATH            "/dev/led"
#define LED0_DEVICE_PATH                "/dev/led0"


src/drivers/led$ vi led.cpp
 This seems to be a general code, but it actually only work on one led for test purpose? This is evidence by the led_on function in px4fmu2_led.c below.

LED::LED() :
        CDev("led", LED0_DEVICE_PATH)
LED::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
        int result = OK;

        switch (cmd) {
        case LED_ON:
                led_on(arg);
                break;

src/drivers/boards/px4fmu-v2$ vi px4fmu2_led.c

__EXPORT void led_on(int led)
{
        if (led == 1) {
                /* Pull down to switch on */
                px4_arch_gpiowrite(GPIO_LED1, false);
        }
}


--------------------------- 1/25/17 pixhawk gpio -----------------------------------------
Board_config.h
/Downloads/Firmware/NuttX/nuttx/arch/arm/src/stm32/stm32_gpio.c
 * Name: stm32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin. Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with stm32_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returns:
 *   OK on success
 *   A negated errono valu on invalid port, or when pin is locked as ALT
 *   function.

/****************************************************************************
 * Name: stm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void stm32_gpiowrite(uint32_t pinset, bool value)


#define 
GPIO_AUX_CS0   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN10)
 
#define 
GPIO_AUX_CS1   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN11)
 
#define 
GPIO_AUX_CS2   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN12)
 
#define 
GPIO_AUX_CS3   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN13)
 
#define 
GPIO_AUX_CS4   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN14)
 
#define 
GPIO_AUX_CS5   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN15)
 
#define 
GPIO_SPI5_CS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN6)

---------------- 1/25/17 pixhawk pwm level shift to drive IBT motor driver---------------
Using sparkfun level shifter, the pwm signal is amplified to 3 vol or so. So it is good now. The logic shifter is used as line drive

------------------ 1/24/17 motor test rc receiver vs pwm from pixhawk --------------------
case#1 : good
	Futaba r6280, esc, motor. Futaba transmitter
            R6280 powered externally, the opto esc does not provide 5v.
	See video at this folder


case#2: bad, then good.
	Pixhawk, esc, motor, esc signal connected to pixhawk main output #2
	Nsh: (1) commander stop, (2) press fail safe switch, (3) pwm test -c 2 -p 1200.
	A possible fix: first send out min pwm signal, this might caliberate/initialze the esc and 
you will hear a music tone: 
pwm test -c 3 -p 900
Then you can send normal pwm signal value, such -p 1200.
Pwm signal produced by pixhawk:
	Freq:
	Duty min:
	Duty max:

Case #3: bad
	Pixhawk, IBT H bridge, IG52 gearmotor
	The pwm signal too weak?-- the pwm output is 2.5 v logic, instead of 3.3 v or 5 v? The 
	IBT expect 5 v logic? The IBT inputs IN and INH consist of TTL/CMOS compatible. 
	Directly wire IN1 or IN2 to 5 vol works fine.

	The pwm output is indeed TTL level, 3.4 v peak to peak. However the voltage drop to 2.4 
When the pwm output is connected to IBT bridge input IN1/IN2.  Meaning the pixhawk 
Output is not strong enough to drive input of IBT. a line driver might be needed. On the
Other hand, the fubata RC receiver’s pwm output have small voltage drop when
Connected to the IBT input and is still functional.

Further note, the esc input stage does not cause votage drop. So both rc receiver and
 Pixhawk pwm are good to drive it.

Also possible the input buffer on IBT, 74HC244, might been compromised or is simply a
count-fiet chip. To be tested with real good 74HC244.

A few notes: the esc+motor beep if no pwm signal is present. The esc does not respond to case #2 well at first: resulting warning fast beep when run pwm command. the esc was in need of configure by certain pwm signal, which is not provided by pixhawk pwm command. After hooked with case #1, the fubata receiver provided some configure signal. Then it work with case #2.

------------------- 1/29/2016 1/29/2016 arduino px4 pwm servo esc test----------------------------------------
pwm is produced by analogWrite. However frequency selection is limited. see AnalogWrite.ino in the servo folder

the music program use DigitalWrite() to produce pwm-alike signal. but use too much cpu time.

the pwm signal can move the cheap servo (both analogy and digital servo). these servo has the same connect. The 3-pin connector order should be modified when connecting to arduino. The pwm frequency should be high enough  (> 100 Hz). if 50 Hz, might be too slow.

The pixhawk produce 400 Hz pwm signal.  The ESC works fine. The servo not responding to the pwm . The reason is because for some reason the Vcc output on the pixhawk is not powered. The servo need power from Vcc pin, not just the signal pin. when connect the servo VCC pin to 5 V (e.g., Vcc from GPS connect, servo works fine.
------------------------------------------------------------------------------------
The IBT-2 H-bridge module from wingxin is an inexpensive, high power motor driver based on two BTS7960 chips with 74hc244  and is readily available from various ebay vendors; see e.g. here.

The link provides more details but here are a few key parameters.
Input voltage : 6V-27V
Maximum Current : 43A
Input level : 3.3V-5V
I am not sure whether the heat sink is sufficient for handling 43A but even if one does not drive the unit to its limits the specifications are still impressive given the typical price point of this module (currently between $13 and $17 including free shipping from China). There is relatively little information available about how to hook up the module with an Arduino controller. This thread on the Arduino forum was useful but the solution wastes a few pins and does not demonstrate bidirectional motor control. In this post I describe a slightly more complete solution that uses an Arduino controller with connected potentiometer to drive a motor via the IBT-2 module from full reverse speed to full forward speed.
For reference here is the description of the input ports and the two supported usage modes (image taken from one of the ebay postings). In this post I leverage usage mode one.






The following Fritzing diagram illustrates the wiring. B+ and B- at the top of the diagram represent the power supply for the motor. A 5k or 10k potentiometer is used to control the speed.

			

