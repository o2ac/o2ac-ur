# Screwing Tools

Screw fastening is a challenging task in robotic assembly. 
Fastening via rotating a robot joint is generally not efficient or feasible, so we use external screwing tools. 

We used two types of screw tools, which share some characteristics:

- They contain compliance (bit cushion) to allow easy force application, without requiring impedance control.
- They are driven by very affordable [Dynamixel XL-320](https://www.robotis.us/dynamixel-xl-320/) motors, allowing stalling detection.

The most significant design difference between them is the use of suction. 

The M3 and M4 screw tools used a suction cup to pick the screw and detect success.
The parts used in these tools are sold by [Sawa](https://sawahb.com/), a manufacturer in Iwate, Japan.
One tool should cost between 1000-1500 USD, which is expensive, but they are extremely durable and reliable.

The set screw tool used no suction, is fully 3D-printed which is significantly cheaper. 
It can be used together with wrenches like [this](https://www.asahi-tool.co.jp/product/catcher-wrench.html) that lock into the screw head, so that screws can still be picked.
However, success will need to be determined manually or via vision.

This section details the latter tool's design and assembly, so others can reproduce a screwing system at low cost.

(TODO (felixvd): Add images)

# 3D printed screw tool

We propose an extremely simple design of a screwing tool for the use of robots. 
To fabricate it, order the standard parts in the purchase list, and 3D-print the customized parts. 
Feel free to try it out yourself!

## Motor
We use [DYNAMIXEL XL-320](https://www.robotis.us/dynamixel-xl-320/), a compact and precise servo motor, which costs only 24 dollars. 
It offers limitless rotation and a maximum torque of 0.39 Nm. 
Using a servo motor as a driver allows us to detect motor stalling (screw success) and . 
A [Power Hub Board](https://www.robotis.us/u2d2-power-hub-board-set/) is used for power supply, and a [U2D2](https://www.robotis.us/u2d2/) for controlling and operating the motor from the PC. 
[Cable 3P-XL](https://www.robotis.us/robot-cable-3p-xl-130mm-5pcs/) is the cable set exclusive for the XL-320 (packaged with the motor and U2D2). 

As this motor has no threaded hole for attaching it, we make a shell to fix it.
<div align="center">
    <img width="100%" src="pic/motor.jpg"></img>
</div>

## Motor Shell

We make use of the anchor holes on the motor body for fixing, as no bolt fastening holes are available. 
Four clamp plates with locating pins are designed to surround and button the motor, and they are further fastened to each other by bolts, which fixes the motor via form closure. 
These four plates form the shell of the motor. 
The shell is equipped with two slide rails and springs, which enables the tooltip to have compliance in one axis.
<div align="center">
    <img width="100%" src="pic/motor_shell.jpg"></img>
</div>

## Compliance module

Uncertainty is always a difficulty in screw fastening tasks, and one simple way to deal with it is to make the tooltip compliant. 
In this design, we introduce compliance in the motor instead of in the tooltip. 
this way, the screw tip can be thin and fit into narrow spaces.
Thus, we mount two [slider rails](https://www.monotaro.com/p/4867/2478/?t.q=%83%8A%83j%83A%83K%83C%83h) on the motor shell and the rails are fastened on the tool shell. 
This mechanism guides the motion of the motor body. 
On the backside of the motor, another clamp plate is attached, and the locating pins here are used to hook the [spings](https://www.amazon.co.jp/200%E5%80%8B%E5%85%A5%E3%82%8A%E3%80%91%E3%82%B9%E3%83%97%E3%83%AA%E3%83%B3%E3%82%B0-%E5%9C%A7%E7%B8%AE%E3%82%B9%E3%83%97%E3%83%AA%E3%83%B3%E3%82%B0-%E3%83%94%E3%83%83%E3%82%AF%E3%83%84%E3%83%BC%E3%83%AB-%E5%8F%8E%E7%B4%8D%E3%82%B1%E3%83%BC%E3%82%B9%E4%BB%98%E3%81%8D-GRANSTAGE/dp/B08QF6V8XC/). 
The holder is fixed on the back end of the tool shell. 
The holder does three things: giving pre-stress on the springs, constraining the extreme position of the motor, leaving paths for the cables.
<div align="center">
    <img width="100%" src="pic/compliance.jpg"></img>
</div>

## Tip
The tip of the screwdriver should be strong and accurate. 
3D printing or making it from scratch is impractical, so we designed a connector that allows a commercial tool to be equipped as the tooltip, for example, a hexagon wrench. 
The connector can lock the inserted hexagon wrench via a tunnel. 
It has two through-holes for bolt fastening and two locating pins for aligning. 
We make use of the original motor flange to fix the connector. 
Two screw bolts are added to provide stable anchor points. 
Note: the screws here should be [M2.6 flat-head screws](https://www.kohnan-eshop.com/shop/g/g497987446528305/) to match the hold's size on the flange.
<div align="center">
    <img width="100%" src="pic/fingertip.jpg"></img>
</div>

## Purchase list
* [DYNAMIXEL XL-320](https://www.robotis.us/dynamixel-xl-320/) $\times$ 1
* [Power Hub Board](https://www.robotis.us/u2d2-power-hub-board-set/) $\times$ 1
* [U2D2](https://www.robotis.us/u2d2/) $\times$ 1
* [Slider Rails](https://www.monotaro.com/p/4867/2478/?t.q=%83%8A%83j%83A%83K%83C%83h) $\times$ 2
* [Spings](https://www.amazon.co.jp/) $\times$ 1 set
* [M2 bolts and nuts](https://www.amazon.co.jp/waves-%E3%82%B1%E3%83%BC%E3%82%B9%E4%BB%98%E3%81%8D-%E3%82%B9%E3%83%86%E3%83%B3%E3%83%AC%E3%82%B9-%E5%90%8430%E5%80%8B-%E8%A8%88230%E5%80%8B/dp/B07J4LCST6/) $\times$ 1 set
* [M2.6 bolts and nuts](https://www.kohnan-eshop.com/shop/g/g497987446528305/) $\times$ 1 pair

## CAD models
* [Assembly (STEP)](models/STEP/tool_assembly.STEP)
* [Components for printing (STL)](models/STL/)
<div align="center">
    <img width="100%" src="pic/component.jpg"></img>
</div>

## Assembly sequence
* Note: Assemble and arrange the cable set of the motor at the beginning.
<div align="center">
    <img width="100%" src="pic/sequence.jpg"></img>
</div>





