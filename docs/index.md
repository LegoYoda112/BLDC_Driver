# 50x50 Drive

![50x50 Drive](hardware/media/drive-front-back.jpg "50x50 Drive")

:::{admonition} Disclaimer
:class: warning
Read/Use at your own risk! Don't assume I know what I'm doing... because I don't. Also this is V0!
:::

This is the documentation for the hardware, firmware and software for the 50x50 drive, a low-cost brushless motor controller.

The two main objectives for this project are
1. Come out of it with a good-enough-to-be-useful low-cost (sub $30) brushless motor controller 
1. Learn things (and have fun while doing that!)

Objective 2 means that a lot of things in this project are going to be homebrew, part because it's a good learning experience and part because it's sometimes fun to go through the process of reinventing the wheel.

This, coupled with objective 1's "good enough" requirement mean that plenty of the implementation is minimal and quick, targeted at being easy to understand, debug and fix.

## Documentation Contents
```{toctree}
:maxdepth: 2
hardware/index
firmware/index
```

## Other
### References
- [Simple-FOC](https://simplefoc.com/) has a fantastic, well-documented FOC implementation
- [Ben Katz's thesis](https://dspace.mit.edu/handle/1721.1/118671)
- [MJBOTS](https://mjbots.com/products/moteus-n1) which has a very similar form factor board that you should probably get if you're interested in this project but are willing to pay some more.