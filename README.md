# OpenHMD Rift Debugger

This is a debugging visualiser for analysing recordings of VR sessions made with the Oculus Rift tracking branches of OpenHMD. 

## Building

When cloning this repository make sure you also install all the submodules.

Either clone with `git clone --recursive` or execute:
```
git submodules init
git submodules update
```
after cloning.

If you're pulling a newer version (re)execute the `git submodules update` command to make sure the modules are up to date. If you're interested in using different branches of the 3rd party modules just CD into their subfolder and you can execute git commands on those repositories.

**Compiling**

There are 2 parts to the debugger - a Godot project and a native addon. You must build  the addon before you can run the debugger.

To build it, you need `meson` (https://mesonbuild.com/), `ninja` and GStreamer 1.x development packages.

```
meson build
ninja -C build
```

**Running**

After building the addon, you can launch the project using:

```
godot Main.tscn
```

License
-------
The source code for the module is released under MIT license (see license file).
Note that the related products used, hidapi, libusb, openhmd and Godot itself all have their own licenses.
