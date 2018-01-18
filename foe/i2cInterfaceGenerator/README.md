# i2cInterfaceGenerator
## Overall description
This software allows you to build a simple register-like I2C interface with
ChibiOS I2C slave driver. From a single xml file describing the desired interface,
it can generate a spec document, C symbols for the various addresses and the code
to include in the ChibiOS driver.

An example of usage can be found in [MotorController](https://github.com/TelecomParistoc/MotorController),
a firmware developped by Telecom Robotics for its robots.

## Usage
To compile the generator, simply run:

  ```$ make```

Then, the syntax is the following:

  ```$ ./xml <config_file>```

The *template_config.txt* file provides you a template to show how to write this
config file. The syntax is simple and follows this pattern: ```name = value```.

**Note:** Don't forget the spaces around the '='.

## Interface description
The xml file describing the interface is composed of a main **interface** node
containing several **register** nodes, each **register** node describing one
"register" to add to the interface.
The following elements can be specified for each register, as sub-nodes::
  - *name* : a short description of the register.
  - *c_name* : the name to use in C code generated (if selected).
  - *type* : the type of the register. Supported values for now are:
    - *uint8*
    - *uint16*
    - *uint32*
  - *address* : the address of the register, in hexedecimal notation (0x..).
    Warning: For 32 bits values, both the given address and address+2 will be used.
  - *read* : boolean indicating whether the register can be read.
  - *write*: boolean indicating whether the register can be written.
  - *category* : the name of the C struct variable containing the C variable, if any.
  - *gen_code* : boolean indicating whether to generate code for this entry. Set it
    to false if you want to write it yourself.

**Note:** Never leave an empty node. If you don't want to specify this element,
simply don't put the sub-node.

## Output
The generator can build up to three different files:
 - a **spec** file, in markdwon, describing the interface. Useful to keep your
    documentation up to date when modifying the interface.
 - a **header** file defining C symbols for the registers address. Move it with
    your other header files.
 - a **c code** file implementing two functions to handle I2C exchanges:
     - ```void i2c_vt_cb(void* param)```
     - ```void i2c_address_match(I2CDriver* i2cp)```
