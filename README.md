# LWB-CC2538

This work implements [Low-power Wireless Bus (LWB)](http://doi.acm.org/10.1145/2426656.2426658) for TI's [CC2538](http://www.ti.com/product/CC2538) Soc for [Contiki OS](http://www.contiki-os.org). LWB is a communication protocol for low-power wireless networks that abstracts the entire network into a shared bus like structure. It supports multiple communication patterns such as one-to-many, many-to-one, and many-to-many.

## Glossy
LWB is built on top of [Glossy](http://doi.acm.org/10.1145/2426656.2426658) which is a flooding protocol that uses concurrent transmissions of packets with identical contents to benefit from [constructive interference](https://en.wikipedia.org/wiki/Interference_%28wave_propagation%29) and [capture effect](https://en.wikipedia.org/wiki/Capture_effect). Glossy can reliably flood the entire network within few milliseconds while offering sub-microsecond time synchronisation without extra cost.

The Glossy implementation takes advantage of the hardware features offered by CC2538 to accurately timestamp packet receptions and schedule packet transmissions. For more details, refer the [implementation notes](implementation-notes.md).

#### Glossy encryption

LWB-CC2538 supports authenticated packet encryption with [AES](https://en.wikipedia.org/wiki/Advanced_Encryption_Standard). It is possible to enable/disable the encryption at runtime. Encryption/decryption is done with the support of hardware acceleration to minimise the processing time.

As Glossy requires the simultaneous transmissions to be identical, encrypted Glossy packets must also be identical. Therefore, all nodes of the network have to use the same key for encrypting packets.

## Implementation
Refer the [implementation notes](implementation-notes.md) for details.

## How to compile and upload LWB demo application

The instructions to compile and upload for [Zolertia Firefly](https://zolertia.io/product/firefly/) nodes on a Ubuntu (14.04<) based Linux system are shown below.

#### Setting up the tool chain and source
1. Install ARM tool chain

   ```
   sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi
   ```
2. Clone lwb-cc2538 repository

   ```
   git clone https://github.com/kasunch/lwb-cc2538.git
   ```
3. Get Contiki source tree if not available.

   ```
   git submodule update --init --recursive
   ```

#### Setting up Node IDs
Before compiling the LWB demo application, nodes should have IDs assigned. To have unique node IDs, static mappings between node IDs and the CC2538's IEEE addresses are used. The node IDs can be statically set in `apps/deployment/deployment.c` file as follows.

```C
struct id_addr {
  uint16_t id;
  uint8_t ieee_addr[IEEE_ADDR_LEN];
};
static struct id_addr id_addr_list[] = {

  {1, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0xb5, 0xf0}},
  {2, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0xb4, 0x59}},
  {3, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0xb4, 0x79}},
  {4, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0x9a, 0xd0}},
  {5, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0x9a, 0xca}},
  {5, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0xb1, 0x47}},

  {0, {0, 0, 0, 0, 0, 0 ,0, 0}}
};
```
If the IEEE addresses are not known, use the following command to see the IEEE addresses of the nodes.

`python contiki/tools/cc2538/cc2538-bsl.py -p <serial port>`

This should output something as follows:

```
Opening port /dev/ttyUSB0, baud 500000
Connecting to target...
CC2538 PG2.0: 512KB Flash, 32KB SRAM, CCFG at 0x0027FFD4
Primary IEEE Address: 00:12:4B:00:06:0D:B5:F0
```

#### Compiling and uploading to the nodes
1. Go to the directory `apps/lwb-test` and run `make`.
2. Upload to the nodes using `make lwb-test.upload`

## Current status

LWB-CC2538 is implemented as a part of a research project. Though the implementation has been extensively tested, there could be bugs in the code. Therefore, neither I guarantee a reliable operation nor encourage you to use this in production systems. Nonetheless, you are very welcome to try the implementation and modify it according to your needs.
