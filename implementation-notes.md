## Implementation notes

LWB-CC2538 implementation is in line with the main stream Contiki source (last checked with the commit `719f712307596c5f35c8f3f40824d9dfb110516b`). In other words, LWB-CC2538 is linked with Contiki as an external project. Therefore, some of the Contiki's CPU and platform files are overridden as needed.

### Content and directory structure
LWB-CC2538 has the following directory structure.

`apps` - Demo applications

`dev/cc2538` - Overridden SoC specific files of Contiki 

`net/glossy` - Glossy implementation

`net/lwb` - LWB implementation

`platform/zoul` - Overridden Firefly node specific files of Contiki  

`contiki` - Contiki source tree

### Used CC2538 specific hardware features
* Timestamping frame receptions:
  MAC Timer is used for timestamping frame receptions. The MAC Timer has an accuracy of 31.25 ns at 32 MHz clock speed and a combined clock tick resolution of 40 bits.

* Scheduling frame transmissions:
  Command Strobe Processor (CSP) is used to schedule the next transmission in the Glossy flood without intervening the CPU.

* AES encryption/decryption:
  AES Cryptoprocessor is used in AES-CCM mode to encrypt and decrypt Glossy frames.

### LWB-CC2538 on other CC2538 based platforms

**Though you will be able to use the same binary compiled for Zolertia Firefly with any other CC2538 based platform, you should take extra caution about the bootloader backdoor configuration of the platform. Otherwise, you could easily disable the backdoor (you might need to use JTAG to re-enable it).**
