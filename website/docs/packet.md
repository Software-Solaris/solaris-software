The Solaris packet is composed of a primary header, secondary header, payload and crc16.
![Solaris Packet](assets/packet.svg){ .center }

The primary header contains:

- Version: version of the SPP currently in use with the software. Major, minor and patch as defined in version.h.
- APID: identifies which service has used the packet to put data inside it. This APID is generated "randomly" by the PUBSUB service when registering a producer.
- Sequence number: each time a service sends a packet, it will send the APID and the packet number, this should be in ascendant order. It allows ground control to detect if we have missed packet in the transport layer (the air).
- Payload length.

The secondary header contains:

- Timestamp: this could be the time since power up of the unit or the time recieved via the GNSS service.
- Drop counter: if because of some error, the packet had to be dropped.

Lastly we have a payload of 50 bytes and a crc16 of 2 bytes.

This stucture is inherited from the Space Packet Protocol (SPP too) and we have adapted it to our necessities.
At first glance, the user can see the full length of the packet is 64 bytes. This is perfect, because it allws us to alineate memory and buffer, since most the of the board use, for example, 64 bytes buffers, etc.

We also have an encryption block, which we will explain later on, that we use to encrypt the data. The full packet is encrypted, so there is no need to explain it here how it works, since it does not affect the packet.h file or derivatives.

A packet like this is never allocated on the fly — there's no dynamic memory in this codebase. You borrow one from a fixed-size pool instead, which is exactly what the Databank chapter, next, is about.
