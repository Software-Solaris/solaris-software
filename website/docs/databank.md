The databank is where the SPP packets — the 64-byte structure described in the previous chapter — are stored. You can think of it as the safe guard of all the packets. 
When you start the program execution, you have to call the function DATABANK_init() - this is called in the core.c file in the CORE_init() since it is SPP core functionality - to initialize the array of packets.

We have defined a macro called K_SPP_DATABANK_SIZE that has a number defined by us, but can be modified to suit your needs. 
The maximum allowed number of packets, are created at startup. That means, if that macro has a value of 50, then, only 50 SPP packets can be withdrawn from the databank. 
Once the databank is empty, no more packets are available to use, which can cause your system to malfunction.
This design decision is made, because we don't allow dynamic memory allocation in our code and we don't want to have it, since it poses a risk to the security and integrity of the code.

We know managing the packets is quite a hard taks, and that the developer can make mistakes and ask for a packet and never return it. That is why we have created the PUBSUB service, which takes care for this problem.
