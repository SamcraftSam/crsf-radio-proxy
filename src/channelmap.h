#pragma once

/* ------- HERE YOU CAN SET UP CHANNEL MAPPING -------- */
/* it does not affect behavior of the drone and handset */
/* however, this setup defines few **important** things */
/* that you should definitely consider before using     */
/* this firmware:                                       */
/* - arm/disarm channels for the failsafe mode          */
/* - pid/control stuff, that might be added here later  */
/*   will rely on yaw/pitch/roll                        */
/* - aux functionalities that **might** be added later  */
/* ---------------------------------------------------- */

#define RC_CHANNELS_NUM 16 // total number of channels in CRSF RC packet

// I define only those channels that I directly use in the code.
// Channels that I get from UART/USB and replay to the radio don't count
#define RC_CHAN_THROTTLE      2
#define RC_CHAN_DISARM_AUX1   4
#define RC_CHAN_DISARM_AUX10 13
