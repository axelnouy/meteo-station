/*
 * error.h
 * Defininitions for error code and error handling.
 * This file is part of the Lora meteo project.
 */

 typedef enum {
    ERROR_NONE = 0,          // No error
    ERROR_INIT_LORA = -1,    // LoRa initialization error
    ERROR_SEND_PACKET = -2,  // Error sending packet
    ERROR_RECEIVE_PACKET = -3 // Error receiving packet
    } ErrorCode;