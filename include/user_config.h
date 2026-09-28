#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#define DEBUG_MODE 1 // 1 = Abilita messaggi di debug e disabilita la trasmissione di pacchetti del serial protocol, 0 = Disabilita messaggi di debug

#if DEBUG_MODE
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif

#endif // USER_CONFIG_H