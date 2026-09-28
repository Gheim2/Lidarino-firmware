#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#define DEBUG_MODE 0 // 1 = Abilita messaggi di debug e disabilita la trasmissione di pacchetti del serial protocol, 0 = Disabilita messaggi di debug

#if DEBUG_MODE
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define COMMAND_TIMEOUT_MS 60000 // 1 minuto di respiro per i test manuali
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define COMMAND_TIMEOUT_MS 500   // Mezzo secondo in modalità reale
#endif

#endif // USER_CONFIG_H