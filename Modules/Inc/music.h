#ifndef _MUSIC_H_
#define _MUSIC_H_

#include "main.h"

//
#define PLAYING_STOP 0
#define PLAYING_INIT_MUSIC 1

#define PLAYING_WARNING_SOUND 11
#define PLAYING_ERROR_SOUND 12

#define PLAYING_GYROCALIB_SOUND 21
#define PLAYING_AUTOAIMING_SOUND 22

//
extern uint8_t buzzer_state;

//
extern uint16_t song_littlestar[];
extern uint16_t song_happybirthday[];
extern uint16_t song_eastred[];

extern uint16_t song_robomasteryou[];
extern uint16_t song_robomasteryou2[];
extern uint16_t song_robomasterlickdog[];

//
extern uint8_t sound_warning[];
extern uint8_t sound_error[];
extern uint8_t sound_gyrocalibrating[];
extern uint8_t sound_autoaiming[];

//
extern void SetBuzzerOff(void);
extern void SetBuzzerFrequence(uint16_t freq);

//
extern void SetBuzzerState(uint8_t state);
extern void PlayingSong(uint16_t *song, uint16_t len);
extern void PlayingSound(uint8_t *sound, uint16_t len);


#endif
