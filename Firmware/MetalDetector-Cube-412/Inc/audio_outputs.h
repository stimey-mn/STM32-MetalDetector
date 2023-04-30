/*
 * audio_outputs.h
 *
 *  Created on: Jun 2, 2019
 *      Author: stimey
 */

#ifndef INC_AUDIO_OUTPUTS_H_
#define INC_AUDIO_OUTPUTS_H_

void init_audio_outputs(void);
void set_audio_volume(uint16_t new_vol); // pct * 10

void set_speaker_max_vol(uint16_t maxvol); // pct * 10
uint16_t get_speaker_max_vol(); // pct * 10

void set_headphone_max_vol(uint16_t maxvol); // pct * 10
uint16_t get_headphone_max_vol();	// pct * 10

#endif /* INC_AUDIO_OUTPUTS_H_ */
