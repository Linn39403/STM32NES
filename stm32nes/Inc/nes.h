#ifndef __NES_H
#define __NES_H

#include "stm32f4xx.h"

extern uint32_t volatile joypad_state;
extern uint32_t joypad_A;
extern uint32_t joypad_B;
extern uint32_t joypad_U;
extern uint32_t joypad_D;
extern uint32_t joypad_L;
extern uint32_t joypad_R;
extern uint32_t joypad_SEL;
extern uint32_t joypad_START;

#pragma anon_unions
typedef struct {
	/* always "NES."*/
	char Constant_NES[4];

	/* Size of PRG ROM in 16KB units */ 
	uint8_t prg_rom_chunks;		

	/* Size of CHR ROM in 8KB units  */	  
	uint8_t chr_rom_chunks;          

	/*
	* 76543210
	* ||||||||
	* |||||||+- Mirroring: 0: horizontal (vertical arrangement) (CIRAM A10 = PPU A11)
	* |||||||              1: vertical (horizontal arrangement) (CIRAM A10 = PPU A10)
	* ||||||+-- 1: Cartridge contains battery-backed PRG RAM ($6000-7FFF) or other persistent memory
	* |||||+--- 1: 512-byte trainer at $7000-$71FF (stored before PRG data)
	* ||||+---- 1: Ignore mirroring control or above mirroring bit; instead provide four-screen VRAM
	* ++++----- Lower nybble of mapper number
	*/
	uint8_t mapper1; 

	/*
	* 76543210
	* ||||||||
	* |||||||+- VS Unisystem
	* ||||||+-- PlayChoice-10 (8 KB of Hint Screen data stored after CHR data)
	* ||||++--- If equal to 2, flags 8-15 are in NES 2.0 format
	* ++++----- Upper nybble of mapper number
	*/
	uint8_t mapper2;
	uint8_t prg_ram_size;
	uint8_t tv_system1;
	uint8_t tv_system2;
	char unused[5];
} NesRomHeader;

typedef struct {
	NesRomHeader;
	uint8_t data[40960];
} NesRom;

extern uint32_t frame_count;
const NesRom* rom_select(int sel);
void nes_init(const NesRom* game);
void nes_frame(uint8_t render);

#endif
