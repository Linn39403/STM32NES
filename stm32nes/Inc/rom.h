#define Constant_NES_Expected 0x4E45531A

#define _USE_ROM_MARIO_
#define _USE_ROM_TANKE_
#define _USE_ROM_MAPPY_

#ifdef _USE_ROM_MARIO_
static const NesRom MarioRomFile =
#include "..\..\ROMS\MarioRom.h"
#endif

#ifdef _USE_ROM_TANKE_
static const NesRom TankRomFile = 
#include "..\..\ROMS\TankRom.h"
#endif

#ifdef _USE_ROM_MAPPY_
static const NesRom MappyRomFile = 
#include "..\..\ROMS\MappyRom.h"
#endif

static const NesRom* romFiles[3] = {&MarioRomFile, &TankRomFile, &MappyRomFile};
const NesRom* rom_select(int sel) {
    return romFiles[sel];
}
