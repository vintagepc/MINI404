/*
    stm32g070_flashint_regdata.h - Flash interface register data for:
	- STM32G07x
	- STM32C092

	Copyright 2022-6 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_ARM_STM32G070_FINT_REGDEF_H
#define HW_ARM_STM32G070_FINT_REGDEF_H

REGDEF_BLOCK_BEGIN()
    REG_B32(PG);
    REG_B32(PER);
    REG_B32(MER1);
    REG_K32(PNB, 10);
    REG_B32(BKER);
	REG_RB();
    REG_B32(MER2);
    REG_B32(STRT);
    REG_B32(OPTSTRT);
    REG_B32(FSTPG);
    REG_R(5);
    REG_B32(EOPIE);
    REG_B32(ERRIE);
	REG_RB();
    REG_B32(OBL_LAUNCH);
	REG_R(2);
    REG_B32(OPTLOCK);
    REG_B32(LOCK);
REGDEF_BLOCK_END(flashif, cr);

REGDEF_BLOCK_BEGIN()
    REG_B32(EOP);
    REG_B32(OPER);
    REG_R(1);
    REG_B32(PROGERR);
    REG_B32(WRPERR);
    REG_B32(PGAERR);
    REG_B32(SIZERR);
    REG_B32(PGSERR);
    REG_B32(MISERR);
    REG_B32(FASTERR);
    REG_R(5);
    REG_B32(OPTVERR);
    REG_B32(BSY1);
    REG_B32(BSY2);
    REG_B32(CFGBSY);
    REG_R(13);
REGDEF_BLOCK_END(flashif, sr);

enum RegIndex{
	RI_ACR                  = (0x00 /4U), /* FLASH Access Control register, */
	RI_KEYR                 = (0x08 /4U), /* FLASH Key register, */
	RI_OPTKEYR              = (0x0C /4U), /* FLASH Option Key register, */
	RI_SR                   = (0x10 /4U), /* FLASH Status register, */
	RI_CR                   = (0x14 /4U), /* FLASH Control register, */
	RI_ECCR                 = (0x18 /4U), /* FLASH ECC register, */
	RI_OPTR                 = (0x20 /4U), /* FLASH Option register, */
	RI_PCROP1ASR            = (0x24 /4U), /* FLASH Bank PCROP area A Start address register, */
	RI_PCROP1AER            = (0x28 /4U), /* FLASH Bank PCROP area A End address register, */
	RI_WRP1AR               = (0x2C /4U), /* FLASH Bank WRP area A address register, */
	RI_WRP1BR               = (0x30 /4U), /* FLASH Bank WRP area B address register, */
	RI_PCROP1BSR            = (0x34 /4U), /* FLASH Bank PCROP area B Start address register, */
	RI_PCROP1BER            = (0x38 /4U), /* FLASH Bank PCROP area B End address register, */
	RI_WRP2AR               = (0x4C /4U), /* FLASH Bank WRP area A address register, */
	RI_WRP2BR               = (0x50 /4U), /* FLASH Bank WRP area B address register, */
	RI_SECR                 = (0x80 /4U), /* FLASH security register , */
	RI_END
};

#endif
