/*
    stm32g070_flashint_regdata.h - Flash interface register data for:
	- STM32F4xx

	Copyright 2022-3 VintagePC <https://github.com/vintagepc/>

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

#ifndef HW_ARM_STM32F4xx_FINT_REGDEF_H
#define HW_ARM_STM32F4xx_FINT_REGDEF_H

REGDEF_BLOCK_BEGIN()
    REG_B32(EOP);
    REG_B32(OPER);
    REG_R(2);
    REG_B32(WRPERR);
    REG_B32(PGAERR);
    REG_B32(PGPERR);
    REG_B32(PGSERR);
    REG_R(8);
    REG_B32(BSY);
    REG_R(15);
REGDEF_BLOCK_END(flashif, sr);

REGDEF_BLOCK_BEGIN()
    REG_B32(PG);
    REG_B32(SER);
    REG_B32(MER);
    REG_K32(SNB, 5);
    REG_K32(PSIZE,2);
    REG_R(5);
    REG_B32(MER1);
    REG_B32(STRT);
    REG_R(7);
    REG_B32(EOPIE);
    REG_R(6);
    REG_B32(LOCK);
REGDEF_BLOCK_END(flashif, cr);

enum RegIndex{
    RI_ACR,
    RI_KEYR,
    RI_OPTKEYR,
    RI_SR,
    RI_CR,
    RI_OPTCR,
    RI_OPTCR1,
    RI_END
};

#endif
