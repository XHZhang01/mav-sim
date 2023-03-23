/****************************************************************************
 * arch/arm/src/kinetis/kinetis_mpuinit.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/userspace.h>

#include "mpu.h"
#include "kinetis_mpuinit.h"
#include "hardware/kinetis_mpu.h"

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_ARM_MPU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAX
#  define MAX(a,b) a > b ? a : b
#endif

#ifndef MIN
#  define MIN(a,b) a < b ? a : b
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_mpuinitialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only restricted SAM3U
 *   resources.
 *
 ****************************************************************************/

void kinetis_mpuinitialize(void)
{
  uintptr_t datastart = MIN(USERSPACE->us_datastart, USERSPACE->us_bssstart);
  uintptr_t dataend   = MAX(USERSPACE->us_dataend,   USERSPACE->us_bssend);

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart &&
              dataend >= datastart);

  /* Show MPU information */

  mpu_showtype();

  /* Configure user flash and SRAM space */

  mpu_user_flash(USERSPACE->us_textstart,
                 USERSPACE->us_textend - USERSPACE->us_textstart);

  mpu_user_intsram(datastart, dataend - datastart);

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

/****************************************************************************
 * Name: kinetis_mpu_uheap
 *
 * Description:
 *  Map the user-heap region.
 *
 *  This logic may need an extension to handle external SDRAM).
 *
 ****************************************************************************/

void kinetis_mpu_uheap(uintptr_t start, size_t size)
{
  mpu_user_intsram(start, size);
}

#elif defined(KINETIS_MPU)

/****************************************************************************
 * Name: kinetis_mpudisable
 *
 * Description:
 *   Configure the MPU to permit All bus masters access to all resources.
 *
 ****************************************************************************/

void kinetis_mpudisable(void)
{
  uint32_t regval;

  regval = getreg32(KINETIS_MPU_CESR);
  regval &= ~MPU_CESR_VLD;
  putreg32(regval, KINETIS_MPU_CESR);
}

#endif /* CONFIG_BUILD_PROTECTED && CONFIG_ARM_MPU */
