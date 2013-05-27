
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include <iostream>

#include "UUID.h"
#include "DebugLog.h"

/******************************************************************************
 * UUID singleton construction and destuction
 */

UUID * UUID::p_Instance = NULL;

UUID * UUID::Instance()
{
    if (p_Instance == NULL)
    {
        p_Instance = new UUID();
    }
    return p_Instance;
}

UUID::~UUID()
{
    DEBUG_INFO("UUID object destruction\n");
}

/******************************************************************************
 * UUID management API
 */

unsigned int UUID::getUUID(void)
{
    UniqueID += 1;
    return UniqueID;
}
