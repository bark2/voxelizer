/*****************************************************************************
* Skeleton for an interface to a parser to read IRIT data files.	     *
******************************************************************************
* Written by Amit Mano						November 2008*
******************************************************************************/

#ifndef	IRIT_SKEL_H
#define	IRIT_SKEL_H

#include <stdlib.h>
#include <string>
#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/attribut.h"
#include "inc_irit/allocate.h"
#include "inc_irit/ip_cnvrt.h"
#include "inc_irit/symb_lib.h"

bool CGSkelProcessIritDataFiles(const char* filename);
void CGSkelDumpOneTraversedObject(IPObjectStruct *PObj, IrtHmgnMatType Mat, void *Data);
int CGSkelGetObjectColor(IPObjectStruct *PObj, double RGB[3]);
const char *CGSkelGetObjectTexture(IPObjectStruct *PObj);
const char *CGSkelGetObjectPTexture(IPObjectStruct *PObj);
int CGSkelGetObjectTransp(IPObjectStruct *PObj, double *Transp);

bool CGSkelStoreData(IPObjectStruct *PObj);

#endif // IRIT_SKEL_H
