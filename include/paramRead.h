/*
 * @Description:
 * @Version: 2.0
 * @Author: Feng Chao
 * @Date: 2021-10-15 13:44:42
 * @LastEditors: Feng Chao
 * @LastEditTime: 2021-11-06 14:00:53
 */
#ifndef _PARAMREAD_H
#define _PARAMREAD_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"

int getfreqinfo(char *fileName, struct Freq_info *freqinfos);
int GetIniKeyString(char *, const char *, const char *, const char *);
int GetIniKeyFloatArray(const char *, const char *, float *, int, const char *);
long long GetIniKeyInt(const char *, const char *, const char *);
float GetIniKeyFloat(const char *, const char *, const char *);
int update_param_key(const char *fileName, const char *key, const char *data);
#endif
