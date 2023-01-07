#include "paramRead.h"

/*
   读取字符类型的值
 */
int GetIniKeyString(char *dst, const char *title, const char *key, const char *filename)
{

    FILE *      fp = NULL;
    int         flag = 0;
    char        sTitle[32], *wTmp;
    char sLine[1024];

    sprintf(sTitle, "[%s]", title);
    if (NULL == (fp = fopen(filename, "r")))
    {
        // fclose(fp);
        perror("fopen");
        return -1;
    }
    else
    {
        while (NULL != fgets(sLine, 1024, fp))
        {
            // 这是注释行
            if (0 == strncmp("//", sLine, 2))
                continue;
            if ('#' == sLine[0])
                continue;

            wTmp = strchr(sLine, '=');
            if ((NULL != wTmp) && (1 == flag))
            {
                if (0 == strncmp(key, sLine, strlen(key)))
                {  // 长度依文件读取的为准

                    if (sLine[strlen(sLine) - 2] == '\r')
                    {
                        sLine[strlen(sLine) - 2] = '\0';
                    }
                    else
                    {
                        sLine[strlen(sLine) - 1] = '\0';
                    }
                    fclose(fp);
                    strcpy(dst, wTmp + 1);
                    return 0;
                }
            }
            else
            {
                if (0 == strncmp(sTitle, sLine, strlen(sTitle)))
                {              // 长度依文件读取的为准
                    flag = 1;  // 找到标题位置
                }
            }
        }
    }
    fclose(fp);
    return -1;
}

/*
   读取数组类型的值
 */
int GetIniKeyFloatArray(const char *title, const char *key, float *array, int arraySize, const char *filename)
{
    FILE *      fp = NULL;
    int         flag = 0;
    char        sTitle[32];
    char sLine[1024];
    char *      p;
    int         i = 0;

    sprintf(sTitle, "[%s]", title);
    if (NULL == (fp = fopen(filename, "r")))
    {
        //  fclose(fp);
        perror("fopen");
        return -1;
    }
    else
    {
        while (NULL != fgets(sLine, 1024, fp))
        {
            // 这是注释行
            if (0 == strncmp("//", sLine, 2))
                continue;
            if ('#' == sLine[0])
                continue;

            if ((1 == flag) && 0 == strncmp(sLine, key, strlen(key)))  //找到键值位置
            {
                p = strtok(sLine + strlen(key) + 1, ",");
                while (i < arraySize)
                {
                    if (p != NULL && *p != '\n' && *p != '\r')
                    {
                        array[i] = atof(p);
                        p = strtok(NULL, ",");
                        i++;
                    }
                    else
                    {
                        if (NULL == fgets(sLine, 1024, fp))
                        {
                            fclose(fp);
                            return -1;
                        }
                        else
                            p = strtok(sLine, ",");
                    }
                }
                fclose(fp);
                return 0;
            }
            else
            {
                if (0 == strncmp(sTitle, sLine, strlen(sTitle)))
                {              // 长度依文件读取的为准
                    flag = 1;  // 找到标题位置
                }
            }
        }
    }
    fclose(fp);
    return -1;
}

/*
   读取整数类型的值
 */
long long GetIniKeyInt(const char *title, const char *key, const char *filename)
{
    char str[1024];
    if (!GetIniKeyString(str, title, key, filename))
        return strtoll(str, NULL, 0);
    else
        return -1;
}

/*
   读取浮点型的值
 */
float GetIniKeyFloat(const char *title, const char *key, const char *filename)
{
    char str[1024];
    if (!GetIniKeyString(str, title, key, filename))
        return atof(str);
    else
        return -1;
}
/*修改内容*/
int update_param_key(const char *fileName,const char*key,const char*data)
{
    static int fileIndex = 0;
    if(fileIndex > 60000) fileIndex = 0;

	FILE *fp = fopen(fileName,"r+");
	if(fp == NULL)
	{
		return -1;
	}
    char tempFile[64] = {0};
    sprintf(tempFile,"temp%d.ini",++fileIndex);
	FILE *fp2 = fopen(tempFile,"w+");
	if(fp2 ==NULL)
	{
		fclose(fp);
		return -2;
	}

	char check_buf[BUFFSIZE]={0};
    char delete_buf[BUFFSIZE]={0};
    char temp[BUFFSIZE] = {0};
    char *ptemp = NULL,*readKey = NULL;
	while(fgets(check_buf,BUFFSIZE,fp) != NULL)
	{
MATCH:        
        readKey = NULL;
        strcpy(temp,check_buf);
        ptemp = temp;
        if(strstr(temp,"=") != NULL)
        {
            readKey = strsep(&ptemp,"=");
        } 
        else
        {
            readKey = check_buf;
        }
		if(strcmp(readKey,key) == 0) //匹配成功
		{
			memset(check_buf,0,BUFFSIZE);
			sprintf(check_buf,"%s=%s\n",key,data);  //把关键字和值的内容填充好
            //printf("update key:%s data:%s\n",key,data);
            while(1)    //忽略该关键字下面行的值
            {
                if(fgets(delete_buf,BUFFSIZE,fp) != NULL) //有下面行
                {
                    if(strstr(delete_buf,"=") != NULL) //有=,说明是第二个关键字,不能忽略,必须要复制到文件中去
                    {
                        fprintf(fp2,"%s",check_buf);
                        strcpy(check_buf,delete_buf); //复制,跳转到下一次匹配(其实也可以不用匹配了)
                        goto MATCH;
                    }
                    else //无=,可能为关键字的值
                    {
                        if((strstr(delete_buf,"#") == NULL && strstr(delete_buf,"//") == NULL) && delete_buf[0] != '\n') //不是注释或者空行,是值,忽略,不复制
                        {
                            continue;
                        }
                        else //是注释或者空行,不忽略,直接复制并跳转
                        {
                            fprintf(fp2,"%s",check_buf);
                            strcpy(check_buf,delete_buf);
                            goto MATCH;
                        }  
                    }
                }
                else //无下面行,结束
                {
                    fprintf(fp2,"%s",check_buf);
                    break;
                }
                
            }	
		}
        else
        {
            fprintf(fp2,"%s",check_buf);
        }	
	}

	fclose(fp);
	fclose(fp2);
	
	//memset(check_buf,0,BUFFSIZE);
	//sprintf(check_buf,"rm -r %s", fileName);
	//system(check_buf);
	memset(check_buf,0,BUFFSIZE);
	sprintf(check_buf,"mv %s %s", tempFile,fileName);
	system(check_buf);
	return 0;
}

