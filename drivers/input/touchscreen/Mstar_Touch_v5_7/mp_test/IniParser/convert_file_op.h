#include "../global.h"

#ifndef __CONVERT_FILE_OP_H
#define __CONVERT_FILE_OP_H

extern int fprintf(struct file *file, const char *fmt, ...);
extern char *kgets(char *dst, int max, struct file *fp);
char *kstrtok(char *s, const char *delim, char **lasts);
extern int katoi(char *str);
/* extern double atof(const char *s); */

extern int file_sync(struct file *file);
extern int file_read(struct file *file, unsigned long long *offset, unsigned char *data, unsigned int size);
extern int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size);
extern struct file *file_open(const char *path, int flags, int rights, int *size);
extern void file_close(struct file *file);

#endif
