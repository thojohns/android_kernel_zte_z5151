#include "convert_file_op.h"

#define EOF -1

int katoi(char *str)
{
	int res = 0, i;		/* Initialize result */

	/* Iterate through all characters of input string and */
	/* update result */
	for (i = 0; str[i] != '\0'; ++i) {
		/* adding a condition for space */
		if (str[i] >= 0x30 && str[i] <= 0x39)
			res = res * 10 + str[i] - '0';
	}

	/* return result. */
	return res;
}

char *kstrtok(char *s, const char *delim, char **lasts)
{
	const char *spanp;
	int c, sc;
	char *tok = NULL;

	/* s may be NULL */
	if (!(delim && lasts))
		return NULL;
	s = *lasts;
	if (s == NULL)
		return NULL;

	/*
	 * Skip (span) leading delimiters (s += strspn(s, delim), sort of).
	 */
cont:
	c = (int)(*s++);
	for (spanp = delim; (sc = *spanp++) != 0;) {
		if (c == sc)
			goto cont;
	}

	if (c == 0) {		/* no non-delimiter characters */
		*lasts = NULL;
		return NULL;
	}
	tok = s - 1;

	/*
	 * Scan token (scan for delimiters: s += strcspn(s, delim), sort of).
	 * Note that delim must have one NUL; we stop if we see that, too.
	 */
	for (;;) {
		c = (int)(*s++);
		spanp = delim;
		do {
			sc = *spanp++;
			if (sc  == c) {
				if (c == 0)
					s = NULL;
				else
					s[-1] = 0;
				*lasts = s;
				return tok;
			}
		} while (sc != 0);
	}
	/* NOTREACHED */
}

int kfgetc(struct file *f)
{
	char c = 0;
/* ssize_t t; */
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(get_ds());

	if (f->f_op && f->f_op->read)
		f->f_op->read(f, &c, 1, &f->f_pos);
	else
		return -EPERM;

	set_fs(oldfs);
	return c;
}

char *kgets(char *dst, int max, struct file *fp)
{
	int c = 0;
	char *p = NULL;

	/* get max bytes or upto a newline */
	for (p = dst, max--; max > 0; max--) {
		c = kfgetc(fp);
		if (c == EOF)
			break;
		*p++ = c;
		if (c == '\n')
			break;
	}
	*p = 0;
	if (p == dst || c == EOF)
		return NULL;

	return p;
}

struct file *file_open(const char *path, int flags, int rights, int *size)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	struct inode *inode;
	int err = 0;

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open(path, flags, rights);
	set_fs(oldfs);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}
#if KERNEL_VERSION(3, 18, 0) >= LINUX_VERSION_CODE
	inode = filp->f_dentry->d_inode;
#else
	inode = filp->f_path.dentry->d_inode;
#endif
	if (size != NULL)
		*size = inode->i_size;

	/* TEST_DBG(0, "%s: fsize = %d\n", __func__, size); */

	return filp;
}

void file_close(struct file *file)
{
	filp_close(file, NULL);
}

int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret;

	TEST_DBG(0, "%s: size = %d\n", __func__, size);

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_write(file, data, size, &offset);

	set_fs(oldfs);
	return ret;
}

int file_read(struct file *file, unsigned long long *offset, unsigned char *data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret;
	loff_t pos = 0;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_read(file, data, size, &pos);

	set_fs(oldfs);
	return ret;
}

int file_sync(struct file *file)
{
	vfs_fsync(file, 0);
	return 0;
}

int fprintf(struct file *file, const char *fmt, ...)
{
	va_list args;
	int length;
	char *buff = NULL;
	/* formatting strings to buffer . */
	buff = kzalloc(1024, GFP_KERNEL);
	if (buff == NULL) {
		pr_err(" malloc buff failed ");
		goto out;
	}
	va_start(args, fmt);
	length = vsnprintf(buff, INT_MAX, fmt, args);
	va_end(args);

	pr_err("size of buff = %d", strlen(buff));

	/* writing the formatted buffer to the file. */
	file_write(file, 0, buff, strlen(buff));
	kfree(buff);
out:
	return length;

}
