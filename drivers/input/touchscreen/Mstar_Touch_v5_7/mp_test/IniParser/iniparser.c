
/*-------------------------------------------------------------------------*/
/**
   @file    iniparser.c
   @author  N. Devillard
   @brief   Parser for ini files.
*/
/*--------------------------------------------------------------------------*/
/*---------------------------- Includes ------------------------------------*/
#include "iniparser.h"

/*---------------------------- Defines -------------------------------------*/
/* #define ASCIILINESZ         (1024 * 16) */
#define ASCIILINESZ         (1024 * 8)
/* #define ASCIILINESZ         (15000) */
#define INI_INVALID_KEY     ((char *) -1)

/*---------------------------------------------------------------------------
						Private to this module
 ---------------------------------------------------------------------------*/
/**
 * This enum stores the status for each parsed line (internal use only).
 */
typedef enum _line_status_ {
	LINE_UNPROCESSED,
	LINE_ERROR,
	LINE_EMPTY,
	LINE_COMMENT,
	LINE_SECTION,
	LINE_VALUE
} line_status;

/*-------------------------------------------------------------------------*/
/**
  @brief    Convert a string to lowercase.
  @param    in   String to convert.
  @param    out Output buffer.
  @param    len Size of the out buffer.
  @return   ptr to the out buffer or NULL if an error occurred.

  This function convert a string into lowercase.
  At most len - 1 elements of the input string will be converted.
 */
/*--------------------------------------------------------------------------*/
static char *strlwc_t(char *in, char *out, unsigned len)
{
	unsigned i;

	pr_err("%s: len = %d\n", __func__, len);

	if (in == NULL || out == NULL || len == 0)
		return NULL;
	i = 0;
	while (in[i] != '\0' && i < len - 1) {
		out[i] = (char)tolower((int)in[i]);
		pr_err("(%s, %d): in[%d] = %c\n", __func__, __LINE__, i, in[i]);
		pr_err("(%s, %d): out[%d] = %c\n", __func__, __LINE__, i, out[i]);
		i++;
	}
	out[i] = '\0';
	pr_err("%s: end ---- out = %s\n", __func__, out);
	return out;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Duplicate a string
  @param    s String to duplicate
  @return   Pointer to a newly allocated string, to be freed with free()

  This is a replacement for strdup(). This implementation is provided
  for systems that do not have it.
 */
/*--------------------------------------------------------------------------*/
static char *xstrdup(char *s)
{
	char *t;
	size_t len;

	if (!s)
		return NULL;

	len = strlen(s) + 1;
	t = kzalloc(len, GFP_KERNEL);
	/* memset(t, 0, sizeof(*t)); */
	if (t) {
		memcpy(t, s, len);
	}
	return t;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Remove blanks at the beginning and the end of a string.
  @param    str  String to parse and alter.
  @return   unsigned New size of the string.
 */
/*--------------------------------------------------------------------------*/

unsigned strstrip_t(char *s)
{
	char *last = NULL;
	char *dest = s;

	if (s == NULL)
		return 0;

	last = s + strlen(s);
	while (isspace((int)*s) && *s)
		s++;
	while (last > s) {
		if (!isspace((int)*(last - 1)))
			break;
		last--;
	}
	*last = (char)0;

	memmove(dest, s, last - s + 1);
	return last - s;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get number of sections in a dictionary
  @param    d   Dictionary to examine
  @return   int Number of sections found in dictionary

  This function returns the number of sections found in a dictionary.
  The test to recognize sections is done on the string stored in the
  dictionary: a section name is given as "section" whereas a key is
  stored as "section:key", thus the test looks for entries that do not
  contain a colon.

  This clearly fails in the case a section name contains a colon, but
  this should simply be avoided.

  This function returns -1 in case of error.
 */
/*--------------------------------------------------------------------------*/
int iniparser_getnsec(const dictionary *d)
{
	int i;
	int nsec;

	if (d == NULL)
		return -EPERM;
	nsec = 0;
	for (i = 0; i < d->size; i++) {
		if (d->key[i] == NULL)
			continue;
		if (strnchr(d->key[i], PAGE_SIZE, ':') == NULL) {
			nsec++;
		}
	}
	return nsec;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get name for section n in a dictionary.
  @param    d   Dictionary to examine
  @param    n   Section number (from 0 to nsec-1).
  @return   Pointer to char string

  This function locates the n-th section in a dictionary and returns
  its name as a pointer to a string statically allocated inside the
  dictionary. Do not free or modify the returned string!

  This function returns NULL in case of error.
 */
/*--------------------------------------------------------------------------*/
char *iniparser_getsecname(const dictionary *d, int n)
{
	int i;
	int foundsec;

	if (d == NULL || n < 0)
		return NULL;
	foundsec = 0;
	for (i = 0; i < d->size; i++) {
		if (d->key[i] == NULL)
			continue;
		if (strnchr(d->key[i], PAGE_SIZE, ':') == NULL) {
			foundsec++;
			if (foundsec > n)
				break;
		}
	}
	if (foundsec <= n) {
		return NULL;
	}
	return d->key[i];
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Dump a dictionary to an opened file pointer.
  @param    d   Dictionary to dump.
  @param    f   Opened file pointer to dump to.
  @return   void

  This function prints out the contents of a dictionary, one element by
  line, onto the provided file pointer. It is OK to specify @c stderr
  or @c stdout as output files. This function is meant for debugging
  purposes mostly.
 */
/*--------------------------------------------------------------------------*/
void iniparser_dump(const dictionary *d, struct file *f)
{

	int i;

	if (d == NULL || f == NULL)
		return;
	for (i = 0; i < d->size; i++) {
		if (d->key[i] == NULL)
			continue;
		if (d->val[i] != NULL) {
			fprintf(f, "[%s]=[%s]\n", d->key[i], d->val[i]);
		} else {
			fprintf(f, "[%s]=UNDEF\n", d->key[i]);
		}
	}
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Save a dictionary to a loadable ini file
  @param    d   Dictionary to dump
  @param    f   Opened file pointer to dump to
  @return   void

  This function dumps a given dictionary into a loadable ini file.
  It is Ok to specify @c stderr or @c stdout as output files.
 */
/*--------------------------------------------------------------------------*/
void iniparser_dump_ini(const dictionary *d, struct file *f)
{
	int i;
	int nsec;
	char *secname;

	if (d == NULL || f == NULL)
		return;

	nsec = iniparser_getnsec(d);
	if (nsec < 1) {
		/* No section in file: dump all keys as they are */
		for (i = 0; i < d->size; i++) {
			if (d->key[i] == NULL)
				continue;
			fprintf(f, "%s = %s\n", d->key[i], d->val[i]);
		}
		return;
	}
	for (i = 0; i < nsec; i++) {
		secname = iniparser_getsecname(d, i);
		iniparser_dumpsection_ini(d, secname, f);
	}
	/* fprintf(f, "\n"); */
	fprintf(f, "%c%c", 0x0D, 0x0A);
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Save a dictionary section to a loadable ini file
  @param    d   Dictionary to dump
  @param    s   Section name of dictionary to dump
  @param    f   Opened file pointer to dump to
  @return   void

  This function dumps a given section of a given dictionary into a loadable ini
  file.  It is Ok to specify @c stderr or @c stdout as output files.
 */
/*--------------------------------------------------------------------------*/
void iniparser_dumpsection_ini(const dictionary *d, char *s, struct file *f)
{
	int j;
	/* char    keym[ASCIILINESZ+1]; */
	int seclen;

	char *keym = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	/* memset(keym, 0, sizeof(*keym)); */

	if (d == NULL || f == NULL) {
		kfree(keym);
		return;
	}
	if (!iniparser_find_entry(d, s)) {
		kfree(keym);
		return;
	}
	seclen = (int)strlen(s);
	fprintf(f, "\n[%s]\n", s);
	snprintf(keym, PAGE_SIZE, "%s:", s);

	for (j = 0; j < (d->size); j++) {

		if (d->key[j] == NULL)
			continue;
		if (!strncmp(d->key[j], keym, seclen + 1)) {
			fprintf(f, "%-30s = %s\n", d->key[j] + seclen + 1, d->val[j] ? d->val[j] : "");
		}

	}
	/* fprintf(f, "\n"); */
	fprintf(f, "%c%c", 0x0d, 0x0a);
	kfree(keym);
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the number of keys in a section of a dictionary.
  @param    d   Dictionary to examine
  @param    s   Section name of dictionary to examine
  @return   Number of keys in section
 */
/*--------------------------------------------------------------------------*/
int iniparser_getsecnkeys(const dictionary *d, char *s)
{
	int seclen, nkeys;
	/* char    keym[ASCIILINESZ+1]; */
	char *keym = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	int j;

	nkeys = 0;
	/* memset(keym, 0, sizeof(*keym)); */

	if (d == NULL) {
		kfree(keym);
		return nkeys;
	}
	if (!iniparser_find_entry(d, s)) {
		kfree(keym);
		return nkeys;
	}
	seclen = (int)strlen(s);
	snprintf(keym, PAGE_SIZE, "%s:", s);

	for (j = 0; j < d->size; j++) {
		if (d->key[j] == NULL)
			continue;
		if (!strncmp(d->key[j], keym, seclen + 1))
			nkeys++;
	}

	kfree(keym);
	return nkeys;

}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the number of keys in a section of a dictionary.
  @param    d    Dictionary to examine
  @param    s    Section name of dictionary to examine
  @param    keys Already allocated array to store the keys in
  @return   The pointer passed as `keys` argument or NULL in case of error

  This function queries a dictionary and finds all keys in a given section.
  The keys argument should be an array of pointers which size has been
  determined by calling `iniparser_getsecnkeys` function prior to this one.

  Each pointer in the returned char pointer-to-pointer is pointing to
  a string allocated in the dictionary; do not free or modify them.
 */
/*--------------------------------------------------------------------------*/
char **iniparser_getseckeys(const dictionary *d, char *s, char **keys)
{
	int i, j, seclen;
	/* char keym[ASCIILINESZ+1]; */
	char *keym = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	/* memset(keym, 0, sizeof(*keym)); */

	if (d == NULL || keys == NULL) {
		kfree(keym);
		return NULL;
	}
	if (!iniparser_find_entry(d, s)) {
		kfree(keym);
		return NULL;
	}
	seclen = (int)strlen(s);
	snprintf(keym, PAGE_SIZE, "%s:", s);

	i = 0;

	for (j = 0; j < d->size; j++) {
		if (d->key[j] == NULL)
			continue;
		if (!strncmp(d->key[j], keym, seclen + 1)) {
			keys[i] = d->key[j];
			i++;
		}
	}

	kfree(keym);

	return keys;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the string associated to a key
  @param    d       Dictionary to search
  @param    key     Key string to look for
  @param    def     Default value to return if key not found.
  @return   pointer to statically allocated character string

  This function queries a dictionary for a key. A key as read from an
  ini file is given as "section:key". If the key cannot be found,
  the pointer passed as 'def' is returned.
  The returned char pointer is pointing to a string allocated in
  the dictionary, do not free or modify it.
 */
/*--------------------------------------------------------------------------*/
/* char *iniparser_getstring(const dictionary *d, char *key, char *def) */
char *iniparser_getstring(const dictionary *d, char *key, char *def)
{
	/* char * lc_key ; */
	char lc_key[100];
	char *sval;
	/* *char tmp_str[ASCIILINESZ+1]; */
	/* char *tmp_str = kzalloc(ASCIILINESZ+1, GFP_KERNEL); */
	/* memset(tmp_str, 0, sizeof(*tmp_str)); */

	if (d == NULL || key == NULL)
		return def;

	/* lc_key = strlwc_t(key, tmp_str, ASCIILINESZ+1); */
	strlcpy(lc_key, key, sizeof(lc_key));
	pr_err("%s: lc_key = %s\n", __func__, lc_key);
	sval = (char *)dictionary_get(d, lc_key, def);
	/* kfree(tmp_str); */
	pr_err("%s: sval = %s\n", __func__, sval);
	return (char *)sval;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the string associated to a key, convert to an int
  @param    d Dictionary to search
  @param    key Key string to look for
  @param    notfound Value to return in case of error
  @return   integer

  This function queries a dictionary for a key. A key as read from an
  ini file is given as "section:key". If the key cannot be found,
  the notfound value is returned.

  Supported values for integers include the usual C notation
  so decimal, octal (starting with 0) and hexadecimal (starting with 0x)
  are supported. Examples:

  "42"      ->  42
  "042"     ->  34 (octal -> decimal)
  "0x42"    ->  66 (hexa  -> decimal)

  Warning: the conversion may overflow in various ways. Conversion is
  totally outsourced to strtol(), see the associated man page for overflow
  handling.

  Credits: Thanks to A. Becker for suggesting strtol()
 */
/*--------------------------------------------------------------------------*/
int iniparser_getint(const dictionary *d, char *key, int notfound)
{
	int res, r;
	char *str;

	pr_err("%s: key = %s\n", __func__, key);

	str = iniparser_getstring(d, key, INI_INVALID_KEY);
	if (str == INI_INVALID_KEY)
		return notfound;
	/* return (int)strtol(str, NULL, 0); */
	r = kstrtol(str, 0, (long *)&res);
	if (r < 0) {
		pr_err("kstrtol error %d\n", r);
	}
	return res;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the string associated to a key, convert to a double
  @param    d Dictionary to search
  @param    key Key string to look for
  @param    notfound Value to return in case of error
  @return   double

  This function queries a dictionary for a key. A key as read from an
  ini file is given as "section:key". If the key cannot be found,
  the notfound value is returned.
 */
/*--------------------------------------------------------------------------*/
/* double iniparser_getdouble(const dictionary *d, char *key, double notfound) */
/* { */
/* char *str ; */

/* str = iniparser_getstring(d, key, INI_INVALID_KEY); */
/* if (str==INI_INVALID_KEY) return notfound ; */
/* return atof(str); */
/* } */

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the string associated to a key, convert to a boolean
  @param    d Dictionary to search
  @param    key Key string to look for
  @param    notfound Value to return in case of error
  @return   integer

  This function queries a dictionary for a key. A key as read from an
  ini file is given as "section:key". If the key cannot be found,
  the notfound value is returned.

  A true boolean is found if one of the following is matched:

  - A string starting with 'y'
  - A string starting with 'Y'
  - A string starting with 't'
  - A string starting with 'T'
  - A string starting with '1'

  A false boolean is found if one of the following is matched:

  - A string starting with 'n'
  - A string starting with 'N'
  - A string starting with 'f'
  - A string starting with 'F'
  - A string starting with '0'

  The notfound value returned if no boolean is identified, does not
  necessarily have to be 0 or 1.
 */
/*--------------------------------------------------------------------------*/
int iniparser_getboolean(const dictionary *d, char *key, int notfound)
{
	int ret;
	char *c;

	c = iniparser_getstring(d, key, INI_INVALID_KEY);
	if (c == INI_INVALID_KEY)
		return notfound;
	if (c[0] == 'y' || c[0] == 'Y' || c[0] == '1' || c[0] == 't' || c[0] == 'T') {
		ret = 1;
	} else if (c[0] == 'n' || c[0] == 'N' || c[0] == '0' || c[0] == 'f' || c[0] == 'F') {
		ret = 0;
	} else {
		ret = notfound;
	}
	return ret;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Finds out if a given entry exists in a dictionary
  @param    ini     Dictionary to search
  @param    entry   Name of the entry to look for
  @return   integer 1 if entry exists, 0 otherwise

  Finds out if a given entry exists in the dictionary. Since sections
  are stored as keys with NULL associated values, this is the only way
  of querying for the presence of sections in a dictionary.
 */
/*--------------------------------------------------------------------------*/
int iniparser_find_entry(const dictionary *ini, char *entry)
{
	int found = 0;

	if (iniparser_getstring(ini, entry, INI_INVALID_KEY) != INI_INVALID_KEY) {
		found = 1;
	}
	return found;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Set an entry in a dictionary.
  @param    ini     Dictionary to modify.
  @param    entry   Entry to modify (entry name)
  @param    val     New value to associate to the entry.
  @return   int 0 if Ok, -1 otherwise.

  If the given entry can be found in the dictionary, it is modified to
  contain the provided value. If it cannot be found, the entry is created.
  It is Ok to set val to NULL.
 */
/*--------------------------------------------------------------------------*/
int iniparser_set(dictionary *ini, char *entry, char *val)
{
	/* char tmp_str[ASCIILINESZ+1]; */
	char *tmp_str = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	/* memset(tmp_str, 0 *tmp_str); */
	return dictionary_set(ini, strlwc_t(entry, tmp_str, sizeof(tmp_str)), val);
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Delete an entry in a dictionary
  @param    ini     Dictionary to modify
  @param    entry   Entry to delete (entry name)
  @return   void

  If the given entry can be found, it is deleted from the dictionary.
 */
/*--------------------------------------------------------------------------*/
void iniparser_unset(dictionary *ini, char *entry)
{
	/* char tmp_str[ASCIILINESZ+1]; */
	char *tmp_str = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	/* memset(tmp_str, 0, *tmp_str); */
	dictionary_unset(ini, strlwc_t(entry, tmp_str, sizeof(tmp_str)));
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Load a single line from an INI file
  @param    input_line  Input line, may be concatenated multi-line input
  @param    section     Output space to store section
  @param    key         Output space to store key
  @param    value       Output space to store value
  @return   line_status value
 */
/*--------------------------------------------------------------------------*/
static line_status iniparser_line(char *input_line, char *section, char *key, char *value)
{
	line_status sta;
	char *line = NULL;
	size_t len;

	char *token;
	char *rest;
	int i, count = 0;
	char delim[1] = "=";

	line = xstrdup(input_line);
	len = strstrip_t(line);

	sta = LINE_UNPROCESSED;
	/* pr_err("================= Parsing start =============\n"); */
	/* pr_err("line = %s\n", line); */
	/* pr_err("line[0] = %c , line[len-1] =%c,\n", line[0], line[len-1]); */
	/* pr_err("length = %d\n", len); */
	/* pr_err("key = %p, section = %p , value = %p\n", &key, &section, &value); */
	/* pr_err("delim:  %c\n", delim[0]); */

	for (i = 0; i < len; i++) {
		if (line[i] == delim[0])
			goto common_case;
	}

	if (len < 1) {
		/* Empty line */
		sta = LINE_EMPTY;
		goto out;
	} else if (line[0] == '#' || line[0] == ';') {
		/* Comment line */
		sta = LINE_COMMENT;
		goto out;
	} else if (line[0] == '[' && line[len - 1] == ']') {
		/* Section name */
		/* sscanf(line, "[%[^]]", section); */
		strlcpy(section, line, PAGE_SIZE);
		strstrip_t(section);
		/* strlwc_t(section, section, len+1); */
		sta = LINE_SECTION;
		goto out;
	} else {
		/* Generate syntax error */
		sta = LINE_ERROR;
		goto out;
	}

common_case:
	rest = line;
	while ((token = kstrtok(rest, "=", &rest))) {
		/* pr_err("%s , %d\n", token, strlen(token)); */
		if (count == 0) {
			/* key = token; */
			strlcpy(key, token, PAGE_SIZE);
		} else {
			/* value = token; */
			strlcpy(value, token, PAGE_SIZE);
		}

		count++;
	}

	/* Usual key=value without quotes, with or without comments */
	/* pr_err("usual case: key = %s , value = %s\n", key, value); */
	strstrip_t(key);
	/* strlwc_t(key, key, len); */
	strstrip_t(value);
	sta = LINE_VALUE;
out:
	/* pr_err("key = %p, section = %p , value = %p\n", &key, &section, &value); */
	/* pr_err("================= Parsing end =============\n");         */
	kfree(line);
	return sta;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Parse an ini file and return an allocated dictionary object
  @param    ininame Name of the ini file to read.
  @return   Pointer to newly allocated dictionary

  This is the parser for ini files. This function is called, providing
  the name of the file to be read. It returns a dictionary object that
  should not be accessed directly, but through accessor functions
  instead.

  The returned dictionary must be freed using iniparser_freedict().
 */
/*--------------------------------------------------------------------------*/
dictionary *iniparser_load(char *ininame)
{
	struct file *in = NULL;

	/* char line    [ASCIILINESZ+1] ; */
	/* char section [ASCIILINESZ+1] ; */
	/* char key     [ASCIILINESZ+1] ; */
	/* char tmp     [(ASCIILINESZ * 2) + 1] ; */
	/* char val     [ASCIILINESZ+1] ; */
	char *line;
	char *section;
	char *key;
	char *tmp;
	char *val;

	int last = 0;
	int len;
	int lineno = 0;
	int errs = 0;

	char EOF[10];
	int fsize = 0;
/* loff_t pos = 0; */

	dictionary *dict;

	strlcpy(EOF, "&^*EOF", sizeof(EOF));

	TEST_DBG(0, "*** %s : ininame = %s ***\n", __func__, ininame);
	in = file_open(ininame, O_RDONLY, 0, &fsize);
	if (in == NULL) {
		pr_err("iniparser: cannot open %s\n", ininame);
		return NULL;
	}

	/* dict = dictionary_new(0) ; */
	TEST_DBG(0, "%s: fsize = %d\n", __func__, fsize);
	dict = dictionary_new(fsize);
	if (!dict) {
		file_close(in);
		return NULL;
	}

	line = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	section = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	key = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	tmp = kzalloc((ASCIILINESZ * 2) + 1, GFP_KERNEL);
	val = kzalloc(ASCIILINESZ + 1, GFP_KERNEL);
	/* memset(line,    0, ASCIILINESZ); */
	/* memset(section, 0, ASCIILINESZ); */
	/* memset(key,     0, ASCIILINESZ); */
	/* memset(val,     0, ASCIILINESZ); */
	/* memset(tmp,     0, ASCIILINESZ * 2); */
	last = 0;

	/* pr_err("key = %p, section = %p , value = %p\n", &key, &section, &val); */

/* while (fgets(line+last, ASCIILINESZ-last, in)!=NULL) { */
	while (kgets(line + last, ASCIILINESZ - last, in) != NULL) {
		lineno++;
		len = (int)strlen(line) - 1;
		if (len == 0)
			continue;
		/* Safety check against buffer overflows */
		/* if (line[len]!='\n' && !feof(in)) { */
		/* pr_err(" lineno = %d, %s\n", lineno, line); */

		if (strncmp(line, EOF, 5) == 0)
			break;

		if (line[len] != '\n' && line[len] != 255 && line[len] != 0 && line[len] != 30) {
			pr_err("iniparse: input line too long in %s (%d)\n", ininame, lineno);
			pr_err("lineno = %d, %d\n", lineno, line[len]);
			dictionary_del(dict);
			kfree(line);
			kfree(section);
			kfree(key);
			kfree(tmp);
			kfree(val);
			/* fclose(in); */
			file_close(in);
			return NULL;
		}
		/* let kernel know there is the end of line */
		/* if(line[len] == 255 || line[len] == 0 || line[len] == 30 || line[len] == 0x45) */
		/* break; */
		/* else if(strcmp(line, "&^*EOF") == 0) */
		/* break; */
		/* if(strncmp(line, EOF, 5) == 0)              */
		/* break; */

		/* Get rid of\n and spaces at end of line */
		while ((len >= 0) && ((line[len] == '\n') || (isspace(line[len])))) {
			line[len] = 0;
			len--;
		}
		if (len < 0) {	/* Line was entirely\n and/or spaces */
			len = 0;
		}
		/* Detect multi-line */
		if (line[len] == '\\') {
			/* Multi-line value */
			last = len;
			continue;
		} else {
			last = 0;
		}
		switch (iniparser_line(line, section, key, val)) {
		case LINE_EMPTY:
		case LINE_COMMENT:
			break;

		case LINE_SECTION:
			/* pr_err("%s: section = %s\n", __func__, section); */
			errs = dictionary_set(dict, section, NULL);
			break;

		case LINE_VALUE:
			snprintf(tmp, PAGE_SIZE, "%s:%s", section, key);
			/* pr_err("%s:  section = %s , key = %s , val = %s\n", __func__, section, key, val); */
			errs = dictionary_set(dict, tmp, val);
			break;

		case LINE_ERROR:
			pr_err("iniparser: syntax error in %s (%d):\n", ininame, lineno);
			pr_err("-> %s\n", line);
			errs++;
			break;

		default:
			break;
		}
		memset(line, 0, ASCIILINESZ);
		last = 0;
		if (errs < 0) {
			pr_err("iniparser: memory allocation failure\n");
			break;
		}
	}
	if (errs) {
		dictionary_del(dict);
		dict = NULL;
	}
	kfree(line);
	kfree(section);
	kfree(key);
	kfree(tmp);
	kfree(val);
	/* fclose(in); */
	file_close(in);
	return dict;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Free all memory associated to an ini dictionary
  @param    d Dictionary to free
  @return   void

  Free all memory associated to an ini dictionary.
  It is mandatory to call this function before the dictionary object
  gets out of the current context.
 */
/*--------------------------------------------------------------------------*/
void iniparser_freedict(dictionary *d)
{
	dictionary_del(d);
}
