#include <stddef.h>

typedef struct xml_unmarkup xml_unmarkup;

xml_unmarkup *xml_unmarkup_new();
void xml_unmarkup_free (xml_unmarkup *xu);

char *xml_unmarkup_string(xml_unmarkup *xu, char *text, size_t len);
