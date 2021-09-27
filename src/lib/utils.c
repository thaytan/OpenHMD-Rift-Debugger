#include "utils.h"
#include <glib.h>

struct xml_unmarkup {
	GRegex *reg;
	GHashTable *codes;
};

xml_unmarkup *xml_unmarkup_new()
{
	xml_unmarkup *xu = malloc(sizeof(xml_unmarkup));

	xu->codes = g_hash_table_new (g_str_hash, g_str_equal);
	g_hash_table_insert (xu->codes, "&quot;", "\"");
	g_hash_table_insert (xu->codes, "&apos;", "\'");
	g_hash_table_insert (xu->codes, "&lt;", "<");
	g_hash_table_insert (xu->codes, "&gt;", ">");
	g_hash_table_insert (xu->codes, "&amp;", "&");

  /* Extra hack to fix bad JSON in early recordings */
	g_hash_table_insert (xu->codes, ", }", " }");

	xu->reg = g_regex_new ("(&(quot|apos|lt|gt|amp);|, })", 0, 0, NULL);

	return xu;
}

void xml_unmarkup_free (xml_unmarkup *xu)
{
	g_hash_table_destroy (xu->codes);
	g_regex_unref (xu->reg);
	free (xu);
}

static gboolean
eval_cb (const GMatchInfo *info,
				 GString					*res,
				 gpointer					data)
{
	xml_unmarkup *xu = data;

	gchar *match;
	gchar *r;

	match = g_match_info_fetch (info, 0);

	r = g_hash_table_lookup (xu->codes, match);

	g_string_append (res, r);
	g_free (match);

	return FALSE;
}

char *xml_unmarkup_string(xml_unmarkup *xu, char *text, size_t len)
{
	return g_regex_replace_eval (xu->reg, text, len, 0, 0, eval_cb, xu, NULL);
}
