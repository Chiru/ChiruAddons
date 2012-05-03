/* -*- Mode: c; c-basic-offset: 2 -*-
 *
 * raptor_internal.h - Redland Parser Toolkit for RDF (Raptor) internals
 *
 * $Id: raptor_internal.h,v 1.158 2005/11/21 03:52:05 dajobe Exp $
 *
 * Copyright (C) 2002-2004, David Beckett http://purl.org/net/dajobe/
 * Institute for Learning and Research Technology http://www.ilrt.bristol.ac.uk/
 * University of Bristol, UK http://www.bristol.ac.uk/
 * 
 * This package is Free Software and part of Redland http://librdf.org/
 * 
 * It is licensed under the following three licenses as alternatives:
 *   1. GNU Lesser General Public License (LGPL) V2.1 or any newer version
 *   2. GNU General Public License (GPL) V2 or any newer version
 *   3. Apache License, V2.0 or any newer version
 * 
 * You may not use this file except in compliance with at least one of
 * the above three licenses.
 * 
 * See LICENSE.html or LICENSE.txt at the top of this package for the
 * complete terms and further detail along with the license texts for
 * the licenses in COPYING.LIB, COPYING and LICENSE-2.0.txt respectively.
 * 
 * 
 */



#ifndef RAPTOR_INTERNAL_H
#define RAPTOR_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef RAPTOR_INTERNAL

/* for the memory allocation functions */
#if defined(HAVE_DMALLOC_H) && defined(RAPTOR_MEMORY_DEBUG_DMALLOC)
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#undef HAVE_STDLIB_H
#endif
#include <dmalloc.h>
#endif


/* Can be over-ridden or undefined in a config.h file or -Ddefine */
#ifndef RAPTOR_INLINE
#define RAPTOR_INLINE inline
#endif

#ifdef LIBRDF_DEBUG
#define RAPTOR_DEBUG 1
#endif

#if defined(RAPTOR_MEMORY_SIGN)
#define RAPTOR_SIGN_KEY 0x08A61080
void* raptor_sign_malloc(size_t size);
void* raptor_sign_calloc(size_t nmemb, size_t size);
void* raptor_sign_realloc(void *ptr, size_t size);
void raptor_sign_free(void *ptr);
  
#define RAPTOR_MALLOC(type, size)   raptor_sign_malloc(size)
#define RAPTOR_CALLOC(type, nmemb, size) raptor_sign_calloc(nmemb, size)
#define RAPTOR_REALLOC(type, ptr, size) raptor_sign_realloc(ptr, size)
#define RAPTOR_FREE(type, ptr)   raptor_sign_free(ptr)

#else
#define RAPTOR_MALLOC(type, size) malloc(size)
#define RAPTOR_CALLOC(type, nmemb, size) calloc(nmemb, size)
#define RAPTOR_REALLOC(type, ptr, size) realloc(ptr, size)
#define RAPTOR_FREE(type, ptr)   free((void*)ptr)

#endif

#ifdef RAPTOR_DEBUG
/* Debugging messages */
#define RAPTOR_DEBUG1(msg) do {fprintf(stderr, "%s:%d:%s: " msg, __FILE__, __LINE__, __func__); } while(0)
#define RAPTOR_DEBUG2(msg, arg1) do {fprintf(stderr, "%s:%d:%s: " msg, __FILE__, __LINE__, __func__, arg1);} while(0)
#define RAPTOR_DEBUG3(msg, arg1, arg2) do {fprintf(stderr, "%s:%d:%s: " msg, __FILE__, __LINE__, __func__, arg1, arg2);} while(0)
#define RAPTOR_DEBUG4(msg, arg1, arg2, arg3) do {fprintf(stderr, "%s:%d:%s: " msg, __FILE__, __LINE__, __func__, arg1, arg2, arg3);} while(0)
#define RAPTOR_DEBUG5(msg, arg1, arg2, arg3, arg4) do {fprintf(stderr, "%s:%d:%s: " msg, __FILE__, __LINE__, __func__, arg1, arg2, arg3, arg4);} while(0)
#define RAPTOR_DEBUG6(msg, arg1, arg2, arg3, arg4, arg5) do {fprintf(stderr, "%s:%d:%s: " msg, __FILE__, __LINE__, __func__, arg1, arg2, arg3, arg4, arg5);} while(0)

#if defined(HAVE_DMALLOC_H) && defined(RAPTOR_MEMORY_DEBUG_DMALLOC)
void* raptor_system_malloc(size_t size);
void raptor_system_free(void *ptr);
#define SYSTEM_MALLOC(size)   raptor_system_malloc(size)
#define SYSTEM_FREE(ptr)   raptor_system_free(ptr)
#else
#define SYSTEM_MALLOC(size)   malloc(size)
#define SYSTEM_FREE(ptr)   free(ptr)
#endif

#define RAPTOR_ASSERT_DIE abort();

#else
/* DEBUGGING TURNED OFF */

/* No debugging messages */
#define RAPTOR_DEBUG1(msg)
#define RAPTOR_DEBUG2(msg, arg1)
#define RAPTOR_DEBUG3(msg, arg1, arg2)
#define RAPTOR_DEBUG4(msg, arg1, arg2, arg3)
#define RAPTOR_DEBUG5(msg, arg1, arg2, arg3, arg4)
#define RAPTOR_DEBUG6(msg, arg1, arg2, arg3, arg4, arg5)

#define SYSTEM_MALLOC(size)   malloc(size)
#define SYSTEM_FREE(ptr)   free(ptr)

#define RAPTOR_ASSERT_DIE

#endif


#ifdef RAPTOR_DISABLE_ASSERT_MESSAGES
#define RAPTOR_ASSERT_REPORT(line)
#else
#define RAPTOR_ASSERT_REPORT(msg) fprintf(stderr, "%s:%d: (%s) assertion failed: " msg "\n", __FILE__, __LINE__, __func__);
#endif


#ifdef RAPTOR_DISABLE_ASSERT

#define RAPTOR_ASSERT(condition, msg) 
#define RAPTOR_ASSERT_RETURN(condition, msg, ret) 
#define RAPTOR_ASSERT_OBJECT_POINTER_RETURN(pointer, type)
#define RAPTOR_ASSERT_OBJECT_POINTER_RETURN_VALUE(pointer, type, ret)

#else

#define RAPTOR_ASSERT(condition, msg) do { \
  if(condition) { \
    RAPTOR_ASSERT_REPORT(msg) \
    RAPTOR_ASSERT_DIE \
  } \
} while(0)

#define RAPTOR_ASSERT_RETURN(condition, msg, ret) do { \
  if(condition) { \
    RAPTOR_ASSERT_REPORT(msg) \
    RAPTOR_ASSERT_DIE \
    return ret; \
  } \
} while(0)

#define RAPTOR_ASSERT_OBJECT_POINTER_RETURN(pointer, type) do { \
  if(!pointer) { \
    RAPTOR_ASSERT_REPORT("object pointer of type " #type " is NULL.") \
    RAPTOR_ASSERT_DIE \
    return; \
  } \
} while(0)

#define RAPTOR_ASSERT_OBJECT_POINTER_RETURN_VALUE(pointer, type, ret) do { \
  if(!pointer) { \
    RAPTOR_ASSERT_REPORT("object pointer of type " #type " is NULL.") \
    RAPTOR_ASSERT_DIE \
    return ret; \
  } \
} while(0)

#endif


/* Fatal errors - always happen */
#define RAPTOR_FATAL1(msg) do {fprintf(stderr, "%s:%d:%s: fatal error: " msg, __FILE__, __LINE__ , __func__); abort();} while(0)
#define RAPTOR_FATAL2(msg,arg) do {fprintf(stderr, "%s:%d:%s: fatal error: " msg, __FILE__, __LINE__ , __func__, arg); abort();} while(0)
#define RAPTOR_FATAL3(msg,arg1,arg2) do {fprintf(stderr, "%s:%d:%s: fatal error: " msg, __FILE__, __LINE__ , __func__, arg1, arg2); abort();} while(0)



/* XML parser includes */
#ifdef RAPTOR_XML_EXPAT
#ifdef HAVE_EXPAT_H
#include <expat.h>
#endif
#ifdef HAVE_XMLPARSE_H
#include <xmlparse.h>
#endif
#endif


typedef struct raptor_sax2_s raptor_sax2;

#ifdef RAPTOR_XML_LIBXML

#include <libxml/parser.h>


/* libxml-only prototypes */


/* raptor_libxml.c exports */
extern void raptor_libxml_init(raptor_sax2* sax2, raptor_uri *base_uri);
extern void raptor_libxml_init_sax_error_handlers(xmlSAXHandler *sax);
extern void raptor_libxml_init_generic_error_handlers(raptor_parser *rdf_parser);

extern void raptor_libxml_validation_error(void *context, const char *msg, ...);
extern void raptor_libxml_validation_warning(void *context, const char *msg, ...);
void raptor_libxml_free(xmlParserCtxtPtr xc);

/* raptor_parse.c - exported to libxml part */
extern void raptor_libxml_update_document_locator(raptor_sax2* sax2, raptor_locator* locator);

/* end of libxml-only */
#endif


/* expat-only prototypes */

#ifdef RAPTOR_XML_EXPAT
/* raptor_expat.c exports */
extern void raptor_expat_init(raptor_sax2* sax2, raptor_uri *base_uri);
extern void raptor_expat_update_document_locator(raptor_sax2* sax2, raptor_locator *locator);

/* raptor_parse.c */
void raptor_xml_unparsed_entity_decl_handler(void *user_data, const unsigned char* entityName, const unsigned char* base, const unsigned char* systemId, const unsigned char* publicId, const unsigned char* notationName);
int raptor_xml_external_entity_ref_handler(void *user_data, const unsigned char* context, const unsigned char* base, const unsigned char* systemId, const unsigned char* publicId);

/* end of expat-only */
#endif


typedef struct raptor_parser_factory_s raptor_parser_factory;
typedef struct raptor_serializer_factory_s raptor_serializer_factory;
typedef struct raptor_id_set_s raptor_id_set;
typedef struct raptor_uri_detail_s raptor_uri_detail;


/* Raptor Namespace Stack node */
struct raptor_namespace_stack_s {
  raptor_namespace* top;
  raptor_uri_handler *uri_handler;
  void *uri_context;
  raptor_simple_message_handler error_handler;
  void *error_data;

  raptor_uri *rdf_ms_uri;
  raptor_uri *rdf_schema_uri;
};


/* Forms:
 * 1) prefix=NULL uri=<URI>      - default namespace defined
 * 2) prefix=NULL, uri=NULL      - no default namespace
 * 3) prefix=<prefix>, uri=<URI> - regular pair defined <prefix>:<URI>
 */
struct raptor_namespace_s {
  /* next down the stack, NULL at bottom */
  struct raptor_namespace_s* next;

  raptor_namespace_stack *nstack;

  /* NULL means is the default namespace */
  const unsigned char *prefix;
  /* needed to safely compare prefixed-names */
  int prefix_length;
  /* URI of namespace or NULL for default */
  raptor_uri *uri;
  /* parsing depth that this ns was added.  It will
   * be deleted when the parser leaves this depth 
   */
  int depth;
  /* Non 0 if is xml: prefixed name */
  int is_xml;
  /* Non 0 if is RDF M&S Namespace */
  int is_rdf_ms;
  /* Non 0 if is RDF Schema Namespace */
  int is_rdf_schema;
};

#ifdef RAPTOR_XML_LIBXML
#define RAPTOR_LIBXML_MAGIC 0x8AF108
#endif

/*
 * Raptor parser object
 */
struct raptor_parser_s {
#ifdef RAPTOR_XML_LIBXML
  int magic;
#endif

  /* can be filled with error location information */
  raptor_locator locator;

  /* non 0 if parser had fatal error and cannot continue */
  int failed;

  /* generated ID counter */
  int genid;

  /* base URI of RDF/XML */
  raptor_uri *base_uri;

  /* static statement for use in passing to user code */
  raptor_statement statement;

  /* FEATURE: 
   * non 0 if scanning for <rdf:RDF> element inside the XML,
   * else require rdf:RDF as first element. See also feature_assume_is_rdf
   */
  int feature_scanning_for_rdf_RDF;

  /* FEATURE:
   * non 0 to allow non-namespaced resource, ID etc attributes
   * on RDF namespaced-elements
   */
  int feature_allow_non_ns_attributes;


  /* FEATURE:
   * non 0 to handle other rdf:parseType values that are not
   * literal or resource
   */
  int feature_allow_other_parseTypes;


  /* FEATURE: 
   * non 0 if rdf:bagID is allowed.
   * RDF/XML Syntax (Revised) 2003 forbids this.
   */
  int feature_allow_bagID;

  /* FEATURE: 
   * non 0 if rdf:Collection generates the <idList> rdf:type rdf:List triple.
   * post-last call WD of RDF/XML Syntax (Revised) 2003 forbids amended
   * the rules to not generate this.
   */
  int feature_allow_rdf_type_rdf_List;

  /* FEATURE: 
   * non 0 if normalizing xml:lang attribute values to lowercase.
   * http://www.w3.org/TR/rdf-concepts/#dfn-language-identifier
   * says this is done when language-tagged literals are in the graph.
   */
  int feature_normalize_language;

  /* FEATURE
   * non 0 if non-NFC literals cause a fatal error rather than
   * a warning.
   */
  int feature_non_nfc_fatal;

  /* FEATURE:
   * non 0 to give a warning when other rdf:parseType values that are not
   * Literal, Resource or Collection are seen.
   */
  int feature_warn_other_parseTypes;

  /* FEATURE:
   * non 0 to check rdf:ID values for duplicates
   */
  int feature_check_rdf_id;

  /* stuff for our user */
  void *user_data;

  void *fatal_error_user_data;
  void *error_user_data;
  void *warning_user_data;

  raptor_message_handler fatal_error_handler;
  raptor_message_handler error_handler;
  raptor_message_handler warning_handler;

  void* unused1; /* UNUSED - re-use struct slot later needed */

  /* parser callbacks */
  raptor_statement_handler statement_handler;

  void *generate_id_handler_user_data;
  raptor_generate_id_handler generate_id_handler;

  int default_generate_id_handler_base;
  char *default_generate_id_handler_prefix;
  size_t default_generate_id_handler_prefix_length;

  /* parser specific stuff */
  void *context;

  struct raptor_parser_factory_s* factory;

  /* namespace callback */
  raptor_namespace_handler namespace_handler;

  void* namespace_handler_user_data;
};


/** A Parser Factory for a syntax */
struct raptor_parser_factory_s {
  struct raptor_parser_factory_s* next;

  /* syntax name */
  const char* name;
  /* alternate syntax name; not mentioned in enumerations */
  const char* alias;

  /* syntax readable label */
  const char* label;
  /* syntax MIME type (or NULL) */
  const char* mime_type;
  /* syntax URI (or NULL) */
  const unsigned char* uri_string;
  
  /* the rest of this structure is populated by the
     parser-specific register function */
  size_t context_length;
  
  /* create a new parser */
  int (*init)(raptor_parser* parser, const char *name);
  
  /* destroy a parser */
  void (*terminate)(raptor_parser* parser);
  
  /* start a parse */
  int (*start)(raptor_parser* parser);
  
  /* parse a chunk of memory */
  int (*chunk)(raptor_parser* parser, const unsigned char *buffer, size_t len, int is_end);

  /* finish the parser factory */
  void (*finish_factory)(raptor_parser_factory* factory);

  /* score recognition of the syntax by a block of characters, the
   *  content identifier or it's suffix or a mime type
   *  (different from the factory-registered one)
   */
  int (*recognise_syntax)(raptor_parser_factory* factory, const unsigned char *buffer, size_t len, const unsigned char *identifier, const unsigned char *suffix, const char *mime_type);

  /* get the Content-Type value of a URI request */
  void (*content_type_handler)(raptor_parser* rdf_parser, const char* content_type);

};


/*
 * Raptor serializer object
 */
struct raptor_serializer_s {
  /* can be filled with error location information */
  raptor_locator locator;

  /* FEATURE:
   * non 0 to write relative URIs wherever possible
   */
  int feature_relative_uris;

  /* FEATURE:
   * non NULL to start serializing from this URI
   */
  raptor_uri* feature_start_uri;

  void *error_user_data;
  void *warning_user_data;

  raptor_message_handler error_handler;
  raptor_message_handler warning_handler;

  /* non 0 if serializer had fatal error and cannot continue */
  int failed;

  /* base URI of RDF/XML */
  raptor_uri *base_uri;

  /* serializer specific stuff */
  void *context;

  /* destination stream for the serialization */
  raptor_iostream *iostream;
  
  struct raptor_serializer_factory_s* factory;

  /* XML 1.0 (10) or XML 1.1 (11) */
  int xml_version;

  /* FEATURE:
   * non 0 to write XML 1.0 or 1.1 declaration (default 1)
   */
  int feature_write_xml_declaration;
};


/** A Serializer Factory for a syntax */
struct raptor_serializer_factory_s {
  struct raptor_serializer_factory_s* next;

  /* syntax name */
  const char* name;
  /* alternate syntax name; not mentioned in enumerations */
  const char* alias;

  /* syntax readable label */
  const char* label;
  /* syntax MIME type (or NULL) */
  const char* mime_type;
  /* syntax URI (or NULL) */
  const unsigned char* uri_string;
  
  /* the rest of this structure is populated by the
     serializer-specific register function */
  size_t context_length;
  
  /* create a new serializer */
  int (*init)(raptor_serializer* serializer, const char *name);
  
  /* destroy a serializer */
  void (*terminate)(raptor_serializer* serializer);
  
  /* add a namespace */
  int (*declare_namespace)(raptor_serializer* serializer, raptor_uri *uri, const unsigned char *prefix);
  
  /* start a serialization */
  int (*serialize_start)(raptor_serializer* serializer);
  
  /* serialize a statement */
  int (*serialize_statement)(raptor_serializer* serializer, const raptor_statement *statment);

  /* end a serialization */
  int (*serialize_end)(raptor_serializer* serializer);
  
  /* finish the serializer factory */
  void (*finish_factory)(raptor_serializer_factory* factory);
};


/* raptor_serialize.c */
void raptor_serializer_register_factory(const char *name, const char *label, const char *mime_type, const char *alias, const unsigned char *uri_string, void (*factory) (raptor_serializer_factory*));


/* raptor_general.c */

raptor_parser_factory* raptor_parser_register_factory(const char *name, const char *label, const char *mime_type, const unsigned char *uri_string, void (*factory) (raptor_parser_factory*));
void raptor_parser_factory_add_alias(raptor_parser_factory* factory, const char *alias);

unsigned char* raptor_generate_id(raptor_parser *rdf_parser, const int id_for_bag, unsigned char* user_bnodeid);
const unsigned char* raptor_inscope_xml_language(raptor_parser *rdf_parser);
raptor_uri* raptor_inscope_base_uri(raptor_parser *rdf_parser);
#ifdef RAPTOR_DEBUG
void raptor_stats_print(raptor_parser *rdf_parser, FILE *stream);
#endif
const char* raptor_basename(const char *name);
raptor_statement* raptor_statement_copy(const raptor_statement *statement);
void raptor_free_statement(raptor_statement *statement);

/* raptor_parse.c */
void raptor_delete_parser_factories(void);

/* raptor_general.c */
extern void raptor_parser_fatal_error(raptor_parser* parser, const char *message, ...);
extern void raptor_parser_error(raptor_parser* parser, const char *message, ...);
extern void raptor_parser_simple_error(void* parser, const char *message, ...);
extern void raptor_parser_error_message_handler(void *user_data, raptor_locator* locator, const char *message);
extern void raptor_parser_warning(raptor_parser* parser, const char *message, ...);
extern void raptor_parser_fatal_error_varargs(raptor_parser* parser, const char *message, va_list arguments);
extern void raptor_parser_fatal_error_message_handler(void *user_data, raptor_locator* locator, const char *message);
extern void raptor_parser_error_varargs(raptor_parser* parser, const char *message, va_list arguments);
extern void raptor_parser_warning_varargs(raptor_parser* parser, const char *message, va_list arguments);
void raptor_parser_warning_message_handler(void *user_data, raptor_locator* locator, const char *message);


/* raptor_parse.c */

int raptor_parser_exec(raptor_parser* rdf_parser, const char *name);

typedef struct raptor_rdfxml_parser_s raptor_rdfxml_parser;

/* Prototypes for common expat/libxml parsing event-handling functions */
extern void raptor_xml_start_element_handler(void *user_data, const unsigned char *name, const unsigned char **atts);
extern void raptor_xml_end_element_handler(void *user_data, const unsigned char *name);
/* s is not 0 terminated. */
extern void raptor_xml_characters_handler(void *user_data, const unsigned char *s, int len);
extern void raptor_xml_cdata_handler(void *user_data, const unsigned char *s, int len);
void raptor_xml_comment_handler(void *user_data, const unsigned char *s);

#ifdef RAPTOR_DEBUG
void raptor_rdfxml_parser_stats_print(raptor_rdfxml_parser* rdf_xml_parser, FILE *stream);
#endif

void raptor_parser_copy_user_state(raptor_parser *to_parser, raptor_parser *from_parser);

/* raptor_feature.c */
int raptor_features_enumerate_common(const raptor_feature feature, const char **name, raptor_uri **uri, const char **label, int flags);

/* raptor_general.c */
extern int raptor_valid_xml_ID(raptor_parser *rdf_parser, const unsigned char *string);
int raptor_check_ordinal(const unsigned char *name);

/* raptor_identifier.c */
#ifdef RAPTOR_DEBUG
void raptor_identifier_print(FILE *stream, raptor_identifier* identifier);
#endif
  
/* raptor_locator.c */


#ifdef HAVE_STRCASECMP
#define raptor_strcasecmp strcasecmp
#define raptor_strncasecmp strncasecmp
#else
#ifdef HAVE_STRICMP
#define raptor_strcasecmp stricmp
#define raptor_strncasecmp strnicmp
#endif
#endif


/* raptor_nfc.c */
int raptor_nfc_check (const unsigned char* string, size_t len, int *error);


/* raptor_namespace.c */

#ifdef RAPTOR_DEBUG
void raptor_namespace_print(FILE *stream, raptor_namespace* ns);
#endif

void raptor_parser_start_namespace(raptor_parser* rdf_parser, raptor_namespace* nspace);


/* 
 * Raptor XML-namespace qualified name (qname), for elements or attributes 
 *
 * namespace is only defined when the XML name has a namespace and
 * only then is uri also given.
 */
struct raptor_qname_s {
  /* Name - always present */
  const unsigned char *local_name;
  int local_name_length;
  /* Namespace or NULL if not in a namespace */
  const raptor_namespace *nspace;
  /* URI of namespace+local_name or NULL if not defined */
  raptor_uri *uri;
  /* optional value - used when name is an attribute */
  const unsigned char *value;
  unsigned int value_length;
};



/* raptor_qname.c */
#ifdef RAPTOR_DEBUG
void raptor_qname_print(FILE *stream, raptor_qname* name);
#endif


/* raptor_uri.c */
void raptor_uri_init(void);

void raptor_uri_init_default_handler(raptor_uri_handler *handler);

/* raptor_parse.c */
void raptor_init_parser_rdfxml(void);
void raptor_init_parser_ntriples(void);
void raptor_init_parser_turtle(void);
void raptor_init_parser_n3(void);
void raptor_init_parser_grddl(void);
void raptor_init_parser_guess(void);

/* raptor_rss.c */
void raptor_init_parser_rss(void);
void raptor_init_serializer_rss10(void);
void raptor_init_serializer_atom(void);

extern const unsigned char * const raptor_atom_namespace_uri;

/* raptor_rfc2396.c */
raptor_uri_detail* raptor_new_uri_detail(const unsigned char *uri_string);
void raptor_free_uri_detail(raptor_uri_detail* uri_detail);
unsigned char* raptor_uri_detail_to_string(raptor_uri_detail *ud, size_t* len_p);

/* raptor_serializer.c */
void raptor_init_serializer_rdfxml(void);
void raptor_init_serializer_ntriples(void);
void raptor_init_serializer_simple(void);
void raptor_delete_serializer_factories(void);
void raptor_serializer_error(raptor_serializer* serializer, const char *message, ...);
void raptor_serializer_simple_error(void* serializer, const char *message, ...);
void raptor_serializer_error_varargs(raptor_serializer* serializer, const char *message,  va_list arguments);
void raptor_serializer_warning(raptor_serializer* serializer, const char *message, ...);
void raptor_serializer_warning_varargs(raptor_serializer* serializer, const char *message, va_list arguments);

/* raptor_serialize_rdfxml.c */  
void raptor_init_serializer_rdfxmla(void);
  
/* raptor_utf8.c */
int raptor_unicode_is_namestartchar(long c);
int raptor_unicode_is_namechar(long c);
int raptor_utf8_is_nfc(const unsigned char *input, size_t length);

/* raptor_www*.c */
#ifdef RAPTOR_WWW_LIBXML
#include <libxml/parser.h>
#include <libxml/xmlerror.h>
#include <libxml/nanohttp.h>
#endif

#ifdef RAPTOR_WWW_LIBCURL
#include <curl/curl.h>
#include <curl/types.h>
#include <curl/easy.h>
#endif

/* Size of buffer used in various raptor_www places for I/O  */
#define RAPTOR_WWW_BUFFER_SIZE 4096

/* WWW library state */
struct  raptor_www_s {
  char *type;
  int free_type;
  int total_bytes;
  int failed;
  int status_code;

  raptor_uri *uri;

#ifdef RAPTOR_WWW_LIBCURL
  CURL* curl_handle;
  CURLcode status;
  char error_buffer[CURL_ERROR_SIZE];
  int curl_init_here;
#endif

#ifdef RAPTOR_WWW_LIBXML
  void *ctxt;
  xmlGenericErrorFunc old_handler;
  char buffer[RAPTOR_WWW_BUFFER_SIZE];
  int is_end;
#endif

#ifdef RAPTOR_WWW_LIBFETCH
  char buffer[RAPTOR_WWW_BUFFER_SIZE];
#endif

  char *user_agent;

  /* proxy URL string or NULL for none */
  char *proxy;
  
  void *write_bytes_userdata;
  raptor_www_write_bytes_handler write_bytes;
  void *content_type_userdata;
  raptor_www_content_type_handler content_type;

  void *error_data;
  raptor_message_handler error_handler;

  /* can be filled with error location information */
  raptor_locator locator;

  char *http_accept;

  FILE* handle;
};



/* internal */
void raptor_www_libxml_init(raptor_www *www);
void raptor_www_libxml_free(raptor_www *www);
int raptor_www_libxml_fetch(raptor_www *www);

void raptor_www_error_varargs(raptor_www *www, const char *message, va_list arguments);
void raptor_www_error(raptor_www *www, const char *message, ...);

void raptor_www_curl_init(raptor_www *www);
void raptor_www_curl_free(raptor_www *www);
int raptor_www_curl_fetch(raptor_www *www);

void raptor_www_libwww_init(raptor_www *www);
void raptor_www_libwww_free(raptor_www *www);
int raptor_www_libwww_fetch(raptor_www *www);

void raptor_www_libfetch_init(raptor_www *www);
void raptor_www_libfetch_free(raptor_www *www);
int raptor_www_libfetch_fetch(raptor_www *www);

  /* raptor_set.c */
raptor_id_set* raptor_new_id_set(void);
void raptor_free_id_set(raptor_id_set* set);
int raptor_id_set_add(raptor_id_set* set, raptor_uri* base_uri, const unsigned char *item, size_t item_len);
#ifdef RAPTOR_DEBUG
void raptor_id_set_stats_print(raptor_id_set* set, FILE *stream);
#endif

/* raptor_sax2.c */
/*
 * SAX2 elements/attributes on stack 
 */
struct raptor_xml_element_s {
  /* NULL at bottom of stack */
  struct raptor_xml_element_s *parent;
  raptor_qname *name;
  raptor_qname **attributes;
  unsigned int attribute_count;

  /* value of xml:lang attribute on this element or NULL */
  const unsigned char *xml_language;

  /* URI of xml:base attribute value on this element or NULL */
  raptor_uri *base_uri;

  /* CDATA content of element and checks for mixed content */
  unsigned char *content_cdata;
  unsigned int content_cdata_length;
  /* how many cdata blocks seen */
  unsigned int content_cdata_seen;
  /* how many contained elements seen */
  unsigned int content_element_seen;

  raptor_sequence *declared_nspaces;
};


/* start of an element */
typedef void (*raptor_sax2_start_element_handler)(void *user_data, const unsigned char *name,  const unsigned char **atts);
/* end of an element */
typedef void (*raptor_sax2_end_element_handler)(void *user_data, const unsigned char *name);
/* characters */
typedef void (*raptor_sax2_characters_handler)(void *user_data, const unsigned char *s, int len);
/* like <![CDATA[...]> */
typedef void (*raptor_sax2_cdata_handler)(void *user_data, const unsigned char *s, int len);
/* comment */
typedef void (*raptor_sax2_comment_handler)(void *user_data, const unsigned char *s);
/* unparsed (NDATA) entity */
typedef void (*raptor_sax2_unparsed_entity_decl_handler)(void *user_data, const unsigned char* entityName, const unsigned char* base, const unsigned char* systemId, const unsigned char* publicId, const unsigned char* notationName);
/* external entity reference */
typedef int (*raptor_sax2_external_entity_ref_handler)(void *user_data, const unsigned char* context, const unsigned char* base, const unsigned char* systemId, const unsigned char* publicId);

struct raptor_sax2_s {
#ifdef RAPTOR_XML_LIBXML
  int magic;
#endif
  void* user_data;
  
#ifdef RAPTOR_XML_EXPAT
  XML_Parser xp;
#ifdef EXPAT_UTF8_BOM_CRASH
  int tokens_count; /* used to see if trying to get location info is safe */
#endif
#endif
#ifdef RAPTOR_XML_LIBXML
  /* structure holding sax event handlers */
  xmlSAXHandler sax;
  /* parser context */
  xmlParserCtxtPtr xc;
  /* pointer to SAX document locator */
  xmlSAXLocatorPtr loc;

#if LIBXML_VERSION < 20425
  /* flag for some libxml eversions*/
  int first_read;
#endif

#endif  

  /* element depth */
  int depth;

  /* stack of elements - elements add after current_element */
  raptor_xml_element *root_element;
  raptor_xml_element *current_element;

  /* start of an element */
  raptor_sax2_start_element_handler start_element_handler;
  /* end of an element */
  raptor_sax2_end_element_handler end_element_handler;
  /* characters */
  raptor_sax2_characters_handler characters_handler;
  /* like <![CDATA[...]> */
  raptor_sax2_cdata_handler cdata_handler;
  /* comment */
  raptor_sax2_comment_handler comment_handler;
  /* unparsed (NDATA) entity */
  raptor_sax2_unparsed_entity_decl_handler unparsed_entity_decl_handler;
  /* external entity reference */
  raptor_sax2_external_entity_ref_handler external_entity_ref_handler;

  raptor_locator *locator;

  void *error_data;
  raptor_message_handler error_handler;

  void *fatal_error_data;
  raptor_message_handler fatal_error_handler;

  void *warning_data;
  raptor_message_handler warning_handler;
};

void raptor_init_sax2(void);
void raptor_finish_sax2(void);

raptor_sax2* raptor_new_sax2(void *user_data, void *error_data, raptor_message_handler error_handler, void *fatal_error_data, raptor_message_handler fatal_error_handler, void *warning_data, raptor_message_handler warning_handler);
void raptor_free_sax2(raptor_sax2 *sax2);

void raptor_sax2_set_start_element_handler(raptor_sax2* sax2, raptor_sax2_start_element_handler handler);
void raptor_sax2_set_end_element_handler(raptor_sax2* sax2, raptor_sax2_end_element_handler handler);
void raptor_sax2_set_characters_handler(raptor_sax2* sax2, raptor_sax2_characters_handler handler);
void raptor_sax2_set_cdata_handler(raptor_sax2* sax2, raptor_sax2_cdata_handler handler);
void raptor_sax2_set_comment_handler(raptor_sax2* sax2, raptor_sax2_comment_handler handler);
void raptor_sax2_set_unparsed_entity_decl_handler(raptor_sax2* sax2, raptor_sax2_unparsed_entity_decl_handler handler);
void raptor_sax2_set_external_entity_ref_handler(raptor_sax2* sax2, raptor_sax2_external_entity_ref_handler handler);
void raptor_sax2_set_locator(raptor_sax2* sax2, raptor_locator* locator);
void raptor_sax2_parse_start(raptor_sax2 *sax2, raptor_uri *base_uri);
int raptor_sax2_parse_chunk(raptor_sax2* sax2, const unsigned char *buffer, size_t len, int is_end);
void raptor_sax2_parse_handle_errors(raptor_sax2* sax2);

raptor_xml_element* raptor_xml_element_pop(raptor_sax2* sax2);
void raptor_xml_element_push(raptor_sax2* sax2, raptor_xml_element* element);
int raptor_sax2_get_depth(raptor_sax2* sax2);
void raptor_sax2_inc_depth(raptor_sax2* sax2);
void raptor_sax2_dec_depth(raptor_sax2* sax2);
const unsigned char* raptor_sax2_inscope_xml_language(raptor_sax2* sax2);
raptor_uri* raptor_sax2_inscope_base_uri(raptor_sax2* sax2);
void raptor_sax2_update_document_locator(raptor_sax2* sax2, raptor_locator* locator);
  
#ifdef RAPTOR_DEBUG
void raptor_print_xml_element(raptor_xml_element *element, FILE* stream);
#endif

void raptor_sax2_start_element(void* user_data, const unsigned char *name, const unsigned char **atts);
void raptor_sax2_end_element(void* user_data, const unsigned char *name);
void raptor_sax2_characters(void* user_data, const unsigned char *s, int len);
void raptor_sax2_cdata(void* user_data, const unsigned char *s, int len);
void raptor_sax2_comment(void* user_data, const unsigned char *s);
void raptor_sax2_unparsed_entity_decl(void* user_data, const unsigned char* entityName, const unsigned char* base, const unsigned char* systemId, const unsigned char* publicId, const unsigned char* notationName);
int raptor_sax2_external_entity_ref(void* user_data, const unsigned char* context, const unsigned char* base, const unsigned char* systemId, const unsigned char* publicId);


/* turtle_parser.y and turtle_lexer.l */
typedef struct raptor_turtle_parser_s raptor_turtle_parser;

/* n3_parser.y and n3_lexer.l */
typedef struct raptor_n3_parser_s raptor_n3_parser;

typedef struct {
  raptor_identifier *subject;
  raptor_identifier *predicate;
  raptor_identifier *object;
} raptor_triple;


/* raptor_rfc2396.c */
struct raptor_uri_detail_s
{
  size_t uri_len;
  /* buffer is the same size as the original uri_len */
  unsigned char *buffer;

  /* URI Components.  These all point into buffer */
  unsigned char *scheme;
  unsigned char *authority;
  unsigned char *path;
  unsigned char *query;
  unsigned char *fragment;

  /* Lengths of the URI Components  */
  size_t scheme_len;
  size_t authority_len;
  size_t path_len;
  size_t query_len;
  size_t fragment_len;
};


/* for time_t */
#if TIME_WITH_SYS_TIME
# include <sys/time.h>
# include <time.h>
#else
# if HAVE_SYS_TIME_H
#  include <sys/time.h>
# else
#  include <time.h>
# endif
#endif

/* parsedate.c */
#ifdef HAVE_INN_PARSEDATE
#include <libinn.h>
#define RAPTOR_PARSEDATE_FUNCTION parsedate
#else
#ifdef HAVE_RAPTOR_PARSE_DATE
time_t raptor_parse_date(char *p, time_t *now);
#define RAPTOR_PARSEDATE_FUNCTION raptor_parse_date
#else
#ifdef HAVE_CURL_CURL_H
#include <curl/curl.h>
#define RAPTOR_PARSEDATE_FUNCTION curl_getdate
#endif
#endif
#endif

/* turtle_common.c */
int raptor_stringbuffer_append_turtle_string(raptor_stringbuffer* stringbuffer, unsigned char *text, size_t len, int delim, raptor_simple_message_handler error_handler, void *error_data);


/* raptor_xsd.c */
raptor_identifier* raptor_new_identifier_from_double(double d);

/* end of RAPTOR_INTERNAL */
#endif


#ifdef __cplusplus
}
#endif

#endif
