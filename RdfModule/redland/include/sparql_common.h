/* -*- Mode: c; c-basic-offset: 2 -*-
 *
 * sparql_common.h - SPARQL lexer/parser shared internals
 *
 * $Id: sparql_common.h,v 1.4 2006/01/12 05:15:52 cmdjb Exp $
 *
 * Copyright (C) 2004-2006, David Beckett http://purl.org/net/dajobe/
 * Copyright (C) 2004-2004, University of Bristol, UK http://www.bristol.ac.uk/
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
 */

#ifndef SPARQL_COMMON_H
#define SPARQL_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif


/* sparql_parser.y */
int sparql_syntax_error(rasqal_query *rq, const char *message, ...);
int sparql_syntax_warning(rasqal_query *rq, const char *message, ...);

int sparql_query_lex(void);


struct rasqal_sparql_query_engine_s {
  /* STATIC lexer */
  yyscan_t scanner;

  int scanner_set;

  /* for error reporting */
  unsigned int lineno;
};


typedef struct rasqal_sparql_query_engine_s rasqal_sparql_query_engine;


#ifdef __cplusplus
}
#endif

#endif
