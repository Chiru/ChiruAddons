/* A Bison parser, made by GNU Bison 2.1.  */

/* Skeleton parser for Yacc-like parsing with Bison,
   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005 Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.  */

/* As a special exception, when this file is copied by Bison into a
   Bison output file, you may use that output file without restriction.
   This special exception was added by the Free Software Foundation
   in version 1.24 of Bison.  */

/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     SELECT = 258,
     FROM = 259,
     WHERE = 260,
     OPTIONAL = 261,
     PREFIX = 262,
     DESCRIBE = 263,
     CONSTRUCT = 264,
     ASK = 265,
     DISTINCT = 266,
     LIMIT = 267,
     UNION = 268,
     BASE = 269,
     BOUND = 270,
     STR = 271,
     LANG = 272,
     DATATYPE = 273,
     ISURI = 274,
     ISBLANK = 275,
     ISLITERAL = 276,
     GRAPH = 277,
     NAMED = 278,
     FILTER = 279,
     OFFSET = 280,
     A = 281,
     ORDER = 282,
     BY = 283,
     REGEX = 284,
     ASC = 285,
     DESC = 286,
     LANGMATCHES = 287,
     SC_AND = 288,
     SC_OR = 289,
     GE = 290,
     LE = 291,
     GT = 292,
     LT = 293,
     NEQ = 294,
     EQ = 295,
     FLOATING_POINT_LITERAL = 296,
     STRING_LITERAL = 297,
     INTEGER_LITERAL = 298,
     BOOLEAN_LITERAL = 299,
     DECIMAL_LITERAL = 300,
     URI_LITERAL = 301,
     URI_LITERAL_BRACE = 302,
     QNAME_LITERAL = 303,
     BLANK_LITERAL = 304,
     QNAME_LITERAL_BRACE = 305,
     IDENTIFIER = 306
   };
#endif
/* Tokens.  */
#define SELECT 258
#define FROM 259
#define WHERE 260
#define OPTIONAL 261
#define PREFIX 262
#define DESCRIBE 263
#define CONSTRUCT 264
#define ASK 265
#define DISTINCT 266
#define LIMIT 267
#define UNION 268
#define BASE 269
#define BOUND 270
#define STR 271
#define LANG 272
#define DATATYPE 273
#define ISURI 274
#define ISBLANK 275
#define ISLITERAL 276
#define GRAPH 277
#define NAMED 278
#define FILTER 279
#define OFFSET 280
#define A 281
#define ORDER 282
#define BY 283
#define REGEX 284
#define ASC 285
#define DESC 286
#define LANGMATCHES 287
#define SC_AND 288
#define SC_OR 289
#define GE 290
#define LE 291
#define GT 292
#define LT 293
#define NEQ 294
#define EQ 295
#define FLOATING_POINT_LITERAL 296
#define STRING_LITERAL 297
#define INTEGER_LITERAL 298
#define BOOLEAN_LITERAL 299
#define DECIMAL_LITERAL 300
#define URI_LITERAL 301
#define URI_LITERAL_BRACE 302
#define QNAME_LITERAL 303
#define BLANK_LITERAL 304
#define QNAME_LITERAL_BRACE 305
#define IDENTIFIER 306




#if ! defined (YYSTYPE) && ! defined (YYSTYPE_IS_DECLARED)
#line 108 "./sparql_parser.y"
typedef union YYSTYPE {
  raptor_sequence *seq;
  rasqal_variable *variable;
  rasqal_literal *literal;
  rasqal_triple *triple;
  rasqal_expression *expr;
  rasqal_graph_pattern *graph_pattern;
  double floating;
  raptor_uri *uri;
  unsigned char *name;
  rasqal_formula *formula;
} YYSTYPE;
/* Line 1447 of yacc.c.  */
#line 153 "sparql_parser.tab.h"
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif





