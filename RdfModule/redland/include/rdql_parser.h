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
     SOURCE = 259,
     FROM = 260,
     WHERE = 261,
     AND = 262,
     FOR = 263,
     USING = 264,
     SC_AND = 265,
     SC_OR = 266,
     STR_NMATCH = 267,
     STR_MATCH = 268,
     STR_NE = 269,
     STR_EQ = 270,
     GE = 271,
     LE = 272,
     GT = 273,
     LT = 274,
     NEQ = 275,
     EQ = 276,
     FLOATING_POINT_LITERAL = 277,
     STRING_LITERAL = 278,
     PATTERN_LITERAL = 279,
     INTEGER_LITERAL = 280,
     BOOLEAN_LITERAL = 281,
     NULL_LITERAL = 282,
     URI_LITERAL = 283,
     QNAME_LITERAL = 284,
     IDENTIFIER = 285
   };
#endif
/* Tokens.  */
#define SELECT 258
#define SOURCE 259
#define FROM 260
#define WHERE 261
#define AND 262
#define FOR 263
#define USING 264
#define SC_AND 265
#define SC_OR 266
#define STR_NMATCH 267
#define STR_MATCH 268
#define STR_NE 269
#define STR_EQ 270
#define GE 271
#define LE 272
#define GT 273
#define LT 274
#define NEQ 275
#define EQ 276
#define FLOATING_POINT_LITERAL 277
#define STRING_LITERAL 278
#define PATTERN_LITERAL 279
#define INTEGER_LITERAL 280
#define BOOLEAN_LITERAL 281
#define NULL_LITERAL 282
#define URI_LITERAL 283
#define QNAME_LITERAL 284
#define IDENTIFIER 285




#if ! defined (YYSTYPE) && ! defined (YYSTYPE_IS_DECLARED)
#line 102 "./rdql_parser.y"
typedef union YYSTYPE {
  raptor_sequence *seq;
  rasqal_variable *variable;
  rasqal_literal *literal;
  rasqal_triple *triple;
  rasqal_expression *expr;
  double floating;
  raptor_uri *uri;
  unsigned char *name;
} YYSTYPE;
/* Line 1447 of yacc.c.  */
#line 109 "rdql_parser.tab.h"
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif





