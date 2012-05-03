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
     PREFIX = 258,
     A = 259,
     AT = 260,
     HAT = 261,
     DOT = 262,
     COMMA = 263,
     SEMICOLON = 264,
     LEFT_SQUARE = 265,
     RIGHT_SQUARE = 266,
     LEFT_ROUND = 267,
     RIGHT_ROUND = 268,
     STRING_LITERAL = 269,
     URI_LITERAL = 270,
     BLANK_LITERAL = 271,
     QNAME_LITERAL = 272,
     IDENTIFIER = 273,
     INTEGER_LITERAL = 274,
     FLOATING_LITERAL = 275,
     DECIMAL_LITERAL = 276,
     ERROR_TOKEN = 277
   };
#endif
/* Tokens.  */
#define PREFIX 258
#define A 259
#define AT 260
#define HAT 261
#define DOT 262
#define COMMA 263
#define SEMICOLON 264
#define LEFT_SQUARE 265
#define RIGHT_SQUARE 266
#define LEFT_ROUND 267
#define RIGHT_ROUND 268
#define STRING_LITERAL 269
#define URI_LITERAL 270
#define BLANK_LITERAL 271
#define QNAME_LITERAL 272
#define IDENTIFIER 273
#define INTEGER_LITERAL 274
#define FLOATING_LITERAL 275
#define DECIMAL_LITERAL 276
#define ERROR_TOKEN 277




#if ! defined (YYSTYPE) && ! defined (YYSTYPE_IS_DECLARED)
#line 121 "./n3_parser.y"
typedef union YYSTYPE {
  unsigned char *string;
  raptor_identifier *identifier;
  raptor_sequence *sequence;
  raptor_uri *uri;
  int integer; /* 0+ for a xsd:integer datatyped RDF literal */
  double floating;
} YYSTYPE;
/* Line 1447 of yacc.c.  */
#line 91 "n3_parser.tab.h"
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif





