#pragma once

/*
 * JIS (Japanese) layout symbol keycodes.
 *
 * The OS is configured with the Japanese (JIS) keyboard layout, so the HID
 * usages the firmware sends are interpreted by the JIS driver rather than the
 * US one. These JP_* macros emit the *character shown in the comment* by using
 * the HID usage + shift state that the JIS layout maps that character to.
 *
 * Modeled after QMK's keymap_japanese.h. Use these instead of the plain ZMK
 * symbol names (PIPE, AT_SIGN, GRAVE, ...) anywhere a literal symbol is wanted.
 *
 * Note: on JIS, plain GRAVE is the 半角/全角 (IME toggle) key, NOT a backtick;
 * the backtick is JP_GRV below.
 */

#include <dt-bindings/zmk/keys.h>

/* --- Base (unshifted) keys whose position differs from US --- */
#define JP_CIRC EQUAL          /* ^ */
#define JP_AT   LBKT           /* @ */
#define JP_LBRC RBKT           /* [ */
#define JP_RBRC BSLH           /* ] */
#define JP_SCLN SEMI           /* ; */
#define JP_COLN SQT            /* : */
#define JP_BSLS INT1           /* \ (backslash, RO key) */
#define JP_YEN  INT_YEN        /* ¥ (yen key) */
#define JP_SLSH FSLH           /* / */
#define JP_MINS MINUS          /* - */
#define JP_COMM COMMA          /* , */
#define JP_DOT  DOT            /* . */

/* --- Shifted keys --- */
#define JP_EXLM LS(N1)         /* ! */
#define JP_DQUO LS(N2)         /* " */
#define JP_HASH LS(N3)         /* # */
#define JP_DLR  LS(N4)         /* $ */
#define JP_PERC LS(N5)         /* % */
#define JP_AMPR LS(N6)         /* & */
#define JP_QUOT LS(N7)         /* ' */
#define JP_LPRN LS(N8)         /* ( */
#define JP_RPRN LS(N9)         /* ) */
#define JP_EQL  LS(JP_MINS)    /* = */
#define JP_TILD LS(JP_CIRC)    /* ~ */
#define JP_GRV  LS(JP_AT)      /* ` (backtick) */
#define JP_LCBR LS(JP_LBRC)    /* { */
#define JP_RCBR LS(JP_RBRC)    /* } */
#define JP_PLUS LS(JP_SCLN)    /* + */
#define JP_ASTR LS(JP_COLN)    /* * */
#define JP_UNDS LS(JP_BSLS)    /* _ */
#define JP_PIPE LS(JP_YEN)     /* | */
#define JP_QUES LS(JP_SLSH)    /* ? */
#define JP_LABK LS(JP_COMM)    /* < */
#define JP_RABK LS(JP_DOT)     /* > */
