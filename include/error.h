#pragma once

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define ERROR(...) {fprintf(stderr, "%s[ERROR] ", KRED); fprintf(stderr, __VA_ARGS__); fprintf(stderr, KNRM "\n");}
#define WARN(...) {fprintf(stdout,"%s[WARN] ", KYEL); fprintf(stderr, __VA_ARGS__); fprintf(stderr, KNRM "\n"); fflush(stdout);}
#define SUCCESS(...) {fprintf(stdout, KGRN); fprintf(stderr, __VA_ARGS__); fprintf(stderr, KNRM "\n"); fflush(stdout);}
